#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <mbedtls/sha256.h>
#include "secrets.h"

// Pan and tilt use separate ULN2003 drivers.
const uint8_t panPins[4] = {18, 19, 20, 21};
const uint8_t tiltPins[4] = {0, 1, 14, 3};
const uint8_t relayPin = 22;
bool relayIsOn = false;
const uint8_t panOrder[4] = {0, 1, 2, 3};
// The vertical motor's confirmed coil order was previously called Pan Order 3.
const uint8_t tiltOrder[4] = {0, 1, 3, 2};
const uint32_t panManualStepDelayUs = 20000;
const uint32_t tiltManualStepDelayUs = 25000;
const uint32_t panCalibrationStepDelayUs = 2000;
const uint32_t tiltCalibrationStepDelayUs = 10000;
const uint16_t smallMove = 32;
const uint16_t mediumMove = 256;

// The circular pan axis measures 3,968 steps end to end. Add 32 steps so a
// sweep from the far end reaches the selected home stop. Vertical tilt uses
// 512 steps to home. There are no physical end sensors on either axis.
const uint16_t panHomeSweep = 3968 + 32;
const uint16_t tiltHomeSweep = 512;
const int32_t panSafeTravel = 3968 - 2 * 32;
const int32_t tiltSafeTravel = 448;
const uint32_t ptzMotionWatchdogMs = 800;

const char *wifiSsid = CAMERA_WIFI_SSID;
const char *wifiPassword = CAMERA_WIFI_PASSWORD;
const char *wifiApSsid = CAMERA_SETUP_AP_SSID;
const char *wifiApPassword = CAMERA_SETUP_AP_PASSWORD;
const char *otaHostname = CAMERA_OTA_HOSTNAME;
const char *otaPassword = CAMERA_OTA_PASSWORD;
bool otaStarted = false;
WebServer server(80);
Preferences cameraPrefs;
Preferences wifiPrefs;
String activeWifiSsid;
String activeWifiPassword;
bool configApActive = false;
bool configApManualHold = false;
uint32_t wifiStartedAtMs = 0;
uint32_t wifiDisconnectedAtMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t configApCloseAtMs = 0;
uint32_t lastConfigApAttemptMs = 0;
String lastAuthNonce;
uint32_t authNonceCount = 0;
const char *cameraPrefsNs = "camera";

enum AxisState : uint8_t { AXIS_STARTING, AXIS_HOMING, AXIS_HOME, AXIS_MOVING, AXIS_UNKNOWN };
struct Axis {
  const char *name;
  const uint8_t *pins;
  const uint8_t *orderAway;
  const uint8_t *orderHome;
  bool reverseHomeOrder;
  bool homeDirectionB;
  AxisState state;
  int32_t position;
  int32_t maxPosition;
  uint16_t target;
  uint16_t completed;
  bool directionB;
  bool homing;
  bool positionKnown;
  uint32_t lastStepUs;
};

Axis pan = {"pan", panPins, panOrder, panOrder, true, true, AXIS_UNKNOWN, 0, panSafeTravel, 0, 0, true, false, false, 0};
Axis tilt = {"tilt", tiltPins, tiltOrder, tiltOrder, true, false, AXIS_UNKNOWN, 0, tiltSafeTravel, 0, 0, false, false, false, 0};
enum BootPhase : uint8_t {
  BOOT_HOMING_BOTH, BOOT_CENTERING_PAN, BOOT_CENTERING_TILT, BOOT_COMPLETE, BOOT_CANCELLED
};
BootPhase bootPhase = BOOT_HOMING_BOTH;
String message = "Starting pan and tilt homing.";
struct ContinuousMotion {
  int8_t panDirection;
  int8_t tiltDirection;
  uint8_t speed;
  bool active;
  uint32_t lastKeepaliveMs;
};
ContinuousMotion ptzMotion = {0, 0, 0, false, 0};
String ptzMotionClient;
uint32_t ptzMotionSequence = 0;

const char *bootPhaseName() {
  switch (bootPhase) {
    case BOOT_HOMING_BOTH: return "homing both";
    case BOOT_CENTERING_PAN: return "centering pan";
    case BOOT_CENTERING_TILT: return "centering tilt";
    case BOOT_COMPLETE: return "complete";
    default: return "cancelled";
  }
}

void setChannels(const uint8_t pins[4], uint8_t mask) {
  for (uint8_t i = 0; i < 4; ++i) digitalWrite(pins[i], (mask & (1U << i)) ? HIGH : LOW);
}
void releaseMotor(const uint8_t pins[4]) { setChannels(pins, 0); }

void stopContinuousAxis(Axis &axis) {
  if (&axis == &pan) ptzMotion.panDirection = 0;
  else ptzMotion.tiltDirection = 0;
  releaseMotor(axis.pins);
  if (axis.state == AXIS_MOVING && !axis.homing) axis.state = axis.positionKnown ? AXIS_HOME : AXIS_UNKNOWN;
  if (!ptzMotion.panDirection && !ptzMotion.tiltDirection) ptzMotion.active = false;
}

void stopContinuousMotion() {
  if (ptzMotion.panDirection) stopContinuousAxis(pan);
  if (ptzMotion.tiltDirection) stopContinuousAxis(tilt);
  ptzMotion.panDirection = 0;
  ptzMotion.tiltDirection = 0;
  ptzMotion.active = false;
}

uint32_t continuousStepDelayUs(const Axis &axis, uint8_t speed) {
  if (&axis == &pan) {
    // Double the pan joystick step rate at every slider setting while keeping
    // its speed curve and travel limits unchanged.
    const uint32_t slow = 25000;
    const uint32_t medium = panManualStepDelayUs / 2;
    const uint32_t fast = 5000;
    if (speed <= 5) return slow - ((slow - medium) * (speed - 1)) / 4;
    return medium - ((medium - fast) * (speed - 5)) / 5;
  }
  const uint32_t slow = 60000;
  const uint32_t medium = tiltManualStepDelayUs;
  const uint32_t fast = 12500;
  if (speed <= 5) return slow - ((slow - medium) * (speed - 1)) / 4;
  return medium - ((medium - fast) * (speed - 5)) / 5;
}

void configureHeldAxis(Axis &axis, int8_t direction, bool panAxis) {
  int8_t &currentDirection = panAxis ? ptzMotion.panDirection : ptzMotion.tiltDirection;
  if (!direction) {
    if (currentDirection) stopContinuousAxis(axis);
    return;
  }
  // Joystick left and down use coil order B; right and up use A. The two
  // axes have different homeDirectionB values, preserving their wiring setup.
  const bool directionB = direction < 0;
  if (axis.positionKnown) {
    const bool towardHome = directionB == axis.homeDirectionB;
    if ((towardHome && axis.position <= 0) || (!towardHome && axis.position >= axis.maxPosition)) {
      if (currentDirection) stopContinuousAxis(axis);
      return;
    }
  } else {
    if (currentDirection) stopContinuousAxis(axis);
    return;
  }
  if (currentDirection != direction || axis.directionB != directionB) {
    releaseMotor(axis.pins);
    axis.completed = 0;
    axis.lastStepUs = micros();
  }
  currentDirection = direction;
  axis.directionB = directionB;
  axis.target = 0;
  axis.homing = false;
  axis.state = AXIS_MOVING;
}

bool setContinuousMotion(int8_t panDirection, int8_t tiltDirection, uint8_t speed) {
  if (panDirection < -1 || panDirection > 1 || tiltDirection < -1 || tiltDirection > 1 || speed > 10) return false;
  if (bootPhase != BOOT_COMPLETE || (panDirection && !pan.positionKnown) || (tiltDirection && !tilt.positionKnown)) return false;
  if (!speed || (!panDirection && !tiltDirection)) {
    stopContinuousMotion();
    return true;
  }
  configureHeldAxis(pan, panDirection, true);
  // The camera is mounted upside down. Invert manual tilt commands centrally;
  // calibration and the motor's stepping sequence keep their existing mapping.
  configureHeldAxis(tilt, -tiltDirection, false);
  ptzMotion.speed = speed;
  ptzMotion.active = ptzMotion.panDirection || ptzMotion.tiltDirection;
  ptzMotion.lastKeepaliveMs = millis();
  message = ptzMotion.active ? "Joystick movement active." : "Movement blocked at software travel limit.";
  return true;
}

bool acceptMotionSequence(bool isStart) {
  const String client = server.arg("client");
  const String sequenceText = server.arg("seq");
  if (client.length() < 12 || client.length() > 40 || sequenceText.length() == 0 || sequenceText.length() > 10) return false;
  for (size_t i = 0; i < client.length(); ++i) if (!(isAlphaNumeric(client[i]) || client[i] == '-')) return false;
  const uint32_t sequence = static_cast<uint32_t>(sequenceText.toInt());
  if (String(sequence) != sequenceText) return false;
  if (client == ptzMotionClient) {
    if (sequence <= ptzMotionSequence) return false;
  } else if (!isStart && server.uri() != "/api/ptz/stop" && server.uri() != "/api/v1/ptz/stop") {
    return false;
  }
  ptzMotionClient = client;
  ptzMotionSequence = sequence;
  return true;
}

void startOta() {
  if (otaStarted || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname(otaHostname);
  ArduinoOTA.setPassword(otaPassword);
  ArduinoOTA.onStart([]() {
    stopContinuousMotion();
    bootPhase = BOOT_CANCELLED;
    pan.state = AXIS_UNKNOWN;
    tilt.state = AXIS_UNKNOWN;
    pan.positionKnown = false;
    tilt.positionKnown = false;
    releaseMotor(panPins);
    releaseMotor(tiltPins);
    Serial.println("OTA update started; motors released.");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA update complete; rebooting."); });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA update error: %u\n", static_cast<unsigned>(error));
  });
  ArduinoOTA.begin();
  otaStarted = true;
  Serial.printf("OTA ready: %s.local (%s:3232)\n", otaHostname, WiFi.localIP().toString().c_str());
}

void relayOn() {
  digitalWrite(relayPin, LOW);
  pinMode(relayPin, OUTPUT);
  relayIsOn = true;
}

void relayOff() {
  pinMode(relayPin, INPUT);
  relayIsOn = false;
}

const char *stateName(AxisState state) {
  switch (state) {
    case AXIS_STARTING: return "starting";
    case AXIS_HOMING: return "homing";
    case AXIS_HOME: return "position known";
    case AXIS_MOVING: return "moving";
    default: return "unknown";
  }
}

String jsonStatus() {
  String json = "{\"panState\":\"" + String(stateName(pan.state)) + "\"";
  json += ",\"panPosition\":" + String(pan.position);
  json += ",\"panCompleted\":" + String(pan.completed);
  json += ",\"panTarget\":" + String(pan.target);
  json += ",\"tiltState\":\"" + String(stateName(tilt.state)) + "\"";
  json += ",\"tiltPosition\":" + String(tilt.position);
  json += ",\"tiltCompleted\":" + String(tilt.completed);
  json += ",\"tiltTarget\":" + String(tilt.target);
  json += ",\"calibration\":\"" + String(bootPhaseName()) + "\"";
  json += ",\"relayOn\":" + String(relayIsOn ? "true" : "false");
  json += ",\"ptzActive\":" + String(ptzMotion.active ? "true" : "false");
  json += ",\"ptzSpeed\":" + String(ptzMotion.speed);
  json += ",\"message\":\"" + message + "\"}";
  return json;
}

#include "dashboard.h"

String sha256Hex(const String &input) {
  uint8_t digest[32];
  mbedtls_sha256(reinterpret_cast<const unsigned char *>(input.c_str()), input.length(), digest, 0);
  char result[65];
  for (size_t i = 0; i < sizeof(digest); ++i) snprintf(result + i * 2, 3, "%02x", digest[i]);
  result[64] = '\0';
  return String(result);
}

bool isDashboardOrigin() {
  const String origin = server.header("Origin");
  const String host = server.header("Host");
  return origin.length() && host.length() && origin == String("http://") + host;
}

String makeWebToken() {
  char token[33];
  for (uint8_t i = 0; i < 16; ++i) snprintf(token + i * 2, 3, "%02x", static_cast<unsigned>(esp_random() & 0xff));
  token[32] = '\0';
  return String(token);
}

bool cameraBrowserAuthorized() {
  const String token = cameraPrefs.getString("webtoken", "");
  const String cookie = server.header("Cookie");
  if (!token.length() || !cookie.length()) return false;
  const String expected = "camAuth=" + token;
  int at = cookie.indexOf(expected);
  if (at < 0) return false;
  const int end = at + expected.length();
  return (at == 0 || cookie[at - 1] == ' ' || cookie[at - 1] == ';') &&
         (end == cookie.length() || cookie[end] == ';' || cookie[end] == ' ');
}

void setCameraBrowserCookie() {
  server.sendHeader("Set-Cookie", "camAuth=" + cameraPrefs.getString("webtoken", "") + "; Path=/; HttpOnly; SameSite=Strict; Max-Age=31536000");
}

bool validCameraIp(const String &ip) {
  if (ip.length() < 7 || ip.length() > 15) return false;
  IPAddress parsed;
  return parsed.fromString(ip);
}

bool validSession(const String &session) {
  if (!session.startsWith("local_") || session.length() < 12 || session.length() > 64) return false;
  for (size_t i = 6; i < session.length(); ++i) {
    const char c = session[i];
    if (!(isAlphaNumeric(c) || c == '-')) return false;
  }
  return true;
}

void sendCameraConfig() {
  const bool configured = cameraPrefs.getString("password", "").length() > 0;
  String json = "{\"configured\":" + String(configured ? "true" : "false");
  json += ",\"authorized\":" + String(cameraBrowserAuthorized() ? "true" : "false");
  json += ",\"ip\":\"" + cameraPrefs.getString("ip", "192.168.1.101") + "\"";
  json += ",\"username\":\"" + cameraPrefs.getString("username", "admin") + "\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void saveCameraConfig() {
  if (!isDashboardOrigin() && server.uri() != "/api/v1/preset/save") { server.send(403, "application/json", "{\"error\":\"same-origin request required\"}"); return; }
  const String ip = server.arg("ip");
  const String username = server.arg("username");
  const String password = server.arg("password");
  const String existingPassword = cameraPrefs.getString("password", "");
  bool validUsername = username.length() > 0 && username.length() <= 32;
  for (size_t i = 0; i < username.length(); ++i) {
    const char c = username[i];
    if (!(isAlphaNumeric(c) || c == '_' || c == '-' || c == '.' || c == '@')) validUsername = false;
  }
  if (!validCameraIp(ip) || !validUsername || password.length() > 63) {
    server.send(400, "application/json", "{\"error\":\"invalid camera settings\"}"); return;
  }
  if (existingPassword.length() && !cameraBrowserAuthorized() && password != existingPassword) {
    server.send(403, "application/json", "{\"error\":\"enter the saved camera password to authorize this browser\"}"); return;
  }
  if (password.length() && (cameraBrowserAuthorized() || !existingPassword.length())) cameraPrefs.putString("password", password);
  else if (!existingPassword.length()) {
    server.send(400, "application/json", "{\"error\":\"enter the camera password for initial setup\"}"); return;
  }
  cameraPrefs.putString("ip", ip);
  cameraPrefs.putString("username", username);
  lastAuthNonce = "";
  authNonceCount = 0;
  setCameraBrowserCookie();
  server.send(200, "application/json", "{\"ok\":true}");
}

void signCameraRequest() {
  if (!isDashboardOrigin() && server.uri() != "/api/v1/preset/go") { server.send(403, "application/json", "{\"error\":\"same-origin request required\"}"); return; }
  if (!cameraBrowserAuthorized()) { server.send(401, "application/json", "{\"error\":\"authorize this browser with the camera password first\"}"); return; }
  const String method = server.arg("method");
  const String session = server.arg("sessionId");
  const String msgType = server.arg("msgType");
  const String action = server.arg("action");
  const String nonce = server.arg("nonce");
  const String realm = server.arg("realm");
  const uint32_t challengeNc = static_cast<uint32_t>(server.arg("nc").toInt());
  const bool allowedWebrtc = method == "Webrtc.SetConfig" && validSession(session) && (msgType == "sdp" || msgType == "ice");
  const bool allowedStream = method == "LiveStream.SetConfig" && validSession(session) &&
      (action == "playMedia" || action == "pingRTC" || action == "resolution_hd" || action == "resolution_sd");
  if ((!allowedWebrtc && !allowedStream) || !nonce.length() || nonce.length() > 256 || !realm.length() || realm.length() > 128 || challengeNc > 1000000) {
    server.send(400, "application/json", "{\"error\":\"unsupported camera signing request\"}"); return;
  }
  const String password = cameraPrefs.getString("password", "");
  const String username = cameraPrefs.getString("username", "admin");
  if (!password.length()) { server.send(409, "application/json", "{\"error\":\"camera credentials are not configured\"}"); return; }
  if (nonce != lastAuthNonce) { lastAuthNonce = nonce; authNonceCount = challengeNc; }
  const uint32_t nc = ++authNonceCount;
  char ncText[16]; snprintf(ncText, sizeof(ncText), "%lu", static_cast<unsigned long>(nc));
  char cnonceBytes[17];
  for (uint8_t i = 0; i < 8; ++i) snprintf(cnonceBytes + i * 2, 3, "%02x", static_cast<unsigned>(esp_random() & 0xff));
  cnonceBytes[16] = '\0';
  const String cnonce(cnonceBytes);
  const String h1 = sha256Hex(username + ":" + realm + ":" + password);
  const int dot = method.indexOf('.');
  const String h2 = sha256Hex(dot >= 0 ? method.substring(0, dot) + ":" + method.substring(dot + 1) : method + ":");
  const String response = sha256Hex(h1 + ":" + nonce + ":" + ncText + ":" + cnonce + ":auth:" + h2);
  String json = "{\"username\":\"" + username + "\",\"realm\":\"" + realm + "\",\"nonce\":\"" + nonce;
  json += "\",\"cnonce\":\"" + cnonce + "\",\"nc\":" + String(nc);
  json += ",\"response\":\"" + response + "\",\"algorithm\":\"SHA-256\"}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void sendStatus() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", jsonStatus());
}

void beginHoming(Axis &axis, uint16_t steps) {
  axis.state = AXIS_HOMING;
  axis.homing = true;
  axis.positionKnown = false;
  axis.directionB = axis.homeDirectionB;
  axis.target = steps;
  axis.completed = 0;
  axis.lastStepUs = micros();
}

bool readPreset(uint8_t id, int32_t &panPosition, int32_t &tiltPosition);
void setAxisTarget(Axis &axis, int32_t target);

void beginCenterMove(Axis &axis) {
  int32_t homePan = pan.maxPosition / 2;
  int32_t homeTilt = tilt.maxPosition / 2;
  if (!readPreset(0, homePan, homeTilt)) {
    homePan = pan.maxPosition / 2;
    homeTilt = tilt.maxPosition / 2;
  }
  setAxisTarget(axis, (&axis == &pan) ? homePan : homeTilt);
}

void beginCalibration() {
  stopContinuousMotion();
  ptzMotionClient = "";
  ptzMotionSequence = 0;
  releaseMotor(panPins);
  releaseMotor(tiltPins);
  pan.position = 0;
  tilt.position = 0;
  pan.positionKnown = false;
  tilt.positionKnown = false;
  pan.homing = false;
  tilt.homing = false;
  pan.state = AXIS_STARTING;
  pan.target = 0;
  pan.completed = 0;
  beginHoming(tilt, tiltHomeSweep);
  beginHoming(pan, panHomeSweep);
  bootPhase = BOOT_HOMING_BOTH;
  message = "Calibration started: homing pan and tilt together.";
  Serial.println(message);
}

void cancelStartupCalibration() {
  bootPhase = BOOT_CANCELLED;
  pan.state = AXIS_UNKNOWN;
  tilt.state = AXIS_UNKNOWN;
  pan.homing = false;
  tilt.homing = false;
  pan.positionKnown = false;
  tilt.positionKnown = false;
  releaseMotor(panPins);
  releaseMotor(tiltPins);
  message = "Calibration stopped. Positions are unknown.";
}

void updateStartupCalibration() {
  switch (bootPhase) {
    case BOOT_HOMING_BOTH:
      if (pan.state == AXIS_HOME && tilt.state == AXIS_HOME) {
        beginCenterMove(pan);
        bootPhase = BOOT_CENTERING_PAN;
        message = "Pan and tilt homed; centering pan first.";
      }
      break;
    case BOOT_CENTERING_PAN:
      if (pan.state == AXIS_HOME) {
        beginCenterMove(tilt);
        bootPhase = BOOT_CENTERING_TILT;
        message = "Pan at Home; moving tilt to Home.";
      }
      break;
    case BOOT_CENTERING_TILT:
      if (tilt.state == AXIS_HOME) {
        bootPhase = BOOT_COMPLETE;
        int32_t homePan, homeTilt;
        message = readPreset(0, homePan, homeTilt)
            ? "Calibration complete. Returned to saved Home position."
            : "Calibration complete. Pan and tilt are centered.";
        Serial.println(message);
      }
      break;
    default: break;
  }
}

void startMove(Axis &axis, char direction, uint16_t steps) {
  if (bootPhase != BOOT_COMPLETE && bootPhase != BOOT_CANCELLED) {
    message = "Calibration in progress. Wait or press STOP.";
    sendStatus();
    return;
  }
  if (axis.state == AXIS_MOVING || axis.state == AXIS_HOMING || axis.state == AXIS_STARTING) {
    message = String(axis.name) + " is busy homing or moving.";
    sendStatus();
    return;
  }
  if (steps == 0 || steps > mediumMove) {
    message = "Web moves must be 1â€“256 steps.";
    sendStatus();
    return;
  }
  const bool towardHome = (direction == 'B') == axis.homeDirectionB;
  if (axis.state == AXIS_HOME && towardHome && axis.position < steps) {
    message = String(axis.name) + " blocked at software home.";
    sendStatus();
    return;
  }
  if (axis.state == AXIS_HOME && !towardHome && axis.position + steps > axis.maxPosition) {
    message = String(axis.name) + " blocked at software far limit.";
    sendStatus();
    return;
  }
  stopContinuousMotion();
  bool positionKnown = axis.state == AXIS_HOME;
  axis.state = AXIS_MOVING;
  axis.homing = false;
  axis.positionKnown = positionKnown;
  axis.directionB = direction == 'B';
  axis.target = steps;
  axis.completed = 0;
  axis.lastStepUs = micros();
  message = String("Moving ") + axis.name + (towardHome ? " toward home." : " away from home.");
  sendStatus();
}

Axis *axisFromRequest() {
  String name = server.arg("axis");
  if (name == "pan") return &pan;
  if (name == "tilt") return &tilt;
  return nullptr;
}
bool denyApControllerAccess();

bool axisBusy(const Axis &axis) {
  return axis.state == AXIS_MOVING || axis.state == AXIS_HOMING || axis.state == AXIS_STARTING;
}

bool motionBusy() {
  return ptzMotion.active || axisBusy(pan) || axisBusy(tilt);
}

bool positionsCalibrated() {
  return bootPhase == BOOT_COMPLETE && pan.positionKnown && tilt.positionKnown &&
         pan.state == AXIS_HOME && tilt.state == AXIS_HOME;
}

void presetKeys(uint8_t id, char *panKey, size_t panSize, char *tiltKey, size_t tiltSize) {
  snprintf(panKey, panSize, "p%up", static_cast<unsigned>(id));
  snprintf(tiltKey, tiltSize, "p%ut", static_cast<unsigned>(id));
}

bool readPreset(uint8_t id, int32_t &panPosition, int32_t &tiltPosition) {
  char panKey[8], tiltKey[8];
  presetKeys(id, panKey, sizeof(panKey), tiltKey, sizeof(tiltKey));
  if (!cameraPrefs.isKey(panKey) || !cameraPrefs.isKey(tiltKey)) return false;
  panPosition = cameraPrefs.getInt(panKey, -1);
  tiltPosition = cameraPrefs.getInt(tiltKey, -1);
  return panPosition >= 0 && panPosition <= pan.maxPosition &&
         tiltPosition >= 0 && tiltPosition <= tilt.maxPosition;
}

void sendPresets() {
  String json = "{\"calibrated\":" + String(positionsCalibrated() ? "true" : "false") + ",\"presets\":[";
  for (uint8_t id = 0; id <= 3; ++id) {
    int32_t pp = -1, tp = -1;
    const bool saved = readPreset(id, pp, tp);
    if (id > 0) json += ',';
    json += "{\"id\":" + String(id) + ",\"saved\":" + String(saved ? "true" : "false");
    if (saved) json += ",\"pan\":" + String(pp) + ",\"tilt\":" + String(tp);
    json += '}';
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void setAxisTarget(Axis &axis, int32_t target) {
  const int32_t delta = target - axis.position;
  if (!delta) { axis.state = AXIS_HOME; return; }
  axis.state = AXIS_MOVING;
  axis.homing = false;
  axis.positionKnown = true;
  axis.directionB = (delta < 0) ? axis.homeDirectionB : !axis.homeDirectionB;
  axis.target = static_cast<uint16_t>(delta < 0 ? -delta : delta);
  axis.completed = 0;
  axis.lastStepUs = micros();
}

void recallPreset(uint8_t id) {
  if (!positionsCalibrated()) {
    server.send(409, "application/json", "{\"error\":\"Calibrate before recalling presets\"}");
    return;
  }
  if (motionBusy()) {
    server.send(409, "application/json", "{\"error\":\"Stop the current movement before recalling a preset\"}");
    return;
  }
  int32_t targetPan = pan.maxPosition / 2;
  int32_t targetTilt = tilt.maxPosition / 2;
  if (!readPreset(id, targetPan, targetTilt)) {
    if (id != 0) {
      server.send(404, "application/json", "{\"error\":\"That preset has not been saved\"}");
      return;
    }
    targetPan = pan.maxPosition / 2;
    targetTilt = tilt.maxPosition / 2;
  }
  setAxisTarget(pan, targetPan);
  setAxisTarget(tilt, targetTilt);
  message = id ? String("Moving to Preset ") + id + "." : "Moving to Home position.";
  sendStatus();
}

void savePreset() {
  if (denyApControllerAccess()) return;
  if (!isDashboardOrigin() && server.uri() != "/api/v1/preset/save") { server.send(403, "application/json", "{\"error\":\"same-origin request required\"}"); return; }
  const String idText = server.arg("id");
  if (idText != "0" && idText != "1" && idText != "2" && idText != "3") {
    server.send(400, "application/json", "{\"error\":\"preset id must be 0, 1, 2, or 3\"}"); return;
  }
  if (!positionsCalibrated()) { server.send(409, "application/json", "{\"error\":\"Calibrate before saving presets\"}"); return; }
  if (motionBusy()) { server.send(409, "application/json", "{\"error\":\"Stop the current movement before saving a preset\"}"); return; }
  const uint8_t id = static_cast<uint8_t>(idText.toInt());
  char panKey[8], tiltKey[8];
  presetKeys(id, panKey, sizeof(panKey), tiltKey, sizeof(tiltKey));
  cameraPrefs.putInt(panKey, pan.position);
  cameraPrefs.putInt(tiltKey, tilt.position);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"saved\":true}");
}

void goPreset() {
  if (denyApControllerAccess()) return;
  if (!isDashboardOrigin() && server.uri() != "/api/v1/preset/go") { server.send(403, "application/json", "{\"error\":\"same-origin request required\"}"); return; }
  const String idText = server.arg("id");
  if (idText != "0" && idText != "1" && idText != "2" && idText != "3") {
    server.send(400, "application/json", "{\"error\":\"preset id must be 0, 1, 2, or 3\"}"); return;
  }
  recallPreset(static_cast<uint8_t>(idText.toInt()));
}


bool clientOnConfigAp() {
  if (!configApActive) return false;
  const IPAddress remote = server.client().remoteIP();
  const IPAddress ap = WiFi.softAPIP();
  return remote[0] == ap[0] && remote[1] == ap[1] && remote[2] == ap[2];
}

bool wifiConfigAuthorized() {
  // Same-origin dashboard GETs often omit Origin; allow read-only status/scan
  // from the connected LAN. Credential changes still require same-origin POST.
  return clientOnConfigAp() || cameraBrowserAuthorized() || isDashboardOrigin() ||
         (WiFi.status() == WL_CONNECTED && !server.header("Origin").length());
}

bool denyApControllerAccess() {
  if (!clientOnConfigAp()) return false;
  server.send(403, "application/json", "{\"error\":\"PTZ controls are available only through the secured home Wi-Fi network\"}");
  return true;
}

bool startConfigAp() {
  if (configApActive) return true;
  const uint32_t now = millis();
  if (lastConfigApAttemptMs && now - lastConfigApAttemptMs < 10000) return false;
  lastConfigApAttemptMs = now;
  WiFi.mode(WIFI_AP_STA);
  const IPAddress apIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIp, gateway, subnet);
  configApActive = WiFi.softAP(wifiApSsid, wifiApPassword, 1, false, 4);
  configApCloseAtMs = 0;
  if (configApActive) Serial.printf("Wi-Fi setup AP ready at http://%s/\n", WiFi.softAPIP().toString().c_str());
  else Serial.println("Could not start Wi-Fi setup AP.");
  return configApActive;
}

void stopConfigAp() {
  if (!configApActive) return;
  WiFi.softAPdisconnect(true);
  configApActive = false;
  configApCloseAtMs = 0;
  WiFi.mode(WIFI_STA);
  Serial.println("Wi-Fi setup AP closed after station connection.");
}

String jsonQuoted(const String &value) {
  String out = "\"";
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (static_cast<uint8_t>(c) < 0x20) out += ' ';
    else out += c;
  }
  out += '"';
  return out;
}

const char wifiPortalPage[] PROGMEM = R"WIFIHTML(
<!doctype html><html lang="en"><meta name="viewport" content="width=device-width,initial-scale=1"><meta charset="utf-8"><title>Sonoffcam Wi-Fi setup</title>
<style>body{margin:0;background:#0b1116;color:#eef6f4;font:16px system-ui,sans-serif}.box{max-width:520px;margin:7vh auto;padding:24px;background:#152129;border:1px solid #29404a;border-radius:14px}h1{margin-top:0}p,small{color:#a2b8bd}label{display:block;margin:14px 0 6px}input,select,button{box-sizing:border-box;width:100%;padding:12px;border-radius:7px;border:1px solid #36535e;background:#0d171d;color:#eef6f4;font:inherit}button{background:#66dbc8;color:#09201e;font-weight:700;cursor:pointer;margin-top:14px}.status{padding:12px;background:#0d171d;border-radius:7px;white-space:pre-line}</style>
<main class="box"><h1>Sonoffcam Wi-Fi setup</h1><p>Choose a network or enter its name, then save its password. Saved credentials are never shown here.</p><div id="status" class="status">Checking connection…</div><label for="networks">Available networks</label><select id="networks"><option value="">Scan for networks</option></select><button type="button" onclick="scan()">Scan networks</button><label for="ssid">Network name (SSID)</label><input id="ssid" maxlength="32" autocomplete="off"><label for="password">Wi-Fi password</label><input id="password" type="password" maxlength="63" autocomplete="new-password"><button type="button" onclick="save()">Save and connect</button><div id="result" class="status" hidden></div></main>
<script>const statusBox=document.querySelector('#status'),select=document.querySelector('#networks'),ssid=document.querySelector('#ssid'),result=document.querySelector('#result');select.onchange=()=>{if(select.value)ssid.value=select.value};async function status(){try{const s=await(await fetch('/wifi/status',{cache:'no-store'})).json();statusBox.textContent=s.connected?'Connected to '+(s.ssid||'Wi-Fi')+'\nAddress: '+s.ip+(s.apActive?'\nSetup AP: Sonoffcam (192.168.4.1)':''):'Not connected yet. Setup AP: Sonoffcam\nPortal: http://192.168.4.1/'}catch{statusBox.textContent='Waiting for controller…'}}async function scan(){select.innerHTML='<option>Scanning…</option>';for(let i=0;i<12;i++){const s=await(await fetch('/wifi/scan',{cache:'no-store'})).json();if(!s.scanning){select.innerHTML='<option value="">Select a network</option>';for(const n of s.networks){const o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid+' ('+n.rssi+' dBm)'+(n.secure?' · secured':'');select.append(o)}return}await new Promise(r=>setTimeout(r,1000))}}async function save(){result.hidden=false;result.textContent='Saving credentials and connecting…';const body=new URLSearchParams({ssid:ssid.value.trim(),password:document.querySelector('#password').value});try{const r=await fetch('/wifi/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,cache:'no-store'}),j=await r.json();if(!r.ok)throw Error(j.error||'Could not save Wi-Fi settings');document.querySelector('#password').value='';result.textContent='Saved. Waiting for connection…';let n=0;const t=setInterval(async()=>{await status();if(++n>150)clearInterval(t)},2000)}catch(e){result.textContent=e.message}}status();setInterval(status,3000);</script></html>
)WIFIHTML";

void sendWifiStatus() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  String json = "{\"connected\":" + String(connected ? "true" : "false");
  json += ",\"apActive\":" + String(configApActive ? "true" : "false");
  json += ",\"ssid\":" + jsonQuoted(connected ? WiFi.SSID() : String());
  json += ",\"ip\":" + jsonQuoted(connected ? WiFi.localIP().toString() : String());
  json += ",\"state\":" + jsonQuoted(connected ? "connected" : (activeWifiSsid.length() ? "connecting" : "needs setup")) + "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handleWifiScan() {
  const int16_t result = WiFi.scanComplete();
  if (result == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "{\"scanning\":true,\"networks\":[]}");
    return;
  }
  if (result == WIFI_SCAN_FAILED) WiFi.scanNetworks(true, true);
  const int16_t count = WiFi.scanComplete();
  if (count < 0) {
    server.send(200, "application/json", "{\"scanning\":true,\"networks\":[]}");
    return;
  }
  String json = "{\"scanning\":false,\"networks\":[";
  for (int16_t i = 0; i < count; ++i) {
    if (i) json += ',';
    json += "{\"ssid\":" + jsonQuoted(WiFi.SSID(i));
    json += ",\"rssi\":" + String(WiFi.RSSI(i));
    json += ",\"secure\":" + String(WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "false" : "true") + "}";
  }
  json += "]}";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

void saveWifiConfig() {
  if (!wifiConfigAuthorized() || !isDashboardOrigin()) {
    server.send(403, "application/json", "{\"error\":\"authorized same-origin Wi-Fi setup is required\"}");
    return;
  }
  const String ssid = server.arg("ssid");
  const String password = server.arg("password");
  if (!ssid.length() || ssid.length() > 32 || password.length() > 63) {
    server.send(400, "application/json", "{\"error\":\"SSID must be 1-32 characters and password at most 63 characters\"}");
    return;
  }
  wifiPrefs.putString("ssid", ssid);
  wifiPrefs.putString("password", password);
  activeWifiSsid = ssid;
  activeWifiPassword = password;
  configApManualHold = false;
  wifiDisconnectedAtMs = millis();
  wifiStartedAtMs = millis();
  lastWifiRetryMs = millis();
  configApCloseAtMs = 0;
  startConfigAp();
  WiFi.disconnect(false, false);
  WiFi.begin(activeWifiSsid.c_str(), activeWifiPassword.c_str());
  server.sendHeader("Cache-Control", "no-store");
  server.send(202, "application/json", "{\"saved\":true,\"connecting\":true}");
}

void setupRoutes() {
  const char *headerKeys[] = {"Origin", "Host", "Cookie"};
  server.collectHeaders(headerKeys, 3);
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    if (clientOnConfigAp()) {
      server.send_P(200, "text/html", wifiPortalPage);
      return;
    }
    const String cameraOrigin = "http://" + cameraPrefs.getString("ip", "192.168.1.101");
    server.sendHeader("Permissions-Policy", "local-network=(self \"" + cameraOrigin + "\"), loopback-network=(self \"" + cameraOrigin + "\")");
    server.send_P(200, "text/html", page);
  });
  server.on("/status", HTTP_GET, []() { if (!denyApControllerAccess()) sendStatus(); });
  server.on("/api/v1/status", HTTP_GET, []() { if (!denyApControllerAccess()) sendStatus(); });
  auto handleControllerRestart = []() {
    if (denyApControllerAccess()) return;
    stopContinuousMotion();
    pan.homing = false; tilt.homing = false;
    pan.state = AXIS_UNKNOWN; tilt.state = AXIS_UNKNOWN;
    pan.positionKnown = false; tilt.positionKnown = false;
    releaseMotor(panPins); releaseMotor(tiltPins);
    relayOff();
    server.sendHeader("Cache-Control", "no-store");
    server.send(202, "application/json", "{\"restarting\":true}");
    delay(250);
    ESP.restart();
  };
  server.on("/api/v1/restart", HTTP_POST, handleControllerRestart);
  server.on("/restart", HTTP_POST, handleControllerRestart);
  server.on("/api/v1/presets", HTTP_GET, []() { if (!denyApControllerAccess()) sendPresets(); });
  server.on("/api/v1/move", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    Axis *axis = axisFromRequest();
    const String direction = server.arg("dir");
    const String stepsText = server.arg("steps");
    if (!axis || (direction != "A" && direction != "B") || !stepsText.length()) {
      server.send(400, "application/json", "{\"error\":\"axis must be pan or tilt; dir must be A or B; steps must be 1-256\"}"); return;
    }
    const int steps = stepsText.toInt();
    if (steps < 1 || steps > mediumMove || String(steps) != stepsText) { server.send(400, "application/json", "{\"error\":\"steps must be 1-256\"}"); return; }
    startMove(*axis, direction[0], static_cast<uint16_t>(steps));
  });
  server.on("/api/v1/stop", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    Axis *axis = axisFromRequest();
    if (!axis) { server.send(400, "application/json", "{\"error\":\"axis must be pan or tilt\"}"); return; }
    if (bootPhase != BOOT_COMPLETE && bootPhase != BOOT_CANCELLED) {
      cancelStartupCalibration();
      sendStatus();
      return;
    }
    const bool interrupted = axisBusy(*axis);
    stopContinuousAxis(*axis);
    if (interrupted) { axis->state = AXIS_UNKNOWN; axis->homing = false; axis->positionKnown = false; releaseMotor(axis->pins); }
    message = String(axis->name) + " stopped.";
    sendStatus();
  });
  server.on("/api/v1/preset/save", HTTP_POST, []() { savePreset(); });
  server.on("/api/v1/preset/go", HTTP_POST, []() { goPreset(); });
  server.on("/api/v1/calibrate", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    if (motionBusy()) { server.send(409, "application/json", "{\"error\":\"Stop the current movement before calibration\"}"); return; }
    beginCalibration();
    sendStatus();
  });
  server.on("/api/v1/relay", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    const String state = server.arg("state");
    if (state == "on") relayOn();
    else if (state == "off") relayOff();
    else { server.send(400, "application/json", "{\"error\":\"state must be on or off\"}"); return; }
    message = state == "on" ? "Relay turned on." : "Relay turned off.";
    sendStatus();
  });
  server.on("/presets", HTTP_GET, []() { if (!denyApControllerAccess()) sendPresets(); });
  server.on("/preset/save", HTTP_POST, []() { savePreset(); });
  server.on("/preset/go", HTTP_POST, []() { goPreset(); });
  server.on("/wifi/status", HTTP_GET, []() {
    if (!wifiConfigAuthorized()) { server.send(403, "application/json", "{\"error\":\"Wi-Fi setup authorization required\"}"); return; }
    sendWifiStatus();
  });
  server.on("/wifi/scan", HTTP_GET, []() {
    if (!wifiConfigAuthorized()) { server.send(403, "application/json", "{\"error\":\"Wi-Fi setup authorization required\"}"); return; }
    handleWifiScan();
  });
  server.on("/wifi/config", HTTP_POST, []() { saveWifiConfig(); });
  server.on("/wifi/configure", HTTP_GET, []() {
    if (clientOnConfigAp()) { server.sendHeader("Location", "/", true); server.send(303, "text/plain", ""); return; }
    if (!cameraBrowserAuthorized()) { server.send(403, "application/json", "{\"error\":\"authorize this browser before opening Wi-Fi setup\"}"); return; }
    configApManualHold = true;
    configApCloseAtMs = 0;
    if (!startConfigAp()) { server.send(500, "application/json", "{\"error\":\"could not start Wi-Fi setup AP\"}"); return; }
    server.send(200, "application/json", "{\"ap\":\"Sonoffcam\",\"url\":\"http://192.168.4.1/\"}");
  });
  auto handleContinuousCommand = []() {
    if (denyApControllerAccess()) return;
    const String panArg = server.arg("pan");
    const String tiltArg = server.arg("tilt");
    const String speedArg = server.arg("speed");
    if ((panArg != "-1" && panArg != "0" && panArg != "1") ||
        (tiltArg != "-1" && tiltArg != "0" && tiltArg != "1") ||
        speedArg.length() == 0 || speedArg.length() > 2) {
      server.send(400, "application/json", "{\"error\":\"pan/tilt must be -1, 0, or 1; speed must be 0-10\"}");
      return;
    }
    const int speedValue = speedArg.toInt();
    if (speedValue < 0 || speedValue > 10 || String(speedValue) != speedArg) {
      server.send(400, "application/json", "{\"error\":\"speed must be 0-10\"}");
      return;
    }
    const int8_t panDirection = static_cast<int8_t>(panArg.toInt());
    const int8_t tiltDirection = static_cast<int8_t>(tiltArg.toInt());
    if (!acceptMotionSequence(server.uri() == "/api/ptz/start" || server.uri() == "/api/v1/ptz/start")) {
      server.send(409, "application/json", "{\"error\":\"stale or invalid joystick command\"}");
      return;
    }
    if (!setContinuousMotion(panDirection, tiltDirection, static_cast<uint8_t>(speedValue))) {
      message = (bootPhase != BOOT_COMPLETE) ? "Calibration must be complete before joystick movement." : "Calibrate before continuous movement; position is unknown.";
      server.send(409, "application/json", jsonStatus());
      return;
    }
    sendStatus();
  };
  server.on("/api/ptz/start", HTTP_POST, handleContinuousCommand);
  server.on("/api/ptz/keepalive", HTTP_POST, handleContinuousCommand);
  server.on("/api/ptz/stop", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    if (!acceptMotionSequence(false)) {
      server.send(409, "application/json", "{\"error\":\"invalid joystick stop command\"}");
      return;
    }
    if (bootPhase != BOOT_COMPLETE && bootPhase != BOOT_CANCELLED) cancelStartupCalibration();
    else {
      stopContinuousMotion();
      for (Axis *axis : {&pan, &tilt}) {
        if (axis->state == AXIS_MOVING || axis->state == AXIS_HOMING) {
          axis->state = AXIS_UNKNOWN;
          axis->homing = false;
          axis->positionKnown = false;
          releaseMotor(axis->pins);
        }
      }
      message = "Emergency stop: pan and tilt released.";
    }
    sendStatus();
  });
  server.on("/api/v1/ptz/start", HTTP_POST, handleContinuousCommand);
  server.on("/api/v1/ptz/keepalive", HTTP_POST, handleContinuousCommand);
  server.on("/api/v1/ptz/stop", HTTP_POST, []() {
    if (denyApControllerAccess()) return;
    if (!acceptMotionSequence(false)) { server.send(409, "application/json", "{\"error\":\"invalid joystick stop command\"}"); return; }
    stopContinuousMotion();
    message = "Emergency stop: pan and tilt released.";
    sendStatus();
  });
  server.on("/camera/config", HTTP_GET, []() { if (!denyApControllerAccess()) sendCameraConfig(); });
  server.on("/api/v1/camera/config", HTTP_GET, []() { if (!denyApControllerAccess()) sendCameraConfig(); });
  server.on("/camera/config", HTTP_POST, []() { if (!denyApControllerAccess()) saveCameraConfig(); });
  server.on("/camera/sign", HTTP_POST, []() { if (!denyApControllerAccess()) signCameraRequest(); });
  server.on("/calibrate", HTTP_GET, []() {
    if (denyApControllerAccess()) return;
    if (motionBusy()) { server.send(409, "application/json", "{\"error\":\"Stop the current movement before calibration\"}"); return; }
    beginCalibration();
    sendStatus();
  });
  server.on("/relay", HTTP_GET, []() {
    if (denyApControllerAccess()) return;
    String state = server.arg("state");
    if (state == "on") {
      relayOn();
      message = "Relay turned on.";
    } else if (state == "off") {
      relayOff();
      message = "Relay turned off.";
    } else {
      message = "Invalid relay state; use on or off.";
    }
    sendStatus();
  });
  server.on("/move", HTTP_GET, []() {
    if (denyApControllerAccess()) return;
    Axis *axis = axisFromRequest();
    char dir = server.arg("dir").length() ? server.arg("dir")[0] : '?';
    uint16_t steps = static_cast<uint16_t>(server.arg("steps").toInt());
    if (!axis || (dir != 'A' && dir != 'B')) { message = "Invalid axis or direction."; sendStatus(); return; }
    startMove(*axis, dir, steps);
  });
  server.on("/stop", HTTP_GET, []() {
    if (denyApControllerAccess()) return;
    Axis *axis = axisFromRequest();
    if (!axis) { message = "Select pan or tilt to stop."; sendStatus(); return; }
    if (bootPhase != BOOT_COMPLETE && bootPhase != BOOT_CANCELLED) {
      cancelStartupCalibration();
      sendStatus();
      return;
    }
    if ((axis == &pan && ptzMotion.panDirection) || (axis == &tilt && ptzMotion.tiltDirection)) stopContinuousAxis(*axis);
    if (axis->state == AXIS_MOVING || axis->state == AXIS_HOMING) {
      axis->state = AXIS_UNKNOWN;
      axis->homing = false;
      axis->positionKnown = false;
      releaseMotor(axis->pins);
      message = String(axis->name) + " stopped. Position is unknown; press Calibrate now to home.";
    } else message = String(axis->name) + " is already stopped.";
    sendStatus();
  });
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
}

void updateAxis(Axis &axis) {
  const int8_t holdDirection = (&axis == &pan) ? ptzMotion.panDirection : ptzMotion.tiltDirection;
  if (ptzMotion.active && holdDirection) {
    const uint32_t now = micros();
    if (now - axis.lastStepUs < continuousStepDelayUs(axis, ptzMotion.speed)) return;
    const bool towardHome = axis.directionB == axis.homeDirectionB;
    const int32_t nextPosition = axis.position + (towardHome ? -1 : 1);
    if (axis.positionKnown && (nextPosition < 0 || nextPosition > axis.maxPosition)) {
      stopContinuousAxis(axis);
      message = String(axis.name) + " stopped at software travel limit.";
      return;
    }
    axis.lastStepUs = now;
    uint8_t phase = axis.completed % 4;
    const uint8_t *sequence = axis.orderAway;
    if (axis.directionB) {
      sequence = axis.orderHome;
      if (axis.reverseHomeOrder) phase = (3 + 4 - phase) % 4;
    }
    setChannels(axis.pins, (1U << sequence[phase]) | (1U << sequence[(phase + 1) % 4]));
    ++axis.completed;
    axis.position = nextPosition;
    return;
  }
  if (axis.state != AXIS_HOMING && axis.state != AXIS_MOVING) return;
  uint32_t now = micros();
  const uint32_t stepDelayUs =
      (bootPhase == BOOT_COMPLETE || bootPhase == BOOT_CANCELLED)
          ? (&axis == &pan ? panManualStepDelayUs : tiltManualStepDelayUs)
          : (&axis == &pan ? panCalibrationStepDelayUs : tiltCalibrationStepDelayUs);
  if (now - axis.lastStepUs < stepDelayUs) return;
  axis.lastStepUs = now;
  uint8_t phase = axis.completed % 4;
  const uint8_t *sequence = axis.orderAway;
  if (axis.directionB) {
    sequence = axis.orderHome;
    if (axis.reverseHomeOrder) phase = (3 + 4 - phase) % 4;
  }
  setChannels(axis.pins, (1U << sequence[phase]) | (1U << sequence[(phase + 1) % 4]));
  ++axis.completed;
  if (axis.completed >= axis.target) {
    releaseMotor(axis.pins);
    if (axis.homing) {
      axis.position = 0;
      axis.positionKnown = true;
      axis.state = AXIS_HOME;
      message = String(axis.name) + " homing complete; at home.";
      Serial.printf("%s homing complete.\n", axis.name);
    } else if (axis.positionKnown) {
      axis.position += (axis.directionB == axis.homeDirectionB)
          ? -static_cast<int32_t>(axis.completed) : axis.completed;
      axis.state = AXIS_HOME;
      message = String(axis.name) + " move complete.";
    } else {
      axis.state = AXIS_UNKNOWN;
      message = String(axis.name) + " manual move complete; position remains unknown.";
    }
    axis.homing = false;
  }
}

void setup() {
  Serial.begin(115200);
  cameraPrefs.begin(cameraPrefsNs, false);
  wifiPrefs.begin("wifi", false);
  activeWifiSsid = wifiPrefs.getString("ssid", wifiSsid);
  activeWifiPassword = wifiPrefs.getString("password", wifiPassword);
  if (!cameraPrefs.getString("webtoken", "").length()) cameraPrefs.putString("webtoken", makeWebToken());
  for (uint8_t pin : panPins) pinMode(pin, OUTPUT);
  for (uint8_t pin : tiltPins) pinMode(pin, OUTPUT);
  pinMode(relayPin, INPUT);
  releaseMotor(panPins);
  releaseMotor(tiltPins);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiStartedAtMs = millis();
  lastWifiRetryMs = wifiStartedAtMs;
  WiFi.begin(activeWifiSsid.c_str(), activeWifiPassword.c_str());
  // Initialize the network stack before WebServer opens its listening socket.
  setupRoutes();
  Serial.println("Starting non-blocking Wi-Fi connection; dashboard and motor calibration remain active.");
  beginCalibration();
}

void loop() {
  const uint32_t now = millis();
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (!connected) {
    configApCloseAtMs = 0;
    if (!wifiDisconnectedAtMs) wifiDisconnectedAtMs = now;
    if (otaStarted) {
      ArduinoOTA.end();
      otaStarted = false;
    }
    if (now - lastWifiRetryMs >= 10000) {
      lastWifiRetryMs = now;
      WiFi.reconnect();
      Serial.println("Wi-Fi disconnected; reconnecting...");
    }
    if (!configApActive && ((now - wifiStartedAtMs >= 20000) || (wifiDisconnectedAtMs && now - wifiDisconnectedAtMs >= 20000))) {
      startConfigAp();
    }
  } else {
    if (wifiDisconnectedAtMs) {
      wifiDisconnectedAtMs = 0;
      Serial.printf("Wi-Fi connected. Dashboard: http://%s/\n", WiFi.localIP().toString().c_str());
    }
    startOta();
    ArduinoOTA.handle();
    if (configApActive && !configApManualHold) {
      if (!configApCloseAtMs) configApCloseAtMs = now + 60000;
      else if (static_cast<int32_t>(now - configApCloseAtMs) >= 0) stopConfigAp();
    }
  }
  server.handleClient();
  if (ptzMotion.active && millis() - ptzMotion.lastKeepaliveMs > ptzMotionWatchdogMs) {
    stopContinuousMotion();
    message = "Joystick keepalive timed out; motors stopped.";
  }
  updateAxis(pan);
  updateAxis(tilt);
  updateStartupCalibration();
}
