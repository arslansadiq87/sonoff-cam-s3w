# ESP32 PTZ Camera Controller

Firmware and a web dashboard for a two-axis pan/tilt mount and relay, built for an ESP32-C6 and a Sonoff CAM-S3W. The ESP32 serves the control page, runs the stepper motors and exposes a REST API for Home Assistant. Camera video uses its native RPC/WebRTC session directly between the browser and camera; the ESP32 does not relay or transcode video.

## Features

- Pan and tilt control with an eight-direction joystick, adjustable speed, emergency stop and software travel limits.
- Automatic startup homing and centering, plus a calibration control.
- Three saveable presets and a Home position.
- Relay control for the camera light or another low-voltage load.
- Browser dashboard with direct Sonoff WebRTC video, quality controls and camera credential authorization.
- Versioned JSON/REST API and Home Assistant example configuration.
- Password-protected Arduino OTA updates after the first USB upload.

## Hardware and wiring

This project targets an ESP32-C6 board compatible with PlatformIO's `esp32-c6-devkitc-1` definition, two ULN2003 stepper driver boards, two 28BYJ-48 stepper motors, and a relay input.

| Function | ESP32-C6 GPIO |
| --- | --- |
| Pan IN1–IN4 | 18, 19, 20, 21 |
| Tilt IN1–IN4 | 0, 1, 14, 3 |
| Relay input | 22 |

Connect each motor to its ULN2003 board and connect the board input pins to the GPIOs above. Use a suitable external motor supply sized for both motors and connect its ground to ESP32 ground. Do not power motors from ESP32 GPIO pins. The relay output is active-low: firmware turns it on by driving GPIO22 LOW and turns it off by returning the pin to input mode. Verify relay-module voltage and isolation before wiring a load.

The motors have no end-stop sensors. Homing uses timed step counts and assumes the mechanism can reach its mechanical home without damage. Keep the mechanism clear during calibration and supervise initial movements. Software limits apply after calibration; they are not a substitute for physical end stops.

### Wiring diagrams

The `schematics/` folder contains wiring references for the controller and its USB circuit:

- [ESP32-C6, stepper drivers, motors and relay wiring](schematics/circuit-studio-3840x2160.png)
- [ESP32-S3 USB circuit schematic](schematics/esp32-s3-usb-circuit-schematic.png)

The USB schematic is for an ESP32-S3 circuit; the firmware build target described above is ESP32-C6. Check the board and pin labels on the diagrams against your hardware before wiring.

## Configure credentials

1. Install [Visual Studio Code](https://code.visualstudio.com/) with the PlatformIO IDE extension, or install PlatformIO Core.
2. Copy `src/secrets.example.h` to `src/secrets.h`.
3. Set `CAMERA_WIFI_SSID` and `CAMERA_WIFI_PASSWORD` to the 2.4 GHz Wi-Fi network the ESP32 will join. SSID capitalization matters.
4. Choose a unique `CAMERA_SETUP_AP_PASSWORD` of at least 8 characters. This protects the temporary setup access point.
5. Set a unique `CAMERA_OTA_PASSWORD`; the OTA helper reads it from the local secrets file.
6. Keep `CAMERA_OTA_HOSTNAME` as `ptz-camera` or choose another mDNS-safe host name.

`src/secrets.h` is ignored by Git. Never commit it. The ESP32 stores Wi-Fi and camera credentials in Preferences/NVS. This firmware does not enable flash encryption, so credentials in device storage are not encrypted at rest.

## Build and first upload over USB

1. Connect the ESP32-C6 by USB and identify its serial port.
2. Open this directory as a PlatformIO project.
3. Build:

   ```sh
   pio run -e esp32-c6-super-mini
   ```

4. Upload over USB, specifying the detected serial port if PlatformIO does not find it:

   ```sh
   pio run -e esp32-c6-super-mini -t upload --upload-port COM5
   ```

   On Linux or macOS, use the board device path, for example `/dev/ttyACM0`.
5. To view connection and calibration progress, open the serial monitor at 115200 baud:

   ```sh
   pio device monitor -b 115200 --port COM5
   ```

After joining Wi-Fi, the dashboard is served on port 80 at the ESP32's assigned address. Reserve that address in the router's DHCP settings if Home Assistant or other devices need a stable address.

## Wi-Fi setup and dashboard

The firmware first attempts credentials in `src/secrets.h`. If it cannot join, it starts a setup access point using `CAMERA_SETUP_AP_SSID` and `CAMERA_SETUP_AP_PASSWORD`. Connect a phone or computer to that network and open `http://192.168.4.1/`. Select the home Wi-Fi network and save its password. The controller reconnects and displays its LAN address in the setup portal and serial output.

Open `http://<ESP32_IP>/` on a device on the same LAN. Use the gear icon to open settings as a modal. In **System**, select **Restart controller** and confirm; the ESP32 releases the motors, restarts, and runs startup calibration. In **Connect to Wi-Fi**, scan nearby networks or type an SSID, enter its password, then select **Save and connect**. The controller stores the credentials and reconnects; it may receive a different IP address, so reserve its address in the router. The separate setup access point remains available as a recovery path. Startup homing begins automatically. Wait for calibration to complete before using position-based moves and presets. If calibration is stopped or interrupted, positions are unknown; run calibration again before relying on travel limits or presets.

### Configure the Sonoff camera stream

1. Open the ESP32 dashboard at `http://<ESP32_IP>/` (not the camera's own OpenEdge page).
2. Open **Camera settings** and enter the camera's LAN IP, username and password.
3. Save the settings. The camera password is stored in ESP32 Preferences and is not returned to the browser.
4. Each browser needs one-time authorization using the saved camera password. After authorization, that browser receives a session cookie and reconnects automatically.
5. Make sure the browser device and camera can exchange WebSocket/RPC and WebRTC traffic. RTSP/ONVIF must be enabled in OpenEdge for the camera firmware configuration used here.

Video travels directly from the camera to each browser using WebRTC. There is no MediaMTX gateway in this project. The Home Assistant REST API controls the ESP32; it does not expose camera video as a Home Assistant camera entity.

## Home Assistant

The ESP32 sensor and commands are in the package [home-assistant/packages/ptz_camera.yaml](home-assistant/packages/ptz_camera.yaml). The project file [home-assistant/configuration.yaml](home-assistant/configuration.yaml) shows the package loader. Copy the package into your Home Assistant `/config/packages/` folder, then add `homeassistant: packages: !include_dir_named packages` to your main `/config/configuration.yaml`. If it already has a `homeassistant:` section, add `packages:` under that existing section rather than creating a duplicate. In the package, replace the example ESP32 host `192.168.1.50` with its reserved address. Check Home Assistant's configuration and restart it.

The package adds a controller status sensor and commands for the relay, calibration, Home, saving the Home position, Preset 1 and a bounded pan move. The dashboard Home card also has a Save button. If a Home position has been saved, the next startup calibration homes both axes and then returns to that saved position; before the first save, it uses the current center behavior. Call commands in automations or scripts as `rest_command.ptz_light_on`, `rest_command.ptz_home`, and so on. The `ptz_pan_right` example runs 32 steps and then stops; use 1 through 256 steps. Preset recalls and calibration require idle motors; presets require valid calibration. The ONVIF camera itself is configured through **Settings → Devices & services**; the live view and PTZ dashboard card remains a separate Lovelace card YAML.

### Add the Sonoff live video stream

The ESP32 dashboard uses the camera's native RPC/WebRTC interface. Home Assistant's standard camera integrations use RTSP or ONVIF, so add the camera's separate RTSP/ONVIF stream; the REST controller API does not provide the video feed.

1. In the camera's OpenEdge local interface, enable **RTSP/ONVIF**. In its camera settings (often under **More Settings**), note the RTSP address, RTSP username and password. SONOFF's RTSP address format is `rtsp://username:password@<CAMERA_IP>:554/av_stream/ch0`; use the exact address shown by your camera if it differs. `ch0` is the primary stream; `ch1` is commonly the secondary/lower-resolution stream. [SONOFF's camera setup guide](https://cube.ewelink.cc/getting-started/add-devices/cameras) describes where to find the address and credentials.
2. Make sure Home Assistant and the camera can reach each other on the LAN. In Home Assistant, open **Settings → Devices & services → Add integration → ONVIF**. If the camera is discovered, select it and enter the ONVIF credentials. Home Assistant's ONVIF integration requires FFmpeg; Home Assistant OS and Container include it. If discovery fails, add **Generic Camera** instead and enter the RTSP stream source from the camera settings, plus its username and password. Home Assistant supports RTSP stream sources in Generic Camera; its Stream integration is included by `default_config`. [ONVIF setup](https://www.home-assistant.io/integrations/onvif), [Generic Camera setup](https://www.home-assistant.io/integrations/generic), and [Stream integration](https://www.home-assistant.io/integrations/stream) are documented by Home Assistant.
3. Add the combined live video and PTZ card to a dashboard. Go to **Edit dashboard → Add card → Manual** and paste the contents of [home-assistant/camera-dashboard-card.yaml](home-assistant/camera-dashboard-card.yaml). Replace each `camera.sonoff_cam_s3w` with the camera entity ID Home Assistant created. The directional buttons make relative moves of 0.15; adjust `distance` and `speed` (0 through 1) for your preference. The center button sends ONVIF `Stop`.

The dashboard card expects the ONVIF camera entity and ONVIF PTZ support; first add/configure the camera and PTZ through the ONVIF integration. Home Assistant's `onvif.ptz` action supports directional pan/tilt, speed, distance and stop modes. The supplied card uses one relative move per tap, not press-and-hold movement. Enter credentials in the integration setup rather than placing a real password in this public project. If Generic Camera is configured with only a stream source and no still image URL, the Home Assistant Stream integration must be enabled (it is automatic when using `default_config`). [ONVIF PTZ action documentation](https://www.home-assistant.io/actions/onvif.ptz)

## REST API

All routes are served by the ESP32 at `http://<ESP32_IP>`. Versioned routes use `/api/v1`. POST requests accept `application/x-www-form-urlencoded` and return JSON. The controller API currently has no API key or login. Keep it on a trusted private LAN; do not expose it through router port forwarding or the public internet.

| Method and path | Fields | Result |
| --- | --- | --- |
| `GET /api/v1/status` | — | Axis states and positions, calibration phase, relay state, joystick state and last message |
| `GET /api/v1/presets` | — | Calibration state and saved positions |
| `POST /api/v1/relay` | `state=on` or `state=off` | Set relay; returns updated status |
| `POST /api/v1/calibrate` | — | Begin homing and centering; returns status |
| `POST /api/v1/restart` | — | Safely release motors and restart the controller |
| `POST /api/v1/move` | `axis=pan|tilt`, `dir=A|B`, `steps=1..256` | Run a bounded step move |
| `POST /api/v1/stop` | `axis=pan|tilt` | Stop that axis; interrupted position becomes unknown |
| `POST /api/v1/preset/save` | `id=0..3` | Save Home (0) or a preset position |
| `POST /api/v1/preset/go` | `id=0..3` | Recall Home (0) or preset 1, 2 or 3 |
| `POST /api/v1/ptz/start` | `pan=-1..1`, `tilt=-1..1`, `speed=0..10`, `client`, `seq` | Start joystick-style continuous movement |
| `POST /api/v1/ptz/keepalive` | Same fields | Renew active movement |
| `POST /api/v1/ptz/stop` | Same fields | Stop continuous movement |
| `GET /api/v1/camera/config` | — | Camera IP, username and configured/authorized state; never includes password |

Continuous joystick clients must use a stable alphanumeric/hyphen `client` ID (12–40 characters) and increasing integer `seq` values. Send keepalive at least every 800 ms while a direction is held. A watchdog releases the motors when updates stop. For simple Home Assistant buttons, use bounded `/move` requests rather than continuous control.

Examples:

```sh
curl http://<ESP32_IP>/api/v1/status
curl -X POST -H 'Content-Type: application/x-www-form-urlencoded' \
  --data 'state=on' http://<ESP32_IP>/api/v1/relay
curl -X POST -H 'Content-Type: application/x-www-form-urlencoded' \
  --data 'axis=pan&dir=A&steps=32' http://<ESP32_IP>/api/v1/move
```

Invalid fields return HTTP 400. Commands blocked because calibration is incomplete, motors are busy, or a position limit would be exceeded return HTTP 409 with a JSON error or current status. Legacy dashboard routes remain for compatibility; integrations should use `/api/v1/...`.

## Motion behavior and safety

- Pan home sweep: 4,000 steps. Safe software travel: 3,904 steps.
- Tilt home sweep: 512 steps. Safe software travel: 448 steps.
- At power-up, both axes home together, then pan centers, then tilt centers.
- Manual API moves accept at most 256 steps per request.
- Joystick commands require calibrated positions and stop after 800 ms without keepalive.
- Joystick pan is twice as fast as its former rate: speed 5 steps every 10 ms; speed 1 every 25 ms; speed 10 is capped at 5 ms. Tilt is unchanged: 25 ms at speed 5, 60 ms at speed 1, 12.5 ms at speed 10.
- Joystick left/right command pan B/A. Up/down command tilt A/B; tilt direction is inverted in software because the camera mount is upside down.
- The relay is active-low and returns off after reboot.

Travel limits assume homing reached the mechanical stops correctly. Watch the mechanism during initial setup and recalibration; stop if it binds.

## OTA firmware updates

OTA becomes available after the board joins Wi-Fi. The hostname comes from `CAMERA_OTA_HOSTNAME` in `src/secrets.h`. The helper reads `CAMERA_OTA_PASSWORD` from that ignored file, so the password is not stored in tracked PlatformIO configuration.

Use the address reported by the router or serial monitor:

```sh
pio run -e esp32-c6-ota -t upload --upload-port 192.168.1.50
```

Replace the example address with the actual ESP32 address. OTA releases both motors and restarts the board; startup homing runs again after reboot. Keep the mechanism clear for the calibration sweep.

## Project layout

- `src/main.cpp`: Wi-Fi, HTTP API, motor control, calibration, presets, relay and OTA.
- `src/dashboard.h`: embedded dashboard HTML/CSS and browser-side camera WebRTC client.
- `src/secrets.example.h`: safe credential template; copy to ignored `src/secrets.h`.
- `home-assistant/packages/ptz_camera.yaml`: Home Assistant package with REST sensor and commands.
- `home-assistant/configuration.yaml`: package-loader example for the main HA configuration.
- `home-assistant/camera-dashboard-card.yaml`: Lovelace live camera and ONVIF PTZ card.
- `ota_upload_config.py`: injects the local OTA password into PlatformIO's upload command without committing it.
- `platformio.ini`: board and build environments.

Build artifacts, local secrets, editor configuration and historical checkpoints are excluded by `.gitignore`.
