Import("env")
import pathlib
import re

secrets = pathlib.Path(env.subst("$PROJECT_DIR")) / "src" / "secrets.h"
if not secrets.exists():
    raise RuntimeError("Missing src/secrets.h; copy src/secrets.example.h and configure it first.")

text = secrets.read_text(encoding="utf-8")
match = re.search(r'^\s*#define\s+CAMERA_OTA_PASSWORD\s+"([^"\r\n]+)"', text, re.MULTILINE)
if not match or match.group(1).startswith("replace-"):
    raise RuntimeError("Set CAMERA_OTA_PASSWORD in src/secrets.h before OTA upload.")
env.Replace(UPLOAD_FLAGS=["--auth=" + match.group(1)])
