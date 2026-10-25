"""Push the receiver sketch to the Arduino UNO Q over adb, restart the app, and check it answers PONG.

  .venv/Scripts/python.exe tools/flash_unoq.py            # flash firmware/arm_receiver_unoq, then PING COM3
  .venv/Scripts/python.exe tools/flash_unoq.py --port COM4
  .venv/Scripts/python.exe tools/flash_unoq.py --sketch firmware/bringup_unoq/bringup_unoq.ino

Needs the board on USB (adb sees it) and App Lab installed once, for its adb.exe and the app on the board.
The compile happens on the board and can take a few minutes. Nothing moves: the sketch starts disarmed.
Set ADB=<path to adb.exe> if yours is somewhere else.
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ADB = Path.home() / "AppData/Local/Arduino15/packages/arduino/tools/adb/32.0.0/adb.exe"
APP = "motorcontrols"                                   # name of the app on the board (App Lab created it)
APP_DIR = f"/home/arduino/ArduinoApps/{APP}"


def run(*args: str, timeout: int = 600) -> str:
    r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    return (r.stdout + r.stderr).strip()


def ping(port: str, wait: float = 3.0) -> bool:
    import serial
    try:
        s = serial.Serial(port, 115200, timeout=0.2)
    except Exception as e:
        print("could not open", port, "-", e)
        return False
    with s:
        time.sleep(2.0)
        s.read(4096)
        s.write(b"PING\n")
        got, t = b"", time.time()
        while time.time() - t < wait and b"PONG" not in got:
            got += s.read(256)
    return b"PONG" in got


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sketch", default="firmware/arm_receiver_unoq/arm_receiver_unoq.ino")
    ap.add_argument("--port", default="COM3", help="serial port to PING afterwards")
    ap.add_argument("--no-flash", action="store_true", help="only restart the app and PING")
    a = ap.parse_args()
    adb = os.environ.get("ADB", str(DEFAULT_ADB))
    if not Path(adb).exists():
        sys.exit(f"adb not found at {adb}: install Arduino App Lab or set ADB=<path>")
    if "device" not in run(adb, "devices").replace("List of devices attached", ""):
        sys.exit("no board on adb: is the UNO Q plugged in and booted (about 40 s after power)?")

    if not a.no_flash:
        src = ROOT / a.sketch
        if not src.exists():
            sys.exit(f"no such sketch: {src}")
        print("pushing", src.name, "->", f"{APP_DIR}/sketch/sketch.ino")
        print(run(adb, "push", str(src), f"{APP_DIR}/sketch/sketch.ino"))
    print("restarting app on the board (compiles there, be patient)...")
    out = run(adb, "shell", f"TMPDIR=/tmp arduino-app-cli app restart user:{APP}")
    tail = [l for l in out.splitlines() if "error" in l.lower() or "restarted" in l.lower()]
    print("\n".join(tail[-5:]) or out[-800:])
    ok = ping(a.port)
    print("PONG: yes, the sketch is running" if ok else "no PONG: the sketch is not answering (see README troubleshooting)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
