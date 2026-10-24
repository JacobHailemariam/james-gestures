"""Check the laptop can talk to the arm sketch, without moving anything.

  python check_board.py COM3            # Windows (Mac: /dev/tty.usbmodemXXXX)

Sends PING and waits for PONG. Only PING is sent: the arm stays disarmed.
"""
from __future__ import annotations

import sys
import time

import serial


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    port = sys.argv[1]
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(2.0)                         # classic Arduinos reset when the port opens
    print("opened", port, "- leftover:", s.read(512) or b"(nothing)")
    s.write(b"PING\n")
    got, t = b"", time.time()
    while time.time() - t < 3 and b"PONG" not in got:
        got += s.read(256)
    s.close()
    print("reply:", got or b"(nothing)")
    if b"PONG" in got:
        print("OK: the sketch is running and hears the laptop")
    else:
        raise SystemExit("no PONG: is the sketch uploaded and running? (UNO Q: started from App Lab?)")


if __name__ == "__main__":
    main()
