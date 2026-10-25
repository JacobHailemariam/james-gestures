"""Drive the arm by typing commands, one joint at a time. For first power-up and motor checks.

  python test_motors.py COM3

Then type:
  arm                 enable motion (servos get no pulse until the first move: use 'home' to bring them in)
  shoulder 100        move a joint (base | shoulder | elbow | wrist) to that many degrees
  shoulder +5         nudge it by 5 degrees (or -5)
  claw 1 / claw 0     close / open
  home                all joints to home (base 135, others 90)
  stop                freeze and disarm (also Ctrl+C / closing the window)
  quit

Every move is clamped to the limits in james_gestures/arm.py. Keep a hand on the kill switch anyway.
"""
from __future__ import annotations

import sys
import time

from james_gestures import SerialLink, default_joints


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    joints = default_joints()
    pos = {n: j.home for n, j in joints.items()}
    link = SerialLink(sys.argv[1])
    print("connected. type 'arm' to enable, 'quit' to exit.")
    try:
        while True:
            try:
                line = input("> ").strip().lower()
            except EOFError:
                break
            if not line:
                continue
            p = line.split()
            if p[0] in ("quit", "q", "exit"):
                break
            if p[0] == "arm":
                link.send("ARM 1")
            elif p[0] == "stop":
                link.send("STOP")
            elif p[0] == "home":
                link.send("ARM 1")
                for n, j in joints.items():
                    pos[n] = j.home
                    link.send(f"J {n} {j.home:.1f}")
                    time.sleep(0.3)
            elif p[0] == "claw" and len(p) == 2:
                link.send(f"CLAW {1 if p[1] == '1' else 0}")
            elif p[0] in joints and len(p) == 2:
                j = joints[p[0]]
                try:
                    v = pos[p[0]] + float(p[1]) if p[1][0] in "+-" else float(p[1])
                except ValueError:
                    print("bad number")
                    continue
                v = j.clamp(v)
                pos[p[0]] = v
                link.send(f"J {p[0]} {v:.1f}")
                print(f"  {p[0]} -> {v:.1f}  (limits {j.lo}..{j.hi})")
            else:
                print("?  try: arm | shoulder 100 | elbow +5 | claw 1 | home | stop | quit")
            time.sleep(0.05)
            waiting = link.ser.read(200)
            if waiting:
                print("  board:", waiting.decode(errors="replace").strip())
    finally:
        link.close()
        print("STOP sent, port closed.")


if __name__ == "__main__":
    main()
