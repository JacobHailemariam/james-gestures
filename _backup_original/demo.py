"""Webcam demo: see what the gesture reader sees, and what the arm would be told.

  python demo.py                         # dry run, prints commands, nothing moves
  python demo.py --port /dev/ttyACM0     # send to the board over serial (Mac: /dev/tty.usbmodemXXXX)
  python demo.py --camera 1              # another webcam

Keys: q quit · h home preset · r raise · e reach · t tuck · space = STOP
"""
from __future__ import annotations

import argparse

import cv2

from james_gestures import ArmConfig, ArmController, GestureReader, PrintLink, SerialLink
from james_gestures.tracker import HandTracker

LINKS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
STATE_COLOR = {"DISARMED": (160, 160, 160), "ARMED": (90, 200, 90), "HOLD": (0, 190, 255), "STOPPED": (60, 60, 230)}


def draw_hand(img, pts, color, thick=2):
    p = [(int(x), int(y)) for x, y in pts]
    for a, b in LINKS:
        cv2.line(img, p[a], p[b], color, thick, cv2.LINE_AA)
    for q in p:
        cv2.circle(img, q, 3, color, -1, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port of the arm board (omit for a dry run)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", default="hand_landmarker.task")
    a = ap.parse_args()

    link = SerialLink(a.port, a.baud) if a.port else PrintLink()
    # the frame is flipped below, so the controller must not mirror it a second time
    arm = ArmController(link, cfg=ArmConfig(mirror=False))
    reader, tracker = GestureReader(), HandTracker(a.model)
    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit(f"camera {a.camera} not available")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)            # mirror so it feels like a mirror
            h, w = frame.shape[:2]
            tr = tracker.process(frame)
            g = reader.read(tr.hand, w, h)
            state = arm.update(g)

            for o in tr.others:
                draw_hand(frame, o, (90, 90, 90), 1)
            if g.hand:
                draw_hand(frame, g.hand, STATE_COLOR.get(state, (255, 255, 255)))
            col = STATE_COLOR.get(state, (255, 255, 255))
            cv2.rectangle(frame, (0, 0), (w, 44), (20, 20, 20), -1)
            cv2.putText(frame, f"{state}   gesture: {g.name}", (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2,
                        cv2.LINE_AA)
            joints = "  ".join(f"{k} {v:5.1f}" for k, v in arm.pos.items())
            cv2.putText(frame, joints, (12, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
            hint = "hold open palm to arm" if not arm.armed else "move to steer · pinch = claw · fist = STOP"
            cv2.putText(frame, hint, (12, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
            cv2.imshow("james-gestures", frame)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                arm.stop()
            presets = {ord("h"): "home", ord("r"): "raise", ord("e"): "reach", ord("t"): "tuck"}
            if k in presets:
                arm.goto_preset(presets[k])
    finally:
        link.send("STOP")
        cap.release()
        tracker.close()
        cv2.destroyAllWindows()
        if hasattr(link, "close"):
            link.close()


if __name__ == "__main__":
    main()
