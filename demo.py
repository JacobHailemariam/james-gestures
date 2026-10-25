"""Webcam demo: see what the gesture reader sees, and what the arm would be told.

  python demo.py                         # dry run, prints commands, nothing moves
  python demo.py --port COM3             # send to the board over serial (Mac: /dev/tty.usbmodemXXXX)
  python demo.py --camera 1              # another webcam
  python demo.py --voice                 # also listen for voice commands (pip install -r requirements-voice.txt)
  python demo.py --sim                   # drive a MuJoCo model of the arm instead (pip install mujoco)
  python demo.py --port COM3 --sim       # both: the sim shows what the real arm was told to do

Steering: open palm = arm / idle · POINT and move: left/right = base, up/down = shoulder · PEACE and move:
left/right = wrist, up/down = elbow · un-point = stop · pinch = claw · fist = STOP
Keys: q quit · h home preset · r raise · e reach · t tuck · f follow (hand steers again) · space = STOP
Voice: stop · grab · release · raise · reach · home · tuck · follow   (see james_gestures/voice.py)
"""
from __future__ import annotations

import argparse

import cv2

from james_gestures import ArmConfig, ArmController, GestureReader, PrintLink, SerialLink, default_joints
from james_gestures.tracker import HandTracker

LINKS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
STATE_COLOR = {"DISARMED": (160, 160, 160), "ARMED": (90, 200, 90), "HOLD": (0, 190, 255), "STOPPED": (60, 60, 230),
               "PRESET": (230, 160, 60)}


class TeeLink:
    """Sends every command to several links (e.g. the MuJoCo sim and the real board)."""

    def __init__(self, *links):
        self.links = links

    def send(self, line: str) -> None:
        for l in self.links:
            l.send(line)

    def close(self) -> None:
        for l in self.links:
            if hasattr(l, "close"):
                l.close()


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
    ap.add_argument("--voice", action="store_true", help="listen for voice commands (offline, Vosk)")
    ap.add_argument("--vosk-model", help="folder of an unzipped Vosk model (default: auto-download small English)")
    ap.add_argument("--mic", type=int, help="microphone device number (default: system default)")
    ap.add_argument("--no-pinch-claw", action="store_true", help="claw only by voice, pinch does nothing")
    ap.add_argument("--sim", action="store_true", help="drive a MuJoCo model of the arm instead of the board")
    ap.add_argument("--speed", type=float, default=1.0,
                    help="multiply every joint's max speed (deg/s) by this, e.g. 2 for a snappier sim")
    a = ap.parse_args()

    link = SerialLink(a.port, a.baud) if a.port else PrintLink()
    if a.sim:                                 # with --port too: the sim shows what the real arm was told
        from james_gestures.sim import SimLink
        link = TeeLink(SimLink(), link)
    # the frame is flipped below, so the controller must not mirror it a second time
    joints = default_joints()
    for jt in joints.values():
        jt.max_speed *= a.speed
    arm = ArmController(link, joints, cfg=ArmConfig(mirror=False, pinch_claw=not a.no_pinch_claw))
    reader, tracker = GestureReader(), HandTracker(a.model)
    voice, heard = None, ""
    if a.voice:
        from james_gestures.voice import VoiceListener
        print("loading voice model (first run downloads ~40 MB)...")
        voice = VoiceListener(a.vosk_model, device=a.mic)
        print("voice ready: say 'james' then a word: james stop / grab / release / raise / reach / home / tuck / follow")
    cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        raise SystemExit(f"camera {a.camera} not available")
    board = next((l for l in getattr(link, "links", [link]) if isinstance(l, SerialLink)), None)
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)            # mirror so it feels like a mirror
            h, w = frame.shape[:2]
            tr = tracker.process(frame)
            g = reader.read(tr.hand, w, h)
            if voice:
                for cmd in voice.poll():
                    ok = arm.voice(cmd)
                    heard = f"{cmd}" + ("" if ok else " (ignored)")
                    print("voice:", heard)
            state = arm.update(g)
            if board and board.poll() and board.dropped:   # board rebooted or USB dropped: it is disarmed, so are we
                board.dropped = False
                arm.stop()
                print("board lost: check its power and USB; reconnecting...")

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
            if not arm.armed:
                hint = "hold open palm to arm"
            elif not arm.steering:
                hint = "preset playing · say 'follow' or press f to steer again · fist = STOP"
            elif arm.stick_gated:
                hint = "move window over: show an open palm, then point again"
            elif arm.active_joint:
                hint = f"driving {arm.active_joint.upper()}  (un-point to stop)"
            else:
                hint = "POINT: left/right=base up/down=shoulder · PEACE: left/right=wrist up/down=elbow · pinch=claw · fist=STOP"
            cv2.putText(frame, hint, (12, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
            if arm.active_joint:                                  # joystick bar: centre = zero, fill = speed
                cx, cy, half = w // 2, h - 70, 120
                cv2.rectangle(frame, (cx - half, cy - 8), (cx + half, cy + 8), (80, 80, 80), 1)
                cv2.rectangle(frame, (cx, cy - 7), (cx + int(arm.drive * half), cy + 7), (60, 220, 255), -1)
                degs = arm.drive * arm.j[arm.active_joint].max_speed
                cv2.putText(frame, f"{arm.active_joint.upper()} {degs:+.0f} deg/s", (cx - 70, cy - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (60, 220, 255), 2, cv2.LINE_AA)
            sim = next((l for l in getattr(link, "links", [link]) if hasattr(l, "highlight")), None)
            if sim:
                sim.highlight(arm.active_joint)
            # tuning readout: finger fold ratios (fist needs ALL <= fist_max_ratio) and thumb-index gap
            if g.ratios:
                tune = "fingers " + " ".join(f"{r:.2f}" for r in g.ratios) + f"   gap {g.pinch_ratio:.2f}"
                cv2.putText(frame, tune, (12, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
            if voice:
                cv2.putText(frame, f"voice: {heard or '...'}", (12, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (255, 210, 120), 1, cv2.LINE_AA)
            if board and board.lost:
                cv2.putText(frame, "BOARD LOST: check power/USB, waiting for PONG...", (12, 122),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow("james-gestures", frame)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                arm.stop()
            if k == ord("f"):
                arm.follow()
            presets = {ord("h"): "home", ord("r"): "raise", ord("e"): "reach", ord("t"): "tuck"}
            if k in presets:
                arm.goto_preset(presets[k])
    finally:
        link.send("STOP")
        if voice:
            voice.close()
        cap.release()
        tracker.close()
        cv2.destroyAllWindows()
        if hasattr(link, "close"):
            link.close()


if __name__ == "__main__":
    main()
