# james-gestures: hand control for the Cal Hacks arm

This is the JAMES gesture stack, packaged on its own to drive our backpack arm. A webcam watches one hand, and the code turns it into safe joint commands sent to the board over USB serial.

```
webcam -> MediaPipe (21 hand points) -> GestureReader -> ArmController -> serial -> arm_receiver.ino -> motors
```

## Run it (5 minutes)

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m pytest -q tests        # 16 tests, synthetic hands, no camera needed
python demo.py                   # dry run: opens the webcam and prints commands, nothing moves
python demo.py --port /dev/tty.usbmodemXXXX    # real arm (Mac); Linux is usually /dev/ttyACM0
```

`hand_landmarker.task` (Google's MediaPipe hand model) is included, so you don't need to download anything.

## Gestures

| Gesture | Arm does |
|---|---|
| Open palm, held 1 s | **ARM**: motion is enabled (the arm always starts disarmed) |
| Fist, held 0.3 s | **STOP**: freeze and disarm (a soft kill switch) |
| Move hand left/right | base (stepper) |
| Move hand up/down | shoulder |
| Hand closer/farther | elbow |
| Pinch / release | claw close / open |
| Hand leaves camera | holds still, disarms after 2 s |

In the demo, `h` `r` `e` `t` play the presets home / raise / reach / tuck, and `space` sends STOP. For voice, call `arm.goto_preset("raise")` from the speech side.

## Safety built in

- Starts disarmed. Nothing moves until a deliberate open palm.
- Every joint is clamped to its limits and **rate-limited in deg/s**, so a tracking glitch can't snap a joint across its range. The base is capped at 270° so the cables can't wrap.
- Only one hand drives the arm: the largest in view, i.e. the closest to the camera. Other hands are ignored.
- **This does not replace a hardware kill switch** on the servo power line. Wire one.

## What is in here

| File | What it is |
|---|---|
| `james_gestures/handpose.py` | From JAMES. Pose tracker (palm / point / pinch / peace) with per-finger hysteresis and short confirm times, plus the rule that a pinch starting in a fist is not a pinch, so curling your hand never closes the claw |
| `james_gestures/motion.py` | From JAMES. 1-euro smoothing (steady hand = steady servo), swipe detection, swept hit test |
| `james_gestures/gestures.py` | Combines the above into one `GestureFrame` per camera frame |
| `james_gestures/arm.py` | `ArmController`: gestures -> safe commands. Joint limits, speeds and presets live at the top |
| `james_gestures/tracker.py` | MediaPipe wrapper: camera frame -> the driving hand's 21 points |
| `demo.py` | Webcam demo with a HUD showing state, gesture and joint angles |
| `firmware/arm_receiver/arm_receiver.ino` | Arduino sketch: serial commands -> stepper + 4 servos |
| `tests/test_arm.py` | 16 tests: poses, noise rejection, arming, stop, claw, speed limits, hand lost, clamping |

## Serial protocol

One command per line, 115200 baud:

```
ARM 1 | ARM 0 | STOP | J <base|shoulder|elbow|wrist> <deg> | CLAW 1 | CLAW 0
```

The board is just a dumb executor, so you can swap the firmware for anything that speaks these lines.

## Tune on the real arm

- `default_joints()` in `arm.py`: each joint's `lo` / `hi` / `home` / `max_speed`. Start slow (30 deg/s) and raise it.
- `firmware`: pins, `STEPS_PER_DEG` (microstepping, gearing), `CLAW_OPEN` / `CLAW_CLOSED`.
- If pinch is too touchy, raise `GestureConfig.pinch_ratio` (0.33 default; a bit higher with gloves).

## Not hardware-tested yet

The Python side is tested with synthetic hands, and the MediaPipe pipeline runs end to end. The Arduino sketch compiles in principle but has never touched real motors, so treat its pins and limits as a starting point. On the **Arduino UNO Q**, check how the laptop's USB serial reaches the MCU sketch before relying on it.

---

Gesture code: MIT, © 2026 Chintha Victor Isaac (from JAMES, github.com/victorisaacchinta-eng/james). MediaPipe hand model: Google, Apache 2.0.
