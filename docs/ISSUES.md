# Open issues: James arm (Cal Hacks 13.0)

Last updated Oct 25, 2026. Written so someone who has never seen the arm can understand where we
are stuck, reproduce each problem without our hardware, and push a fix.

## 1. What the system is, in one minute

```
webcam -> MediaPipe hand landmarks -> GestureReader -> ArmController -> USB serial (COM3) -> Arduino UNO Q
mic    -> Vosk, wake word "james"   -> VoiceListener --^                                       |
                                                                         PCA9685 servo board + TB6600 stepper
```

- **Laptop side (Python):** `demo.py` runs everything. `james_gestures/` holds the gesture, voice and
  control code. This is where most open issues live, and **all of it runs without hardware**.
- **Board side (C++):** `firmware/arm_receiver_unoq/arm_receiver_unoq.ino` runs on the UNO Q's
  microcontroller. It reads one text command per line and moves the motors.
- **Protocol (fixed, do not change):** `PING` (board replies `PONG`) | `ARM 1` | `ARM 0` | `STOP` |
  `J <base|shoulder|elbow|wrist> <deg>` | `CLAW 1` | `CLAW 0`.

The physical arm: a stepper turns the base; a 60 kg servo is the shoulder (red link); two 25 kg servos
are the elbow and the wrist (the two purple links); two small servos open and close a geared claw.

| Joint | Motor | Board output |
|---|---|---|
| base | NEMA17 stepper on a TB6600 driver | pins D9 (step), D3 (dir), D4 (enable) |
| shoulder | 60 kg servo | PCA9685 channel 4 |
| elbow | 25 kg servo | channel 0 |
| wrist | 25 kg servo, **held straight by the firmware** (claw is loose) | channel 1 |
| claw | two 9 g servos, mirrored | channel 15 (left jaw), 14 (right jaw) |

All servo angles: 90 = 1500 us = straight. "james home" puts every joint at 90.

## 2. Run it on your laptop, no hardware

```
git clone https://github.com/JacobHailemariam/james-gestures.git
cd james-gestures
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt -r requirements-voice.txt mujoco     (Mac/Linux: .venv/bin/pip)
.venv\Scripts\python -m pytest -q                       # should print: 66 passed
.venv\Scripts\python demo.py --sim --camera 0 --voice   # webcam + mic drive a 3D model of the arm
```

Without `--port`, nothing is sent to hardware: commands print in the terminal as `-> J shoulder 100.0`
and the MuJoCo window shows the arm on a model of our backpack.

Gestures: open palm held 1 s = arm. Point (one finger) and move = base (left/right) and shoulder
(up/down), like a joystick from where you started pointing. Peace sign = elbow (up/down). Pinch = claw.
Fist = stop. Voice: say "james" then a word: home, raise, reach, tuck, grab, release, stop, follow,
left shoulder, right shoulder, left hip, right hip.

Useful tools: `tools/eval_gestures.py` (synthetic gesture accuracy sweep), `tools/eval_voice.py` (records
the mic and counts false commands), `tools/solve_poses.py` (computes named poses from the CAD geometry).

## 3. Open issues, most important first

Each one has: what we see, how to reproduce it without hardware, what we already know, and what a fix
probably looks like. Line numbers are for commit `1d78dfa`.

### Issue 1: after any voice preset, hand gestures do nothing

**What we see.** We say "james home", the arm moves. Then open palm, point and peace sign do nothing
at all. From our side it looked like "gestures never work".

**Cause (confirmed by a code review and a reproduction).** `ArmController.goto_preset` in
`james_gestures/arm.py` sets `self.steering = False` so the hand can't drag the arm off a preset. Nothing
turns it back on except saying "james follow" or pressing `f`. The only hint is a small grey line at the
bottom of the window.

**Reproduce.** `python demo.py --sim --camera 0 --voice`. Say "james home". Hold an open palm, then point
and move: the HUD says PRESET and nothing moves. Say "james follow": now it works.

**Fix we wrote and then reverted** (commit `6207a83`, still in history): holding an open palm for 1 s
during a preset gives control back to the hand. We reverted the whole laptop side to a known-good
version under time pressure, so this needs to be re-applied cleanly with its test
(`test_open_palm_takes_steering_back_after_a_preset` in that commit).

### Issue 2: real hands are recognised much worse than our tests suggest

**What we see.** All 66 tests pass, and a synthetic sweep (`tools/eval_gestures.py`) says 100% accuracy.
On a real webcam in a bright, noisy hall, arming with an open palm is unreliable and point/peace often
read as something else.

**What we know.**
- Tests use perfect synthetic hands (`handpose.synth`). Real MediaPipe landmarks from a laptop camera at
  an angle produce finger ratios the thresholds were never checked against.
- One dropped tracking frame resets the 1 s open-palm timer (`arm.py`, the `g.hand is None` branch), so a
  flickering hand may never finish arming. Fixed in `6207a83`, reverted with Issue 1.
- Pinch is checked before point/peace in `handpose.py`; a straight index with the thumb close beside it can
  read as pinch.
- Any hand in view counts, including someone far in the background.

**What would help most.** Run the demo with your own hand and watch the HUD line "fingers a b c d gap e".
Those are the numbers the classifier sees. Tell us, for an open palm, a point and a peace sign, what the
four finger ratios and the gap are. Thresholds are in `james_gestures/handpose.py` (`UP_ON`, `DOWN`,
`CONFIRM_S`) and `james_gestures/gestures.py` (`GestureConfig`). You know this code best: it is your JAMES
`handpose.py`, unchanged.

### Issue 3: the board reboots when motors draw power

**What we see.** The UNO Q's Linux side resets (its log just stops, no shutdown message) when the stepper
driver is enabled or a big servo moves. After a reboot the sketch does not restart by itself, so every
command is silently ignored until we run `python tools/flash_unoq.py --no-flash`.

**What we know.** With all motor power disconnected, `ARM 1` and joint commands run fine for minutes,
so it is electrical, not code. Likely the motor return current flows through the board's ground and
USB cable. The firmware already only enables the stepper while it moves.

**Software side still open.** The demo never checks that the board answers, so after a reboot it keeps
showing ARMED while the board ignores everything. A PING-on-start check and a BOARD LOST warning were in
`6207a83` and got reverted. There is also no auto-start of the sketch after a reboot (a cron line on the
board, see the end of this file).

### Issue 4: commands arrive late

**What we see.** The arm sometimes executes commands seconds after they were given.

**What we know.** The demo sends a joint update every frame (about 30 per second per moving joint).
The UNO Q passes every byte from the laptop through its Linux side to the microcontroller. We
limited sends to 10 per second per joint in `845621d` (reverted with the rest). Two demo instances running
at once also produced exactly this symptom once, so check `tasklist | findstr python` first.

### Issue 5: voice fires on noise, or dies silently

**What we know.** Without the wake word, hall noise produced 6.6 false commands per minute. With "james"
required, 0 in 100 seconds. Still open: if the mic stream stops or the listener thread throws, voice just
stops and nothing on screen says so (`james_gestures/voice.py`, `VoiceListener._loop` and `poll`). Also
untested: how reliably a real person saying "james stop" in a loud room is recognised.

### Issue 6: calibration we can only do on the real arm

Not for you, listed so you know the state: the two claw servos' open/closed angles (20/95 are
placeholders) and whether they close toward each other; whether channel 0 is really the elbow and 1 the
wrist; the stepper direction; the named poses (`left shoulder` etc.) were computed from CAD, not measured.

## 4. How to work on this with us

- Branch from `main`, run `python -m pytest -q` before every commit, push, and tell us in Slack.
- Do not change the serial protocol or the firmware without asking Jacob; he owns `firmware/` and
  `check_board.py`.
- Never force-push.
- Fixes for Issues 1, 2 and 5 can be built and fully tested on your laptop with `--sim`.

Board auto-start after a power cut (run on a laptop with the board on USB; Jacob has to approve it):
```
adb shell
(crontab -l 2>/dev/null; echo "@reboot sleep 40 && TMPDIR=/tmp /usr/bin/arduino-app-cli app start user:motorcontrols") | crontab -
```
