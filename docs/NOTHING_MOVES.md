# Servos don't move on the real arm: handoff to Victor Isaac

Oct 25, 2026. Jacob, Semir and Victor E are at Cal Hacks with the arm. Victor Isaac is remote and has no
hardware, so this file lists exactly what was measured, what is ruled out, and what is left.

**The problem:** with the board on USB and the demo running, no servo on the arm moves. Not from voice, not
from gestures, not from typed commands.

## 1. What is proven to work (measured Oct 25, commit `a314bef`)

| Layer | Test | Result |
|---|---|---|
| Laptop to board | `SerialLink("COM3")`, then `ARM 1`, `ARM 0` | `OK ARM`, `OK DISARM`, board answers PING |
| Camera | `cv2.VideoCapture(0)` | live 640x480 frames, brightness 125 (not black) |
| MediaPipe | `HandTracker.process` on a frame | runs without error |
| Voice | `VoiceListener()` for 3 s | starts, `problem` is empty |
| Board to PCA9685 | bringup sketch `scan` | `found 0x40`, PCA9685 FOUND |
| PCA9685 output | bringup sketch `servo` | swept channel 4 from 60 to 120 degrees twice, no errors |
| Firmware commands | `J shoulder 130/50/90`, `J elbow 120/90`, `CLAW 1/0` sent raw to the receiver | accepted silently, as designed |
| Tests | `python -m pytest -q` | 81 passed |

So the software path from laptop to the PCA9685 chip works. **The open question is whether the servos
physically moved during the bringup sweep** (Jacob to confirm). That single answer splits the problem:

- **Bringup sweep did NOT move the shoulder** → it is power or wiring, not code (section 2).
- **Bringup sweep DID move it, receiver does not** → it is the receiver firmware (section 3).

## 2. If it is power or wiring (most likely)

The PCA9685 makes the control pulse. The servo's own red and black wires must get power from a separate
supply. Things that produce exactly "chip answers, servo dead":

1. **No power on the servo.** The 60 kg servo (shoulder, channel 4) needs 6 to 8.4 V on its red and black
   wires. The 25 kg servos and the 9 g claw servos need 5 to 6.8 V on the PCA9685's V+ screw terminal.
   The team's battery pack is 5 cells in series. If alkaline, that is ~7.5 V: OK for the 60 kg only,
   too high for the rest. Measure it with a multimeter while connected.
2. **No common ground.** The battery's negative must also connect to the board's GND, or the signal has
   no reference and the servo ignores it.
3. **The PCA9685's OE pin** pulled high (some boards have a jumper) disables every output.
4. **Wrong channel.** The signal wire must be on the channel's middle (yellow/orange) pin. Shoulder is
   channel 4, elbow 0, wrist 1, claw 15 (left) and 14 (right).
5. **Brown-out.** Twice tonight the UNO Q rebooted the moment motors drew power (its log just stops). After
   a reboot the sketch does not restart by itself, so commands go nowhere. Check: `python
   check_board.py COM3`. Likely cause: motor current flowing through the board's ground and USB cable.

## 3. If it is the receiver firmware

The last time a servo was **confirmed** to move on the real arm was commit `3d7506c`. Jacob said the
shoulder moved "way too fast" with `test_motors.py`. Every firmware change since then is a suspect:

| Commit | Change |
|---|---|
| `083ffc5` | Slew limit: commands set a goal, `tickServos()` in `loop()` moves the pulse 1.2 degrees per 20 ms |
| `3b136f2` | Per-channel trim; the first command on a channel snaps to 90 then slews |
| `44b20cc` | Two claw servos, channels 14 and 15 |
| `1d99838` | `WRIST_LOCK`: wrist held at 90 |
| `1d78dfa` | Claw channels swapped |

The firmware is `firmware/arm_receiver_unoq/arm_receiver_unoq.ino`. Look at `servoDeg`, `servoWrite`,
`tickServos`, `allServosOff`, and `loop()`. A quick test: flash the `3d7506c` version of that file (see
section 4) and send `J shoulder 110`. If it moves, the bug is in the slew code.

Nothing in the bringup sketch uses the slew code. That is why the two results above separate the cases.

## 4. Commands for whoever has the board

All from the `james-gestures` folder, with the venv active (`(.venv)` in the prompt), in Git Bash.

```
python check_board.py COM3                    # PONG = the receiver is running
python tools/flash_unoq.py                    # flash the receiver (compiles on the board, ~1 min)
python tools/flash_unoq.py --sketch firmware/bringup_unoq/bringup_unoq.ino   # flash the hardware test
python tools/flash_unoq.py --no-flash         # restart the sketch after a power cut
python test_motors.py COM3                    # type: arm, shoulder 110, shoulder 90, stop, quit
```

To test an old firmware version without losing the current one:
```
git show 3d7506c:firmware/arm_receiver_unoq/arm_receiver_unoq.ino > firmware/old_test.ino
python tools/flash_unoq.py --sketch firmware/old_test.ino
rm firmware/old_test.ino                      # don't commit it; reflash the current receiver afterwards
```

The bringup sketch is driven by typing words over the same serial port: `scan`, `servo`, `base`, `off`.
It does not answer PING, so `flash_unoq.py` prints "no PONG" after flashing it, which is expected.
Reflash the receiver afterwards.

## 5. What we need back from you

- Read section 3's firmware code for a reason servos would get no pulse after `J` commands while the
  bringup sketch's direct `pwm.writeMicroseconds` works.
- If you find it, push a fix plus a note on what to test. We will flash and report.
- Do not change the serial protocol or the pin and channel numbers; those are confirmed on the hardware.
