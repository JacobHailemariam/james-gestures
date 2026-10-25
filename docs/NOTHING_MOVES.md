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

---

## 6. Answer from Victor Isaac (remote)

**Short version: the receiver code path is correct, but it had one real hole that matches your symptom
exactly, and the power supply in your photos most likely can't drive these servos. Both are addressed below.**

### 6.1 What I checked in the firmware

I compiled `arm_receiver_unoq.ino` on a PC against mock Arduino, Wire, Monitor and PCA9685 libraries and
replayed your exact sequence: `PING`, `ARM 1`, `J base 135`, `J shoulder 130`, `J elbow 120`, `J wrist 90`,
`CLAW 1`. The receiver writes the right pulses: shoulder ends at 1796 us (130 degrees), elbow 1722 us, both
claw jaws mirrored, STOP freezes. The slew, trim and claw changes since `3d7506c` are not the bug. So the
software path from `J` to `writeMicroseconds` works, like your bringup sketch showed.

### 6.2 The hole that matches "chip answers, nothing moves"

When the PCA9685's own supply dips (a servo's start-up current, a shared ground), the chip **resets itself**:
it wakes up asleep, at 200 Hz, with every output off, **and it still answers on I2C**. The receiver only set it
up once at boot, so after one dip every later command was accepted and nothing moved. That is exactly the
state in your table: board answers, `OK ARM`, commands accepted, no motion.

**Fixed (this commit):**

1. **Watchdog.** Every 250 ms the receiver checks the chip's frequency setting. If the chip has reset, it
   sets it up again, restores every pulse it was making, and prints
   `WARN PCA9685 had reset (power dip on its VCC?), restored. resets=N`.
   It also prints `WARN PCA9685 not answering on I2C` if the chip disappears.
2. **One servo wakes at a time.** `ARM 1` plus the first commands used to start shoulder, elbow, wrist and both
   claw servos in the same millisecond, five stall currents at once, which is a classic brown-out. New
   channels now come up one every 300 ms. Each prints `OK ENERGIZE chN`.

Protocol, pins and channel numbers are unchanged. The laptop ignores the new lines; `test_motors.py` shows
them as `board: ...`, which is the point: you can now see what the board is doing.

### 6.3 Power: the most likely root cause (from the team photos)

The photos show the servo power coming from a small breadboard power module (the black board with the
barrel jack and USB socket), and a 9 V battery on the table. A module like that is usually limited to about
0.5 to 1 A, and a 9 V battery to well under 1 A. Your servos need far more:

| Servo | Voltage | Current when it starts or holds a load |
|---|---|---|
| Shoulder, 60 kg (ch 4) | 6 to 8.4 V | several amps |
| Elbow, wrist, 25 kg (ch 0, 1) | 5 to 6.8 V | 2 to 3 A each |
| Claw, 9 g (ch 14, 15) | 4.8 to 6 V | under 1 A each |

What it needs: **one supply of 6 V with at least 5 A** to the PCA9685's V+ screw terminal (a bench supply
from the hardware desk, a 5 A+ BEC/UBEC set to 6 V from a 2S LiPo, or a 4xAA NiMH pack as a stopgap), and
**its negative also wired to the board's GND**. If the 60 kg servo gets its own 7.4 V supply instead, its
negative must join the same ground too. Do not put 7.5 V on V+: that is too high for the 25 kg and 9 g servos.

### 6.4 Three-minute test, in this order

1. Flash the new receiver: `python tools/flash_unoq.py`, then `python check_board.py COM3` (expect `PONG`).
2. Multimeter on DC volts across the PCA9685 **V+ and GND** screw terminals. Expect 5 to 6.8 V at rest.
3. `python test_motors.py COM3`, type `arm`, then `shoulder 110`. Expect `board: OK ENERGIZE ch4` and the
   shoulder to stiffen, then move. Watch the meter while it moves:
   - **Voltage drops below about 4.5 V, or the board stops answering** → the supply is too weak (6.3).
   - **`board: WARN PCA9685 had reset`** → the chip lost power for a moment: supply or ground (6.3).
     It now recovers by itself, but the supply still needs fixing.
   - **`OK ENERGIZE ch4`, steady voltage, servo limp** → check the servo plug orientation on channel 4
     (signal on the yellow/orange middle-side pin, brown/black to GND) and the OE pin/jumper (must be low).
   - **No `OK ENERGIZE` line at all** → the command never arrived: run `check_board.py` again; if no PONG,
     `python tools/flash_unoq.py --no-flash`.
4. Once the shoulder moves, run `demo.py --port COM3`. Servos now wake one after another after arming;
   that short stagger is intentional.
