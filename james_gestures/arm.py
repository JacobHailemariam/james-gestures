"""Gestures -> safe arm commands.

The safety model, in order of priority:
  1. The arm starts DISARMED. Nothing moves until you hold an open palm for `arm_hold_s`.
  2. A fist held for `stop_hold_s` sends STOP and disarms. Use it like a soft kill switch.
     (Keep a real hardware kill switch on the servo power line too. This is not a substitute.)
  3. If the hand leaves the camera, the arm holds still; after `lost_disarm_s` it disarms.
  4. Every joint target is clamped to its limits, smoothed, and rate-limited in deg/s,
     so a tracking glitch can never snap a joint across its range.

Steering while ARMED is a joystick, one joint at a time ("point to drive"):
  open palm             -> idle, nothing moves
  point (1 finger)      -> clutch in. Where you start pointing is the zero. Hand right/left of it turns the
                           base, above/below it lifts/lowers the shoulder. Speed grows with the offset
                           (dead zone, then up to max_speed). Only the larger axis moves. Un-point = stop.
  peace (2 fingers)     -> same joystick for the far joints: left/right = wrist, up/down = elbow
  pinch / release       -> claw close / open
Hand position is never mapped to an angle, so a jitter or a lean can't move the arm: only a deliberate
offset from where you clutched in does.
Voice (voice.py -> ArmController.voice): "stop" always works, "grab"/"release" drive the claw, a preset name
("raise", "reach", "home", "tuck") moves there and pauses hand steering until "follow". Voice can never ARM:
that still needs the held open palm. Presets obey the same clamps and rate limits as the hand.

Output is a small text protocol (one command per line), see README.md and firmware/arm_receiver_unoq/arm_receiver_unoq.ino:
  ARM 1 | ARM 0 | STOP | J <joint> <degrees> | CLAW 1 | CLAW 0
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

from .gestures import GestureFrame


@dataclass
class Joint:
    name: str
    lo: float
    hi: float
    home: float
    max_speed: float = 60.0     # deg/s, keep low near the user's head

    def clamp(self, v: float) -> float:
        return max(self.lo, min(self.hi, v))


def default_joints() -> dict[str, Joint]:
    # base capped at 270 deg so the cables can't wrap; tune lo/hi/home on the real arm
    return {
        "base": Joint("base", 0, 270, 135, 90),      # 45 felt slow in the sim; stepper can do ~110 at STEP_US=1000
        "shoulder": Joint("shoulder", 20, 160, 90, 45),
        "elbow": Joint("elbow", 20, 160, 90, 60),
        "wrist": Joint("wrist", 0, 180, 90, 90),
    }


PRESETS = {
    "home": {"base": 135, "shoulder": 90, "elbow": 90, "wrist": 90},
    "raise": {"shoulder": 150, "elbow": 120},
    "reach": {"shoulder": 110, "elbow": 40},
    "tuck": {"shoulder": 30, "elbow": 150},
    # The four demo poses. PLACEHOLDERS: raise/tuck with the base turned 45 deg either way. Nobody has measured
    # them on the real arm yet. Steer to the pose in the sim and say "james save left shoulder" (or edit
    # poses.json); saved poses override these at startup. Which way is "left" depends on the mounting.
    "left shoulder": {"base": 90, "shoulder": 150, "elbow": 120, "wrist": 90},
    "right shoulder": {"base": 180, "shoulder": 150, "elbow": 120, "wrist": 90},
    "left hip": {"base": 90, "shoulder": 30, "elbow": 150, "wrist": 90},
    "right hip": {"base": 180, "shoulder": 30, "elbow": 150, "wrist": 90},
}
POSES_FILE = "poses.json"      # user-saved poses, {name: {joint: deg}}; loaded over PRESETS by ArmController


def load_poses(path: str | Path | None = POSES_FILE) -> dict:
    """Merge saved poses from `path` into PRESETS. Missing file = nothing to do."""
    if not path or not Path(path).exists():
        return {}
    saved = json.loads(Path(path).read_text(encoding="utf-8"))
    for name, pose in saved.items():
        PRESETS[name] = {k: float(v) for k, v in pose.items()}
    return saved


def save_pose(name: str, pose: dict, path: str | Path | None = POSES_FILE) -> None:
    """Store `pose` under `name` in PRESETS and in the poses file (which keeps every saved pose)."""
    PRESETS[name] = {k: round(float(v), 1) for k, v in pose.items()}
    if path:
        p = Path(path)
        saved = json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}
        saved[name] = PRESETS[name]
        p.write_text(json.dumps(saved, indent=2) + "\n", encoding="utf-8")


@dataclass
class ArmConfig:
    arm_hold_s: float = 1.0       # open palm this long to arm
    stop_hold_s: float = 0.3      # fist this long to STOP
    lost_disarm_s: float = 2.0    # hand gone this long while armed -> disarm
    smooth: float = 0.25          # EMA factor on targets (0..1, lower = smoother, laggier)
    deadband_deg: float = 1.0     # don't resend a joint unless it moved this much
    stick_dead: float = 0.04      # joystick: offset from the clutch point (fraction of frame) ignored below this
    stick_full: float = 0.20      # ... and full speed at this offset
    stick_lock_axis: bool = True  # once a move starts on one joint, a twitch on the other axis can't switch
                                  # joints; un-point to pick again
    stick_max_s: float = 3.0      # a move may last this long after it starts; then the stick goes dead until
                                  # an open palm is shown (0 = no limit)
    mirror: bool = True           # webcam is mirrored: moving your hand right should turn the arm right
    pinch_claw: bool = True       # pinch closes / release opens the claw (set False to leave the claw to voice)
    poses_file: str | None = POSES_FILE  # saved poses ("james save left shoulder"); None = do not load or save
    voice_arms: bool = True       # a voice command (not stop/follow) arms the arm by itself when it is locked, so
                                  # "james home" works with no hand in view. The hand-lost disarm only applies
                                  # once a hand has been used since arming.
    claw_release_ratio: float = 0.46  # open the claw only once the thumb-index gap is really above this
                                  # (= GestureConfig.pinch_ratio * pinch_release). One noisy frame can
                                  # un-confirm a held pinch; that must not drop what the claw is holding.


class ArmController:
    def __init__(self, link, joints: dict[str, Joint] | None = None, cfg: ArmConfig | None = None,
                 clock=time.monotonic):
        self.link = link
        self.j = joints or default_joints()
        self.cfg = cfg or ArmConfig()
        self.clock = clock
        self.armed = False
        self.pos = {n: jt.home for n, jt in self.j.items()}       # where we told the arm to be
        self.target = dict(self.pos)
        self._sent = {}
        self._claw = None                 # last claw state sent (None = never sent)
        self._last_pinch = False          # claw follows pinch CHANGES, so a voice "grab" isn't undone next frame
        self.steering = True              # False while a preset plays: the hand stops aiming the arm
        self._anchor = None               # fingertip (norm) where the current point/peace began
        self.active_joint = None          # joint the joystick is driving right now (for the HUD / sim)
        self.drive = 0.0                  # -1..1 joystick deflection on that joint
        self.stick_gated = False          # move window used up: stick is dead until an open palm
        self._hand_used = False           # a hand has driven since the last ARM 1 (enables hand-lost disarm)
        load_poses(self.cfg.poses_file)
        self._stick_joint = None          # joint locked in for the current move
        self._move_since = None           # clock when the current move left the dead zone
        self._palm_since = self._fist_since = None
        self._last_seen = self._last_tick = clock()
        self._palm_ref = None
        self.state = "DISARMED"

    # ---------- public ----------
    def update(self, g: GestureFrame) -> str:
        now = self.clock()
        dt = max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        c = self.cfg

        if g.hand is None:
            self._palm_since = self._fist_since = None
            if self.armed and not self.steering:
                # a voice preset is playing: keep moving to it even with no hand in view
                self.state = "PRESET"
                self._step(dt)
                return self.state
            if self.armed and self._hand_used and now - self._last_seen > c.lost_disarm_s:
                self._disarm("hand lost")
            elif self.armed:
                self.state = "HOLD" if self._hand_used else "ARMED"   # voice-armed, no hand yet: just wait
                self.target = dict(self.pos)          # freeze where we are
            return self.state
        self._last_seen = now
        if self.armed:
            self._hand_used = True

        # fist -> STOP (checked first, always wins)
        self._fist_since = (self._fist_since or now) if g.fist else None
        if self._fist_since is not None and now - self._fist_since >= c.stop_hold_s:
            if self.armed or self.state != "STOPPED":
                self.link.send("STOP")
                self._disarm("fist", stopped=True)
            return self.state

        # open palm -> ARM
        self._palm_since = (self._palm_since or now) if g.open_palm else None
        if g.open_palm:
            self.stick_gated = False              # palm shown: the stick may be used again
        if not self.armed and self._palm_since is not None and now - self._palm_since >= c.arm_hold_s:
            self._arm()
            self._palm_ref = g.palm_px or None

        pinch_changed = g.pinch != self._last_pinch
        if pinch_changed and not g.pinch and g.pinch_ratio is not None and g.pinch_ratio <= c.claw_release_ratio:
            pinch_changed = False             # pinch lost to noise but the thumb is still on the finger: keep holding
        else:
            self._last_pinch = g.pinch
        if not self.armed:
            return self.state

        self.state = "ARMED" if self.steering else "PRESET"
        if self.steering:
            self._joystick(g, dt)
        if pinch_changed and c.pinch_claw:
            self.set_claw(1 if g.pinch else 0)
        self._step(dt)
        return self.state

    def stop(self) -> None:
        """Software STOP from a key, a voice command or a UI button: freeze and disarm."""
        self.link.send("STOP")
        self._disarm("stop", stopped=True)

    def set_claw(self, closed: int) -> bool:
        """Close (1) or open (0) the claw, from a pinch or from voice. Only while armed."""
        if not self.armed:
            return False
        if closed != self._claw:
            self._claw = closed
            self.link.send(f"CLAW {closed}")
        return True

    def follow(self) -> bool:
        """Hand steering back on after a preset (voice "follow")."""
        if not self.armed:
            return False
        self.steering = True
        return True

    def goto_preset(self, name: str) -> bool:
        """Move to a named pose. The hand stops steering until follow() so it can't drag the arm off the preset."""
        p = PRESETS.get(name)
        if p is None or not self.armed:
            return False
        for k, v in p.items():
            self.target[k] = self.j[k].clamp(v)
        self.steering = False
        return True

    def voice(self, cmd: str) -> bool:
        """Apply one voice command (see voice.py). STOP works even when disarmed. Any other command arms the
        arm first if it is locked (cfg.voice_arms), so voice works with no hand in the picture."""
        if cmd == "stop":
            self.stop()
            return True
        if cmd.startswith("save:"):                   # works armed or not: it records where the arm is now
            save_pose(cmd[5:], self.pos, self.cfg.poses_file)
            return True
        if not self.armed and self.cfg.voice_arms and cmd != "follow" and (cmd in ("grab", "release") or cmd in PRESETS):
            self._arm()
        if cmd == "grab":
            return self.set_claw(1)
        if cmd == "release":
            return self.set_claw(0)
        if cmd == "follow":
            return self.follow()
        return self.goto_preset(cmd)

    def tick(self) -> None:
        """Advance toward targets without a new gesture frame (e.g. while a preset plays)."""
        now = self.clock()
        dt = max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        if self.armed:
            self._step(dt)

    # ---------- internals ----------
    def _arm(self) -> None:
        self.armed, self.state = True, "ARMED"
        self.steering = True
        self._hand_used = False
        self._palm_since = None
        self.link.send("ARM 1")

    def _joystick(self, g: GestureFrame, dt: float) -> None:
        """Point / peace = clutch in; offset from where it began drives one joint at a speed."""
        c = self.cfg
        pair = ("base", "shoulder") if g.point else ("wrist", "elbow") if g.peace else None
        if pair is None or g.norm is None or self.stick_gated:
            self._anchor, self._stick_joint, self._move_since = None, None, None
            self.active_joint, self.drive = None, 0.0
            self.target = dict(self.pos)                            # idle (or gated): hold still
            return
        if self._anchor is None:
            self._anchor = g.norm
        dx, dy = g.norm[0] - self._anchor[0], self._anchor[1] - g.norm[1]   # dy: up = positive
        if c.mirror:
            dx = -dx
        if c.stick_lock_axis and self._stick_joint in pair:          # a move is under way: stay on its joint
            name = self._stick_joint
            off = dx if name == pair[0] else dy
        else:
            name, off = (pair[0], dx) if abs(dx) >= abs(dy) else (pair[1], dy)
        mag = (abs(off) - c.stick_dead) / max(c.stick_full - c.stick_dead, 1e-6)
        drive = (1 if off > 0 else -1) * max(0.0, min(1.0, mag))
        if drive != 0.0 and self._stick_joint is None:               # the move starts now
            self._stick_joint, self._move_since = name, self.clock()
        if c.stick_max_s and self._move_since is not None and self.clock() - self._move_since >= c.stick_max_s:
            self.stick_gated = True                                  # window over: dead until an open palm
            self._anchor, self._stick_joint, self._move_since = None, None, None
            self.active_joint, self.drive = None, 0.0
            self.target = dict(self.pos)
            return
        self.active_joint, self.drive = name, drive
        jt = self.j[name]
        self.target = dict(self.pos)
        self.target[name] = jt.clamp(self.pos[name] + drive * jt.max_speed * dt)

    def _aim(self, name: str, raw: float) -> None:
        jt = self.j[name]
        a = self.cfg.smooth
        self.target[name] = jt.clamp(self.target[name] + a * (jt.clamp(raw) - self.target[name]))

    def _step(self, dt: float) -> None:
        for n, jt in self.j.items():
            err = self.target[n] - self.pos[n]
            lim = jt.max_speed * dt
            self.pos[n] = jt.clamp(self.pos[n] + max(-lim, min(lim, err)))
            last = self._sent.get(n)
            if last is None or abs(self.pos[n] - last) >= self.cfg.deadband_deg:
                self.link.send(f"J {n} {self.pos[n]:.1f}")
                self._sent[n] = self.pos[n]

    def _disarm(self, why: str, stopped: bool = False) -> None:
        if self.armed and not stopped:
            self.link.send("ARM 0")
        self.armed = False
        self.state = "STOPPED" if stopped else "DISARMED"
        self.target = dict(self.pos)
        self.steering = True
        self._anchor, self.active_joint, self.drive = None, None, 0.0
        self._stick_joint, self._move_since, self.stick_gated = None, None, False
        self._palm_since = self._fist_since = None
        # the claw is NOT reset: re-arming must never open it by itself and drop what it is holding


@dataclass
class PrintLink:
    """Dry run: prints commands instead of sending them. Default, so nothing moves by accident."""
    log: list = field(default_factory=list)
    quiet: bool = False

    def send(self, line: str) -> None:
        self.log.append(line)
        if not self.quiet:
            print("->", line)


class SerialLink:
    """Writes commands to the microcontroller over USB serial (pip install pyserial).

    Survives the board dropping off USB (power cut, cable knocked, reboot): send() never raises. The link
    marks itself lost, reopens the port about once a second and then PINGs until the sketch answers PONG.
    A rebooted board starts disarmed, so the caller must disarm too: check `dropped` every frame."""

    RETRY_S = 1.0

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self._serial = serial
        self.port, self.baud = port, baud
        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2.0)            # most Arduinos reset when the port opens
        self.lost = False          # True while the board is unreachable (port gone or no PONG yet)
        self.dropped = False       # set when the link is lost; the caller clears it after disarming
        self._port_open = True
        self._next_try = 0.0
        self._inbuf = b""

    def send(self, line: str) -> None:
        if self.lost:
            self.poll()
            if self.lost:
                return
        try:
            self.ser.write((line + "\n").encode())
        except (self._serial.SerialException, OSError):
            self._mark_lost()

    def _mark_lost(self) -> None:
        self.lost = self.dropped = True
        self._port_open = False
        try:
            self.ser.close()
        except Exception:
            pass
        self._next_try = time.monotonic() + self.RETRY_S

    def poll(self) -> bool:
        """Call every frame. Retries the port while lost; returns True while the board is unreachable."""
        if not self.lost:
            return False
        now = time.monotonic()
        if now < self._next_try:
            return True
        self._next_try = now + self.RETRY_S
        try:
            if not self._port_open:
                self.ser = self._serial.Serial(self.port, self.baud, timeout=0)
                self._port_open = True
                self._inbuf = b""
            self._inbuf += self.ser.read(4096)
            if b"PONG" in self._inbuf:
                self.lost = False
                return False
            self.ser.write(b"PING\n")
        except (self._serial.SerialException, OSError):
            self._port_open = False
            try:
                self.ser.close()
            except Exception:
                pass
        return True

    def close(self) -> None:
        try:
            self.ser.write(b"STOP\n")
            self.ser.close()
        except Exception:
            pass
