"""Gestures -> safe arm commands.

The safety model, in order of priority:
  1. The arm starts DISARMED. Nothing moves until you hold an open palm for `arm_hold_s`.
  2. A fist held for `stop_hold_s` sends STOP and disarms. Use it like a soft kill switch.
     (Keep a real hardware kill switch on the servo power line too. This is not a substitute.)
  3. If the hand leaves the camera, the arm holds still; after `lost_disarm_s` it disarms.
  4. Every joint target is clamped to its limits, smoothed, and rate-limited in deg/s,
     so a tracking glitch can never snap a joint across its range.

Mapping while ARMED (all relative, so it works wherever you stand):
  fingertip left/right  -> base   (stepper)
  fingertip up/down     -> shoulder
  hand closer/farther   -> elbow  (palm size vs. the size when you armed)
  pinch / release       -> claw close / open
Voice can call `goto_preset("home")` etc. at any time; presets obey the same rate limits.

Output is a small text protocol (one command per line), see README.md and firmware/arm_receiver.ino:
  ARM 1 | ARM 0 | STOP | J <joint> <degrees> | CLAW 1 | CLAW 0
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

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
        "base": Joint("base", 0, 270, 135, 45),
        "shoulder": Joint("shoulder", 20, 160, 90, 45),
        "elbow": Joint("elbow", 20, 160, 90, 60),
        "wrist": Joint("wrist", 0, 180, 90, 90),
    }


PRESETS = {
    "home": {"base": 135, "shoulder": 90, "elbow": 90, "wrist": 90},
    "raise": {"shoulder": 150, "elbow": 120},
    "reach": {"shoulder": 110, "elbow": 40},
    "tuck": {"shoulder": 30, "elbow": 150},
}


@dataclass
class ArmConfig:
    arm_hold_s: float = 1.0       # open palm this long to arm
    stop_hold_s: float = 0.3      # fist this long to STOP
    lost_disarm_s: float = 2.0    # hand gone this long while armed -> disarm
    smooth: float = 0.25          # EMA factor on targets (0..1, lower = smoother, laggier)
    deadband_deg: float = 1.0     # don't resend a joint unless it moved this much
    elbow_gain: float = 120.0     # deg of elbow per 100% change in palm size
    mirror: bool = True           # webcam is mirrored: moving your hand right should turn the arm right


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
        self._claw = None
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
            if self.armed and now - self._last_seen > c.lost_disarm_s:
                self._disarm("hand lost")
            elif self.armed:
                self.state = "HOLD"
                self.target = dict(self.pos)          # freeze where we are
            return self.state
        self._last_seen = now

        # fist -> STOP (checked first, always wins)
        self._fist_since = (self._fist_since or now) if g.fist else None
        if self._fist_since is not None and now - self._fist_since >= c.stop_hold_s:
            if self.armed or self.state != "STOPPED":
                self.link.send("STOP")
                self._disarm("fist", stopped=True)
            return self.state

        # open palm -> ARM
        self._palm_since = (self._palm_since or now) if g.open_palm else None
        if not self.armed and self._palm_since is not None and now - self._palm_since >= c.arm_hold_s:
            self.armed, self.state = True, "ARMED"
            self._palm_ref = g.palm_px or None
            self.link.send("ARM 1")
            self._palm_since = None

        if not self.armed:
            return self.state

        self.state = "ARMED"
        if g.norm is not None:
            x, y = g.norm
            if c.mirror:
                x = 1.0 - x
            b, s = self.j["base"], self.j["shoulder"]
            self._aim("base", b.lo + x * (b.hi - b.lo))
            self._aim("shoulder", s.hi - y * (s.hi - s.lo))          # hand up -> shoulder up
            if self._palm_ref and g.palm_px:
                e = self.j["elbow"]
                change = (g.palm_px - self._palm_ref) / self._palm_ref    # + = hand closer
                self._aim("elbow", e.home - change * c.elbow_gain)        # closer -> extend
        claw = 1 if g.pinch else 0
        if claw != self._claw:
            self._claw = claw
            self.link.send(f"CLAW {claw}")
        self._step(dt)
        return self.state

    def stop(self) -> None:
        """Software STOP from a key, a voice command or a UI button: freeze and disarm."""
        self.link.send("STOP")
        self._disarm("stop", stopped=True)

    def goto_preset(self, name: str) -> bool:
        p = PRESETS.get(name)
        if p is None or not self.armed:
            return False
        for k, v in p.items():
            self.target[k] = self.j[k].clamp(v)
        return True

    def tick(self) -> None:
        """Advance toward targets without a new gesture frame (e.g. while a preset plays)."""
        now = self.clock()
        dt = max(0.0, min(0.2, now - self._last_tick))
        self._last_tick = now
        if self.armed:
            self._step(dt)

    # ---------- internals ----------
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
        self._palm_since = self._fist_since = None
        self._claw = None                     # resend claw state after re-arming


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
    """Writes commands to the microcontroller over USB serial (pip install pyserial)."""

    def __init__(self, port: str, baud: int = 115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2.0)            # most Arduinos reset when the port opens

    def send(self, line: str) -> None:
        self.ser.write((line + "\n").encode())

    def close(self) -> None:
        try:
            self.ser.write(b"STOP\n")
            self.ser.close()
        except Exception:
            pass
