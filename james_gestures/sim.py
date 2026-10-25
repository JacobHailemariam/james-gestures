"""MuJoCo stand-in for the arm: same link interface as SerialLink/PrintLink, so no hardware is needed.

    link = SimLink()                       # opens the viewer window
    arm = ArmController(link)

Understands the same lines the firmware does (ARM, STOP, J <joint> <deg>, CLAW). Joint angles are drawn kinematically
(no physics), with home (base 135, others 90) at the model's zero pose.

Needs: pip install mujoco
"""
from __future__ import annotations

import math
from pathlib import Path

MODEL = Path(__file__).resolve().parent.parent / "sim" / "arm.xml"
HOME = {"base": 135.0, "shoulder": 90.0, "elbow": 90.0, "wrist": 90.0}
CLAW_OPEN, CLAW_CLOSED = 0.0, 0.02          # slide of each finger toward the centre line, metres:
                                            # 0 = fingers at their mounts, 4 cm apart (open); 0.02 = touching (closed)


class SimLink:
    def __init__(self, model_path: str | Path = MODEL, show: bool = True):
        import mujoco
        import mujoco.viewer

        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.armed = False
        self._base_rgba = self.model.geom_rgba.copy()
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if show else None
        self._claw(0)
        self._refresh()

    def send(self, line: str) -> None:
        p = line.split()
        if not p:
            return
        if p[0] == "STOP":
            self.armed = False
        elif p[0] == "ARM" and len(p) == 2:
            self.armed = p[1] == "1"
        elif self.armed and p[0] == "CLAW" and len(p) == 2:
            self._claw(int(p[1]))
        elif self.armed and p[0] == "J" and len(p) == 3 and p[1] in HOME:
            self._joint(p[1], float(p[2]))
        self._refresh()

    def _joint(self, name: str, deg: float) -> None:
        self.data.joint(name).qpos[0] = math.radians(deg - HOME[name])

    def _claw(self, closed: int) -> None:
        g = CLAW_CLOSED if closed else CLAW_OPEN
        self.data.joint("claw").qpos[0] = -g
        self.data.joint("claw_r").qpos[0] = g

    def highlight(self, joint: str | None) -> None:
        """Light up the link the joystick is driving (geom named after the joint in arm.xml)."""
        for name in HOME:
            gid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_GEOM, name)
            if gid < 0:
                continue
            on = name == joint
            self.model.geom_rgba[gid] = (1.0, 0.95, 0.2, 1.0) if on else self._base_rgba[gid]
        self._refresh()

    def _refresh(self) -> None:
        self._mj.mj_forward(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()

    def close(self) -> None:
        if self.viewer is not None:
            self.viewer.close()
