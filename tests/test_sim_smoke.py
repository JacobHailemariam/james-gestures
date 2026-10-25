"""MuJoCo smoke test: ArmController -> SimLink (headless), every preset ends where it was told to.

SimLink is kinematic (joint angles are written straight into qpos, no actuators), so "stepping the sim" here
means ticking the rate-limited controller with a fake clock until the joints have had time to arrive, then
reading the model's qpos back. Skips cleanly when mujoco is not installed."""
import math

import pytest

mujoco = pytest.importorskip("mujoco")

from james_gestures import ArmConfig, ArmController, PRESETS, default_joints  # noqa: E402
from james_gestures import handpose as HP  # noqa: E402
from james_gestures.gestures import GestureReader  # noqa: E402
from james_gestures.sim import HOME, MODEL, SimLink  # noqa: E402

W, H = 640, 480
TOL_DEG = 1.0


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def sim_deg(link, name):
    return math.degrees(link.data.joint(name).qpos[0]) + HOME[name]


@pytest.fixture
def rig():
    link = SimLink(MODEL, show=False)              # headless: no viewer window
    clock = Clock()
    arm = ArmController(link, cfg=ArmConfig(mirror=False), clock=clock)
    reader = GestureReader(clock=clock)
    try:
        yield link, arm, reader, clock
    finally:
        link.close()


def settle(arm, clock, seconds, dt=1 / 60):
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        arm.tick()


def arm_with_palm(arm, reader, clock, seconds=1.5, dt=1 / 30):
    for _ in range(int(round(seconds / dt))):
        clock.t += dt
        arm.update(reader.read(HP.synth("open", (320, 400), 120), W, H, t=clock.t))


def test_model_loads_headless(rig):
    link, arm, reader, clock = rig
    assert link.viewer is None
    for name in HOME:
        assert link.model.joint(name).id >= 0
    assert abs(sim_deg(link, "base") - 135) < 1e-9


def test_disarmed_sim_ignores_joint_commands(rig):
    link, arm, reader, clock = rig
    link.send("J base 200")
    assert abs(sim_deg(link, "base") - 135) < 1e-9       # not armed: ignored by the sim too


@pytest.mark.parametrize("preset", list(PRESETS))
def test_preset_reaches_target_in_sim(rig, preset):
    link, arm, reader, clock = rig
    arm_with_palm(arm, reader, clock)
    assert arm.armed and link.armed
    # start from an off-home pose so every preset actually has to move something
    arm.goto_preset("raise")
    settle(arm, clock, 4.0)
    assert arm.goto_preset(preset)
    settle(arm, clock, 4.0)                              # slowest joint is 45 deg/s; 4 s covers 180 deg
    joints = default_joints()
    for name in HOME:
        want = joints[name].clamp(PRESETS[preset].get(name, arm.pos[name]))
        got = sim_deg(link, name)
        assert abs(got - want) <= TOL_DEG, (preset, name, got, want)
        assert joints[name].lo <= got <= joints[name].hi


def test_stop_freezes_sim_and_blocks_further_motion(rig):
    link, arm, reader, clock = rig
    arm_with_palm(arm, reader, clock)
    arm.goto_preset("raise")
    settle(arm, clock, 0.5)                              # part way there
    mid = {n: sim_deg(link, n) for n in HOME}
    arm.stop()
    assert not link.armed and not arm.armed
    settle(arm, clock, 2.0)
    for n in HOME:
        assert abs(sim_deg(link, n) - mid[n]) < 1e-9


def test_claw_follows_commands_only_when_armed(rig):
    link, arm, reader, clock = rig
    open_gap = link.data.joint("claw_r").qpos[0]
    link.send("CLAW 1")                                  # disarmed: ignored
    assert link.data.joint("claw_r").qpos[0] == open_gap
    arm_with_palm(arm, reader, clock)
    assert arm.set_claw(1)
    assert link.data.joint("claw_r").qpos[0] > open_gap     # finger_r slides +y toward the centre = closed
    arm.set_claw(0)
    assert link.data.joint("claw_r").qpos[0] == open_gap
