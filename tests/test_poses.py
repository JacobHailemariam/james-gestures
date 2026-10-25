"""Named poses by voice, and saving the current pose under a name."""
import json

from james_gestures import ArmConfig, ArmController, PrintLink
from james_gestures.arm import PRESETS, load_poses
from james_gestures.voice import parse


def test_parse_two_word_poses_and_save():
    assert parse("left shoulder") == ["left shoulder"]
    assert parse("james right hip") == ["right hip"]
    assert parse("save left hip") == ["save:left hip"]
    assert parse("save home") == ["save:home"]
    assert parse("save") == []
    assert parse("shoulder") == []                       # half a pose name is not a command


def test_voice_pose_arms_and_goes(tmp_path):
    link = PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(poses_file=str(tmp_path / "p.json")))
    assert arm.voice("left shoulder") and arm.armed
    assert arm.target["base"] == PRESETS["left shoulder"]["base"]
    assert arm.target["shoulder"] == PRESETS["left shoulder"]["shoulder"]


def test_save_then_load_roundtrip(tmp_path):
    f = tmp_path / "poses.json"
    link = PrintLink(quiet=True)
    arm = ArmController(link, cfg=ArmConfig(poses_file=str(f)))
    arm.pos.update({"base": 200.0, "shoulder": 123.0, "elbow": 77.0, "wrist": 45.0})
    assert arm.voice("save:right hip")
    on_disk = json.loads(f.read_text())
    assert on_disk["right hip"] == {"base": 200.0, "shoulder": 123.0, "elbow": 77.0, "wrist": 45.0}
    PRESETS["right hip"] = {"base": 1, "shoulder": 1, "elbow": 1, "wrist": 1}   # forget it ...
    load_poses(f)                                                               # ... and reload from the file
    assert PRESETS["right hip"]["base"] == 200.0
    arm2 = ArmController(PrintLink(quiet=True), cfg=ArmConfig(poses_file=str(f)))
    assert arm2.voice("right hip") and arm2.target["shoulder"] == 123.0
