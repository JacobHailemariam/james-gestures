"""james_gestures: the JAMES hand-gesture stack, packaged to drive a robot arm.

    from james_gestures import GestureReader, ArmController, PrintLink
"""
from . import handpose, motion
from .arm import PRESETS, ArmConfig, ArmController, Joint, PrintLink, SerialLink, default_joints
from .gestures import GestureConfig, GestureFrame, GestureReader

__all__ = ["handpose", "motion", "GestureConfig", "GestureFrame", "GestureReader", "ArmConfig", "ArmController",
           "Joint", "PRESETS", "PrintLink", "SerialLink", "default_joints"]
__version__ = "0.2.0"
