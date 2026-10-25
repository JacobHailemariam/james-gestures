# CAD files

## claw_print_plate.stl
The claw's parts laid out flat on one print plate, exported from SolidWorks (Victor E's design).
Units: millimetres. Plate is about 522 x 225 mm, every part 30 mm tall, one watertight mesh.
Contains: two fingers with gear teeth, two gears, the claw base, pins and spacers.

This is for 3D printing, not for the simulation: the parts are not in their assembled positions.
For the sim, use assembly.step below.

## assembly.step
The full arm assembly exported from SolidWorks 2026 by Victor E (Oct 25, 2026), STEP AP214.
Units: CENTIMETRES (not mm). Every part is in its assembled position.
Arm: BaseNPlate, BaseLid, NEMARef, Link1-4, Hinge60, Hinge25, 60ServoRef, 25Servo.
Claw (gear_claw_v3_1 sub-assembly): housing, jaw_A/B, pad_A/B, pinion_A/B, sleeve_A/B, top_bridge,
M2/M3 screws, and two servos (servo_A, servo_B).
Note: the claw CAD has two servos; the firmware currently drives one claw servo (PCA9685 channel 15).
Use this file (not claw_print_plate.stl) to place parts in sim/arm.xml.
