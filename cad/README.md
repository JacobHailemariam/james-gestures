# CAD files

## claw_print_plate.stl
The claw's parts laid out flat on one print plate, exported from SolidWorks (Victor E's design).
Units: millimetres. Plate is about 522 x 225 mm, every part 30 mm tall, one watertight mesh.
Contains: two fingers with gear teeth, two gears, the claw base, pins and spacers.

This is for 3D printing, not for the simulation: the parts are not in their assembled positions.
For the MuJoCo sim we would need the claw exported as an assembly (STEP, or one STL per part
in assembled position) from SolidWorks.

The full arm assembly (RoboticArm_3.SLDASM) is not stored here: it only opens in SolidWorks and
needs all its part files. Export it as STEP to share it.
