Helper wrapper around MIT EVT's Can_Validator library for use in MY17 control system.

Compatible with both AtMega boards and ARM boards.

Make sure to run `git submodule init` and `git submodule update` after cloning.

Before using this library, run the `main.py` file in `can_validator` (with Python 3). This file
generates all of the necessary source files that are listed in `.gitignore`.
These files are dependent on the CAN spec. If you want to change one of these files, don't edit them
directly, but instead figure out what needs to change in the CAN spec. (If that
still doesn't work, edit the generator scripts in `can_validator` rather than
the generated files.) Source files that aren't ignored aren't generated from the
CAN spec, so they can be edited directly.

*Note:* The CAN messages to and from the motor controller are structured
differently than other CAN messages used in MY17, so they are not completely
described in the CAN spec. For more information about motor controller CAN
messages, see `mc.h`, `can_validator\templates\special_cases.txt`, or the motor
controller manual.
