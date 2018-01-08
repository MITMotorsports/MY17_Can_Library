Helper wrapper around MIT EVT's Can_Validator library for use in MY17 control system.

Compatible with both AtMega boards and ARM boards.

# Installation
If you're cloning this for the first time, use `git clone --recursive` to pull all submodules.
However, if you're switching from another branch/already have this repository cloned non-recursively,
make sure to run `git submodule init` and `git submodule update`.

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

# Use
You'll need to include `MY17_Can_Library.h` and define your architecture (define either `CAN_ARCHITECTURE_ARM` or `CAN_ARCHITECTURE_AVR`). Make sure to call `CanInit` with the appropriate baudrate, and then you can then call `Can_MsgType` to receive the latest CAN message. From there you can check what type of CAN message it is and handle it accordingly. See other MY17 projects for examples.
