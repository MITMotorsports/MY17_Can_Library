"""
Generate MY17_Can_Libary.h file.
Run this file to write just MY17_Can_Libary.h or main.py to write all files.
"""
import sys
sys.path.append("ParseCAN")
import ParseCAN
from common import can_lib_h_path, spec_path, struct_paths, unused_messages


def write(output_path, spec_path, struct_paths, unused_messages):
    """
    Generate MY17_Can_Libary.h file.

    :param output_path: file to be written to
    :param spec_path: CAN spec path
    :param struct_paths: list of paths to struct header files (the ones generated by structs.py)
    :param unused_messages: list of messages that are in the CAN spec but unused
    """
    spec = ParseCAN.spec.can(spec_path)
    with open(output_path, 'w') as f:
        # Setup file
        f.write("""#ifndef _MY17_CAN_LIBRARY_H
#define _MY17_CAN_LIBRARY_H

#include <stdint.h>
#include <stdbool.h>

""")

        # Create enum
        f.write("""typedef enum {
  Can_No_Msg,
  Can_Unknown_Msg,
  Can_Error_Msg,
""")
        for message in spec.messages.values():
            f.write("  Can_" + message.name + "_Msg,\n")

        # Add motor controller special cases
        f.write("""  Can_MC_ErrorAndWarning_Msg,
  Can_MC_DataReading_Msg,
  Can_MC_State_Msg,
  Can_Vcu_MCRequest_Msg,
  Can_Vcu_MCTorque_Msg,
} Can_MsgID_T;
""")

        # Finish initial setup
        f.write("""
Can_MsgID_T Can_MsgType(void);

#include "can_raw.h"

#define TO_CAN(name) \\
  void name ## _ToCan(name ## _T *type_in, Frame *can_out)

#define FROM_CAN(name) \\
  void name ## _FromCan(Frame *can_in, name ## _T *type_out)

#define DECLARE(name) \\
  Can_ErrorID_T name ##_Read(name ## _T *type); \\
  Can_ErrorID_T name ##_Write(name ## _T *type); \\
  TO_CAN(name); \\
  FROM_CAN(name);

Can_ErrorID_T Can_Unknown_Read(Frame *frame);
Can_ErrorID_T Can_Error_Read(void);

void to_bitstring(uint8_t in[], uint64_t *out);
void from_bitstring(uint64_t *in, uint8_t out[]);

""")

        # Add structs includes
        for path in struct_paths:
            f.write('#include "' + path.replace("..", "").replace("/", "") + '"\n')
        f.write('#include "mc.h"\n\n')

        # Write DECLARE statements
        for message in spec.messages.values():
            if message.name in unused_messages:
                continue
            f.write("DECLARE(Can_" + message.name + ")\n")

        # Special cases (motor controller messages)
        f.write("""DECLARE(Can_Vcu_MCRequest)
DECLARE(Can_Vcu_MCTorque)
DECLARE(Can_MC_DataReading)
DECLARE(Can_MC_ErrorAndWarning)
DECLARE(Can_MC_State)

""")

        # Finish up
        f.write(
            "#undef DECLARE\n" +
            "#endif // _MY17_CAN_LIBRARY_H")


if __name__ == "__main__":
    write(can_lib_h_path, spec_path, struct_paths.values(), unused_messages)
