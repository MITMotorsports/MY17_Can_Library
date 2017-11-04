import sys
sys.path.append("CAN_Api/src")
from CANSpec import CANSpec
import common

spec_path = "CAN_Api/fsae_can_spec.yml"
output_path = "../ids.h"

spec = CANSpec(spec_path)

with open(output_path, 'w') as f:
    f.write('_MY17_CAN_LIBRARY_IDS_H\n')
    f.write('_MY17_CAN_LIBRARY_IDS_H\n\n')
    f.write('#include "can_validator/fsae_can_spec.h"\n\n')

    for message in spec.messages.values():
        for segment_name, segment in message.segments.items():
            if segment.c_type == 'enum':
                f.write('typedef enum {\n')
                for value_name, value in segment.values.items():
                    message_name = common.get_msg_enum_name(message.name).upper()

                    # Fix name mismatch
                    message_name = message_name.replace('HEARTBEAT', 'STATE')
                    f.write(
                        "  " + message_name + "_" + value_name.upper() + " = " +
                          '____' + message.name + '__' + segment_name + '__' + value_name + ",\n")
                f.write("} " + common.get_msg_enum_name(message.name).replace('Heartbeat', 'State') + "ID_T;\n\n")

    f.write('#endif // _MY17_CAN_LIBRARY_IDS_H')
