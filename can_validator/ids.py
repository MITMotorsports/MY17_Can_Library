import sys
sys.path.append("ParseCAN")
import ParseCAN
import common

# The VCU_MC_MESSAGE and MC_RESPONSE messages need to be combined into one enum, which will be stored here
can_mc_reg_lines = []

def write(output_path, spec_path):
    """
    Generate ids.h file, which includes enums for segments of type enum.

    :param output_path: file to be written to
    :param spec_path: CAN spec path
    """
    spec = ParseCAN.spec.can(spec_path)
    with open(output_path, 'w') as f:
        f.write('#ifndef _MY17_CAN_LIBRARY_IDS_H\n')
        f.write('#define _MY17_CAN_LIBRARY_IDS_H\n\n')
        f.write('#include "can_validator/fsae_can_spec.h"\n\n')

        for message in spec.messages.values():
            for segment_name, segment in message.segments.items():
                if segment.c_type == 'enum':
                    if not (message.name == "VCU_MC_MESSAGE" or message.name == "MC_RESPONSE"):
                        f.write('typedef enum {\n')
                        for value_name, value in segment.values.items():
                            message_name = common.get_msg_enum_name(message.name).upper()

                            # Fix name mismatch
                            message_name = message_name.replace('HEARTBEAT', 'STATE')
                            f.write(
                                "  " + message_name + "_" + value_name.upper() + " = " +
                                  '____' + message.name + '__' + segment_name + '__' + value_name + ",\n")
                        f.write("} " + common.get_msg_enum_name(message.name).replace('Heartbeat', 'State') + "ID_T;\n\n")
                    else:
                        for value_name, value in segment.values.items():
                            if value_name.upper() == "TORQUE_CMD" and message.name == "VCU_MC_MESSAGE":
                                # Both of the message have the same value name, but we only want the one from VCU_MC_MESSAGE
                                continue
                            val_name = value_name.upper()

                            # Fix name mismatch
                            val_name = value_name.replace("ACTUAL_FILTERED", "AFTER_DISPLAY")
                            can_mc_reg_lines.append("  CAN_MC_REG_" + val_name + " = " + '____' + message.name + '__' +
                                segment_name + '__' + value_name + ",\n")
        f.write('typedef enum {\n')
        for line in can_mc_reg_lines:
            f.write(line)
        f.write("} Can_MC_RegID_T;\n\n")


        f.write('#endif // _MY17_CAN_LIBRARY_IDS_H')

if __name__ == "__main__":
    write("../ids.h", "ParseCAN/fsae_can_spec.yml")
