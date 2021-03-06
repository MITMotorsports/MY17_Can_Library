"""
Generate header files that set up structs (bms.h, can_node.h, etc.).
Run this file to write just these files or main.py to write all files.
"""
import sys
sys.path.append("ParseCAN")
import ParseCAN
from common import struct_paths, spec_path, unused_segments

expected_keys = ["bms", "cannode", "currentsensor", "dash", "vcu"]


def write(output_paths, spec_path, unused_segments):
    """
    Write the header files for the main structs in the library.

    :param output_paths: a dictionary with a path for the BMS, CAN node, current sensor, dash, and VCU header files
    :param spec_path: path to the CAN spec
    """
    spec = ParseCAN.spec.can(spec_path)
    try:
        for key in expected_keys:
            with open(output_paths[key], 'w') as f:
                f.write("#ifndef _MY17_CAN_LIBRARY_" + key.upper() + "_H\n")
                f.write("#define _MY17_CAN_LIBRARY_" + key.upper() + "_H\n\n")
                f.write('#include "can_validator/fsae_can_spec.h"\n\n')
                f.write("#include <stdint.h>\n")
                f.write("#include <stdbool.h>\n\n")
                f.write('#include "ids.h"\n\n')

                # Hardcode special cases
                if key == 'vcu':
                    f.write("""typedef struct {
  Can_MC_RegID_T requestType;
  uint8_t period;
} Can_Vcu_MCRequest_T;

typedef struct {
  int16_t torque_cmd;
} Can_Vcu_MCTorque_T;
""")

                for message in spec.messages.values():
                    if (message.name.lower().startswith(key) or message.name.lower().startswith('rear' + key) or
                            message.name.lower().startswith('front' + key)):
                        if message.name.upper() == "VCU_MC_MESSAGE":  # MC special case
                            continue
                        f.write("typedef struct {\n")
                        for segment_name, segment in message.segments.items():
                            if segment.c_type != "enum":
                                field_name = segment_name
                                if field_name in unused_segments:
                                    continue
                                f.write("  " + segment.c_type + " " + field_name + ";\n")
                            else:
                                enum_name = "Can_" + message.name

                                # Fix name mismatch (for backwards compatibility)
                                enum_name = enum_name.replace('Heartbeat', 'State')
                                enum_name += "ID_T"
                                if enum_name == "Can_Vcu_DashStateID_T":
                                    enum_name = "Can_Vcu_LimpState_T"

                                f.write("  " + enum_name + " " + segment_name + ";\n")
                        f.write("} Can_" + message.name + "_T;\n\n")
                f.write("#endif // _MY17_CAN_LIBRARY_" + key.upper() + "_H")
    except KeyError as e:
        print("No path passed for " + e.args[0] + " header file, please add it to output_paths dictionary")
        raise


if __name__ == "__main__":
    write(struct_paths, spec_path, unused_segments)
