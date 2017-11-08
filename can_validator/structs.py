import sys
sys.path.append("ParseCAN")
import ParseCAN
from math import ceil
import common

expected_keys = ["bms", "can_node", "current_sensor", "dash", "vcu"]
def write(output_paths, spec_path):
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
                    f.write(
                        "typedef enum {\n" +
                        "  CAN_LIMP_NORMAL = 0,\n" +
                        "  CAN_LIMP_50,\n" +
                        "  CAN_LIMP_33,\n" +
                        "  CAN_LIMP_25\n" +
                        "} Can_Vcu_LimpState_T;\n\n" +
                        "typedef struct {\n" +
                        "  Can_MC_RegID_T requestType;\n" +
                        "  uint8_t period;\n" +
                        "} Can_Vcu_MCRequest_T;\n\n" +
                        "typedef struct {\n" +
                        "  int16_t torque_cmd;\n" +
                        "} Can_Vcu_MCTorque_T;\n"
                    )

                for message in spec.messages.values():
                    if message.name.lower().startswith(key) or message.name.lower().startswith('rear_' + key) or message.name.lower().startswith('front_' + key):
                        if message.name == "VCU_MC_MESSAGE":
                            continue
                        f.write("typedef struct {\n")
                        # Hardcode special case
                        if message.name == "VCU_DASH_HEARTBEAT":
                            f.write("  Can_Vcu_LimpState_T limp_state;\n")
                        for segment_name, segment in message.segments.items():
                            if segment.c_type != "enum":
                                field_name = common.get_field_name(segment_name)
                                if field_name == 'reserved' or field_name == 'unused':
                                    continue
                                # Fix name mismatch
                                if "CURRENT_SENSOR" in message.name:
                                    field_name = field_name.replace("pack_current", "current_mA")
                                    field_name = field_name.replace("pack_voltage", "voltage_mV")
                                    field_name = field_name.replace("pack_power", "power_W")
                                f.write("  " + segment.c_type + " " + field_name + ";\n")
                            else:
                                enum_name = common.get_msg_enum_name(message.name)
                                # Fix name mismatch
                                enum_name = enum_name.replace('Heartbeat', 'State')
                                enum_name += "ID_T"
                                f.write("  " + enum_name + " " + common.get_field_name(segment_name) + ";\n")
                        f.write("} " + common.get_msg_enum_name(message.name) + "_T;\n\n")
                f.write("#endif // _MY17_CAN_LIBRARY_" + key.upper() + "_H")
    except KeyError as e:
        print("No path passed for " + e.args[0] + " header file, please add it to output_paths dictionary")
        raise


if __name__ == "__main__":
    paths = {
        "bms": "../bms.h",
        "can_node": "../can_node.h",
        "current_sensor": "../current_sensor.h",
        "dash": "../dash.h",
        "vcu": "../vcu.h"
    }
    write(paths, "ParseCAN/fsae_can_spec.yml")
