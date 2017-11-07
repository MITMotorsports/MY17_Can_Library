import sys
sys.path.append("ParseCAN")
import ParseCAN
from math import ceil
import common

unused_messages = [
    "LV_BATTERY_VOLTAGE",
    "ACCELEROMETER_HORIZONTAL",
    "ACCELEROMETER_VERTICAL",
    "GYRO_VERTICAL",
    "GYRO_HORIZONTAL",
    "MAGNETOMETER_HORIZONTAL",
    "MAGNETOMETER_VERTICAL",
    "MC_RESPONSE",
    "VCU_MC_MESSAGE"
]


def write(output_path, spec_path, base_path, special_cases_path):
    """
    Generate MY17_Can_Libary.c file.

    :param output_path: file to be written to
    :param spec_path: CAN spec path
    :param base_path: File with template code that's not autogenerated
    :param special_cases_path: File with code for special cases of functions that aren't based off of CAN spec
    """
    spec = ParseCAN.spec.can(spec_path)
    with open(output_path, 'w') as f:
        # Copy over base
        with open(base_path) as base:
            lines = base.readlines()
            f.writelines(lines)

        # Write switch statement
        f.write(
            "Can_MsgID_T Can_MsgType(void) {\n" +
            "  lastError = Can_RawRead(&lastMessage);\n" +
            "  if (lastError == Can_Error_NO_RX) {\n" +
            "    return Can_No_Msg;\n" +
            "  } else if (lastError != Can_Error_NONE) {\n" +
            "    return Can_Error_Msg;\n" +
            "  }\n\n" +
            "  uint16_t id = lastMessage.id;\n" +
            "  uint16_t first_byte = lastMessage.data[0];\n\n" +
            "  switch(id) {\n")
        for message in spec.messages.values():
            if message.name in unused_messages:
                continue
            f.write(
                "    case " + message.name + "__id:\n" +
                "      return " + common.get_msg_enum_name(message.name) + "_Msg;\n")
        # Special cases
        f.write(
            "    case MC_RESPONSE__id:\n" +
            "      if (first_byte == CAN_MC_REG_ERRORS_AND_WARNINGS) {\n" +
            "        return Can_MC_ErrorAndWarning_Msg;\n" +
            "      } else if (first_byte == CAN_MC_REG_STATE) {\n" +
            "        return Can_MC_State_Msg;\n" +
            "      } else {\n" +
            "        return Can_MC_DataReading_Msg;\n" +
            "      }\n")
        f.write(
            "    case VCU_MC_MESSAGE__id:\n" +
            "      if (first_byte == CAN_MC_REG_MSG_REQUEST || first_byte == CAN_MC_REG_MSG_EVENT_REQUEST) {\n" +
            "        return Can_Vcu_MCRequest_Msg;\n" +
            "      } else if (first_byte == CAN_MC_REG_TORQUE_CMD) {\n" +
            "        return Can_Vcu_MCTorque_Msg;\n" +
            "      } else {\n" +
            "        return Can_Unknown_Msg;\n" +
            "      }\n")

        f.write(
            "    default:\n" +
            "      return Can_Unknown_Msg;\n" +
            "   }\n" +
            "}\n\n")

        for message in spec.messages.values():
            if message.name in unused_messages:
                continue

            # Write TO_CAN
            f.write(
                "TO_CAN(" + common.get_msg_enum_name(message.name) + "){\n" +
                "  uint64_t bitstring = 0;\n")

            length = 0
            for segment_name, segment in message.segments.items():
                field_name = common.get_field_name(segment_name)

                # Some segment names appear in multiple messages, but only need to be changed in some of these messages
                if message.name == "VCU_BMS_HEARTBEAT" and field_name == "state":
                    field_name = "alwaysTrue"
                if "CURRENT_SENSOR" in message.name:
                    field_name = field_name.replace("pack_current", "current_mA")
                    field_name = field_name.replace("pack_voltage", "voltage_mV")
                    field_name = field_name.replace("pack_power", "power_W")

                if field_name == "unused" or field_name == "reserved":
                    continue

                if message.is_big_endian:
                    f.write(
                        "  bitstring = INSERT(type_in->" + field_name + ", bitstring, " + str(segment.position[0]) + ", " +
                        str(segment.position[1]-segment.position[0]+1) + ");\n")
                else:
                    f.write(
                        "  " + segment.c_type + " " + field_name + "_swap_value = swap_" + segment.c_type[:-2] +
                        "(type_in->" + field_name + ");\n" +
                        "  bitstring = INSERT(" + field_name + "_swap_value, bitstring, " + str(segment.position[0]) + ", " +
                        str(segment.position[1]-segment.position[0]+1) + ");\n\n")

                length += segment.position[1]-segment.position[0]+1
            f.write(
                "  from_bitstring(&bitstring, can_out->data);\n" +
                "  can_out->id = " + message.name + "__id;\n" +
                "  can_out->len = " + str(ceil(length / 8)) + ";\n" +
                "}\n\n")

            # Write FROM_CAN
            f.write(
                "FROM_CAN(" + common.get_msg_enum_name(message.name) + "){\n" +
                "  uint64_t bitstring = 0;\n" +
                "  to_bitstring(can_in->data, &bitstring);\n")
            for segment_name, segment in message.segments.items():
                field_name = common.get_field_name(segment_name)
                if message.name == "VCU_BMS_HEARTBEAT" and field_name == "state":
                    field_name = "alwaysTrue"
                if "CURRENT_SENSOR" in message.name:
                    field_name = field_name.replace("pack_current", "current_mA")
                    field_name = field_name.replace("pack_voltage", "voltage_mV")
                    field_name = field_name.replace("pack_power", "power_W")
                if field_name == "unused" or field_name == "reserved":
                    continue
                if message.is_big_endian:
                    if segment.c_type.startswith("int"):  # Check if signed int
                        f.write(
                            "  type_out->" + field_name + " = SIGN(EXTRACT(bitstring, " + str(segment.position[0]) + ", " +
                            str(segment.position[1]-segment.position[0]+1) + "), " + str(segment.position[1]-
                                                                                         segment.position[0]+1)+ ");\n")
                    else:
                        f.write(
                            "  type_out->" + field_name + " = EXTRACT(bitstring, " + str(segment.position[0]) + ", " +
                            str(segment.position[1]-segment.position[0]+1) + ");\n")
                else:
                    if segment.c_type.startswith("int"):  # Check if signed int
                        # TODO Check if we can use the signed swap directly -- what's here is based off old CAN_Library code
                        f.write(
                            "  " + segment.c_type + " " + field_name + "_swap_value=(" + segment.c_type + ")(swap_u" +
                            segment.c_type[:-2] + "(EXTRACT(bitstring, " + str(segment.position[0]) + ", " +
                            str(segment.position[1]-segment.position[0]+1) + ")));\n")
                    else:
                        f.write(
                            "  " + segment.c_type + " " + field_name + "_swap_value=swap_u" +
                            segment.c_type[:-2] + "(EXTRACT(bitstring, " + str(segment.position[0]) + ", " +
                            str(segment.position[1]-segment.position[0]+1) + ")));\n")
                    f.write("  type_out->" + field_name + " = " + field_name + "_swap_value;\n")
            f.write("}\n\n")

        # Write TO_CAN and FROM_CAN functions that are not described by the spec
        with open(special_cases_path) as special:
            lines = special.readlines()
            f.writelines(lines)

        # Write DEFINE statements
        for message in spec.messages.values():
            if message.name in unused_messages:
                continue
            f.write("DEFINE(" + common.get_msg_enum_name(message.name) + ")\n")

if __name__ == "__main__":
    write("../MY17_Can_Library.c", "ParseCAN/fsae_can_spec.yml", "../MY17_Can_Library_BASE.txt", "special_cases.txt")
