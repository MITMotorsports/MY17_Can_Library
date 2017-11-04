import sys
sys.path.append("CAN_Api/src")
from CANSpec import CANSpec
from math import ceil

spec_path = "CAN_Api/fsae_can_spec.yml"
output_path = "../MY17_Can_Library.c"
base_path = "../MY17_Can_Library_BASE.txt"  # File with template code that's not autogenerated
special_cases = "special_cases.txt"  # Special cases of functions that aren't based off of CAN spec

spec = CANSpec(spec_path)

# Maps parts of segment names to their corresponding parts of names in structs
field_name_mappings = {
    "requested_torque": "torque",
    "is_ok": "is_alive",
    "wheel_speed": "wheel_speed_mRPM",
    "right_throttle_pot": "accel_1_raw",
    "left_throttle_pot": "accel_2_raw",
    "front_brake_pressure": "brake_1_raw",
    "rear_brake_pressure": "brake_2_raw",
    "ave": "avg",
    "min_cell_temp_id": "id_min_cell_temp",
    "max_cell_temp_id": "id_max_cell_temp",
    "min_cell_voltage_id": "id_min_cell_voltage",
    "max_cell_voltage_id": "id_max_cell_voltage",
    "error_type": "type",
    "heartbeat_on":  "ok",
    "request_type": "type",
    "steering_pot": "steering_raw",
    "soc_percentage": "soc",
    "pack_energy": "energy_Wh"
}

# Maps parts of message names to their corresponding parts of names in the Can_MsgID_T enum
msg_enum_name_mappings = {
    "Front_Can_Node": "FrontCanNode",
    "Rear_Can_Node": "RearCanNode",
    "Raw_Values": "RawValues",
    "Wheel_Speed": "WheelSpeed",
    "Vcu_Bms_Heartbeat": "Vcu_BmsHeartbeat",
    "Vcu_Dash_Heartbeat": "Vcu_DashHeartbeat",
    "Cell_Temps": "CellTemps",
    "Pack_Status": "PackStatus",
    "Current_Sensor": "CurrentSensor",
    "Driver_Output": "DriverOutput",
    "Bms_Errors": "Bms_Error"
}

unused_messages = [
    "LV_BATTERY_VOLTAGE",
    "ACCELEROMETER_HORIZONTAL",
    "ACCELEROMETER_VERTICAL",
    "GYRO_VERTICAL",
    "GYRO_HORIZONTAL",
    "MAGNETOMETER_HORIZONTAL",
    "MAGNETOMETER_VERTICAL",
    "VCU_MC_MESSAGE",
    "MC_RESPONSE"
]


def get_field_name(segment_name):
    """
    Find the name of the field in the message's struct for the particular segment.

    :param segment_name: name of the segment
    :return: name from struct
    """
    field_name = segment_name.lower()
    for old, new in field_name_mappings.items():
        field_name = field_name.replace(old, new)
    return field_name


def get_msg_enum_name(message_name):
    """
    Find the name of the message in the Can_MsgID_T enum for the particular CAN message
    
    :param message_name: message (from CAN spec)
    :return: name used in the Can_MsgID_T enum
    """
    msg_enum_name = message_name.title()
    for old, new in msg_enum_name_mappings.items():
        msg_enum_name = msg_enum_name.replace(old, new)
    return "Can_" + msg_enum_name


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
            "      return " + get_msg_enum_name(message.name) + "_Msg;\n")
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
            "TO_CAN(" + get_msg_enum_name(message.name) + "){\n" +
            "  uint64_t bitstring = 0;\n")

        length = 0
        for segment_name, segment in message.segments.items():
            field_name = get_field_name(segment_name)

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
            "FROM_CAN(" + get_msg_enum_name(message.name) + "){\n" +
            "  uint64_t bitstring = 0;\n" +
            "  to_bitstring(can_in->data, &bitstring);\n")
        for segment_name, segment in message.segments.items():
            field_name = get_field_name(segment_name)
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
    with open(special_cases) as special:
        lines = special.readlines()
        f.writelines(lines)

    # Write DEFINE statements
    for message in spec.messages.values():
        if message.name in unused_messages:
            continue
        f.write("DEFINE(" + get_msg_enum_name(message.name) + ")\n")
