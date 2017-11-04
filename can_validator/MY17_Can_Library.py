import sys
sys.path.append("CAN_Api/src")
from CANSpec import CANSpec
from math import ceil

spec_path = "CAN_Api/fsae_can_spec.yml"
target_path = "../MY17_Can_Library.c"
base_path = "../MY17_Can_Library_BASE.c"
special_path = "special_can_handlers.txt"

spec = CANSpec(spec_path)

field_name_mappings = {
    "requested_torque" : "torque",
    "is_ok" : "is_alive",
    "wheel_speed" : "wheel_speed_mRPM",
    "right_throttle_pot" : "accel_1_raw",
    "left_throttle_pot" : "accel_2_raw",
    "front_brake_pressure" : "brake_1_raw",
    "rear_brake_pressure" : "brake_2_raw",
    "ave" : "avg",
    "min_cell_temp_id" : "id_min_cell_temp",
    "max_cell_temp_id" : "id_max_cell_temp",
    "min_cell_voltage_id" : "id_min_cell_voltage",
    "max_cell_voltage_id" : "id_max_cell_voltage",
    "error_type" : "type",
    "heartbeat_on" :  "ok",
    "request_type" : "type",
    "steering_pot" : "steering_raw",
    "soc_percentage" : "soc"
}

msg_enum_name_mappings = {
    "Front_Can_Node" : "FrontCanNode",
    "Rear_Can_Node" : "RearCanNode",
    "Raw_Values" : "RawValues",
    "Wheel_Speed" : "WheelSpeed",
    "Vcu_Bms_Heartbeat" : "Vcu_BmsHeartbeat",
    "Vcu_Dash_Heartbeat" : "Vcu_DashHeartbeat",
    "Cell_Temps" : "CellTemps",
    "Pack_Status" : "PackStatus",
    "Current_Sensor" : "CurrentSensor",
    "Driver_Output" : "DriverOutput",
    "Bms_Errors" : "Bms_Error"
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
    field_name = segment_name.lower()
    for old, new in field_name_mappings.items():
        field_name = field_name.replace(old, new)
    return field_name

def get_msg_enum_name(message_name):
    msg_enum_name = message_name.title()
    for old, new in msg_enum_name_mappings.items():
        msg_enum_name = msg_enum_name.replace(old, new)
    return "Can_" + msg_enum_name

with open(target_path, 'w') as f:
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

    # Write CAN TO_CAN and FROM_CAN
    for message in spec.messages.values():
        if message.name in unused_messages:
            continue
        f.write(
            "TO_CAN(" + get_msg_enum_name(message.name) + "){\n" +
            "  uint64_t bitstring = 0;\n")

        length = 0
        for segment_name, segment in message.segments.items():
            field_name = get_field_name(segment_name)

            print(message.name)
            print("CURRENT_SENSOR" in message.name)

            if message.name == "VCU_BMS_HEARTBEAT" and field_name == "state":
                field_name = "alwaysTrue"
            if "CURRENT_SENSOR" in message.name:
                field_name = field_name.replace("pack_current", "current_mA")
                field_name = field_name.replace("pack_voltage", "voltage_mV")
                field_name = field_name.replace("pack_power", "power_W")
            print(field_name)

            if field_name == "unused" or field_name == "reserved":
                continue
            f.write(
                "  bitstring = INSERT(type_in->" + field_name + ", bitstring, " + str(segment.position[0]) + ", " +
                str(segment.position[1]-segment.position[0]) + ");\n")
            length += segment.position[1]-segment.position[0]
        f.write(
            "  from_bitstring(&bitstring, can_out->data);\n" +
            "  can_out->id = " + message.name + "__id;\n" +
            "  can_out->len = " + str(ceil(length / 8)) + ";\n" +
            "}\n\n")

        f.write(
            "FROM_CAN(" + get_msg_enum_name(message.name) + "){\n" +
            "  uint64_t bitstring = 0;\n" +
            "  to_bitstring(can_in->data, &bitstring);\n")
        for segment_name, segment in message.segments.items():
            field_name = get_field_name(segment_name)
            if (message.name == "VCU_BMS_HEARTBEAT" and field_name == "state"):
                field_name = "alwaysTrue"
            if (field_name == "unused" or field_name == "reserved"):
                continue
            f.write(
                "  type_out->" + field_name + " = EXTRACT(bitstring, " + str(segment.position[0]) + ", " +
                str(segment.position[1]-segment.position[0]) + ");\n")
        f.write("}\n\n")


    # Write special CAN statements
    with open(special_path) as special:
        lines = special.readlines()
        f.writelines(lines)

    # Write DEFINE statements
    for message in spec.messages.values():
        if message.name in unused_messages:
            continue
        f.write("DEFINE(" + get_msg_enum_name(message.name) + ")\n")
