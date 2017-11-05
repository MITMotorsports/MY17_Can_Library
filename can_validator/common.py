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
