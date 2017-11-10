# Maps parts of segment names to their corresponding parts of names in structs
field_name_mappings = {
    "wheel_speed": "wheel_speed_mRPM",
    "pack_energy": "energy_Wh",
    "always_true": "alwaysTrue"
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
