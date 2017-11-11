import fsae_can_spec
import MY17_Can_Library
import ids
import structs

spec_path = "ParseCAN/fsae_can_spec.yml"
header_spec_path =

unused_messages = [
    "Lv_Battery_Voltage",
    "Accelerometer_Horizontal",
    "Accelerometer_Vertical",
    "Gyro_Vertical",
    "Gyro_Horizontal",
    "Magnetometer_Horizontal",
    "Magnetometer_Vertical",
    "Mc_Response",
    "Vcu_Mc_Message"
]

structs_paths = {
    "bms": "../bms.h",
    "cannode": "../can_node.h",
    "currentsensor": "../current_sensor.h",
    "dash": "../dash.h",
    "vcu": "../vcu.h"
}

fsae_can_spec.write("fsae_can_spec.h", spec_path)
MY17_Can_Library.write("../MY17_Can_Library.c", spec_path, "../MY17_Can_Library_BASE.txt", "special_cases.txt")
ids.write("../ids.h", spec_path)
structs.write(structs_paths, "ParseCAN/fsae_can_spec.yml")
