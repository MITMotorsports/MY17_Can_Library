import sys
sys.path.append("CAN_Api/src")
from CANSpec import CANSpec
import re

spec_path = "CAN_Api/fsae_can_spec.yml"
header_path = "fsae_can_spec.h"

spec = CANSpec(spec_path)

with open(header_path, 'w') as f:
    clean_header_path = re.sub('[^A-Za-z0-9_]+', '_', header_path).upper()
    f.write('#ifndef ____' + clean_header_path + '\n')
    f.write('#define ____' + clean_header_path + '\n\n')
    for id, message in spec.messages.items():
        f.write('#define ' + message.name + '__id ' + str(id) + '\n')
        if message.frequency:
            f.write('#define ' + message.name + '__freq ' + str(int(message.frequency)) + '\n')
        else:
            f.write('#define ' + message.name + '__freq ' + str(-1) + '\n')
        for segment_name, segment in message.segments.items():
            f.write('#define __' + message.name + '__' + segment_name + '__start ' + str(segment.position[0]) + '\n')
            f.write('#define __' + message.name + '__' + segment_name + '__end ' + str(segment.position[1]) + '\n')
            for value_name, value in segment.values.items():
                if value.range[0] == value.range[1]:
                    f.write('#define ____' + message.name + '__' + segment_name + '__' + value_name + ' ' + str(value.range[0]) + '\n')
                else:
                    f.write('#define ____' + message.name + '__' + segment_name + '__' + value_name + '__FROM ' + str(value.range[0]) + '\n')
                    f.write('#define ____' + message.name + '__' + segment_name + '__' + value_name + '__TO ' + str(value.range[1]) + '\n')

        f.write('\n')
    f.write('#endif // ____' + clean_header_path + '\n')
