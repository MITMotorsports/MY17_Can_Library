#!/usr/bin/env python

import argparse
import re
from datetime import datetime as dt
import datetime

def intb(intstr, reverse_endian=False):
    if intstr[0:2] == '0b':
        if reverse_endian:
            return int('0b' + bin(int(intstr,2))[2:][::-1],2)
        return int(intstr, 2)
    elif intstr[0:2] == '0x':
        if reverse_endian:
            return int('0b' + bin(int(intstr,16))[2:][::-1],2)
        return int(intstr, 16)
    else:
        if reverse_endian:
            return int('0b' + bin(int(intstr))[2:][::-1],2)
        return int(intstr)


def parse_frequency(freq_str):
    m = re.search('(\d\.*\d*)*([A-Za-z]*)', freq_str)
    unit = m.group(2)
    num = float(m.group(1))
    if unit.lower() == 'hz':
        return num
    elif unit == 'khz':
        return num*1000
    elif unit.lower() == 'mhz':
        return num*pow(10,6)
    else:
        print 'Unrecognized unit during frequency parse with',freq_str


class Message:
    ''' Contains fields of a CAN message and a message timestamp'''
    ''' NOTE: You cannot hash this because __eq__ is overwritten with non-unique function'''

    def __init__(self, timestamp, can_id_str, can_format_str, data_str):
        self.timestamp = timestamp
        if can_id_str == "Form Error" or can_id_str == "Stuff Error":
            self.error = True
            self.can_id = None
            self.can_format = None
            self.data = None
        else:
            self.error = False
            self.can_id = int('0x' + can_id_str, 16)
            self.can_format = can_format_str
            self.data = int('0x' + data_str, 16)

    def interpret(self, spec):
        pass

    def __str__(self):
        if self.error:
            return '[Message: ' + dt.strftime(self.timestamp, '%H:%M:%S.%f') + ' ERROR]'
        else:
            return '[Message: ' + dt.strftime(self.timestamp, '%H:%M:%S.%f') + \
                    ' ID:' + str(self.can_id) + ' Data:' + str(self.data) + ']'

    def __eq__(self, other):
        return self.error == other.error and self.can_id == other.can_id and self.data == other.data


class InterpretedMessageBlock:
    def __init__(self, start_timestamp, end_timestamp, error, id_symbol, data_symbols, num_messages):
        self.start_timestamp = start_timestamp
        self.end_timestamp = end_timestamp
        self.id_symbol = id_symbol
        self.data_symbols = data_symbols
        self.num_messages = num_messages
        self.error = error

    def __str__(self):
        if self.error is True:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' \n\t\tBUS READ ERROR\n\t]'
        elif self.error and not self.id_symbol:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' \n\t\t' + self.error + '\n\t]'
        elif self.error and self.id_symbol:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' \n\t\tID: ' + self.id_symbol + ' \n\t\t' + self.error + '\n\t]'
        else:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' \n\t\tID:' + self.id_symbol + ' \n\t\tData:' + str(self.data_symbols) + '\n\t]'



class MessageBlock:
    def __init__(self, start_timestamp, end_timestamp, error, can_id, data, num_messages):
        self.start_timestamp = start_timestamp
        self.end_timestamp = end_timestamp
        self.can_id = can_id
        self.data = data
        self.num_messages = num_messages
        self.error = error

    def symbolify(self, spec):
        if self.error: return InterpretedMessageBlock(self.start_timestamp, self.end_timestamp, True, None, None, self.num_messages)

        error = ''
        try:
            message_type = spec[self.can_id]
        except KeyError:
            error += 'CAN ID {0} NOT IN SPECS'.format(hex(self.can_id))
            return InterpretedMessageBlock(self.start_timestamp, self.end_timestamp, error, None, None,self.num_messages)

        data_str = bin(self.data)[2:]
        data_symbols = {}
        for segment in message_type.segments:
            data_part = int(data_str[segment.boundary[0]:segment.boundary[1]+1])
            if segment.is_float:
                data_symbols[segment.name + ' ' + segment.values[None]] = data_part
            else:
                no_match = True
                for value, symbol in segment.values.items():
                    if type(value) == type(1) and data_part == value:
                        data_symbols[segment.name] = value
                        no_match = False
                        break
                    elif type(value) == type((1,1)) and value[0] <= data_part and data_part <= value[1]:
                        data_symbols[segment.name] = value
                        no_match = False
                        break
                if no_match:
                    error += 'Data value {0} for {1} not adherent to spec.'.format(hex(data_part), segment.name)
        return InterpretedMessageBlock(self.start_timestamp, self.end_timestamp, error, message_type.name, data_symbols, self.num_messages)

    def __str__(self):
        if self.error:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' ERROR]'
        else:
            return '[MessageBlock: ' + dt.strftime(self.start_timestamp, '%H:%M:%S.%f') + '-' + dt.strftime(self.end_timestamp, '%H:%M:%S.%f') + ' COUNT:' + str(self.num_messages)  + ' ID:' + hex(self.can_id) + ' Data:' + hex(self.data) + ']'


class Log:
    '''
        Parses the header of a MiniMon CSV CAN log eavesdrop
        and saves data into object fields

        Notes:
            - self.baud_rate is in bits/second
            - self.start_time and self.stop_time is a datetime object
    '''
    def __init__(self, header_lines):
        self.minimon_log_format = header_lines[0]

        date_str = header_lines[1].split(': ')[1].strip()
        start_str = date_str + ' ' + header_lines[2].split(': ')[1].strip()
        end_str = date_str + ' ' + header_lines[2].split(': ')[1].strip()

        self.start_time = dt.strptime(start_str, '%m/%d/%Y %I:%M:%S %p')
        self.stop_time = dt.strptime(end_str, '%m/%d/%Y %I:%M:%S %p')

        self.error_frame_count = int(header_lines[4].split()[3])
        self.overrun_count = int(header_lines[4].split()[1])

        baud_parts = header_lines[5].split()
        if baud_parts[2].startswith('kbit'):
            self.baud_rate = int(baud_parts[1])*1000
        elif baud_parts[2].startswith('mbit'):
            self.baud_rate = int(baud_parts[1])*pow(10,6)
        elif baud_parts[2].startswith('bit'):
            self.baud_rate = int(baud_parts[1])

        self.messages = []
        self.message_blocks = []
        self.interpreted_message_blocks = []

    '''
        Parses and saves a message line in the MiniMon log.
        Time saved as datetime object, data and ID fields assumed to be hex
        Format of message line assumed to be:
        "Time","Identifier (hex)","Format","Flags","Data (hex)"
    '''
    def add_message_from_line(self, line):
        line = re.sub(r"\"", "", line)
        line_parts = line.split(',')
        hours = int(line_parts[0].split(':')[0])
        minutes = int(line_parts[0].split(':')[1])
        seconds = int(line_parts[0].split(':')[2].split('.')[0])
        milliseconds = int(line_parts[0].split(':')[2].split('.')[1])

        # Check that the log is indeed timing to the millisecond
        if len(line_parts[0].split(':')[2].split('.')[1]) > 3:
            print 'Unexpected sub-second length. Expected three-digits of subsecond length. Line:\n' + line

        timedelta = datetime.timedelta(hours=hours,minutes=minutes,seconds=seconds,milliseconds=milliseconds)

        can_id_str = line_parts[1] # May contain error string; checked in Message constructor
        format_str = line_parts[2]
        data_str = line_parts[4]

        message = Message(self.start_time + timedelta, can_id_str, format_str, data_str)
        self.messages.append(message)

    ''' Assumes that Log has had messaged added to it '''
    def compress(self):
        if not self.messages: print 'Warning: messages have not been added to log'
        if not self.messages: self.message_blocks = []
        else:
            curr_message = self.messages[0]
            num_messages_in_block = 0
            for message in self.messages:
                if message == curr_message:
                    num_messages_in_block += 1
                else:
                    message_block = MessageBlock(curr_message.timestamp, message.timestamp, curr_message.error, curr_message.can_id, curr_message.data, num_messages_in_block)
                    num_messages_in_block = 1
                    curr_message = message
                    self.message_blocks.append(message_block)

            message_block = MessageBlock(curr_message.timestamp, self.messages[-1].timestamp, curr_message.error, curr_message.can_id, curr_message.data, num_messages_in_block)
            self.message_blocks.append(message_block)

    ''' Assumes that Log has been compressed '''
    def symbolify(self, spec):
        if not self.message_blocks: print 'Warning: log has not been compressed, or has no messages'
        for block in self.message_blocks:
            interpreted_block = block.symbolify(spec)
            self.interpreted_message_blocks.append(interpreted_block)

    '''Frequency analysis checks that messages that need to be sent periodically with certain frequency are being sent at atleast that rate. Ignores all 'Form Error' and 'Stuff Error' MiniMon read-outs that result from faulty UART connections'''
    def check_freqs(self, spec):
        print 'Checking frequency constraints...'
        if not self.messages: return 'Warning: log has no messages. Cannot verify frequency contrainsts'
        tracked_freqs = {}
        for message in self.messages:
            if not message.error:
                first_message = message

        problematic_ids = set() #contains set of can_id, and last updated
        for can_id, message_type in spec.items():
            if message_type.freq:
                delta = datetime.timedelta(seconds=1/float(message_type.freq))
                tracked_freqs[message_type.can_id] = delta, first_message.timestamp

        for message in self.messages:
            if message.error: continue
            if message.can_id in tracked_freqs:
                tracked_freqs[message.can_id] = tracked_freqs[message.can_id][0], message.timestamp
            for can_id, (delta, timestamp) in tracked_freqs.items():
                if message.timestamp.second == 52:
                    print timestamp, delta, delta+timestamp, message.timestamp, delta+timestamp < message.timestamp
                if delta+timestamp < message.timestamp:
                    problematic_ids.add((can_id,timestamp))
                    del tracked_freqs[can_id]

        errors = ''
        for can_id, timestamp in problematic_ids:
            if first_message.timestamp == timestamp:
                errors += '{0} (CAN ID {1}) do not satisfy frequency constraints. Problematic time frame begins at the beginning of .'.format(spec[can_id].name, hex(can_id))
            else:
                errors += '{0} (CAN ID {1}) do not satisfy frequency constraints:\nProblematic time frame starts at {2}.\n'.format(spec[can_id].name, hex(can_id), str(timestamp - self.start_time))


        return errors


    def __str__(self):
        if not self.interpreted_message_blocks: print 'Warning: log has not been symbolified, or has no messages'
        return '[MiniMon Log: StartTime:' + dt.strftime(self.start_time,'%H:%M:%S.%f') + ' Messages:\n\t' + \
                '\n\t'.join([str(message_block) for message_block in self.interpreted_message_blocks]) + '\n]'


class DataSegment:
    def __init__(self, name, boundary, is_float):
        self.boundary = boundary
        self.name = name
        # Maps values to their corresponding symbol
        self.values = {}
        self.is_float = is_float

    def add_value(self, value, symbol):
        self.values[value] = symbol

    '''
        Checks that:
            - Name and boundary are defined
            - Data values or corresponding value identifiers are not repeated
            - Data values fit within allocated space
            - Segment boundaries adhere to the CAN specs
    '''
    def validate(self):
        errors = ''
        if not self.name or not self.boundary:
            print 'No name or boundary defined for segment' + str(self.name) +' '+ str(self.boundary)
            sys.exit(1)
        if not (self.boundary[0] > -1 and self.boundary[1] < 64 and self.boundary[0] <= self.boundary[1]):
            errors += 'Boundary checks failed for ' + str(self.name) + '\n'

        boundary_size = self.boundary[1] - self.boundary[0] + 1

        if self.is_float:
            if len(self.values) > 1 or (None not in self.values):
                errors += "Invalid float data field for segment {0}".format(self.name)
                return errors, True
            return '', False
        else:
            for value, symbol in self.values.items():
                if type(value) == type(1):
                    if len(bin(value)[2:]) > boundary_size:
                        errors += "Data value {0} will no fit in allocated segment with name {1}\n".format(value, self.name)
                    if value < 0:
                        errors += "Negative values in %s" % self.name + '\n'
                else:
                    assert type(value) == type((1,1)) #data must be a threshold range
                    if len(bin(value[0])[2:]) > boundary_size or len(bin(value[1])[2:]) > boundary_size:
                        errors += "Data value {0} will no fit in allocated segment with name {1}\n".format(value, self.name)
                    if value[1] < value[0]:
                        errors += "Data value threshold {0} does not make sense.\n".format(value)
                    if value < 0:
                        errors += "Negative values in %s" % self.name + '\n'

            if len(set(self.values.keys())) < len(self.values.keys()):
                errors += "Duplicate values detected in data segment %s" % self.name

            if len(set(self.values.values())) < len(self.values.values()):
                errors += "Duplicate value symbols detected in data segment %s" % self.name

            if errors: return errors, True
            else: return '', False

    def __str__(self):
        out = 'DATA_NAME=' + self.name + ' POSITION=' + str(self.boundary) + '\n\t'
        out += '\n\t'.join([str(value) + ' ' + symbol for value, symbol in self.values.items()])
        return out


class MessageType:
    def __init__(self, name, can_id, endian, freq):
        self.name = name
        self.can_id = can_id
        self.endian = endian
        self.freq = freq
        self.segments = []

    '''
        Adds a new allocation to the data field
        Maintains the data segment/allocations in order
    '''
    def add_data_segment(self, segment):
        for i, s in enumerate(self.segments):
            if s.boundary[0] > segment.boundary[0]:
                self.segments.insert(i, segment)
                return
        self.segments.append(segment)

    '''
        Checks that:
            - Name and ID are defined
            - Boundaries do not overlap and are in sorted order
            - CAN ID fits within CAN frame allocation are within correct range
            - Data field names are unique
            - Data field allocations are valid
    '''
    def validate(self):
        errors = ''
        errors_present = False
        if not self.name or not self.can_id:
            print 'No name or ID defined for message type' + str(self.name) +' '+ str(self.can_id)
            sys.exit(1)

        if self.segments:
            last_segment  = self.segments[0]
            for segment in self.segments[1:]:
                if last_segment.boundary[1] >= segment.boundary[0]:
                    errors += 'Overlapping bit position boundaries in message type ' + self.name + '\n'
                    errors_present = True

        if len(bin(self.can_id)[2:]) > 11 or self.can_id < 0:
            errors += "Defined CAN ID for message type %s does not fit in CAN frame" % self.name + '\n'
            errors_present = True

        names = set()
        for segment in self.segments:
            error, present = segment.validate()
            errors += error
            errors_present = errors_present or present
            if segment.name in names:
                errors += 'Repeated data segment name {0} in message type {1} '.format(segment.name, self.name) + '\n'
                errors_present = True
            else:
                names.add(segment.name)


        return errors, errors_present

    def __str__(self):
        out = 'MESSAGE_NAME=' + str(self.name) + ' ID=' + str(self.can_id) + ' ENDIAN=' + str(self.endian) + ' FREQ=' + str(self.freq) + '\n\t'
        out += '\n\t'.join([str(segment) for segment in self.segments])
        return out


def parse_message_line(line):
    assert line.startswith('MESSAGE_NAME')

    message_parts = line.split()
    NAME, ID, ENDIAN, FREQ = tuple([None]*4)

    for message_part in message_parts:
        part = message_part.split('=')

        if part[0] == 'MESSAGE_NAME':
            NAME = part[1]
        elif part[0] == 'ID':
            ID = intb(part[1])
        elif part[0] == 'ENDIAN':
            if part[1] == 'LITTLE':
                ENDIAN = 'LITTLE'
            elif part[1] == 'BIG':
                ENDIAN = 'BIG'
            else:
                print 'Unrecognized endian on line',i

        elif part[0] == 'FREQ':
            FREQ = parse_frequency(part[1])
        else:
            print 'Unrecognized message parameter on line',i

    return MessageType(NAME, ID, ENDIAN, FREQ)


def parse_data_line(line):
    assert line.startswith('DATA_NAME')

    segment_info = line.split()
    NAME, BOUNDARY, IS_FLOAT = None, None, False


    for info in segment_info:
        if info == 'FLOAT':
            IS_FLOAT=True
        else:
            parts = info.split('=')
            if parts[0] == 'DATA_NAME':
                NAME = parts[1]
            elif parts[0] == 'POSITION':
                boundary_parts = parts[1].split(':')
                BOUNDARY = (intb(boundary_parts[0]), intb(boundary_parts[1]))
            else:
                print 'Unrecognized field in data segment declaration line:\n\t',line

    return DataSegment(NAME, BOUNDARY, IS_FLOAT)


def parse_data_lines(lines):
    data_segment = parse_data_line(lines[0])

    for line in lines[1:]:
        parts = line.split()
        if len(parts) == 1: #MUST BE FLOAT
            assert data_segment.is_float
            data_segment.add_value(None, parts[0])
        else:
            if '-' in parts[0]:
                range_parts = parts[0].split('-')
                assert len(range_parts) == 2
                data_segment.add_value((intb(range_parts[0]), intb(range_parts[1])), parts[1])
            else:
                data_segment.add_value(intb(parts[0]), parts[1])

    return data_segment


def parse_message_lines(lines):
    message_type = parse_message_line(lines[0])
    data_begin_indices = [i for i, line in enumerate(lines) if line.strip().startswith('DATA_NAME')]

    if len(data_begin_indices) != 0:
        for i, begin_index in enumerate(data_begin_indices[:-1]):
            data_segment = parse_data_lines(lines[begin_index:data_begin_indices[i+1]])
            message_type.add_data_segment(data_segment)

        data_segment = parse_data_lines(lines[data_begin_indices[-1]:])
        message_type.add_data_segment(data_segment)

    return message_type


'''
    Given a valid specification returns a dictionary of valid MessageType objects
    mapping MessageType ID to the MessageType object
'''
def parse_spec_file(spec_file):
    spec_file.seek(0)
    filtered_lines = filter(lambda line: line[0] != '#' and line.strip() != '', spec_file.readlines())

    filtered_lines = [line.strip() for line in filtered_lines]
    message_begin_indices = [i for i, line in enumerate(filtered_lines) if line.startswith("MESSAGE_NAME")]

    message_types = {}

    for i, begin_index in enumerate(message_begin_indices[:-1]):
        message = parse_message_lines(filtered_lines[begin_index:message_begin_indices[i+1]])
        message_types[message.can_id] = message

    message = parse_message_lines(filtered_lines[message_begin_indices[-1]:])
    message_types[message.can_id] = message

    return message_types


def parse_message_log(log_file):
    log_file.seek(0)
    lines = log_file.readlines()
    header_end_index = 0
    for i, line in enumerate(lines):
        if line.startswith("\"Time"):
            header_end_index = i
    if header_end_index == 0:
        print 'Problem parsing log! Does file have header?'
    log = Log(lines[0:header_end_index])
    for line in lines[header_end_index+1:]:
        log.add_message_from_line(line)

    return log


'''
    Reads spec file, and validates it
    For every message on MiniMon CSV CAN bus log:
        Validates that message has valid ID, and valid data and returns the interpretation
        Compresses the output so that the same message repeated multiple times in sequence occupies only one line, with time difference and frequency
        Checks that message frequency constraints in the spec struct are maintained
'''
def check_log(spec_file, log_file):
    spec = parse_spec_file(spec_file)
    log = parse_message_log(log_file)
    log.compress()
    log.symbolify(spec)
    errors = log.check_freqs(spec)
    return log, errors

'''
    Checks that:
        - Message type names and IDs do not collide
        - MessageType and DataSegment objects are valid (see validation methods in DataSegment and MessageType)
'''
def validate_spec(spec_file):
    message_types = parse_spec_file(spec_file)

    errors_present = False
    errors = ''
    names = set()
    ids = set()

    for message_id, message_type in message_types.items():
        error, present = message_type.validate()
        errors += error
        errors_present = errors_present or present

        if message_type.name in names:
            errors += 'Repeated message type name %s ' % message_type.name + '\n'
            errors_present = True
        else:
            names.add(message_type.name)

        if message_type.can_id in ids:
            errors += 'Repeated message type ID %d ' % message_type.can_id + '\n'
            errors_present = True
        else:
            ids.add(message_type.can_id)

    if errors_present:
        return 'Errors present!\n' + errors
    else:
        return 'No errors in specification file %s!' % spec_file.name

def rh(text):
  return text.replace("-", "_")  


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Validates our CAN usage specification and analyzes that logs conform to our specification")
    parser.add_argument('--validate_spec', dest='validate_spec', type=file, nargs=1, help='Validates your CAN usage specification')
    parser.add_argument('--generate_header', dest='generate_header', type=str, nargs=2, help='Validates your CAN usage specification and generates corresponding C header file')
    parser.add_argument('--check_log', dest='check_log', type=file, nargs=2, help='Check your CAN usage specification')

    args = parser.parse_args()

    if args.validate_spec:
        errors = validate_spec(args.validate_spec[0])
        args.validate_spec[0].close()
        print errors
    elif args.generate_header:
        spec_file = open(args.generate_header[0], 'r')
        output_file = args.generate_header[1]
        try:
            errors = validate_spec(spec_file)
            print errors
            if errors.startswith('No errors in'):
                message_types = parse_spec_file(spec_file)
                with open(output_file, 'w') as f:
                    for message_id, message_type in message_types.items():
                        message_name = rh(message_type.name)
                        f.write('#define ' + message_name + '__id ' + str(message_type.can_id) + '\n')
                        if message_type.freq:
                            f.write('#define ' + message_name + '__freq ' + str(int(message_type.freq)) + '\n')
                        else:
                            f.write('#define ' + message_name + '__freq ' + str(-1) + '\n')
                        for segment in message_type.segments:
                            segment_name = rh(segment.name)
                            f.write('#define __' + message_name + '__' + segment_name + '__start ' + str(segment.boundary[0]) + '\n')
                            f.write('#define __' + message_name + '__' + segment_name + '__end ' + str(segment.boundary[1]) + '\n')
                            for value, symbol in segment.values.items():
                                if isinstance(value, tuple):
                                    f.write('#define ____' + message_name + '__' + segment_name + '__' + symbol + '__FROM ' + str(value[0]) + '\n')
                                    f.write('#define ____' + message_name + '__' + segment_name + '__' + symbol + '__TO ' + str(value[1]) + '\n')
                                else:
                                    if value is None:
                                        f.write('#define ____' + message_name + '__' + segment_name + '__' + symbol + ' ' + str(0) + '\n')
                                    else:
                                        f.write('#define ____' + message_name + '__' + segment_name + '__' + symbol + ' ' + str(value) + '\n')


                        f.write('\n')
        finally:
            spec_file.close()

        print 'Wrote constants to ' + output_file + '!'
    elif args.check_log:
        print 'First validating spec file...'
        errors = validate_spec(args.check_log[0])
        print errors
        print 'Reading and interpreting log...'
        log, errors = check_log(args.check_log[0], args.check_log[1])
        args.check_log[0].close()
        args.check_log[1].close()
        print log
        if errors: print 'FREQUENCY CONSTRAIN ERRORS:\n', errors
        else: print 'No frequency contraints violated!'
    else:
        parser.print_help()
        sys.exit(2)
