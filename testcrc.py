import serial
import struct
import crcmod

# Establish a serial connection
ser = serial.Serial('/dev/ttyUSB0', 250000, timeout=1)  # Adjust the port and baud rate as needed

# Define a function to calculate the CRC-16-CCITT
crc16 = crcmod.predefined.mkPredefinedCrcFun('crc-ccitt-false')

def send_command(command_id, params=[]):
    # Create the message block content
    message_content = struct.pack('B', command_id)
    for param in params:
        message_content += vlq_encode(param)
    
    # Create the message block
    length = 5 + len(message_content)
    sequence = 0x10  # Example sequence number
    crc = crc16(struct.pack('BB', length, sequence) + message_content)
    message_block = struct.pack('BB', length, sequence) + message_content + struct.pack('H', crc) + b'\x7e'
    
    # Send the message block
    ser.write(message_block)

def vlq_encode(value):
    # Variable Length Quantity encoding
    result = bytearray()
    value = (value << 1) ^ (value >> 31)
    while value >= 0x80:
        result.append((value & 0x7f) | 0x80)
        value >>= 7
    result.append(value & 0x7f)
    return bytes(result)

# Define command ID for get_config (based on the data dictionary)
id_get_config = 2  # This should be obtained from the data dictionary

# Send the get_config command
send_command(id_get_config)

# Read and process the response
def read_response():
    response = ser.read_until(b'\x7e')
    if len(response) < 5:
        return None
    
    length, sequence = struct.unpack('BB', response[:2])
    content = response[2:-3]
    crc_received, = struct.unpack('H', response[-3:-1])
    crc_calculated = crc16(response[:-3])
    
    if crc_received != crc_calculated:
        print("CRC mismatch")
        return None
    
    return content

response = read_response()
if response:
    print("Received response:", response)

# Close the serial connection
ser.close()
