import serial
import struct
import json
import time

class KlipperSerialConnection:
    SYNC_BYTE = 0x7E
    
    def __init__(self, port, baudrate=250000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.sequence = 0

    def connect(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print(f"Disconnected from {self.port}.")

    def calculate_crc(self, message):
        # CCITT CRC-16
        crc = 0xFFFF
        for byte in message:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def encode_command(self, command):
        command_id = self.get_command_id(command)
        if command_id is None:
            raise ValueError(f"Unknown command: {command}")

        message = struct.pack('B', command_id)
        return message

    def get_command_id(self, command):
        # This function should map command names to their corresponding IDs.
        # Here is a simple example:
        command_map = {
            "get_config": 0x01  # Example command ID
        }
        return command_map.get(command, None)

    def send_command(self, command):
        if self.serial_conn and self.serial_conn.is_open:
            encoded_command = self.encode_command(command)
            length = len(encoded_command) + 5  # Length of the message including headers and trailers
            sequence_byte = (self.sequence & 0x0F) | 0x10  # Sequence number with reserved bits

            message = bytearray()
            message.append(length)
            message.append(sequence_byte)
            message.extend(encoded_command)

            crc = self.calculate_crc(message)
            message.extend(struct.pack('>H', crc))
            message.append(self.SYNC_BYTE)

            self.serial_conn.write(message)
            self.sequence += 1

            print(f"Sent command: {command}")
        else:
            print("Serial connection not open.")

    def read_response(self):
        if self.serial_conn and self.serial_conn.is_open:
            response = self.serial_conn.read_until(bytes([self.SYNC_BYTE]))
            if len(response) < 5:
                print("Invalid response length.")
                return None

            length = response[0]
            sequence = response[1]
            content = response[2:-3]
            received_crc = struct.unpack('>H', response[-3:-1])[0]

            calculated_crc = self.calculate_crc(response[:-3])
            if received_crc != calculated_crc:
                print("CRC mismatch.")
                return None

            try:
                decoded_message = json.loads(content.decode())
                print(f"Received response: {decoded_message}")
                return decoded_message
            except json.JSONDecodeError as e:
                print(f"Error decoding JSON: {e}")
                return None
        else:
            print("Serial connection not open.")
            return None

if __name__ == "__main__":
    klipper_conn = KlipperSerialConnection("/dev/ttyACM0")
    klipper_conn.connect()
   
