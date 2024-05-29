import serial

class KlipperSerialConnection:
    def __init__(self, port, baudrate=250000):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None

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

    def send_command(self, command):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(command.encode())
            print(f"Sent command: {command}")
        else:
            print("Serial connection not open.")

    def read_response(self):
        if self.serial_conn and self.serial_conn.is_open:
            response = self.serial_conn.readline()        
            msg = data.decode()
            print(f"Received response: {msg}")
            return response
        else:
            print("Serial connection not open.")
            return None

if __name__ == "__main__":
    klipper_conn = KlipperSerialConnection("/dev/serial/by-id/usb-Klipper_stm32g0b1xx_m8p-if00")
    klipper_conn.connect()
    klipper_conn.send_command("get_config")
    klipper_conn.read_response()
    while 1:
        klipper_conn.send_command("get_config")
        klipper_conn.read_response()
    	

    klipper_conn.disconnect()
