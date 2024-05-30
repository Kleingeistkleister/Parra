import struct
import time

class DigitalOutputController:
    def __init__(self, serial_conn):
        self.serial_conn = serial_conn

    def send_command(self, command):
        self.serial_conn.write(command.encode())

    def config_digital_out(self, oid, pin, value, default_value, max_duration):
        command = f"config_digital_out oid={oid} pin={pin} value={'1' if value else '0'} default_value={'1' if default_value else '0'} max_duration={max_duration}\n"
        self.send_command(command)

    def set_digital_out_pwm_cycle(self, oid, cycle_ticks):
        command = f"set_digital_out_pwm_cycle oid={oid} cycle_ticks={cycle_ticks}\n"
        self.send_command(command)

    def queue_digital_out(self, oid, clock, on_ticks):
        command = f"queue_digital_out oid={oid} clock={clock} on_ticks={on_ticks}\n"
        self.send_command(command)

    def update_digital_out(self, oid, value):
        command = f"update_digital_out oid={oid} value={'1' if value else '0'}\n"
        self.send_command(command)

    def set_digital_out(self, pin, value):
        command = f"set_digital_out pin={pin} value={'1' if value else '0'}\n"
        self.send_command(command)

if __name__ == "__main__":
    # Assume serial_conn is an instance of the KlipperSerialConnection class
    controller = DigitalOutputController()


    # Wait for commands to be processed
    time.sleep(1)
