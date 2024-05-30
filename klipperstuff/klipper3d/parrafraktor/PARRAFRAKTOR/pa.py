import threading
import time
from stupidArtnet import StupidArtnet

class Stepper:
    def __init__(self, mcu, step_pin, dir_pin, endstop_pin, steps_per_rotation, max_speed, acceleration, microsteps, dmx_start_address):
        self.mcu = mcu
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.endstop_pin = endstop_pin
        self.steps_per_rotation = steps_per_rotation
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.microsteps = microsteps
        self.dmx_start_address = dmx_start_address
        self.position = 0  # Initial position in mm
        self.oid_stepper = self.allocate_oid()
        self.oid_endstop = self.allocate_oid()
        self.configure_stepper()
        self.configure_endstop()
        self.register_callbacks()
        self.set_microsteps(self.microsteps)  # Set initial microsteps

    def allocate_oid(self):
        oid_command = "allocate_oids count=1"
        response = self.mcu.send_command(oid_command)
        return int(response.split('=')[1])  # Assuming response format is "oid=N"

    def configure_stepper(self):
        config_command = (
            f"config_stepper oid={self.oid_stepper} step_pin={self.step_pin} "
            f"dir_pin={self.dir_pin} invert_step=0"
        )
        self.mcu.send_command(config_command)

    def configure_endstop(self):
        config_command = (
            f"config_endstop oid={self.oid_endstop} pin={self.endstop_pin} pull_up=1 stepper_count=1"
        )
        self.mcu.send_command(config_command)

    def set_direction(self, direction):
        direction_value = 1 if direction == 'forward' else 0
        direction_command = f"set_direction oid={self.oid_stepper} dir={direction_value}"
        self.mcu.send_command(direction_command)

    def move(self, target_position, speed):
        distance_mm = target_position - self.position
        steps = int(distance_mm * self.steps_per_rotation)
        speed = int(speed * self.steps_per_rotation)
        move_command = f"queue_step oid={self.oid_stepper} steps={steps} speed={speed}"
        self.mcu.send_command(move_command)
        self.position = target_position

    def handle_endstop_triggered(self, data):
        self.position = 0  # Reset position to 0 when endstop is triggered
        print(f"Endstop triggered: {data}")

    def register_callbacks(self):
        self.mcu.register_callback('endstop_triggered', self.handle_endstop_triggered)

    def set_microsteps(self, microsteps):
        microsteps_command = f"set_microsteps oid={self.oid_stepper} microsteps={microsteps}"
        self.mcu.send_command(microsteps_command)

    def stop(self):
        self.mcu.stop()

class Controller:
    def __init__(self, mcu, dmx_universe, dmx_ip, num_channels, steppers_config):
        self.mcu = mcu
        self.dmx_handler = DMXHandler(dmx_universe, dmx_ip, num_channels, self)
        self.steppers = []
        for config in steppers_config:
            stepper = Stepper(
                mcu=self.mcu,
                step_pin=config['step_pin'],
                dir_pin=config['dir_pin'],
                endstop_pin=config['endstop_pin'],
                steps_per_rotation=config['steps_per_rotation'],
                max_speed=config['max_speed'],
                acceleration=config['acceleration'],
                microsteps=config['microsteps'],
                dmx_start_address=config['dmx_start_address']
            )
            self.steppers.append(stepper)
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def control_loop(self):
        while self.running:
            self.update_steppers()
            time.sleep(0.1)  # Update rate

    def update_steppers(self):
        for stepper in self.steppers:
            target_position, speed, acceleration, microsteps = self.dmx_handler.get_dmx_data(stepper.dmx_start_address)
            stepper.set_microsteps(microsteps)
            current_position = stepper.position
            if target_position != current_position:
                direction = 'forward' if target_position > current_position else 'backward'
                stepper.set_direction(direction)
                if direction == 'backward' and current_position < target_position:
                    stepper.move(current_position, 0)  # Brake to stop before reversing
                stepper.move(target_position, speed)
        self.flush_all_commands()

    def flush_all_commands(self):
        for stepper in self.steppers:
            stepper.flush_commands()

    def stop(self):
        self.running = False
        self.control_thread.join()
        self.dmx_handler.stop()
        for stepper in self.steppers:
            stepper.stop()
        self.mcu.stop()


class DMXHandler:
    def __init__(self, universe, ip, num_channels, controller):
        self.artnet = StupidArtnet(ip, universe, num_channels)
        self.controller = controller
        self.artnet.start()
        self.running = True
        self.thread = threading.Thread(target=self.update)
        self.thread.start()

    def update(self):
        while self.running:
            data = self.artnet.read()
            self.dmx_data = data
            time.sleep(0.1)  # Update rate

    def get_dmx_data(self, start_address):
        target_position = self.dmx_data[start_address] + (self.dmx_data[start_address + 1] << 8)
        speed = self.dmx_data[start_address + 2]
        acceleration = self.dmx_data[start_address + 3]
        microsteps = self.dmx_data[start_address + 4]
        return target_position, speed, acceleration, microsteps

    def stop(self):
        self.running = False
        self.thread.join()
        self.artnet.stop()
