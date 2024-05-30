import socket, time, math, configparser, queue, sys, os, gc
import optparse, logging, time, collections, importlib, util
import queuelogger, msgproto, configfile, pins, mcu
from stupidArtnet import StupidArtnetServer







# Main class to handle stepper motors, calculate target, speed, position, etc.
class Parrafraktor:
    def __init__(self, config):
        self.system_time = time.time()  # Corrected from time.Time()
        self.name = 'Parrafraktor X'
        self.universe = 0
        self.dmx_start_address = 1
        self.dmx_channel_mode = 4
        self.dmx_debug_channel = 2
        self.ip_address = ""
        self.is_connected_to_network = False
        self.dmx_timer = 0.001  # 1ms we need this later
        self.objects = collections

        #serial mcu connection
        self.mcu = None

        # Art-Net receiver
        
        self.artnet = None
        self.artnet_listener = None

        # Create DMX values lists
        self.dmx_values_old = []
        self.dmx_values_new = []

        # Initialize the stepper motors and UART steppers to empty lists
        self.steppers = []
        self.uart_steppers = []
        self.create_steppers(self.dmx_start_address)

    # Read DMX values from the Art-Net server for the steppers, each stepper has 4 channels for the target_position_coarse, target_position_fine, speed
    # and extra stuff like homing or enabling
    # The debug channel is for debugging purposes later
    def read_dmx(self, dmx_start_address):
        buffer = self.artnet.get_buffer(self.artnet_listener)
        if buffer:
            self.dmx_values_old = self.dmx_values_new
            self.dmx_values_new = buffer[dmx_start_address : (dmx_start_address + (len(self.steppers) * self.dmx_channel_mode) + self.dmx_debug_channel)]
        else:
            self.manual_mode()
            print('No DMX values received')

    # Set the target position for the stepper motor from the DMX values. Target position is a 16-bit value,
    # the first 8 bits are the coarse value and the second 8 bits are the fine value
    def set_target_position(self, stepper):
        coarse_address = stepper.dmx_address
        fine_address = stepper.dmx_address + 1

        coarse_value = self.dmx_values_old[coarse_address]
        fine_value = self.dmx_values_old[fine_address]

        position = (coarse_value << 8) | fine_value

        if position > stepper.max_pos:
            position = stepper.max_pos
        elif position < stepper.min_pos:
            position = stepper.min_pos

        stepper.set_target_position(position)

    # Set target for all steppers from DMX values
    def set_all_target_positions(self):
        for stepper in self.steppers:
            self.set_target_position(stepper)

    # Set the speed for the stepper motor from the DMX values
    def set_dmx_speed(self, stepper):
        speed_address = stepper.get_dmx_channel(2)
        speed = self.dmx_values_old[speed_address]
        stepper.set_speed(speed)

    # Set speed for all steppers from DMX values
    def set_all_dmx_speeds(self):
        for stepper in self.steppers:
            self.set_dmx_speed(stepper)

    # Calculate and set the next step speed for a stepper
    def set_next_step_speed(self, stepper):
        position = stepper.get_position()
        target_position = stepper.get_target_position()
        speed = stepper.get_speed()
        max_speed = stepper.get_max_speed()
        acceleration = stepper.get_acceleration()

        # Calculate the distance to the target position
        distance = abs(target_position - position)

        # Calculate the acceleration distance
        acceleration_distance = (max_speed ** 2) / (2 * acceleration)

        # Check if the stepper is in the acceleration phase
        if distance <= acceleration_distance:
            # Calculate the current speed based on the acceleration
            current_speed = math.sqrt(2 * acceleration * distance)
        # Check if the stepper is in the deceleration phase
        elif distance > (target_position - acceleration_distance):
            # Calculate the current speed based on the deceleration
            current_speed = math.sqrt(2 * acceleration * (target_position - distance))
        # Stepper is in the cruise phase
        else:
            # Set the current speed to the maximum speed
            current_speed = max_speed

        # Set the current speed for the stepper
        stepper.set_speed(current_speed)



    def set_all_next_speed(self):
        for stepper in self.steppers:
            self.set_next_step_speed(stepper)

    # Create movement parameters for a stepper
    def create_move(self, stepper):
        step_pin = stepper.get_step_pin()
        dir_pin = stepper.get_dir_pin()
        position = stepper.get_position()
        target = stepper.get_target_position()
        speed = stepper.get_speed()

        # Get direction for movement
        if target == position:
            return None
        direction = 1 if target > position else -1

        # Set new stepper position from movement direction
        stepper.set_position(position + direction)

        move_param = (step_pin, dir_pin, direction, speed)
        return move_param

    def create_all_moves(self):
        move_list = []

        for stepper in self.steppers:
            move = self.create_move(stepper)
            if move:
                move_list.append(move)
        return move_list

    # Read DMX, create targets from DMX, set speed from DMX, create moves and send them to the MCU
    def parrafraktor_run(self):
        self.read_dmx(self.dmx_start_address)
        self.set_all_target_positions()
        self.set_all_dmx_speeds()
        move_list = self.create_all_moves()
        self.mcu_connection.send_command(move_list)

    # Create a stepper object for each stepper motor and set the DMX address for each stepper motor
    def create_steppers(self, dmx_start_address):
        self.dmx_start_address = dmx_start_address
        self.steppers = []
        for steppers in range(6):
            i= 1
            stepper = Stepper('Stepper ' + str(i), 0, 0, 0, 1, 1, dmx_start_address + (i * self.dmx_channel_mode))
            self.steppers.append(stepper)
            i += 1
            
    
    

    # add objects to the object list
    def add_object(self, name, obj):
        if name in self.objects:
            raise self.config_error(
                "Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    
    # Get the object for name    
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        prefix = module + ' '
        objs = [(n, self.objects[n])
                for n in self.objects if n.startswith(prefix)]
        if module in self.objects:
            return [(module, self.objects[module])] + objs
        return objs
    
    def load_object(self, config, section, default=configfile.sentinel):
        if section in self.objects:
            return self.objects[section]
        module_parts = section.split()
        module_name = module_parts[0]
        py_name = os.path.join(os.path.dirname(__file__), 'extras', module_name + '.py')
        py_dirname = os.path.join(os.path.dirname(__file__), 'extras', module_name, '__init__.py')
        
        if not os.path.exists(py_name) and not os.path.exists(py_dirname):
            if default is not configfile.sentinel:
                return default
            raise self.config_error("Unable to load module '%s'" % (section,))
        mod = importlib.import_module('extras.' + module_name)
        init_func = 'load_config'
        
        if len(module_parts) > 1:
            init_func = 'load_config_prefix'
        init_func = getattr(mod, init_func, None)
        
        if init_func is None:
            if default is not configfile.sentinel:
                return default
            raise self.config_error("Unable to load module '%s'" % (section,))
        self.objects[section] = init_func(config.getsection(section))
        return self.objects[section]
    
    def _read_config(self):
        self.objects['configfile'] = pconfig = configfile.PrinterConfig(self)
        config = pconfig.read_main_config()
   
        # Create printer components
        for m in [pins, mcu]:
            m.add_printer_objects(config)
            
        for section_config in config.get_prefix_sections(''):
            self.load_object(config, section_config.get_name(), None)
            
        # Validate that there are no undefined parameters in the config file
        pconfig.check_unused_options(config)
        





    def manual_mode(self):
        pass

    ############################################### GETTER ###############################################
    def get_other_parrafraktor_parts(self):
        # Get other Parrafraktor parts
        pass

    ############################################### SETTER ###############################################
    def set_mcu(self, mcu):
        self.mcu = mcu

    def set_arnet(self, artnet):
        self.artnet = artnet
        self.register_artnet_listener()
        
    def register_artnet_listener(self):
        self.artnet_listener = self.artnet.register_listener(self.universe, callback_function=self.test_callback)
    ############################################### OTHER METHODS ###############################################
    def send_network_message(self, ip_address, message):
        try:
            # Create a UDP socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Set a timeout for receiving responses
            s.settimeout(1)

            # Iterate over IP addresses in the same range except for the current machine
            for i in range(1, 256):
                target_ip = ip_address.rsplit('.', 1)[0] + '.' + str(i)
                if target_ip != ip_address:
                    s.sendto(message.encode(), (target_ip, 12345))  # Adjust the port number as needed
            s.close()
        except socket.error:
            print("Unable to send message")

    def get_ip_address(self):
        try:
            # Create a socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Connect to any external server
            s.connect(("8.8.8.8", 80))
            # Get the local IP address
            ip_address = s.getsockname()[0]
            self.is_connected_to_network = True
            return ip_address
        except socket.error:
            self.is_connected_to_network = False
            return "Unable to get IP Address"

    # DMX callback function
    def test_callback(self, data):
        print('Callback function called')

class Stepper:
    def __init__(self, name, dir_pin, step_pin, enable_pin, microsteps, rotation_distance, dmx_address):
        self.name = name
        self.dir_pin = dir_pin
        self.step_pin = step_pin
        self.enable_pin = enable_pin
        self.direction = 0
        self.microsteps = microsteps
        self.rotation_distance = rotation_distance

        self.dmx_address = dmx_address
        self.dmx_channels = [dmx_address, dmx_address + 1, dmx_address + 2, dmx_address + 3]

        self.position = 0
        self.target_position = 0
        self.speed = 0
        self.max_speed = 100
        self.acceleration = 2
        self.max_pos = self.rotation_distance * self.microsteps / 2
        self.min_pos = self.max_pos * -1

    ############################################### GETTER ###############################################
    def get_name(self):
        return self.name
    def get_dir_pin(self):
        return self.dir_pin
    def get_step_pin(self):
        return self.step_pin
    def get_enable_pin(self):
        return self.enable_pin
    def get_direction(self):
        return self.direction

    def get_microsteps(self):
        return self.microsteps
    def get_rotation_distance(self):
        return self.rotation_distance
    def get_dmx_address(self):
        return self.dmx_address
    def get_dmx_channel(self, i):
        return self.dmx_channels[i]
    def get_position(self):
        return self.position
    def get_target_position(self):
        return self.target_position
    def get_speed(self):
        return self.speed
    def get_max_speed(self):
        return self.max_speed
    def get_acceleration(self):
        return self.acceleration

    ############################################### SETTER ###############################################
    def set_name(self, name):
        self.name = name
    def set_direction(self, direction):
        self.direction = direction
    def set_microsteps(self, microsteps):
        self.microsteps = microsteps
        self.set_position_limits()
    def set_rotation_distance(self, rotation_distance):
        self.rotation_distance = rotation_distance
        self.set_position_limits()
    def set_dmx_address(self, dmx_address):
        self.dmx_address = dmx_address
    def set_dmx_channels(self, dmx_channels):
        self.dmx_channels = dmx_channels
    def set_speed(self, speed):
        self.speed = speed
    def set_max_speed(self, max_speed):
        self.max_speed = max_speed
    def set_acceleration(self, acceleration):
        self.acceleration = acceleration

    def set_position_limits(self):
        self.max_pos = self.rotation_distance * self.microsteps / 2
        self.min_pos = self.max_pos * -1
    def set_position(self, position):
        self.position = position
    def set_target_position(self, target_position):
        self.target_position = target_position

class UartStepper(Stepper):
    def __init__(self, name, dir_pin, step_pin, enable_pin, microsteps, rotation_distance, uart_pin, diag_pin, run_current=0.800, stealthchop_threshold=None):
        super().__init__(name, dir_pin, step_pin, enable_pin, microsteps, rotation_distance, None)
        self.uart_pin = uart_pin
        self.diag_pin = diag_pin
        self.run_current = run_current
        self.stealthchop_threshold = stealthchop_threshold

class MCU:
    def __init__(self, port):
        self.port = port
        self.command_queue = queue.Queue()

    def connect(self):
        
        print('Connecting to port:', self.port)

    def disconnect(self):
        print('Disconnecting from port:', self.port)

    def get_MCU_status(self):
        print('Getting status of the MCU')

    def send_command(self, command_block):
        print('Sending command:', command_block)

    def receive_response(self):
        print('Receiving response from the MCU')

    def put_queue(self, command):
        self.command_queue.put(command)
        print('Command added to queue:', command)

class CommandWrapper:
    def __init__(self, command, mcu):
        self.command = command
        self.mcu = mcu

    def execute(self):
        print('Executing command:', self.command)

def main():
    config_file = 'your_config_file.ini'
    parrafraktor_part = Parrafraktor(config_file)

    artnet = StupidArtnetServer()
    mcu = MCU('/dev/ttyUSB0')
    parrafraktor_part.set_mcu(mcu)  #  MCU connection setup
    parrafraktor_part.set_artnet(artnet) # create artnet receiver
    
    #create thread here!!!
    parrafraktor_part.parrafraktor_run()

if __name__ == "__main__":
    sys.exit(int(main() or 0))


    