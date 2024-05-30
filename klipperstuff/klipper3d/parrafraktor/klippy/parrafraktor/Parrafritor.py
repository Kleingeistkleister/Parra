
import queue
import threading
import serial
import configparser
import socket
import time
from stupidArtnet.StupidArtnetServer import StupidArtnetServer

# Dummy class for data output (to be implemented later)
class DataOut:
    def __init__(self):
        pass


class Controller:
    def __init__(self, name ,steppers , dmx_start_address, dmx_channel_mode , control_center_ip , control_center_port):
        self.name = name
        
        self.ip_address = self.get_ip_address()
        self.dmx_start_address = dmx_start_address
        self.artnet_server = StupidArtnetServer()
        self.dmx_channel_mode = dmx_channel_mode
        self.control_center_ip = control_center_ip
        self.control_center_port = control_center_port
        self.data_out = DataOut()  # Initialize data output object
        self.steppers = None
        
        if self.ip_address:
            print("IP Address:", self.ip_address)
        else:
            print("Failed to get IP address.")
            
        print(f"{self.name} created")
        
        #creating steppers
    def set_steppers(self, steppers):
        self.steppers = steppers
        for i in steppers:
            print(f"{i.get_name()} registerd at controller {self.name}")
        

    def get_ip_address(self):
        try:
            # Create a socket object
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
            # Connect to a remote server
            s.connect(("8.8.8.8", 80))
        
            # Get the local IP address
            ip_address = s.getsockname()[0]
        
            return ip_address
        except Exception as e:
            print("Error getting IP address:", e)
            return None
    
    def get_artnet_server(self):
        return self.artnet_server

    def stop_artnet(self):
        # Stop Artnet server
        self.artnet_server.close()


class Stepper:
    def __init__(self, name,id_number, step_pin, dir_pin, enable_pin, stop_pin,
                  step_invert , dir_invert, steps, max_speed, max_acceleration, micro_steps ,
                 full_revolution_mode, stepper_start_address , dmx_channel_mode, controller, UART_address):
        """
        Initialize Stepper object.

        Parameters:
        - step_pin: Pin number for step signal.
        - dir_pin: Pin number for direction signal.
        - enable_pin: Pin number for enabling the stepper motor.
        - stop_pin: Pin number for stop switch.
        - dir_invert: Boolean flag for inverting direction signal.
        - step_invert: Boolean flag for inverting step signal.
        - steps: Number of steps per revolution.
        - micro_step: Number of micro_steps per step.
        - max_speed: Maximum speed of the stepper motor.
        - max_acceleration: Maximum acceleration of the stepper motor.
        - dmx_mode: DMX mode for controlling stepper motor.
        - revolution_mode: Boolean flag for full or half counting revolution mode.
        """
        
        #name
        self.name = name
        self.id_number = id_number
        # Pin configurations
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.stop_pin = stop_pin
        
        # Direction and step signal configurations
        self.dir_invert = dir_invert
        self.step_invert = step_invert
        
        # Stepper motor values
        self.position = 0
        self.direction = 1
        self.target = 0
        self.speed = 0     
        self.acceleration = 0
        
        #calculation parameters
        self.max_acceleration = max_acceleration      
        self.max_speed = max_speed * micro_steps  #in 16 micro step mode it would be 16 times slower   
        
        
        
        # Step counting mode
        self.steps = steps
        self.micro_steps = micro_steps
        # A 1.8° stepper has 200 (400 in 0.9°) steps per revolution in full step mode or more in micro step mode
        self.steps_per_revolution = micro_steps * steps  
        self.steps_per_half_revolution = 0.5 * self.steps_per_revolution
        
        # DMX configurations
        self.dmx_channel_mode = dmx_channel_mode
        self.stepper_start_address = stepper_start_address
        self.dmx_channel_values_new = [0] * dmx_channel_mode
        self.dmx_channel_values_old = self.dmx_channel_values_new
        self.dmx_input = "ARTNET"  # NORMALLY ARTNET but can change it to cable later
        self.artnet_timeout = 100   # creating a timeout to switch to standalone mode *later ;)
        
        # Control and counting
        self.full_revolution_mode = full_revolution_mode
        self.UART_address = UART_address
        self.running = False
        self.stop = False
        
        self.controller = controller
        
        #pass stepper object to controller object
        self.controller.set_steppers(self)    
        self.set_up_stepper(self, step_pin, 0)
        self.set_up_stepper(self, step_pin, 0)
        


    def set_up_stepper(self , pin , state):
        print(f"set_digital_out {pin} {state}")

############################################################## GETTER & SETTER METHODES ##################################################        
    # Getter methods for each variable
    def get_name(self):
        return self.name
    
    def get_id_number(self):
        return self.id_number
    
    def get_step_pin(self):
        return self.step_pin
    
    def get_dir_pin(self):
        return self.dir_pin
    
    def get_enable_pin(self):
        return self.enable_pin
    
    def get_stop_pin(self):
        return self.stop_pin
    
    def get_dir_invert(self):
        return self.dir_invert
    
    def get_step_invert(self):
        return self.step_invert
    
    def get_position(self):
        return self.position
    
    def get_direction(self):
        return self.direction
    
    def get_target(self):
        return self.target
    
    def get_speed(self):
        return self.speed
    
    def get_acceleration(self):
        return self.acceleration
    
    def get_max_acceleration(self):
        return self.max_acceleration
    
    def get_max_speed(self):
        return self.max_speed
    
    def get_steps(self):
        return self.steps
    
    def get_micro_steps(self):
        return self.micro_steps
    
    def get_steps_per_revolution(self):
        return self.steps_per_revolution
    
    def get_steps_per_half_revolution(self):
        return self.steps_per_half_revolution
    
    def get_dmx_channel_mode(self):
        return self.dmx_channel_mode
    
    def get_stepper_start_address(self):
        return self.stepper_start_address
    
    def get_dmx_channel_values_new(self):
        return self.dmx_channel_values_new
    
    def get_dmx_channel_values_old(self):
        return self.dmx_channel_values_old    
        
    def get_dmx_input(self):
        return self.dmx_input    
    
    def get_artnet_timeout(self):
        return self.artnet_timeout
    
    def get_full_revolution_mode(self):
        return self.full_revolution_mode
    
    def get_UART_address(self):
        return self.UART_address
    
    def get_running(self):
        return self.running
    
    def get_stop(self):
        return self.stop  
    
        # Setter methods for attributes
    def set_name(self, name):
        self._name = name

    def set_id_number(self, id_number):
        self._id_number = id_number

    def set_dir_invert(self):
        self._dir_invert = not self._dir_invert

    def set_step_invert(self):
        self._step_invert = not self._step_invert

    def set_position(self, position):
        self._position = position

    def set_direction(self, direction):
        self._direction = direction

    def set_target(self, target):
        self._target = target

    def set_speed(self, speed):
        if speed > self._max_speed:
            speed = self._max_speed
        self._speed = speed

    def set_acceleration(self, acceleration):
        self._acceleration = acceleration

    def set_max_acceleration(self, max_acceleration):
        self._max_acceleration = max_acceleration

    def set_max_speed(self, max_speed):
        self._max_speed = max_speed

    def set_steps(self):
        self._steps *= self.micro_steps

    def set_micro_steps(self, micro_steps):
        self._micro_steps = micro_steps

    def set_dmx_channel_mode(self, dmx_channel_mode):
        self._dmx_channel_mode = dmx_channel_mode

    def set_stepper_start_address(self, stepper_start_address):
        self._stepper_start_address = stepper_start_address

    def set_dmx_channel_values_new(self, dmx_channel_values_new):
        self._dmx_channel_values_new = dmx_channel_values_new

    def set_dmx_channel_values_old(self, dmx_channel_values_old):
        self._dmx_channel_values_old = dmx_channel_values_old
        
    def set_dmx_input(self, src):
        self.dmx_input = src

    def set_artnet_timeout(self, artnet_timeout):
        self._artnet_timeout = artnet_timeout

    def set_full_revolution_mode(self):
        self._full_revolution_mode = not self._full_revolution_mode

    def set_UART_address(self, UART_address):
        self._UART_address = UART_address

    def set_running(self, running):
        self._running = running

    def set_stop(self, stop):
        self._stop = stop
    
 
    
 ####################################################### STARTING POINT FOR STEPPER #####################################################   
    #add function calls via commands, debug or do other stuff
    
    def start_stepper(self, command):
        if command == "HOMEING":
            self.homeing(5)
        self.stepper_main_loop()
        pass


    def stepper_main_loop(self):
        while(self.stop == False):
            self.stepper_get_artnet()
            self.process_dmx
        print(f"{self.name} HAS STOPPED, starting DEBUG function")    
        time.sleep(5)
        self.stepper_debug()
        
    #here we can chose between artnet-dmx or dmx-cable
    def read_dmx(self):        
        if self.dmx_input == "ARTNET":
            self.stepper_get_artnet()
        elif self.dmx_input == "DMX":
            self.stepper_get_dmx()
        else:
            self.stepper_auto_mode(2)
            



    #implementation of ARTNET DMX
    def stepper_get_artnet(self):    
        # Retrieve the DMX buffer values for the stepper's channels according to starting address of stepper and dmx channel mode -1 = 5
        buffer = self.controller.artnet_server.get_buffer(self.starting_address, self.starting_address + (self.dmx_channel_mode-1))
        # just a test
        print(self.dmx_channels)
        if buffer is None or not buffer:
            # Handle empty buffer, set all channels to 0
            self.dmx_channel_values_new = self.dmx_channel_values_old
        else:
            # Assign the buffer values to self.dmx_channels and store last dmx values
            self.dmx_channel_values_old = self.dmx_channel_values_new
            self.dmx_channel_values_new = buffer
            

    #implementation of weired DMX
    def stepper_get_dmx(self):
        buffer = 1
        self.dmx_channel_values_old = self.dmx_channel_values_new
        self.dmx_channel_values_new = buffer
     

    def stepper_debug(self):
        print("debug msg 1")
        pass



    #the target position is calculated on dmx channel 1 in coarse position with revolution_count, it can be 360 or 180° so its either full or half steps per rotation.
    #and adding a fine target with channel 2
    def calculate_target_position(self, channel_1_value, channel_2_value):
        
        #depending on the full or half rotation count boolean, the dmx values are calculated to full or half positions.
        #half positions are more fine.
        if self.full_revolution_count == True:
            dmx_steps = self.steps_per_revolution
        else:
            dmx_steps = self.steps_per_half_revolution
            
        single_step = dmx_steps / 127
        
        #if dmx channel 1 and 2 are both 0, make a continious rotation in negative direction and in positive if both are 255
        #this position will never be reached, because of the automated set 0 position after a full or half rotation.
        # need to make a hotfix for acceleration
        if (channel_1_value == 0 and channel_2_value == 0 ):
            return dmx_steps * -2
        elif (channel_1_value == 255 and channel_2_value == 255 ):
            return dmx_steps * 2


        #calculate coarse target with dmx channel 1
        if channel_1_value == 0:
            dmx_target = -dmx_steps
            
        elif channel_1_value == 127:
            dmx_target = 0
        elif channel_1_value == 255:
            dmx_target = dmx_steps
        else:
            dmx_target = round((channel_1_value - 127) / 127 * dmx_steps)
    
        #calculate fine target with dmx channel 2
        if channel_2_value > 127:
            dmx_target += round((channel_2_value - 127) / 127 * single_step)
        elif channel_2_value < 127:
            dmx_target -= round((-channel_2_value + 127) / 127 * single_step)
        
        return dmx_target
    
    def calculate_speed_value(self, channel_3_value , channel_4_value):
        speed = (channel_3_value / 255) + (channel_4_value / 255)
        return speed

    
    #implementing a auto move mode depending on a sensor
    def stepper_auto_mode(self, temp):
        print(temp)
        

class SPICommunicationThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.queue = queue.Queue()
        self.spi_device = None  # Placeholder for SPI device

    def run(self):
        while True:
            # Check if queue has any G-code commands
            if not self.queue.empty():
                gcode_command = self.queue.get()
                self.send_gcode(gcode_command)
            else:
                # Optionally implement some waiting mechanism here
                pass

    def send_gcode(self, gcode_command):
        # Placeholder function for sending G-code over SPI
        print("Sending G-code:", gcode_command)
        # Here you would implement the SPI communication to send the G-code command

    def add_to_queue(self, gcode_command):
        self.queue.put(gcode_command)


def create_controller_from_config(config_file):
    config = configparser.ConfigParser()
    
    try:
        config.read(config_file)
    except configparser.Error as e:
        print(f"Error reading configuration file: {e}")
        return None
    
    try:
        controller_config = config['client']
        
        name = controller_config.get('name')
        dmx_start_address = controller_config.getint('dmx_start_address')
        dmx_channel_mode = controller_config.getint('dmx_channel_mode')
        control_center_ip = controller_config.get('control_center_ip')
        control_center_port = controller_config.getint('control_center_port')
        
        controller = Controller(
            name=name,
            dmx_start_address=dmx_start_address,
            dmx_channel_mode=dmx_channel_mode,
            control_center_ip=control_center_ip,
            control_center_port=control_center_port
        )
        
        return controller
    except Exception as e:
        print(f"Error creating Controller object: {e}")
        return None


def create_steppers_from_config(config_file, controller):
    config = configparser.ConfigParser()
    
    try:
        config.read(config_file)
    except configparser.Error as e:
        print(f"Error reading configuration file: {e}")
        return []
    
    steppers = []
    
    for section_name in config.sections():
        if section_name.startswith('stepper_'):
            stepper_config = config[section_name]
            
            
            id_number = stepper_config.getint('id_number')
            step_pin = stepper_config.get('step_pin')
            dir_pin = stepper_config.get('dir_pin')
            enable_pin = stepper_config.get('enable_pin')
            stop_pin = stepper_config.get('stop_pin')
            
            step_invert = stepper_config.getboolean('step_invert', fallback=False)
            dir_invert = stepper_config.getboolean('dir_invert', fallback=False)
            
            try:
                max_speed = float(stepper_config.get('max_speed'))  # Convert to float
                max_acceleration = float(stepper_config.get('max_acceleration'))  # Convert to float
            except ValueError as e:
                print(f"Error converting max_speed or max_acceleration to float in section {section_name}: {e}")
                continue
            
            try:
                steps = stepper_config.getint('steps')
                micro_steps = stepper_config.getint('micro_steps')
                stepper_start_address = stepper_config.get('stepper_start_address')
                dmx_channels = stepper_config.getint('dmx_channels')
            except ValueError as e:
                print(f"Error converting steps, micro steps, or dmx_channels to int in section {section_name}: {e}")
                continue
            

            
            full_revolution_mode = stepper_config.getboolean('full_revolution_mode')
            UART_address = stepper_config.get('UART_address')
            
            # Create Stepper object and append to the list
            stepper = Stepper(
                name=section_name,
                id_number=id_number,                
                step_pin=step_pin,
                dir_pin=dir_pin,
                enable_pin=enable_pin,
                stop_pin=stop_pin,
                step_invert=step_invert,
                dir_invert=dir_invert,
                max_speed=max_speed,
                max_acceleration=max_acceleration,
                steps=steps,
                micro_steps=micro_steps,
                full_revolution_mode=full_revolution_mode,
                stepper_start_address=stepper_start_address,
                dmx_channel_mode=dmx_channels,
                controller = controller,
                UART_address=UART_address
            )
            
            # Increment the DMX start address for the next stepper
            
            
            steppers.append(stepper)
            print(f"{section_name} created")
            
    return steppers




if __name__ == "__main__":
    
    #create controller
    controller = create_controller_from_config('controller.cfg')
    
    #create steppers
    steppers = create_steppers_from_config('steppers.cfg' , controller)
    #parse stepper objects to control class
    controller.set_steppers(steppers)
    


