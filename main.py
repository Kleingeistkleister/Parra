import configparser
from controller import Controller
import time
from mcu import MCU
from stupidArtnet import StupidArtnetServer
import stepper

# Create a callback to handle data when received from artnet
def test_callback(data):
    """Test function to receive callback data."""
    # The received data is an array of the channels value (no headers)
    pass

class ConfigHandler:
    
    @staticmethod
    def read_config(file_path):
        config = configparser.ConfigParser()
        config.read(file_path)

        # Initialize default values
        part_id = None
        mcu_port = None
        mcu_baudrate = None
        steppers_config = []
        dmx_universe = None
        dmx_start_address = None
        dmx_channel_mode = None

        for section in config.sections():
            if section.startswith('stepper'):
                stepper_config = {
                    'stepper_id': config.get(section, 'stepper_id'),
                    'step_pin': config.get(section, 'step_pin'),
                    'dir_pin': config.get(section, 'dir_pin'),
                    'enable_pin': config.get(section, 'enable_pin'),
                    'uart_pin': config.get(section, 'uart_pin'),
                    'endstop_pin': config.get(section, 'endstop_pin'),
                    'steps_per_rotation': config.getint(section, 'steps_per_rotation'),
                    'max_speed': config.getint(section, 'max_speed'),
                    'acceleration': config.getint(section, 'acceleration'),
                    'microsteps': config.getint(section, 'microsteps'),
                    'uart_address': config.getint(section, 'uart_address'),
                    'stealthchop_threshold': config.getint(section, 'stealthchop_threshold'),
                    'dmx_start_address': config.getint(section, 'dmx_start_address')
                }
                steppers_config.append(stepper_config)

            elif section.startswith('dmx'):
                dmx_universe = config.getint(section, 'dmx_universe')
                dmx_start_address = config.getint(section, 'dmx_start_address')
                dmx_channel_mode = config.get(section, 'dmx_channel_mode')
                
            elif section.startswith('mcu'):
                mcu_port = config.get(section, 'port')
                mcu_baudrate = config.getint(section, 'baudrate')
                
            elif section.startswith('controller'):
                part_id = config.get(section, 'part_id')
                
        return part_id, mcu_port, mcu_baudrate, steppers_config, dmx_universe, dmx_start_address, dmx_channel_mode

if __name__ == "__main__":
    config_file_path = 'config.cfg'  # Provide the path to your configuration file
    
    # Read the configuration file
    part_id, mcu_port, mcu_baudrate, steppers_config, dmx_universe, dmx_start_address, dmx_channel_mode = ConfigHandler.read_config(config_file_path)

    # Create an MCU object
    mcu = MCU(mcu_port, mcu_baudrate)
    
    # Create a Controller object
    controller = Controller(part_id, mcu, steppers_config, dmx_universe, dmx_start_address, dmx_channel_mode)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()
