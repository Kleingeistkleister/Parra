import configparser
from controller import Controller
import time
from mcu import MCU
from stupidArtnet import StupidArtnetServer
import stepper
import config

        
    
# create a callback to handle data when received from artnet
def test_callback(data):
    """Test function to receive callback data."""
    # the received data is an array
    # of the channels value (no headers)
    pass


if __name__ == "__main__":
    config_file_path = 'config.cfg'  # Provide the path to your configuration file
    
    # Read the configuration file
    part_id, mcu_port, mcu_baudrate, steppers_config, dmx_universe, dmx_start_address,dmx_channel_mode = config.ConfigReader.read_config(config_file_path)

    # Create an MCU object
    mcu = MCU(mcu_port, mcu_baudrate)
    
    # Create a Controller object
    controller = Controller(part_id, mcu, steppers_config,dmx_universe, dmx_start_address, dmx_channel_mode)
    
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop()
        
