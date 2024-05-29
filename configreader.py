from configparser import ConfigParser

class ConfigParser:
    
    def read_config(file_path):
        config = configparser.ConfigParser()
        config.read(file_path)

        # Reading controller configuration
        
        # Reading stepper configurations
        steppers_config = []
        

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

            if section.startswith('dmx'):
                dmx_universe = config.getint(section, 'dmx_universe')
                dmx_start_address = config.getint(section, 'dmx_start_address')
                dmx_channel_mode = config.get(section, 'dmx_channel_mode')
                
            if section.startswith('mcu'):
                mcu_port = config.get(section, 'port')
                mcu_baudrate = config.getint(section, 'baudrate')
                
            if section.startswith('controller'):
                part_id = config.get(section, 'part_id')
                
            return part_id, mcu_port, mcu_baudrate, steppers_config, dmx_universe, dmx_start_address, dmx_channel_mode
