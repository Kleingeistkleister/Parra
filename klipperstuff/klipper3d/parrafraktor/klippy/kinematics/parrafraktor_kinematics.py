# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, importlib
import mcu, chelper, kinematics.extruder , stepper




class ParrafraktorKinematics:
    def __init__(self, parrafraktor_part , config):
        self.printer = config.get_printer()
        # Initialisieren Sie die sechs Steppermotoren      
        self.steppers = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyzabc']
      
        self.dmx_start_address = 1
        self.dmx_channel_mode = 4
        self.dmx_values_new = []
        self.dmx_values_old = []
        
        self.steppers = []      
        self.stepper_ammount = range(self.steppers)  
        
        # Set the stepper motor parameters
        self.max_speed = [100, 100, 100, 100, 100, 100]
        self.current_speed = [0, 0, 0, 0, 0, 0]
        self.max_acceleration = [0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
        self.break_speed    = [100, 100, 100, 100, 100, 100]
        
        self.positons = [0, 0, 0, 0, 0, 0]
        self.start_position = [0, 0, 0, 0, 0, 0]
        self.position_target = [100, 100, 100, 100, 100, 100]
        self.last_target = [0, 0, 0, 0, 0, 0]
        self.axis = ['X', 'Y', 'Z', 'A', 'B', 'C']        
        
 
        # Create the move code queue
        self.move_code_queue = queue.Queue()

        def run_dmx(self):
            while 1:
                self.readDmx(self.dmx_start_address)
                self.process_dmx_values()
                self.create_moves()
            

    # create the next move for every stepper
        def create_moves(self):
            for i in range(self.stepper_ammount):
                self.move_code_queue.append(self.create_move(i))
            


        def create_move(self, stepper):
            position = self.position_target[stepper]
            target = self.position_target[stepper]
            max_speed = self.max_speed[stepper]
            max_acceleration = self.max_acceleration[stepper]            
            current_speed = self.current_speed[stepper]
            axis = self.axis[stepper]
        
            # Calculate distance to move
            distance = target - position
            # Calculate acceleration and deceleration distances
            acceleration_distance = abs(distance) * 0.05  # 5% of total distance for acceleration
            deceleration_distance = acceleration_distance  # Deceleration equals acceleration
    
            # Check if movement is needed
            if target == position:
                return ""  # No movement needed
    
            # Determine movement direction
            direction = 1 if target > position else -1
    
            # Calculate the speed for the next step by acceleration
            if current_speed < max_speed:
                current_speed += max_acceleration
            if current_speed > max_speed:
                    current_speed = max_speed
            else:
                current_speed -= max_acceleration
                if current_speed < 0:
                    current_speed = 0
    
            # Calculate the distance to move
            distance_to_move = min(abs(distance), current_speed)
            # Calculate the new position
            position += distance_to_move * direction
    
            # Set the new position and speed
            self.position_target[stepper] = position
            self.current_speed[stepper] = current_speed
    
            # Create the G-Code for this stepper motor
            move_code = "G1 {}{} Speed{}\n".format(axis, position, current_speed)
    
            return move_code
        

    

########################################################        DMX                ########################################################

   
    def process_dmx_values(self):
        channel = 0
        for i in range(self.stepper_ammount):
        # get dmx values for each stepper motor
            channel_a_value = self.dmx_values_new[i + channel + 1]
            channel_b_value = self.dmx_values_new[i + channel + 2]
            channel_c_value = self.dmx_values_new[i + channel + 3]
            channel_d_value = self.dmx_values_new[i + channel + 4]
            channel += self.dmx_channel_mode 
        # set the new position and max_speed for the stepper motor
            self.position_target[i] = self.dmx_value_to_pos(channel_a_value, channel_b_value)
            self.max_speed[i] = self.dmx_value_to_speed(channel_c_value)
    
    # Get DMX values from artnet listener
    # The 4 channel mode provides 4 channels for each stepper object. We use 6 stepper drivers. The last 2 DMX channels are for further developme or debugging    
    def readDmx(self , start_address):
        self.dmx_values_old = self.dmx_values_new
        buffer = artnet.get_buffer(artnet_listener)   
        if buffer:
            self.dmx_values_new = buffer[self.dmx_start_address : (self.dmx_start_address + ( self.stepper_ammount * self.dmx_channel_mode) + 2)]
  
    def dmx_to_val(self , value, input_min  , input_max , output_min , output_max):       
    # Scale the value to the new range
        scaled_value = ((value - input_min) / (input_max - input_min)) * (output_max - output_min) + output_min
        return int(scaled_value)
    
    def dmx_value_to_pos(self, channel_a_value , channel_b_value):
        position_coarse = self.dmx_to_val(channel_a_value , 0 , 255 , 0 , 100)
        position_fine = self.dmx_to_val(channel_b_value , 0 , 255 , 0 , 100)
        position = position_coarse + position_fine
        return position
   
    
    def dmx_value_to_speed(self, channel_c_value):
        speed = self.dmx_to_val(channel_c_value , 0 , 255 , 0 , 100)
        return speed
        

    def process_dmx_input(self, dmx_values):
        # Parse DMX values to extract position, speed, and acceleration
        position = (dmx_values[0] << 8) | dmx_values[1]  # Combine two 8-bit values into a 16-bit position
        max_speed = dmx_values[2]
        max_acceleration = dmx_values[3]

        # Update maximum speed and acceleration
        self.set_max_speed(max_speed)
        self.set_max_acceleration(max_acceleration)

        # Return the target position
        return position    

    def calc_position(self, stepper_positions):
        return [0, 0, 0, 0, 0, 0]

    def set_position(self, newpos, homing_axes):
        
        pass
   

    def get_status(self, eventtime):
        
        return {
            'homed_axes': '',
            'axis_minimum': self.axes_minmax,
            'axis_maximum': self.axes_minmax,
        }

def load_kinematics(parrafraktor_part , config):    
    return ParrafraktorKinematics(parrafraktor_part , config)




###########################################################################################################################################




class CartKinematics:
    def __init__(self, toolhead, config):
        
        self.printer = config.get_printer()
        # Setup axis rails
        self.dual_carriage_axis = None
        self.dual_carriage_rails = []
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in 'xyz']
        

        for rail, axis in zip(self.rails, 'xyz'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis.encode())
            
        ranges = [r.get_range() for r in self.rails]
        
        self.axes_min = toolhead.Coord(*[r[0] for r in ranges], e=0.)
        self.axes_max = toolhead.Coord(*[r[1] for r in ranges], e=0.)
        self.dc_module = None
        
       
            
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        self.printer.register_event_handler("stepper_enable:motor_off",
                                            self._motor_off)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        
        self.max_z_velocity = config.getfloat('max_z_velocity', max_velocity,
                                              above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', max_accel,
                                           above=0., maxval=max_accel)
        self.limits = [(1.0, -1.0)] * 3
        

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]   

    def calc_position(self, stepper_positions):
        return [stepper_positions[rail.get_name()] for rail in self.rails]

    def update_limits(self, i, range):
        l, h = self.limits[i]
        # Only update limits if this axis was already homed,
        # otherwise leave in un-homed state.
        if l <= h:
            self.limits[i] = range
    
    # Override the rail at index i with a new rail
    # This is used to replace a rail with a new one
    # that has been homed
    def override_rail(self, i, rail):
        self.rails[i] = rail
        

          
    def set_position(self, newpos, homing_axes):
        # Set the position of the rails
        # If the axis is being homed, update the limits
        # to reflect the new position
        # This is used to set the position of the rails
        # after homing
 
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
                
    def note_z_not_homed(self):
        # Helper for Safe Z Home
        self.limits[2] = (1.0, -1.0)
        
    def home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)
        
        
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            if self.dc_module is not None and axis == self.dual_carriage_axis:
                self.dc_module.home(homing_state)
            else:
                self.home_axis(homing_state, axis, self.rails[axis])
                
    def _motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
        

    # Check if the move is within the limits of the printer
    # to prevent the printer from moving outside its limits
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise move.move_error("Must home axis first")
                raise move.move_error()
            
    # Check if the move is within the limits of the printer
    # to prevent the printer from moving outside its limits
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
        


    def get_status(self, eventtime):        
        axes = [a for a, (l, h) in zip("xyz", self.limits) if l <= h]
        return {
            'homed_axes': "".join(axes),
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
        }

def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)
