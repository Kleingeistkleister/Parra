class Stepper:
    def __init__(self, controller, mcu, stepper_id, oid, step_pin, dir_pin, enable_pin, steps_per_rotation, max_speed,
                 acceleration, microsteps, uart_pin, uart_diag , stealthchop_threshold, dmx_start_address, dmx_channel_mode):
        self.controller = controller
        self.stepper_id = stepper_id
        self.mcu = mcu
        self.oid = oid
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.uart_pin = uart_pin
        self.uart_diag = uart_diag
        
        self.microsteps = microsteps 
        self.steps_per_rotation = steps_per_rotation * microsteps
        self.max_speed = max_speed
        self.acceleration = acceleration
              
        self.stealthchop_threshold = stealthchop_threshold
  
        self.position = 0
        self.pos_direction = 1
        self.current_speed = 0.
       
        self.target = 0
        self.diretion = 1
        self.last_direction
        self.is_accel = False
        self.is_active = False
        
        self.next_sleep_time = 0
        self.next_step_time = 0
        self.next_step_interval = 0
        self.next_step_count = 0
        self.next_step_add = 0
        self.next_step_dir = 0
        self.next_step_clock = 0
        self.next_step_position = 0
        self.next_step_speed = 0
        

        self.last_step_time = 0
        self.last_step_interval = 0
        self.last_step_count = 0
        self.last_step_add = 0
        self.last_step_dir = 0
        self.last_step_clock = 0
        self.last_step_position = 0
        self.last_step_speed = 0
        
        

        self.dmx_channel_values = []
        self.add_dmx_channels(dmx_start_address, dmx_channel_mode)


    def add_dmx_channels(self, start_address, dmx_channel_mode):
        # Add the DMX channels for this stepper
        for i in range(dmx_channel_mode):
            self.dmx_channel_values.append(start_address + i)
            
    def get_dmx_value(self, i):
        return self.dmx_channel_values[i]
    def set_dmx_value(self, i, value):
        self.dmx_channel_values[i] = value        
    
    def set_position(self, position):
        self.position = position
        
    def calculate_target_dmx(self, target):
        
            coarse_target = self.dmx_channel_values[0]
            fine_target = self.dmx_channel_values[1]

            target = coarse_target + (fine_target / 255)
           
            if target == 0:
               target = self.steps_per_rotation * 2
            elif target == coarse_target * fine_target:
                target = self.steps_per_rotation * -2
                
            print(f"Stepper {self.stepper_id} target: {target}")
            self.target = target
                            

    def get_max_speed_dmx(self, stepper):
            self.max_speed = stepper.get_dmx_value(2)
          

    def calculate_acceleration_dmx(self,stepper ):        
            stepper.acceleration = stepper.get_dmx_value(3)
            return stepper.acceleration
    
    #set the direction of the stepper
    def next_direction(self, steps_to_move):
        if steps_to_move == 0:
           return False
        # update last step direction 
        self.last_step_dir = self.next_step_dir
        # Determine the direction to move
        if steps_to_move > 0:
            if self.last_step_dir != 1:
                self.pos_direction = 1
                self.next_step_dir = 1
        elif steps_to_move < 0:
            if self.last_step_dir != 0:
                self.pos_direction = -1
                self.next_step_dir = 0
                
        #check if direction pin setup has to be changed.        
        if self.next_step_dir == self.last_step_dir:
            return False
        
        return True
                

    def calculate_next_step(self):
        target = self.target
        steps_to_move = target - self.position
        
        #if no steps to move return
        if steps_to_move == 0:
            return
        # If direction has changed, set the direction pin
        if self.next_direction(steps_to_move) == True:
            self.controller.fast_cmd_queue.put(self.set_next_step_dir(self.next_step_dir))
        
        

        # Calculate the next cycle time for mcu_pwm
        total_distance = abs(steps_to_move)
        acceleration_distance = min(total_distance * 0.05, 100)
        deceleration_distance = min(total_distance * 0.05, 100)
        constant_speed_distance = total_distance - acceleration_distance - deceleration_distance

        # Calculate the next cycle time for mcu_pwm based on acceleration, constant speed, and deceleration
        if self.is_accel:
            if self.position < acceleration_distance:
                next_cycle_time = self.calculate_acceleration_time(self.position, acceleration_distance)
            elif self.position < acceleration_distance + constant_speed_distance:
                next_cycle_time = self.calculate_constant_speed_time(constant_speed_distance)
            else:
                next_cycle_time = self.calculate_deceleration_time(self.position - acceleration_distance - constant_speed_distance, deceleration_distance)
        else:
            next_cycle_time = self.calculate_constant_speed_time(total_distance)

             
            
        # Queue the next step in the controller move_cmd_queue.
        self.controller.move_cmd_queue.put(self.create_step(next_cycle_time))
        #update the position of the controller stepper
        self.position = self.position + self.pos_direction

    #Command to queue a step
    def create_step(self, interval):
         return f"queue_step oid={self.oid} interval={interval} count=1 add=0"    

    #Command to set direction pin  
    def set_next_step_dir(self, direction):      
       return f"set_next_step_dir oid={self.oid} dir={direction}"     

    #Command to get current position of mcu stepper
    def mcu_stepper_get_position(self):
        # Format and send the stepper_get_position command to the micro-controller
        return f"stepper_get_position oid={self.oid}"
        













    ###########################
    ##### Getter methods
    ###########################    
    def get_mcu(self):
        return self.mcu

    def get_oid(self):
        return self.oid

    def get_step_pin(self):
        return self.step_pin

    def get_dir_pin(self):
        return self.dir_pin

    def get_enable_pin(self):
        return self.enable_pin

    def get_steps_per_rotation(self):
        return self.steps_per_rotation

    def get_max_speed(self):
        return self.max_speed

    def get_acceleration(self):
        return self.acceleration

    def get_microsteps(self):
        return self.microsteps

    def get_stealthchop_threshold(self):
        return self.stealthchop_threshold

    def get_position(self):
        return self.position

    def get_current_speed(self):
        return self.current_speed

    def get_direction(self):
        return self.direction

    ################ Setter methods###########
    def set_mcu(self, mcu):
        self.mcu = mcu

    def set_oid(self, oid):
        self.oid = oid

    def set_step_pin(self, step_pin):
        self.step_pin = step_pin

    def set_dir_pin(self, dir_pin):
        self.dir_pin = dir_pin

    def set_enable_pin(self, enable_pin):
        self.enable_pin = enable_pin

    def set_steps_per_rotation(self, steps_per_rotation):
        self.steps_per_rotation = steps_per_rotation

    def set_max_speed(self, max_speed):
        self.max_speed = max_speed

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration

    def set_microsteps(self, microsteps):
        self.microsteps = microsteps

    def set_stealthchop_threshold(self, stealthchop_threshold):
        self.stealthchop_threshold = stealthchop_threshold

    def set_position(self, position):
        self.position = position

    def set_current_speed(self, current_speed):
        self.current_speed = current_speed

    def set_direction(self, direction):
        self.direction = direction
