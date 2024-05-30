import queue
from mcu import MCU
import stepper
import stupidArtnet

class Controller:
    def __init__(self, part_id, mcu, steppers_config, dmx_universe, dmx_start_address, dmx_channel_mode):
        self.oid = 1
        self.objects = {}
        self.id = part_id
        self.mcu = mcu
        self.dmx_universe = dmx_universe
        self.dmx_start_address = dmx_start_address
        self.dmx_channel_mode = dmx_channel_mode
        self.dmx_debug_channel = 2
        self.steppers = []
        self.endstops = []
        self.dmx_values_old = []
        self.dmx_values_new = []
        self.dmx_cycles = 100

        for config in steppers_config:
            stepper_inst = stepper.Stepper(
                controller=self,
                oid=self.alloc_oid(),
                mcu=self.mcu,
                stepper_id=config['stepper_id'],
                step_pin=config['step_pin'],
                dir_pin=config['dir_pin'],
                enable_pin=config['enable_pin'],
                endstop_pin=config['endstop_pin'],
                uart_pin=config['uart_pin'],
                steps_per_rotation=config['steps_per_rotation'],
                max_speed=config['max_speed'],
                acceleration=config['acceleration'],
                microsteps=config['microsteps'],
                uart_address=config['uart_address'],
                stealthchop_threshold=config['stealthchop_threshold'],
                dmx_start_address=config['dmx_start_address']
            )
            self.steppers.append(stepper_inst)
            self.mcu.add_stepper(stepper_inst)

        self.artnet_handler = stupidArtnet.StupidArtnetServer()
        self.artnet = self.artnet_handler.register_listener(dmx_universe, callback_function=self.artnet_callback)

        self.move_cmd_queue = queue.Queue()
        self.fast_cmd_queue = queue.Queue()

    def register_callbacks(self):
        self.mcu.register_callback('endstop_triggered', self.endstop_triggered)

    def endstop_triggered(self, data):
        oid = data['oid']
        if oid in self.objects:
            stepper = self.steppers[oid - len(self.steppers)]
            stepper.set_position(0)
        else:
            print(f"Stepper with OID {oid} not found.")
        self.objects[oid].endstop_triggered(data)

    def alloc_oid(self):
        next_oid = self.oid
        self.oid += 1
        return next_oid

    def register_object(self, obj):
        oid = self.alloc_oid()
        self.objects[oid] = obj
        return oid

    def setup_steppers(self):
        for stepper in self.steppers:
            self.fast_cmd_queue.put(stepper.setup())
            self.fast_cmd_queue.put(stepper.setup_endstop(self.alloc_oid()))

    def setup_endstops(self):
        for stepper in self.steppers:
            self.fast_cmd_queue.put(stepper.setup_endstop(self.alloc_oid()))

    def artnet_callback(data):
        pass

    def read_dmx(self, dmx_start_address):
        buffer = self.artnet.get_buffer(self.artnet)
        if buffer:
            self.dmx_values_old = self.dmx_values_new
            self.dmx_values_new = buffer[dmx_start_address:(dmx_start_address + (len(self.steppers) * self.dmx_channel_mode) + self.dmx_debug_channel)]
        else:
            self.manual_mode()
            print('No DMX values received')

    def set_dmx_values(self, stepper, channel, value):
        stepper.set_dmx_value(channel, value)

    def set_dmx(self):
        for i, stepper in enumerate(self.steppers):
            for j in range(self.dmx_channel_mode):
                self.set_dmx_values(stepper, j, self.dmx_values_new[i * self.dmx_channel_mode + j])

    def manual_mode(self):
        pass

    def send_mcu_config(self):
        for stepper in self.steppers:
            stepper.send_config()

    def run(self):
        self.mcu.connect()
        self.setup_steppers()
        self.setup_endstops()
        while not self.fast_cmd_queue.empty():
            self.mcu.send_command(self.fast_cmd_queue.get())

        while True:
            if self.dmx_cycles == 100:
                self.read_dmx(self.dmx_start_address)
                self.set_dmx()
                self.dmx_cycles = 0

            for i, stepper in enumerate(self.steppers):
                stepper.calculate_target_dmx(self.dmx_values_old[i * self.dmx_channel_mode + self.dmx_channel_mode - 1])

            self.dmx_cycles += 1

            while not self.fast_cmd_queue.empty():
                self.mcu.send_command(self.fast_cmd_queue.get())

            if not self.move_cmd_queue.empty():
                self.mcu.send_command(self.move_cmd_queue.get())

    def stop(self):
        self.artnet_handler.stop()
        self.mcu.disconnect()
