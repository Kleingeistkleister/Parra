# Printer stepper support
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import chelper

class error(Exception):
    pass


######################################################################
# Steppers
######################################################################

MIN_BOTH_EDGE_DURATION = 0.000000200

# Interface to low-level mcu and chelper code
class MCU_stepper:
    def __init__(self, name, step_pin_params, dir_pin_params,
                 rotation_dist, steps_per_rotation,
                 step_pulse_duration=None, units_in_radians=False):
        self._name = name
        self._rotation_dist = rotation_dist
        self._steps_per_rotation = steps_per_rotation
        self._step_pulse_duration = step_pulse_duration
        self._units_in_radians = units_in_radians
        self._step_dist = rotation_dist / steps_per_rotation
        self._mcu = step_pin_params['chip']
        self._oid = oid = self._mcu.create_oid()
        self._mcu.register_config_callback(self._build_config)
        self._step_pin = step_pin_params['pin']
        self._invert_step = step_pin_params['invert']
        if dir_pin_params['chip'] is not self._mcu:
            raise self._mcu.get_printer().config_error(
                "Stepper dir pin must be on same mcu as step pin")
        self._dir_pin = dir_pin_params['pin']
        self._invert_dir = self._orig_invert_dir = dir_pin_params['invert']
        self._step_both_edge = self._req_step_both_edge = False
        self._mcu_position_offset = 0.
        self._reset_cmd_tag = self._get_position_cmd = None
        self._active_callbacks = []
        ffi_main, ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(ffi_lib.stepcompress_alloc(oid), ffi_lib.stepcompress_free)
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, self._invert_dir)
        self._mcu.register_stepqueue(self._stepqueue)
        self._stepper_kinematics = None
        self._itersolve_generate_steps = ffi_lib.itersolve_generate_steps
        self._itersolve_check_active = ffi_lib.itersolve_check_active
        self._trapq = ffi_main.NULL
        self._mcu.get_printer().register_event_handler('klippy:connect', self._query_mcu_position)
        
        self.max_speed = 300
        self.max_acceleration = 300
       


    def get_mcu(self):
        return self._mcu
    
    def get_name(self, short=False):
        if short and self._name.startswith('stepper_'):
            return self._name[8:]
        return self._name
    
    def units_in_radians(self):
        # Returns true if distances are in radians instead of millimeters
        return self._units_in_radians
    
    def get_pulse_duration(self):
        return self._step_pulse_duration, self._step_both_edge
    
    def setup_default_pulse_duration(self, pulse_duration, step_both_edge):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = pulse_duration
        self._req_step_both_edge = step_both_edge
        
    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)
        
    def _build_config(self):
        if self._step_pulse_duration is None:
            self._step_pulse_duration = .000002
        invert_step = self._invert_step
        sbe = int(self._mcu.get_constants().get('STEPPER_BOTH_EDGE', '0'))
        if (self._req_step_both_edge and sbe
            and self._step_pulse_duration <= MIN_BOTH_EDGE_DURATION):
            # Enable stepper optimized step on both edges
            self._step_both_edge = True
            self._step_pulse_duration = 0.
            invert_step = -1
        step_pulse_ticks = self._mcu.seconds_to_clock(self._step_pulse_duration)
        self._mcu.add_config_cmd(
            "config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d"
            " step_pulse_ticks=%u" % (self._oid, self._step_pin, self._dir_pin,
                                      invert_step, step_pulse_ticks))
        self._mcu.add_config_cmd("reset_step_clock oid=%d clock=0"
                                 % (self._oid,), on_restart=True)
        step_cmd_tag = self._mcu.lookup_command(
            "queue_step oid=%c interval=%u count=%hu add=%hi").get_command_tag()
        dir_cmd_tag = self._mcu.lookup_command(
            "set_next_step_dir oid=%c dir=%c").get_command_tag()
        self._reset_cmd_tag = self._mcu.lookup_command(
            "reset_step_clock oid=%c clock=%u").get_command_tag()
        self._get_position_cmd = self._mcu.lookup_query_command(
            "stepper_get_position oid=%c",
            "stepper_position oid=%c pos=%i", oid=self._oid)
        max_error = self._mcu.get_max_stepper_error()
        max_error_ticks = self._mcu.seconds_to_clock(max_error)
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_fill(self._stepqueue, max_error_ticks,
                                  step_cmd_tag, dir_cmd_tag)
    def get_oid(self):
        return self._oid
    
    def get_step_dist(self):
        return self._step_dist
    def get_rotation_distance(self):
        return self._rotation_dist, self._steps_per_rotation
    def set_rotation_distance(self, rotation_dist):
        mcu_pos = self.get_mcu_position()
        self._rotation_dist = rotation_dist
        self._step_dist = rotation_dist / self._steps_per_rotation
        self.set_stepper_kinematics(self._stepper_kinematics)
        self._set_mcu_position(mcu_pos)
    def get_dir_inverted(self):
        return self._invert_dir, self._orig_invert_dir
    
    def set_dir_inverted(self, invert_dir):
        invert_dir = not not invert_dir
        if invert_dir == self._invert_dir:
            return
        self._invert_dir = invert_dir
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.stepcompress_set_invert_sdir(self._stepqueue, invert_dir)
        self._mcu.get_printer().send_event("stepper:set_dir_inverted", self)
        
    def calc_position_from_coord(self, coord):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    
    def set_position(self, coord):
        mcu_pos = self.get_mcu_position()
        sk = self._stepper_kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_position(sk, coord[0], coord[1], coord[2])
        self._set_mcu_position(mcu_pos)
        
    def get_commanded_position(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        return ffi_lib.itersolve_get_commanded_pos(self._stepper_kinematics)
    
    def get_mcu_position(self):
        mcu_pos_dist = self.get_commanded_position() + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    
    def _set_mcu_position(self, mcu_pos):
        mcu_pos_dist = mcu_pos * self._step_dist
        self._mcu_position_offset = mcu_pos_dist - self.get_commanded_position()
        
    def get_past_mcu_position(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        pos = ffi_lib.stepcompress_find_past_position(self._stepqueue, clock)
        return int(pos)
    
    def mcu_to_commanded_position(self, mcu_pos):
        return mcu_pos * self._step_dist - self._mcu_position_offset
    
    def dump_steps(self, count, start_clock, end_clock):
        ffi_main, ffi_lib = chelper.get_ffi()
        data = ffi_main.new('struct pull_history_steps[]', count)
        count = ffi_lib.stepcompress_extract_old(self._stepqueue, data, count, start_clock, end_clock)
        return (data, count)
    
    def get_stepper_kinematics(self):
        return self._stepper_kinematics
    
    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        mcu_pos = 0
        if old_sk is not None:
            mcu_pos = self.get_mcu_position()
        self._stepper_kinematics = sk
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.itersolve_set_stepcompress(sk, self._stepqueue, self._step_dist)
        self.set_trapq(self._trapq)
        self._set_mcu_position(mcu_pos)
        return old_sk
    
    def note_homing_end(self):
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_reset(self._stepqueue, 0)
        if ret:
            raise error("Internal error in stepcompress")
        data = (self._reset_cmd_tag, self._oid, 0)
        ret = ffi_lib.stepcompress_queue_msg(self._stepqueue, data, len(data))
        if ret:
            raise error("Internal error in stepcompress")
        self._query_mcu_position()
        
    def _query_mcu_position(self):
        if self._mcu.is_fileoutput():
            return
        params = self._get_position_cmd.send([self._oid])
        last_pos = params['pos']
        if self._invert_dir:
            last_pos = -last_pos
        print_time = self._mcu.estimated_print_time(params['#receive_time'])
        clock = self._mcu.print_time_to_clock(print_time)
        ffi_main, ffi_lib = chelper.get_ffi()
        ret = ffi_lib.stepcompress_set_last_position(self._stepqueue, clock,
                                                     last_pos)
        if ret:
            raise error("Internal error in stepcompress")
        self._set_mcu_position(last_pos)
        self._mcu.get_printer().send_event("stepper:sync_mcu_position", self)
        
    def get_trapq(self):
        return self._trapq
    def set_trapq(self, tq):
        ffi_main, ffi_lib = chelper.get_ffi()
        if tq is None:
            tq = ffi_main.NULL
        ffi_lib.itersolve_set_trapq(self._stepper_kinematics, tq)
        old_tq = self._trapq
        self._trapq = tq
        return old_tq
    
    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)
        
    def generate_steps(self, flush_time):
        # Check for activity if necessary
        if self._active_callbacks:
            sk = self._stepper_kinematics
            ret = self._itersolve_check_active(sk, flush_time)
            if ret:
                cbs = self._active_callbacks
                self._active_callbacks = []
                for cb in cbs:
                    cb(ret)
        # Generate steps
        sk = self._stepper_kinematics
        ret = self._itersolve_generate_steps(sk, flush_time)
        if ret:
            raise error("Internal error in stepcompress")
        
    def is_active_axis(self, axis):
        ffi_main, ffi_lib = chelper.get_ffi()
        a = axis.encode()
        return ffi_lib.itersolve_is_active_axis(self._stepper_kinematics, a)
    
    def set_max_speed(self, speed):
        self.max_speed = speed
        
    def set_max_acceleration(self, acceleration):
        self.max_acceleration = acceleration


