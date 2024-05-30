# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, importlib
import mcu, chelper, kinematics.extruder , kinematics 
from stupidArtnet import StupidArtnetServer

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = start_pos
        self.end_pos = end_pos
        self.accel = toolhead.max_accel
        self.junction_deviation = toolhead.junction_deviation
        self.timing_callbacks = []
        self.is_kinematic_move = True
        self.move_d = abs(end_pos - start_pos)
        if self.move_d < .000000001:
            self.is_kinematic_move = False
        else:
            self.velocity = min(speed, toolhead.max_velocity)
        self.min_move_t = self.move_d / self.velocity if self.velocity != 0 else 0
        self.max_start_v2 = 0.
        self.max_cruise_v2 = self.velocity**2 if self.velocity != 0 else 0
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * self.move_d * toolhead.max_accel_to_decel

    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed if speed != 0 else 0
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)

    def move_error(self, msg="Move out of range"):
        pass

    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        junction_cos_theta = 1.0  # Simplified for single stepper
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos_theta))
        R_jd = sin_theta_d2 / (1. - sin_theta_d2)
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5*(1.0+junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        self.max_start_v2 = min(
            R_jd * self.junction_deviation * self.accel,
            R_jd * prev_move.junction_deviation * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(
            self.max_start_v2
            , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)

    def set_junction(self, start_v2, cruise_v2, end_v2):
        half_inv_accel = .5 / self.accel
        accel_d = (cruise_v2 - start_v2) * half_inv_accel
        decel_d = (cruise_v2 - end_v2) * half_inv_accel
        cruise_d = self.move_d - accel_d - decel_d
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        self.accel_t = accel_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_d / cruise_v if cruise_v != 0 else 0
        self.decel_t = decel_d / ((end_v + cruise_v) * 0.5)

LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class LookAheadQueue:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.queue = []
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def get_last(self):
        if self.queue:
            return self.queue[-1]
        return None
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count-1, -1, -1):
            move = queue[i]
            reachable_start_v2 = next_end_v2 + move.delta_v2
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (
                        smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            mc_v2 = peak_cruise_v2
                            for m, ms_v2, me_v2 in reversed(delayed):
                                mc_v2 = min(mc_v2, ms_v2)
                                m.set_junction(min(ms_v2, mc_v2), mc_v2
                                               , min(me_v2, mc_v2))
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    move.set_junction(min(start_v2, cruise_v2), cruise_v2
                                      , min(next_end_v2, cruise_v2))
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count or not flush_count:
            return
        # Generate step times for all moves ready to be flushed
        self.toolhead._process_moves(queue[:flush_count])
        # Remove processed moves from the queue
        del queue[:flush_count]
    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)

BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
BGFLUSH_LOW_TIME = 0.200
BGFLUSH_BATCH_TIME = 0.200
BGFLUSH_EXTRA_TIME = 0.250
MIN_KIN_TIME = 0.100
MOVE_BATCH_TIME = 0.500
STEPCOMPRESS_FLUSH_TIME = 0.050
SDS_CHECK_TIME = 0.001 # step+dir+step filter in stepcompress.c
MOVE_HISTORY_EXPIRE = 30.

DRIP_SEGMENT_TIME = 0.050
DRIP_TIME = 0.100
class DripModeEndSignal(Exception):
    pass

# Main code to track events (and their timing) on the printer toolhead
class ToolHeadOld:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.lookahead = LookAheadQueue(self)
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.commanded_pos = [0., 0., 0., 0.]
        # Velocity and acceleration control
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        min_cruise_ratio = 0.5
        if config.getfloat('minimum_cruise_ratio', None) is None:
            req_accel_to_decel = config.getfloat('max_accel_to_decel', None,
                                                 above=0.)
            if req_accel_to_decel is not None:
                config.deprecate('max_accel_to_decel')
                min_cruise_ratio = 1. - min(1., (req_accel_to_decel
                                                 / self.max_accel))
        self.min_cruise_ratio = config.getfloat('minimum_cruise_ratio',
                                                min_cruise_ratio,
                                                below=1., minval=0.)
        self.square_corner_velocity = config.getfloat(
            'square_corner_velocity', 5., minval=0.)
        self.junction_deviation = self.max_accel_to_decel = 0.
        self._calc_junction_deviation()
        # Input stall detection
        self.check_stall_time = 0.
        self.print_stall = 0
        # Input pause tracking
        self.can_pause = True
        if self.mcu.is_fileoutput():
            self.can_pause = False
        self.need_check_pause = -1.
        # Print time tracking
        self.print_time = 0.
        self.special_queuing_state = "NeedPrime"
        self.priming_timer = None
        self.drip_completion = None
        # Flush tracking
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.do_kick_flush_timer = True
        self.last_flush_time = self.min_restart_time = 0.
        self.need_flush_time = self.step_gen_time = self.clear_history_time = 0.
        # Kinematic step generation scan window time tracking
        self.kin_flush_delay = SDS_CHECK_TIME
        self.kin_flush_times = []
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.step_generators = []
        # Create kinematics class
        gcode = self.printer.lookup_object('gcode')
        self.Coord = gcode.Coord
        self.extruder = kinematics.extruder.DummyExtruder(self.printer)
        kin_name = config.get('kinematics')
        try:
            mod = importlib.import_module('kinematics.' + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)
        # Register commands
        gcode.register_command('G4', self.cmd_G4)
        gcode.register_command('M400', self.cmd_M400)
        gcode.register_command('SET_VELOCITY_LIMIT',
                               self.cmd_SET_VELOCITY_LIMIT,
                               desc=self.cmd_SET_VELOCITY_LIMIT_help)
        gcode.register_command('M204', self.cmd_M204)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        # Load some default modules
        modules = ["gcode_move", "homing", "idle_timeout", "statistics",
                   "manual_probe", "tuning_tower"]
        for module_name in modules:
            self.printer.load_object(config, module_name)
            
    # Print time and flush tracking
    def _advance_flush_time(self, flush_time):
        flush_time = max(flush_time, self.last_flush_time)
        # Generate steps via itersolve
        sg_flush_want = min(flush_time + STEPCOMPRESS_FLUSH_TIME,
                            self.print_time - self.kin_flush_delay)
        sg_flush_time = max(sg_flush_want, flush_time)
        for sg in self.step_generators:
            sg(sg_flush_time)
        self.min_restart_time = max(self.min_restart_time, sg_flush_time)
        # Free trapq entries that are no longer needed
        clear_history_time = self.clear_history_time
        if not self.can_pause:
            clear_history_time = flush_time - MOVE_HISTORY_EXPIRE
        free_time = sg_flush_time - self.kin_flush_delay
        self.trapq_finalize_moves(self.trapq, free_time, clear_history_time)
        self.extruder.update_move_time(free_time, clear_history_time)
        # Flush stepcompress and mcu steppersync
        for m in self.all_mcus:
            m.flush_moves(flush_time, clear_history_time)
        self.last_flush_time = flush_time
        
    def _advance_move_time(self, next_print_time):
        pt_delay = self.kin_flush_delay + STEPCOMPRESS_FLUSH_TIME
        flush_time = max(self.last_flush_time, self.print_time - pt_delay)
        self.print_time = max(self.print_time, next_print_time)
        want_flush_time = max(flush_time, self.print_time - pt_delay)
        while 1:
            flush_time = min(flush_time + MOVE_BATCH_TIME, want_flush_time)
            self._advance_flush_time(flush_time)
            if flush_time >= want_flush_time:
                break
            
    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        kin_time = max(est_print_time + MIN_KIN_TIME, self.min_restart_time)
        kin_time += self.kin_flush_delay
        min_print_time = max(est_print_time + BUFFER_TIME_START, kin_time)
        if min_print_time > self.print_time:
            self.print_time = min_print_time
            self.printer.send_event("toolhead:sync_print_time",
                                    curtime, est_print_time, self.print_time)
            
    def _process_moves(self, moves):
        # Resync print_time if necessary
        if self.special_queuing_state:
            if self.special_queuing_state != "Drip":
                # Transition from "NeedPrime"/"Priming" state to main state
                self.special_queuing_state = ""
                self.need_check_pause = -1.
            self._calc_print_time()
        # Queue moves into trapezoid motion queue (trapq)
        next_move_time = self.print_time
        
        for move in moves:
            if move.is_kinematic_move:
                self.trapq_append(
                    self.trapq, next_move_time,
                    move.accel_t, move.cruise_t, move.decel_t,
                    move.start_pos[0], move.start_pos[1], move.start_pos[2],
                    move.axes_r[0], move.axes_r[1], move.axes_r[2],
                    move.start_v, move.cruise_v, move.accel)
            if move.axes_d[3]:
                self.extruder.move(next_move_time, move)
            next_move_time = (next_move_time + move.accel_t
                              + move.cruise_t + move.decel_t)
            for cb in move.timing_callbacks:
                cb(next_move_time)
        # Generate steps for moves
        if self.special_queuing_state:
            self._update_drip_move_time(next_move_time)
        self.note_mcu_movequeue_activity(next_move_time + self.kin_flush_delay,
                                         set_step_gen_time=True)
        self._advance_move_time(next_move_time)
        
    def _flush_lookahead(self):
        # Transit from "NeedPrime"/"Priming"/"Drip"/main state to "NeedPrime"
        self.lookahead.flush()
        self.special_queuing_state = "NeedPrime"
        self.need_check_pause = -1.
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.
    def flush_step_generation(self):
        self._flush_lookahead()
        self._advance_flush_time(self.step_gen_time)
        self.min_restart_time = max(self.min_restart_time, self.print_time)
        
    def get_last_move_time(self):
        if self.special_queuing_state:
            self._flush_lookahead()
            self._calc_print_time()
        else:
            self.lookahead.flush()
        return self.print_time
    
    def _check_pause(self):
        eventtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(eventtime)
        buffer_time = self.print_time - est_print_time
        if self.special_queuing_state:
            if self.check_stall_time:
                # Was in "NeedPrime" state and got there from idle input
                if est_print_time < self.check_stall_time:
                    self.print_stall += 1
                self.check_stall_time = 0.
            # Transition from "NeedPrime"/"Priming" state to "Priming" state
            self.special_queuing_state = "Priming"
            self.need_check_pause = -1.
            if self.priming_timer is None:
                self.priming_timer = self.reactor.register_timer(
                    self._priming_handler)
            wtime = eventtime + max(0.100, buffer_time - BUFFER_TIME_LOW)
            self.reactor.update_timer(self.priming_timer, wtime)
        # Check if there are lots of queued moves and pause if so
        while 1:
            pause_time = buffer_time - BUFFER_TIME_HIGH
            if pause_time <= 0.:
                break
            if not self.can_pause:
                self.need_check_pause = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1., pause_time))
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
        if not self.special_queuing_state:
            # In main state - defer pause checking until needed
            self.need_check_pause = est_print_time + BUFFER_TIME_HIGH + 0.100
            
    def _priming_handler(self, eventtime):
        self.reactor.unregister_timer(self.priming_timer)
        self.priming_timer = None
        try:
            if self.special_queuing_state == "Priming":
                self._flush_lookahead()
                self.check_stall_time = self.print_time
        except:
            logging.exception("Exception in priming_handler")
            self.printer.invoke_shutdown("Exception in priming_handler")
        return self.reactor.NEVER
    
    def _flush_handler(self, eventtime):
        try:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            if not self.special_queuing_state:
                # In "main" state - flush lookahead if buffer runs low
                print_time = self.print_time
                buffer_time = print_time - est_print_time
                if buffer_time > BUFFER_TIME_LOW:
                    # Running normally - reschedule check
                    return eventtime + buffer_time - BUFFER_TIME_LOW
                # Under ran low buffer mark - flush lookahead queue
                self._flush_lookahead()
                if print_time != self.print_time:
                    self.check_stall_time = self.print_time
            # In "NeedPrime"/"Priming" state - flush queues if needed
            while 1:
                end_flush = self.need_flush_time + BGFLUSH_EXTRA_TIME
                if self.last_flush_time >= end_flush:
                    self.do_kick_flush_timer = True
                    return self.reactor.NEVER
                buffer_time = self.last_flush_time - est_print_time
                if buffer_time > BGFLUSH_LOW_TIME:
                    return eventtime + buffer_time - BGFLUSH_LOW_TIME
                ftime = est_print_time + BGFLUSH_LOW_TIME + BGFLUSH_BATCH_TIME
                self._advance_flush_time(min(end_flush, ftime))
        except:
            logging.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER
    
    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)
    def set_position(self, newpos, homing_axes=()):
        
        self.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        ffi_lib.trapq_set_position(self.trapq, self.print_time,
                                   newpos[0], newpos[1], newpos[2])
        self.commanded_pos[:] = newpos
        self.kin.set_position(newpos, homing_axes)
        self.printer.send_event("toolhead:set_position")
        
    def move(self, newpos, speed):
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.lookahead.add_move(move)
        if self.print_time > self.need_check_pause:
            self._check_pause()
            
    def manual_move(self, coord, speed):
        curpos = list(self.commanded_pos)
        for i in range(len(coord)):
            if coord[i] is not None:
                curpos[i] = coord[i]
        self.move(curpos, speed)
        self.printer.send_event("toolhead:manual_move")
        
    def dwell(self, delay):
        next_print_time = self.get_last_move_time() + max(0., delay)
        self._advance_move_time(next_print_time)
        self._check_pause()
        
    def wait_moves(self):
        self._flush_lookahead()
        eventtime = self.reactor.monotonic()
        while (not self.special_queuing_state
               or self.print_time >= self.mcu.estimated_print_time(eventtime)):
            if not self.can_pause:
                break
            eventtime = self.reactor.pause(eventtime + 0.100)
            
    def set_extruder(self, extruder, extrude_pos):
        self.extruder = extruder
        self.commanded_pos[3] = extrude_pos
        
    def get_extruder(self):
        return self.extruder
    # Homing "drip move" handling
    
    def _update_drip_move_time(self, next_print_time):
        flush_delay = DRIP_TIME + STEPCOMPRESS_FLUSH_TIME + self.kin_flush_delay
        while self.print_time < next_print_time:
            if self.drip_completion.test():
                raise DripModeEndSignal()
            curtime = self.reactor.monotonic()
            est_print_time = self.mcu.estimated_print_time(curtime)
            wait_time = self.print_time - est_print_time - flush_delay
            if wait_time > 0. and self.can_pause:
                # Pause before sending more steps
                self.drip_completion.wait(curtime + wait_time)
                continue
            npt = min(self.print_time + DRIP_SEGMENT_TIME, next_print_time)
            self.note_mcu_movequeue_activity(npt + self.kin_flush_delay,
                                             set_step_gen_time=True)
            self._advance_move_time(npt)
            
    def drip_move(self, newpos, speed, drip_completion):
        self.dwell(self.kin_flush_delay)
        # Transition from "NeedPrime"/"Priming"/main state to "Drip" state
        self.lookahead.flush()
        self.special_queuing_state = "Drip"
        self.need_check_pause = self.reactor.NEVER
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.check_stall_time = 0.
        self.drip_completion = drip_completion
        # Submit move
        try:
            self.move(newpos, speed)
        except self.printer.command_error as e:
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
            self.flush_step_generation()
            raise
        # Transmit move in "drip" mode
        try:
            self.lookahead.flush()
        except DripModeEndSignal as e:
            self.lookahead.reset()
            self.trapq_finalize_moves(self.trapq, self.reactor.NEVER, 0)
        # Exit "Drip" state
        self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
        self.flush_step_generation()
        
    # Misc commands
    def stats(self, eventtime):
        max_queue_time = max(self.print_time, self.last_flush_time)
        for m in self.all_mcus:
            m.check_active(max_queue_time, eventtime)
        est_print_time = self.mcu.estimated_print_time(eventtime)
        self.clear_history_time = est_print_time - MOVE_HISTORY_EXPIRE
        buffer_time = self.print_time - est_print_time
        is_active = buffer_time > -60. or not self.special_queuing_state
        if self.special_queuing_state == "Drip":
            buffer_time = 0.
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time, max(buffer_time, 0.), self.print_stall)
    
    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.lookahead.queue
        return self.print_time, est_print_time, lookahead_empty
    
    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        res = dict(self.kin.get_status(eventtime))
        res.update({ 'print_time': print_time,
                     'stalls': self.print_stall,
                     'estimated_print_time': estimated_print_time,
                     'extruder': self.extruder.get_name(),
                     'position': self.Coord(*self.commanded_pos),
                     'max_velocity': self.max_velocity,
                     'max_accel': self.max_accel,
                     'minimum_cruise_ratio': self.min_cruise_ratio,
                     'square_corner_velocity': self.square_corner_velocity})
        return res
    
    def _handle_shutdown(self):
        self.can_pause = False
        self.lookahead.reset()
        
    def get_kinematics(self):
        return self.kin
    
    def get_trapq(self):
        return self.trapq
    
    def register_step_generator(self, handler):
        self.step_generators.append(handler)
        
    def note_step_generation_scan_time(self, delay, old_delay=0.):
        self.flush_step_generation()
        if old_delay:
            self.kin_flush_times.pop(self.kin_flush_times.index(old_delay))
        if delay:
            self.kin_flush_times.append(delay)
        new_delay = max(self.kin_flush_times + [SDS_CHECK_TIME])
        self.kin_flush_delay = new_delay
        
    def register_lookahead_callback(self, callback):
        last_move = self.lookahead.get_last()
        if last_move is None:
            callback(self.get_last_move_time())
            return
        last_move.timing_callbacks.append(callback)
        
    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        self.need_flush_time = max(self.need_flush_time, mq_time)
        if set_step_gen_time:
            self.step_gen_time = max(self.step_gen_time, mq_time)
        if self.do_kick_flush_timer:
            self.do_kick_flush_timer = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
            
    def get_max_velocity(self):
        return self.max_velocity, self.max_accel
    
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = self.max_accel * (1. - self.min_cruise_ratio)
        
    def cmd_G4(self, gcmd):
        # Dwell
        delay = gcmd.get_float('P', 0., minval=0.) / 1000.
        self.dwell(delay)
        
    def cmd_M400(self, gcmd):
        # Wait for current moves to finish
        self.wait_moves()
    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits"
    
    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        max_velocity = gcmd.get_float('VELOCITY', None, above=0.)
        max_accel = gcmd.get_float('ACCEL', None, above=0.)
        square_corner_velocity = gcmd.get_float(
            'SQUARE_CORNER_VELOCITY', None, minval=0.)
        min_cruise_ratio = gcmd.get_float(
            'MINIMUM_CRUISE_RATIO', None, minval=0., below=1.)
        if min_cruise_ratio is None:
            req_accel_to_decel = gcmd.get_float('ACCEL_TO_DECEL',
                                                None, above=0.)
            if req_accel_to_decel is not None and max_accel is not None:
                min_cruise_ratio = 1. - min(1., req_accel_to_decel / max_accel)
            elif req_accel_to_decel is not None and max_accel is None:
                min_cruise_ratio = 1. - min(1., (req_accel_to_decel
                                                 / self.max_accel))
        if max_velocity is not None:
            self.max_velocity = max_velocity
        if max_accel is not None:
            self.max_accel = max_accel
        if square_corner_velocity is not None:
            self.square_corner_velocity = square_corner_velocity
        if min_cruise_ratio is not None:
            self.min_cruise_ratio = min_cruise_ratio
        self._calc_junction_deviation()
        msg = ("max_velocity: %.6f\n"
               "max_accel: %.6f\n"
               "minimum_cruise_ratio: %.6f\n"
               "square_corner_velocity: %.6f" % (
                   self.max_velocity, self.max_accel,
                   self.min_cruise_ratio, self.square_corner_velocity))
        self.printer.set_rollover_info("toolhead", "toolhead: %s" % (msg,))
        if (max_velocity is None and max_accel is None
            and square_corner_velocity is None and min_cruise_ratio is None):
            gcmd.respond_info(msg, log=False)
            
    def cmd_M204(self, gcmd):
        # Use S for accel
        accel = gcmd.get_float('S', None, above=0.)
        if accel is None:
            # Use minimum of P and T for accel
            p = gcmd.get_float('P', None, above=0.)
            t = gcmd.get_float('T', None, above=0.)
            if p is None or t is None:
                gcmd.respond_info('Invalid M204 command "%s"'
                                  % (gcmd.get_commandline(),))
                return
            accel = min(p, t)
        self.max_accel = accel
        self._calc_junction_deviation()


class ParrafraktorHandler:
    def __init__(self, config, dmx_start_adress):
        
        self.reactor = self.printer.get_reactor()
        self.kinematics = kinematics.NoneKinematics(self, config)
        self.steppers = self.kinematics.get_steppers()
        self.dmx_values = [0] * 512
        self.dmx_values_old = [0] * 512
        self.dmx_values_new = [0] * 512
        self.dmx_start_adress = dmx_start_adress
        self.dmx_channel_mode = 4
        self.dmx_debug_channels = 2
        self.speeds = [0.] * len(self.steppers)
        self.targets = [0] * len(self.steppers)
        self.artnet = StupidArtnetServer(0)
        self.artnet_listener = self.artnet.get_listener()
        self.artnet_listener.register_handler(self.get_artnet_data)
        
        def get_artnet_channel(self, address):
            buffer = self.artnet.get_buffer(self.artnet_listener)
            if buffer:
                self.dmx_values_new[address] = buffer[address]
                self.dmx_values_old[address] = self.dmx_values_new[address]
            else:
                print(f'No DMX values received for address: {address}')
                
        def get_artnet_data_range(self, start, end):
            for i in range(start, end):
                self.get_artnet_channel(i)

                
        def set_stepper_dmx_values(self):
            for i in range(4):
                stepper_index = i * 4
                for j in range(4):
                    dmx_index = stepper_index + j
                    self.steppers[i].set_dmx_values(self.dmx_values_old[dmx_index])


        def run(self):
            while True:
                #self.reactor.sleep(0.1)
                # read dmx values from artnet
                self.get_artnet_data_range(self.dmx_start_address , self.dmx_start_address +
                                           self.dmx_channel_mode * len(self.steppers) + self.dmx_debug_channels)
                # set dmx values of the steppers
                self.set_stepper_dmx_values()
                # get the target positions of the steppers
                self.get_stepper_target()
                # get the speed of the steppers
                self.get_stepper_speed()
                # schedule the moves
                self.schedule_moves(self.targets, self.speeds)

class ToolHead:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.commanded_pos = [0, 0, 0, 0, 0, 0]
        self.current_speed = [0, 0, 0, 0, 0, 0]
        self.step_generators = []
        self.lookahead = LookAheadQueue(self)
        self.lookahead.set_flush_time(BUFFER_TIME_HIGH)
        self.flush_timer = self.reactor.register_timer(self._flush_handler)

    def dmx_input_handler(self, dmx_data):
        # Parse DMX data and create moves for each stepper
        for i in range(6):
            dmx_value = dmx_data.get(i, 0)
            target_pos, speed = self._calculate_target_for_stepper(dmx_value)
            self._update_stepper_state(i, target_pos, speed)

    def _calculate_target_for_stepper(self, dmx_value):
        # Example mapping: DMX value 0-255 to position 0-100 and speed 0-10
        max_position = 100
        max_speed = 10
        target_pos = (dmx_value / 255.0) * max_position
        speed = (dmx_value / 255.0) * max_speed
        return target_pos, speed

    def _update_stepper_state(self, stepper_index, target_pos, speed):
        current_pos = self.commanded_pos[stepper_index]
        if target_pos != current_pos:
            move = self._create_move(current_pos, target_pos, speed)
            self._process_move(stepper_index, move)
            self.commanded_pos[stepper_index] = target_pos
            self.current_speed[stepper_index] = speed

    def _create_move(self, current_pos, target_pos, speed):
        # Create a move object (you need to define the Move class)
        return Move(current_pos, target_pos, speed)

    def _process_move(self, stepper_index, move):
        next_move_time = self.reactor.monotonic()
        self.trapq_append(self.trapq, next_move_time, move)
        next_move_time += move.duration
        self._advance_move_time(next_move_time)

    def _flush_handler(self, eventtime):
        # Handle flushing the move queue
        self._advance_flush_time(eventtime)
        return self.reactor.NEVER

    def _advance_flush_time(self, flush_time):
        # Generate steps via step generators
        for sg in self.step_generators:
            sg(flush_time)
        self.last_flush_time = flush_time

    def _advance_move_time(self, next_move_time):
        self.print_time = max(self.print_time, next_move_time)
        self._advance_flush_time(next_move_time)
                    

class ToolHead5:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.kinematics = kinematics.NoneKinematics(self, config)
        self.steppers = self.kinematics.get_steppers()
        self.last_position = [0.] * len(self.steppers)
        self.speed = [0.] * len(self.steppers)
        self.move_queue = []
        self.last_flush_time = self.reactor.monotonic()
        

        self.artnet = StupidArtnetServer(0)
        self.dmx_start_adress = 1
        self.dmx_address = [0] * 512
        self.dmx_channel_mode = 4
        self.dmx_debug_channels = 2
        self.dmx_values_new = [0] * (len(self.steppers) * self.dmx_channel_mode + self.dmx_debug_channels)
        self.dmx_values_old = [0] * (len(self.steppers) * self.dmx_channel_mode + self.dmx_debug_channels)
        self.speeds = [0.] * len(self.steppers)
        self.targets = [0] * len(self.steppers)
        

    def get_artnet_data(self, address):
        buffer = self.artnet.get_buffer(self.artnet_listener)
        
        if buffer:            
                self.dmx_values_new[address] = buffer[address]
                self.dmx_values_old[address] = self.dmx_values_new[address]                
        else:
            
            print('No DMX values received')
            
    # sets the dmx values of the steppers
    def set_stepper_dmx_values(self):
        for i in range(len(self.steppers)):
            stepper_index = i * 4
            for j in range(4):
                dmx_index = stepper_index + j
                self.steppers[i].set_dmx_values(self.dmx_values_old[dmx_index])

    def get_stepper_target(self):
        for i in range(len(self.steppers)):
           self.targets[i] = self.stepper[i].calculate_target()
            
    def get_stepper_speed(self):
        for i in range(len(self.steppers)):
            self.speeds[i] = self.stepper[i].calculate_speed()
  
    def schedule_moves(self, positions, speeds):
        moves = []
        for i, stepper in enumerate(self.steppers):
            current_pos = stepper.get_position()
            move = Move(self, current_pos, positions[i], speeds[i])
            moves.append(move)
            stepper.move(move)
        self.flush_moves(moves)

    def move_stepper(self, stepper, target_pos, speed):
        current_pos = stepper.get_position()
        move = Move(self, current_pos, target_pos, speed)
        stepper.move(move)
        self.flush_stepper(stepper)
        
    def flush_moves(self, moves):
        self.reactor.update_move_time(self.last_flush_time)
        for stepper in self.steppers:
            stepper.flush_move()
        self.last_flush_time = self.reactor.monotonic()
    


    def get_status(self, eventtime):
        status = self.kinematics.get_status(eventtime)
        return {
            'pos': self.last_position,
            'speed': self.speed,
            **status
        }
    



def add_printer_objects(config):
    config.get_printer().add_object('toolhead', ToolHead(config))
    kinematics.extruder.add_printer_objects(config)
