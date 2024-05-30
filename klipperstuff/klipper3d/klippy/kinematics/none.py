# Dummy "none" kinematics support (for developer testing)
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from klippy.kinematics import Kinematics
from klippy.stepper import PrinterStepper


class NoneKinematics:
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        self.axes_minmax = toolhead.Coord(0., 0., 0., 0.)
        self.steppers = self.init_steppers(config)

    def init_steppers(self, config):
        steppers = []
        for i in range(6):
            stepper_config = config.getsection(f'stepper_{i}')
            if stepper_config:
                stepper = PrinterStepper(stepper_config)
                steppers.append(stepper)
                self.toolhead.register_stepper(stepper)
        return steppers

    def get_steppers(self):
        return self.steppers

    def calc_position(self, stepper_positions):
        return stepper_positions

    def set_position(self, newpos, homing_axes):
        for i, pos in enumerate(newpos):
            self.steppers[i].set_position(pos)

    def home(self, homing_state):
        pass

    def check_move(self, move):
        pass

    def get_status(self, eventtime):
        return {
            'homed_axes': 'abcdef',
            'axis_minimum': self.axes_minmax,
            'axis_maximum': self.axes_minmax,
        }
