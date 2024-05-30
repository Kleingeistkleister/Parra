// GPIO Output Control Commands
//
// Copyright (C) 2024 Your Name <your@email.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "basecmd.h"
#include "board/gpio.h"
#include "command.h"

void command_set_gpio_output(uint32_t *args) {
    uint32_t pin = args[0];
    uint32_t value = args[1];
    
    // Setup GPIO pin as an output
    gpio_setup_output(pin);
    
    // Set the value of the GPIO pin
    gpio_set_output_value(pin, value);
}

DECL_COMMAND(command_set_gpio_output, "set_gpio_output pin=%u value=%u");
