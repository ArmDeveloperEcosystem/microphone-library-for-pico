/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_ANALOG_MICROPHONE_H_
#define _PICO_ANALOG_MICROPHONE_H_

//  #include "hardware/pio.h"
#include <stddef.h>

typedef void (*analog_samples_ready_handler_t)(void);

struct analog_microphone_config {
    unsigned int gpio;
    float bias_voltage;
    unsigned int sample_rate;
    unsigned int sample_buffer_size;
};

int analog_microphone_init(const struct analog_microphone_config* config);
void analog_microphone_deinit();

int analog_microphone_start();
void analog_microphone_stop();

void analog_microphone_set_samples_ready_handler(analog_samples_ready_handler_t handler);

int analog_microphone_read(signed short* buffer, size_t samples);

#endif
