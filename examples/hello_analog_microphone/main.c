/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/analog_microphone.h"
#include "tusb.h"

struct analog_microphone_config config = {
    .gpio = 26,
    .bias_voltage = 1.25,
    .sample_rate = 8000,
    .sample_buffer_size = 256,
};

int16_t sample_buffer[256];
volatile int samples_read = 0;

void on_analog_samples_ready()
{
  samples_read = analog_microphone_read(sample_buffer, 256);
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    printf("hello analog microphone\n");

    if (analog_microphone_init(&config) < 0) {
        printf("analog microphone initialization failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    analog_microphone_set_samples_ready_handler(on_analog_samples_ready);
    
    if (analog_microphone_start() < 0) {
        printf("PDM microphone start failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    while (1) {
        int sample_count = samples_read;
        samples_read = 0;
        
        for (int i = 0; i < sample_count; i++) {
            printf("%d\n", sample_buffer[i]);
        }
    }

    return 0;
}
