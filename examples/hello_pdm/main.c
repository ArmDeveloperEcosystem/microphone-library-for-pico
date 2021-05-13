/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/microphone/pdm.h"
#include "tusb.h"

struct pdm_microphone_config config = {
    .gpio_clk = 2,
    .gpio_data = 3,
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 8000,
    .sample_buffer_size = 256,
};

int16_t sample_buffer[256];
volatile int samples_read = 0;

void on_pdm_samples_ready()
{
  samples_read = pdm_microphone_read(sample_buffer, 256);
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    printf("hello PDM microphone\n");

    if (pdm_microphone_init(&config) < 0) {
        printf("PDM microphone initialization failed!\n");
        while (1) {
            tight_loop_contents();
        }
    }

    pdm_microphone_set_samples_ready_handler(on_pdm_samples_ready);
    
    if (pdm_microphone_start() < 0) {
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
