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
    .pio = pio0,
    .pio_sm = 0,
    .sample_rate = 8000,
    .gpio_clk = 2,
    .gpio_data = 3,
};

int16_t sample_buffer[256];
volatile int samples_read = 0;

void on_pdm_data(int16_t* samples, uint num_samples)
{
    memcpy(sample_buffer, samples, num_samples * sizeof(samples[0]));

    samples_read = num_samples;
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    // printf("hello PDM microphone\n");

    pdm_microphone_init(&config);
    pdm_microphone_start(on_pdm_data);

    while (1) {
        while (samples_read == 0) {
            tight_loop_contents();
        }

        for (int i = 0; i < samples_read; i++) {
            printf("%d\n", sample_buffer[i]);
        }

        samples_read = 0;
    }

    return 0;
}
