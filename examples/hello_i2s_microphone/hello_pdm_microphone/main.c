/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * This examples captures data from a PDM microphone using a sample
 * rate of 8 kHz and prints the sample values over the USB serial
 * connection.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/i2s_microphone.h"
#include "tusb.h"


#define SCK 2
#define WS 3 // needs to be SCK +1
#define SD 6
#define BPS 16 
#define SAMPLE_RATE       16000
#define FFT_SIZE          256
#define SPECTRUM_SHIFT    4
#define INPUT_BUFFER_SIZE ((FFT_SIZE / 2) * SPECTRUM_SHIFT)
#define INPUT_SHIFT       0
#define BUTTON 15
#define LEN_WAVE_HEADER 44

// variables
int16_t sample_buffer[SIZEOF_HALF_DMA_BUFFER_IN_BYTES];
volatile int samples_read = 0;
machine_i2s_obj_t* i2s0;
typedef void (*I2SHandler) (machine_i2s_obj_t* i2s0);

void on_i2s_mic_samples_ready(machine_i2s_obj_t* i2s0){
    samples_read = machine_i2s_stream_read(i2s0, (void*)&sample_buffer, INPUT_BUFFER_SIZE * 2);
    if(samples_read != 0){
        printf("Data OK\r\n");
    }
}

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();
    while (!tud_cdc_connected()) {
        tight_loop_contents();
    }

    printf("hello i2s microphone\n");

    // initialize the PDM microphone
    i2s0 = machine_i2s_make_new(0, SCK, WS, SD, RX, BPS, MONO, SIZEOF_DMA_BUFFER_IN_BYTES, SAMPLE_RATE);
    if(i2s0 == NULL){
        printf("Failed to initialize MIC I2S!\n");
        while (1) { tight_loop_contents(); }
    }
    else{
        printf("|        MIC I2S initialize OK       |\n");
    }

    // set callback that is called when all the samples in the library
    // internal sample buffer are ready for reading
    i2s_microphone_set_samples_ready_handler(on_i2s_mic_samples_ready);
    
    while (1) {
        // wait for new samples
        while (samples_read == 0) { tight_loop_contents(); }

        // store and clear the samples read from the callback
        int sample_count = samples_read;
        samples_read = 0;
        
        // loop through any new collected samples
        for (int i = 0; i < sample_count; i++) {
            printf("%d\n", sample_buffer[i]);
        }
    }

    return 0;
}
