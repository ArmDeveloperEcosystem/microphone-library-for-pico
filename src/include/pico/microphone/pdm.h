/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_MICROPHONE_PDM_H_
#define _PICO_MICROPHONE_PDM_H_

#include "hardware/pio.h"

typedef void (*pdm_data_handler_t)(int16_t*, uint);

struct pdm_microphone_config {
    PIO pio;
    uint pio_sm;
    uint sample_rate;
    uint gpio_clk;
    uint gpio_data;
    // TODO: pass in buffer???
};

int pdm_microphone_init(const struct pdm_microphone_config* config);
int pdm_microphone_start(pdm_data_handler_t handler);

#endif
