/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "OpenPDM2PCM/OpenPDMFilter.h"

#include "pdm-microphone.pio.h"

#include "pico/microphone/pdm.h"

#define PDM_DECIMATION 64
#define PCM_BUFFER_SIZE 16

static int dma_channel = -1;
static uint8_t pdm_buffer[2][PCM_BUFFER_SIZE * (PDM_DECIMATION / 8)];
static volatile int pdm_buffer_index = 0;
static int16_t pcm_buffer[PCM_BUFFER_SIZE];

static TPDMFilter_InitStruct pdm_filter;
static pdm_data_handler_t pdm_data_handler = NULL;

static void dma_handler()
{
    // clear IRQ
    dma_hw->ints0 = 1u << dma_channel;
    
    // get the current buffer index
    int read_index = pdm_buffer_index;
    
    // get the next capture index to send the dma to start
    pdm_buffer_index = (pdm_buffer_index + 1) % 2;
  
    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(dma_channel, pdm_buffer[pdm_buffer_index], sizeof(pdm_buffer[0]));

    uint8_t* in = pdm_buffer[read_index];
    int16_t* out = pcm_buffer;

    int filter_stride = (pdm_filter.Fs / 1000);

    for (int i = 0; i < (PCM_BUFFER_SIZE / filter_stride); i++) {
#if PDM_DECIMATION == 64
        Open_PDM_Filter_64(in, out, 16, &pdm_filter);
#elif PDM_DECIMATION == 128
        Open_PDM_Filter_128(in, out, 16, &pdm_filter);
#else
        #error "Unsupported PDM_DECIMATION value!"
#endif

        in += filter_stride * (PDM_DECIMATION / 8);
        out += filter_stride;
    }

    pdm_data_handler(pcm_buffer, PCM_BUFFER_SIZE);
}

int pdm_microphone_init(const struct pdm_microphone_config* config)
{
    uint pio_sm_offset = pio_add_program(config->pio, &pdm_microphone_data_program);

    float clk_div = clock_get_hz(clk_sys) / (config->sample_rate * PDM_DECIMATION * 2);

    pdm_microphone_data_init(
        config->pio,
        config->pio_sm,
        pio_sm_offset,
        clk_div,
        config->gpio_clk,
        config->gpio_data
    );

    dma_channel = dma_claim_unused_channel(true);
    
    // TODO: handle claim failure
    
    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, pio_get_dreq(config->pio, config->pio_sm, false));

    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);

    dma_channel_set_irq0_enabled(dma_channel, true);

    dma_channel_configure(dma_channel, &dma_channel_cfg, NULL, &config->pio->rxf[config->pio_sm], sizeof(pdm_buffer[pdm_buffer_index]), false);

    pdm_filter.Fs = config->sample_rate;
    pdm_filter.LP_HZ = config->sample_rate / 2;
    pdm_filter.HP_HZ = 10;
    pdm_filter.In_MicChannels = 1;
    pdm_filter.Out_MicChannels = 1;
    pdm_filter.Decimation = PDM_DECIMATION;
    pdm_filter.MaxVolume = 0;

    Open_PDM_Filter_Init(&pdm_filter);

    return -1;
}

int pdm_microphone_start(pdm_data_handler_t handler)
{
    pdm_data_handler = handler;

    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(dma_channel, pdm_buffer[pdm_buffer_index], sizeof(pdm_buffer[0]));

    return -1;
}
