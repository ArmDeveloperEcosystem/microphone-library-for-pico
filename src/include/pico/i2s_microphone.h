/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#ifndef _PICO_I2S_MICROPHONE_H_
#define _PICO_I2S_MICROPHONE_H_

#include "hardware/pio.h"

#define MAX_I2S_RP2 (2)

// The DMA buffer size was empirically determined.  It is a tradeoff between:
// 1. memory use (smaller buffer size desirable to reduce memory footprint)
// 2. interrupt frequency (larger buffer size desirable to reduce interrupt frequency)
#define SIZEOF_DMA_BUFFER_IN_BYTES (512 * 2)
#define SIZEOF_HALF_DMA_BUFFER_IN_BYTES (SIZEOF_DMA_BUFFER_IN_BYTES / 2)
#define I2S_NUM_DMA_CHANNELS (2)

#define NUM_I2S_USER_FORMATS (4)
#define I2S_RX_FRAME_SIZE_IN_BYTES (8)

#define SAMPLES_PER_FRAME (2)
#define PIO_INSTRUCTIONS_PER_BIT (2)

#define STATIC static
#define mp_hal_pin_obj_t uint
#define m_new(type, num) ((type *)(malloc(sizeof(type) * (num))))
#define m_new_obj(type) (m_new(type, 1))

typedef void (*i2s_samples_ready_handler_t)(void);

typedef enum {
    RX,
    TX
} i2s_mode_t;

typedef enum {
    MONO,
    STEREO
} format_t;

typedef enum {
    BLOCKING,
    NON_BLOCKING,
    UASYNCIO
} io_mode_t;

typedef enum {
    GP_INPUT = 0,
    GP_OUTPUT = 1
} gpio_dir_t;

typedef struct _ring_buf_t {
    uint8_t *buffer;
    size_t head;
    size_t tail;
    size_t size;
} ring_buf_t;


// Buffer protocol
typedef struct _mp_buffer_info_t {
    void *buf;      // can be NULL if len == 0
    size_t len;     // in bytes
    int typecode;   // as per binary.h
} mp_buffer_info_t;

typedef struct _machine_i2s_obj_t {
    uint8_t i2s_id;
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t ws;
    mp_hal_pin_obj_t sd;
    i2s_mode_t mode;
    int8_t bits;
    format_t format;
    int32_t rate;
    int32_t ibuf;
    io_mode_t io_mode;
    PIO pio;
    uint8_t sm;
    const pio_program_t *pio_program;
    uint prog_offset;
    int dma_channel[I2S_NUM_DMA_CHANNELS];
    uint8_t dma_buffer[SIZEOF_DMA_BUFFER_IN_BYTES];
    ring_buf_t ring_buffer;
    uint8_t *ring_buffer_storage;
    uint8_t flagHandler;
} machine_i2s_obj_t;

typedef void (*i2s_samples_ready_handler_t)(void);

STATIC machine_i2s_obj_t* machine_i2s_make_new(uint8_t i2s_id, mp_hal_pin_obj_t sck, mp_hal_pin_obj_t ws, mp_hal_pin_obj_t sd, i2s_mode_t i2s_mode, int8_t i2s_bits, format_t i2s_format, int32_t ring_buffer_len, int32_t i2s_rate);

STATIC void machine_i2s_deinit(machine_i2s_obj_t *self);

STATIC void i2s_microphone_set_samples_ready_handler(i2s_samples_ready_handler_t handler);


#endif
