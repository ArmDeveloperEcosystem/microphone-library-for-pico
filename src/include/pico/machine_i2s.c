/*
  machine_i2s.c -
    I2S digital audio input C library for the Raspberry Pi Pico RP2040

    Copyright (C) 2022 Sfera Labs S.r.l. - All rights reserved.

    For information, see:
    http://www.sferalabs.cc/

  This code is adapted from the I2S implementation of the RP2 MicroPython port
  by Mike Teachman, available at:
  https://github.com/micropython/micropython/blob/master/ports/rp2/machine_i2s.c
  Retrieved on January 25 2022.
*/

/* Original header */

/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2021 Mike Teachman
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#define MAX_I2S_RP2 (2)

// The DMA buffer size was empirically determined.  It is a tradeoff between:
// 1. memory use (smaller buffer size desirable to reduce memory footprint)
// 2. interrupt frequency (larger buffer size desirable to reduce interrupt frequency)
#define SIZEOF_DMA_BUFFER_IN_BYTES (256)
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
} machine_i2s_obj_t;

// Buffer protocol
typedef struct _mp_buffer_info_t {
    void *buf;      // can be NULL if len == 0
    size_t len;     // in bytes
    int typecode;   // as per binary.h
} mp_buffer_info_t;

STATIC machine_i2s_obj_t* machine_i2s_obj[MAX_I2S_RP2] = {NULL, NULL};

// The frame map is used with the readinto() method to transform the audio sample data coming
// from DMA memory (32-bit stereo) to the format specified
// in the I2S constructor.  e.g.  16-bit mono
STATIC const int8_t i2s_frame_map[NUM_I2S_USER_FORMATS][I2S_RX_FRAME_SIZE_IN_BYTES] = {
    {-1, -1,  0,  1, -1, -1, -1, -1 },  // Mono, 16-bits
    { 0,  1,  2,  3, -1, -1, -1, -1 },  // Mono, 32-bits
    {-1, -1,  0,  1, -1, -1,  2,  3 },  // Stereo, 16-bits
    { 0,  1,  2,  3,  4,  5,  6,  7 },  // Stereo, 32-bits
};

STATIC const PIO pio_instances[NUM_PIOS] = {pio0, pio1};

//  PIO program for 16-bit write
//    set(x, 14)                  .side(0b01)
//    label('left_channel')
//    out(pins, 1)                .side(0b00)
//    jmp(x_dec, "left_channel")  .side(0b01)
//    out(pins, 1)                .side(0b10)
//    set(x, 14)                  .side(0b11)
//    label('right_channel')
//    out(pins, 1)                .side(0b10)
//    jmp(x_dec, "right_channel") .side(0b11)
//    out(pins, 1)                .side(0b00)
STATIC const uint16_t pio_instructions_write_16[] = {59438, 24577, 2113, 28673, 63534, 28673, 6213, 24577};
STATIC const pio_program_t pio_write_16 = {
    pio_instructions_write_16,
    sizeof(pio_instructions_write_16) / sizeof(uint16_t),
    -1
};

//  PIO program for 32-bit write
//    set(x, 30)                  .side(0b01)
//    label('left_channel')
//    out(pins, 1)                .side(0b00)
//    jmp(x_dec, "left_channel")  .side(0b01)
//    out(pins, 1)                .side(0b10)
//    set(x, 30)                  .side(0b11)
//    label('right_channel')
//    out(pins, 1)                .side(0b10)
//    jmp(x_dec, "right_channel") .side(0b11)
//    out(pins, 1)                .side(0b00)
STATIC const uint16_t pio_instructions_write_32[] = {59454, 24577, 2113, 28673, 63550, 28673, 6213, 24577};
STATIC const pio_program_t pio_write_32 = {
    pio_instructions_write_32,
    sizeof(pio_instructions_write_32) / sizeof(uint16_t),
    -1
};

//  PIO program for 32-bit read
//    set(x, 30)                  .side(0b00)
//    label('left_channel')
//    in_(pins, 1)                .side(0b01)
//    jmp(x_dec, "left_channel")  .side(0b00)
//    in_(pins, 1)                .side(0b11)
//    set(x, 30)                  .side(0b10)
//    label('right_channel')
//    in_(pins, 1)                .side(0b11)
//    jmp(x_dec, "right_channel") .side(0b10)
//    in_(pins, 1)                .side(0b01)
STATIC const uint16_t pio_instructions_read_32[] = {57406, 18433, 65, 22529, 61502, 22529, 4165, 18433};
STATIC const pio_program_t pio_read_32 = {
    pio_instructions_read_32,
    sizeof(pio_instructions_read_32) / sizeof(uint16_t),
    -1
};

STATIC uint8_t dma_get_bits(i2s_mode_t mode, int8_t bits);
STATIC void dma_irq0_handler(void);
STATIC void dma_irq1_handler(void);
STATIC void machine_i2s_deinit(machine_i2s_obj_t *self);

// Ring Buffer
// Thread safe when used with these constraints:
// - Single Producer, Single Consumer
// - Sequential atomic operations
// One byte of capacity is used to detect buffer empty/full

STATIC void ringbuf_init(ring_buf_t *rbuf, uint8_t *buffer, size_t size) {
    rbuf->buffer = buffer;
    rbuf->size = size;
    rbuf->head = 0;
    rbuf->tail = 0;
}

STATIC bool ringbuf_push(ring_buf_t *rbuf, uint8_t data) {
    size_t next_tail = (rbuf->tail + 1) % rbuf->size;

    if (next_tail != rbuf->head) {
        rbuf->buffer[rbuf->tail] = data;
        rbuf->tail = next_tail;
        return true;
    }

    // full
    return false;
}

STATIC bool ringbuf_pop(ring_buf_t *rbuf, uint8_t *data) {
    stdio_flush();
    if (rbuf->head == rbuf->tail) {
        // empty
        return false;
    }

    *data = rbuf->buffer[rbuf->head];
    rbuf->head = (rbuf->head + 1) % rbuf->size;
    return true;
}

STATIC bool ringbuf_is_empty(ring_buf_t *rbuf) {
    return rbuf->head == rbuf->tail;
}

STATIC bool ringbuf_is_full(ring_buf_t *rbuf) {
    return ((rbuf->tail + 1) % rbuf->size) == rbuf->head;
}

STATIC size_t ringbuf_available_data(ring_buf_t *rbuf) {
    return (rbuf->tail - rbuf->head + rbuf->size) % rbuf->size;
}

STATIC size_t ringbuf_available_space(ring_buf_t *rbuf) {
    return rbuf->size - ringbuf_available_data(rbuf) - 1;
}

STATIC int8_t get_frame_mapping_index(int8_t bits, format_t format) {
    if (format == MONO) {
        if (bits == 16) {
            return 0;
        } else { // 32 bits
            return 1;
        }
    } else { // STEREO
        if (bits == 16) {
            return 2;
        } else { // 32 bits
            return 3;
        }
    }
}

STATIC uint32_t fill_appbuf_from_ringbuf(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {

    // copy audio samples from the ring buffer to the app buffer
    // loop, copying samples until the app buffer is filled
    // For uasyncio mode, the loop will make an early exit if the ring buffer becomes empty
    // Example:
    //   a MicroPython I2S object is configured for 16-bit mono (2 bytes per audio sample).
    //   For every frame coming from the ring buffer (8 bytes), 2 bytes are "cherry picked" and
    //   copied to the supplied app buffer.
    //   Thus, for every 1 byte copied to the app buffer, 4 bytes are read from the ring buffer.
    //   If a 8kB app buffer is supplied, 32kB of audio samples is read from the ring buffer.

    uint32_t num_bytes_copied_to_appbuf = 0;
    uint8_t *app_p = (uint8_t *)appbuf->buf;
    uint8_t appbuf_sample_size_in_bytes = (self->bits == 16? 2 : 4) * (self->format == STEREO ? 2: 1);
    uint32_t num_bytes_needed_from_ringbuf = appbuf->len * (I2S_RX_FRAME_SIZE_IN_BYTES / appbuf_sample_size_in_bytes);
    uint8_t discard_byte;
    while (num_bytes_needed_from_ringbuf) {

        uint8_t f_index = get_frame_mapping_index(self->bits, self->format);

        for (uint8_t i = 0; i < I2S_RX_FRAME_SIZE_IN_BYTES; i++) {
            int8_t r_to_a_mapping = i2s_frame_map[f_index][i];
            if (r_to_a_mapping != -1) {
                if (self->io_mode == BLOCKING) {
                    // poll the ringbuf until a sample becomes available,  copy into appbuf using the mapping transform
                    while (ringbuf_pop(&self->ring_buffer, app_p + r_to_a_mapping) == false) {
                        ;
                    }
                    num_bytes_copied_to_appbuf++;
                } else if (self->io_mode == UASYNCIO) {
                    if (ringbuf_pop(&self->ring_buffer, app_p + r_to_a_mapping) == false) {
                        // ring buffer is empty, exit
                        goto exit;
                    } else {
                        num_bytes_copied_to_appbuf++;
                    }
                } else {
                    return 0;  // should never get here (non-blocking mode does not use this function)
                }
            } else { // r_a_mapping == -1
                // discard unused byte from ring buffer
                if (self->io_mode == BLOCKING) {
                    // poll the ringbuf until a sample becomes available
                    while (ringbuf_pop(&self->ring_buffer, &discard_byte) == false) {
                        ;
                    }
                } else if (self->io_mode == UASYNCIO) {
                    if (ringbuf_pop(&self->ring_buffer, &discard_byte) == false) {
                        // ring buffer is empty, exit
                        goto exit;
                    }
                } else {
                    return 0;  // should never get here (non-blocking mode does not use this function)
                }
            }
            num_bytes_needed_from_ringbuf--;
        }
        app_p += appbuf_sample_size_in_bytes;
    }
exit:
    return num_bytes_copied_to_appbuf;
}

STATIC uint32_t copy_appbuf_to_ringbuf(machine_i2s_obj_t *self, mp_buffer_info_t *appbuf) {

    // copy audio samples from the app buffer to the ring buffer
    // loop, reading samples until the app buffer is emptied
    // for uasyncio mode, the loop will make an early exit if the ring buffer becomes full

    uint32_t a_index = 0;

    while (a_index < appbuf->len) {
        if (self->io_mode == BLOCKING) {
            // copy a byte to the ringbuf when space becomes available
            while (ringbuf_push(&self->ring_buffer, ((uint8_t *)appbuf->buf)[a_index]) == false) {
                ;
            }
            a_index++;
        } else if (self->io_mode == UASYNCIO) {
            if (ringbuf_push(&self->ring_buffer, ((uint8_t *)appbuf->buf)[a_index]) == false) {
                // ring buffer is full, exit
                break;
            } else {
                a_index++;
            }
        } else {
            return 0;  // should never get here (non-blocking mode does not use this function)
        }
    }

    return a_index;
}

// function is used in IRQ context
STATIC void empty_dma(machine_i2s_obj_t *self, uint8_t *dma_buffer_p) {
    // when space exists, copy samples into ring buffer
    if (ringbuf_available_space(&self->ring_buffer) >= SIZEOF_HALF_DMA_BUFFER_IN_BYTES) {
        for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES; i++) {
            ringbuf_push(&self->ring_buffer, dma_buffer_p[i]);
        }
    }
}

// function is used in IRQ context
STATIC void feed_dma(machine_i2s_obj_t *self, uint8_t *dma_buffer_p) {
    // when data exists, copy samples from ring buffer
    if (ringbuf_available_data(&self->ring_buffer) >= SIZEOF_HALF_DMA_BUFFER_IN_BYTES) {

        // copy a block of samples from the ring buffer to the dma buffer.
        // STM32 HAL API has a stereo I2S implementation, but not mono
        // mono format is implemented by duplicating each sample into both L and R channels.
        if ((self->format == MONO) && (self->bits == 16)) {
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES / 4; i++) {
                for (uint8_t b = 0; b < sizeof(uint16_t); b++) {
                    ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i * 4 + b]);
                    dma_buffer_p[i * 4 + b + 2] = dma_buffer_p[i * 4 + b]; // duplicated mono sample
                }
            }
        } else if ((self->format == MONO) && (self->bits == 32)) {
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES / 8; i++) {
                for (uint8_t b = 0; b < sizeof(uint32_t); b++) {
                    ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i * 8 + b]);
                    dma_buffer_p[i * 8 + b + 4] = dma_buffer_p[i * 8 + b]; // duplicated mono sample
                }
            }
        } else { // STEREO, both 16-bit and 32-bit
            for (uint32_t i = 0; i < SIZEOF_HALF_DMA_BUFFER_IN_BYTES; i++) {
                ringbuf_pop(&self->ring_buffer, &dma_buffer_p[i]);
            }
        }
    } else {
        // underflow.  clear buffer to transmit "silence" on the I2S bus
        memset(dma_buffer_p, 0, SIZEOF_HALF_DMA_BUFFER_IN_BYTES);
    }
}

STATIC void irq_configure(machine_i2s_obj_t *self) {
    if (self->i2s_id == 0) {
        irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
        irq_set_enabled(DMA_IRQ_0, true);
    } else {
        irq_set_exclusive_handler(DMA_IRQ_1, dma_irq1_handler);
        irq_set_enabled(DMA_IRQ_1, true);
    }
}

STATIC void irq_deinit(machine_i2s_obj_t *self) {
    if (self->i2s_id == 0) {
        irq_set_enabled(DMA_IRQ_0, false);
        irq_remove_handler(DMA_IRQ_0, dma_irq0_handler);
    } else {
        irq_set_enabled(DMA_IRQ_1, false);
        irq_remove_handler(DMA_IRQ_1, dma_irq1_handler);
    }
}

STATIC int pio_configure(machine_i2s_obj_t *self) {
    if (self->mode == TX) {
        if (self->bits == 16) {
            self->pio_program = &pio_write_16;
        } else {
            self->pio_program = &pio_write_32;
        }
    } else { // RX
        self->pio_program = &pio_read_32;
    }

    // find a PIO with a free state machine and adequate program space
    PIO candidate_pio;
    bool is_free_sm;
    bool can_add_program;
    for (uint8_t p = 0; p < NUM_PIOS; p++) {
        candidate_pio = pio_instances[p];
        is_free_sm = false;
        can_add_program = false;

        for (uint8_t sm = 0; sm < NUM_PIO_STATE_MACHINES; sm++) {
            if (!pio_sm_is_claimed(candidate_pio, sm)) {
                is_free_sm = true;
                break;
            }
        }

        if (pio_can_add_program(candidate_pio,  self->pio_program)) {
            can_add_program = true;
        }

        if (is_free_sm && can_add_program) {
            break;
        }
    }

    if (!is_free_sm) {
        return -1;
    }

    if (!can_add_program) {
        return -2;
    }

    self->pio = candidate_pio;
    self->sm = pio_claim_unused_sm(self->pio, false);
    self->prog_offset = pio_add_program(self->pio, self->pio_program);
    pio_sm_init(self->pio, self->sm, self->prog_offset, NULL);

    pio_sm_config config = pio_get_default_sm_config();

    float pio_freq = self->rate *
        SAMPLES_PER_FRAME *
        dma_get_bits(self->mode, self->bits) *
        PIO_INSTRUCTIONS_PER_BIT;
    float clkdiv = clock_get_hz(clk_sys) / pio_freq;
    sm_config_set_clkdiv(&config, clkdiv);

    if (self->mode == TX) {
        sm_config_set_out_pins(&config, self->sd, 1);
        sm_config_set_out_shift(&config, false, true, dma_get_bits(self->mode, self->bits));
        sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_TX);  // double TX FIFO size
    } else { // RX
        sm_config_set_in_pins(&config, self->sd);
        sm_config_set_in_shift(&config, false, true, dma_get_bits(self->mode, self->bits));
        sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);  // double RX FIFO size
    }

    sm_config_set_sideset(&config, 2, false, false);
    sm_config_set_sideset_pins(&config, self->sck);
    sm_config_set_wrap(&config, self->prog_offset, self->prog_offset + self->pio_program->length - 1);
    pio_sm_set_config(self->pio, self->sm, &config);

    return 0;
}

STATIC void pio_deinit(machine_i2s_obj_t *self) {
    if (self->pio) {
        pio_sm_set_enabled(self->pio, self->sm, false);
        pio_sm_unclaim(self->pio, self->sm);
        pio_remove_program(self->pio, self->pio_program, self->prog_offset);
    }
}

STATIC void gpio_init_i2s(PIO pio, uint8_t sm, mp_hal_pin_obj_t pin_num, uint8_t pin_val, gpio_dir_t pin_dir) {
    uint32_t pinmask = 1 << pin_num;
    pio_sm_set_pins_with_mask(pio, sm, pin_val << pin_num, pinmask);
    pio_sm_set_pindirs_with_mask(pio, sm, pin_dir << pin_num, pinmask);
    pio_gpio_init(pio, pin_num);
}

STATIC void gpio_configure(machine_i2s_obj_t *self) {
    gpio_init_i2s(self->pio, self->sm, self->sck, 0, GP_OUTPUT);
    gpio_init_i2s(self->pio, self->sm, self->ws, 0, GP_OUTPUT);
    if (self->mode == TX) {
        gpio_init_i2s(self->pio, self->sm, self->sd, 0, GP_OUTPUT);
    } else { // RX
        gpio_init_i2s(self->pio, self->sm, self->sd, 0, GP_INPUT);
    }
}

STATIC uint8_t dma_get_bits(i2s_mode_t mode, int8_t bits) {
    if (mode == TX) {
        return bits;
    } else { // RX
        // always read 32 bit words for I2S e.g.  I2S MEMS microphones
        return 32;
    }
}

// determine which DMA channel is associated to this IRQ
STATIC uint dma_map_irq_to_channel(uint irq_index) {
    for (uint ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if ((dma_irqn_get_channel_status(irq_index, ch))) {
            return ch;
        }
    }
    // This should never happen
    return -1;
}

// note:  first DMA channel is mapped to the top half of buffer, second is mapped to the bottom half
STATIC uint8_t *dma_get_buffer(machine_i2s_obj_t *i2s_obj, uint channel) {
    for (uint8_t ch = 0; ch < I2S_NUM_DMA_CHANNELS; ch++) {
        if (i2s_obj->dma_channel[ch] == channel) {
            return i2s_obj->dma_buffer + (SIZEOF_HALF_DMA_BUFFER_IN_BYTES * ch);
        }
    }
    // This should never happen
    return NULL;
}

STATIC int dma_configure(machine_i2s_obj_t *self) {
    uint8_t num_free_dma_channels = 0;
    for (uint8_t ch = 0; ch < NUM_DMA_CHANNELS; ch++) {
        if (!dma_channel_is_claimed(ch)) {
            num_free_dma_channels++;
        }
    }
    if (num_free_dma_channels < I2S_NUM_DMA_CHANNELS) {
        return -1;
    }

    for (uint8_t ch = 0; ch < I2S_NUM_DMA_CHANNELS; ch++) {
        self->dma_channel[ch] = dma_claim_unused_channel(false);
    }

    // The DMA channels are chained together.  The first DMA channel is used to access
    // the top half of the DMA buffer.  The second DMA channel accesses the bottom half of the DMA buffer.
    // With chaining, when one DMA channel has completed a data transfer, the other
    // DMA channel automatically starts a new data transfer.
    enum dma_channel_transfer_size dma_size = (dma_get_bits(self->mode, self->bits) == 16) ? DMA_SIZE_16 : DMA_SIZE_32;
    for (uint8_t ch = 0; ch < I2S_NUM_DMA_CHANNELS; ch++) {
        dma_channel_config dma_config = dma_channel_get_default_config(self->dma_channel[ch]);
        channel_config_set_transfer_data_size(&dma_config, dma_size);
        channel_config_set_chain_to(&dma_config, self->dma_channel[(ch + 1) % I2S_NUM_DMA_CHANNELS]);

        uint8_t *dma_buffer = self->dma_buffer + (SIZEOF_HALF_DMA_BUFFER_IN_BYTES * ch);
        if (self->mode == TX) {
            channel_config_set_dreq(&dma_config, pio_get_dreq(self->pio, self->sm, true));
            channel_config_set_read_increment(&dma_config, true);
            channel_config_set_write_increment(&dma_config, false);
            dma_channel_configure(self->dma_channel[ch],
                &dma_config,
                (void *)&self->pio->txf[self->sm],                      // dest = PIO TX FIFO
                dma_buffer,                                             // src = DMA buffer
                SIZEOF_HALF_DMA_BUFFER_IN_BYTES / (dma_get_bits(self->mode, self->bits) / 8),
                false);
        } else { // RX
            channel_config_set_dreq(&dma_config, pio_get_dreq(self->pio, self->sm, false));
            channel_config_set_read_increment(&dma_config, false);
            channel_config_set_write_increment(&dma_config, true);
            dma_channel_configure(self->dma_channel[ch],
                &dma_config,
                dma_buffer,                                             // dest = DMA buffer
                (void *)&self->pio->rxf[self->sm],                      // src = PIO RX FIFO
                SIZEOF_HALF_DMA_BUFFER_IN_BYTES / (dma_get_bits(self->mode, self->bits) / 8),
                false);
        }
    }

    for (uint8_t ch = 0; ch < I2S_NUM_DMA_CHANNELS; ch++) {
        dma_irqn_acknowledge_channel(self->i2s_id, self->dma_channel[ch]);  // clear pending.  e.g. from SPI
        dma_irqn_set_channel_enabled(self->i2s_id, self->dma_channel[ch], true);
    }

    return 0;
}

STATIC void dma_deinit(machine_i2s_obj_t *self) {
    for (uint8_t ch = 0; ch < I2S_NUM_DMA_CHANNELS; ch++) {
        int channel = self->dma_channel[ch];

        // unchain the channel to prevent triggering a transfer in the chained-to channel
        dma_channel_config dma_config = dma_get_channel_config(channel);
        channel_config_set_chain_to(&dma_config, channel);
        dma_channel_set_config(channel, &dma_config, false);

        dma_irqn_set_channel_enabled(self->i2s_id, channel, false);
        dma_channel_abort(channel);  // in case a transfer is in flight
        dma_channel_unclaim(channel);
    }
}

STATIC void dma_irq_handler(uint8_t irq_index) {
    int dma_channel = dma_map_irq_to_channel(irq_index);
    if (dma_channel == -1) {
        // This should never happen
        return;
    }

    machine_i2s_obj_t *self = machine_i2s_obj[irq_index];
    if (self == NULL) {
        // This should never happen
        return;
    }

    uint8_t *dma_buffer = dma_get_buffer(self, dma_channel);
    if (dma_buffer == NULL) {
        // This should never happen
        return;
    }

    if (self->mode == RX) {
        empty_dma(self, dma_buffer);
        dma_irqn_acknowledge_channel(irq_index, dma_channel);
        dma_channel_set_write_addr(dma_channel, dma_buffer, false);
    }
}

STATIC void dma_irq0_handler(void) {
    dma_irq_handler(0);
}

STATIC void dma_irq1_handler(void) {
    dma_irq_handler(1);
}


STATIC int machine_i2s_init_helper(machine_i2s_obj_t *self,
              mp_hal_pin_obj_t sck, mp_hal_pin_obj_t ws, mp_hal_pin_obj_t sd,
              i2s_mode_t i2s_mode, int8_t i2s_bits, format_t i2s_format,
              int32_t ring_buffer_len, int32_t i2s_rate) {
    //
    // ---- Check validity of arguments ----
    //

    // does WS pin follow SCK pin?
    // note:  SCK and WS are implemented as PIO sideset pins.  Sideset pins must be sequential.
    if (ws != (sck + 1)) {
        return -1;
    }

    // is Mode valid?
    if ((i2s_mode != RX) &&
        (i2s_mode != TX)) {
        return -2;
    }

    // is Bits valid?
    if ((i2s_bits != 16) &&
        (i2s_bits != 32)) {
        return -3;
    }

    // is Format valid?
    if ((i2s_format != MONO) &&
        (i2s_format != STEREO)) {
        return -4;
    }

    // is Rate valid?
    // Not checked

    // is Ibuf valid?
    if (ring_buffer_len > 0) {
        self->ring_buffer_storage = m_new(uint8_t, ring_buffer_len);
        ;
        ringbuf_init(&self->ring_buffer, self->ring_buffer_storage, ring_buffer_len);
    } else {
        return -5;
    }

    self->sck = sck;
    self->ws = ws;
    self->sd = sd;
    self->mode = i2s_mode;
    self->bits = i2s_bits;
    self->format = i2s_format;
    self->rate = i2s_rate;
    self->ibuf = ring_buffer_len;
    self->io_mode = BLOCKING;

    irq_configure(self);
    int err = pio_configure(self);
    if (err != 0) {
        return err;
    }
    gpio_configure(self);
    err = dma_configure(self);
    if (err != 0) {
        return err;
    }

    pio_sm_set_enabled(self->pio, self->sm, true);
    dma_channel_start(self->dma_channel[0]);

    return 0;
}

STATIC machine_i2s_obj_t* machine_i2s_make_new(uint8_t i2s_id,
              mp_hal_pin_obj_t sck, mp_hal_pin_obj_t ws, mp_hal_pin_obj_t sd,
              i2s_mode_t i2s_mode, int8_t i2s_bits, format_t i2s_format,
              int32_t ring_buffer_len, int32_t i2s_rate) {
    if (i2s_id >= MAX_I2S_RP2) {
        return NULL;
    }

    machine_i2s_obj_t *self;
    if (machine_i2s_obj[i2s_id] == NULL) {
        self = m_new_obj(machine_i2s_obj_t);
        machine_i2s_obj[i2s_id] = self;
        self->i2s_id = i2s_id;
    } else {
        self = machine_i2s_obj[i2s_id];
        machine_i2s_deinit(self);
    }

    if (machine_i2s_init_helper(self, sck, ws, sd, i2s_mode, i2s_bits,
            i2s_format, ring_buffer_len, i2s_rate) != 0) {
        return NULL;
    }
    return self;
}

STATIC void machine_i2s_deinit(machine_i2s_obj_t *self) {
    // use self->pio as in indication that I2S object has already been de-initialized
    if (self->pio != NULL) {
        pio_deinit(self);
        dma_deinit(self);
        irq_deinit(self);
        free(self->ring_buffer_storage);
        self->pio = NULL;  // flag object as de-initialized
    }
}

STATIC int machine_i2s_stream_read(machine_i2s_obj_t *self, void *buf_in, size_t size) {
    if (self->mode != RX) {
        printf("Here");
        return -1;
    }

    uint8_t appbuf_sample_size_in_bytes = (self->bits / 8) * (self->format == STEREO ? 2: 1);
    if (size % appbuf_sample_size_in_bytes != 0) {
        return -2;
    }

    if (size == 0) {
        return 0;
    }

    mp_buffer_info_t appbuf;
    appbuf.buf = (void *)buf_in;
    appbuf.len = size;
    uint32_t num_bytes_read = fill_appbuf_from_ringbuf(self, &appbuf);
    return num_bytes_read;
}
