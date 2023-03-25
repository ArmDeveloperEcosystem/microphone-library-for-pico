/**
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "pico/stdlib.h"
#include "pico/machine_i2s.c"
#include "pico/multicore.h"

#include "usb_microphone.h"

#define SCK 3
#define WS 4 // needs to be SCK +1
#define SD 6
#define BPS 32 // 24 is not valid in this implementation, but INMP441 outputs 24 bits samples
#define RATE 16000

// variables
uint16_t sample_buffer[SAMPLE_BUFFER_SIZE];

// callback functions
void on_usb_microphone_tx_ready();

void core1_entry() {

	machine_i2s_obj_t* i2s0 = machine_i2s_make_new(0, SCK, WS, SD, RX, BPS, MONO, /*ringbuf_len*/SIZEOF_DMA_BUFFER_IN_BYTES, RATE);
	int32_t buffer[I2S_RX_FRAME_SIZE_IN_BYTES /4];

    while (1){
		// // run the USB microphone task continuously
		for(int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
			machine_i2s_stream_read(i2s0, (void*)&buffer[0], I2S_RX_FRAME_SIZE_IN_BYTES);
			sample_buffer[i] = buffer[0]/2000; // right channel is empty, play using $ cat /dev/ttyACM0 | xxd -r -p | aplay -r16000 -c1 -fS32_BE
		}
	}
}

int main() {
	stdio_init_all();

	// initialize the USB microphone interface
	usb_microphone_init();
	usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

	multicore_launch_core1(core1_entry);

	while (1) {
		

		usb_microphone_task();
	}
}

void on_usb_microphone_tx_ready()
{
  // Callback from TinyUSB library when all data is ready
  // to be transmitted.
  //
  // Write local buffer to the USB microphone
  usb_microphone_write(sample_buffer, sizeof(sample_buffer));
}
