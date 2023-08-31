#include "stdio.h"

#include "tusb.h"
#include "pico/stdlib.h"
#include "pico/i2s_microphone.h"
#include "usb_microphone.h"

#define SCK 2
#define WS 3 // needs to be SCK +1
#define SD 6
#define BPS 16 
#define RATE 16000
#define BUTTON 15
#define SIZE_APP_BUFFER SIZEOF_DMA_BUFFER_IN_BYTES/8

uint32_t byteReads = 0;
int16_t sample_buffer[SIZE_APP_BUFFER];
machine_i2s_obj_t* i2s0;
uint8_t id_i2s = 0;

//// Callback function I2S /////
void on_i2s_samples_ready(){
    byteReads = machine_i2s_stream_read(i2s0, (void*)&sample_buffer, SIZE_APP_BUFFER); // app buffer len <= dma buffer len / 8
    printf("%d\r\n",byteReads);
}

// Callback function USB ///
void on_usb_microphone_tx_ready(){
  usb_microphone_write(sample_buffer, sizeof(sample_buffer));
}

void init(){
    stdio_init_all();
    gpio_init(BUTTON);
    gpio_set_dir(BUTTON, GPIO_IN);
    gpio_pull_up(BUTTON);
}

int main(){
    init();
    i2s0 = machine_i2s_make_new(id_i2s, SCK, WS, SD,
                                RX, BPS, MONO,
                                SIZEOF_DMA_BUFFER_IN_BYTES*2, // set ring buffer len >= dma buffer len 
                                RATE);
                                
    printf("I2S OK \n\n");
    i2s_microphone_set_samples_ready(on_i2s_samples_ready, id_i2s);

    usb_microphone_init();
    usb_microphone_set_tx_ready_handler(on_usb_microphone_tx_ready);

    while (1){
        usb_microphone_task();
    }
    
    return 0;
}