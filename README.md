# Microphone Library for Pico

Capture audio from a microphone on your [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/) or any [RP2040](https://www.raspberrypi.org/products/rp2040/) based board. 🎤

## Hardware

* RP2040 board
  
  * [Raspberry Pi Pico](https://www.raspberrypi.org/products/raspberry-pi-pico/)

* Microphones
  
  * I2S
    
    * [INMP441 I2S Omnidirectional Microphone](https://hshop.vn/products/cam-bien-am-thanh-inmp441-i2s-omnidirectional-microphone)
  
  * Analog
    
    * [Electret Microphone Amplifier - MAX9814 with Auto Gain Control](https://www.adafruit.com/product/1713) 
  
  * PDM
    
    * [Adafruit PDM MEMS Microphone Breakout](https://www.adafruit.com/product/3492)

### Default Pinout

#### I2S Microphone

| Raspberry Pi Pico / RP2040 | Analog Microphone |
| -------------------------- | ----------------- |
| 3.3V                       | VCC               |
| GND                        | GND               |
| GPIO 2                     | SCK               |
| GPIO 3                     | WS                |
| GPIO 6                     | SD                |
| GND                        | L/R               |

#### Analog Microphone

| Raspberry Pi Pico / RP2040 | Analog Microphone |
| -------------------------- | ----------------- |
| 3.3V                       | VCC               |
| GND                        | GND               |
| GPIO 26                    | OUT               |

#### PDM Microphone

| Raspberry Pi Pico / RP2040 | PDM Microphone |
| -------------------------- | -------------- |
| 3.3V                       | VCC            |
| GND                        | GND            |
| GND                        | SEL            |
| GPIO 2                     | DAT            |
| GPIO 3                     | CLK            |

GPIO pins are configurable in examples or API.

## Examples

[Git Examples about microphone](https://github.com/dattran-itrvn/microphone-library-for-pico/tree/Binh_Dev)

#### Set up Pico SDK

[Set up the Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)

1. Set `PICO_SDK_PATH`
   
   ```sh
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

2. Create `build` dir, run `cmake` and `make`:
   
   ```
   mkdir build
   cd build
   cmake .. -DPICO_BOARD=pico
   make
   ```

3. Copy example `.uf2` to Pico when in BOOT mode.

## License

[Apache-2.0 License](LICENSE)

## Acknowledgements

##### - Mic Analog or Mic PDM

The [OpenPDM2PCM](https://os.mbed.com/teams/ST/code/X_NUCLEO_CCA02M1//file/53f8b511f2a1/Middlewares/OpenPDM2PCM/) library is used to filter raw PDM data into PCM. The [TinyUSB](https://github.com/hathach/tinyusb) library is used in the `usb_microphone` example.

---

##### - Mic I2S

Get data raw using I2S protocol and using algorithm fill buffer to decode audio. 

Buffer App = Buffer Ring / 4 = Buffer DMA / 8. 

To record data, using [TinyUSB]([GitHub - hathach/tinyusb: An open source cross-platform USB stack for embedded system](https://github.com/hathach/tinyusb)) library is used in the `usb_microphone` example. Buffer output <= 32 bytes <=> 256 bytes in DMA buffer

Using [Audacity software]([Download | Audacity ®](https://www.audacityteam.org/download/) get audio output 

---
