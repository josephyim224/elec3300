# elec3300

## progress
| urgency | difficulty | task | description | done? |
| --- | --- | --- | --- | --- |
| - | - | design pcb | | v |
| - | - | solder pcb | | v |
| - | - | mech design | | car(v) |
| 0 | 0 | button | button interrupt | |
| 0 | 0 | led | led display | v |
| 1 | 1 | motor driver | (car) pwm & direction control | v |
| 1 | 2 | uart | communicate | |
| 1 | 2 | i2c | communicate | car(v), remote(v) |
| 1 | 3 | mpu6050 (i2c) | port and simplify Adafruit version | car(v) |
| 2 | 4 | ssd1306 lcd (i2c) | display line and text (opt) | car(v), remote(v) |
| 3 | 2 | ws2812b RGB led | display R/G/B first, either bitbang or spi (opt) | car(v), remote(v) |
| 4 | 1 | vibration motor | (remote) pwm output | |
| 4 | 2 | 24c02 eeprom (i2c) | read and write data | |
| 4 | ? | encoder | (car) encoder counter | v |
| 5 | 4 | gy530 TOF (i2c) | get distance | |


## car_hw

- see output/schematic.pdf

## car_sw

- use STM32CubeIDE

## car_mech

- see output/schematic.pdf

## remote_hw

- see output/schematic.pdf

## remote_sw

- Reference https://www.codementor.io/@hbendali/getting-started-with-stm8-development-tools-on-gnu-linux-zu59yo35x
- Template from https://my.st.com/content/my_st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm8-embedded-software/stsw-stm8069.html
- Library from https://github.com/stecman/stm8s-sdcc
- stm8flash https://github.com/vdudouyt/stm8flash
1. git clone the repo
2. ```sudo apt-get install libusb-1.0-0-dev sdcc -y```
3. make && sudo make install
4. ```bash ./flash.sh``` for complilation and flash

## remote_mech

- see output/schematic.pdf
