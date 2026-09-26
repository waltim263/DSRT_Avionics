# DSRT_Avionics

  First iteration of our STM32 project being used to interface with several communication and data collection modules.

## Wiring (NUCLEO-F446RE)

All modules are powered from 3V3 (CN7-16) and GND (CN7-20). Firmware lives in
`avionics_interface_v0/App` (drivers) and is started from `main.c`.

| Module | Signal | MCU pin | Nucleo connector |
|---|---|---|---|
| **SPI1** (mode 3, 1 MHz) | SCK / MISO / MOSI | PB3 / PA6 / PA7 | CN10-31 / CN10-13 / CN10-15 |
| BNO085 IMU | CS | PA4 | CN7-32 |
| | INT | PC7 | CN10-19 |
| | RST | PA8 | CN10-23 |
| | P0 (WAKE) | PB10 | CN10-25 |
| | P1 | 3V3 | CN7-16 |
| BMP388 barometer | CS | PA15 | CN7-17 |
| | INT | not used | |
| **SPI2** (mode 0, 4 MHz) | SCK / MISO / MOSI | PA9 / PB14 / PB15 | CN10-21 / CN10-28 / CN10-26 |
| RFM95 LoRa radio | CS | PB12 | CN10-16 |
| | G0 (DIO0) | PB5 | CN10-29 |
| | RST | PC6 | CN10-4 |
| **SDIO** (4-bit) | CLK / CMD | PB2 / PD2 | CN10-22 / CN7-4 |
| SD card breakout | D0 / D1 / D2 / D3 | PC8 / PB0 / PB1 / PC11 | CN10-2 / CN7-34 / CN10-24 / CN7-2 |

Radio packets use the Adafruit/RadioHead defaults (915 MHz, SF7, BW 125 kHz,
CR 4/5), so a Feather or Raspberry Pi running Adafruit's RFM9x library can be
the ground station. The payload layout is documented in `App/Inc/avionics.h`.
