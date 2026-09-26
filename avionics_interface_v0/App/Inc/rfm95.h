/**
 * @file    rfm95.h
 * @brief   HopeRF RFM95/96/98 (Semtech SX1276) LoRa radio, SPI driver.
 *
 * Wiring (see Pin_Layout.xlsx): SPI2 (mode 0), CS = PB12, G0/DIO0 = PB5,
 * RST = PC6.
 *
 * Radio settings match the RadioHead / Adafruit RFM9x library defaults
 * (BW 125 kHz, CR 4/5, SF7, CRC on, 8 symbol preamble, sync word 0x12) and
 * every packet starts with RadioHead's 4 byte header (to, from, id, flags).
 * A ground station running Adafruit's Arduino or CircuitPython RFM9x
 * library on the same frequency can therefore receive these packets
 * directly.
 */
#ifndef RFM95_H
#define RFM95_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** Largest payload after the 4 byte RadioHead header (255 byte FIFO limit). */
#define RFM95_MAX_PAYLOAD 251U

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *rst_port;
  uint16_t rst_pin;
  GPIO_TypeDef *dio0_port;
  uint16_t dio0_pin;
  bool tx_busy;
  uint8_t tx_id;
} rfm95_t;

/**
 * Reset the radio, check the silicon version and configure LoRa mode.
 * @param freq_mhz  carrier frequency, e.g. 915.0f or 433.0f; it must match
 *                  the module variant and the ground station
 * @param tx_dbm    transmit power, 5 to 20 dBm (PA_BOOST)
 */
HAL_StatusTypeDef rfm95_init(rfm95_t *dev, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin,
                             GPIO_TypeDef *rst_port, uint16_t rst_pin,
                             GPIO_TypeDef *dio0_port, uint16_t dio0_pin,
                             float freq_mhz, int8_t tx_dbm);

/**
 * Start transmitting a packet and return immediately. Returns HAL_BUSY
 * while a previous packet is still on air. Call rfm95_poll() from the main
 * loop to find out when the transmission has finished.
 */
HAL_StatusTypeDef rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len);

/** Service the radio; returns true while a transmission is in progress. */
bool rfm95_poll(rfm95_t *dev);

/** Switch to continuous receive mode (e.g. to listen for ground commands). */
HAL_StatusTypeDef rfm95_start_rx(rfm95_t *dev);

/**
 * Fetch a received packet, if any, while in receive mode.
 * @return payload length (header stripped), 0 if nothing was received,
 *         -1 if a packet failed its CRC
 */
int rfm95_receive(rfm95_t *dev, uint8_t *buf, uint8_t buf_len, int16_t *rssi_dbm);

#endif /* RFM95_H */
