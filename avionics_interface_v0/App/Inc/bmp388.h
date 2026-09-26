/**
 * @file    bmp388.h
 * @brief   Bosch BMP388 barometric pressure / temperature sensor, SPI driver.
 *
 * Wiring (see Pin_Layout.xlsx): SPI1, CS = PA15.
 * The BMP388 accepts SPI mode 0 or mode 3 (it samples SCK when CS falls),
 * so it shares SPI1 with the BNO085, which requires mode 3.
 */
#ifndef BMP388_H
#define BMP388_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  /* Calibration coefficients, already scaled to floating point (datasheet 9.1). */
  float t1, t2, t3;
  float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
} bmp388_calib_t;

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  bmp388_calib_t calib;
} bmp388_t;

typedef struct {
  float pressure_pa;
  float temperature_c;
} bmp388_data_t;

/**
 * Reset the sensor, check the chip ID, load calibration and start continuous
 * (normal mode) sampling at 50 Hz with x8 pressure / x1 temperature
 * oversampling and IIR coefficient 3.
 */
HAL_StatusTypeDef bmp388_init(bmp388_t *dev, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin);

/** True when a new pressure and temperature conversion is ready. */
bool bmp388_data_ready(bmp388_t *dev);

/** Read and compensate the latest pressure and temperature. */
HAL_StatusTypeDef bmp388_read(bmp388_t *dev, bmp388_data_t *out);

/**
 * Altitude in metres above the reference pressure (barometric formula).
 * Pass the pressure measured on the pad to get height above ground level,
 * or 101325 Pa for an approximate altitude above sea level.
 */
float bmp388_altitude_m(float pressure_pa, float reference_pa);

#endif /* BMP388_H */
