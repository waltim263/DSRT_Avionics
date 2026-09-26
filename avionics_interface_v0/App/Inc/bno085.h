/**
 * @file    bno085.h
 * @brief   CEVA/Hillcrest BNO085 9-DOF IMU, SHTP / SH-2 protocol over SPI.
 *
 * Wiring (see Pin_Layout.xlsx): SPI1 (mode 3, <= 3 MHz), CS = PA4,
 * INT = PC7, RST = PA8, P0/WAKE = PB10, P1 tied to 3.3 V.
 * P0 and P1 must both be high when the chip leaves reset to select SPI;
 * the WAKE GPIO idles high so this holds.
 *
 * The BNO085 cannot be polled: it pulls INT low when it has data, and the
 * host must then read it. Call bno085_service() often from the main loop
 * (at least as often as the fastest report rate).
 */
#ifndef BNO085_H
#define BNO085_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* SH-2 sensor report IDs (SH-2 Reference Manual, section 6.5). */
#define BNO085_REPORT_ACCELEROMETER        0x01U
#define BNO085_REPORT_GYROSCOPE            0x02U
#define BNO085_REPORT_MAGNETIC_FIELD       0x03U
#define BNO085_REPORT_LINEAR_ACCELERATION  0x04U
#define BNO085_REPORT_ROTATION_VECTOR      0x05U
#define BNO085_REPORT_GRAVITY              0x06U
#define BNO085_REPORT_GAME_ROTATION_VECTOR 0x08U

typedef struct {
  float x, y, z;
} bno085_vec3_t;

typedef struct {
  float i, j, k, real;
} bno085_quat_t;

/** Latest value of every report the driver understands. */
typedef struct {
  bno085_vec3_t accel_mps2;        /* includes gravity */
  bno085_vec3_t linear_accel_mps2; /* gravity removed */
  bno085_vec3_t gravity_mps2;
  bno085_vec3_t gyro_rps;          /* calibrated, rad/s */
  bno085_vec3_t mag_ut;            /* calibrated, uT */
  bno085_quat_t rotation;          /* fused orientation (accel+gyro+mag) */
  float rotation_accuracy_rad;     /* heading accuracy estimate */
  bno085_quat_t game_rotation;     /* fused orientation without the magnetometer */
  uint32_t updated;                /* bit (1 << report ID) set for each report received
                                     during the last bno085_service() call */
} bno085_data_t;

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *int_port;
  uint16_t int_pin;
  GPIO_TypeDef *rst_port;
  uint16_t rst_pin;
  GPIO_TypeDef *wake_port;
  uint16_t wake_pin;
  uint8_t seq[6];                 /* per-channel SHTP sequence numbers */
  uint32_t enabled_interval_us[32]; /* reports to restore after a sensor reset */
  bool reset_seen;
  bno085_data_t data;
} bno085_t;

/** Hardware reset the sensor and wait for it to report that it is ready. */
HAL_StatusTypeDef bno085_init(bno085_t *dev, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin,
                              GPIO_TypeDef *int_port, uint16_t int_pin,
                              GPIO_TypeDef *rst_port, uint16_t rst_pin,
                              GPIO_TypeDef *wake_port, uint16_t wake_pin);

/**
 * Ask the sensor to send a report periodically.
 * @param report_id  one of BNO085_REPORT_*
 * @param interval_us  report period, e.g. 10000 for 100 Hz
 */
HAL_StatusTypeDef bno085_enable_report(bno085_t *dev, uint8_t report_id, uint32_t interval_us);

/**
 * Read every packet the sensor has queued and update dev->data.
 * @return true if at least one sensor report was received
 */
bool bno085_service(bno085_t *dev);

#endif /* BNO085_H */
