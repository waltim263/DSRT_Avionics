/**
 * @file    avionics.c
 * @brief   Sensor bring-up, sampling loop and radio telemetry.
 */
#include "avionics.h"
#include "main.h"
#include "spi.h"
#include "bmp388.h"
#include "bno085.h"
#include "rfm95.h"

#define GROUND_SAMPLES 25U

avionics_telemetry_t g_telemetry;

static bmp388_t baro;
static bno085_t imu;
static rfm95_t radio;
static uint8_t status;
static float ground_pressure_pa = 101325.0f;
static uint32_t last_tx_ms;

static int16_t to_fixed(float v, float scale)
{
  float x = v * scale;
  if (x > 32767.0f) {
    return 32767;
  }
  if (x < -32768.0f) {
    return -32768;
  }
  return (int16_t)x;
}

/* Average a second of pressure readings on the pad as the AGL reference. */
static void calibrate_ground_pressure(void)
{
  float sum = 0.0f;
  uint32_t n = 0;
  uint32_t start = HAL_GetTick();
  bmp388_data_t d;

  while (n < GROUND_SAMPLES && HAL_GetTick() - start < 2000U) {
    if (bmp388_data_ready(&baro) && bmp388_read(&baro, &d) == HAL_OK) {
      sum += d.pressure_pa;
      n++;
    }
  }
  if (n > 0U) {
    ground_pressure_pa = sum / (float)n;
  }
}

void avionics_init(void)
{
  status = 0;

  if (bmp388_init(&baro, &hspi1, BMP388_CS_GPIO_Port, BMP388_CS_Pin) == HAL_OK) {
    status |= AVIONICS_STATUS_BMP388;
    calibrate_ground_pressure();
  }

  if (bno085_init(&imu, &hspi1, BNO085_CS_GPIO_Port, BNO085_CS_Pin,
                  BNO085_INT_GPIO_Port, BNO085_INT_Pin,
                  BNO085_RST_GPIO_Port, BNO085_RST_Pin,
                  BNO085_WAKE_GPIO_Port, BNO085_WAKE_Pin) == HAL_OK &&
      bno085_enable_report(&imu, BNO085_REPORT_ROTATION_VECTOR, AVIONICS_IMU_INTERVAL_US) == HAL_OK &&
      bno085_enable_report(&imu, BNO085_REPORT_ACCELEROMETER, AVIONICS_IMU_INTERVAL_US) == HAL_OK &&
      bno085_enable_report(&imu, BNO085_REPORT_GYROSCOPE, AVIONICS_IMU_INTERVAL_US) == HAL_OK) {
    status |= AVIONICS_STATUS_BNO085;
  }

  if (rfm95_init(&radio, &hspi2, RFM95_CS_GPIO_Port, RFM95_CS_Pin,
                 RFM95_RST_GPIO_Port, RFM95_RST_Pin,
                 RFM95_G0_GPIO_Port, RFM95_G0_Pin,
                 AVIONICS_RADIO_FREQ_MHZ, AVIONICS_RADIO_TX_DBM) == HAL_OK) {
    status |= AVIONICS_STATUS_RFM95;
  }

  g_telemetry.status = status;
  last_tx_ms = HAL_GetTick();
}

void avionics_run(void)
{
  avionics_telemetry_t *t = &g_telemetry;

  if ((status & AVIONICS_STATUS_BNO085) != 0U && bno085_service(&imu)) {
    const bno085_data_t *d = &imu.data;
    t->quat[0] = to_fixed(d->rotation.i, 16384.0f);
    t->quat[1] = to_fixed(d->rotation.j, 16384.0f);
    t->quat[2] = to_fixed(d->rotation.k, 16384.0f);
    t->quat[3] = to_fixed(d->rotation.real, 16384.0f);
    t->accel[0] = to_fixed(d->accel_mps2.x, 256.0f);
    t->accel[1] = to_fixed(d->accel_mps2.y, 256.0f);
    t->accel[2] = to_fixed(d->accel_mps2.z, 256.0f);
    t->gyro[0] = to_fixed(d->gyro_rps.x, 512.0f);
    t->gyro[1] = to_fixed(d->gyro_rps.y, 512.0f);
    t->gyro[2] = to_fixed(d->gyro_rps.z, 512.0f);
  }

  if ((status & AVIONICS_STATUS_BMP388) != 0U && bmp388_data_ready(&baro)) {
    bmp388_data_t d;
    if (bmp388_read(&baro, &d) == HAL_OK) {
      t->pressure_pa = d.pressure_pa;
      t->altitude_agl_m = bmp388_altitude_m(d.pressure_pa, ground_pressure_pa);
      t->temperature_cc = to_fixed(d.temperature_c, 100.0f);
    }
  }

  if ((status & AVIONICS_STATUS_RFM95) != 0U) {
    uint32_t now = HAL_GetTick();
    if (!rfm95_poll(&radio) && now - last_tx_ms >= AVIONICS_TELEMETRY_PERIOD_MS) {
      last_tx_ms = now;
      t->time_ms = now;
      t->status = status;
      (void)rfm95_send(&radio, (const uint8_t *)t, sizeof(*t));
    }
  }
}
