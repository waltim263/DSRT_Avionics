/**
 * @file    bno085.c
 * @brief   BNO085 driver: SHTP transport over SPI plus the subset of SH-2
 *          needed to enable and decode the common sensor reports.
 *          References: BNO08X datasheet (1000-3927), SH-2 Reference Manual
 *          (1000-3625) and SH-2 SHTP Reference Manual (1000-3535).
 */
#include "bno085.h"
#include <string.h>

/* SHTP channels. */
#define CH_COMMAND          0U
#define CH_EXECUTABLE       1U
#define CH_CONTROL          2U
#define CH_REPORTS          3U
#define CH_WAKE_REPORTS     4U

#define SHTP_HEADER_LEN     4U
#define RX_BUF_LEN          512U /* the start-up advertisement is ~280 bytes */
#define CHUNK_LEN           64U

#define EXEC_RESET_COMPLETE 0x01U
#define SH2_SET_FEATURE     0xFDU
#define SH2_TIMESTAMP_REBASE 0xFAU
#define SH2_BASE_TIMESTAMP  0xFBU

#define SPI_TIMEOUT_MS      20U
#define WAKE_TIMEOUT_MS     200U
#define RESET_TIMEOUT_MS    1000U
#define MAX_READS_PER_SERVICE 16U

static uint8_t rx_buf[RX_BUF_LEN];

static void process_packet(bno085_t *dev, uint8_t channel, const uint8_t *p, uint16_t len);

static bool int_asserted(bno085_t *dev)
{
  return HAL_GPIO_ReadPin(dev->int_port, dev->int_pin) == GPIO_PIN_RESET;
}

static bool wait_for_int(bno085_t *dev, uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();

  while (!int_asserted(dev)) {
    if (HAL_GetTick() - start >= timeout_ms) {
      return false;
    }
  }
  return true;
}

/**
 * One full-duplex SHTP transaction. Sends @p tx (a complete SHTP packet, or
 * nothing when tx_len is 0) and at the same time receives whatever packet
 * the sensor has queued, so no incoming data is lost when writing.
 */
static HAL_StatusTypeDef shtp_transfer(bno085_t *dev, const uint8_t *tx, uint16_t tx_len)
{
  uint8_t tx_chunk[CHUNK_LEN];
  uint8_t rx_chunk[CHUNK_LEN];
  HAL_StatusTypeDef st = HAL_OK;

  if (!int_asserted(dev)) {
    if (tx_len == 0U) {
      return HAL_OK; /* nothing to read */
    }
    /* Ask the sensor to wake up and accept a write. */
    HAL_GPIO_WritePin(dev->wake_port, dev->wake_pin, GPIO_PIN_RESET);
    bool ready = wait_for_int(dev, WAKE_TIMEOUT_MS);
    if (!ready) {
      HAL_GPIO_WritePin(dev->wake_port, dev->wake_pin, GPIO_PIN_SET);
      return HAL_TIMEOUT;
    }
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dev->wake_port, dev->wake_pin, GPIO_PIN_SET);

  /* Exchange headers first to learn how long the incoming packet is. */
  memset(tx_chunk, 0, SHTP_HEADER_LEN);
  if (tx_len >= SHTP_HEADER_LEN) {
    memcpy(tx_chunk, tx, SHTP_HEADER_LEN);
  }
  st = HAL_SPI_TransmitReceive(dev->hspi, tx_chunk, rx_buf, SHTP_HEADER_LEN, SPI_TIMEOUT_MS);

  uint16_t rx_len = (uint16_t)(rx_buf[0] | (rx_buf[1] << 8)) & 0x7FFFU;
  if (rx_buf[0] == 0xFFU && rx_buf[1] == 0xFFU) {
    rx_len = 0; /* bus idle / no sensor */
  }
  uint16_t total = (rx_len > tx_len) ? rx_len : tx_len;

  for (uint16_t pos = SHTP_HEADER_LEN; st == HAL_OK && pos < total; ) {
    uint16_t n = (uint16_t)(total - pos);
    if (n > CHUNK_LEN) {
      n = CHUNK_LEN;
    }
    for (uint16_t i = 0; i < n; i++) {
      tx_chunk[i] = (pos + i < tx_len) ? tx[pos + i] : 0x00U;
    }
    st = HAL_SPI_TransmitReceive(dev->hspi, tx_chunk, rx_chunk, n, SPI_TIMEOUT_MS);
    /* Keep what fits in rx_buf; anything longer is clocked out and dropped. */
    for (uint16_t i = 0; i < n && pos + i < RX_BUF_LEN; i++) {
      rx_buf[pos + i] = rx_chunk[i];
    }
    pos = (uint16_t)(pos + n);
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  if (st == HAL_OK && rx_len > SHTP_HEADER_LEN) {
    uint16_t kept = (rx_len > RX_BUF_LEN) ? RX_BUF_LEN : rx_len;
    process_packet(dev, rx_buf[2], &rx_buf[SHTP_HEADER_LEN], (uint16_t)(kept - SHTP_HEADER_LEN));
  }
  return st;
}

static HAL_StatusTypeDef shtp_send(bno085_t *dev, uint8_t channel, const uint8_t *payload, uint16_t len)
{
  uint8_t pkt[SHTP_HEADER_LEN + 32];
  uint16_t total = (uint16_t)(len + SHTP_HEADER_LEN);

  if (len > sizeof(pkt) - SHTP_HEADER_LEN) {
    return HAL_ERROR;
  }
  pkt[0] = (uint8_t)(total & 0xFFU);
  pkt[1] = (uint8_t)(total >> 8);
  pkt[2] = channel;
  pkt[3] = dev->seq[channel]++;
  memcpy(&pkt[SHTP_HEADER_LEN], payload, len);
  return shtp_transfer(dev, pkt, total);
}

static int16_t le16(const uint8_t *p)
{
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void read_vec3(bno085_vec3_t *v, const uint8_t *p, float scale)
{
  v->x = (float)le16(&p[0]) * scale;
  v->y = (float)le16(&p[2]) * scale;
  v->z = (float)le16(&p[4]) * scale;
}

static void read_quat(bno085_quat_t *q, const uint8_t *p)
{
  const float q14 = 1.0f / 16384.0f;
  q->i = (float)le16(&p[0]) * q14;
  q->j = (float)le16(&p[2]) * q14;
  q->k = (float)le16(&p[4]) * q14;
  q->real = (float)le16(&p[6]) * q14;
}

/** Length in bytes of an input report, or 0 if the driver does not know it. */
static uint16_t report_length(uint8_t id)
{
  switch (id) {
    case SH2_BASE_TIMESTAMP:
    case SH2_TIMESTAMP_REBASE:              return 5;
    case BNO085_REPORT_ACCELEROMETER:
    case BNO085_REPORT_GYROSCOPE:
    case BNO085_REPORT_MAGNETIC_FIELD:
    case BNO085_REPORT_LINEAR_ACCELERATION:
    case BNO085_REPORT_GRAVITY:             return 10;
    case BNO085_REPORT_GAME_ROTATION_VECTOR: return 12;
    case BNO085_REPORT_ROTATION_VECTOR:
    case 0x09U: /* geomagnetic rotation vector */ return 14;
    case 0x07U: /* uncalibrated gyroscope */
    case 0x0FU: /* uncalibrated magnetic field */ return 16;
    default:                                return 0;
  }
}

static void process_reports(bno085_t *dev, const uint8_t *p, uint16_t len)
{
  bno085_data_t *d = &dev->data;
  uint16_t pos = 0;

  while (pos < len) {
    uint8_t id = p[pos];
    uint16_t rlen = report_length(id);
    if (rlen == 0U || pos + rlen > len) {
      return; /* unknown report: its length is unknown, so stop here */
    }
    /* Sensor reports: [id, seq, status, delay, data...] */
    const uint8_t *r = &p[pos + 4];
    switch (id) {
      case BNO085_REPORT_ACCELEROMETER:       read_vec3(&d->accel_mps2, r, 1.0f / 256.0f); break;
      case BNO085_REPORT_LINEAR_ACCELERATION: read_vec3(&d->linear_accel_mps2, r, 1.0f / 256.0f); break;
      case BNO085_REPORT_GRAVITY:             read_vec3(&d->gravity_mps2, r, 1.0f / 256.0f); break;
      case BNO085_REPORT_GYROSCOPE:           read_vec3(&d->gyro_rps, r, 1.0f / 512.0f); break;
      case BNO085_REPORT_MAGNETIC_FIELD:      read_vec3(&d->mag_ut, r, 1.0f / 16.0f); break;
      case BNO085_REPORT_ROTATION_VECTOR:
        read_quat(&d->rotation, r);
        d->rotation_accuracy_rad = (float)le16(&r[8]) / 4096.0f;
        break;
      case BNO085_REPORT_GAME_ROTATION_VECTOR: read_quat(&d->game_rotation, r); break;
      default: break;
    }
    if (id < 32U) {
      d->updated |= (1UL << id);
    }
    pos = (uint16_t)(pos + rlen);
  }
}

static void process_packet(bno085_t *dev, uint8_t channel, const uint8_t *p, uint16_t len)
{
  switch (channel) {
    case CH_EXECUTABLE:
      if (len >= 1U && p[0] == EXEC_RESET_COMPLETE) {
        dev->reset_seen = true;
      }
      break;
    case CH_REPORTS:
    case CH_WAKE_REPORTS:
      process_reports(dev, p, len);
      break;
    default:
      /* Advertisement (channel 0) and control responses are not needed. */
      break;
  }
}

HAL_StatusTypeDef bno085_init(bno085_t *dev, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin,
                              GPIO_TypeDef *int_port, uint16_t int_pin,
                              GPIO_TypeDef *rst_port, uint16_t rst_pin,
                              GPIO_TypeDef *wake_port, uint16_t wake_pin)
{
  memset(dev, 0, sizeof(*dev));
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->int_port = int_port;
  dev->int_pin = int_pin;
  dev->rst_port = rst_port;
  dev->rst_pin = rst_pin;
  dev->wake_port = wake_port;
  dev->wake_pin = wake_pin;

  /* WAKE (PS0) must be high as the chip leaves reset to select SPI mode. */
  HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(wake_port, wake_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(rst_port, rst_pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(rst_port, rst_pin, GPIO_PIN_SET);

  /* Drain the start-up packets until the "reset complete" message. */
  uint32_t start = HAL_GetTick();
  while (!dev->reset_seen) {
    uint32_t elapsed = HAL_GetTick() - start;
    if (elapsed >= RESET_TIMEOUT_MS || !wait_for_int(dev, RESET_TIMEOUT_MS - elapsed)) {
      return HAL_TIMEOUT;
    }
    if (shtp_transfer(dev, NULL, 0) != HAL_OK) {
      return HAL_ERROR;
    }
  }
  dev->reset_seen = false;

  /* Read anything else queued after the reset message. */
  (void)bno085_service(dev);
  return HAL_OK;
}

HAL_StatusTypeDef bno085_enable_report(bno085_t *dev, uint8_t report_id, uint32_t interval_us)
{
  uint8_t cmd[17] = {0};

  cmd[0] = SH2_SET_FEATURE;
  cmd[1] = report_id;
  /* cmd[2] feature flags, cmd[3..4] change sensitivity: unused */
  cmd[5] = (uint8_t)(interval_us);
  cmd[6] = (uint8_t)(interval_us >> 8);
  cmd[7] = (uint8_t)(interval_us >> 16);
  cmd[8] = (uint8_t)(interval_us >> 24);
  /* cmd[9..12] batch interval, cmd[13..16] sensor-specific: unused */

  if (report_id < 32U) {
    dev->enabled_interval_us[report_id] = interval_us;
  }
  return shtp_send(dev, CH_CONTROL, cmd, sizeof(cmd));
}

bool bno085_service(bno085_t *dev)
{
  dev->data.updated = 0;

  for (uint32_t i = 0; i < MAX_READS_PER_SERVICE && int_asserted(dev); i++) {
    if (shtp_transfer(dev, NULL, 0) != HAL_OK) {
      break;
    }
  }

  /* The sensor forgets its configuration if it resets on its own. */
  if (dev->reset_seen) {
    dev->reset_seen = false;
    for (uint8_t id = 0; id < 32U; id++) {
      if (dev->enabled_interval_us[id] != 0U) {
        (void)bno085_enable_report(dev, id, dev->enabled_interval_us[id]);
      }
    }
  }

  return dev->data.updated != 0U;
}
