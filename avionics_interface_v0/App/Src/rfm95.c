/**
 * @file    rfm95.c
 * @brief   SX1276 LoRa driver. Register map from the Semtech SX1276 datasheet.
 */
#include "rfm95.h"

#define REG_FIFO              0x00U
#define REG_OP_MODE           0x01U
#define REG_FRF_MSB           0x06U
#define REG_FRF_MID           0x07U
#define REG_FRF_LSB           0x08U
#define REG_PA_CONFIG         0x09U
#define REG_OCP               0x0BU
#define REG_FIFO_ADDR_PTR     0x0DU
#define REG_FIFO_TX_BASE      0x0EU
#define REG_FIFO_RX_BASE      0x0FU
#define REG_FIFO_RX_CURRENT   0x10U
#define REG_IRQ_FLAGS         0x12U
#define REG_RX_NB_BYTES       0x13U
#define REG_PKT_RSSI          0x1AU
#define REG_MODEM_CONFIG1     0x1DU
#define REG_MODEM_CONFIG2     0x1EU
#define REG_PREAMBLE_MSB      0x20U
#define REG_PREAMBLE_LSB      0x21U
#define REG_PAYLOAD_LENGTH    0x22U
#define REG_MODEM_CONFIG3     0x26U
#define REG_DIO_MAPPING1      0x40U
#define REG_VERSION           0x42U
#define REG_PA_DAC            0x4DU

#define MODE_LONG_RANGE       0x80U
#define MODE_SLEEP            0x00U
#define MODE_STDBY            0x01U
#define MODE_TX               0x03U
#define MODE_RX_CONTINUOUS    0x05U

#define IRQ_RX_DONE           0x40U
#define IRQ_CRC_ERROR         0x20U
#define IRQ_TX_DONE           0x08U

#define DIO0_RX_DONE          0x00U
#define DIO0_TX_DONE          0x40U

#define PA_BOOST              0x80U

#define SX1276_VERSION        0x12U
#define FXOSC_HZ              32000000.0f
#define RH_BROADCAST          0xFFU
#define RH_HEADER_LEN         4U
#define SPI_TIMEOUT_MS        10U

static void cs_low(rfm95_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void cs_high(rfm95_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef write_burst(rfm95_t *dev, uint8_t reg, const uint8_t *data, uint8_t len)
{
  uint8_t addr = (uint8_t)(reg | 0x80U);
  HAL_StatusTypeDef st;

  cs_low(dev);
  st = HAL_SPI_Transmit(dev->hspi, &addr, 1, SPI_TIMEOUT_MS);
  if (st == HAL_OK && len > 0U) {
    st = HAL_SPI_Transmit(dev->hspi, (uint8_t *)data, len, SPI_TIMEOUT_MS);
  }
  cs_high(dev);
  return st;
}

static HAL_StatusTypeDef read_burst(rfm95_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
  uint8_t addr = (uint8_t)(reg & 0x7FU);
  HAL_StatusTypeDef st;

  cs_low(dev);
  st = HAL_SPI_Transmit(dev->hspi, &addr, 1, SPI_TIMEOUT_MS);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(dev->hspi, data, len, SPI_TIMEOUT_MS);
  }
  cs_high(dev);
  return st;
}

static HAL_StatusTypeDef write_reg(rfm95_t *dev, uint8_t reg, uint8_t val)
{
  return write_burst(dev, reg, &val, 1);
}

static uint8_t read_reg(rfm95_t *dev, uint8_t reg)
{
  uint8_t val = 0;
  (void)read_burst(dev, reg, &val, 1);
  return val;
}

static HAL_StatusTypeDef set_mode(rfm95_t *dev, uint8_t mode)
{
  return write_reg(dev, REG_OP_MODE, MODE_LONG_RANGE | mode);
}

static void set_tx_power(rfm95_t *dev, int8_t dbm)
{
  /* RFM95 modules only have the PA_BOOST pin connected. */
  if (dbm > 20) {
    dbm = 20;
  } else if (dbm < 5) {
    dbm = 5;
  }
  if (dbm > 17) {
    write_reg(dev, REG_PA_DAC, 0x87U); /* +20 dBm high power mode */
    write_reg(dev, REG_OCP, 0x31U);    /* over-current limit 140 mA */
    dbm -= 3;
  } else {
    write_reg(dev, REG_PA_DAC, 0x84U);
    write_reg(dev, REG_OCP, 0x2BU);    /* default 100 mA */
  }
  write_reg(dev, REG_PA_CONFIG, (uint8_t)(PA_BOOST | (uint8_t)(dbm - 2)));
}

HAL_StatusTypeDef rfm95_init(rfm95_t *dev, SPI_HandleTypeDef *hspi,
                             GPIO_TypeDef *cs_port, uint16_t cs_pin,
                             GPIO_TypeDef *rst_port, uint16_t rst_pin,
                             GPIO_TypeDef *dio0_port, uint16_t dio0_pin,
                             float freq_mhz, int8_t tx_dbm)
{
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->rst_port = rst_port;
  dev->rst_pin = rst_pin;
  dev->dio0_port = dio0_port;
  dev->dio0_pin = dio0_pin;
  dev->tx_busy = false;
  dev->tx_id = 0;
  cs_high(dev);

  /* Hardware reset: hold low >100 us, then wait 5 ms for the chip. */
  HAL_GPIO_WritePin(rst_port, rst_pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(rst_port, rst_pin, GPIO_PIN_SET);
  HAL_Delay(10);

  if (read_reg(dev, REG_VERSION) != SX1276_VERSION) {
    return HAL_ERROR;
  }

  /* LoRa mode can only be selected while asleep. */
  set_mode(dev, MODE_SLEEP);
  HAL_Delay(10);
  if (read_reg(dev, REG_OP_MODE) != (MODE_LONG_RANGE | MODE_SLEEP)) {
    return HAL_ERROR;
  }

  /* Use the whole 256 byte FIFO for either direction. */
  write_reg(dev, REG_FIFO_TX_BASE, 0x00U);
  write_reg(dev, REG_FIFO_RX_BASE, 0x00U);

  set_mode(dev, MODE_STDBY);

  /* BW 125 kHz, CR 4/5, explicit header; SF7, CRC on; AGC on. */
  write_reg(dev, REG_MODEM_CONFIG1, 0x72U);
  write_reg(dev, REG_MODEM_CONFIG2, 0x74U);
  write_reg(dev, REG_MODEM_CONFIG3, 0x04U);
  write_reg(dev, REG_PREAMBLE_MSB, 0x00U);
  write_reg(dev, REG_PREAMBLE_LSB, 0x08U);

  uint32_t frf = (uint32_t)((freq_mhz * 1000000.0f) / (FXOSC_HZ / 524288.0f));
  write_reg(dev, REG_FRF_MSB, (uint8_t)(frf >> 16));
  write_reg(dev, REG_FRF_MID, (uint8_t)(frf >> 8));
  write_reg(dev, REG_FRF_LSB, (uint8_t)frf);

  set_tx_power(dev, tx_dbm);
  return HAL_OK;
}

HAL_StatusTypeDef rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len)
{
  uint8_t hdr[RH_HEADER_LEN];

  if (len > RFM95_MAX_PAYLOAD) {
    return HAL_ERROR;
  }
  if (rfm95_poll(dev)) {
    return HAL_BUSY;
  }

  set_mode(dev, MODE_STDBY);
  write_reg(dev, REG_FIFO_ADDR_PTR, 0x00U);

  /* RadioHead header: to, from, id, flags. */
  hdr[0] = RH_BROADCAST;
  hdr[1] = RH_BROADCAST;
  hdr[2] = dev->tx_id++;
  hdr[3] = 0x00U;
  write_burst(dev, REG_FIFO, hdr, RH_HEADER_LEN);
  write_burst(dev, REG_FIFO, data, len);
  write_reg(dev, REG_PAYLOAD_LENGTH, (uint8_t)(len + RH_HEADER_LEN));

  write_reg(dev, REG_DIO_MAPPING1, DIO0_TX_DONE);
  write_reg(dev, REG_IRQ_FLAGS, 0xFFU);
  if (set_mode(dev, MODE_TX) != HAL_OK) {
    return HAL_ERROR;
  }
  dev->tx_busy = true;
  return HAL_OK;
}

bool rfm95_poll(rfm95_t *dev)
{
  if (!dev->tx_busy) {
    return false;
  }
  /* DIO0 goes high on TxDone; the chip then returns to standby by itself. */
  if (HAL_GPIO_ReadPin(dev->dio0_port, dev->dio0_pin) == GPIO_PIN_SET ||
      (read_reg(dev, REG_IRQ_FLAGS) & IRQ_TX_DONE) != 0U) {
    write_reg(dev, REG_IRQ_FLAGS, 0xFFU);
    dev->tx_busy = false;
  }
  return dev->tx_busy;
}

HAL_StatusTypeDef rfm95_start_rx(rfm95_t *dev)
{
  if (rfm95_poll(dev)) {
    return HAL_BUSY;
  }
  write_reg(dev, REG_DIO_MAPPING1, DIO0_RX_DONE);
  write_reg(dev, REG_IRQ_FLAGS, 0xFFU);
  return set_mode(dev, MODE_RX_CONTINUOUS);
}

int rfm95_receive(rfm95_t *dev, uint8_t *buf, uint8_t buf_len, int16_t *rssi_dbm)
{
  uint8_t pkt[256];

  if (HAL_GPIO_ReadPin(dev->dio0_port, dev->dio0_pin) != GPIO_PIN_SET) {
    return 0;
  }
  uint8_t flags = read_reg(dev, REG_IRQ_FLAGS);
  write_reg(dev, REG_IRQ_FLAGS, 0xFFU);
  if ((flags & IRQ_RX_DONE) == 0U) {
    return 0;
  }
  if ((flags & IRQ_CRC_ERROR) != 0U) {
    return -1;
  }

  uint8_t n = read_reg(dev, REG_RX_NB_BYTES);
  write_reg(dev, REG_FIFO_ADDR_PTR, read_reg(dev, REG_FIFO_RX_CURRENT));
  read_burst(dev, REG_FIFO, pkt, n);

  if (rssi_dbm != NULL) {
    /* HF port (RFM95 at 868/915 MHz); use -164 for 433 MHz modules. */
    *rssi_dbm = (int16_t)(-157 + (int16_t)read_reg(dev, REG_PKT_RSSI));
  }
  if (n < RH_HEADER_LEN) {
    return 0;
  }
  n -= RH_HEADER_LEN;
  if (n > buf_len) {
    n = buf_len;
  }
  for (uint8_t i = 0; i < n; i++) {
    buf[i] = pkt[RH_HEADER_LEN + i];
  }
  return n;
}
