/**
 * @file    bmp388.c
 * @brief   Bosch BMP388 SPI driver. Register map and compensation formulas
 *          follow the Bosch BMP388 datasheet (BST-BMP388-DS001).
 */
#include "bmp388.h"
#include <math.h>
#include <string.h>

#define BMP388_CHIP_ID_VAL   0x50U

#define REG_CHIP_ID          0x00U
#define REG_ERR              0x02U
#define REG_STATUS           0x03U
#define REG_DATA             0x04U /* 6 bytes: press xlsb/lsb/msb, temp xlsb/lsb/msb */
#define REG_PWR_CTRL         0x1BU
#define REG_OSR              0x1CU
#define REG_ODR              0x1DU
#define REG_CONFIG           0x1FU
#define REG_CALIB            0x31U /* 21 bytes of trimming coefficients */
#define REG_CMD              0x7EU

#define CMD_SOFT_RESET       0xB6U

#define STATUS_DRDY_PRESS    0x20U
#define STATUS_DRDY_TEMP     0x40U

#define PWRCTRL_PRESS_EN         0x01U
#define PWRCTRL_TEMP_EN          0x02U
#define PWRCTRL_MODE_NORMAL      0x30U

#define OSR_P_X8             0x03U
#define OSR_T_X1             (0x00U << 3)
#define ODR_50_HZ            0x02U
#define IIR_COEF_3           (0x02U << 1)

#define ERR_CONF             0x04U

#define SPI_TIMEOUT_MS       10U

static void cs_low(bmp388_t *dev)  { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET); }
static void cs_high(bmp388_t *dev) { HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET); }

static HAL_StatusTypeDef read_regs(bmp388_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
  /* SPI read: address with bit 7 set, then one dummy byte, then data. */
  uint8_t hdr[2] = { (uint8_t)(reg | 0x80U), 0x00U };
  HAL_StatusTypeDef st;

  cs_low(dev);
  st = HAL_SPI_Transmit(dev->hspi, hdr, sizeof(hdr), SPI_TIMEOUT_MS);
  if (st == HAL_OK) {
    st = HAL_SPI_Receive(dev->hspi, buf, len, SPI_TIMEOUT_MS);
  }
  cs_high(dev);
  return st;
}

static HAL_StatusTypeDef write_reg(bmp388_t *dev, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { (uint8_t)(reg & 0x7FU), val };
  HAL_StatusTypeDef st;

  cs_low(dev);
  st = HAL_SPI_Transmit(dev->hspi, buf, sizeof(buf), SPI_TIMEOUT_MS);
  cs_high(dev);
  return st;
}

static void parse_calib(bmp388_calib_t *c, const uint8_t *b)
{
  uint16_t t1  = (uint16_t)(b[1] << 8 | b[0]);
  uint16_t t2  = (uint16_t)(b[3] << 8 | b[2]);
  int8_t   t3  = (int8_t)b[4];
  int16_t  p1  = (int16_t)(b[6] << 8 | b[5]);
  int16_t  p2  = (int16_t)(b[8] << 8 | b[7]);
  int8_t   p3  = (int8_t)b[9];
  int8_t   p4  = (int8_t)b[10];
  uint16_t p5  = (uint16_t)(b[12] << 8 | b[11]);
  uint16_t p6  = (uint16_t)(b[14] << 8 | b[13]);
  int8_t   p7  = (int8_t)b[15];
  int8_t   p8  = (int8_t)b[16];
  int16_t  p9  = (int16_t)(b[18] << 8 | b[17]);
  int8_t   p10 = (int8_t)b[19];
  int8_t   p11 = (int8_t)b[20];

  c->t1  = (float)t1 * 256.0f;                         /* / 2^-8  */
  c->t2  = (float)t2 / 1073741824.0f;                  /* / 2^30  */
  c->t3  = (float)t3 / 281474976710656.0f;             /* / 2^48  */
  c->p1  = ((float)p1 - 16384.0f) / 1048576.0f;        /* (x - 2^14) / 2^20 */
  c->p2  = ((float)p2 - 16384.0f) / 536870912.0f;      /* (x - 2^14) / 2^29 */
  c->p3  = (float)p3 / 4294967296.0f;                  /* / 2^32  */
  c->p4  = (float)p4 / 137438953472.0f;                /* / 2^37  */
  c->p5  = (float)p5 * 8.0f;                           /* / 2^-3  */
  c->p6  = (float)p6 / 64.0f;                          /* / 2^6   */
  c->p7  = (float)p7 / 256.0f;                         /* / 2^8   */
  c->p8  = (float)p8 / 32768.0f;                       /* / 2^15  */
  c->p9  = (float)p9 / 281474976710656.0f;             /* / 2^48  */
  c->p10 = (float)p10 / 281474976710656.0f;            /* / 2^48  */
  c->p11 = (float)p11 / 36893488147419103232.0f;       /* / 2^65  */
}

HAL_StatusTypeDef bmp388_init(bmp388_t *dev, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  uint8_t id = 0;
  uint8_t calib[21];
  uint8_t err;

  memset(dev, 0, sizeof(*dev));
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  cs_high(dev);

  /* The BMP388 starts in I2C mode; the first CS falling edge switches it to
   * SPI, so the first read is a throwaway. */
  (void)read_regs(dev, REG_CHIP_ID, &id, 1);

  if (write_reg(dev, REG_CMD, CMD_SOFT_RESET) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(10);
  (void)read_regs(dev, REG_CHIP_ID, &id, 1); /* re-enter SPI mode after reset */

  if (read_regs(dev, REG_CHIP_ID, &id, 1) != HAL_OK || id != BMP388_CHIP_ID_VAL) {
    return HAL_ERROR;
  }

  if (read_regs(dev, REG_CALIB, calib, sizeof(calib)) != HAL_OK) {
    return HAL_ERROR;
  }
  parse_calib(&dev->calib, calib);

  /* Bosch "drone" profile: x8 pressure, x1 temperature, IIR 3, 50 Hz. */
  if (write_reg(dev, REG_OSR, OSR_P_X8 | OSR_T_X1) != HAL_OK ||
      write_reg(dev, REG_ODR, ODR_50_HZ) != HAL_OK ||
      write_reg(dev, REG_CONFIG, IIR_COEF_3) != HAL_OK ||
      write_reg(dev, REG_PWR_CTRL, PWRCTRL_PRESS_EN | PWRCTRL_TEMP_EN | PWRCTRL_MODE_NORMAL) != HAL_OK) {
    return HAL_ERROR;
  }

  /* conf_err is set if the ODR is too fast for the chosen oversampling. */
  if (read_regs(dev, REG_ERR, &err, 1) != HAL_OK || (err & ERR_CONF) != 0U) {
    return HAL_ERROR;
  }
  return HAL_OK;
}

bool bmp388_data_ready(bmp388_t *dev)
{
  uint8_t status = 0;

  if (read_regs(dev, REG_STATUS, &status, 1) != HAL_OK) {
    return false;
  }
  return (status & (STATUS_DRDY_PRESS | STATUS_DRDY_TEMP)) == (STATUS_DRDY_PRESS | STATUS_DRDY_TEMP);
}

HAL_StatusTypeDef bmp388_read(bmp388_t *dev, bmp388_data_t *out)
{
  uint8_t b[6];
  const bmp388_calib_t *c = &dev->calib;

  if (read_regs(dev, REG_DATA, b, sizeof(b)) != HAL_OK) {
    return HAL_ERROR;
  }

  float up = (float)((uint32_t)b[2] << 16 | (uint32_t)b[1] << 8 | b[0]);
  float ut = (float)((uint32_t)b[5] << 16 | (uint32_t)b[4] << 8 | b[3]);

  /* Temperature compensation (datasheet 9.2). */
  float pd1 = ut - c->t1;
  float pd2 = pd1 * c->t2;
  float t = pd2 + (pd1 * pd1) * c->t3;

  /* Pressure compensation (datasheet 9.3). */
  float t2 = t * t;
  float t3 = t2 * t;
  float out1 = c->p5 + c->p6 * t + c->p7 * t2 + c->p8 * t3;
  float out2 = up * (c->p1 + c->p2 * t + c->p3 * t2 + c->p4 * t3);
  float up2 = up * up;
  float out3 = up2 * (c->p9 + c->p10 * t) + up2 * up * c->p11;

  out->temperature_c = t;
  out->pressure_pa = out1 + out2 + out3;
  return HAL_OK;
}

float bmp388_altitude_m(float pressure_pa, float reference_pa)
{
  return 44330.0f * (1.0f - powf(pressure_pa / reference_pa, 0.190295f));
}
