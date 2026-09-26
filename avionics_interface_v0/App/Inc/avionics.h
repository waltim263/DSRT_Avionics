/**
 * @file    avionics.h
 * @brief   Top-level flight software: brings up the sensors and radio, then
 *          samples them and downlinks telemetry from the main loop.
 */
#ifndef AVIONICS_H
#define AVIONICS_H

#include <stdint.h>

/* Must match the radio module (red dot = 433 MHz, green/blue = 868/915 MHz)
 * and the ground station. */
#ifndef AVIONICS_RADIO_FREQ_MHZ
#define AVIONICS_RADIO_FREQ_MHZ   915.0f
#endif
#define AVIONICS_RADIO_TX_DBM     17
#define AVIONICS_TELEMETRY_PERIOD_MS 250U /* SF7/125 kHz: ~80 ms on air per packet */
#define AVIONICS_IMU_INTERVAL_US  10000U  /* 100 Hz IMU reports */

/* Bits in avionics_telemetry_t.status: set when the part initialised. */
#define AVIONICS_STATUS_BMP388    0x01U
#define AVIONICS_STATUS_BNO085    0x02U
#define AVIONICS_STATUS_RFM95     0x04U

/**
 * Downlink packet, little-endian, 35 bytes. Sent after the 4 byte RadioHead
 * header. Decode on the ground with Python:
 *   struct.unpack('<Iff11hB', payload)
 */
typedef struct __attribute__((packed)) {
  uint32_t time_ms;          /* ms since boot */
  float pressure_pa;
  float altitude_agl_m;      /* relative to the pressure measured at boot */
  int16_t temperature_cc;    /* 0.01 degC */
  int16_t quat[4];           /* i, j, k, real; Q14 (divide by 16384) */
  int16_t accel[3];          /* m/s^2, Q8 (divide by 256), includes gravity */
  int16_t gyro[3];           /* rad/s, Q9 (divide by 512) */
  uint8_t status;            /* AVIONICS_STATUS_* bits */
} avionics_telemetry_t;

_Static_assert(sizeof(avionics_telemetry_t) == 35, "telemetry layout changed: update the ground decoder");

/** Initialise every sensor; parts that fail are flagged in the status byte. */
void avionics_init(void);

/** One pass of the main loop. Call continuously from while (1). */
void avionics_run(void);

/** Latest telemetry frame, handy to inspect in the debugger. */
extern avionics_telemetry_t g_telemetry;

#endif /* AVIONICS_H */
