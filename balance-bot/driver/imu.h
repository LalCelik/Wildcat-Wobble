#pragma once

#include "nrf_twi_mngr.h"

// Chip address for accelerometer
static const uint8_t LSM303AGR_ACC_ADDRESS = 0x68;  // MPU-6050 (AD0 low)
static const uint8_t LSM303AGR_GYRO_ADDRESS = 0x68;

// Measurement data type
typedef struct {
  float x_axis;
  float y_axis;
  float z_axis;
} lsm303agr_measurement_t;

// Register definitions for accelerometer
typedef enum {
  INT_COUNTER_REG_A = 0x0E,
  WHO_AM_I_A = 0x75,       // MPU-6050 WHO_AM_I
  CTRL_REG1_A = 0x1C,      // MPU-6050 ACCEL_CONFIG
  CTRL_REG2_A = 0x21,
  CTRL_REG3_A = 0x22,
  CTRL_REG4_A = 0x6B,      // MPU-6050 PWR_MGMT_1 (wake)
  CTRL_REG5_A = 0x6B,      // MPU-6050 PWR_MGMT_1 (reset)
  CTRL_REG6_A = 0x25,
  REFERENCE_DATACAPTURE_A = 0x26,
  STATUS_REG_A = 0x27,
  OUT_X_L_A = 0x3C,        // MPU-6050 ACCEL_XOUT_L
  OUT_X_H_A = 0x3B,        // MPU-6050 ACCEL_XOUT_H
  OUT_Y_L_A = 0x3E,        // MPU-6050 ACCEL_YOUT_L
  OUT_Y_H_A = 0x3D,        // MPU-6050 ACCEL_YOUT_H
  OUT_Z_L_A = 0x40,        // MPU-6050 ACCEL_ZOUT_L
  OUT_Z_H_A = 0x3F,        // MPU-6050 ACCEL_ZOUT_H
  FIFO_CTRL_REG_A = 0x2E,
  FIFO_SRC_REG_A = 0x2F,
  INT1_CFG_A = 0x30,
  INT1_SRC_A = 0x31,
  INT1_THS_A = 0x32,
  INT1_DURATION_A = 0x33,
  INT2_CFG_A = 0x34,
  INT2_SRC_A = 0x35,
  INT2_THS_A = 0x36,
  INT2_DURATION_A = 0x37,
  CLICK_CFG_A = 0x38,
  CLICK_SRC_A = 0x39,
  CLICK_THS_A = 0x3A,
  TIME_LIMIT_A = 0x3B,
  TIME_LATENCY_A = 0x3C,
  TIME_WINDOW_A = 0x3D,
  ACT_THS_A = 0x3E,
  ACT_DUR_A = 0x3F,
} lsm303agr_acc_reg_t;

typedef enum {
  WHO_AM_I_G = 0x75,
  CTRL_REG1_G = 0x1B,
  OUT_X_H_G = 0x43,
  OUT_X_L_G = 0x44,
  OUT_Y_H_G = 0x45,
  OUT_Y_L_G = 0x46,
  OUT_Z_H_G = 0x47,
  OUT_Z_L_G = 0x48,
} lsm303agr_gyro_reg_t;

// Function prototypes

// Initialize and configure the LSM303AGR accelerometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c);

// Read all three axes on the accelerometer
//
// Return measurements as floating point values in g's
lsm303agr_measurement_t lsm303agr_read_accelerometer(void);

// Compute tilt angle from the Z axis (phi) from accelerometer data
//
// Return angle in degrees
float lsm303agr_tilt_angle(lsm303agr_measurement_t acc);

// Signed tilt angles in degrees (range -90 to +90)
float lsm303agr_pitch_angle(lsm303agr_measurement_t acc);
float lsm303agr_roll_angle(lsm303agr_measurement_t acc);

lsm303agr_measurement_t lsm303agr_read_gyroscope(void);
