
// Initializes sensor and communicates over I2C
// Reads acceleration and computes tilt angle

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "imu.h"
#include "nrf_delay.h"

// ---Configuration values---

// Accelerometer register settings
#define ACC_REBOOT_VAL          0x80  // PWR_MGMT_1: device reset
#define ACC_BDU_VAL             0x00  // PWR_MGMT_1
#define ACC_CONFIG_VAL          0x00  // ACCEL_CONFIG

// Delays (ms)
#define REBOOT_DELAY_MS         100

// Accelerometer conversion
#define ACC_SENSITIVITY_MG      (1000.0 / 16384.0) 
#define MG_PER_G                1000.0

// Gyroscope values
#define GYRO_CONFIG_VAL         0x00
#define GYRO_SENSITIVITY_MDPS   (1000.0 / 131.0)


// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;

// Helper function to perform a 1-byte I2C read of a given register
//
// i2c_addr - address of the device to read from
// reg_addr - address of the register within the device to read
//
// returns 8-bit read value
static uint8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr) {

  uint8_t rx_buf = 0;
  nrf_twi_mngr_transfer_t const read_transfer[] = {
    //TODO: implement me
    NRF_TWI_MNGR_WRITE(i2c_addr, &reg_addr, 1, NRF_TWI_MNGR_NO_STOP ),
    NRF_TWI_MNGR_READ(i2c_addr, &rx_buf, 1, 0),
  };

  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, read_transfer, 2, NULL); //2 because we read and write

  if (result != NRF_SUCCESS) {
    // Likely error codes:
    //  NRF_ERROR_INTERNAL            (0x0003) - something is wrong with the driver itself
    //  NRF_ERROR_INVALID_ADDR        (0x0010) - buffer passed was in Flash instead of RAM
    //  NRF_ERROR_BUSY                (0x0011) - driver was busy with another transfer still
    //  NRF_ERROR_DRV_TWI_ERR_OVERRUN (0x8200) - data was overwritten during the transaction
    //  NRF_ERROR_DRV_TWI_ERR_ANACK   (0x8201) - i2c device did not acknowledge its address
    //  NRF_ERROR_DRV_TWI_ERR_DNACK   (0x8202) - i2c device did not acknowledge a data byte
    printf("I2C transaction failed! Error: %lX\n", result);
  }

  return rx_buf;
}

// Helper function to perform a 1-byte I2C write of a given register
//
// i2c_addr - address of the device to write to
// reg_addr - address of the register within the device to write
static void i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
  //Note: there should only be a single two-byte transfer to be performed
  uint8_t tx_buf[2] = {reg_addr, data};
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    NRF_TWI_MNGR_WRITE(i2c_addr, tx_buf, 2, 0),
  };
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);

  if (result != NRF_SUCCESS) {
    // Likely error codes:
    //  NRF_ERROR_INTERNAL            (0x0003) - something is wrong with the driver itself
    //  NRF_ERROR_INVALID_ADDR        (0x0010) - buffer passed was in Flash instead of RAM
    //  NRF_ERROR_BUSY                (0x0011) - driver was busy with another transfer still
    //  NRF_ERROR_DRV_TWI_ERR_OVERRUN (0x8200) - data was overwritten during the transaction
    //  NRF_ERROR_DRV_TWI_ERR_ANACK   (0x8201) - i2c device did not acknowledge its address
    //  NRF_ERROR_DRV_TWI_ERR_DNACK   (0x8202) - i2c device did not acknowledge a data byte
    printf("I2C transaction failed! Error: %lX\n", result);
  }
}

// Initialize and configure the LSM303AGR accelerometer
//
// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;

  // ---Initialize Accelerometer---

  // Reboot acclerometer
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG5_A, ACC_REBOOT_VAL);
  nrf_delay_ms(REBOOT_DELAY_MS); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG4_A, ACC_BDU_VAL);

  // Configure accelerometer at 100Hz, normal mode (10-bit)
  // Enable x, y and z axes
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, CTRL_REG1_A, ACC_CONFIG_VAL);

  // ---Initialize Gyroscope---
  // Configure Gyroscope
  i2c_reg_write(LSM303AGR_GYRO_ADDRESS, CTRL_REG1_G, GYRO_CONFIG_VAL);

  //the MPU's loww-pass filter
  //filters out the vibration before it affects the pitch and gyro 
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, 0x1A, 0x03);

  // Read WHO AM I register
  // Always returns the same value if working
  //TODO: read the Accelerometer WHO AM I register and check the result
  // i2c_addr - address of the device to read from
  // reg_addr - address of the register within the device to read
  uint8_t read_buf = i2c_reg_read(LSM303AGR_ACC_ADDRESS, WHO_AM_I_A);
  printf("acc: %x\n", read_buf); // expect 68 (MPU-6050)

}

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  uint8_t x_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_L_A);
  uint8_t x_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_H_A);

  int16_t c_x = (int16_t)((x_b << 8) | x_a);

  uint8_t y_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_L_A);
  uint8_t y_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_H_A);

  int16_t c_y = (int16_t)((y_b << 8) | y_a);
  
  uint8_t z_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_L_A);
  uint8_t z_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_H_A);

  int16_t c_z = (int16_t)((z_b << 8) | z_a);

  float x_mg = (c_x * ACC_SENSITIVITY_MG);
  float y_mg = (c_y * ACC_SENSITIVITY_MG);
  float z_mg = (c_z * ACC_SENSITIVITY_MG);

  lsm303agr_measurement_t measurement = {0};

  measurement.x_axis = x_mg / MG_PER_G; //divided by 1000 because we want g
  measurement.y_axis = y_mg / MG_PER_G;
  measurement.z_axis = z_mg / MG_PER_G;

  return measurement;
}

lsm303agr_measurement_t lsm303agr_read_gyroscope(void) {
  //TODO: implement me
  uint8_t x_a = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_X_L_G);
  uint8_t x_b = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_X_H_G);

  int16_t c_x = (int16_t)((x_b << 8) | x_a);

  uint8_t y_a = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_Y_L_G);
  uint8_t y_b = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_Y_H_G);

  int16_t c_y = (int16_t)((y_b << 8) | y_a);
  
  uint8_t z_a = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_Z_L_G);
  uint8_t z_b = i2c_reg_read(LSM303AGR_GYRO_ADDRESS, OUT_Z_H_G);

  int16_t c_z = (int16_t)((z_b << 8) | z_a);

  float x_mg = (c_x * GYRO_SENSITIVITY_MDPS);
  float y_mg = (c_y * GYRO_SENSITIVITY_MDPS);
  float z_mg = (c_z * GYRO_SENSITIVITY_MDPS);

  lsm303agr_measurement_t measurement = {0};

  measurement.x_axis = x_mg / MG_PER_G; //divided by 1000 because we want g
  measurement.y_axis = y_mg / MG_PER_G;
  measurement.z_axis = z_mg / MG_PER_G;

  return measurement;
}

// Compute tilt angle (phi) from the Z axis using accelerometer data
// Return angle in degrees
float lsm303agr_tilt_angle(lsm303agr_measurement_t acc) {
  float phi_rad = atan2f(sqrtf(acc.x_axis*acc.x_axis + acc.y_axis*acc.y_axis), acc.z_axis);
  return phi_rad * 180.0 / M_PI;
}

// Pitch: rotation about X (follows x axis)
float lsm303agr_pitch_angle(lsm303agr_measurement_t acc) {
  return atan2f(acc.x_axis, sqrtf(acc.y_axis*acc.y_axis + acc.z_axis*acc.z_axis)) * 180.0f / M_PI;
}

// Roll: rotation about X (follows y axis)
float lsm303agr_roll_angle(lsm303agr_measurement_t acc) {
  return atan2f(acc.y_axis, sqrtf(acc.x_axis*acc.x_axis + acc.z_axis*acc.z_axis)) * 180.0f / M_PI;
}

