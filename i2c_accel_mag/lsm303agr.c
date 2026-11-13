// LSM303AGR driver for Microbit_v2
//
// Initializes sensor and communicates over I2C
// Capable of reading temperature, acceleration, and magnetic field strength

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "lsm303agr.h"
#include "nrf_delay.h"

// ---Configuration values---

// Accelerometer register settings
#define ACC_REBOOT_VAL          0x80  // CTRL_REG5_A: reboot memory content
#define ACC_BDU_VAL             0x80  // CTRL_REG4_A: block data update
#define ACC_CONFIG_VAL          0x57  // CTRL_REG1_A: 100Hz normal mode x/y/z enabled

// Magnetometer register settings
#define MAG_REBOOT_VAL          0x40  // CFG_REG_A_M: reboot memory content
#define MAG_BDU_VAL             0x10  // CFG_REG_C_M: block data update
#define MAG_CONFIG_VAL          0x0C  // CFG_REG_A_M: 100Hz continuous mode

// Temperature sensor settings
#define TEMP_ENABLE_VAL         0xC0  // TEMP_CFG_REG_A: enable temperature sensor

// Delays (ms)
#define REBOOT_DELAY_MS         100
#define TEMP_READY_DELAY_MS     100

// Temperature conversion
#define TEMP_SCALE              (1.0 / 256.0)  // degrees C per LSB
#define TEMP_OFFSET_C           25.0           // degrees C at raw value 0

// Accelerometer conversion
#define ACC_DATA_SHIFT          6      // 10-bit  data
#define ACC_SENSITIVITY_MG      3.9    // mg per LSB
#define MG_PER_G                1000.0

// Magnetometer conversion
#define MAG_SENSITIVITY_MGAUSS  1.5    // mgauss per LSB
#define MGAUSS_PER_UT           10.0

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

// Initialize and configure the LSM303AGR accelerometer/magnetometer
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

  // Read WHO AM I register
  // Always returns the same value if working
  //TODO: read the Accelerometer WHO AM I register and check the result
  // i2c_addr - address of the device to read from
  // reg_addr - address of the register within the device to read
  uint8_t read_buf = i2c_reg_read(LSM303AGR_ACC_ADDRESS, WHO_AM_I_A);
  printf("acc: %x\n", read_buf); //51 in decimal


  // ---Initialize Magnetometer---

  // Reboot magnetometer
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, MAG_REBOOT_VAL);
  nrf_delay_ms(REBOOT_DELAY_MS); // needs delay to wait for reboot

  // Enable Block Data Update
  // Only updates sensor data when both halves of the data has been read
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_C_M, MAG_BDU_VAL);

  // Configure magnetometer at 100Hz, continuous mode
  i2c_reg_write(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, MAG_CONFIG_VAL);

  // Read WHO AM I register
  //TODO: read the Magnetometer WHO AM I register and check the result
  uint8_t read_mag = i2c_reg_read(LSM303AGR_MAG_ADDRESS, WHO_AM_I_M);
  printf("mag: %x\n", read_mag); //64 in decimal 

  // ---Initialize Temperature---

  // Enable temperature sensor
  i2c_reg_write(LSM303AGR_ACC_ADDRESS, TEMP_CFG_REG_A, TEMP_ENABLE_VAL);
  nrf_delay_ms(TEMP_READY_DELAY_MS); // needs delay to be ready
}

// Read the internal temperature sensor
//
// Return measurement as floating point value in degrees C
float lsm303agr_read_temperature(void) {
  //TODO: implement me
  uint8_t read_temp_la = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_L_A);
  uint8_t read_temp_ha = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_TEMP_H_A);

  int16_t combined_read = ((int16_t)read_temp_ha << 8) | read_temp_la;

  float temp = ((float)combined_read * TEMP_SCALE) + TEMP_OFFSET_C; //celsius

  return temp;
}

lsm303agr_measurement_t lsm303agr_read_accelerometer(void) {
  //TODO: implement me
  uint8_t x_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_L_A);
  uint8_t x_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_X_H_A);

  int16_t c_x = (int16_t)((x_b << 8) | x_a);
  int16_t combined_x = c_x >> ACC_DATA_SHIFT;

  uint8_t y_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_L_A);
  uint8_t y_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Y_H_A);

  int16_t c_y = (int16_t)((y_b << 8) | y_a);
  int16_t combined_y = c_y >> ACC_DATA_SHIFT;
  
  uint8_t z_a = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_L_A);
  uint8_t z_b = i2c_reg_read(LSM303AGR_ACC_ADDRESS, OUT_Z_H_A);

  int16_t c_z = (int16_t)((z_b << 8) | z_a);
  int16_t combined_z = c_z >> ACC_DATA_SHIFT;

  float x_mg = (combined_x * ACC_SENSITIVITY_MG);
  float y_mg = (combined_y * ACC_SENSITIVITY_MG);
  float z_mg = (combined_z * ACC_SENSITIVITY_MG);

  lsm303agr_measurement_t measurement = {0};

  measurement.x_axis = x_mg / MG_PER_G; //divided by 1000 because we want g
  measurement.y_axis = y_mg / MG_PER_G;
  measurement.z_axis = z_mg / MG_PER_G;

  return measurement;
}


lsm303agr_measurement_t lsm303agr_read_magnetometer(void) {
  //TODO: implement me

  uint8_t x_a = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_L_REG_M);
  uint8_t x_b = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTX_H_REG_M);

  int16_t combined_x = (((int16_t)x_b << 8) | x_a);

  uint8_t y_a = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_L_REG_M);
  uint8_t y_b = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTY_H_REG_M);

  int16_t combined_y = (((int16_t)y_b << 8) | y_a);

  uint8_t z_a = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_L_REG_M);
  uint8_t z_b = i2c_reg_read(LSM303AGR_MAG_ADDRESS, OUTZ_H_REG_M);

  int16_t combined_z = (((int16_t)z_b << 8) | z_a);


  lsm303agr_measurement_t measurement = {0};

  float x_mgauss = combined_x * MAG_SENSITIVITY_MGAUSS;
  float y_mgauss = combined_y * MAG_SENSITIVITY_MGAUSS;
  float z_mgauss = combined_z * MAG_SENSITIVITY_MGAUSS;

  measurement.x_axis = x_mgauss / MGAUSS_PER_UT; //divided by 10 because we want uT
  measurement.y_axis = y_mgauss / MGAUSS_PER_UT;
  measurement.z_axis = z_mgauss / MGAUSS_PER_UT;

  return measurement;
}


// Compute tilt angle (phi) from the Z axis using accelerometer data
// AN-1057 equation 13: phi = atan( sqrt(Ax^2 + Ay^2) / Az )
//
// Return angle in degrees
float lsm303agr_tilt_angle(lsm303agr_measurement_t acc) {
  float phi_rad = atan(sqrt(acc.x_axis * acc.x_axis + acc.y_axis * acc.y_axis) / acc.z_axis);
  return phi_rad * 180.0 / M_PI;
}
