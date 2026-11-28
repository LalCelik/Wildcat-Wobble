// Light Driver

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "light.h"
#include "nrf_delay.h"

// Pointer to an initialized I2C instance to use for transactions
static const nrf_twi_mngr_t* i2c_manager = NULL;

// Helper function to send a 1-byte command (no register address)
static void i2c_cmd_write(uint8_t i2c_addr, uint8_t cmd) {
  //create array of i2c transfer instructions (nrf_twi_mngr_transfer_t is struct type by nordic)
  nrf_twi_mngr_transfer_t const write_transfer[] = {
    
    //Args 
    //(1)i2c adress of device we want to communicate with 
    //(2) memeory addr. of byte info we send
    //(3) num of bytes
    //(4) Transfer Flags
    NRF_TWI_MNGR_WRITE(i2c_addr, &cmd, 1, 0),
  };
  //Store result of tranfer **THIS IS A BLOCKING FUNCTION!!**
  ret_code_t result = nrf_twi_mngr_perform(i2c_manager, NULL, write_transfer, 1, NULL);
  if (result != NRF_SUCCESS) {
    printf("I2C cmd write failed! Error: %lX\n", result);
  }
}

// i2c - pointer to already initialized and enabled twim instance
void lsm303agr_init(const nrf_twi_mngr_t* i2c) {
  i2c_manager = i2c;

  // Power on Light Sensor
  i2c_cmd_write(BH1750_LIGHT_ADDRESS, 0x01);
  nrf_delay_ms(100);

  // Continuously High Resolution Mode
  i2c_cmd_write(LSM303AGR_LIGHT_ADDRESS, 0x10);