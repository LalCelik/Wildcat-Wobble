#pragma once

#include "stdint.h"
#include "nrf_twi_mngr.h"


// I2C address for light sensor
static const uint8_t BH1750_ADDRESS = 0x23;

// Function that takes in pointer of nordic struct that we call i2c
void light_init(const nrf_twi_mngr_t* i2c);

// Read light function
float light_read_lux(void);