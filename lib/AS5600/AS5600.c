//
// Created by marijn on 9/24/24.
//

#include "AS5600.h"


uint8_t AS5600_get_status(I2C_t* i2c) {
	uint8_t status = 0;
	I2C_master_read_reg(i2c, AS5600_ADDRESS, AS5600_REGISTER_STATUS, &status, 1, 10);
	return status;
}