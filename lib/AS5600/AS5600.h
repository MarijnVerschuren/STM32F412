//
// Created by marijn on 9/24/24.
//

#ifndef STM32F412_AS5600_H
#define STM32F412_AS5600_H
#include "I2C.h"


#define AS5600_ADDRESS					0x36

// AS5600 status registers
#define AS5600_REGISTER_STATUS			0x0B
#define AS5600_REGISTER_AGC				0x1A
#define AS5600_REGISTER_MAGNITUDE_HIGH	0x1B
#define AS5600_REGISTER_MAGNITUDE_LOW	0x1C
#define AS5600_REGISTER_BURN			0xFF



uint8_t AS5600_get_status(I2C_t* i2c);


#endif //STM32F412_AS5600_H
