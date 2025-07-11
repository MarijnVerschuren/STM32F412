//
// Created by marijn on 9/24/24.
//

#ifndef STM32F412_I2C_H
#define STM32F412_I2C_H
#include "GPIO.h"
#include "sys.h"


typedef enum {
	I2C_PIN_DISABLE =	0x00000000UL,
	// I2C1
	I2C1_SCL_B6 =		0x46100154UL,
	I2C1_SDA_B7 =		0x47100154UL,
	I2C1_SCL_B8 =		0x48100154UL,
	I2C1_SDA_B9 = 		0x49100154UL,
	// I2C2
	I2C2_SDA_B9 =		0x99100164UL,
	I2C2_SCL_B10 =		0x4A100164UL
} I2C_GPIO_t;

typedef enum {
	I2C_ADDR_7BIT =		0b0u,
	I2C_ADDR_10BIT =	0b1u,
} I2C_address_t;


/*!< init / enable / disable */
void disable_I2C_clock(I2C_t* i2c);
void fconfig_I2C(I2C_GPIO_t scl, I2C_GPIO_t sda, uint16_t own_address, I2C_address_t address_type, uint8_t dual_address);
void config_I2C(I2C_GPIO_t scl, I2C_GPIO_t sda, uint8_t own_address);  // default to 7bit addressing
void reset_I2C(I2C_t* i2c);
void enable_I2C(I2C_t* i2c);
void disable_I2C(I2C_t* i2c);
/*!< master input / output */
uint8_t I2C_master_address(I2C_t* i2c, uint8_t i2c_address, uint32_t timeout);
uint32_t I2C_master_write(I2C_t* i2c, uint8_t i2c_address, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t I2C_master_read(I2C_t* i2c, uint8_t i2c_address, uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t I2C_master_write_reg(I2C_t* i2c, uint8_t i2c_address, uint8_t reg_address, const uint8_t* buffer, uint32_t size, uint32_t timeout);
uint32_t I2C_master_read_reg(I2C_t* i2c, uint8_t i2c_address, uint8_t reg_address, uint8_t* buffer, uint32_t size, uint32_t timeout);
// TODO: slave!!!


#endif //STM32F412_I2C_H
