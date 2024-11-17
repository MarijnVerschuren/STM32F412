//
// Created by marijn on 11/4/24.
//

#include "SPI.h"



/*!<
 * functions
 * */
void fconfig_SPI_master(SPI_GPIO_t _sck, SPI_GPIO_t _mosi, SPI_GPIO_t _miso, uint32_t flags, uint16_t crc_poly) {
	dev_pin_t sck, mosi, miso;
	*((uint32_t*)&sck) = _sck ;
	*((uint32_t*)&mosi) = _mosi;
	*((uint32_t*)&miso) = _miso;
	enable_id(_sck); SPI_t* spi = id_to_dev(_sck);
	enable_dev(spi); fconfig_GPIO(int_to_GPIO(sck.port), sck.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, sck.alt);
	if (_mosi) { fconfig_GPIO(int_to_GPIO(mosi.port), mosi.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, mosi.alt); }
	if (_miso) { fconfig_GPIO(int_to_GPIO(miso.port), miso.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, miso.alt); }
	spi->CR1 = (flags & 0xFFFFUL) | 0x00000304UL;  // enable: master mode, software NSS and SSI
	spi->CR2 = (flags >> 16U);
	spi->I2SCFGR = 0x00000000UL;
	spi->CRCPR = crc_poly;
}

void config_SPI_master(SPI_GPIO_t sck, SPI_GPIO_t mosi, SPI_GPIO_t miso, uint32_t flags) {
	fconfig_SPI_master(sck, mosi, miso, flags, 0);
}

void fconfig_SPI_slave(SPI_GPIO_t _sck, SPI_GPIO_t _mosi, SPI_GPIO_t _miso, uint32_t flags, uint16_t crc_poly) {
	dev_pin_t sck, mosi, miso;
	*((uint32_t*)&sck) = _sck ;
	*((uint32_t*)&mosi) = _mosi;
	*((uint32_t*)&miso) = _miso;
	enable_id(_sck); SPI_t* spi = id_to_dev(_sck);
	enable_dev(spi); fconfig_GPIO(int_to_GPIO(sck.port), sck.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, sck.alt);
	if (_mosi) { fconfig_GPIO(int_to_GPIO(mosi.port), mosi.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, mosi.alt); }
	if (_miso) { fconfig_GPIO(int_to_GPIO(miso.port), miso.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, miso.alt); }
	spi->CR1 = (flags & 0xFFFBUL) | 0x00000200UL;  // enable: software NSS
	spi->CR2 = (flags >> 16U);
	spi->I2SCFGR = 0x00000000UL;
	spi->CRCPR = crc_poly;
}

void config_SPI_slave(SPI_GPIO_t sck, SPI_GPIO_t mosi, SPI_GPIO_t miso, uint32_t flags) {
	fconfig_SPI_slave(sck, mosi, miso, flags, 0);
}

uint32_t SPI_master_write8(SPI_t* spi, const uint8_t* buffer, uint32_t size, uint32_t timeout) {
	spi->CR1 |= 0x00000040UL;
	uint64_t start = tick;
	uint32_t i = 0;
	for (; i < size; i++) {
		while (!(spi->SR & 0x00000002UL))	{ if ( tick - start > timeout) { goto SPI_master_write8_end; } }
		spi->DR = buffer[i];
	}
	while (spi->SR & 0x00000080UL)			{ if ( tick - start > timeout) { goto SPI_master_write8_end; } }
	i++; SPI_master_write8_end:
	spi->CR1 &= ~0x00000040UL;
	return i;
}

uint32_t SPI_master_read8(SPI_t* spi, uint8_t* buffer, uint32_t size, uint32_t timeout) {
	spi->CR1 |= 0x00000040UL;
	uint64_t start = tick;
	uint32_t i = 0;
	for (; i < size; i++) {
		while (!(spi->SR & 0x00000001UL))	{ if ( tick - start > timeout) { goto SPI_master_read8_end; } }
		buffer[i] = (_IO uint8_t)spi->DR;
	}
	while (spi->SR & 0x00000080UL)			{ if ( tick - start > timeout) { goto SPI_master_read8_end; } }
	i++; SPI_master_read8_end:
	spi->CR1 &= ~0x00000040UL;
	return i;
}

// TODO: 22.5.8