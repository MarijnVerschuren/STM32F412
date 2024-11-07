//
// Created by marijn on 11/4/24.
//

#include "SPI.h"



/*!<
 * static
 * */
static inline void enable_SPI(SPI_t* spi) {
	if (spi == SPI1) { RCC->APB2ENR |= 0x00000001UL; }
	if (spi == SPI2) { RCC->APB2ENR |= 0x00001000UL; }
	if (spi == SPI3) { RCC->APB2ENR |= 0x00004000UL; }
}
static inline void disable_SPI(SPI_t* spi) {
	if (spi == SPI1) { RCC->APB2RSTR |= 0x00000001UL; }
	if (spi == SPI2) { RCC->APB2RSTR |= 0x00001000UL; }
	if (spi == SPI3) { RCC->APB2RSTR |= 0x00004000UL; }
}

static SPI_t* int_to_SPI(uint8_t pnum) {  // TODO: unify with other periph converters
	return (SPI_t*)(APB2PERIPH_BASE + (pnum << 12U));
}


/*!<
 * functions
 * */
void fconfig_SPI_master(SPI_GPIO_t _sck, SPI_GPIO_t _mosi, SPI_GPIO_t _miso, uint32_t flags, uint16_t crc_poly) {
	dev_pin_t sck, mosi, miso;
	*((uint32_t*)&sck) = _sck ;
	*((uint32_t*)&mosi) = _mosi;
	*((uint32_t*)&miso) = _miso;
	SPI_t* spi = int_to_SPI(sck.periph);
	enable_SPI(spi); fconfig_GPIO(int_to_GPIO(sck.port), sck.pin, GPIO_alt_func | GPIO_high_speed | GPIO_push_pull, sck.alt);
	if (_mosi) { fconfig_GPIO(int_to_GPIO(mosi.port), mosi.pin, GPIO_alt_func | GPIO_high_speed | GPIO_push_pull, mosi.alt); }
	if (_miso) { fconfig_GPIO(int_to_GPIO(miso.port), miso.pin, GPIO_alt_func | GPIO_high_speed | GPIO_push_pull, miso.alt); }
	spi->CR1 = (flags & 0xFFFFUL) | 0x00000004UL;
	spi->CR2 = ((flags >> 16U) & 0xFFFFUL);
	spi->I2SCFGR = 0x00000000UL;
	spi->CRCPR = crc_poly;
}

void config_SPI_master(SPI_GPIO_t sck, SPI_GPIO_t mosi, SPI_GPIO_t miso, uint32_t flags) {
	fconfig_SPI_master(sck, mosi, miso, flags, 0);
}

uint32_t SPI_master_write8(SPI_t* spi, const uint8_t* buffer, uint32_t size, uint32_t timeout) {
	spi->CR1 |= 0x00000044UL;
	uint64_t start = tick;
	for (uint32_t i = 0; i < size; i++) {
		while (!(spi->SR & 0x00000002UL)) { if ( tick - start > timeout) { return i; } }
		spi->DR = buffer[i];
	} return size;
}

// TODO: 22.5.8