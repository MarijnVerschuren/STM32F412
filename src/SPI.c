//
// Created by marijn on 11/4/24.
//

#include "SPI.h"


/*!<
 * IRQ variables
 * */  // TODO: linker alloc
io_buffer_t* SPI_buffer[5];


/*!<
 * IRQ handler
 * */
static _FINLINE void SPI_IRQ_handler(SPI_t* spi, io_buffer_t* io) {
	if (!io)		{ return; }
	if (!io->ptr)	{ return; }
	while (spi->SR & 0x00000001UL && io->ien) {
		((uint8_t*)io->ptr)[io->i++] = (_IO uint8_t)spi->DR;
		if (io->i == io->size) {					// this is assuming that 'i' is never changed otherwise
			if (io->fifo) { io->i = 0; return; }	// reset offset
			spi->CR2 &= ~0x00000040UL;				// turn off IRQ
		}
	}
	while (spi->SR & 0x00000002UL && io->oen) {
		spi->DR = ((uint8_t*)io->ptr)[io->o++];
		if (io->o == io->size) {					// this is assuming that 'o' is never changed otherwise
			// TODO: FIFO in io mode?
			spi->CR2 &= ~0x00000080UL;				// turn off IRQ
		}
	}
}

void SPI1_handler() { SPI_IRQ_handler(SPI1, SPI_buffer[0]); }
void SPI2_handler() { SPI_IRQ_handler(SPI2, SPI_buffer[1]); }
void SPI3_handler() { SPI_IRQ_handler(SPI3, SPI_buffer[2]); }
void SPI4_handler() { SPI_IRQ_handler(SPI4, SPI_buffer[3]); }
//void SPI5_handler() { SPI_IRQ_handler(SPI5, SPI_buffer[4]); }


/*!<
 * init / enable / disable
 * */
void fconfig_SPI_master(SPI_GPIO_t _sck, SPI_GPIO_t _mosi, SPI_GPIO_t _miso, uint32_t flags, uint16_t crc_poly) {
	dev_pin_t sck, mosi, miso;
	*((uint32_t*)&sck) = _sck ;
	*((uint32_t*)&mosi) = _mosi;
	*((uint32_t*)&miso) = _miso;
	enable_id(_sck); SPI_t* spi = id_to_dev(_sck);
	fconfig_GPIO(int_to_GPIO(sck.port), sck.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, sck.alt);
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

void fconfig_SPI_slave(SPI_GPIO_t _nss, SPI_GPIO_t _sck, SPI_GPIO_t _mosi, SPI_GPIO_t _miso, uint32_t flags, uint16_t crc_poly) {
	dev_pin_t nss, sck, mosi, miso;
	*((uint32_t*)&nss) = _nss;
	*((uint32_t*)&sck) = _sck;
	*((uint32_t*)&mosi) = _mosi;
	*((uint32_t*)&miso) = _miso;
	enable_id(_sck); SPI_t* spi = id_to_dev(_sck);
	fconfig_GPIO(int_to_GPIO(nss.port), nss.pin, GPIO_alt_func | GPIO_push_pull, nss.alt);
	fconfig_GPIO(int_to_GPIO(sck.port), sck.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, sck.alt);
	if (_mosi) { fconfig_GPIO(int_to_GPIO(mosi.port), mosi.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, mosi.alt); }
	if (_miso) { fconfig_GPIO(int_to_GPIO(miso.port), miso.pin, GPIO_alt_func | GPIO_very_high_speed | GPIO_push_pull, miso.alt); }
	spi->CR1 = (flags & 0xFFFBUL);//| 0x00000200UL;  // enable: software NSS
	spi->CR2 = (flags >> 16U);
	spi->I2SCFGR = 0x00000000UL;
	spi->CRCPR = crc_poly;
}

void config_SPI_slave(SPI_GPIO_t nss, SPI_GPIO_t sck, SPI_GPIO_t mosi, SPI_GPIO_t miso, uint32_t flags) {
	fconfig_SPI_slave(nss, sck, mosi, miso, flags, 0);
}


/*!<
 * usage
 * */
uint32_t SPI_write8(SPI_t* spi, const uint8_t* buffer, uint32_t size, uint32_t timeout) {
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

uint32_t SPI_read8(SPI_t* spi, uint8_t* buffer, uint32_t size, uint32_t timeout) {
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


/*!<
 * IRQ
 * */
void start_SPI_read8_IRQ(SPI_t* spi, io_buffer_t* buffer, uint8_t priority) {
	spi->CR2 |= 0x00000040UL;				// enable RXNE interrupt
	register uint32_t irqn = 0;
	switch ((uint32_t)spi) {
	case SPI1_BASE:	irqn = SPI1_IRQn; SPI_buffer[0] = buffer; break;
	case SPI2_BASE:	irqn = SPI2_IRQn; SPI_buffer[1] = buffer; break;
	case SPI3_BASE:	irqn = SPI3_IRQn; SPI_buffer[2] = buffer; break;
	case SPI4_BASE:	irqn = SPI4_IRQn; SPI_buffer[3] = buffer; break;
	case SPI5_BASE:	irqn = SPI5_IRQn; SPI_buffer[4] = buffer; break;
	}
	NVIC_set_IRQ_priority(irqn, priority);
	NVIC_enable_IRQ(irqn);
	SPI1->CR1 |= 0x00000040UL;				// enable SPI
}

void stop_SPI_read8_IRQ(SPI_t* spi) {
	spi->CR2 &= ~0x00000040UL;				// disable RXNE interrupt
	register uint32_t irqn = 0;
	switch ((uint32_t)spi) {
		case SPI1_BASE:	irqn = SPI1_IRQn; SPI_buffer[0] = 0x00000000UL; break;
		case SPI2_BASE:	irqn = SPI2_IRQn; SPI_buffer[1] = 0x00000000UL; break;
		case SPI3_BASE:	irqn = SPI3_IRQn; SPI_buffer[2] = 0x00000000UL; break;
		case SPI4_BASE:	irqn = SPI4_IRQn; SPI_buffer[3] = 0x00000000UL; break;
		case SPI5_BASE:	irqn = SPI5_IRQn; SPI_buffer[4] = 0x00000000UL; break;
	} NVIC_disable_IRQ(irqn);
}

// TODO: 22.5.8