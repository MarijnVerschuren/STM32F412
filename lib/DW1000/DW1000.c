//
// Created by marijn on 12/5/24.
//

#include "DW1000.h"
#include "DW1000_regs.h"


/*!<
 * functions
 * */
static inline void DW1000_read_reg(DW1000_t* dw1000, uint16_t reg, uint8_t offset, uint8_t* buffer, uint32_t size) {
	reg &= 0x3FU;
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*)&reg, 1, DW1000_TIMEOUT);
	}
	SPI_read8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 1);
}
static inline void DW1000_write_reg(DW1000_t* dw1000, uint16_t reg, uint8_t offset, uint8_t* buffer, uint32_t size) {
	reg = (reg & 0x3FU) | 0x80;
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*)&reg, 1, DW1000_TIMEOUT);
	}
	SPI_write8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 1);
}



void DW1000_init(DW1000_t* dw1000) {
	uint32_t tmp;
	// reset
	GPIO_write(dw1000->NRST_port, dw1000->NRST_pin, 0);
	delay_ms(1);
	GPIO_write(dw1000->NRST_port, dw1000->NRST_pin, 1);

	// check devid
	DW1000_read_reg(dw1000, DEV_ID_ID, 0, (void*)&tmp, 4);
	if (tmp != 0xDECA0130UL) { for(;;); }

	// enable clocks
	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);
	tmp &= 0xFFFCU;	// reset sys_clk select
	tmp |= 0x0001U;	// select XTI as sys_clk
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);

	// aon config
	tmp = 0;
	DW1000_write_reg(dw1000, AON_ID, AON_WCFG_OFFSET, (void*)&tmp, 2);
	DW1000_write_reg(dw1000, AON_ID, AON_CFG0_OFFSET, (void*)&tmp, 2);
	DW1000_write_reg(dw1000, AON_ID, AON_CTRL_OFFSET, (void*)&tmp, 1);
	tmp = AON_CTRL_SAVE;
	DW1000_write_reg(dw1000, AON_ID, AON_CTRL_OFFSET, (void*)&tmp, 1);

	// reset HIF, TX, RX and PMSC
	tmp = PMSC_CTRL0_RESET_ALL;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (void*)&tmp, 1);
	//delay_ms(1);
	tmp = PMSC_CTRL0_RESET_CLEAR;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_SOFTRESET_OFFSET, (void*)&tmp, 1);

	// configure the CPLL lock detect
	tmp = EC_CTRL_PLLLCK;
	DW1000_write_reg(dw1000, EXT_SYNC_ID, EC_CTRL_OFFSET, (void*)&tmp, 1);

	// LDO tune config
	// Write the address
	tmp = LDOTUNE_ADDRESS;
	DW1000_write_reg(dw1000,OTP_IF_ID, OTP_ADDR, (void*)&tmp, 2);

	// Perform OTP Read - Manual read mode has to be set
	tmp = OTP_CTRL_OTPREAD | OTP_CTRL_OTPRDEN;
	DW1000_write_reg(dw1000,OTP_IF_ID, OTP_CTRL, (void*)&tmp, 1);
	tmp = 0;
	DW1000_write_reg(dw1000,OTP_IF_ID, OTP_CTRL, (void*)&tmp, 1);
	DW1000_read_reg(dw1000, OTP_IF_ID, OTP_RDAT, (void*)&tmp, 4);
	if((tmp & 0xFF) != 0) {
		tmp = OTP_SF_LDO_KICK;
		DW1000_write_reg(dw1000,OTP_IF_ID, OTP_SF, (void*)&tmp, 1); // Set load LDO kick bit
	}

	//uint16_t otp_xtaltrim_and_rev = _dwt_otpread(XTRIM_ADDRESS) & 0xffff;
	// TODO: OTP?

	// force enable LDE
	tmp = 0x0301U;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);
	tmp = OTP_CTRL_LDELOAD;
	DW1000_write_reg(dw1000,OTP_IF_ID, OTP_CTRL, (void*)&tmp,2); // Set load LDE kick bit

	//delay_ms(1);

	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);
	tmp &= 0xFEU;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);

	// AON
	tmp = 0;
	DW1000_write_reg(dw1000, AON_ID, AON_CFG1_OFFSET, (void*)&tmp, 1);
}