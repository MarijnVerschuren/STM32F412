//
// Created by marijn on 12/5/24.
//

#include "DW1000.h"
#include "DW1000_regs.h"

// ! TODO
uint32_t txFCTRL;


/*!<
 * constants
 * */
DW1000_config_t dw1000_cfg = {
	.chan =				2,
	.prf =				DWT_PRF_64M,
	.txPreambLength =	DWT_PLEN_1024,
	.rxPAC =			DWT_PAC32,
	.txCode =			9,
	.rxCode =			9,
	.nsSFD =			1,
	.dataRate =			DWT_BR_110K,
	.phrMode =			DWT_PHRMODE_STD,
	.sfdTO =			(1025 + 64 - 32) /* TODO: SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


/*!<
 * LL functions
 * */
static inline void DW1000_read_reg(DW1000_t* dw1000, uint32_t reg, uint16_t offset, uint8_t* buffer, uint32_t size) {
	reg &= 0x3FU;
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset & 0xFF00) {
		reg |= 0x8040U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 3, DW1000_TIMEOUT);
	} else if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*)&reg, 1, DW1000_TIMEOUT);
	}
	SPI_read8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 1);
}

static inline void DW1000_write_reg(DW1000_t* dw1000, uint32_t reg, uint16_t offset, uint8_t* buffer, uint32_t size) {
	reg = (reg & 0x3FU) | 0x80;
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 0);
	if (offset & 0xFF00) {
		reg |= 0x8040U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 3, DW1000_TIMEOUT);
	} else if (offset) {
		reg |= 0x40U | (offset << 8);
		SPI_write8(dw1000->spi, (void*)&reg, 2, DW1000_TIMEOUT);
	} else {
		SPI_write8(dw1000->spi, (void*)&reg, 1, DW1000_TIMEOUT);
	}
	SPI_write8(dw1000->spi, buffer, size, DW1000_TIMEOUT);
	GPIO_write(dw1000->NSS_port, dw1000->NSS_pin, 1);
}


/*!<
 * functions
 * */
void DW1000_init(DW1000_t* dw1000) {
	uint32_t tmp;
	// reset
	GPIO_write(dw1000->NRST_port, dw1000->NRST_pin, 0);
	delay_ms(1);
	GPIO_write(dw1000->NRST_port, dw1000->NRST_pin, 1);

	// check devid
	DW1000_read_reg(dw1000, DEV_ID_ID, 0, (void*)&tmp, 4);
	if (tmp != 0xDECA0130UL) { for(;;); }

	// enable clock
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
	delay_ms(1);
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
	delay_ms(1);
	DW1000_read_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);
	tmp &= 0xFEU;
	DW1000_write_reg(dw1000, PMSC_ID, PMSC_CTRL0_OFFSET, (void*)&tmp, 2);

	// AON
	tmp = 0;
	DW1000_write_reg(dw1000, AON_ID, AON_CFG1_OFFSET, (void*)&tmp, 1);
}


void DW1000_config(DW1000_t* dw1000, DW1000_config_t* cfg) {
	uint32_t tmp = 0;
	uint32_t sysCFGreg;
	uint16_t reg16 = 0x28F4; // rx code 9 (lde_replicaCoeff)

	DW1000_read_reg(dw1000, SYS_CFG_ID, 0x00, (void*)&sysCFGreg, 4);
	if(DWT_BR_110K == cfg->dataRate) {
		sysCFGreg |= SYS_CFG_RXM110K;
		reg16 >>= 3; // lde_replicaCoeff must be divided by 8
	} else {
		sysCFGreg &= (~SYS_CFG_RXM110K) ;
	}

	sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
	sysCFGreg |= (SYS_CFG_PHR_MODE_11 & ((uint32_t)cfg->phrMode << SYS_CFG_PHR_MODE_SHFT));

	DW1000_write_reg(dw1000, SYS_CFG_ID, 0x00, (void*)&sysCFGreg, 4);

	// Set the lde_replicaCoeff
	DW1000_write_reg(dw1000, LDE_IF_ID, LDE_REPC_OFFSET, (void*)&reg16, 2);

	tmp = LDE_PARAM1;
	DW1000_write_reg(dw1000,LDE_IF_ID, LDE_CFG1_OFFSET, (void*)&tmp, 1); // 8-bit configuration register

	if(dw1000_cfg.prf - DWT_PRF_16M){
		tmp = LDE_PARAM3_64;
		DW1000_write_reg(dw1000, LDE_IF_ID, LDE_CFG2_OFFSET, (void*)&tmp, 2); // 16-bit LDE configuration tuning register
	} else {
		tmp = LDE_PARAM3_16;
		DW1000_write_reg(dw1000, LDE_IF_ID, LDE_CFG2_OFFSET, (void*)&tmp, 2);
	}

	// Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
	tmp = FS_PLLCFG_CH2;
	DW1000_write_reg(dw1000, FS_CTRL_ID, FS_PLLCFG_OFFSET, (void*)&tmp, 4);
	tmp = FS_PLLTUNE_CH2;
	DW1000_write_reg(dw1000, FS_CTRL_ID, FS_PLLTUNE_OFFSET, (void*)&tmp, 1);

	// Configure RF RX blocks (for specified channel/bandwidth)
	tmp = RF_RXCTRLH_NBW;
	DW1000_write_reg(dw1000, RF_CONF_ID, RF_RXCTRLH_OFFSET, (void*)&tmp, 1);

	// Configure RF TX blocks (for specified channel and PRF)
	// Configure RF TX control
	tmp = RF_TXCTRL_CH2;
	DW1000_write_reg(dw1000, RF_CONF_ID, RF_TXCTRL_OFFSET, (void*)&tmp, 4);

	// Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
	// DTUNE0
	tmp = DRX_TUNE0b_110K_NSTD;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE0b_OFFSET, (void*)&tmp, 2);

	// DTUNE1
	tmp = DRX_TUNE1a_PRF64;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE1a_OFFSET, (void*)&tmp, 2);
	tmp = DRX_TUNE1b_110K;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE1b_OFFSET, (void*)&tmp, 2);

	// DTUNE2
	tmp = DRX_TUNE2_PRF64_PAC32;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_TUNE2_OFFSET, (void*)&tmp, 4);

	// DTUNE3 (SFD timeout)
	// Don't allow 0 - SFD timeout will always be enabled
	if(cfg->sfdTO == 0)
	{
		cfg->sfdTO = DWT_SFDTOC_DEF;
	}
	tmp = cfg->sfdTO;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_SFDTOC_OFFSET, (void*)&tmp, 2);

	// Configure AGC parameters
	tmp = AGC_TUNE2_VAL;
	DW1000_write_reg(dw1000, AGC_CFG_STS_ID, 0xC, (void*)&tmp, 4);
	tmp = AGC_TUNE1_64M;
	DW1000_write_reg(dw1000, AGC_CFG_STS_ID, 0x4, (void*)&tmp, 2);

	// Set (non-standard) user SFD for improved performance,
	uint8_t nsSfd_result = 0;
	uint8_t useDWnsSFD = 0;
	if(cfg->nsSFD)
	{
		// Write non standard (DW) SFD length
		tmp = DW_NS_SFD_LEN_110K;
		DW1000_write_reg(dw1000, USR_SFD_ID, 0x00, (void*)&tmp, 1);
		nsSfd_result = 3;
		useDWnsSFD = 1;
	}

	uint8_t chan = cfg->chan;
	uint32_t regval =  (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) | // Transmit Channel
			  (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) | // Receive Channel
			  (CHAN_CTRL_RXFPRF_MASK & ((uint32_t)cfg->prf << CHAN_CTRL_RXFPRF_SHIFT)) | // RX PRF
			  ((CHAN_CTRL_TNSSFD|CHAN_CTRL_RNSSFD) & ((uint32_t)nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
			  (CHAN_CTRL_DWSFD & ((uint32_t)useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) | // Use DW nsSFD
			  (CHAN_CTRL_TX_PCOD_MASK & ((uint32_t)cfg->txCode << CHAN_CTRL_TX_PCOD_SHIFT)) | // TX Preamble Code
			  (CHAN_CTRL_RX_PCOD_MASK & ((uint32_t)cfg->rxCode << CHAN_CTRL_RX_PCOD_SHIFT)) ; // RX Preamble Code

	DW1000_write_reg(dw1000, CHAN_CTRL_ID, 0x00, (void*)&regval, 4);

	// Set up TX Preamble Size, PRF and Data Rate
	txFCTRL = ((uint32_t)(cfg->txPreambLength | cfg->prf) << TX_FCTRL_TXPRF_SHFT) | ((uint32_t)cfg->dataRate << TX_FCTRL_TXBR_SHFT);

	DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*)&txFCTRL, 4);

	// Request TX start and TRX off at the same time
	tmp = SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF;
	DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET, (void*)&tmp, 1);

	// set rx antenna delay
	tmp = RX_ANT_DLY;
	DW1000_write_reg(dw1000, LDE_IF_ID, LDE_RXANTD_OFFSET, (void*)&tmp, 2);
	// set tx antenna delay
	tmp = TX_ANT_DLY;
	DW1000_write_reg(dw1000, LDE_IF_ID, TX_ANTD_OFFSET, (void*)&tmp, 2);

	/* tx specific functions */
	if (dw1000->tx) {
		// set rx after tx delay
		uint32_t val;
		DW1000_read_reg(dw1000, ACK_RESP_T_ID, 0x00, (void*)&val, 4) ; // Read ACK_RESP_T_ID register
		val &= ~(ACK_RESP_T_W4R_TIM_MASK) ; // Clear the timer (19:0)
		val |= (POLL_TX_TO_RESP_RX_DLY_UUS & ACK_RESP_T_W4R_TIM_MASK) ; // In UWB microseconds (e.g. turn the receiver on 20uus after TX)
		DW1000_write_reg(dw1000, ACK_RESP_T_ID, 0x00,(void*)&val, 4);

		// set tx timeout
		DW1000_read_reg(dw1000, SYS_CFG_ID, 3, (void*)&tmp, 1); // Read at offset 3 to get the upper byte only

		if(RESP_RX_TIMEOUT_UUS > 0) {
			uint32_t time = RESP_RX_TIMEOUT_UUS;
			DW1000_write_reg(dw1000, RX_FWTO_ID, RX_FWTO_OFFSET, (void*)&time, 2);
			tmp |= (uint8_t)(SYS_CFG_RXWTOE>>24); // Shift RXWTOE mask as we read the upper byte only
			// OR in 32bit value (1 bit set), I know this is in high byte.
			sysCFGreg |= SYS_CFG_RXWTOE;

			DW1000_write_reg(dw1000, SYS_CFG_ID, 3, (void*)&tmp, 1); // Write at offset 3 to write the upper byte only
		} else {
			tmp &= ~((uint8_t)(SYS_CFG_RXWTOE>>24)); // Shift RXWTOE mask as we read the upper byte only
			// AND in inverted 32bit value (1 bit clear), I know this is in high byte.
			sysCFGreg &= ~(SYS_CFG_RXWTOE);

			DW1000_write_reg(dw1000, SYS_CFG_ID, 3, (void*)&tmp, 1); // Write at offset 3 to write the upper byte only
		}
	}
	/* end of tx specific functions */

	// set preamble timeout
	tmp = PRE_TIMEOUT;
	DW1000_write_reg(dw1000, DRX_CONF_ID, DRX_PRETOC_OFFSET, (void*)&tmp, 2);

}


void DW1000_initiator(DW1000_t* dw1000) {
	/* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
	tx_poll_msg[2] = frame_seq_nb;
	DW1000_write_reg(dw1000, TX_BUFFER_ID, 0, tx_poll_msg, sizeof(tx_poll_msg) - 2);
	uint32_t tmp = txFCTRL | sizeof(tx_poll_msg) | ((uint32_t)0x0U << TX_FCTRL_TXBOFFS_SHFT) | ((uint32_t)0x1U << TX_FCTRL_TR_SHFT);
	DW1000_write_reg(dw1000, TX_FCTRL_ID, 0x00, (void*)&tmp, 4);

	// start TX
	tmp = SYS_CTRL_WAIT4RESP | SYS_CTRL_TXSTRT;
	DW1000_write_reg(dw1000, SYS_CTRL_ID, SYS_CTRL_OFFSET,(void*)&tmp, 1);
	// TODO
}

void DW1000_responder(DW1000_t* dw1000) {

}
