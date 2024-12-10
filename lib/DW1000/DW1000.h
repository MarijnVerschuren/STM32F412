//
// Created by marijn on 12/5/24.
//

#ifndef STM32F412_DW1000_H
#define STM32F412_DW1000_H
#include "SPI.h"


/*!<
 * defines
 * */
#define DW1000_TIMEOUT 10
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define PRE_TIMEOUT 8
/* tx only */
#define POLL_TX_TO_RESP_RX_DLY_UUS 300
#define RESP_RX_TIMEOUT_UUS 2700


/*!<
 * messages
 * */
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t frame_seq_nb = 0;

/*!<
 * types
 * */
typedef struct {
	SPI_t* spi;
	GPIO_t* NSS_port;
	GPIO_t* NRST_port;
	uint8_t NSS_pin:	4;
	uint8_t NRST_pin:	4;
	uint8_t tx:			1;
} DW1000_t;

typedef struct
{
	uint8_t chan;           //!< channel number {1, 2, 3, 4, 5, 7 }
	uint8_t prf;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
	uint8_t txPreambLength; //!< DWT_PLEN_64..DWT_PLEN_4096
	uint8_t rxPAC;          //!< Acquisition Chunk Size (Relates to RX preamble length)
	uint8_t txCode;         //!< TX preamble code
	uint8_t rxCode;         //!< RX preamble code
	uint8_t nsSFD;          //!< Boolean should we use non-standard SFD for better performance
	uint8_t dataRate;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
	uint8_t phrMode;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
	uint16_t sfdTO;         //!< SFD timeout value (in symbols)
} DW1000_config_t;

extern DW1000_config_t dw1000_cfg;

/*!<
 * functions
 * */
void DW1000_init(DW1000_t* dw1000);
void DW1000_config(DW1000_t* dw1000, DW1000_config_t* cfg);

void DW1000_initiator(DW1000_t* dw1000);
void DW1000_responder(DW1000_t* dw1000);


#endif //STM32F412_DW1000_H
