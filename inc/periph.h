//
// Created by marijn on 9/7/24.
//

#ifndef STM32F412_PERIPH_H
#define STM32F412_PERIPH_H
#include "memory_map.h"
#include "base.h"


/*!< core peripherals */
#define SYS_TICK				((SYS_TICK_t*)SYS_TICK_BASE)
#define NVIC					((NVIC_t*)NVIC_BASE)
#define SCB						((SCB_t*)SCB_BASE)


/*!< APB1 peripherals */
#define TIM2					((TIM_t*)TIM2_BASE)
#define TIM3					((TIM_t*)TIM3_BASE)
#define TIM4					((TIM_t*)TIM4_BASE)
#define TIM5					((TIM_t*)TIM5_BASE)
#define TIM6					((TIM_t*)TIM6_BASE)
#define TIM7					((TIM_t*)TIM7_BASE)
#define TIM12					((TIM_t*)TIM12_BASE)
#define TIM13					((TIM_t*)TIM13_BASE)
#define TIM14					((TIM_t*)TIM14_BASE)
#define RTC						((RTC_t*)RTC_BASE)
#define SPI2					((SPI_t*)SPI2_BASE)
#define SPI3					((SPI_t*)SPI3_BASE)
#define USART2					((USART_t*)USART2_BASE)
#define USART3					((USART_t*)USART3_BASE)
#define I2C1					((I2C_t*)I2C1_BASE)
#define I2C2					((I2C_t*)I2C2_BASE)
#define I2C3					((I2C_t*)I2C3_BASE)
#define PWR						((PWR_t*)PWR_BASE)


/*!< APB2 peripherals */
#define TIM1					((TIM_t*)TIM1_BASE)
#define TIM8					((TIM_t*)TIM8_BASE)
#define USART1					((USART_t*)USART1_BASE)
#define USART6					((USART_t*)USART6_BASE)
#define ADC1					((ADC_t*)ADC1_BASE)
#define ADC_COMMON				((ADC_COMMON_t*)ADC_COMMON_BASE)
#define SPI1					((SPI_t*)SPI1_BASE)
#define SPI4					((SPI_t*)SPI4_BASE)
#define SYSCFG					((SYSCFG_t*)SYSCFG_BASE)
#define EXTI					((EXTI_t *)EXTI_BASE)
#define TIM9					((TIM_t*)TIM9_BASE)
#define TIM10					((TIM_t*)TIM10_BASE)
#define TIM11					((TIM_t*)TIM11_BASE)


/*!< AHB1 peripherals */
#define GPIOA					((GPIO_t*)GPIOA_BASE)
#define GPIOB					((GPIO_t*)GPIOB_BASE)
#define GPIOC					((GPIO_t*)GPIOC_BASE)
#define GPIOH					((GPIO_t*)GPIOH_BASE)
#define RCC						((RCC_t*)RCC_BASE)
#define FLASH					((FLASH_t*)FLASH_BASE)


/*!< AHB2 peripherals */


/*!< USB peripherals */
#define USB_OTG_FS						((USB_OTG_GlobalTypeDef*)USB_OTG_DEVICE_BASE)


/*!<
 * core peripheral types
 * */
typedef struct {
	_IO uint32_t	CTRL;			/* control and status                0x00 */
	_IO uint32_t	LOAD;			/* reload value                      0x04 */
	_IO uint32_t	VAL;			/* current value                     0x08 */
	_I  uint32_t	CALIB;			/* calibration                       0x0C */
} SYS_TICK_t;

typedef struct {
	_IO uint32_t ISER[8U];			/* interrupt enable                 0x000 */
	uint32_t _0[24U];
	_IO uint32_t ICER[8U];			/* interrupt disable                0x080 */
	uint32_t _1[24U];
	_IO uint32_t ISPR[8U];			/* interrupt set pending            0x100 */
	uint32_t _2[24U];
	_IO uint32_t ICPR[8U];			/* interrupt clear pending			0x180 */
	uint32_t _3[24U];
	_IO uint32_t IABR[8U];			/* interrupt active bit             0x200 */
	uint32_t _4[56U];
	_IO uint8_t  IP[240U];			/* interrupt priority               0x300 */
	uint32_t _5[644U];
	_O	uint32_t STIR;				/* software trigger interrupt       0xE00 */
} NVIC_t;

typedef struct {
	_I  uint32_t	CPUID;			/* CPUID base                        0x00 */
	_IO uint32_t	ICSR;			/* interrupt control and state       0x04 */
	_IO uint32_t	VTOR;			/* vector table offset               0x08 */
	_IO uint32_t	AIRCR;			/* software interrupt, reset control 0x0C */
	_IO uint32_t	SCR;			/* system control                    0x10 */
	_IO uint32_t	CCR;			/* configuration control             0x14 */
	_IO uint8_t 	SHP[12U];		/* system handler priority      0x18-0x23 */
	_IO uint32_t	SHCSR;			/* system handler control and state  0x24 */
	_IO uint32_t	CFSR;			/* configurable fault status         0x28 */
	_IO uint32_t	HFSR;			/* hard fault status                 0x2C */
	_IO uint32_t	DFSR;			/* debug fault status                0x30 */
	_IO uint32_t	MMFAR;			/* memory manage fault address       0x34 */
	_IO uint32_t	BFAR;			/* bus fault address                 0x38 */
	_IO uint32_t	AFSR;			/* auxiliary fault status            0x3C */
	_I  uint32_t	PFR[2U];		/* processor feature            0x40-0x44 */
	_I  uint32_t	DFR;			/* debug feature                     0x48 */
	_I  uint32_t	ADR;			/* auxiliary feature                 0x4C */
	_I  uint32_t	MMFR[4U];		/* memory model feature         0x50-0x5C */
	_I  uint32_t	ISAR[5U];		/* instruction set attributes   0x60-0x70 */
	uint32_t		_0[5U];			/*                              0x74-0x84 */
	_IO uint32_t	CPACR;			/* coprocessor access control        0x88 */
} SCB_t;


/*!<
 * peripheral types
 * */
/*!< PWR */
typedef struct {
	_IO uint32_t	CR;				/* power control                     0x00 */
	_IO uint32_t	CSR;			/* power control and status          0x04 */
} PWR_t;

/*!< FLASH */
typedef struct {
	_IO uint32_t	ACR;			/* access control                    0x00 */
	_IO uint32_t	KEYR;			/* key                               0x04 */
	_IO uint32_t	OPTKEYR;		/* option key                        0x08 */
	_IO uint32_t	SR;				/* status                            0x0C */
	_IO uint32_t	CR;				/* control                           0x10 */
	_IO uint32_t	OPTCR;			/* option control                    0x14 */
	_IO uint32_t	OPTCR1;			/* option control 1                  0x18 */
} FLASH_t;

/*!< RCC */
typedef struct {
	_IO uint32_t	CR;				/* clock control                     0x00 */
	_IO uint32_t	PLLCFGR;		/* PLL configuration                 0x04 */
	_IO uint32_t	CFGR;			/* clock configuration               0x08 */
	_IO uint32_t	CIR;			/* clock interrupt                   0x0C */
	_IO uint32_t	AHB1RSTR;		/* AHB1 peripheral disable           0x10 */
	_IO uint32_t	AHB2RSTR;		/* AHB2 peripheral disable           0x14 */
		uint32_t	_0[2];			/*                              0x18-0x1C */
	_IO uint32_t	APB1RSTR;		/* APB1 peripheral disable           0x20 */
	_IO uint32_t	APB2RSTR;		/* APB2 peripheral disable           0x24 */
		uint32_t	_1[2];			/*                              0x28-0x2C */
	_IO uint32_t	AHB1ENR;		/* AHB1 peripheral enable            0x30 */
	_IO uint32_t	AHB2ENR;		/* AHB2 peripheral enable            0x34 */
		uint32_t	_2[2];			/*                              0x38-0x3C */
	_IO uint32_t	APB1ENR;		/* APB1 peripheral enable            0x40 */
	_IO uint32_t	APB2ENR;		/* APB2 peripheral enable            0x44 */
		uint32_t	_3[2];			/*                              0x48-0x4C */
	_IO uint32_t	AHB1LPENR;		/* AHB1 peripheral low power enable  0x50 */
	_IO uint32_t	AHB2LPENR;		/* AHB2 peripheral low power enable  0x54 */
		uint32_t	_4[2];			/*                              0x58-0x5C */
	_IO uint32_t	APB1LPENR;		/* APB1 peripheral low power enable  0x60 */
	_IO uint32_t	APB2LPENR;		/* APB2 peripheral low power enable  0x64 */
		uint32_t	_5[2];			/*                              0x68-0x6C */
	_IO uint32_t	BDCR;			/* backup domain control             0x70 */
	_IO uint32_t	CSR;			/* clock control & status            0x74 */
		uint32_t	_6[2];			/*                              0x78-0x7C */
	_IO uint32_t	SSCGR;			/* spread spectrum clock generation  0x80 */
	_IO uint32_t	PLLI2SCFGR;		/* PLLI2S configuration              0x84 */
		uint32_t	_7;				/*                                   0x88 */
	_IO uint32_t	DCKCFGR;		/* dedicated clocks configuration    0x8C */
	_IO uint32_t	CKGATENR;		/* clocks gated enable               0x90 */
	_IO uint32_t	DCKCFGR2;		/* dedicated clocks configuration 1  0x94 */
} RCC_t;

/*!< SYSCFG */
typedef struct {
	_IO uint32_t MEMRMP;			/* memory remap                      0x00 */
	_IO uint32_t PMC;				/* peripheral mode configuration     0x04 */
	_IO uint32_t EXTICR[4];			/* external interrupt configuration  0x08-0x14 */
		uint32_t _;
	_IO uint32_t CFGR2;				/* configuration                     0x1C */
	_IO uint32_t CMPCR;				/* compensation cell control         0x20 */
	_IO uint32_t CFGR;				/* configuration                     0x24 */
} SYSCFG_t;

/*!< GPIO */
typedef struct {
	_IO uint32_t	MODER;			/* port mode                         0x00 */
	_IO uint32_t	OTYPER;			/* port output type                  0x04 */
	_IO uint32_t	OSPEEDR;		/* port output speed                 0x08 */
	_IO uint32_t	PUPDR;			/* port pull-up/pull-down            0x0C */
	_IO uint32_t	IDR;			/* port input data                   0x10 */
	_IO uint32_t	ODR;			/* port output data                  0x14 */
	_IO uint32_t	BSRR;			/* port bit set/reset                0x18 */
	_IO uint32_t	LCKR;			/* port configuration lock           0x1C */
	_IO uint32_t	AFR[2];			/* alternate function           0x20-0x24 */
} GPIO_t;

/*!< EXTI */
typedef struct {
	_IO uint32_t IMR;				/* interrupt mask                    0x00 */
	_IO uint32_t EMR;				/* event mask                        0x04 */
	_IO uint32_t RTSR;				/* rising trigger selection          0x08 */
	_IO uint32_t FTSR;				/* falling trigger selection         0x0C */
	_IO uint32_t SWIER;				/* software interrupt event          0x10 */
	_IO uint32_t PR;				/* pending                           0x14 */
} EXTI_t;

/*!< TIM */
typedef struct {
	_IO uint32_t CR1;				/* control 1                         0x00 */
	_IO uint32_t CR2;				/* control 2                         0x04 */
	_IO uint32_t SMCR;				/* slave mode control                0x08 */
	_IO uint32_t DIER;				/* DMA/interrupt enable              0x0C */
	_IO uint32_t SR;				/* status                            0x10 */
	_IO uint32_t EGR;				/* event generation                  0x14 */
	_IO uint32_t CCMR1;				/* capture/compare mode 1            0x18 */
	_IO uint32_t CCMR2;				/* capture/compare mode 2            0x1C */
	_IO uint32_t CCER;				/* capture/compare enable            0x20 */
	_IO uint32_t CNT;				/* counter                           0x24 */
	_IO uint32_t PSC;				/* prescaler,                        0x28 */
	_IO uint32_t ARR;				/* auto-reload                       0x2C */
	_IO uint32_t RCR;				/* repetition counter                0x30 */
	_IO uint32_t CCR1;				/* capture/compare 1                 0x34 */
	_IO uint32_t CCR2;				/* capture/compare 2                 0x38 */
	_IO uint32_t CCR3;				/* capture/compare 3                 0x3C */
	_IO uint32_t CCR4;				/* capture/compare 4                 0x40 */
	_IO uint32_t BDTR;				/* break and dead-time               0x44 */
	_IO uint32_t DCR;				/* DMA control                       0x48 */
	_IO uint32_t DMAR;				/* DMA address for full transfer     0x4C */
	_IO uint32_t OR;				/* option                            0x50 */
} TIM_t;

/*!< RTC */
typedef struct {
	_IO uint32_t	TR;				/* time                              0x00 */
	_IO uint32_t	DR;				/* date                              0x04 */
	_IO uint32_t	CR;				/* control                           0x08 */
	_IO uint32_t	ISR;			/* initialization and status         0x0C */
	_IO uint32_t	PRER;			/* prescaler                         0x10 */
	_IO uint32_t	WUTR;			/* wakeup timer                      0x14 */
	_IO uint32_t	CALIBR;			/* calibration                       0x18 */
	_IO uint32_t	ALRMAR;			/* alarm A                           0x1C */
	_IO uint32_t	ALRMBR;			/* alarm B                           0x20 */
	_IO uint32_t	WPR;			/* write protection                  0x24 */
	_IO uint32_t	SSR;			/* sub second                        0x28 */
	_IO uint32_t	SHIFTR;			/* shift control                     0x2C */
	_IO uint32_t	TSTR;			/* time stamp time                   0x30 */
	_IO uint32_t	TSDR;			/* time stamp date                   0x34 */
	_IO uint32_t	TSSSR;			/* time-stamp sub second             0x38 */
	_IO uint32_t	CALR;			/* calibration                       0x3C */
	_IO uint32_t	TAFCR;			/* tamper and af configuration       0x40 */
	_IO uint32_t	ALRMASSR;		/* alarm A sub second                0x44 */
	_IO uint32_t	ALRMBSSR;		/* alarm B sub second                0x48 */
		uint32_t	_0;				/*                                   0x4C */
	_IO uint32_t	BKPR[20];		/* backup registers             0x50-0x9C */
} RTC_t;

/*!< USART */
typedef struct {
	_IO uint32_t	SR;				/* status                            0x00 */
	_IO uint32_t	DR;				/* data                              0x04 */
	_IO uint32_t	BRR;			/* baud rate                         0x08 */
	_IO uint32_t	CR1;			/* Control 1                         0x0C */
	_IO uint32_t	CR2;			/* Control 2                         0x10 */
	_IO uint32_t	CR3;			/* Control 3                         0x14 */
	_IO uint32_t	GTPR;			/* Guard time and prescaler          0x18 */
} USART_t;

/*!< SPI */
typedef struct {
	_IO	uint32_t	CR1;			/* control 1                         0x00 */
	_IO	uint32_t	CR2;			/* control 2                         0x04 */
	_IO	uint32_t	SR;				/* status                            0x08 */
	_IO	uint32_t	DR;				/* data                              0x0C */
	_IO	uint32_t	CRCPR;			/* CRC polynomial                    0x10 */
	_IO	uint32_t	RXCRCR;			/* RX CRC                            0x14 */
	_IO	uint32_t	TXCRCR;			/* TX CRC                            0x18 */
	_IO	uint32_t	I2SCFGR;		/* I2S configuration                 0x1C */
	_IO	uint32_t	I2SPR;			/* I2S prescaler                     0x20 */
} SPI_t;

/*!< IWDG TODO */

/*!< CRC TODO */

/*!< RNG TODO */

/*!< ADC */
typedef struct {
	_IO uint32_t SR;				/* status                            0x00 */
	_IO uint32_t CR1;				/* control 1                         0x04 */
	_IO uint32_t CR2;				/* control 2                         0x08 */
	_IO uint32_t SMPR1;				/* sample time 1                     0x0C */
	_IO uint32_t SMPR2;				/* sample time 2                     0x10 */
	_IO uint32_t JOFR1;				/* injected channel data offset 1    0x14 */
	_IO uint32_t JOFR2;				/* injected channel data offset 2    0x18 */
	_IO uint32_t JOFR3;				/* injected channel data offset 3    0x1C */
	_IO uint32_t JOFR4;				/* injected channel data offset 4    0x20 */
	_IO uint32_t HTR;				/* watchdog higher threshold         0x24 */
	_IO uint32_t LTR;				/* watchdog lower threshold          0x28 */
	_IO uint32_t SQR1;				/* regular sequence 1                0x2C */
	_IO uint32_t SQR2;				/* regular sequence 2                0x30 */
	_IO uint32_t SQR3;				/* regular sequence 3                0x34 */
	_IO uint32_t JSQR;				/* injected sequence                 0x38 */
	_IO uint32_t JDR1;				/* injected data 1                   0x3C */
	_IO uint32_t JDR2;				/* injected data 2                   0x40 */
	_IO uint32_t JDR3;				/* injected data 3                   0x44 */
	_IO uint32_t JDR4;				/* injected data 4                   0x48 */
	_IO uint32_t DR;				/* regular data                      0x4C */
} ADC_t;

typedef struct {
	_IO uint32_t CSR;				/* status                    ADC1 + 0x300 */
	_IO uint32_t CCR;				/* control                   ADC1 + 0x304 */
	_IO uint32_t CDR;				/* dual / triple mode data   ADC1 + 0x308 */
} ADC_COMMON_t;

/*!< I2C */
typedef struct {
	_IO uint32_t CR1;				/* control 1                         0x00 */
	_IO uint32_t CR2;				/* control 2                         0x04 */
	_IO uint32_t OAR1;				/* own address 1                     0x08 */
	_IO uint32_t OAR2;				/* own address 2                     0x0C */
	_IO uint32_t DR;				/* data                              0x10 */
	_IO uint32_t SR1;				/* status 1                          0x14 */
	_IO uint32_t SR2;				/* status 2                          0x18 */
	_IO uint32_t CCR;				/* clock control                     0x1C */
	_IO uint32_t TRISE;				/* TRISE                             0x20 */
	_IO uint32_t FLTR;				/* FLTR                              0x24 */
} I2C_t;


/*!<
 * USB systems
 * */
/*!< USB global */
typedef struct {
	_IO uint32_t GOTGCTL;			/* OTG control and status            0x00 */
	_IO uint32_t GOTGINT;			/* OTG interrupt                     0x04 */
	_IO uint32_t GAHBCFG;			/* AHB configuration                 0x08 */
	_IO uint32_t GUSBCFG;			/* USB configuration                 0x0C */
	_IO uint32_t GRSTCTL;			/* reset control                     0x10 */
	_IO uint32_t GINTSTS;			/* interrupt                         0x14 */
	_IO uint32_t GINTMSK;			/* interrupt mask                    0x18 */
	_IO uint32_t GRXSTSR;			/* receive status queue read         0x1C */
	_IO uint32_t GRXSTSP;			/* receive status queue read & pop   0x20 */
	_IO uint32_t GRXFSIZ;			/* receive FIFO size                 0x24 */
	_IO uint32_t DIEPTXF0_HNPTXFSIZ;/* EP0/non periodic tx-FIFO size     0x28 */
	_IO uint32_t HNPTXSTS;			/* non periodic tx-FIFO/queue status 0x2C */
		uint32_t _0[2];				/*                              0x30-0x34 */
	_IO uint32_t GCCFG;				/* GPIO config                       0x38 */
	_IO uint32_t CID;				/* user ID                           0x3C */
		uint32_t _1[3];				/*                              0x40-0x48 */
	_IO uint32_t GHWCFG3;			/* user HW config 3                  0x4C */
		uint32_t _2;				/*                                   0x50 */
	_IO uint32_t GLPMCFG;			/* LPM register                      0x54 */
		uint32_t _3;				/*                                   0x58 */
	_IO uint32_t GDFIFOCFG;			/* DFIFO software config register    0x5C */
		uint32_t _4[40];			/*                              0x60-0xFC */
	_IO uint32_t HPTXFSIZ;			/* host periodic tx-FIFO size       0x100 */
	_IO uint32_t DIEPTXF[0x0F];		/* dev periodic tx-FIFO             0x104 */
} USB_OTG_GlobalTypeDef;

/*!< USB device */
typedef struct {
	_IO uint32_t DCFG;				/* configuration                     0x00 */
	_IO uint32_t DCTL;				/* control                           0x04 */
	_I	uint32_t DSTS;				/* status                            0x08 */
		uint32_t _0;				/*                                   0x0C */
	_IO uint32_t DOEPMSK;			/* IEP mask                          0x10 */
	_IO uint32_t DIEPMSK;			/* OEP mask                          0x14 */
	_IO uint32_t DAINT;				/* endpoint interrupt                0x18 */
	_IO uint32_t DAINTMSK;			/* endpoint interrupt mask           0x1C */
		uint32_t _1[2];				/*                              0x20-0x24 */
	_IO uint32_t DVBUSDIS;			/* VBUS discharge                    0x28 */
	_IO uint32_t DVBUSPULSE;		/* VBUS pulse                        0x2C */
	_IO uint32_t DTHRCTL;			/* threshold                         0x30 */
	_IO uint32_t DIEPEMPMSK;		/* IEP empty mask                    0x34 */
	_IO uint32_t DEACHINT;			/* dedicated EP interrupt            0x38 */
	_IO uint32_t DEACHMSK;			/* dedicated EP mask                 0x3C */
		uint32_t _2;				/*                                   0x40 */
	_IO uint32_t DINEP1MSK;			/* dedicated IEP1 mask               0x44 */
		uint32_t _3[15];			/*                              0x44-0x7C */
	_IO uint32_t DOUTEP1MSK;		/* dedicated OEP1 mask               0x84 */
} USB_OTG_DeviceTypeDef;

/*!< USB endpoint */
typedef struct{
  _IO uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  _IO uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  _IO uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  _IO uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  _IO uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

typedef struct{
  _IO uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  _IO uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  _IO uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  _IO uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

//typedef struct {
//	_IO uint32_t CTL;				/* control                           0x00 */
//		uint32_t _0;				/*                                   0x04 */
//	_IO uint32_t INT;				/* interrupt                         0x08 */
//		uint32_t _1;				/*                                   0x0C */
//	_IO uint32_t TSIZ;				/* transfer size                     0x10 */
//	_IO uint32_t DMA;				/* DMA address                       0x14 */
//	_IO uint32_t DTXFSTS;			/* tx-FIFO status                    0x18 */
//		uint32_t _2;				/*                                   0x1C */
//} USB_EP_t;


/*!< USB host */
typedef struct {
	_IO uint32_t HCFG;				/* configuration                     0x00 */
	_IO uint32_t HFIR;				/* frame interval                    0x04 */
	_IO uint32_t HFNUM;				/* frame number/frame                0x08 */
		uint32_t _0;				/*                                   0x0C */
	_IO uint32_t HPTXSTS;			/* periodic tx-FIFO / queue status   0x10 */
	_IO uint32_t HAINT;				/* all channels interrupt            0x14 */
	_IO uint32_t HAINTMSK;			/* all channels interrupt mask       0x18 */
} USB_host_t;

/*!< USB host channel */
typedef struct {
	_IO uint32_t HCCHAR;			/* channel characteristics           0x00 */
	_IO uint32_t HCSPLT;			/* channel split control             0x04 */
	_IO uint32_t HCINT;				/* channel interrupt                 0x08 */
	_IO uint32_t HCINTMSK;			/* channel interrupt mask            0x0C */
	_IO uint32_t HCTSIZ;			/* channel transfer size             0x10 */
	_IO uint32_t HCDMA;				/* channel DMA address               0x14 */
		uint32_t _0[2];				/*                                   0x18 */
} USB_host_channel_t;


#endif // STM32F412_PERIPH_H
