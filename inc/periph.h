//
// Created by marijn on 9/7/24.
//

#ifndef STM32F412_PERIPH_H
#define STM32F412_PERIPH_H
#include "memory_map.h"
#include "types.h"


/*!< core peripherals */
#define SCB						((SCB_t*)SCB_BASE)
#define SYS_TICK				((SYS_TICK_t*)SYS_TICK_BASE)

/*!< APB1 peripherals */
#define PWR						((PWR_t*)PWR_BASE)

/*!< APB2 peripherals */

/*!< AHB1 peripherals */
#define GPIOA					((GPIO_t*)GPIOA_BASE)
#define GPIOB					((GPIO_t*)GPIOB_BASE)
#define GPIOC					((GPIO_t*)GPIOC_BASE)
#define GPIOH					((GPIO_t*)GPIOH_BASE)
#define RCC						((RCC_t*)RCC_BASE)
#define FLASH					((FLASH_t*)FLASH_BASE)

/*!< AHB2 peripherals */


/*!<
 * core peripheral types
 * */
typedef struct {  // TODO:
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
	uint32_t		RESERVED0[5U];	/*                              0x74-0x84 */
	_IO uint32_t	CPACR;			/* coprocessor access control        0x88 */
} SCB_t;

typedef struct {
	_IO uint32_t CTRL;				/* control and status                0x00 */
	_IO uint32_t LOAD;				/* reload value                      0x04 */
	_IO uint32_t VAL;				/* current value                     0x08 */
	_I  uint32_t CALIB;				/* calibration                       0x0C */
} SYS_TICK_t;


/*!<
 * peripheral types
 * */
typedef struct {
	_IO uint32_t CR;   /* power control                                  0x00 */
	_IO uint32_t CSR;  /* power control and status                       0x04 */
} PWR_t;

typedef struct {
	_IO uint32_t	CR;				/* clock control                     0x00 */
	_IO uint32_t	PLLCFGR;		/* PLL configuration                 0x04 */
	_IO uint32_t	CFGR;			/* clock configuration               0x08 */
	_IO uint32_t	CIR;			/* clock interrupt                   0x0C */
	_IO uint32_t	AHB1RSTR;		/* AHB1 peripheral disable           0x10 */
	_IO uint32_t	AHB2RSTR;		/* AHB2 peripheral disable           0x14 */
	uint32_t		RESERVED0[2];	/*                              0x18-0x1C */
	_IO uint32_t	APB1RSTR;		/* APB1 peripheral disable           0x20 */
	_IO uint32_t	APB2RSTR;		/* APB2 peripheral disable           0x24 */
	uint32_t		RESERVED1[2];	/*                              0x28-0x2C */
	_IO uint32_t	AHB1ENR;		/* AHB1 peripheral enable            0x30 */
	_IO uint32_t	AHB2ENR;		/* AHB2 peripheral enable            0x34 */
	uint32_t		RESERVED2[2];	/*                              0x38-0x3C */
	_IO uint32_t	APB1ENR;		/* APB1 peripheral enable            0x40 */
	_IO uint32_t	APB2ENR;		/* APB2 peripheral enable            0x44 */
	uint32_t		RESERVED3[2];	/*                              0x48-0x4C */
	_IO uint32_t	AHB1LPENR;		/* AHB1 peripheral low power enable  0x50 */
	_IO uint32_t	AHB2LPENR;		/* AHB2 peripheral low power enable  0x54 */
	uint32_t		RESERVED4[2];	/*                              0x58-0x5C */
	_IO uint32_t	APB1LPENR;		/* APB1 peripheral low power enable  0x60 */
	_IO uint32_t	APB2LPENR;		/* APB2 peripheral low power enable  0x64 */
	uint32_t		RESERVED5[2];	/*                              0x68-0x6C */
	_IO uint32_t	BDCR;			/* backup domain control             0x70 */
	_IO uint32_t	CSR;			/* clock control & status            0x74 */
	uint32_t		RESERVED6[2];	/*                              0x78-0x7C */
	_IO uint32_t	SSCGR;			/* spread spectrum clock generation  0x80 */
	_IO uint32_t	PLLI2SCFGR;		/* PLLI2S configuration              0x84 */
	uint32_t		RESERVED7;		/*                                   0x88 */
	_IO uint32_t	DCKCFGR;		/* dedicated clocks configuration    0x8C */
	_IO uint32_t	CKGATENR;		/* clocks gated enable               0x90 */
	_IO uint32_t	DCKCFGR1;		/* dedicated clocks configuration 1  0x94 */
} RCC_t;

typedef struct {
	_IO uint32_t	ACR;			/* access control                    0x00 */
	_IO uint32_t	KEYR;			/* key                               0x04 */
	_IO uint32_t	OPTKEYR;		/* option key                        0x08 */
	_IO uint32_t	SR;				/* status                            0x0C */
	_IO uint32_t	CR;				/* control                           0x10 */
	_IO uint32_t	OPTCR;			/* option control                    0x14 */
	_IO uint32_t	OPTCR1;			/* option control 1                  0x18 */
} FLASH_t;

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



#endif // STM32F412_PERIPH_H
