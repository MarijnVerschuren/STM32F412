//
// Created by marijn on 9/9/24.
//

#ifndef STM32F412_MEMORY_MAP_H
#define STM32F412_MEMORY_MAP_H


/*!<
 * core hardware memory map
 * */
#define SCS_BASE			(0xE000E000UL)
#define ITM_BASE			(0xE0000000UL)
#define DWT_BASE			(0xE0001000UL)
#define TPI_BASE			(0xE0040000UL)
#define CORE_DEBUG_BASE		(0xE000EDF0UL)
/*!< core hardware peripherals */
#define SYS_TICK_BASE		(SCS_BASE +  0x0010UL)
#define NVIC_BASE			(SCS_BASE +  0x0100UL)
#define SCB_BASE			(SCS_BASE +  0x0D00UL)


/*!<
 * extended hardware memory map
 * */
#define FLASH_MEMORY_BASE	0x08000000UL
#define SRAM_BASE			0x20000000UL
#define PERIPH_BASE			0x40000000UL
// TODO: BIT BAND, OTP?
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE		(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE		(PERIPH_BASE + 0x10000000UL)


/*!<
 * APB1 peripheral map
 * */


/*!<
 * APB2 peripheral map
 * */


/*!<
 * AHB1 peripheral map
 * */
#define GPIOA_BASE			(AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE			(AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE			(AHB1PERIPH_BASE + 0x0800UL)
#define GPIOH_BASE			(AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE			(AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE			(AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_BASE			(AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE			(AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE	(DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE	(DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE	(DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE	(DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE	(DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE	(DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE	(DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE	(DMA1_BASE + 0x0B8UL)
#define DMA2_BASE			(AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE	(DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE	(DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE	(DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE	(DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE	(DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE	(DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE	(DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE	(DMA2_BASE + 0x0B8UL)


/*!<
 * AHB2 peripheral map
 * */


/*!<
 * peripheral register maps
 * */
/*!< SCB */
#define SCB_CPUID		0x00
#define SCB_ICSR		0x04
#define SCB_VTOR		0x08
#define SCB_AIRCR		0x0C
#define SCB_SCR			0x10
#define SCB_CCR			0x14
#define SCB_SHP			0x18
#define SCB_SHCSR		0x24
#define SCB_CFSR		0x28
#define SCB_HFSR		0x2C
#define SCB_DFSR		0x30
#define SCB_MMFAR		0x34
#define SCB_BFAR		0x38
#define SCB_AFSR		0x3C
#define SCB_PFR			0x40
#define SCB_DFR			0x48
#define SCB_ADR			0x4C
#define SCB_MMFR		0x50
#define SCB_ISAR		0x60
#define SCB_CPACR		0x88

/*!< RCC */
#define RCC_CR			0x00
#define RCC_PLLCFGR		0x04
#define RCC_CFGR		0x08
#define RCC_CIR			0x0C
#define RCC_AHB1RSTR	0x10
#define RCC_AHB2RSTR	0x14
#define RCC_APB1RSTR	0x20
#define RCC_APB2RSTR	0x24
#define RCC_AHB1ENR		0x30
#define RCC_AHB2ENR		0x34
#define RCC_APB1ENR		0x40
#define RCC_APB2ENR		0x44
#define RCC_AHB1LPENR	0x50
#define RCC_AHB2LPENR	0x54
#define RCC_APB1LPENR	0x60
#define RCC_APB2LPENR	0x64
#define RCC_BDCR		0x70
#define RCC_CSR			0x74
#define RCC_SSCGR		0x80
#define RCC_PLLI2SCFGR	0x84
#define RCC_DCKCFGR		0x8C
#define RCC_CKGATENR	0x90
#define RCC_DCKCFGR1	0x94

/*!< FLASH */
#define FLASH_ACR		0x00
#define FLASH_KEYR		0x04
#define FLASH_OPTKEYR	0x08
#define FLASH_SR		0x0C
#define FLASH_CR		0x10
#define FLASH_OPTCR		0x14
#define FLASH_OPTCR1	0x18


#endif // STM32F412_MEMORY_MAP_H
