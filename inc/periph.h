//
// Created by marijn on 9/7/24.
//

#ifndef ARM_BOOTLOADER_PERIPH_H
#define ARM_BOOTLOADER_PERIPH_H


/*!<
 * core hardware memory map
 * */
#define SCS_BASE			(0xE000E000UL)
#define ITM_BASE			(0xE0000000UL)
#define DWT_BASE			(0xE0001000UL)
#define TPI_BASE			(0xE0040000UL)
#define CORE_DEBUG_BASE		(0xE000EDF0UL)

#define SYS_TICK_BASE		(SCS_BASE +  0x0010UL)
#define NVIC_BASE			(SCS_BASE +  0x0100UL)
#define SCB_BASE			(SCS_BASE +  0x0D00UL)


/*!<
 * extended hardware memory map
 * */
#define FLASH_BASE			0x08000000UL
#define SRAM_BASE			0x20000000UL
#define PERIPH_BASE			0x40000000UL
// TODO BIT BAND?
// TODO: OTP?

#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE		(PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE		(PERIPH_BASE + 0x10000000UL)


/*!< APB1 */

/*!< APB2 */

/*!< AHB1 */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)

/*!< AHB2 */


#endif //ARM_BOOTLOADER_PERIPH_H
