#include "base.h"
#include "periph.h"
#include "sys.h"
#include "GPIO.h"
#include "USART.h"
#include "RTC.h"
#include "I2C.h"
#include "ADC.h"
#include "tim.h"
#include "SPI.h"

#include "fn.h"


/*!<
 * IRQ
 * */
void EXTI2_handler(void) {
	EXTI->PR = 0x00000004UL;
}


// application
void main(void) {
	set_SYS_hardware_config(PWR_LVL_NOM, 25000000);						// 3v3, HSE@25MHz
	//set_SYS_oscilator_config(0, 1, 0, 1, 1, 0, 0, 0, 0);				// enable HSE, LSE, CSS and the backup-domain
	set_SYS_oscilator_config(0, 1, 0, 0, 1, 0, 0, 0, 0);				// enable HSE, CSS and the backup-domain
	set_SYS_PLL_config(PLL_CLK_SRC_HSE, 12U, 96U, PLL_P_DIV_4, 0, 0);	// enable PLL @ 25MHz / 12 * 96 / 2 -> 100MHz
	//set_SYS_backup_domain_config();									// enable backup domain
	//set_SYS_RTC_config(RTC_SRC_LSE, 0U);								// enable RTC
	set_SYS_clock_config(SYS_CLK_SRC_PLL_P, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV);
	set_SYS_tick_config(1);												// enable SYS_tick with interrupt
	sys_init();															// write settings

	// GPIO
	config_GPIO(GPIOA, 3, GPIO_output | GPIO_no_pull | GPIO_push_pull);
	config_GPIO(GPIOA, 4, GPIO_output | GPIO_no_pull | GPIO_push_pull);

	// EXTI TODO: flags??
	config_EXTI_GPIO(GPIOA, 2, 1, 1);
	NVIC_set_IRQ_priority(EXTI2_IRQn, 0);

	// ADC
	//config_ADC(ADC_CLK_DIV2 | ADC_INJ_DISC | ADC_RES_12B | ADC_EOC_SINGLE | ADC_INJ_TRIG_TIM1_TRGO | ADC_INJ_TRIG_MODE_RISING);
	//config_ADC_watchdog(0, ADC_WDG_TYPE_INJECTED, 200, 3900);
	//config_ADC_IRQ(1, ADC_IRQ_JEOC | ADC_IRQ_WDG);
	//config_ADC_GPIO_inj_channel(GPIOA, 0, ADC_SAMPLE_28_CYCLES, 409, 0);

	// TIM
	//config_TIM_master(TIM1, 10000, 100, TIM_TRGO_UPDATE);  // 100 Hz
	//start_TIM_update_irq(TIM1);

	// USART
	//config_UART(USART1_TX_A9, USART1_RX_A10, 115200);

	// I2C
	//config_I2C(I2C2_B10_SCL, I2C2_B9_SDA, 0x00);

	/*!< SPI */
	GPIO_write(GPIOA, 4, 1);
	config_SPI_master(
		SPI1_SCK_A5, SPI1_MOSI_A7, SPI1_MISO_A6,
		SPI_ENDIANNESS_MSB | SPI_CPHA_FIRST_EDGE |
		SPI_CPOL_LOW | SPI_MODE_DUPLEX | SPI_FRAME_MOTOROLA |
		SPI_FIFO_TH_HALF | SPI_DATA_8 | SPI_CLK_DIV_16
	);
	io_buffer_t* buffer = init_io_buffer(64, IO_BUFFER_FIFO | IO_BUFFER_IEN);
	// RTC
	//RTC_timestamp_t test = UNIX_BCD(1735689599);
	//uint32_t ts = 1726832418;
	//uconfig_RTC(ts, RTC_WAKEUP_DISABLE, RTC_WAKEUP_DIV16, 0x0000U);
	//config_RTC_ext_ts(1U, RTC_TS_POLARITY_RISING);

	uint8_t buf[20];
	for (uint8_t i = 0; i < 20; i++) { buf[i] = i; }

	for (;;) {
		GPIO_write(GPIOA, 4, 0);
		SPI_write8(SPI1, &buf, 20, 20);
		GPIO_write(GPIOA, 4, 1);
		delay_ms(10);
		//DW1000_responder(&dw1000);
	}
}
// https://stackoverflow.com/questions/38695895/override-a-weak-function-a-with-a-function-b
