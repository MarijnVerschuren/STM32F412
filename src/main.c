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
#include "pwm.h"

#include "usb/hid.h"
#include "usb/usb.h"

#include "fn.h"


/*!<
 * defines
 * */
//#define CUSTOM
#if defined(CUSTOM)
#define LED_PORT GPIOB
#define LED_PIN	 0

/*!< pin array
 * |A1, A0|		|__, __|
 * |A3, A2|		|RX, TX|
 * |B5, B4|		|__, __|
 * |B7, B6|		|__, __|
 * */

#else
#define LED_PORT GPIOC
#define LED_PIN	 13
#endif


uint8_t HID_buffer[8] = {0, 0, 0x4, 0, 0, 0, 0, 0};


/*!<
 * IRQ
 * */
extern volatile uint8_t USB_IRQ_log[256];
extern volatile uint8_t USB_IRQ_cnt;


void config_USB(
	USB_OTG_GlobalTypeDef* usb,
	uint8_t enable_SOF, uint8_t enable_low_power
);
void start_USB(USB_OTG_GlobalTypeDef* usb);

// application
int main(void) {
	set_SYS_hardware_config(PWR_LVL_NOM, 25000000);									// 3v3, HSE@25MHz
	set_SYS_oscilator_config(0, 1, 0, 1, 1, 0, 0, 0, 0);							// enable HSE, LSE, CSS and the backup-domain
	set_SYS_PLL_config(PLL_CLK_SRC_HSE, 25U, 192U, PLL_P_DIV_2, PLL_P_DIV_4, 0);	// enable PLL @ 25MHz / 25 * 192 / (2 -> 98MHz, 4 -> 48MHz)
	set_SYS_backup_domain_config();													// enable backup domain
	//set_SYS_RTC_config(RTC_SRC_LSE, 0U);											// enable RTC
	set_SYS_clock_config(SYS_CLK_SRC_PLL_P, AHB_CLK_NO_DIV, APBx_CLK_DIV2, APBx_CLK_NO_DIV);
	set_SYS_tick_config(1);															// enable SYS_tick with interrupt
	sys_init();																		// write settings

	config_GPIO(LED_PORT, LED_PIN, GPIO_output | GPIO_push_pull);
	MX_USB_DEVICE_Init();
//USB_OFF:
//	GPIO_write(LED_PORT, LED_PIN, 0);
//	while (hUsbDeviceFS.dev_state != DEV_STATE_CONFIGURED);
//	GPIO_write(LED_PORT, LED_PIN, 1);
//
//	uint8_t code[6] = {0, 0, 0, 0, 0, 0};
//	uint8_t i;
//	uint8_t delay = 18;  // min: 18
//	// main loop
//	for(;;) {
//		if (USB_handle.dev_state != DEV_STATE_CONFIGURED) { goto USB_OFF; }
//		delay_ms(1000);
//		for (i = 0; i < 6; i++) {
//			HID_buffer[2] = code[i] + 0x1E;
//			send_HID_report(&USB_handle, HID_buffer, 8);
//			delay_ms(delay);
//			HID_buffer[2] = 0;
//			send_HID_report(&USB_handle, HID_buffer, 8);
//			delay_ms(delay);
//		}
//		HID_buffer[2] = 0x28;
//		send_HID_report(&USB_handle, HID_buffer, 8);
//		delay_ms(delay);
//		HID_buffer[2] = 0;
//		send_HID_report(&USB_handle, HID_buffer, 8);
//		delay_ms(delay);
//
//		for (i = 0; i < 6; i++) {
//			code[i] = (code[i] + 1) % 10;
//			if (code[i]) { break; }
//		}
//	}
	for(;;) {
		delay_ms(100);
		GPIO_toggle(LED_PORT, LED_PIN);
	}
}
// https://stackoverflow.com/questions/38695895/override-a-weak-function-a-with-a-function-b
