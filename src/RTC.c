//
// Created by marijn on 9/17/24.
//
#include "RTC.h"


// 22.3.8
void fconfig_RTC(
	uint8_t async_pre, uint16_t sync_pre, RTC_timestamp_t time,
	RTC_wakeup_t wakeup, RTC_wakeup_div_t wakeup_div, uint16_t wakeup_reload
) {
	RCC->APB1ENR |= 0x400;					// enable RCC bus clock
	PWR->CR |= 0x100UL;						// enable BDP
	RTC->WPR = 0xCAUL;						// write key 0 into the write protect register
	RTC->WPR = 0x53UL;						// write key 1 into the write protect register

	RTC->ISR |=	0x80UL;						// enter initialization mode (RTC is stopped)
	while (!(RTC->ISR & 0x40UL));			// wait until RTC enters initialization mode
	RTC->PRER = (
		((async_pre & 0x7F) << 16)	|		// asynchronous prescaler
		((sync_pre & 0x7FFF) << 0)			// synchronous prescaler
	);

	RTC->CR =  (
		0x00000020UL					|	// enable shadow register bypass
		((wakeup & 0b1U) << 10U)		|	// set wake-up enable setting
		(((wakeup >> 1) & 0b1U) << 14U)	|	// set wake-up interrupt setting
		(wakeup_div << 0U)					// set wake-up clock setting
	);
	RTC->WUTR = wakeup_reload;				// set wake-up timer reload
	RTC->DR = (
		((time.year / 10U) << 20U)	|		// set year tens
		((time.year % 10U) << 16U)	|		// set year units
		((time.month / 10U) << 12U)	|		// set month tens
		((time.month % 10U) << 8U)	|		// set month units
		((time.day_unit) << 13U)	|		// set week day unit
		((time.day / 10U) << 4U)	|		// set day tens
		((time.day % 10U) << 0U)			// set day units
	);
	RTC->TR = (
		((time.hour / 10U) << 20U)	|		// set hour tens
		((time.hour % 10U) << 16U)	|		// set hour units
		((time.min / 10U) << 12U)	|		// set min tens
		((time.min % 10U) << 8U)	|		// set min units
		((time.sec / 10U) << 4U)	|		// set sec tens
		((time.sec % 10U) << 0U)			// set sec units
	);


	RTC->ISR &=	~0x80UL;					// exit initialization mode (RTC is started)
	while (RTC->ISR & 0x40UL);				// wait until RTC exits initialization mode
	RTC->WPR = 0x0UL;						// re-enable write protection
	PWR->CR &= ~0x100UL;					// disable BDP
}

void config_RTC(RTC_timestamp_t time, RTC_wakeup_t wakeup, RTC_wakeup_div_t wakeup_div, uint16_t wakeup_reload) {
	fconfig_RTC(
		RTC_clock_frequency / 0x100,
		0x100, time, wakeup, wakeup_div,
		wakeup_reload
	);
}
void sconfig_RTC(RTC_timestamp_t time) {
	fconfig_RTC(
		RTC_clock_frequency / 0x100U, 0x100U,
		time, RTC_WAKEUP_DISABLE, RTC_WAKEUP_DIV16,
		0x0000U
	);
}

void reset_RTC() {
	// TODO
}

RTC_timestamp_t get_RTC(void) {
	uint32_t TR = RTC->TR;
	uint32_t DR = RTC->TR;
	RTC_timestamp_t t;
	t.year =	(((DR >> 20U) & 0xFU) * 10) + ((DR >> 16U) & 0xFU);
	t.month =	(((DR >> 12U) & 0x1U) * 10) + ((DR >> 8U) & 0xFU);
	t.day =		(((DR >> 4U) & 0x3U) * 10) + ((DR >> 0U) & 0xFU);
	t.day_unit =((DR >> 13U) & 0x7U);
	t.hour =	(((TR >> 20U) & 0x3U) * 10) + ((TR >> 16U) & 0xFU);
	t.min =		(((TR >> 12U) & 0x7U) * 10) + ((TR >> 8U) & 0xFU);
	t.sec =		(((TR >> 4U) & 0x7U) * 10) + ((TR >> 0U) & 0xFU);
	return t;
}