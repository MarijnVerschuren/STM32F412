//
// Created by marijn on 3/2/25.
//

#ifndef STM32F412_PWM_H
#define STM32F412_PWM_H
#include "tim.h"
#include "GPIO.h"

/*!< init / enable / disable */
void config_PWM(TIM_GPIO_t pwm_pin, uint32_t prescaler, uint32_t period);

#endif //STM32F412_PWM_H
