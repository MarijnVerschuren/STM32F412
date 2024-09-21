//
// Created by marijn on 9/19/24.
//

#ifndef STM32F412_EXTI_H
#define STM32F412_EXTI_H
#include "GPIO.h"
#include "NVIC.h"


/*!< init / enable / disable */
void config_GPIO_EXTI(uint8_t EXTI_line, GPIO_t* EXTI_port, uint8_t falling_edge, uint8_t rising_edge);
void config_EXTI(uint8_t EXTI_line, uint8_t falling_edge, uint8_t rising_edge);
void start_EXTI(uint8_t EXTI_line);
void stop_EXTI(uint8_t EXTI_line);


#endif //STM32F412_EXTI_H
