#ifndef TIMERS_H
#define TIMERS_H

#include "stm32g0xx_hal.h"  // Adjust this include based on your STM32 series

/* Function prototypes */
void TIMERS_Init(void);
void TIMERS_Stop(TIM_HandleTypeDef *htim);
void WATCHDOG_Reset(void);

/* External variables */
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

#endif /* TIMERS_H */
