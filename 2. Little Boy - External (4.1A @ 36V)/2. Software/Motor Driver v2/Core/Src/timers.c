#include "timers.h"
#include "pid_controller.h"
#include "encoder.h"
#include "can.h"

// TODO: Fix this shit...

// What if: Timer Triggers CS Low, CS Low Results in SPI_Receive_IT which we can then use to trigger PID?
// Technically high speed enough for T to still be 1ms

/* Timer handles */
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

extern uint16_t realAngle;


/* TIMERS_Init: Initialize TIM13 and TIM14 */
void TIMERS_Init(void)
{
    /* Initialize TIM14 as a periodic interrupt timer */
    HAL_TIM_Base_Start_IT(&htim14);

    /* Initialize TIM13 as a watchdog timer */
    HAL_TIM_Base_Start_IT(&htim13);
}

/* TIMERS_Stop: Stop the specified timer */
void TIMERS_Stop(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
}

/* WATCHDOG_Reset: Reset the watchdog timer */
void WATCHDOG_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim13, 0);
}

/* Interrupt handlers */

/* TIM14 Interrupt Handler */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim14);
}

/* TIM13 Interrupt Handler */
void TIM8_UP_TIM13_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim13);
}

/* Callback function called when TIM13 or TIM14 interrupt occurs */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim14)
    {
    	realAngle = Read_MA730_Angle();
    }
    else if (htim == &htim13)
    {
        /* TIM13 (watchdog) timeout occurred */
        /* Watchdog timeout handling code here */
        // Example: Reset the system or log the event
    }
}
