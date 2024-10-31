/* timers.c */
#include "timers.h"
#include "pid_controller.h"
#include "encoder.h"
#include "can.h"
#include "config.h"

/* Timer handles defined in main.c */
extern TIM_HandleTypeDef htim2;  // Periodic timer for encoder/PID
extern TIM_HandleTypeDef htim3;  // Watchdog timer

/* PID Controller instance */
static PIDController pid;

/* Initialize timers and PID */
void TIMERS_Init(void)
{
    /* Initialize encoder first */
    Encoder_Init();

    /* Initialize PID controller */
    PIDController_Init(&pid, &config);

    /* Start timers */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
}

/* Watchdog reset */
void WATCHDOG_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}

/* Timer interrupt handlers */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

/* Timer callback where the main work happens */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        Update_Encoder_Angle();
        CheckMagneticField();

        /* Just call the update function - output is stored in config */
        PIDController_Update(&pid, &config);

        /* Use the output directly from config */
        // Motor_SetSpeed(config.pidOutput);

        WATCHDOG_Reset();
    }
    else if (htim == &htim3)
    {
        /* Watchdog timeout occurred */
        CAN_Log_Now("ERR:WATCHDOG_TIMEOUT");

        /* Reset PID controller */
        PIDController_Reset(&pid);

        /* Enter safe state */
        // Motor_Stop();  // You'll need to implement this
    }
}
