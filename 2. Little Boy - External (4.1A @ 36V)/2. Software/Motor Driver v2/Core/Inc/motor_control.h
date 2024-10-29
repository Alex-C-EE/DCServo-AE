/* motor_control.h */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"  // Adjust the header according to your STM32 series
#include <stdint.h>

/* Exported Types ------------------------------------------------------------*/

/**
 * @brief  Enumeration for motor direction.
 */
typedef enum {
    MOTOR_FORWARD = 0,
    MOTOR_BACKWARD,
    MOTOR_STOP
} Motor_Direction_t;

/* Exported Functions --------------------------------------------------------*/

/**
 * @brief  Initializes the motor control module.
 * @retval None
 */
void Motor_Init(void);

/**
 * @brief  Sets the motor driving value between -100 and 100.
 * @param  drivingValue: Value between -100 and 100 representing duty cycle and direction.
 * @retval None
 */
void Motor_SetDrivingValue(float drivingValue);

/**
 * @brief  Stops the motor.
 * @retval None
 */
void Motor_Stop(void);

/**
 * @brief  Gets the motor current by reading the ADC.
 * @retval Current in amperes (or voltage proportional if calibration is needed).
 */
float Motor_GetCurrent(void);

/* Optional: If other modules need to be aware of the ADC callback, you can declare it here. */
/**
 * @brief  Callback function called when the ADC Watchdog threshold is exceeded.
 * @param  hadc: ADC handle.
 * @retval None
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CONTROL_H */
