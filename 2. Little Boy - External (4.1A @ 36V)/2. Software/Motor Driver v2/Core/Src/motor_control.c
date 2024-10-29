/* motor_control.c */

#include "motor_control.h"
#include "main.h"  // Contains handles for htim2 and hadc1
#include <math.h>  // For fabsf function

/* Declare external variables */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

/* Private variables */
static float currentDutyCycle = 0.0f;
static Motor_Direction_t currentDirection = MOTOR_STOP;

/* Private function prototypes */
static void UpdatePWMOutputs(void);

/* Public functions */

/**
 * @brief  Initializes the motor control module.
 * @retval None
 */
void Motor_Init(void) {
    // Configure Timer for 200kHz PWM
    // Assuming Timer clock is already set appropriately
    // Ensure the timer is stopped before reconfiguring
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);

    // Start PWM channels
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // CH1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // CH2

    // Initialize ADC and start in interrupt mode for the watchdog
    HAL_ADC_Start_IT(&hadc1);

    // Set initial motor state
    Motor_Stop();
}

/**
 * @brief  Sets the motor driving value between -100 and 100.
 * @param  drivingValue: Value between -100 and 100
 * @retval None
 */
void Motor_SetDrivingValue(float drivingValue) {
    // Clamp the driving value between -100 and 100
    if (drivingValue > 100.0f) drivingValue = 100.0f;
    if (drivingValue < -100.0f) drivingValue = -100.0f;

    // Determine direction
    if (drivingValue > 0.0f) {
        currentDirection = MOTOR_FORWARD;
    } else if (drivingValue < 0.0f) {
        currentDirection = MOTOR_BACKWARD;
    } else {
        currentDirection = MOTOR_STOP;
    }

    // Calculate duty cycle
    currentDutyCycle = fabsf(drivingValue) / 100.0f;  // Convert to 0.0 to 1.0

    UpdatePWMOutputs();
}

/**
 * @brief  Stops the motor.
 * @retval None
 */
void Motor_Stop(void) {
    currentDutyCycle = 0.0f;
    currentDirection = MOTOR_STOP;
    UpdatePWMOutputs();
}

/**
 * @brief  Gets the motor current by reading the ADC.
 * @retval Current in amperes (or voltage proportional if calibration is needed)
 */
float Motor_GetCurrent(void) {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
        float voltage = (adcValue / 4095.0f) * 3.3f;  // Assuming 12-bit ADC and 3.3V reference
        // Convert voltage to current as per your sensor's characteristics
        // For example, using a shunt resistor:
        // float current = voltage / shunt_resistance;
        return voltage;  // Replace with actual current calculation
    } else {
        // Handle ADC read error
        return -1.0f;  // Indicate an error
    }
}

/* Private functions */

/**
 * @brief  Updates the PWM outputs based on the current duty cycle and direction.
 * @retval None
 */
static void UpdatePWMOutputs(void) {
    uint32_t pulseValue = (uint32_t)(currentDutyCycle * (__HAL_TIM_GET_AUTORELOAD(&htim2) + 1));

    switch (currentDirection) {
        case MOTOR_FORWARD:
            // CH1 active, CH2 inactive
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulseValue);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            break;

        case MOTOR_BACKWARD:
            // CH2 active, CH1 inactive
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulseValue);
            break;

        case MOTOR_STOP:
        default:
            // Both channels inactive
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            break;
    }
}

/* ADC Watchdog Callback */
/**
 * @brief  This function is called when the ADC Watchdog threshold is exceeded.
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == hadc1.Instance) {
        // Stop the motor when current exceeds threshold
        Motor_Stop();
        // Optionally, set a flag or take additional actions
    }
}
