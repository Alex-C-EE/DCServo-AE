/* motor_control.c */
#include "motor_control.h"
#include "main.h" // Contains handles for TIM1 and ADC1
#include <math.h> // For fabs function
#include <stdbool.h>

/* External handles from main.c */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

/* Configuration structure */
typedef struct {
    float brakeStrength;         // Braking power (0.0 to 1.0)
    float angleThreshold;        // Minimum angle change to consider motor stopped
    uint32_t brakeTimeout;       // Maximum braking time in milliseconds
    uint32_t angleCheckInterval; // Time between angle checks in milliseconds
} Motor_Config_t;

/* Private variables */
static float currentDutyCycle = 0.0f;
static Motor_Direction_t currentDirection = MOTOR_STOP;
static Motor_Direction_t lastDirection = MOTOR_STOP;
static bool isBraking = false;
static uint32_t brakeStartTime = 0;
static float lastAngle = 0.0f;
static uint32_t lastAngleCheckTime = 0;

/* Default configuration */
static Motor_Config_t config = {
    .brakeStrength = 0.3f,        // 30% brake strength
    .angleThreshold = 0.1f,       // 0.1 degree threshold
    .brakeTimeout = 2000,         // 2 second timeout
    .angleCheckInterval = 50      // Check angle every 50ms
};

/* Private function prototypes */
static void UpdatePWMOutputs(void);
static bool IsMotorStopped(float currentAngle);

/* Initialize motor control */
void Motor_Init(void) {
    // Start TIM1 PWM channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Forward channel
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // Backward channel

    // Enable ADC watchdog on channel 11
    HAL_ADC_Start_IT(&hadc1);

    // Set initial state
    Motor_Stop();
}

/* Configure motor parameters */
void Motor_Configure(float brakeStrength, float angleThreshold,
                    uint32_t brakeTimeout, uint32_t angleCheckInterval) {
    config.brakeStrength = fmaxf(0.0f, fminf(brakeStrength, 1.0f));
    config.angleThreshold = angleThreshold;
    config.brakeTimeout = brakeTimeout;
    config.angleCheckInterval = angleCheckInterval;
}

/* Set motor speed directly from PID output */
void Motor_SetSpeed(double pidOutput) {
    // Store last direction before changing it
    lastDirection = currentDirection;

    // PID output is already clamped to ±100%
    float drivingValue = (float)pidOutput;

    // Determine direction based on polarity
    if (drivingValue > 0.0f) {
        currentDirection = MOTOR_FORWARD;
    } else if (drivingValue < 0.0f) {
        currentDirection = MOTOR_BACKWARD;
    } else {
        currentDirection = MOTOR_STOP;
    }

    // Convert to duty cycle (0.0 to 1.0)
    currentDutyCycle = fabs(drivingValue) / 100.0f;
    isBraking = false;  // Clear braking flag when setting new speed

    UpdatePWMOutputs();
}

/* Emergency stop without braking */
void Motor_Stop(void) {
    lastDirection = currentDirection;
    currentDutyCycle = 0.0f;
    currentDirection = MOTOR_STOP;
    isBraking = false;
    UpdatePWMOutputs();
}

/* Dynamic braking */
void Motor_Brake(void) {
    if (!isBraking) {
        // Initialize braking
        lastDirection = currentDirection;
        brakeStartTime = HAL_GetTick();
        isBraking = true;
        lastAngleCheckTime = brakeStartTime;

        // Apply reverse current based on last direction
        currentDutyCycle = config.brakeStrength;
        currentDirection = (lastDirection == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;

        UpdatePWMOutputs();
    }
}

/* Update braking state - should be called in main loop */
void Motor_UpdateBraking(float currentAngle) {
    if (!isBraking) {
        return;
    }

    uint32_t currentTime = HAL_GetTick();

    // Check if it's time to measure angle change
    if (currentTime - lastAngleCheckTime >= config.angleCheckInterval) {
        if (IsMotorStopped(currentAngle) ||
            (currentTime - brakeStartTime >= config.brakeTimeout)) {
            // Motor has stopped or timeout reached - end braking
            Motor_Stop();
            return;
        }

        // Update last angle for next check
        lastAngle = currentAngle;
        lastAngleCheckTime = currentTime;
    }
}

/* Private: Check if motor has effectively stopped */
static bool IsMotorStopped(float currentAngle) {
    float angleChange = fabs(currentAngle - lastAngle);
    return angleChange < config.angleThreshold;
}

/* Get current from DRV8251 current sense output */
float Motor_GetCurrent(void) {
    // Start ADC conversion
    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        uint32_t adcValue = HAL_ADC_GetValue(&hadc1);

        // DRV8251 current sense conversion
        // Assuming 3.3V reference and 12-bit ADC
        float voltage = (adcValue / 4095.0f) * 3.3f;

        // Convert voltage to current based on DRV8251 specs
        float current = voltage / 0.5f; // Adjust ratio as needed

        return current;
    }

    return -1.0f; // Error case
}

/* Private: Update PWM outputs */
static void UpdatePWMOutputs(void) {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pulseValue = (uint32_t)(currentDutyCycle * period);

    switch (currentDirection) {
        case MOTOR_FORWARD:
            // For forward: CH1 active, CH2 inactive
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulseValue);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            break;

        case MOTOR_BACKWARD:
            // For backward: CH2 active, CH1 inactive
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulseValue);
            break;

        case MOTOR_STOP:
        default:
            // Stop motor by setting both channels to 0
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            break;
    }
}

/* ADC Watchdog callback for overcurrent protection */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        // Overcurrent detected - emergency stop
        Motor_Stop();
        // Report error over CAN
        CAN_Log_Now("ERR:OVERCURRENT");
    }
}

/* Get current brake status */
bool Motor_IsBraking(void) {
    return isBraking;
}

/* Get last known direction */
Motor_Direction_t Motor_GetLastDirection(void) {
    return lastDirection;
}
