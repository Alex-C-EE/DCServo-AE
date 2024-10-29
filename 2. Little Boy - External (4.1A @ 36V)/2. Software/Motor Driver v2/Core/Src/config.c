#include "config.h"

/* Global configuration instance */
Config_t config;

/* Initialize configuration with default values */
void Config_Init(void) {
    config.maxRPM = 3000;
    config.encoderComm = ENCODER_COMM_SPI;
    config.encoderType = ENCODER_TYPE_INCREMENTAL;
    config.maxSpeed = 100;
    config.maxAcceleration = 50;
    config.maxJerk = 10;
    config.positionLimitUpper = 127;
    config.positionLimitLower = -127;
    config.maxErrorThreshold = 5;
    config.powerMode = POWER_MODE_NORMAL;
    config.feedbackOutput = true;
    config.kp = 1.0f;
    config.ki = 0.0f;
    config.kd = 0.0f;
    config.targetAngle = 0.0f;      // Initialize target angle
    config.currentAngle = 0.0f;     // Initialize current angle
}
