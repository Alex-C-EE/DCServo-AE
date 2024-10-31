/* config.c */
#include "config.h"

/* Global configuration instance */
Config_t config;

/* Default configuration values */
static const Config_t default_config = {
    .maxRPM = 1000,
    .encoderComm = ENCODER_COMM_SPI,
    .encoderType = ENCODER_TYPE_ABSOLUTE,
    .maxSpeed = 100,
    .maxAcceleration = 50,
    .maxJerk = 25,
    .positionLimitUpper = 360,
    .positionLimitLower = 0,
    .maxErrorThreshold = 5,
    .powerMode = POWER_MODE_NORMAL,
    .feedbackOutput = true,

    /* PID Parameters */
    .kp = 1.0,
    .ki = 0.1,
    .kd = 0.01,
    .targetAngle = 0.0,
    .currentAngle = 0.0,
    .currentVelocity = 0.0,
    .pidOutput = 0.0,

    /* Enhanced PID Parameters */
    .tau = 0.02,
    .Kaw = 0.1,
    .disturbanceThreshold = 5.0,

    /* Braking Parameters */
    .braking = {
        .mode = BRAKE_MODE_ELECTRICAL,
        .brakeStrength = 0.3f,         // 30% default brake strength
        .angleThreshold = 0.1f,        // 0.1 degree threshold
        .brakeTimeout = 2000,          // 2 second timeout
        .angleCheckInterval = 50,       // Check every 50ms
        .currentLimit = 10.0f          // 10A current limit during braking
    },

    /* System Status */
    .status = {
        .errorCount = 0,
        .overcurrentFlag = false,
        .positionLimitFlag = false,
        .communicationErrorFlag = false,
        .brakingActiveFlag = false
    },

    /* Configuration Version */
    .configVersion = 1
};

void Config_Init(void) {
    // Copy default configuration to working configuration
    config = default_config;
}

bool Config_Validate(void) {
    // Check for valid ranges
    if (config.maxRPM <= 0) return false;
    if (config.maxSpeed <= 0) return false;
    if (config.maxAcceleration <= 0) return false;
    if (config.maxJerk <= 0) return false;

    // Check PID parameters
    if (config.kp < 0.0 || config.ki < 0.0 || config.kd < 0.0) return false;

    // Check position limits
    if (config.positionLimitLower >= config.positionLimitUpper) return false;

    // Check enhanced PID parameters
    if (config.tau <= 0.0 || config.Kaw < 0.0) return false;

    // Validate braking parameters
    if (config.braking.brakeStrength < 0.0f || config.braking.brakeStrength > 1.0f) return false;
    if (config.braking.angleThreshold < 0.0f) return false;
    if (config.braking.brakeTimeout == 0) return false;
    if (config.braking.angleCheckInterval == 0) return false;
    if (config.braking.currentLimit <= 0.0f) return false;

    return true;
}

void Config_SetPIDParameters(double kp, double ki, double kd) {
    config.kp = kp;
    config.ki = ki;
    config.kd = kd;
}

void Config_SetPositionLimits(int16_t lower, int16_t upper) {
    if (lower < upper) {
        config.positionLimitLower = lower;
        config.positionLimitUpper = upper;
    }
}

void Config_SetTargetAngle(double angle) {
    // Constrain target angle to position limits
    if (angle > config.positionLimitUpper) {
        config.targetAngle = (double)config.positionLimitUpper;
    } else if (angle < config.positionLimitLower) {
        config.targetAngle = (double)config.positionLimitLower;
    } else {
        config.targetAngle = angle;
    }
}

void Config_SetBrakingParameters(BrakeMode_t mode, float brakeStrength,
                               float angleThreshold, uint32_t timeout,
                               float currentLimit) {
    // Validate and set braking parameters
    config.braking.mode = mode;

    // Clamp brake strength between 0 and 1
    config.braking.brakeStrength = (brakeStrength < 0.0f) ? 0.0f :
                                  (brakeStrength > 1.0f) ? 1.0f : brakeStrength;

    // Ensure positive angle threshold
    config.braking.angleThreshold = (angleThreshold < 0.0f) ? 0.0f : angleThreshold;

    // Ensure minimum timeout of 100ms
    config.braking.brakeTimeout = (timeout < 100) ? 100 : timeout;

    // Ensure positive current limit
    config.braking.currentLimit = (currentLimit <= 0.0f) ?
                                 default_config.braking.currentLimit : currentLimit;
}

void Config_ResetStatus(void) {
    config.status.errorCount = 0;
    config.status.overcurrentFlag = false;
    config.status.positionLimitFlag = false;
    config.status.communicationErrorFlag = false;
    config.status.brakingActiveFlag = false;
}
