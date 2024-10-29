#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

/* Encoder Communication Types */
typedef enum {
    ENCODER_COMM_SPI = 1,
    ENCODER_COMM_I2C,
    ENCODER_COMM_PWM,
    ENCODER_COMM_ABZ
} EncoderComm_t;

/* Encoder Types */
typedef enum {
    ENCODER_TYPE_INCREMENTAL = 1,
    ENCODER_TYPE_ABSOLUTE
} EncoderType_t;

/* Power Modes */
typedef enum {
    POWER_MODE_NORMAL = 1,
    POWER_MODE_LOW_POWER
} PowerMode_t;

/* Configuration Structure */
typedef struct {
    int maxRPM; //
    EncoderComm_t encoderComm; //
    EncoderType_t encoderType; //
    volatile int8_t maxSpeed; //
    volatile int8_t maxAcceleration; //
    volatile int8_t maxJerk; //
    int8_t positionLimitUpper; ///
    int8_t positionLimitLower; //
    int8_t maxErrorThreshold; //
    PowerMode_t powerMode; //
    bool feedbackOutput; //

    /* PID Parameters */
    volatile double kp;               /* Proportional gain */
    volatile double ki;               /* Integral gain */
    volatile double kd;               /* Derivative gain */
    volatile double targetAngle;     /* Desired setpoint */
    volatile double currentAngle;    /* Current measurement, set by SPI Reading */

    /* Additional Parameters for Enhanced PID */
    double tau;                   /* Derivative filter time constant (3.a) */
    double Kaw;                   /* Back-calculation anti-windup gain (3.d) */
    double disturbanceThreshold;  /* Threshold to detect significant disturbances (3.f) */
} Config_t;

/* Externally accessible configuration instance */
extern Config_t config;

/* Initialization function */
void Config_Init(void);

#endif /* CONFIG_H */
