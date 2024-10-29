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
    int8_t maxSpeed; //
    int8_t maxAcceleration; //
    int8_t maxJerk; //
    int8_t positionLimitUpper; ///
    int8_t positionLimitLower; //
    int8_t maxErrorThreshold; //
    PowerMode_t powerMode; //
    bool feedbackOutput; //
    float kp;
    float ki;
    float kd;
    float targetAngle;     //
    float currentAngle;    // Set by SPI Reading
} Config_t;

/* Externally accessible configuration instance */
extern Config_t config;

/* Initialization function */
void Config_Init(void);

#endif /* CONFIG_H */
