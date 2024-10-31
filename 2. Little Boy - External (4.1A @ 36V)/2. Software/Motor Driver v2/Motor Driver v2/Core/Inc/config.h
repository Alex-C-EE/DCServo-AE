/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Enumerations ---------------------------------------------------------------*/

/**
 * @brief  Enumeration for Encoder Communication Types.
 */
typedef enum {
    ENCODER_COMM_SPI = 1, /**< SPI Communication */
    ENCODER_COMM_I2C,     /**< I2C Communication */
    ENCODER_COMM_PWM,     /**< PWM Communication */
    ENCODER_COMM_ABZ      /**< ABZ Communication */
} EncoderComm_t;

/**
 * @brief  Enumeration for Encoder Types.
 */
typedef enum {
    ENCODER_TYPE_INCREMENTAL = 1, /**< Incremental Encoder */
    ENCODER_TYPE_ABSOLUTE         /**< Absolute Encoder */
} EncoderType_t;

/**
 * @brief  Enumeration for Power Modes.
 */
typedef enum {
    POWER_MODE_NORMAL = 1,      /**< Normal Power Mode */
    POWER_MODE_LOW_POWER        /**< Low Power Mode */
} PowerMode_t;

/**
 * @brief  Enumeration for Braking Modes.
 */
typedef enum {
    BRAKE_MODE_NONE = 0,        /**< No braking */
    BRAKE_MODE_ELECTRICAL,      /**< Electrical braking using reverse current */
    BRAKE_MODE_FREEWHEEL        /**< Let motor spin freely */
} BrakeMode_t;

/* Structures ----------------------------------------------------------------*/

/**
 * @brief  Structure to hold system status flags.
 */
typedef struct {
    uint32_t errorCount;               /**< Count of encountered errors */
    bool overcurrentFlag;              /**< Flag indicating overcurrent condition */
    bool positionLimitFlag;            /**< Flag indicating position limit breach */
    bool communicationErrorFlag;       /**< Flag indicating communication errors */
    bool brakingActiveFlag;           /**< Flag indicating active braking state */
} SystemStatus_t;

/**
 * @brief  Structure holding braking parameters.
 */
/* Braking parameter structure in config.h */
typedef struct {
    BrakeMode_t mode;                 /**< Current braking mode */
    float brakeStrength;              /**< Braking power (0.0 to 1.0) */
    float angleThreshold;             /**< Minimum angle change to consider motor stopped */
    uint32_t brakeTimeout;            /**< Maximum braking time in milliseconds */
    uint32_t angleCheckInterval;      /**< Time between angle checks in milliseconds */
    float currentLimit;               /**< Maximum current during braking */
} BrakingParams_t;

/**
 * @brief  Configuration structure holding all system parameters.
 */
typedef struct {
    /* General Settings */
    int maxRPM;                        /**< Maximum RPM allowed */
    EncoderComm_t encoderComm;         /**< Encoder Communication Type */
    EncoderType_t encoderType;         /**< Encoder Type */

    /* Motion Parameters */
    volatile int16_t maxSpeed;         /**< Maximum speed (units as per application) */
    volatile int16_t maxAcceleration;  /**< Maximum acceleration (units as per application) */
    volatile int16_t maxJerk;          /**< Maximum jerk (units as per application) */
    int16_t positionLimitUpper;        /**< Upper position limit */
    int16_t positionLimitLower;        /**< Lower position limit */
    int16_t maxErrorThreshold;         /**< Maximum error threshold for deadband */

    /* Power Management */
    PowerMode_t powerMode;             /**< Current power mode */
    bool feedbackOutput;               /**< Flag to enable/disable feedback output */

    /* PID Controller Parameters */
    double kp;                          /**< Proportional gain */
    double ki;                          /**< Integral gain */
    double kd;                          /**< Derivative gain */
    volatile double targetAngle;        /**< Target angle for the controller */
    volatile double currentAngle;       /**< Current measured angle */
    volatile double currentVelocity;    /**< Current measured velocity */
    volatile double pidOutput;          /**< Output from the PID controller */

    /* Enhanced PID Parameters */
    double tau;                         /**< Time constant for derivative filter */
    double Kaw;                         /**< Back-calculation anti-windup gain */
    double disturbanceThreshold;        /**< Threshold to detect significant disturbances */

    /* Braking Configuration */
    BrakingParams_t braking;            /**< Structure holding braking parameters */

    /* System Status */
    SystemStatus_t status;              /**< Structure holding system status flags */

    /* Configuration Management */
    uint32_t configVersion;             /**< Version identifier for configuration */
} Config_t;

/* External Variables ---------------------------------------------------------*/

/**
 * @brief  Externally accessible global configuration instance.
 */
extern Config_t config;

/* Function Prototypes --------------------------------------------------------*/

/**
 * @brief  Initializes the system configuration with default or preset values.
 * @retval None
 */
void Config_Init(void);

/**
 * @brief  Validates the current configuration parameters.
 * @retval true   If the configuration is valid.
 * @retval false  If the configuration is invalid.
 */
bool Config_Validate(void);

/**
 * @brief  Sets the PID controller parameters.
 * @param  kp   Proportional gain.
 * @param  ki   Integral gain.
 * @param  kd   Derivative gain.
 * @retval None
 */
void Config_SetPIDParameters(double kp, double ki, double kd);

/**
 * @brief  Sets the upper and lower position limits.
 * @param  lower  Lower position limit.
 * @param  upper  Upper position limit.
 * @retval None
 */
void Config_SetPositionLimits(int16_t lower, int16_t upper);

/**
 * @brief  Sets the target angle for the controller.
 * @param  angle  Desired target angle in degrees.
 * @retval None
 */
void Config_SetTargetAngle(double angle);

/**
 * @brief  Configures the braking parameters.
 * @param  mode            Braking mode to use.
 * @param  brakeStrength   Braking power (0.0 to 1.0).
 * @param  angleThreshold  Minimum angle change to consider motor stopped.
 * @param  timeout         Maximum braking time in milliseconds.
 * @param  currentLimit    Maximum current during braking.
 * @retval None
 */
void Config_SetBrakingParameters(BrakeMode_t mode, float brakeStrength,
                               float angleThreshold, uint32_t timeout,
                               float currentLimit);

/**
 * @brief  Resets the system status flags to their default states.
 * @retval None
 */
void Config_ResetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
