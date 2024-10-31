/* pid_controller.h */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "config.h"  // Ensure Config_t is defined in this header

/* Typedef -------------------------------------------------------------------*/

/**
 * @brief  Structure to represent a PID controller.
 */
typedef struct {
    /* Controller State Variables */
    double integrator;        /**< Integral term accumulator */
    double prevError;         /**< Previous error value for derivative calculation */
    double differentiator;    /**< Derivative term */
    double prevMeasurement;   /**< Previous measurement to prevent derivative kick */
    double out;               /**< Controller output */

    /* Controller Gains */
    double Kp;                /**< Proportional gain */
    double Ki;                /**< Integral gain */
    double Kd;                /**< Derivative gain */

    /* Output Limits */
    double limMin;            /**< Minimum output limit */
    double limMax;            /**< Maximum output limit */

    /* Additional Parameters */
    double tau;               /**< Time constant for derivative filter */
    double Kaw;               /**< Back-calculation anti-windup gain */
    double T;                 /**< Sample time in seconds */
} PIDController;

/* Exported Functions --------------------------------------------------------*/

/**
 * @brief  Initializes the PID controller with configuration parameters.
 * @param  pid Pointer to the PIDController structure to initialize.
 * @param  config Pointer to the Config_t structure containing configuration parameters.
 * @retval None
 */
void PIDController_Init(PIDController *pid, const Config_t *config);

/**
 * @brief  Updates the PID controller gains and output limits based on new configuration.
 * @param  pid Pointer to the PIDController structure to update.
 * @param  config Pointer to the Config_t structure containing updated configuration parameters.
 * @retval None
 */
void PIDController_UpdateGains(PIDController *pid, const Config_t *config);

/**
 * @brief  Executes a single update cycle of the PID controller.
 *         Computes the controller output based on current measurements and target.
 * @param  pid Pointer to the PIDController structure.
 * @param  config Pointer to the Config_t structure containing current state and targets.
 * @retval None
 *
 * @note   The function updates the `pidOutput` field in the `Config_t` structure.
 */
void PIDController_Update(PIDController *pid, Config_t *config);

/**
 * @brief  Resets the internal state of the PID controller.
 *         Clears integrator and differentiator terms to prevent windup.
 * @param  pid Pointer to the PIDController structure to reset.
 * @retval None
 */
void PIDController_Reset(PIDController *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
