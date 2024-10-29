#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "config.h"

/* PID Controller structure */
typedef struct {
    double integrator;       /* Integral term */
    double prevError;        /* Previous error */
    double differentiator;   /* Derivative term */
    double prevMeasurement;  /* Previous measurement */
    double out;              /* Controller output */

    double Kp;               /* Proportional gain */
    double Ki;               /* Integral gain */
    double Kd;               /* Derivative gain */

    double limMin;           /* Minimum output limit */
    double limMax;           /* Maximum output limit */

    double tau;              /* Derivative filter time constant */
    double T;                /* Sample time */

    double Kaw;              /* Back-calculation anti-windup gain */
} PIDController;

/* Function Prototypes */

/* Initialize PID Controller */
void PIDController_Init(PIDController *pid, const Config_t *config);

/* Update controller gains and limits from config */
void PIDController_UpdateGains(PIDController *pid, const Config_t *config);

/* PID Controller Update Function */
double PIDController_Update(PIDController *pid, const Config_t *config);

/* Reset the controller's internal state */
void PIDController_Reset(PIDController *pid);

#endif /* PID_CONTROLLER_H */
