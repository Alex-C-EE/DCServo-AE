#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/* PID Controller Structure */
typedef struct {

    /* Controller gains */
    float Kp;      /* Proportional gain */
    float Ki;      /* Integral gain */
    float Kd;      /* Derivative gain */

    /* Derivative low-pass filter time constant */
    float tau;     /* Time constant for derivative filter */

    /* Output limits */
    float limMin;  /* Minimum output limit (negative for reverse rotation) */
    float limMax;  /* Maximum output limit */

    /* Sample time (in seconds) */
    float T;       /* Sampling time */

    /* Controller memory */
    float integrator;       /* Integrator term */
    float prevError;        /* Previous error (for derivative calculation) */
    float differentiator;   /* Differentiator term */
    float prevMeasurement;  /* Previous measurement (if needed) */

    /* Controller output */
    float out;     /* PID controller output */

} PIDController;

/* Function prototypes */
void  PIDController_Init(PIDController *pid);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif /* PID_CONTROLLER_H */
