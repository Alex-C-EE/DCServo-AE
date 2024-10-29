#include "pid_controller.h"
#include <math.h>

/* Initialize PID Controller */
void PIDController_Init(PIDController *pid, const Config_t *config) {
    if (pid == NULL || config == NULL) return;

    /* Clear controller variables */
    pid->integrator = 0.0;
    pid->prevError = 0.0;
    pid->differentiator = 0.0;
    pid->prevMeasurement = 0.0;
    pid->out = 0.0;

    /* Set initial gains from config */
    pid->Kp = config->kp;
    pid->Ki = config->ki;
    pid->Kd = config->kd;

    /* Set output limits based on config maxSpeed */
    pid->limMin = -((double)config->maxSpeed);
    pid->limMax = (double)config->maxSpeed;

    /* Initialize other parameters from config */
    pid->tau = config->tau;       /* Set tau from config (3.a) */
    pid->Kaw = config->Kaw;       /* Set back-calculation gain from config (3.d) */
    pid->T = 0.001;                /* 1ms sample time - adjust based on your system */

    /* Optionally, initialize integrator and differentiator if needed */
}

/* Update controller gains and limits from config */
void PIDController_UpdateGains(PIDController *pid, const Config_t *config) {
    if (pid == NULL || config == NULL) return;

    pid->Kp = config->kp;
    pid->Ki = config->ki;
    pid->Kd = config->kd;
    pid->limMin = -((double)config->maxSpeed);
    pid->limMax = (double)config->maxSpeed;

    /* Update tau and Kaw if they can change dynamically */
    pid->tau = config->tau;
    pid->Kaw = config->Kaw;
}

/* PID Controller Update Function */
double PIDController_Update(PIDController *pid, const Config_t *config) {
    /* Validate inputs */
    if (pid == NULL || config == NULL) {
        return 0.0;
    }

    /* Update gains in case they've changed */
    PIDController_UpdateGains(pid, config);

    /* Calculate error signal */
    double error = config->targetAngle - config->currentAngle;

    /* Apply deadband using maxErrorThreshold from config */
    if (fabs(error) < (double)config->maxErrorThreshold) {
        error = 0.0;
    }

    /* === Proportional Term === */
    double proportional = pid->Kp * error;

    /* === Integral Term === */
    if (pid->Ki != 0.0) {
        /* Trapezoidal integration */
        double newIntegrator = pid->integrator +
            0.5 * pid->Ki * pid->T * (error + pid->prevError);

        pid->integrator = newIntegrator;

        /* === Back-Calculation Anti-Windup === (3.d) */
        /* Calculate the unclamped output */
        double unclampedOutput = proportional + pid->integrator + pid->differentiator;

        /* Apply output limits */
        double limitedOutput = unclampedOutput;
        if (limitedOutput > pid->limMax) {
            limitedOutput = pid->limMax;
        } else if (limitedOutput < pid->limMin) {
            limitedOutput = pid->limMin;
        }

        /* Back-calculation: Adjust integrator to account for output limiting */
        double integratorAdjustment = pid->Kaw * (limitedOutput - unclampedOutput) * pid->T;
        pid->integrator += integratorAdjustment;

        /* Clamp integrator to prevent excessive buildup */
        if (pid->integrator > pid->limMax) {
            pid->integrator = pid->limMax;
        } else if (pid->integrator < pid->limMin) {
            pid->integrator = pid->limMin;
        }
    }

    /* === Derivative Term === */
    if (pid->Kd != 0.0) {
        /* Derivative on measurement to prevent derivative kick */
        double derivative = -(config->currentAngle - pid->prevMeasurement) / pid->T;

        /* Low-pass filter the derivative term */
        double alpha = (2.0 * pid->tau - pid->T) / (2.0 * pid->tau + pid->T);
        double beta = (2.0 * pid->Kd) / (2.0 * pid->tau + pid->T);
        pid->differentiator = alpha * pid->differentiator + beta * derivative;
    }

    /* === Compute Output === */
    double output = proportional + pid->integrator + pid->differentiator;

    /* === Apply Output Limits === */
    if (output > pid->limMax) {
        output = pid->limMax;
    } else if (output < pid->limMin) {
        output = pid->limMin;
    }

    /* === Detect Significant Disturbances and Reset if Necessary === (3.f) */
    if (fabs(error) > config->disturbanceThreshold) {
        PIDController_Reset(pid);
    }

    /* === Store Error and Measurement for Next Iteration === */
    pid->prevError = error;
    pid->prevMeasurement = config->currentAngle;
    pid->out = output;

    return output;
}

/* Reset the controller's internal state */
void PIDController_Reset(PIDController *pid) {
    if (pid == NULL) return;

    pid->integrator = 0.0;
    pid->prevError = 0.0;
    pid->differentiator = 0.0;
    pid->prevMeasurement = 0.0;
    pid->out = 0.0;
}
