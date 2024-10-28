#include "pid_controller.h"

// Handles the PID Controller model found on the device. Expects a reference to a PIDController struct, target value and actual value.
// Should work as a stand-alone module in other projects.


/* Initialize PID Controller */
void PIDController_Init(PIDController *pid) {

    /* Clear controller variables */
    pid->integrator     = 0.0f;
    pid->prevError      = 0.0f;
    pid->differentiator = 0.0f;
    pid->prevMeasurement= 0.0f;
    pid->out            = 0.0f;
}

/* PID Controller Update Function */
float PIDController_Update(PIDController *pid, float setpoint, float measurement) {

    /* Calculate error signal */
    float error = setpoint - measurement;

    /* === Proportional Term === */
    float proportional = pid->Kp * error;

    /* === Integral Term === */
    /* Update integrator with trapezoidal rule */
    pid->integrator += 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    /* === Derivative Term (on error) === */
    /* Calculate discrete derivative with filtering */
    float derivative = (error - pid->prevError) / pid->T;
    pid->differentiator = (2.0f * pid->tau - pid->T) / (2.0f * pid->tau + pid->T) * pid->differentiator
                        + (2.0f * pid->Kd) / (2.0f * pid->tau + pid->T) * derivative;

    /* === Compute Unbounded Output === */
    float output = proportional + pid->integrator + pid->differentiator;

    /* === Apply Output Limits === */
    float output_limited = output;

    if (output_limited > pid->limMax) {
        output_limited = pid->limMax;
    } else if (output_limited < pid->limMin) {
        output_limited = pid->limMin;
    }

    /* === Anti-Windup via Back-Calculation === */
    /* Calculate difference between limited and unlimited output */
    float delta_u = output_limited - output;

    /* Adjust integrator to account for the saturation */
    pid->integrator += delta_u;

    /* === Store Current Error and Measurement for Next Iteration === */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* === Update Controller Output === */
    pid->out = output_limited;

    /* Return the controller output */
    return pid->out;
}
