/* motor_control.h */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} Motor_Direction_t;

void Motor_Init(void);
void Motor_SetSpeed(double pidOutput);
void Motor_Stop(void);
float Motor_GetCurrent(void);

#endif /* MOTOR_CONTROL_H */
