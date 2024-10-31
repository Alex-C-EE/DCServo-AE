/* encoder.h */
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "config.h"

/**
 * @brief Initializes the encoder hardware and related settings.
 */
void Encoder_Init(void);

/**
 * @brief Updates the current angle based on encoder readings.
 */
void Update_Encoder_Angle(void);

/**
 * @brief Resets the encoder angle to zero.
 */
void Encoder_SetZero(void);

/**
 * @brief Checks the presence and strength of the magnetic field.
 */
void CheckMagneticField(void);

/**
 * @brief Calculates the current velocity based on encoder data.
 *
 * @return The calculated velocity as a double.
 */
double Encoder_CalculateVelocity(void);  // New function

#endif /* ENCODER_H */
