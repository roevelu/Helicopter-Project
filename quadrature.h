// *******************************************************
//
// quadrature.h
//
// Quadrature encoding done to determine the yaw, and calibration
//for the yaw is completed on startup.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


#ifndef QUADRATURE_H_
#define QUADRATURE_H_

// *******************************************************
//
//Globals and constants defined for PWM configuration
//
// *******************************************************

#define CHANNEL_A GPIO_PIN_0
#define CHANNEL_B GPIO_PIN_1
#define YAW_REF GPIO_PIN_4
#define NUMBER_TEETH 112
#define HALF_EDGES_PER_ROTATION 224
#define EDGES_PER_ROTATION 448
#define ANGLE_CHANGE_PER_INTERRUPT ((0.8035714286))

extern int32_t g_encoderValue;
extern bool g_yawCalibrationFlag;

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

//*****************************************************************************
//
// Quadrature interrupt handler to set off an interrupt whenever a change in angle
// occurs.
//
//*****************************************************************************
void
quadIntHandler (void);

//*****************************************************************************
//
// Quadrature initialisation
//
//*****************************************************************************
void
initGPIO(void);

//*****************************************************************************
//
// Function to convert g_encoderValue to Angle to display;
// Coded to wrap around so that the yaw is only an output
// between -180 and 180 degrees.
//
//*****************************************************************************
double
convertEncoderToAngle(void);


#endif /* QUADRATURE_H_ */
