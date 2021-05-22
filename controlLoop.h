// *******************************************************
//
// controlLoop.h
//
// Contains PID control for altitude and yaw.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker
//
// *******************************************************

#ifndef CONTROLLOOP_H_
#define CONTROLLOOP_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// *******************************************************
//
//Globals and constants defined
//
// *******************************************************

#define MAX_INT_CONTROL_MAIN 200
#define MAX_INT_CONTROL_TAIL 200
#define PROPORTIONAL_PWM_ANGLE_RANGE 24

extern double g_pGainAltitude;
extern double g_iGainAltitude;
extern double g_dGainAltitude;
extern double g_pGainYaw;
extern double g_iGainYaw;
extern double g_dGainYaw;
extern double g_errorIntMain;
extern double g_errorIntTail;
extern uint32_t g_controlAltitude;
extern uint32_t g_controlYaw;
extern double g_currentAngle;
extern int16_t g_percentAltitude;
extern int16_t g_setPointAlt;
extern int16_t g_setPointYaw;

// *******************************************************
//
// PID loop for the main motor which controls the altitude
//
// *******************************************************
uint32_t
pidUpdateMain (double setpoint, double alt, double p, double i, double d, double dt);
// *******************************************************

// PID loop for the tail motor which controls the yaw and
// counteracts rotation from the main rotor.
//
// *******************************************************
uint32_t
pidUpdateTail (double setpoint, double yaw, double main_control, double p, double i, double d, double dt);

// *******************************************************
//
// Code for calibrating the main rotor by determining the
// duty cycle needed to just begin to lift
//
// *******************************************************
enum state
calibrateMain(void);

#endif /* CONTROLLOOP_H_ */

