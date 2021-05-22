// *******************************************************
//
// controlLoop.c
//
// Contains PID control for altitude and yaw.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "pwm.h"
#include "flightStates.h"
#include "controlLoop.h"

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

double g_pGainAltitude = 1.2; // Alt gains
double g_iGainAltitude = 0.2;
double g_dGainAltitude = 0.4;
double g_pGainYaw = 4; // Yaw gains
double g_iGainYaw = 0.3;
double g_dGainYaw = 0.4;
double g_errorIntMain = 0; // Errors
double g_errorIntTail = 0;
double errorPrevMain = 0;
double errorPrevTail = 0;
uint32_t g_controlAltitude = 0; // Control efforts
uint32_t g_controlYaw = 0;
double g_currentAngle = 0; // Current Values
int16_t g_percentAltitude;
int16_t g_setPointAlt = 0;
int16_t g_setPointYaw = 0;
int16_t g_baseLinePwmMain = 10; // Baseline PWM at initalistation
int16_t g_baseLinePwmTail = 5;


// *******************************************************
//
// PID loop for the main motor which controls the altitude
//
// *******************************************************
uint32_t pidUpdateMain (double setpoint, double alt, double p, double i, double d, double dt){
    double error = setpoint - alt;
    double error_derivative = (error - errorPrevMain) / dt;
    uint32_t control; // controller response

    g_errorIntMain += error * dt;

    // Setting limits on the integral error
    if(g_errorIntMain > MAX_INT_CONTROL_MAIN){
        g_errorIntMain = MAX_INT_CONTROL_MAIN;
    }

    if(g_errorIntMain < -MAX_INT_CONTROL_MAIN){
        g_errorIntMain = -MAX_INT_CONTROL_MAIN;
    }

    // Calculate control
    control = error * p + g_errorIntMain * i + g_baseLinePwmMain + error_derivative * d;

    // Cap control response
    if(control <= PWM_MIN_DUTY){
        control = PWM_MIN_DUTY;
    }

    if(control >= PWM_MAX_DUTY_MAIN){
        control = PWM_MAX_DUTY_MAIN;
    }

    errorPrevMain = error;
    return control;
}

// *******************************************************
//
// PID loop for the tail motor which controls the yaw and
// counteracts rotation from the main rotor.
//
// *******************************************************
uint32_t pidUpdateTail (double setpoint, double yaw, double main_control, double p, double i, double d, double dt){
    double error = 0;

    // Calculate error for yaw considering number space is -180 to 180
    // The series of if statements ensures that the shortest distance
    // is always used for the control response
    if ((360 + yaw - setpoint) < (-yaw + setpoint)){
        error = -(p*PROPORTIONAL_PWM_ANGLE_RANGE);
    } else if ((setpoint < 0) && ((setpoint - (PROPORTIONAL_PWM_ANGLE_RANGE*p)) <= -180) && ((yaw + (PROPORTIONAL_PWM_ANGLE_RANGE*p)) > 180)) {
        error = (360 - yaw + setpoint);
    } else if (yaw < (setpoint - 180)) {
        yaw = 180 - setpoint;
        error = setpoint - yaw;
    } else if (yaw > (setpoint + 180)) {
        yaw = -180 + setpoint;
        error = setpoint - yaw;
    } else {
        error = setpoint - yaw;
    }

    double error_derivative = (error - errorPrevTail) / dt;

    uint32_t control;

    g_errorIntTail += error * dt;

    // Setting limits on the integral error
    if(g_errorIntTail > MAX_INT_CONTROL_TAIL){
        g_errorIntTail = MAX_INT_CONTROL_TAIL;
    }

    if(g_errorIntTail < -MAX_INT_CONTROL_TAIL){
        g_errorIntTail = -MAX_INT_CONTROL_TAIL;
    }

    // Calculate control
    control = error * p + g_errorIntTail * i + error_derivative * d + g_baseLinePwmTail;

    // Cap control response
    if(control <= PWM_MIN_DUTY){
        control = PWM_MIN_DUTY;
    }

    if(control >= PWM_MAX_DUTY_TAIL){
            control = PWM_MAX_DUTY_TAIL;
    }

    errorPrevTail = error;
    return control;
}

// *******************************************************
//
// Code for calibrating the main rotor by determining the
// duty cycle needed to just begin to lift
//
// *******************************************************
enum state calibrateMain(void){
    if(g_percentAltitude <= 1){
        g_baseLinePwmMain += 1;
        g_errorIntMain = 0;
        g_baseLinePwmTail = g_controlYaw;
        return CALIBRATE_ALT;
    } else {
        return CALIBRATE_YAW;
    }
}



