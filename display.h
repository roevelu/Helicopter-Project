// *******************************************************
//
// display.h
//
// Supports the OrbitOLED display. It shows Altitude in %, Angle
// in degrees, and the main and rear PWM duty cycles
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************

#ifndef DISPLAY_H_
#define DISPLAY_H_


#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// Constants
//
//*****************************************************************************
#define DISP_TICK_RATE_HZ 2 // poll display at 2 HZ


//*****************************************************************************
//
// Function to initialise the display of the screen
//
//*****************************************************************************
void
initDisplay (void);

//*****************************************************************************
//
// Function to display the Percentage Altitude, Yaw, and PWM motor values (which
// are global variables and do not need to be passed in)
//
//*****************************************************************************
void
screenDisplay(uint16_t g_percentAltitude, double g_currentAngle, uint8_t dispMainPWM, uint8_t dispTailPWM);

#endif /*DISPLAY_H_*/
