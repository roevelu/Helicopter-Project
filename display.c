// *******************************************************
//
// display.c
//
// Supports the OrbitOLED display. It shows Altitude in %, Angle
// in degrees, and the main and rear PWM duty cycles
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "quadrature.h"
#include "controlLoop.h"


//*****************************************************************************
//
// Function to initialise the Orbit OLED display
//
//*****************************************************************************
void
initDisplay (void)
{
    OLEDInitialise ();
}

//*****************************************************************************
//
// Function to display the PWM duty cycle for the main and tail rotor, the percentage altitude
// and the current yaw angle.
//
//*****************************************************************************
void screenDisplay(uint16_t g_percentAltitude, double g_currentAngle, uint8_t g_dispMainPWM, uint8_t g_dispTailPWM){
    char string[17];  // 16 characters across the display


    usnprintf (string, sizeof(string), "PWM: M %3d", (int) g_dispMainPWM);
    OLEDStringDraw (string, 0, 0);

    usnprintf (string, sizeof(string), "PWM: T %3d", (int) g_dispTailPWM);
    OLEDStringDraw (string, 0, 1);

    usnprintf (string, sizeof(string), "Altitude: %3d%%", (int) g_percentAltitude);
    OLEDStringDraw (string, 0, 2);

    usnprintf (string, sizeof(string), "Angle = %4d", (int) g_currentAngle);
    OLEDStringDraw (string, 0, 3);

}
