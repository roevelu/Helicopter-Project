// *******************************************************
//
// switches.c
//
// Setup of the switch on the board to control flight and
// landing mode.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "inc/tm4c123gh6pm.h"  // Board specific defines (for PF0)
#include "switches.h"

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

bool g_switch_state;
bool g_newg_switch_state;
bool g_switch_stateChanged = false;

// *******************************************************
//
// Initialisation of the switch GPIO
//
// *******************************************************
void initSwitch() {
    SysCtlPeripheralEnable(SW1_PERIPH);
    GPIOPinTypeGPIOInput(SW1_PORT_BASE, SW1_PIN);
    GPIOPadConfigSet(SW1_PORT_BASE, SW1_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);
    g_switch_state = GPIOPinRead(SW1_PORT_BASE, SW1_PIN)
                      == SW1_PIN;
}

// *******************************************************
//
// Update of the switch peripheral to see if its state
// has changed.
//
// *******************************************************
void updateSwitch(void) {
     g_newg_switch_state = GPIOPinRead(SW1_PORT_BASE, SW1_PIN)
                              == SW1_PIN;
    if (g_newg_switch_state != g_switch_state) {
        g_switch_stateChanged = true;
    }
    g_switch_state = g_newg_switch_state;
}

// *******************************************************
//
// Checking of the switch to determine its new state, or
// if it is unchanged.
//
// *******************************************************
uint8_t checkSwitch() {
    if (g_switch_stateChanged) {
        g_switch_stateChanged = false;
        if (g_switch_state) {
            return SWITCH_UP;
        } else {
            return SWITCH_DOWN;
        }
    } else {
        return UNCHANGED;
    }
}
