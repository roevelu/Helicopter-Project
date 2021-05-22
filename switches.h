// *******************************************************
//
// switches.h
//
// Setup of the switch on the board to control flight and
// landing mode.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker
//
// *******************************************************


#ifndef SWITCHES_H_
#define SWITCHES_H_


#include <stdint.h>
#include <stdbool.h>

// *******************************************************
//
// Defining the switch states, polling rate, and the defining
// the GPIO for the switch.
//
// ******************************************************
#define SWITCH_TICK_RATE_HZ 30 // Poll display at 0.01 Hz
enum switchStates {SWITCH_DOWN = 0, SWITCH_UP, UNCHANGED};
// SW1 button
#define SW1_PERIPH  SYSCTL_PERIPH_GPIOA
#define SW1_PORT_BASE  GPIO_PORTA_BASE
#define SW1_PIN  GPIO_PIN_7

// *******************************************************
//
// Initialisation of the switch GPIO is done.
//
// *******************************************************
void
initSwitch (void);

// *******************************************************
//
// Update of the switch peripheral to see if its state
// has changed.
//
// ******************************************************
void
updateSwitch (void);

// *******************************************************
//
// Checking of the switch to determine its new state, or
// if it is unchanged.
//
// *******************************************************
uint8_t
checkSwitch(void);

#endif /* SWITCHES_H_ */
