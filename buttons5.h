// *******************************************************
//
// buttons5.h
//
// Support for a set of FIVE specific buttons on the Tiva/Orbit.
// The buttons are:  UP and DOWN (on the Orbit daughterboard) plus
// LEFT and RIGHT on the Tiva, and a virtual RESET button.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// Based on P.J. Bones UCECE code
// 
// *******************************************************


#ifndef BUTTONS_H_
#define BUTTONS_H_


#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
// Constants
//
//*****************************************************************************

#define BUTTON_TICK_RATE_HZ 30

enum butNames {UP = 0, DOWN, LEFT, RIGHT, RESET,NUM_BUTS};
enum butStates {RELEASED = 0, PUSHED, NO_CHANGE};
// UP button
#define UP_BUT_PERIPH  SYSCTL_PERIPH_GPIOE
#define UP_BUT_PORT_BASE  GPIO_PORTE_BASE
#define UP_BUT_PIN  GPIO_PIN_0
#define UP_BUT_NORMAL  false
// DOWN button
#define DOWN_BUT_PERIPH  SYSCTL_PERIPH_GPIOD
#define DOWN_BUT_PORT_BASE  GPIO_PORTD_BASE
#define DOWN_BUT_PIN  GPIO_PIN_2
#define DOWN_BUT_NORMAL  false
// LEFT button
#define LEFT_BUT_PERIPH  SYSCTL_PERIPH_GPIOF
#define LEFT_BUT_PORT_BASE  GPIO_PORTF_BASE
#define LEFT_BUT_PIN  GPIO_PIN_4
#define LEFT_BUT_NORMAL  true
// RIGHT button
#define RIGHT_BUT_PERIPH  SYSCTL_PERIPH_GPIOF
#define RIGHT_BUT_PORT_BASE  GPIO_PORTF_BASE
#define RIGHT_BUT_PIN  GPIO_PIN_0
#define RIGHT_BUT_NORMAL  true
// RESET button
#define RESET_BUT_PERIPH  SYSCTL_PERIPH_GPIOA
#define RESET_BUT_PORT_BASE  GPIO_PORTA_BASE
#define RESET_BUT_PIN  GPIO_PIN_6
#define RESET_BUT_NORMAL  true

#define NUM_BUT_POLLS 3 // NUM_BUT_POLLS set according to the polling rate.

// *******************************************************
//
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants above.
//
// *******************************************************
void
initButtons (void);

// *******************************************************
//
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
//
// *******************************************************
void
updateButtons (void);

// *******************************************************
//
// checkButton: Function returns the new button state if the button state
// (PUSHED or RELEASED) has changed since the last call, otherwise returns
// NO_CHANGE.  The argument butName should be one of constants in the
// enumeration butStates, excluding 'NUM_BUTS'. Safe under interrupt.
//
// *******************************************************
uint8_t
checkButton (uint8_t butName);

#endif /*BUTTONS_H_*/
