// *******************************************************
//
// flightStates.h
//
// Defines the states with the FSM
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************

#ifndef FLIGHTSTATES_H_
#define FLIGHTSTATES_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// *******************************************************
//
// Globals and constants defined
//
// *******************************************************


// Enum for cleaner readable FSM state code
enum state {
    CALIBRATE_ADC = 0,
    WAITING_ON_SWITCH,
    CALIBRATE_ALT,
    CALIBRATE_YAW,
    LANDING,
    LANDED,
    FLYING,
};

// Set initial state of the helicopter
static enum
state currentState = CALIBRATE_ADC;

#endif /* FLIGHTSTATES_H_ */
