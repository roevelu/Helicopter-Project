//*****************************************************************************
//
// File: altitude.h
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// Measures the altitude by taking regular ADC samples and averages
// them in in a circular buffer.
//
//*****************************************************************************


#ifndef ALTITUDE_H_
#define ALTITUDE_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "circBufT.h"

// *******************************************************
//
// Globals and constants to include
//
// *******************************************************
#define ALT_TICK_RATE_HZ 300

extern circBuf_t g_inBuffer; // Buffer of size BUF_SIZE integers (sample values)

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void);

//*****************************************************************************
//
// Initialisation
//
//*****************************************************************************
void
initADC (void);



#endif /* ALTITUDE_H_ */
