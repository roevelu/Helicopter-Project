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

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "circBufT.h"


//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
circBuf_t g_inBuffer; // Buffer of size BUF_SIZE integers (sample values)

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Gets a value from the ADC, places it in a circular buffer, and
// then clears the interupt
//
//*****************************************************************************

void ADCIntHandler(void)
{
    uint32_t ulValue;

    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    writeCircBuf(&g_inBuffer, ulValue);
    ADCIntClear(ADC0_BASE, 3);
}

//*****************************************************************************
//
// Initialisation of ADC to calibrate height
// from the alititude port.
// Interrupt configured from

//
//*****************************************************************************
void initADC(void)
{
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);


    // Configure ADC step. Interrupt flag set when the sample is done.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
    ADC_CTL_END);


    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntRegister(ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);

}

