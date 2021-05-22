//*****************************************************************************
//
// Milestone2.c - 
//
// Author:  Maggie Booker, Luke Roeven, and Anahita Piri
// Last modified:   26/03/2020
//
//*****************************************************************************
// Based on the 'convert' series from 2016
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
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "buttons4.h"
#include "display.h"
#include "altitude.h"
#include "quadrature.h"

//*****************************************************************************
//
// Constants
//
//*****************************************************************************
#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 800
#define QUANTISATION12BIT 4069
#define ONEVOLTAGEDROP (QUANTISATION12BIT) / 4

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
static uint32_t g_ulSampCnt;    // Counter for the interrupts


//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt.
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Initiate a conversion
    //
    ADCProcessorTrigger(ADC0_BASE, 3); 
    g_ulSampCnt++;
}

//*****************************************************************************
//
// Initialisation functions for the clock (incl. SysTick), ADC, display
//
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / SAMPLE_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

int
main(void)
{
    //Defining variables
    uint16_t i;
    int16_t percentAltitude;
    int16_t initADCValues = 1;
    int32_t mean = 0;
    int16_t displayMode = 0;
    int16_t displaycounter = 0;
    int32_t ADCHeliLandedVoltage;
    int32_t ADCHeliMinVoltage;
    double currentAngle = 0;

    //Initialization mode
    initClock ();
    initGPIO();
    initADC ();
    initDisplay ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initButtons ();

    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {

        // Poll the buttons
        updateButtons ();

        //Code obtained from ADCDemo.c from Lab 3
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and displays it, together with the sample number.

        int32_t sum = 0;
        for (i = 0; i < BUF_SIZE; i++){
            sum = sum + readCircBuf (&g_inBuffer);
        }


        mean = ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE); // Mean calculation code from the lecture


        // Code for initalistation of ADC values
        if(initADCValues && ((mean != 0) && (g_ulSampCnt > (500)))){
            //The ADC representation of the voltage at the start of the program
            //where the value is not an actual voltage but the 2^12 binary representation of the voltage
            ADCHeliLandedVoltage = mean;
            ADCHeliMinVoltage = ADCHeliLandedVoltage - ONEVOLTAGEDROP;
            initADCValues = 0;
        }

        // Calculate and display the rounded mean of the buffer contents
        percentAltitude = (100 -  ((mean - ADCHeliMinVoltage) * 100) / (ADCHeliLandedVoltage - ADCHeliMinVoltage) );

        //Only update the display every 10 cycles to reduce flickering.
        displaycounter++;

        if ((displaycounter >= 10) && (!initADCValues)) {
            currentAngle = convertEncoderToAngle();
            screenDisplay(mean,percentAltitude,displayMode, currentAngle);
        }

        // Milestone 1.5
        //Pressing the Left button will reinitialize the altitude the helicopter is at to zero.
        if((checkButton (LEFT) == PUSHED)) {
                    ADCHeliLandedVoltage = mean;    // The ADC representation of the voltage at the start of the program
                                                    // where the value is not an actual voltage but the 2^12 binary representation of the voltage
                    ADCHeliMinVoltage = ADCHeliLandedVoltage - ONEVOLTAGEDROP;
                    encoderValue = 0;
                }

         // Milestone 1.6
         //Operating the Up button will change the display of the altitude from percentage to mean ADC value
         //A second push turns display blank
         //And thereafter any pushes go through the same 3 states
        if ((checkButton (UP) == PUSHED)) {
                    displayMode ++;
                    if (displayMode > 2){
                        displayMode = 0;
                    }
                }

        //SysCtlDelay (SysCtlClockGet() / 100);  // Update display at ~ 66 Hz
    }
}

