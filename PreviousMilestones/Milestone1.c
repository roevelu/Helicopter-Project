//*****************************************************************************
//
// Milestone1.c - Simple interrupt driven program which samples with AIN0
//
// Author:  Maggie Booker, Luke Roeven, and Anahita Piri
// Last modified:   22/03/2020
//
//*****************************************************************************
// Based on the 'convert' series from 2016
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
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

//*****************************************************************************
//
// Constants
//
//*****************************************************************************
#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 800
#define QUANTISATION12BIT 4069
#define ONEVOLTAGEDROP (QUANTISATION12BIT) / 4;

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
static circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
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
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
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

void 
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);    
                             
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);
  
    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);
  
    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}


void
initDisplay (void)
{
    // intialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
//
// Function to display the Percentage Altitude
//
//*****************************************************************************
void
displayAltitudeVal(int16_t percentAltitude)
{
    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Milestone 1", 0, 0); // Display title

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Altitude: %3d %%", percentAltitude);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}

//*****************************************************************************
//
// Function to display the mean ADC value (10-bit value, note) and sample count.
//
//*****************************************************************************
void
displayMeanVal(uint16_t mean)
{

    char string[17];  // 16 characters across the display

    OLEDStringDraw ("Milestone 1", 0, 0); // Display title

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf (string, sizeof(string), "Mean ADC = %4d", mean);
    // Update line on display.
    OLEDStringDraw (string, 0, 1);
}

//*****************************************************************************
//
// Function to display blank screen
//
//*****************************************************************************
void
displayBlank(void)
{
    OrbitOledClear();
}

//*****************************************************************************
//
// Function to display correct screen based on number of Up button presses
//
//*****************************************************************************
void screenDisplay(uint16_t mean,uint16_t percentAltitude,uint16_t displayMode){
    if (displayMode == 0){
        displayAltitudeVal(percentAltitude);
    } else if (displayMode == 1){
        displayMeanVal(mean);
    } else if (displayMode == 2){
        displayBlank();
    }
}


int
main(void)
{
    //Defining variables
    uint16_t i;
    int8_t percentAltitude;
    int16_t initADCValues = 1;
    int32_t mean;
    int16_t displayMode = 0;
    int16_t displaycounter = 0;
    uint32_t ADCHeliLandedVoltage;
    uint32_t ADCHeliMinVoltage;

    //Initialization mode
    initClock ();
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
        // circular buffer and display it, together with the sample number.
        int32_t sum = 0;
        for (i = 0; i < BUF_SIZE; i++){
            sum = sum + readCircBuf (&g_inBuffer);
        }

        mean = ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE); // Mean calculation code from the lecture


        // Code for initalistation of ADC values
        if(initADCValues && (mean != 0)){
            //The ADC representation of the voltage at the start of the program
            //where the value is not an actual voltage but the 2^12 binary representation of the voltage
            ADCHeliLandedVoltage = mean;
            ADCHeliMinVoltage = ADCHeliLandedVoltage - ONEVOLTAGEDROP;
            initADCValues = 0;
        }

        // Calculate and display the rounded mean of the buffer contents
        percentAltitude = (100 -  ((mean - ADCHeliMinVoltage) * 100) / (ADCHeliLandedVoltage - ADCHeliMinVoltage) );
        if (mean < ADCHeliMinVoltage) {
            percentAltitude = 100;
        }

        //Only update the display every 10 cycles to reduce flickering.
        displaycounter++;
                if (displaycounter >= 10) {
                    screenDisplay(mean,percentAltitude,displayMode);
                }

        // Milestone 1.5
        //Pressing the Left button will reinitialize the altitude the helicopter is at to zero.
            if((checkButton (LEFT) == PUSHED)) {
                        ADCHeliLandedVoltage = mean;    // The ADC representation of the voltage at the start of the program
                                                        // where the value is not an actual voltage but the 2^12 binary representation of the voltage
                        ADCHeliMinVoltage = ADCHeliLandedVoltage - ONEVOLTAGEDROP;
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


        SysCtlDelay (SysCtlClockGet() / 100);  // Update display at ~ 66 Hz
    }
}

