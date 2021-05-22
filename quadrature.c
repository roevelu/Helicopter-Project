// *******************************************************
//
// quadrature.c
//
// Quadrature encoding done to determine the yaw, and calibration
//for the yaw is completed on startup.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


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
#include "inc/tm4c123gh6pm.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "pwm.h"
#include "controlLoop.h"
#include "quadrature.h"
#include "flightStates.h"
#include "quadrature.h"


//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

int32_t g_encoderValue = 0;


static bool g_prev_A;
static bool g_current_A;
static bool g_current_B;
bool g_yawCalibrationFlag = false;

//*****************************************************************************
//
// Simplified lookup Table for quadrature encoder
//
//*****************************************************************************
static int8_t quadratureLookup[4] = {
    -1,
    1,
    1,
    -1
};

//*****************************************************************************
//
// Quadrature interrupt handler to set off an interrupt whenever a change in angle
// occurs.
//
//***************************************************************************
void
quadIntHandler (void){

    g_prev_A = g_current_A;
    g_current_A = GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A);
    g_current_B = GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_B);
    g_encoderValue = (g_encoderValue + quadratureLookup[(g_prev_A<<1) | (g_current_B)]) % (EDGES_PER_ROTATION);
    g_currentAngle = convertEncoderToAngle();

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

}

//*****************************************************************************
//
// Calibration Quadrature interrupt handler to set off an interrupt for the yaw
// at the reference orientation - once at the start. The helicopter is set to
// orient itself at that orientation until further instruction.
//
//*****************************************************************************
void
quadIntRefHandler (void){
    g_encoderValue = 0;
    if (g_yawCalibrationFlag == false){
        g_errorIntTail = 0; // global error vars
        g_yawCalibrationFlag = true;
        g_setPointYaw = 0;
    }
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);

}

//*****************************************************************************
//
// Quadrature initialisation
//
//*****************************************************************************
void
initGPIO(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    GPIOIntRegister(GPIO_PORTB_BASE, quadIntHandler); // Set ports int functions for interupts
    GPIOIntRegister(GPIO_PORTC_BASE, quadIntRefHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B); // Set A and B channels,  and Yaw_Ref pin as inputs
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, YAW_REF);

    GPIOPadConfigSet (GPIO_PORTC_BASE, YAW_REF, GPIO_STRENGTH_2MA, // Week pull up for active low pin
           GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTB_BASE, CHANNEL_A | CHANNEL_B, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTC_BASE, YAW_REF, GPIO_BOTH_EDGES);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
    GPIOIntEnable(GPIO_PORTC_BASE, YAW_REF);

    IntPrioritySet(INT_GPIOC, 0x20);
    IntPrioritySet(INT_GPIOB, 0x60);

    g_prev_A = GPIOPinRead(GPIO_PORTB_BASE, CHANNEL_A);

}


//*****************************************************************************
//
// Function to convert g_encoderValue to Angle to display;
// Coded to wrap around so that the yaw is only an output
// between -180 and 180 degrees.
//
//*****************************************************************************
double
convertEncoderToAngle(){
    double g_currentAngle = (g_encoderValue * ANGLE_CHANGE_PER_INTERRUPT);
    if(g_currentAngle < -180){
        g_currentAngle += 360;
    } else if(g_currentAngle > 180){
        g_currentAngle -= 360;
    }
    return(g_currentAngle);
}
