// *******************************************************
//
// Final.c
//
// Main file for the ENCE361 Helicopter project.
// The code controls a toy which is constrained to
// only yaw and move up and down. The code is designed
// for the TM4C123G Tiva microcontroller and uses:
// - ADC converters, and quadrature encoders for sensors
// - PWM outputs for the controller (two DC motors)
// - A timer triggered scheduler
// - Interrupt based foreground tasks
// - An OLED display
// - UART for communication to the terminal
// Enjoy!
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************

#include <buttons5.h>
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "circBufT.h"
#include "OrbitOLED/OrbitOLEDInterface.h"
#include "OrbitOLED/lib_OrbitOled/OrbitOled.h"
#include "display.h"
#include "altitude.h"
#include "quadrature.h"
#include "pwm.h"
#include "uart.h"
#include "controlLoop.h"
#include "switches.h"
#include "flightStates.h"

//*****************************************************************************
//
// Constants
//
//*****************************************************************************
#define SYSTICK_RATE_HZ 800
#define BUF_SIZE 20
#define SAMPLE_RATE_HZ 800
#define QUANTISATION12BIT 4095 // 2 ^ 12 - 1
#define ONEVOLTAGEDROP (QUANTISATION12BIT) / 4 * 1.2 // Change in number of bits for one volt
#define CALIBRATION_TICK_RATE_HZ 4
#define N_TASKS 7
#define ALT_STEP 10
#define YAW_STEP 15
#define INIT_ADC_BUFFER_WAIT 500
#define MAX_PERCENT_ALT 100
#define ACCEPTABLE_LANDED_YAW_ERROR 3
#define ACCEPTABLE_LANDED_ALT_ERROR 5
#define ACCEPTABLE_LANDING_YAW_ERROR 1
#define YAW_CALIBRATION_TAIL_PWM 50

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

static uint32_t g_ulSampCnt;    // Counter for the interrupts
static uint8_t g_taskCount = 0; // Counter for scheduler

// Structure to hold the scheduled tasks
static struct scheduled_task {
    uint8_t delay;
    uint8_t ready;
    uint8_t period;
};

// Enumerate type for iterating in the SysTickIntHandler
static enum tasks{uart = 0, disp, adc,pwm,buttons,switches,calibration};

// Instance of the scheduled task structure with correct size for the number of tasks
static struct scheduled_task scheduledTasks[N_TASKS];

static char* currentStateCharArray[] = {"Calibrating ADC","Waiting for Switch","Calibrating Altitude","Calibrating Yaw","Landing","Landed","Flying"};

//*****************************************************************************
//
// The interrupt handler for the for SysTick interrupt
//
//*****************************************************************************
void
SysTickIntHandler(void)
{
    // Loop through tasks and decrement the delay struct attribute for each task --- set to run if reached 0
    for (g_taskCount = 0; g_taskCount < N_TASKS; g_taskCount++) {
        if (scheduledTasks[g_taskCount].delay == 0) {
            scheduledTasks[g_taskCount].ready = true;
            scheduledTasks[g_taskCount].delay = scheduledTasks[g_taskCount].period; // Reset task
        } else {
            scheduledTasks[g_taskCount].delay -= 1;
        }
    }


    // Initiate a conversion
    ADCProcessorTrigger(ADC0_BASE, 3);
    g_ulSampCnt++;

    //Control calculations which are inside SysTick for constant dt
    if(currentState != CALIBRATE_ADC){
        g_controlAltitude = pidUpdateMain(g_setPointAlt, g_percentAltitude,g_pGainAltitude,g_iGainAltitude,g_dGainAltitude,(double) 1/SYSTICK_RATE_HZ);
        g_controlYaw = pidUpdateTail(g_setPointYaw, g_currentAngle, g_controlAltitude, g_pGainYaw,g_iGainYaw,g_dGainYaw,(double) 1/SYSTICK_RATE_HZ);
    }

    // Check to see if calibration is complete and set to next mode if true
    if ((g_yawCalibrationFlag == true) && (currentState == CALIBRATE_YAW)){
        currentState = FLYING;
    }


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

    // Set up the period for the SysTick timer
    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_RATE_HZ);

    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
//
// Initialisation tasks for timer triggered scheduler.
// Delay and ready are set to 0 so all operations occur in the first loop
// Period denotes number of SYS TICK ints before the task is ready again
//
//*****************************************************************************
void
initTasks (void)
{

    scheduledTasks[uart].delay = 0;
    scheduledTasks[uart].ready = false;
    scheduledTasks[uart].period = SYSTICK_RATE_HZ / UART_TICK_RATE_HZ;

    scheduledTasks[disp].delay = 0;
    scheduledTasks[disp].ready = false;
    scheduledTasks[disp].period = SYSTICK_RATE_HZ / DISP_TICK_RATE_HZ;

    scheduledTasks[adc].delay = 0;
    scheduledTasks[adc].ready = true;
    scheduledTasks[adc].period = SYSTICK_RATE_HZ / ALT_TICK_RATE_HZ;

    scheduledTasks[pwm].delay = 0;
    scheduledTasks[pwm].ready = false;
    scheduledTasks[pwm].period = SYSTICK_RATE_HZ / ALT_TICK_RATE_HZ;

    scheduledTasks[buttons].delay = 0;
    scheduledTasks[buttons].ready = false;
    scheduledTasks[buttons].period = SYSTICK_RATE_HZ / BUTTON_TICK_RATE_HZ;

    scheduledTasks[switches].delay = 0;
    scheduledTasks[switches].ready = false;
    scheduledTasks[switches].period = SYSTICK_RATE_HZ / SWITCH_TICK_RATE_HZ;

    scheduledTasks[calibration].delay = 0;
    scheduledTasks[calibration].ready = false;
    scheduledTasks[calibration].period = SYSTICK_RATE_HZ / CALIBRATION_TICK_RATE_HZ;

}


int
main(void)

{
    //Defining variables
    uint16_t i; // iterator for adc
    int32_t mean = 0;
    int32_t ADCHeliLandedVoltage;
    int32_t ADCHeliMinVoltage;
    uint32_t ui32MainFreq = PWM_START_RATE_HZ; // Setting start Freq for PWM gen
    uint32_t ui32TailFreq = PWM_START_RATE_HZ;

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_GPIO);
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralReset (UP_BUT_PERIPH);
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);
    SysCtlPeripheralReset (SW1_PERIPH);

    //Initialization
    initTasks();
    initClock ();
    initGPIO();
    initADC ();
    initialisePWM ();
    initDisplay ();
    initialiseUSB_UART ();
    initCircBuf (&g_inBuffer, BUF_SIZE);
    initButtons ();
    initSwitch ();

    // Initialisation is complete, so turn on the output.
    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, true);
    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, true);

    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {
        // Switches task
        if (scheduledTasks[switches].ready){
            scheduledTasks[switches].ready = false;
            updateSwitch();
            const uint8_t switchState = checkSwitch();
            switch (currentState) {
                case FLYING:
                    if (switchState == SWITCH_DOWN) {
                        currentState = LANDING;
                        g_errorIntMain = 0;
                        g_errorIntTail = 0;
                    }
                    break;
                case LANDED:
                    if (switchState == SWITCH_UP) {
                        currentState = FLYING;
                        g_errorIntMain = 0;
                        g_errorIntTail = 0;
                    }
                    break;
                case WAITING_ON_SWITCH:
                    if (switchState == SWITCH_UP) {
                        currentState = CALIBRATE_ALT;
                    }
            }
        }

        // Button task
        if ((scheduledTasks[buttons].ready && (currentState != CALIBRATE_ADC) && (currentState == FLYING) && (currentState != CALIBRATE_YAW) && (currentState != CALIBRATE_ALT))){
           scheduledTasks[buttons].ready = false;

           // Poll the buttons
           updateButtons ();

           if((checkButton (UP) == PUSHED) && (g_setPointAlt < 100)) {
               g_setPointAlt += ALT_STEP;
               g_errorIntMain = 0;
           }
           if((checkButton (DOWN) == PUSHED) && (g_setPointAlt > 0)) {
               g_setPointAlt -= ALT_STEP;
               g_errorIntMain = 0;
           }
           if((checkButton (LEFT) == PUSHED)) {
               g_setPointYaw -= YAW_STEP;
               if (g_setPointYaw <= -180) {
                   g_setPointYaw += 360;
               }
           }
           if((checkButton (RIGHT) == PUSHED)) {
               g_setPointYaw += YAW_STEP;
               if (g_setPointYaw > 180) {
                   g_setPointYaw -= 360;
               }
           }
       }

        // Pwm task
        if ((scheduledTasks[pwm].ready)){
            scheduledTasks[pwm].ready = false;
            switch (currentState) {
                case CALIBRATE_ALT:
                    setMainPWM (ui32MainFreq, g_controlAltitude);
                    setTailPWM (ui32TailFreq, g_controlYaw);
                    break;
                case CALIBRATE_YAW:
                    setMainPWM (ui32MainFreq, g_controlAltitude);
                    setTailPWM (ui32TailFreq, YAW_CALIBRATION_TAIL_PWM);
                    break;
                case FLYING:
                    setMainPWM (ui32MainFreq, g_controlAltitude);
                    setTailPWM (ui32TailFreq, g_controlYaw);
                    break;
                case LANDING:
                    setMainPWM (ui32MainFreq, g_controlAltitude);
                    setTailPWM (ui32TailFreq, g_controlYaw);
                    g_setPointYaw = 0; // Setpoint for Yaw
                    if ((g_currentAngle > -ACCEPTABLE_LANDING_YAW_ERROR) && (g_currentAngle < ACCEPTABLE_LANDING_YAW_ERROR)) {
                        g_setPointAlt -= 1;
                        if (g_setPointAlt <= 0) {
                            g_setPointAlt = 0;
                        }
                    } if ((g_currentAngle >= -ACCEPTABLE_LANDED_YAW_ERROR) && (g_currentAngle <= ACCEPTABLE_LANDED_YAW_ERROR) && (g_percentAltitude <= ACCEPTABLE_LANDED_ALT_ERROR)) {
                        currentState = LANDED;
                    }
                    break;
                case LANDED:
                    setMainPWM (ui32MainFreq, 0);
                    setTailPWM (ui32TailFreq, 0);
                    g_setPointAlt = 0;
                    break;
            }
        }

        // Adc task
        // Code obtained from ADCDemo.c from Lab 3
        // Background task: calculate the (approximate) mean of the values in the
        // circular buffer and displays it, together with the sample number.
        if ((scheduledTasks[adc].ready)){
               scheduledTasks[adc].ready = false;
               int32_t sum = 0;
               for (i = 0; i < BUF_SIZE; i++){
                   sum = sum + readCircBuf (&g_inBuffer);
               }

               mean = ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE); // Mean calculation code from the lecture

               if((currentState == CALIBRATE_ADC)){

                   if (((mean != 0) && (g_ulSampCnt > (INIT_ADC_BUFFER_WAIT)))) {
                       ADCHeliLandedVoltage = mean;
                       ADCHeliMinVoltage = ADCHeliLandedVoltage - ONEVOLTAGEDROP;
                       currentState = WAITING_ON_SWITCH;
                   }
               }

               // Calculate the percentage altitude accounting for int division
               g_percentAltitude = (MAX_PERCENT_ALT -  ((mean - ADCHeliMinVoltage) * MAX_PERCENT_ALT) / (ADCHeliLandedVoltage - ADCHeliMinVoltage) );

        }

        // Uart task
        if ((scheduledTasks[uart].ready && (currentState != CALIBRATE_ADC))){
            scheduledTasks[uart].ready = false;
            usprintf (g_statusStr,
                    "\r\n"
                    "|YAW: S=%2d A=%2d "
                    "|ALT: S=%2d A=%2d "
                    "|PWM: M=%2d T=%2d "
                    "|Mode: %s \r\n"
                    "\r\n",
                    (int) g_setPointYaw, (int) g_currentAngle,
                    (int)g_setPointAlt, (int) g_percentAltitude,
                    (int) g_dispMainPWM, (int) g_dispTailPWM,
                    currentStateCharArray[currentState]);
            UARTSend (g_statusStr);
        }

        // Calibration task
        if ((scheduledTasks[calibration].ready)) {
            scheduledTasks[calibration].ready = false;
            if ((currentState == CALIBRATE_ALT)){
                currentState = calibrateMain();
            }
        }

        // Display task
        if ((scheduledTasks[disp].ready) && (currentState != CALIBRATE_ADC)){
            scheduledTasks[disp].ready = false;
            screenDisplay(g_percentAltitude, g_currentAngle, g_dispMainPWM, g_dispTailPWM);
        }
    }
}
