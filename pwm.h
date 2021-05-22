// *******************************************************
//
// pwm.h
//
// Sets the PWM duty cycles to power the motors
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker
//
// *******************************************************


#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"

// *******************************************************
//
// Globals and constants defined for PWM configuration
//
// *******************************************************

#define PWM_START_RATE_HZ  200
#define PWM_RATE_STEP_HZ   50
#define PWM_RATE_MIN_HZ    150
#define PWM_RATE_MAX_HZ    300
#define PWM_FIXED_DUTY     0
#define PWM_MIN_DUTY       2
#define PWM_MAX_DUTY       98
#define PWM_MAX_DUTY_MAIN  80
#define PWM_MAX_DUTY_TAIL  70
#define PWM_DIVIDER_CODE   SYSCTL_PWMDIV_4
#define PWM_DIVIDER        1
#define PWM_TICK_RATE_HZ 30 // Ouzput PWM at 30 HZ

//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5, J4-05
#define PWM_MAIN_BASE        PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//  PWM Hardware Details M1PWM5 (gen 2)
//  ---Tail Rotor PWM: PF1, J3-10
#define PWM_TAIL_BASE        PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

extern uint8_t g_dispTailPWM;
extern uint8_t g_dispMainPWM;

// *******************************************************
//
// Function sets the main PWM rotor duty cycle
//
// *******************************************************
void
setMainPWM (uint32_t ui32Freq, uint32_t ui32Duty);

// *******************************************************
//
// Function sets the rear PWM rotor duty cycle
//
// *******************************************************
void
setTailPWM (uint32_t ui32Freq, uint32_t ui32Duty);

// *******************************************************
//
// Function initialises the PWM
//
// *******************************************************
void
initialisePWM (void);


#endif /* PWM_H_ */
