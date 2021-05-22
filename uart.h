// *******************************************************
//
// uart.h
//
// UART code is set to output to a port.
// Code modified from lab code.
//
// Authors: Luke Roeven (ljr83)
//          Anahita Piri (api48)
//          Maggie Booker (meb139)
//
// *******************************************************


#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// *******************************************************
//
// Globals and constants defined for uart
//
// *******************************************************

#define UART_TICK_RATE_HZ 4
#define MAX_STR_LEN 256
//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

extern char g_statusStr[MAX_STR_LEN + 1];


//********************************************************
//
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//
//********************************************************
void
initialiseUSB_UART (void);

//**********************************************************************
//
// Transmit a string via UART0
//
//**********************************************************************
void
UARTSend (char *pucBuffer);

#endif /* UART_H_ */
