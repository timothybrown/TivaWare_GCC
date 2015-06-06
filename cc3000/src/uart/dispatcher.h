//*****************************************************************************
//
// dispatcher.h  - CC3000 Host Driver Implementation.
// Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
//
//*****************************************************************************

#ifndef __DISPATCHER_H__
#define __DISPATCHER_H__

#include <string.h>
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/udma.h"
#include "driverlib/uart.h"

/* Library API Includes */
//#include "hci.h"
//#include "spi.h"
//#include "board.h"

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef  __cplusplus
extern "C" {
#endif

//
// Define the size of UART IF buffer for RX
//
#define UART_IF_BUFFER                      256
#define PIN_HIGH                            0xFF
#define PIN_LOW                             (!PIN_HIGH)
#define TRUE                                1
#define FALSE                               (!TRUE)

//
// Define the UART IF buffer
//
extern unsigned char g_ucUARTBuffer[];
//flag for new UART command indication
extern volatile unsigned char uart_have_cmd;
//counter for the number of bytes received
extern volatile unsigned long g_ulRxBuffCount;
void DispatcherUARTConfigure(unsigned long ucSysClock);
void DispatcherUartSendPacket(unsigned char *inBuff, unsigned short usLength);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __DISPATCHER_H__
