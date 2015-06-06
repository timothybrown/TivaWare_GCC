//*****************************************************************************
//
// dispatcher.c  - CC3000 Host Driver Implementation.
//
// Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
//
//*****************************************************************************


//*****************************************************************************
//
//! \addtogroup dispatcher_api
//! @{
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "dispatcher.h"
#include "hci.h"
#include "spi.h"
#include "board.h"

unsigned long g_uluDMAErrCount =0;

//*****************************************************************************
//
//! DispatcherUartSendPacket
//!
//!  \param  inBuff    pointer to the UART input buffer
//!  \param  usLength  buffer length
//!
//!  \return none
//!
//!  \brief  The function sends to UART a buffer of given length
//
//*****************************************************************************
void DispatcherUartSendPacket(unsigned char *inBuff, unsigned short usLength)
{
    unsigned long ulIndex=0;
    for (ulIndex =0 ; ulIndex < usLength; ulIndex++)
    {
        UARTprintf("%c",inBuff[ulIndex]);
       //MAP_UARTCharPut(CC3000_UART_BASE, *inBuff++);
   }
}

//*****************************************************************************
//
//! Cofigure the UART
//!
//!  @param  none
//!
//!  @return none
//!
//!  @brief  Cofigure the UART
//
//*****************************************************************************
void
DispatcherUARTConfigure(unsigned long ucSysClock)
{

    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(CC3000_UART_SYSCTL_PERIPH_GPIO);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(CC3000_UART_SYSCTL_PERIPH_UART);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(CC3000_UART_GPIO_RX);
    MAP_GPIOPinConfigure(CC3000_UART_GPIO_TX);
    MAP_GPIOPinTypeUART(CC3000_UART_GPIO_PORT_BASE, CC3000_UART_GPIO_PINS);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    //UARTClockSourceSet(CC3000_UART_BASE, UART_CLOCK_PIOSC);

    MAP_UARTIntDisable(CC3000_UART_BASE, 0xFFFFFFFF);

    // Set both the TX and RX trigger thresholds to 4.  This will be used by
    // the uDMA controller to signal when more data should be transferred.  The
    // uDMA TX and RX channels will be configured so that it can transfer 2
    // bytes in a burst when the UART is ready to transfer more data.
    //
    UARTFIFOLevelSet(CC3000_UART_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(CC3000_UART_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

    //
    // Clear all the pending interrupts
    //
    MAP_UARTIntClear(CC3000_UART_BASE, 0xFFFFFFFF);

    //
    // Enable the UART interrupt
    //
    MAP_IntEnable(CC3000_UART_INT);\
    MAP_UARTIntEnable(CC3000_UART_BASE, UART_INT_RT|UART_INT_RX);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
