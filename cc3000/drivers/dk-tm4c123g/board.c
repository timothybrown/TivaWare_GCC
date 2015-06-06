//*****************************************************************************
//
// board.c -  Board functions for CC3000 on DK-TM4C123G.
//
// Copyright (c) 2014-2015 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "socket.h"
#include "cc3000_common.h"
#include "netapp.h"
#include "spi.h"
#include "hci.h"
#include "dispatcher.h"
#include "spi_version.h"
#include "board.h"

//*****************************************************************************
//
// Initialize the board's I/O
//
//*****************************************************************************
void pio_init()
{
    //
    // The FPU should be enabled because some compilers will use floating-
    // point registers, even for non-floating-point code.  If the FPU is not
    // enabled this will cause a fault.  This also ensures that floating-
    // point operations could be added to this application and would work
    // correctly and use the hardware floating-point unit.  Finally, lazy
    // stacking is enabled for interrupt handlers.  This allows floating-
    // point instructions to be used within interrupt handlers, but at the
    // expense of extra stack usage.
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Initialize the system clock.
    //
    initClk();

    //
    // Configure the system peripheral bus that IRQ & EN pin are mapped to.
    //
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_IRQ_PORT);
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_SW_EN_PORT);

    //
    // Disable all the interrupts before configuring the lines.
    //
    MAP_GPIOIntDisable(SPI_GPIO_IRQ_BASE, 0xFF);

    //
    // Configure the WLAN_IRQ pin as an input.
    //
    MAP_GPIOPinTypeGPIOInput(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
    GPIOPadConfigSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_STRENGTH_2MA,
                        GPIO_PIN_TYPE_STD_WPU);

    //
    // Setup the GPIO interrupt for this pin
    //
    MAP_GPIOIntTypeSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_FALLING_EDGE);

    //
    // Configure the pins for the enable signal to the CC3000.
    //
    MAP_GPIOPinTypeGPIOOutput(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN);
    MAP_GPIODirModeSet(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_DIR_MODE_OUT );
    MAP_GPIOPadConfigSet(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_STRENGTH_2MA,
                         GPIO_PIN_TYPE_STD_WPD );

    MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, PIN_LOW);
    SysCtlDelay(600000);
    SysCtlDelay(600000);
    SysCtlDelay(600000);

    //
    // Disable WLAN CS with pull up Resistor
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SPI_PORT);
    MAP_GPIOPinTypeGPIOOutput(SPI_CS_PORT, SPI_CS_PIN);
    GPIOPadConfigSet(SPI_CS_PORT, SPI_CS_PIN, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, PIN_HIGH);

    //
    // Enable interrupt for WLAN_IRQ pin
    //
    MAP_GPIOIntEnable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);

    //
    // Clear interrupt status
    //
    SpiCleanGPIOISR();

    //
    // Turn on the GPIO interrupt for the IRQ line in the NVIC.
    //
    MAP_IntEnable(INT_GPIO_SPI);

}

//*****************************************************************************
//
// ReadWlanInterruptPin
//
//*****************************************************************************
long ReadWlanInterruptPin(void)
{
    return MAP_GPIOPinRead(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
// Enable waln IrQ pin
//
//*****************************************************************************
void WlanInterruptEnable()
{
    MAP_GPIOIntEnable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
// Disable waln IrQ pin
//
//*****************************************************************************
void WlanInterruptDisable()
{
    MAP_GPIOIntDisable(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);
}

//*****************************************************************************
//
// This functions enables and disables the CC3000 Radio
//
//*****************************************************************************
void WriteWlanPin( unsigned char val )
{
    if(val)
    {
        MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN,PIN_HIGH);
    }
    else
    {
        MAP_GPIOPinWrite(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, PIN_LOW);
    }

}

//*****************************************************************************
//
// Init SysTick timer.
//
//*****************************************************************************
void
InitSysTick(void)
{
    //
    // Configure SysTick to occur 10 times per second and enable its interrupt.
    //
    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
//
// The interrupt handler for the SysTick timer.
// This handler is called every 1ms
//
//*****************************************************************************
void
SysTickHandler(void)
{
    static unsigned long ulTickCount = 0;

    //
    // Increment the tick counter.
    //
    ulTickCount++;

    //
    // Has half a second passed since we last called the event handler?
    //
    if(ulTickCount >= (SYSTICK_PER_SECOND / 2))
    {
        //
        // Yes = call the unsolicited event handler.  We need to do this a
        // few times each second.
        //
        hci_unsolicited_event_handler();
        ulTickCount = 0;
    }
}

//*****************************************************************************
//
// Init the device with 16 MHz clock.
//
//*****************************************************************************
void initClk(void)
{
    //
    // 16 MHz Crystal on Board. SSI Freq - configure M4 Clock to be ~50 MHz
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                        SYSCTL_XTAL_16MHZ);
}

