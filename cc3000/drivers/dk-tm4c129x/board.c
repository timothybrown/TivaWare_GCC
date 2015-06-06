//*****************************************************************************
//
// board.c -  Board functions for CC3000 stack on DK-TM4C129X
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
// Global to hold clock frequency. It is returned during ClockFreqSet and must
// be held onto. SysCtlClockGet() is no longer a valid function on TM4C129x
// chips.
//
//*****************************************************************************
uint32_t g_ui32SysClock=0;

//*****************************************************************************
//
// A structure used to hold information defining the location of a single LED
// on the board.
//
//*****************************************************************************
typedef struct
{
    tBoardLED eLED;
    uint32_t ui32Periph;
    uint32_t ui32Base;
    uint8_t ui8Pin;
    bool bHighIsOn;
}
tLEDInfo;

//*****************************************************************************
//
// A structure defining all the LEDs on this board.
//
//*****************************************************************************
const tLEDInfo g_psLEDs[] =
{
    {RED_LED, CC3000_LED_RED_SYSCTL_PERIPH, CC3000_LED_RED_PORT,
     CC3000_LED_RED_PIN, true},
    {GREEN_LED, CC3000_LED_GREEN_SYSCTL_PERIPH, CC3000_LED_GREEN_PORT,
     CC3000_LED_GREEN_PIN, true},
    {BLUE_LED, CC3000_LED_BLUE_SYSCTL_PERIPH, CC3000_LED_BLUE_PORT,
     CC3000_LED_BLUE_PIN, true},
};

#define NUM_LEDS (sizeof(g_psLEDs) / sizeof(tLEDInfo))

//*****************************************************************************
//
// Initialize the board's I/O
//
//*****************************************************************************
void pio_init()
{
    //  Board Initialization start
    //
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
    //Init the device with 16 MHz clock.
    //
    initClk();

    /* Configure the system peripheral bus that IRQ & EN pin are map to */
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_IRQ_PORT);
    MAP_SysCtlPeripheralEnable( SYSCTL_PERIPH_SW_EN_PORT);

    //
    // Disable all the interrupts before configuring the lines
    //
    MAP_GPIOIntDisable(SPI_GPIO_IRQ_BASE, 0xFF);
    MAP_GPIOIntDisable(SPI_GPIO_SW_EN_BASE, 0xFF);

    //
    // Configure WLAN_IRQ pin as input
    //
    MAP_GPIOPinTypeGPIOInput(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN);

    GPIOPadConfigSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_STRENGTH_2MA,
                        GPIO_PIN_TYPE_STD_WPU);
    //
    // Setup the GPIO interrupt for this pin
    //
    MAP_GPIOIntTypeSet(SPI_GPIO_IRQ_BASE, SPI_IRQ_PIN, GPIO_FALLING_EDGE);

    //
    // Configure WLAN chip
    //
    MAP_GPIOPinTypeGPIOOutput(SPI_GPIO_SW_EN_BASE, SPI_EN_PIN);
    MAP_GPIODirModeSet( SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_DIR_MODE_OUT );
    MAP_GPIOPadConfigSet( SPI_GPIO_SW_EN_BASE, SPI_EN_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD );

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

    MAP_IntEnable(INT_GPIO_SPI);

    //init LED
    initLEDs();
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
    SysTickPeriodSet(g_ui32SysClock / SYSTICK_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
//
// The interrupt handler for the SysTick timer.  This handler is called every 1ms
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
// Initialize the Clock
//
//*****************************************************************************
void initClk(void)
{
    //
    // Set the system clock rate to the frequency defined in board.h.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480),
                                            SYSCLOCK_RATE_HZ);
}

//*****************************************************************************
//
// Initialize all LEDs on the board.
//
//*****************************************************************************
void initLEDs()
{
    uint32_t ui32Loop;

    //
    // Loop through each of the configured LEDs
    //
    for(ui32Loop = 0; ui32Loop < NUM_LEDS; ui32Loop++)
    {
        //
        // Enable the GPIO peripheral containing the LED control line.
        //
        MAP_SysCtlPeripheralEnable(g_psLEDs[ui32Loop].ui32Periph);

        //
        // Configure the LED pin as an output.
        //
        MAP_GPIOPinTypeGPIOOutput(g_psLEDs[ui32Loop].ui32Base,
                                  g_psLEDs[ui32Loop].ui8Pin);

        //
        // Turn the LED off.
        //
        MAP_GPIOPinWrite(g_psLEDs[ui32Loop].ui32Base,
                         g_psLEDs[ui32Loop].ui8Pin,
                         (g_psLEDs[ui32Loop].bHighIsOn ?
                          0 : g_psLEDs[ui32Loop].ui8Pin));
    }
}

//*****************************************************************************
//
// Turns a single board LED on.
//
//*****************************************************************************
void turnLedOn(tBoardLED eLED)
{
    uint32_t ui32Loop;

    //
    // Loop through each of the LEDs on this board looking for the one we've
    // been asked for.
    //
    for(ui32Loop = 0; ui32Loop < NUM_LEDS; ui32Loop++)
    {
        //
        // Have we found the requested LED's information?
        //
        if(g_psLEDs[ui32Loop].eLED == eLED)
        {
            //
            // Yes - turn it on.
            //
            MAP_GPIOPinWrite(g_psLEDs[ui32Loop].ui32Base,
                             g_psLEDs[ui32Loop].ui8Pin,
                             (g_psLEDs[ui32Loop].bHighIsOn ?
                              g_psLEDs[ui32Loop].ui8Pin : 0));

            //
            // We found the LED so there's no point in continuing the loop.
            //
            return;
        }
    }
}

//*****************************************************************************
//
// Turns off a single LED on the board.
//
//*****************************************************************************
void turnLedOff(tBoardLED eLED)
{
    uint32_t ui32Loop;

    //
    // Loop through each of the LEDs on this board looking for the one we've
    // been asked for.
    //
    for(ui32Loop = 0; ui32Loop < NUM_LEDS; ui32Loop++)
    {
        //
        // Have we found the requested LED's information?
        //
        if(g_psLEDs[ui32Loop].eLED == eLED)
        {
            //
            // Yes - turn it off.
            //
            MAP_GPIOPinWrite(g_psLEDs[ui32Loop].ui32Base,
                             g_psLEDs[ui32Loop].ui8Pin,
                             (g_psLEDs[ui32Loop].bHighIsOn ?
                              0 : g_psLEDs[ui32Loop].ui8Pin));

            //
            // We found the LED so there's no point in continuing the loop.
            //
            return;
        }
    }
}
