//*****************************************************************************
//
// cc3000_ssid_scan.c - Basic WiFi scanning for access point SSIDs.
//
// Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.1.71 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/cpu.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"
#include "drivers/pinout.h"
#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "cc3000_common.h"
#include "netapp.h"
#include "spi.h"
#include "hci.h"
#include "spi_version.h"
#include "board.h"
#include "host_driver_version.h"
#include "security.h"

//*****************************************************************************
//! \addtogroup example_list
//! <h1>CC3000 WiFi Access Point SSID Scanning Example (cc3000_ssid_scan)</h1>
//!
//! This example requires a CC3000 WiFi BoosterPack attached to the
//! BoosterPack 2 connector of the EK-TM4C1294XL LaunchPad.  After booting and
//! initializing the CC3000, the application initiates a WiFi scan for access
//! points.  When the scan completes, the SSID, BSSID and security protocol
//! supported by each detected access point are output on the UART0 connection
//! available over the virtual COM port connection provided by the board's ICDI
//! debug interface.
//!
//! To view output from the application, set your host system's serial terminal
//! to use 115200bps, 8-N-1.
//!
//! The application may be rebuilt to support a CC3000 connected to the
//! board's BoosterPack 1 connector by replacing the label
//! ``CC3000_USE_BOOSTERPACK2'' in the build environment with
//! ``CC3000_USE_BOOSTERPACK1''.
//!
//! For information on the CC3000 software stack and API, please consult the
//! wiki at http://processors.wiki.ti.com/index.php/CC3000.
//
//*****************************************************************************

#define PLATFORM_VERSION                        5
#define CC3000_APP_BUFFER_SIZE                  5
#define CC3000_RX_BUFFER_OVERHEAD_SIZE          20
#define SL_VERSION_LENGTH                       11
#define NETAPP_IPCONFIG_MAC_OFFSET              20

#define NUM_CHANNELS                            13

//*****************************************************************************
//
// Return codes from wlan_ioctl_statusget() which appear to be missing from
// the CC3000 SDK.
//
//*****************************************************************************
#define WLAN_STATUS_DISCONNECTED                0
#define WLAN_STATUS_SCANNING                    1
#define WLAN_STATUS_CONNECTING                  2
#define WLAN_STATUS_CONNECTED                   3

//*****************************************************************************
//
// The structure returned by a call to wlan_ioctl_get_scan_results.
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32NumNetworks;
    uint32_t ui32Status;
    uint8_t  ui8ValidRSSI;
    uint8_t  ui8SecuritySSIDLen;
    uint16_t ui16Time;
    char     pcSSID[32];
    char     pcBSSID[6];
}
tScanResult;

//*****************************************************************************
//
// Status values found in the tScanResult ui32Status field.
//
//*****************************************************************************
#define SCAN_AGED_RESULT                            0
#define SCAN_RESULT_VALID                           1
#define SCAN_NO_RESULT                              2

//*****************************************************************************
//
// Masks related to values found in the tScanResult ui8ValidRSSI field.
//
//*****************************************************************************
#define SCAN_IS_VALID                               0x01
#define SCAN_RSSI_MASK                              0xFE

//*****************************************************************************
//
// Masks and labels related to values found in the tScanResult
// ui8SecuritySSIDLen field.
//
//*****************************************************************************
#define SCAN_SEC_MASK                               0x03
#define SCAN_SEC_SHIFT                                 0
#define SCAN_SEC_OPEN                               0x00
#define SCAN_SEC_WEP                                0x40
#define SCAN_SEC_WPA                                0x80
#define SCAN_SEC_WPA2                               0xC0
#define SCAN_SEC_INDEX(x) (((x) & SCAN_SEC_MASK) >> SCAN_SEC_SHIFT)

char *g_ppcSecurity[] = {"Open", "WEP", "WPA", "WPA2"};

//*****************************************************************************
//
// Mask, shift and macro to extract the SSID length from the ui8SecuritySSIDLen
// field.
//
//*****************************************************************************
#define SCAN_SSID_LEN_MASK                          0xFC
#define SCAN_SSID_LEN_SHIFT                            2
#define SCAN_SSID_LEN(x) (((x) & SCAN_SSID_LEN_MASK) >> SCAN_SSID_LEN_SHIFT)

//*****************************************************************************
//
// Declaration of global to hold clock frequency.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// driver patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char *
sendDriverPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// bootloader patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns 0.
//
//*****************************************************************************
char *
sendBootLoaderPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// The function, used during CC3000 initialization, returns a pointer to the
// firmware patch.  Since there is no patch provided at runtime (the patches
// are taken from the EEPROM and not from the host), it returns NULL.
//
//*****************************************************************************
char *
sendWLFWPatch(unsigned long *Length)
{
    *Length = 0;
    return((char *)0);
}

//*****************************************************************************
//
// Handle asynchronous events from the CC3000.
//
//*****************************************************************************
void CC3000_AsyncCallback(long lEventType, char *data, unsigned char length)
{
    //
    // Stub only - in this application, we don't need to consider any
    // asynchronous events.
    //
}

//*****************************************************************************
//
// This function performs all board level initialization and starts up the
// CC3000.
//
//*****************************************************************************
int
initDriver(void)
{
    //
    // Configure the pinout for the board but don't worry about Ethernet or
    // USB for this example.
    //
    PinoutSet(false, false);

    //
    // Set the system clock and initialize GPIOs used in this configuration.
    // Note that this call sets the global variable g_ui32SysClock as a side
    // effect.
    //
    pio_init();

    //
    // Initialize the SPI and IRQ lines connecting the CC3000.
    //
    init_spi(2000000, g_ui32SysClock);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Configure UART.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);

    //
    // Tell the user that we're up and running.
    //
    UARTprintf("\nWiFi SSID Scan Example Started.\n");

    //
    // Tell the user the driver version we're using.
    //
    UARTprintf("Driver version %d.%d\n\n",
               SPI_VERSION_NUMBER, DRIVER_VERSION_NUMBER);

    //
    // Tell the WiFi driver which application- and board-specific functions
    // to call in response to various events and when interrupt and pin
    // control is required.
    //
    UARTprintf("Initializing CC3000 WiFi stack... ");
    wlan_init(CC3000_AsyncCallback, sendWLFWPatch, sendDriverPatch,
              sendBootLoaderPatch, ReadWlanInterruptPin,
              WlanInterruptEnable, WlanInterruptDisable,
              WriteWlanPin);

    //
    // Trigger a WLAN device
    //
    UARTprintf("Done.\nStarting WiFi stack... ");
    wlan_start(0);

    //
    // Mask out all non-required events from CC3000.
    //
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT |
                        HCI_EVNT_WLAN_ASYNC_PING_REPORT);

    //
    //  Wait about 100mS or so.
    //
    ROM_SysCtlDelay((g_ui32SysClock / 10) / 3);

    //
    // Set up the periodic system tick and enable its interrupt.
    //
    InitSysTick();

    UARTprintf("Done.\n");

    return(0);
}

//*****************************************************************************
//
// Application main entry function.
//
//*****************************************************************************
int
main(void)
{
    tScanResult sScanResult;
    long lRetcode, lLoop, lCount;
    unsigned long pulIntervalList[NUM_CHANNELS];

    //
    //  Initialize the hardware.
    //
    initDriver();

    //
    // Set default values for the periodic scan interval per channel.
    //
    for(lLoop = 0; lLoop < NUM_CHANNELS; lLoop++)
    {
        pulIntervalList[lLoop] = 2000;
    }

    //
    // Tell the CC3000 to start scanning for available SSIDs.  The parameters
    // here are all the default values.  Note that wlan_start() has already
    // been called inside initDriver().
    //
    UARTprintf("Setting SSID scan parameters... ");
    lRetcode = wlan_ioctl_set_scan_params(1, 20, 30, 2, 0x7FF, -80, 0, 205,
                                          pulIntervalList);

    UARTprintf("Done\n");

    //
    // Did we set the parameters successfully?
    //
    if(lRetcode == 0)
    {
        //
        // Yes. Wait for the scan to stop.
        //
        UARTprintf("Scanning...\n");

        do
        {
            lRetcode = wlan_ioctl_statusget();
        }
        while(lRetcode == WLAN_STATUS_SCANNING);

        UARTprintf("Scan completed. Querying results...\n");

        //
        // Retrieve the first scan result to allow us to determine how many
        // APs were found (if any).
        //
        lRetcode = wlan_ioctl_get_scan_results(0,
                                               (unsigned char *)&sScanResult);
        if(lRetcode == 0)
        {
            //
            // We got the first scan result.  Did the scan complete
            // successfully?
            //
            if((sScanResult.ui32Status == SCAN_NO_RESULT) ||
               (sScanResult.ui32NumNetworks == 0))
            {
                //
                // We found no networks.
                //
                UARTprintf("No wireless networks found.\n");
            }
            else
            {
                //
                // We found something so read the rest of the results and
                // dump the information.
                //
                UARTprintf("Found %d networks.\n\n", sScanResult.ui32NumNetworks);

                //
                // Remember the number of networks that were found.
                //
                lLoop = (long)sScanResult.ui32NumNetworks;

                //
                // Initialize our network number.
                //
                lCount = 1;

                do
                {
                    uint32_t ui32SSIDLen;

                    //
                    // Ensure that the SSID is NULL terminated.
                    //
                    ui32SSIDLen = SCAN_SSID_LEN(sScanResult.ui8SecuritySSIDLen);

                    //
                    // If the SSID length we decode is larger than 31,
                    // something's wrong but clip the length accordingly. We
                    // clip at 32 characters to ensure that we don't overwrite
                    // the start of the BSSID field which appears next in
                    // memory.
                    //
                    if(ui32SSIDLen > 31)
                    {
                        ui32SSIDLen = 31;
                    }

                    sScanResult.pcSSID[ui32SSIDLen] = '\0';

                    //
                    // Print information on the current network.
                    //
                    if(sScanResult.ui8ValidRSSI & SCAN_IS_VALID)
                    {
                        //
                        // Extract the security information from the returned
                        // structure.
                        //
                        ui32SSIDLen =
                                SCAN_SEC_INDEX(sScanResult.ui8SecuritySSIDLen);

                        UARTprintf("%2d: %02x%02x%02x%02x%02x%02x %s %s\n",
                                   lCount,
                                   sScanResult.pcBSSID[0],
                                   sScanResult.pcBSSID[1],
                                   sScanResult.pcBSSID[2],
                                   sScanResult.pcBSSID[3],
                                   sScanResult.pcBSSID[4],
                                   sScanResult.pcBSSID[5],
                                   g_ppcSecurity[ui32SSIDLen],
                                   sScanResult.pcSSID);
                    }
                    else
                    {
                        //
                        // The structure passed back to us was marked as
                        // invalid.
                        //
                        UARTprintf("%d - Invalid entry received!\n", lCount);
                    }

                    //
                    // Decrement our network counter.
                    //
                    lLoop--;

                    //
                    // Increment our network number.
                    //
                    lCount++;

                    //
                    // Get the information on the next network if there is
                    // another one to get.
                    //
                    if(lLoop)
                    {
                        lRetcode = wlan_ioctl_get_scan_results(0,
                                                (unsigned char *)&sScanResult);
                    }
                }
                while(lLoop && (lRetcode == 0));
            }
        }
        else
        {
            UARTprintf("Error from wlan_ioctl_get_scan_results!\n");
        }
    }
    else
    {
        //
        // Tell the user an error was reported.
        //
        UARTprintf("Error from wlan_ioctl_set_scan_params!\n");
    }

    UARTprintf("\nScanning completed.\n");

    //
    // Stop the WiFi.
    //
    wlan_stop();

    //
    // We're done - spin forever.
    //
    while(1)
    {
        //
        // Put the CPU to sleep unless there's an interrupt to process.
        //
        CPUwfi();
    }
}
