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
// This is part of revision 2.1.1.71 of the DK-TM4C129X Firmware Package.
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
#include "grlib/grlib.h"
#include "grlib/widget.h"
#include "grlib/canvas.h"
#include "grlib/listbox.h"
#include "grlib/pushbutton.h"
#include "drivers/frame.h"
#include "drivers/pinout.h"
#include "drivers/kentec320x240x16_ssd2119.h"
#include "drivers/touch.h"
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
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//*****************************************************************************
//! \addtogroup example_list
//! <h1>CC3000 WiFi Access Point SSID Scanning Example (cc3000_ssid_scan)</h1>
//!
//! This example requires a CC3000 WiFi BoosterPack attached to the
//! the DK-TM4C129X Development Kit.  After booting and initializing the
//! CC3000, the application initiates a WiFi scan for access points. When
//! the scan completes, the SSID, BSSID and security protocol supported by
//! each detected access point are shown.  Another scan can be started by
//! pressing the ``Scan'' button on the touchscreen display.
//!
//! By default, the application is built to support a CC3000 BoosterPack
//! connected to the BoosterPack1 connector of the DK-TM4C129X board.
//!
//! The application may be rebuilt to support a CC3000 BoosterPack connected to
//! the board's BoosterPack 1 or BoosterPack 2 connector or a CC3000 EM
//! connected to the board's EM connectors by setting one of the labels
//! ``CC3000_USE_BOOSTERPACK1'',  ``CC3000_USE_BOOSTERPACK2'' or
//! ``CC3000_USE_EM'' in the build environment.  When connecting the CC3000 to
//! BoosterPack2, jumpers J16 and J17 must be moved to ensure that SPI signals
//! (rather than I2C) are routed to the appropriate BoosterPack connector pins.
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
// Storage for the SSID listbox widget string table and the actual strings
// themselves.
//
//*****************************************************************************
#define MAX_SSIDS 16
#define MAX_SSID_STRING_LEN 64
const char *g_ppcSSIDStringPtrs[MAX_SSIDS];
char g_ppcSSIDStrings[MAX_SSIDS][MAX_SSID_STRING_LEN];

//*****************************************************************************
//
// Widget definitions
//
//*****************************************************************************
void OnBtnScan(tWidget *psWidget);

extern tCanvasWidget g_sBackground;
extern tCanvasWidget g_sStatus;

//*****************************************************************************
//
// The listbox used to display the access points found.
//
//*****************************************************************************
ListBox(g_sSSIDList, &g_sBackground, 0, 0,
        &g_sKentec320x240x16_SSD2119,
        20, 60, 280, 170, (LISTBOX_STYLE_OUTLINE | LISTBOX_STYLE_LOCKED),
        ClrBlack, ClrDarkBlue, ClrSilver, ClrWhite, ClrWhite,
        g_psFontFixed6x8, g_ppcSSIDStringPtrs,
        MAX_SSIDS, 0, 0);

//*****************************************************************************
//
// The button used to change to the next higher directory.
//
//*****************************************************************************
RectangularButton(g_sScanBtn, &g_sBackground, &g_sSSIDList, 0,
                  &g_sKentec320x240x16_SSD2119, 250, 26, 50, 28,
                  (PB_STYLE_OUTLINE | PB_STYLE_TEXT_OPAQUE | PB_STYLE_TEXT |
                   PB_STYLE_FILL | PB_STYLE_RELEASE_NOTIFY),
                   ClrBlack, ClrBlue, ClrWhite, ClrWhite,
                   g_psFontCmss18, "Scan", 0, 0, 0, 0, OnBtnScan);

//*****************************************************************************
//
// The canvas widget displaying the scanning status.
//
//*****************************************************************************
#define SIZE_STATUS_STRING 40
char g_pcStatus[SIZE_STATUS_STRING];
Canvas(g_sStatus, &g_sBackground, &g_sScanBtn, 0,
       &g_sKentec320x240x16_SSD2119, 20, 36, 200, 24,
       CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT,
       ClrBlack, 0, ClrWhite, g_psFontCmss14, g_pcStatus, 0, 0);

//*****************************************************************************
//
// The canvas widget acting as the background to the whole display area other
// than the border (which is drawn using the FrameDraw() function).
//
//*****************************************************************************
Canvas(g_sBackground, WIDGET_ROOT, 0, &g_sStatus,
       &g_sKentec320x240x16_SSD2119, 10, 26, 300, 204,
       CANVAS_STYLE_FILL, ClrBlack, 0, 0, 0, 0, 0, 0);

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
void
CC3000_AsyncCallback(long lEventType, char *data, unsigned char length)
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
    // Configure the pinout for the board.
    //
    PinoutSet();

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

    return(0);
}

//*****************************************************************************
//
// Update the status string on the display and output the same string to the
// UART.
//
//*****************************************************************************
void
StatusUpdate(char *pcString, ...)
{
    va_list arg;

    //
    // Start the varargs processing.
    //
    va_start(arg, pcString);

    //
    // Update the status widget with the new information.
    //
    uvsnprintf(g_pcStatus, SIZE_STATUS_STRING, pcString, arg);

    //
    // Send the status to the UART.
    //
    UARTprintf("%s\n", g_pcStatus);

    //
    // Make sure the status string gets repainted.
    //
    WidgetPaint((tWidget *)&g_sStatus);

    //
    // Force an immediate repaint.
    //
    WidgetMessageQueueProcess();

    //
    // End the varargs processing.
    //
    va_end(arg);
}

//*****************************************************************************
//
// Perform a scan for WiFi access points and update the SSID listbox with the
// results.  Returns true on success, false on error.
//
//*****************************************************************************
bool
ScanForNetworks(void)
{
    tScanResult sScanResult;
    long lRetcode, lLoop, lStringIndex;
    unsigned long pulIntervalList[NUM_CHANNELS];
    bool bRetcode;


    //
    // Assume all is well unless we determine otherwise.
    //
    bRetcode = true;

    //
    // Make sure the SSID list is empty.
    //
    ListBoxClear(&g_sSSIDList);
    lStringIndex = 0;

    //
    // Start the WiFi device.
    //
    StatusUpdate("Starting WiFi device...");
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
    StatusUpdate("Setting SSID scan parameters...");
    lRetcode = wlan_ioctl_set_scan_params(1, 100, 100, 5, 0x7FF, -80, 0, 205,
                                          pulIntervalList);

    //
    // Did we set the parameters successfully?
    //
    if(lRetcode == 0)
    {
        //
        // Yes. Wait for the scan to stop.
        //
        StatusUpdate("Scanning...");

        do
        {
            lRetcode = wlan_ioctl_statusget();
        }
        while(lRetcode == WLAN_STATUS_SCANNING);

        StatusUpdate("Scan completed. Querying results...");

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
                StatusUpdate("No wireless networks found.");
            }
            else
            {
                //
                // We found something so read the rest of the results and
                // dump the information.
                //
                StatusUpdate("Found %d WiFi access points.",
                             sScanResult.ui32NumNetworks);

                //
                // Remember the number of networks that were found.
                //
                lLoop = (long)sScanResult.ui32NumNetworks;

                do
                {
                    uint32_t ui32SSIDLen;

                    //
                    // Ensure that the SSID is NULL terminated.
                    //
                    ui32SSIDLen = SCAN_SSID_LEN(sScanResult.ui8SecuritySSIDLen);

                    //
                    // If the SSID length we decode is larger than 31,
                    // something's wrong but clip the length accordingly.  Note
                    // that we clip at 32 characters rather than 32 because we
                    // need to add a terminating 0 to the string and we don't
                    // want to overwrite the BSSID field.
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

                        usnprintf(g_ppcSSIDStrings[lStringIndex],
                                  MAX_SSID_STRING_LEN,
                                  "%2d: %02x%02x%02x%02x%02x%02x %s %s ",
                                  lStringIndex,
                                  sScanResult.pcBSSID[0],
                                  sScanResult.pcBSSID[1],
                                  sScanResult.pcBSSID[2],
                                  sScanResult.pcBSSID[3],
                                  sScanResult.pcBSSID[4],
                                  sScanResult.pcBSSID[5],
                                  g_ppcSecurity[ui32SSIDLen],
                                  sScanResult.pcSSID);

                        UARTprintf("%s\n", g_ppcSSIDStrings[lStringIndex]);

                        //
                        // Add the string to the listbox.
                        //
                        ListBoxTextAdd(&g_sSSIDList,
                                       g_ppcSSIDStrings[lStringIndex]);

                        //
                        // Move on to the next string.
                        //
                        lStringIndex++;
                    }
                    else
                    {
                        //
                        // The structure passed back to us was marked as
                        // invalid.
                        //
                        UARTprintf("%d - Invalid entry received!\n",
                                   sScanResult.ui32NumNetworks);
                    }

                    //
                    // Decrement our network counter.
                    //
                    lLoop--;

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
                while(lLoop && (lRetcode == 0) && (lStringIndex < MAX_SSIDS));
            }
        }
        else
        {
            StatusUpdate("Error from wlan_ioctl_get_scan_results!");
            bRetcode = false;
        }
    }
    else
    {
        //
        // Tell the user an error was reported.
        //
        StatusUpdate("Error from wlan_ioctl_set_scan_params!");
        bRetcode = false;
    }

    //
    // Refresh the list box now that we've changed its content.
    //
    WidgetPaint((tWidget *)&g_sSSIDList);

    //
    // Stop the WiFi now that we've finished the scan.
    //
    wlan_stop();

    //
    // Tell the caller how we got on.
    //
    return(bRetcode);
}

//*****************************************************************************
//
// Handler for the "Scan" button widget.  This function is called when a user
// presses the "Scan" button.  It performs a new scan for wireless access
// points.
//
//*****************************************************************************
void
OnBtnScan(tWidget *psWidget)
{
    //
    // Start a new network scan.
    //
    ScanForNetworks();
}

//*****************************************************************************
//
// Application main entry function.
//
//*****************************************************************************
int
main(void)
{
    tContext sContext;

    //
    //  Initialize the hardware.
    //
    initDriver();

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
    // Initialize the display driver.
    //
    Kentec320x240x16_SSD2119Init(g_ui32SysClock);

    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

    //
    // Draw the application frame.
    //
    FrameDraw(&sContext, "cc3000-ssid-scan");

    //
    // Initialize the touch screen driver.
    //
    TouchScreenInit(g_ui32SysClock);

    //
    // Set the touch screen event handler.
    //
    TouchScreenCallbackSet(WidgetPointerMessage);

    //
    // Add the compile-time defined widgets to the widget tree and send an
    // initial paint message.
    //
    WidgetAdd(WIDGET_ROOT, (tWidget *)&g_sBackground);
    WidgetPaint(WIDGET_ROOT);

    //
    // Make sure the SSID list is empty.
    //
    ListBoxClear(&g_sSSIDList);

    //
    // Set the initial status.
    //
    StatusUpdate("Initializing CC3000 WiFi stack...");

    //
    // Tell the WiFi driver which application- and board-specific functions
    // to call in response to various events and when interrupt and pin
    // control is required.
    //
    wlan_init(CC3000_AsyncCallback, sendWLFWPatch, sendDriverPatch,
              sendBootLoaderPatch, ReadWlanInterruptPin,
              WlanInterruptEnable, WlanInterruptDisable,
              WriteWlanPin);

    //
    // Set up the periodic system tick and enable its interrupt.
    //
    InitSysTick();

    //
    // Perform the initial scan.
    //
    ScanForNetworks();

    //
    // Spin forever, processing the message queue.
    //
    while(1)
    {
        WidgetMessageQueueProcess();
    }
}
