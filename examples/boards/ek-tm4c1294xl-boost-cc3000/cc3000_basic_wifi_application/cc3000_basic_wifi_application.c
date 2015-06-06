//*****************************************************************************
//
// basic_wifi_application.c - A command-line driven CC3000 WiFi example.
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
// This is part of revision 2.1.1.71 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"
#include "utils/ustdlib.h"
#include "utils/uartstdio.h"
#include "utils/cmdline.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

#include "wlan.h"
#include "evnt_handler.h"
#include "nvmem.h"
#include "socket.h"
#include "netapp.h"
#include "spi.h"
#include "hci.h"

#include "dispatcher.h"
#include "spi_version.h"
#include "board.h"
#include "application_version.h"
#include "host_driver_version.h"
#include "security.h"
#include "application_commands.h"

//*****************************************************************************
//! \addtogroup example_list
//! <h1>CC3000 Basic WiFi Example (cc3000_basic_wifi_application)</h1>
//!
//! This is a basic WiFi application for the CC3000 BoosterPack. This
//! application is a command line wrapper for various functions that the
//! CC3000 can provide. Please refer to the CC3000 wiki at
//! http://processors.wiki.ti.com/index.php/CC3000 for more information on
//! the commands provided.
//!
//! To see available commands type ``help'' at the serial terminal prompt.
//! The terminal is connected in 8-N-1 mode at 115200 baud.
//!
//! This example defaults to using BoosterPack2. If you would like to use
//! BoosterPack1 instead please change the define CC3000_USE_BOOSTERPACK2 in
//! the project settings to CC3000_USE_BOOSTERPACK1 and rebuild.
//!
//! To use this example you must first connect to an existing unencrypted
//! wireless network. This can be done by using the ``smartconfig'' command
//! with the associated smartphone application. Alternatively, the connection
//! can be made manually by using the 'connect' command. Once connected you can
//! do any of the following.
//!
//! <b>Configure an IP address:</b>
//!
//! <ol>
//! <li>To use DHCP to allocate a dynamic IP address ``ipconfig'' or
//! ``ipconfig 0 0 0'' or,
//! <li>To allocate a static IP address use ``ipconfig a.b.c.d'' where
//! ``a.b.c.d'' is the required, dotted-decimal format address.
//! </ol>
//!
//! <b>Send and receive UDP data:</b>
//!
//! <ol>
//! <li>Open a UDP socket ``socketopen UDP''.
//! <li>Bind the socket to a local port ``bind 8080''.
//! <li>Send or receive data ``senddata 192.168.1.101 8080 helloworld'' or
//! ``receivedata''.  In the senddata case, the provided parameters identify
//! the IP address of the remote host and the remote port number to which the
//! data is to be sent.
//! </ol>
//!
//! <b>Send and receive TCP data:</b>
//!
//! <ol>
//! <li>Open a TCP socket ``socketopen TCP''.
//! <li>Bind the socket to a local port ``bind 8080''.
//! <li>Send a request to the remote server ``senddata 192.168.1.101 8080
//! helloworld''.  On the first ``senddata'' after opening the socket, the
//! socket is connected to the specified remote host and port.  On further
//! ``senddata'' requests, the remote address and port are ignored and the
//! existing connection is used.
//! <li>Receive data from the remote server ``receivedata''.
//! </ol>
//!
//! Note that, in the current implementation, the application only supports
//! acting as a TCP client.  The CC3000 also supports incoming connections
//! as required to operate as a TCP server but this example does not yet
//! include support for this feature.
//!
//! <b>Send mDNS advertisement:</b>
//!
//! <ol>
//! <li>``mdnsadvertise cc3000''
//! </ol>
//!
//! <b>Close the open socket:</b>
//!
//! <ol>
//! <li>``socketclose''
//! </ol>
//!
//! <b>Disconnect from network:</b>
//!
//! <ol>
//! <li>``disconnect''
//! </ol>
//!
//! <b>Reset the CC3000:</b>
//!
//! <ol>
//! <li>``resetcc3000''
//! </ol>
//!
//! <b>Delete connection policy:</b>
//!
//! This deletes the connection policy from CC3000 memory so that the device
//! won't auto connect whenever it is reset in future.
//!
//! <ol>
//! <li>``deletepolicy''
//! </ol>
//
//*****************************************************************************

#ifdef DEBUG
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    //
    // Tell the user about the error reported.
    //
    UARTprintf("Runtime error at line %d of file %s!\n", ui32Line, pcFilename);

    while(1)
    {
        //
        // Hang here to allow debug.
        //
    }
}
#endif // DEBUG

//*****************************************************************************
//
//  Global variables used by this program for event tracking.
//
//*****************************************************************************
volatile uint32_t g_ui32SmartConfigFinished, g_ui32CC3000Connected,
                  g_ui32CC3000DHCP,g_ui32OkToDoShutDown,
                  g_ui32CC3000DHCP_configured;

//*****************************************************************************
//
// Global to see if socket has been connected to (valid for TCP only)
//
//*****************************************************************************
bool g_bSocketConnected = false;

//*****************************************************************************
//
//  Smart Config flag variable. Used to stop Smart Config.
//
//*****************************************************************************
volatile uint8_t g_ui8StopSmartConfig;

//*****************************************************************************
//
// Global socket handle.
//
//*****************************************************************************
volatile uint32_t g_ui32Socket = SENTINEL_EMPTY;

//*****************************************************************************
//
// Global to hold socket type. Rewritten each time 'socketopen' is called.
//
//*****************************************************************************
sockaddr g_tSocketAddr;

//*****************************************************************************
//
// Flag used to signify type of socket, TCP or UDP.
//
//*****************************************************************************
uint32_t g_ui32SocketType = 0;

//*****************************************************************************
//
// Flag used to denote of bind has been called on socket yet
//
//*****************************************************************************
uint32_t g_ui32BindFlag = SENTINEL_EMPTY;

//*****************************************************************************
//
// Simple config prefix
//
//*****************************************************************************
char g_pcCC3000_prefix[] = {'T', 'T', 'T'};

//*****************************************************************************
//
// Input buffer for the command line interpreter.
//
//*****************************************************************************
static char g_cInput[MAX_COMMAND_SIZE];

//*****************************************************************************
//
// Device name used by default for smart config response & mDNS advertising.
//
//*****************************************************************************
char g_pcdevice_name[] = "home_assistant";

//*****************************************************************************
//
// AES key "smartconfigAES16"
//
//*****************************************************************************
const uint8_t g_pui8smartconfigkey[] = {0x73,0x6d,0x61,0x72,0x74,0x63,0x6f,
                                        0x6e,0x66,0x69,0x67,0x41,0x45,0x53,
                                        0x31,0x36};

//*****************************************************************************
//__no_init is used to prevent the buffer initialization in order to
// prevent hardware WDT expiration  before entering 'main()'.
//for every IDE, different syntax exists :
// __CCS__ for CCS v5
// __IAR_SYSTEMS_ICC__ for IAR Embedded Workbench
//
// Reception from the air, buffer - the max data length  + headers
//
//*****************************************************************************

//
// Code Composer Studio pragmas.
//
#if defined(__CCS__) || defined(ccs)
uint8_t g_pui8CC3000_Rx_Buffer[CC3000_APP_BUFFER_SIZE +
                                            CC3000_RX_BUFFER_OVERHEAD_SIZE];

//
// IAR Codebench, aka ewarm pragmas.
//
#elif defined(__IAR_SYSTEMS_ICC__) || (ewarm)
__no_init uint8_t g_pui8CC3000_Rx_Buffer[CC3000_APP_BUFFER_SIZE +
                                            CC3000_RX_BUFFER_OVERHEAD_SIZE];

//
// Code Sourcery, GCC, and Keil all use the same pragmas.
//
#else
uint8_t g_pui8CC3000_Rx_Buffer[CC3000_APP_BUFFER_SIZE +
                                            CC3000_RX_BUFFER_OVERHEAD_SIZE];
#endif

//*****************************************************************************
//
// This function returns a pointer to the driver patch.  Since there is
// no patch (patches are taken from the EEPROM and not from the host) it
// returns NULL.
//
//*****************************************************************************
char *
sendDriverPatch(unsigned long *Length)
{
    *Length = 0;
    return(NULL);
}

//*****************************************************************************
//
// This function returns a pointer to the bootloader patch.  Since there
// is no patch (patches are taken from the EEPROM and not from the host)
// it returns NULL.
//
//*****************************************************************************
char *
sendBootLoaderPatch(unsigned long *Length)
{
    *Length = 0;
    return(NULL);
}

//*****************************************************************************
//
// This function returns a pointer to the driver patch.  Since there is
// no patch (patches are taken from the EEPROM and not from the host)
// it returns NULL.
//
//*****************************************************************************
char *
sendWLFWPatch(unsigned long *Length)
{
    *Length = 0;
    return(NULL);
}

//*****************************************************************************
//
// This function handles asynchronous events that come from CC3000.
//
//*****************************************************************************
void
CC3000_UsynchCallback(long lEventType, char *pcData, unsigned char ucLength)
{
    netapp_pingreport_args_t *psPingData;

    //
    // Handle completion of simple config callback
    //
    if(lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
    {

        UARTprintf("\r    Received asynchronous simple config done event "
                         "from CC3000\n>",0x08);

        g_ui32SmartConfigFinished = 1;
        g_ui8StopSmartConfig = 1;
    }

    //
    // Handle unsolicited connect callback.
    //
    if(lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
    {
        //
        // Set global variable to indicate connection.
        //
        g_ui32CC3000Connected = 1;
    }

    //
    // Handle unsolicited disconnect callback. Turn LED Green -> Red.
    //
    if(lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
    {
        UARTprintf("\r    Received Unsolicited Disconnect from CC3000\n>");

        g_ui32CC3000Connected = 0;
        g_ui32CC3000DHCP = 0;
        g_ui32CC3000DHCP_configured = 0;

        //
        // Turn off the LED3.
        //
        turnLedOff(LED3);

        //
        // Turn back on the LED1.
        //
        turnLedOn(LED1);
    }

    //
    // Handle DHCP connection callback.
    //
    if(lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
    {
        //
        // Notes:
        // 1) IP config parameters are received swapped
        // 2) IP config parameters are valid only if status is OK,
        //      i.e. g_ui32CC3000DHCP becomes 1
        //

        //
        // Only if status is OK, the flag is set to 1 and the addresses are
        // valid
        //
        if( *(pcData + NETAPP_IPCONFIG_MAC_OFFSET) == 0)
        {
            UARTprintf("\r    DHCP Connected. IP: %d.%d.%d.%d\n",
                       pcData[3], pcData[2], pcData[1], pcData[0]);

            //
            // DHCP success, set global accordingly.
            //
            g_ui32CC3000DHCP = 1;

            //
            // Turn on the LED3.
            //
            turnLedOn(LED3);
        }
        else
        {
            //
            // DHCP failed, set global accordingly.
            //
            g_ui32CC3000DHCP = 0;
        }
    }
    //
    // Ping event handler
    //
    if(lEventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT)
    {
        //
        // Ping data received, print to screen
        //
        psPingData = (netapp_pingreport_args_t *)pcData;
#ifdef DEBUG
        UARTprintf("    Data Received='\n");
        for(lEventType = 0; lEventType < ucLength; lEventType++)
        {
            UARTprintf("%d,",pcData[lEventType]);
        }
        UARTprintf("'\n");
#endif

        //
        // Test for ping failure
        //
        if(psPingData->min_round_time == -1)
        {
            UARTprintf("\r    Ping Failed. Please check address and try "
                       "again.\n>");
        }
        else
        {
            UARTprintf("\r    Ping Results:\n"
                       "    sent: %d, received: %d, min time: %dms,"
                       " max time: %dms, avg time: %dms\n>",
                       psPingData->packets_sent, psPingData->packets_received,
                       psPingData->min_round_time, psPingData->max_round_time,
                       psPingData->avg_round_time);
        }
    }

    //
    // Handle shutdown callback.
    //
    if(lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
    {
        //
        // Set global variable to indicate the device can be shutdown.
        //
        g_ui32OkToDoShutDown = 1;
    }
}

//*****************************************************************************
//
// This function initializes a CC3000 device and triggers it to start
// operation
//
//*****************************************************************************
int
initDriver(void)
{
    //
    // Initialize the system clock and board pinout.
    //
    pio_init();

    //
    // Initialize SPI
    //
    init_spi(1000000,g_ui32SysClock);

    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Initialize the CC3000 firmware.
    //
    wlan_init(CC3000_UsynchCallback, sendWLFWPatch, sendDriverPatch,
              sendBootLoaderPatch, ReadWlanInterruptPin,
              WlanInterruptEnable, WlanInterruptDisable, WriteWlanPin);

    //
    // Start the WiFI stack on the CC3000.
    //
    wlan_start(0);

    //
    // Turn on the LED1 to indicate that we are active and initiated
    // WLAN successfully.
    //
    turnLedOn(LED1);

    //
    // Mask out all non-required events from CC3000
    //
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT);

    DispatcherUARTConfigure(g_ui32SysClock);

    ROM_SysCtlDelay(1000000);

    //
    // Print version string.
    //
    UARTprintf("\n\n\rCC3000 Basic Wifi Application: driver version "
                "%d.%d.%d.%d \n\r",PLATFORM_VERSION, APPLICATION_VERSION,
                SPI_VERSION_NUMBER, DRIVER_VERSION_NUMBER );
    UARTprintf("Type 'help' for a list of commands\n");

    //
    // Set flag to stop smart config if running.
    //
    g_ui8StopSmartConfig = 0;

    //
    // Configure SysTick to occur X times per second, to use as a time
    // reference. Enable SysTick to generate interrupts.
    //
    InitSysTick();

    return(0);
}

//*****************************************************************************
//
// Takes in a string of the form YYY.YYY.YYY.YYY (where YYY is a string
// representation of a decimal between0 and 255), converts each pair to a
// decimal. Used to parse user input from the command line.
//
// Returns 0 on success, -1 on fail.
//
//*****************************************************************************
int
DotDecimalDecoder(char *pcString, uint8_t *pui8Val1, uint8_t *pui8Val2,
                  uint8_t *pui8Val3, uint8_t *pui8Val4)
{
    uint32_t ui32Block1,ui32Block2,ui32Block3,ui32Block4;
    char *pcEndData, *pcStartData;

    //
    // Extract 1st octet of address.
    //
    pcStartData = pcString;
    pcEndData = ustrstr(pcStartData,".");
    ui32Block1 = ustrtoul(pcStartData, 0,10);

    //
    // Extract 2nd octet of address.
    //
    pcStartData = pcEndData +1;
    pcEndData = ustrstr(pcStartData,".");
    ui32Block2 = ustrtoul(pcStartData, 0,10);

    //
    // Extract 3rd octet of address.
    //
    pcStartData = pcEndData +1;
    pcEndData = ustrstr(pcStartData,".");
    ui32Block3 = ustrtoul(pcStartData, 0,10);

    //
    // Extract 4th octet of address.
    //
    pcStartData = pcEndData +1;
    pcEndData = ustrstr(pcStartData,".");
    ui32Block4 = ustrtoul(pcStartData, 0,10);

    //
    // Validate data. Valid values are between 0->255.
    //
    if((ui32Block1 > 255) || (ui32Block2 > 255) || (ui32Block3 > 255) ||
       (ui32Block4 > 255))
    {
        //
        // Exit with failure if any values are > 255
        //
        return(COMMAND_FAIL);
    }

    //
    // Assign address values to variables passed in
    //
    *pui8Val1 = (uint8_t)ui32Block1;
    *pui8Val2 = (uint8_t)ui32Block2;
    *pui8Val3 = (uint8_t)ui32Block3;
    *pui8Val4 = (uint8_t)ui32Block4;

    return(0);
}

//*****************************************************************************
//
// Table of valid command strings, callback functions and help messages.  This
// is used by the cmdline module.
//
//*****************************************************************************
tCmdLineEntry g_psCmdTable[] =
{
    {"help",          CMD_help,
                  " : Display this list of commands." },
    {"smartconfig",   CMD_smartConfig,
                  " : First time simple configuration. Use app on smartphone\n"
"                     to connect to network." },
    {"connect",       CMD_connect,
                  " : [1]SSID : Connect to an open access point." },
    {"ipconfig",      CMD_ipConfig,
                  " : [1]Local IP address [2]Default gateway\n"
"                     [3](optional) Network mask. For DHCP give no arguments."},
    {"socketopen",    CMD_socketOpen,
                  " : [1]UDP/TCP : Open socket, specify TCP or UDP." },
    {"bind",          CMD_bind,
                  " : [1]Port to bind socket to" },
    {"senddata",      CMD_sendData,
                  " : [1]IP Address [2]Destination Port\n"
"                     [3]Data to send ( < 255 bytes, no spaces allowed)." },
    {"receivedata",   CMD_receiveData,
                  " : Receive data on a socket." },
    {"mdnsadvertise", CMD_mdnsadvertise,
                  " : [1](optional) name to broadcast via mDNS to "
                  "connected network." },
    {"resetcc3000",   CMD_cc3000reset,
                  " : Reset the CC3000."},
    {"socketclose",   CMD_socketClose,
                  " : Close the open socket." },
    {"disconnect",    CMD_disconnect,
                  " : Disconnect from the network." },
    {"deletepolicy",  CMD_deletePolicy,
                  " : Delete the automatic connection policy. On reset CC3000\n"
"                     will not automatically reconnect to the network." },
    {"ping",          CMD_ping,
                  " : [1]IP [2](optional) Maximum tries,\n"
"                     [3](optional) Timeout in ms."},
    { 0, 0, 0 }
};

//*****************************************************************************
//
// This function triggers a smart configuration process on CC3000.
//
//*****************************************************************************
void StartSmartConfig(void)
{
    g_ui32SmartConfigFinished = 0;
    g_ui32CC3000Connected = 0;
    g_ui32CC3000DHCP = 0;
    g_ui32OkToDoShutDown=0;

    //
    // Reset all the previous configuration
    //
    wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);

    //
    // Delete all previous profiles. 255 means delete all.
    //
    wlan_ioctl_del_profile(255);

    //
    // Inform user of status.
    //
    UARTprintf("    Waiting on Smartphone Smart Config App...\n");

    //
    // Wait until CC3000 is disconnected
    //
    while(g_ui32CC3000Connected == 1)
    {
        ROM_SysCtlDelay(100);
        hci_unsolicited_event_handler();
    }

    //
    // Start blinking LED1 during Smart Configuration process.
    //
    turnLedOn(LED1);

    //
    // Set the prefix used for smart config.
    //
    wlan_smart_config_set_prefix(g_pcCC3000_prefix);
    turnLedOff(LED1);

    //
    // Start the SmartConfig process.
    //
    wlan_smart_config_start(1);
    turnLedOn(LED1);

    //
    // Wait for Smart config to finish. Flash LED.
    //
    while(g_ui32SmartConfigFinished == 0)
    {
        turnLedOff(LED1);
        ROM_SysCtlDelay(16500000);
        turnLedOn(LED1);
        ROM_SysCtlDelay(16500000);
    }
    turnLedOn(LED1);

    //
    // Create new entry for AES encryption key.
    //
    nvmem_create_entry(NVMEM_AES128_KEY_FILEID,16);

    //
    // Write AES key to NVMEM.
    //
    aes_write_key((unsigned char *)(&g_pui8smartconfigkey[0]));

    //
    // Decrypt configuration information and add profile.
    //
    wlan_smart_config_process();

    //
    // Configure to connect automatically to the AP retrieved in the
    // Smart config process.
    //
    wlan_ioctl_set_connection_policy(DISABLE, DISABLE, ENABLE);

    //
    // Reset the CC3000, necessary to apply configuration.
    //
    wlan_stop();

    //
    // Tell user we're done with smart config.
    //
    UARTprintf("\r    Smart Config DONE\n");

    //
    // Mandatory delay between calls to wlan_stop and wlan_start.
    //
    ROM_SysCtlDelay(g_ui32SysClock / 3000);
    //
    // Start up the CC3000 again.
    //
    wlan_start(0);

    //
    // Mask out all non-required events, these events will be ignored by the
    // asynchronous callback.
    //
    wlan_set_event_mask(HCI_EVNT_WLAN_KEEPALIVE | HCI_EVNT_WLAN_UNSOL_INIT);
}

//*****************************************************************************
//
// Print the help strings for all commands.
//
//*****************************************************************************
int
CMD_help(int argc, char **argv)
{
    int32_t i32Index;

    (void)argc;
    (void)argv;

    //
    // Start at the beginning of the command table.
    //
    i32Index = 0;

    //
    // Get to the start of a clean line on the serial output.
    //
    UARTprintf("\nAvailable Commands\n------------------\n\n");

    //
    // Display strings until we run out of them.
    //
    while(g_psCmdTable[i32Index].pcCmd)
    {
        //
        // Display one command's help information.
        //
        UARTprintf("%17s %s\n", g_psCmdTable[i32Index].pcCmd,
                   g_psCmdTable[i32Index].pcHelp);
        i32Index++;

        //
        // Make sure the text makes it to the host.  Without this flush call,
        // we can overflow the transmit buffer and lose some of the help
        // information.
        //
        UARTFlushTx(false);
    }

    //
    // Leave a blank line after the help strings.
    //
    UARTprintf("\n");

    return(0);
}

//*****************************************************************************
//
// This function runs the Smart Configuration process.
// This function is meant to be run in tandem with the smartphone
// application. The user enters the network information on the smart phone and
// then saves it to the microcontroller. Easiest way to connect CC3000 to a
// network.
//
//*****************************************************************************
int
CMD_smartConfig(int argc, char **argv)
{
    StartSmartConfig();

    return(0);
}

//*****************************************************************************
//
// Manually connect to an open network. For advanced users.
// Arguments:
// [1]SSID
//
//*****************************************************************************
int
CMD_connect(int argc, char **argv)
{
    uint32_t ui32SsidLen, ui32x;
    char *pui8Ssid;

    //
    // Validate input.
    //
    if(argc < 2)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 2)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }
    else if(strlen(argv[1]) >= 255)
    {
        UARTprintf("Length of SSID must be less than 255\n");
        return(CMDLINE_INVALID_ARG);
    }

    //
    // Extract the SSID from the input parameters and determine the string
    // length.
    //
    pui8Ssid = argv[1];
    ui32SsidLen = ustrlen(argv[1]);

    //
    // Call low level connect function. See documentation for more information.
    //
#ifndef CC3000_TINY_DRIVER
    wlan_connect(WLAN_SEC_UNSEC, pui8Ssid, ui32SsidLen,NULL, NULL, 0);
#else
    wlan_connect(pui8Ssid,ui32SsidLen);
#endif

    //
    // Tell user what we're doing.
    //
    UARTprintf("    Attempting to connect (5 second timeout)...\n");

    //
    // Wait for connect message for 5 seconds,
    //
    for(ui32x = 0; ui32x < 5000; ui32x++)
    {
        //
        // Check to see if we're connected to a network yet.
        //
        if(g_ui32CC3000DHCP == 1)
        {
            return(0);
        }

        //
        // Delay 1ms
        //
        ROM_SysCtlDelay(g_ui32SysClock / 3000);
    }
    UARTprintf("    Connection Failed. Please check the network name.\n");

    return(0);

}

//*****************************************************************************
//
// Open a UDP or TCP socket.
// Arguments:
// [1] 'TCP' or 'UDP' to specify socket type
//
//*****************************************************************************
int
CMD_socketOpen(int argc, char **argv)
{
    int32_t i32Check = 0;

    //
    // Validate input.
    //
    if(argc < 2)
    {
        UARTprintf("    Please specify socket type, 'TCP' or 'UDP'\n");
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 2)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }
    else if( (g_ui32CC3000DHCP == 0) || (g_ui32CC3000Connected == 0) )
    {
        UARTprintf("    Please connect to a network before opening a socket.");
        UARTprintf("\n");
        return(CMDLINE_BAD_CMD);
    }
    else if(argc == 2)
    {
        if((argv[1][0] != 'u') && (argv[1][0] != 'U') &&
           (argv[1][0] != 't') && (argv[1][0] != 'T')   )
        {
            UARTprintf("    Please provide Type of Socket 'UDP' or 'TCP' .\n");
            return(CMDLINE_INVALID_ARG);
        }
    }

    //
    // Reset global socket type holder.
    //
    g_ui32SocketType = 0;

    //
    // Socket is of type UDP.
    //
    if((argv[1][0] == 'U') || (argv[1][0] == 'u'))
    {
        //
        // Open Socket.
        //
        i32Check = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

        //
        // Set global variable that holds type of socket.
        //
        g_ui32SocketType = IPPROTO_UDP;

        //
        // Inform user of the socket being opened.
        //
        UARTprintf("    Socket is of type UDP\n");
    }

    //
    // Socket is of type TCP.
    //
    else if((argv[1][0] == 't') || (argv[1][0] == 'T'))
    {
        //
        // Open Socket.
        //
        i32Check = socket(AF_INET, SOCK_DGRAM, IPPROTO_TCP);

        //
        // Set global variable that holds type of socket.
        //
        g_ui32SocketType = IPPROTO_TCP;

        //
        // Inform user of the socket being opened.
        //
        UARTprintf("    Socket is of type TCP\n");
    }

    //
    // Error Checking.
    //
    if(i32Check >= 0)
    {
        UARTprintf("    Socket Handle is '%d'\n",i32Check);
        g_ui32Socket = i32Check;
        return(0);
    }
    else
    {
        UARTprintf("    Socket Function returned an error."
                    "   Socket not opened on error code '%d'.\n",i32Check);
        g_ui32SocketType = 0;
        return(0);
    }
}

//*****************************************************************************
//
// Close the open socket.
// Arguments: None
//
//*****************************************************************************
int
CMD_socketClose(int argc, char **argv)
{
    uint32_t ui32Check;

    //
    // Shut down open socket connection for TCP
    //
    g_bSocketConnected = false;

    //
    // Close Socket
    //
    ui32Check = closesocket(g_ui32Socket);

    //
    // Error Checking
    //
    if(COMMAND_SUCCESS == ui32Check)
    {
        UARTprintf("    Socket Closed Successfully.\n");
        g_ui32Socket = SENTINEL_EMPTY;
        return(COMMAND_SUCCESS);
    }
    else
    {
        UARTprintf("    Socket Close Failed.\n");
    }
    return(0);
}

//*****************************************************************************
//
// Send Data to a destination port at a given IP Address.
// Arguments:
// [1] IP Address in the form xxx.XXX.yyy.YYY
// [2] Port as an integer 0->65536
// [3] Data to send as a string, <255bytes, no spaces
//
//*****************************************************************************
int
CMD_sendData(int argc, char **argv)
{
    int32_t i32Check = 0;
    char * pui8Data;
    uint32_t ui32DataLength, ui32Port;
    uint8_t ui8IPBlock1,ui8IPBlock2,ui8IPBlock3,ui8IPBlock4;

    //
    // Extract IP Address.
    //
    i32Check = DotDecimalDecoder(argv[1],&ui8IPBlock1,&ui8IPBlock2,&ui8IPBlock3,
                                    &ui8IPBlock4);

    //
    // Validate Input.
    //
    if(g_ui32Socket == SENTINEL_EMPTY)
    {
        UARTprintf("    Please Open a socket before tying this command.\n");
        return(CMDLINE_INVALID_ARG);
    }
    else if(argc < 4)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 4)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }
    //
    // Validate Message Size, between 1-1460 bytes.
    //
    else if( (strlen(argv[3]) > 1460) || (strlen(argv[3]) < 1) )
    {
        UARTprintf("Invalid Message, must send 1-1460 bytes\n");
        return(CMDLINE_INVALID_ARG);
    }
    //
    // Validate Port is 0->255.
    //
    else if( ustrtoul(argv[2],0,10) > (65536))
    {
        UARTprintf("    Port must be 0->65536\n");
        return(CMDLINE_INVALID_ARG);
    }
    //
    // Check return value of dot decimal converter.
    //
    else if(i32Check == COMMAND_FAIL)
    {
        UARTprintf("    Invalid IP Address. Valid IP is 0.0.0.0 -> "
                    "255.255.255.255\n");
        return(CMDLINE_INVALID_ARG);
    }

    //
    // Data pointer.
    //
    pui8Data = argv[3];

    //
    // Data length to send.
    //
    ui32DataLength = strlen(argv[3]);

    //
    // The family is always AF_INET on CC3000.
    //
    g_tSocketAddr.sa_family = AF_INET;

    //
    // The destination port.
    //
    ui32Port = ustrtoul(argv[2],0,10);
    g_tSocketAddr.sa_data[0] = (ui32Port & 0xFF00) >> 8;
    g_tSocketAddr.sa_data[1] = (ui32Port & 0x00FF) >> 0;

    //
    // The destination IP address.
    //
    g_tSocketAddr.sa_data[2] = ui8IPBlock1;
    g_tSocketAddr.sa_data[3] = ui8IPBlock2;
    g_tSocketAddr.sa_data[4] = ui8IPBlock3;
    g_tSocketAddr.sa_data[5] = ui8IPBlock4;

    //
    // Call low level send Function.
    //
    if(g_ui32SocketType == IPPROTO_UDP)
    {
        //
        // Send UDP packet.
        //
        UARTprintf("    Sending UDP Packet...\n");
        i32Check = sendto(g_ui32Socket, pui8Data, ui32DataLength, 0,
                            &g_tSocketAddr,sizeof(sockaddr));
    }
    else if(g_ui32SocketType == IPPROTO_TCP)
    {
        //
        // Connect to TCP Socket on Server (if not already conneted)
        //
        if(g_bSocketConnected == false)
        {
            UARTprintf("    Connecting to TCP Socket on Server...\n");
            i32Check = connect(g_ui32Socket, &g_tSocketAddr, sizeof(sockaddr));
            if(i32Check != 0)
            {
                UARTprintf("    Connect failed with error code '%d'\n", i32Check);
                UARTprintf("    Please make sure there is a server with the "
                           "specified socket open to connect to\n");
                return(0);
            }
            else
            {
                //
                // Socket connected successfully
                //
                g_bSocketConnected = true;
            }
        }

        //
        // Send TCP Packet.
        //
        UARTprintf("    Sending TCP Packet...\n");
        i32Check = send(g_ui32Socket, pui8Data, ui32DataLength, 0);
    }

    //
    // Validate completion of send.
    //
    if(i32Check == -1)
    {
        UARTprintf("    Send Data Failed with code '%d'\n",i32Check);
    }
    else
    {
        UARTprintf("    Send Data Success: sent %d bytes.\n", i32Check);
    }

    return(0);
}

//*****************************************************************************
//
// Receives data from the opened socket on the binded port. Prints received
// data to the terminal.
//
// Arguments: None
//
// Requires SocketOpen and Bind to have been previously called.
//
//*****************************************************************************
int
CMD_receiveData(int argc, char **argv)
{
    int32_t i32ReturnValue;
    socklen_t tRxPacketLength;
    uint32_t ui32x = 0, ui32Count = 0;
    bool bRunOnce = true;

    //
    // Validate Input.
    //
    if((g_ui32Socket == SENTINEL_EMPTY) || g_ui32BindFlag == SENTINEL_EMPTY)
    {
        UARTprintf("    Please Open a socket and Bind it to a port before "
                   "receiving data.\n");
        return(CMDLINE_BAD_CMD);
    }

    //
    // Receive UDP Data.
    //
    if(g_ui32SocketType == IPPROTO_UDP)
    {
        //
        // Tell user what we're doing.
        //
        UARTprintf("    Looking for UDP Packets...\n");

        //
        // Get all data received.  This may require multiple calls.
        //
        do
        {
            //
            // Get data.
            //
            i32ReturnValue = recvfrom(g_ui32Socket, g_pui8CC3000_Rx_Buffer,
                                      CC3000_APP_BUFFER_SIZE, 0,
                                      &g_tSocketAddr, &tRxPacketLength);

            //
            // Check data validity.
            //
            if(bRunOnce && (0 >= i32ReturnValue))
            {
                //
                // No data received on first try.
                //
                UARTprintf("    No data received.\n");
                return(0);
            }

            //
            // Print data to screen.
            //
            if(bRunOnce)
            {
                UARTprintf("    Received %d bytes of data: \n    '",
                            i32ReturnValue);
            }
            for(ui32x = 0; ui32x < i32ReturnValue; ui32x++, ui32Count++)
            {
                //
                // Add column wrapping to make output pretty
                //
                if(((ui32Count % 60) == 0) && (ui32Count > 0))
                {
                    UARTprintf("\n    ");
                }

                //
                // Print text to screen
                //
                UARTprintf("%c", g_pui8CC3000_Rx_Buffer[ui32x]);
            }
            bRunOnce = false;

        } while(i32ReturnValue == CC3000_APP_BUFFER_SIZE);
    }
    //
    // Receive TCP data.
    //
    else if(g_ui32SocketType == IPPROTO_TCP)
    {
        //
        // We've been asked to receive a TCP packet.
        //
        UARTprintf("    Looking for TCP Packets...\n");

        //
        // Get all data received.  This may require multiple calls.
        //
        do
        {
            //
            // Get Data
            //
            i32ReturnValue = recv(g_ui32Socket, g_pui8CC3000_Rx_Buffer,
                                  CC3000_APP_BUFFER_SIZE, 0);

            //
            // Check Data Validity
            //
            if(bRunOnce && (0 >= i32ReturnValue))
            {
                //
                // No data received on first try.
                //
                UARTprintf("    No data received.\n");
                return(0);
            }

            //
            // Print data to screen.
            //
            if(bRunOnce)
            {
                UARTprintf("    Received %d bytes of data: \n    '",
                            i32ReturnValue);
            }
            for(ui32x = 0; ui32x < i32ReturnValue; ui32x++, ui32Count++)
            {
                //
                // Add column wrapping to make output pretty.
                //
                if( ((ui32Count % 60) == 0) && (ui32Count > 0))
                {
                    UARTprintf("\n    ");
                }

                //
                // Print text to screen
                //
                UARTprintf("%c", g_pui8CC3000_Rx_Buffer[ui32x]);
            }
            bRunOnce = false;

        } while(i32ReturnValue == CC3000_APP_BUFFER_SIZE);
    }

    UARTprintf("'\n\n");

    return(0);
}

//*****************************************************************************
//
// Bind the open socket to a selected port.
// Arguments:
// [1] Port to Bind to
//
// Requires Socket Open before running.
//
//*****************************************************************************
int
CMD_bind(int argc, char **argv)
{
    uint32_t ui32Port = 0;
    int8_t i8Check = 0;

    //
    // Validate input.
    //
    if((ustrtoul(argv[1],0,10)) > 65536)
    {
        UARTprintf("    Invalid Port, must be 0->65536\n");
        return(CMDLINE_INVALID_ARG);
    }
    else if(g_ui32Socket == SENTINEL_EMPTY)
    {
        UARTprintf("    Socket not open, please run socketopen.\n");
        return(0);
    }
    else if(argc < 2)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 2)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }

    //
    // Family is Always AF_INET on CC3000
    //
    g_tSocketAddr.sa_family = AF_INET;

    //
    // Set the port to bind the socket to.
    //
    ui32Port = ustrtoul(argv[1],0,10);
    g_tSocketAddr.sa_data[0] = (ui32Port & 0xFF00) >> 8;
    g_tSocketAddr.sa_data[1] = (ui32Port & 0x00FF) >> 0;

    //
    // Set IP to 0.0.0.0
    //
    memset (&g_tSocketAddr.sa_data[2], 0, 4);

    //
    // Low Level API call
    //
    i8Check = bind(g_ui32Socket, &g_tSocketAddr, sizeof(sockaddr));

    if(i8Check == 0)
    {
        UARTprintf("    Bind Successful to port %d, 0x%x\n",
                   (g_tSocketAddr.sa_data[0] << 8) + g_tSocketAddr.sa_data[1],
                   (g_tSocketAddr.sa_data[0] << 8) + g_tSocketAddr.sa_data[1]);

        //
        // Set global flag variable to indicate the socket has been bound.
        //
        g_ui32BindFlag = 0;
    }
    else
    {
        UARTprintf("    Bind Failed. bind() returned code '%d'\n",i8Check);

        //
        // set global flag variable to indicate the socket is not bound.
        //
        g_ui32BindFlag = SENTINEL_EMPTY;
    }

    return(0);
}

//*****************************************************************************
//
// This command configures the IP address for the CC3000.
// This configuration can also be accomplished by running smartconfig.
// For DHCP give no arguments or specify 0 for all arguments.
// For a static IP specify the arguments
// Arguments:
// [1]  IPaddress in dot-decimal format xxx.XXX.yyy.YYY
// [2]  Default Gateway in dot-decimal format xxx.XXX.yyy.YYY
// [3]  Network Mask (optional, if not given assumed to be 255.255.255.0)
//
// To test connectivity ping the address from a computer.
//
//*****************************************************************************
int
CMD_ipConfig(int argc, char **argv)
{
    int32_t i32Check;
    uint8_t ui8IP[4], ui8Gateway[4], ui8NetMask[4];
    uint32_t ui32IP = 0, ui32Gateway = 0, ui32NetMask = 0, ui32DNS = 0;

    //
    // Validate input.
    //
    if(1 == argc)
    {
        //
        // If using DHCP, set all variables to 0.
        //
        ui8NetMask[0] = 0;
        ui8NetMask[1] = 0;
        ui8NetMask[2] = 0;
        ui8NetMask[3] = 0;

        ui8Gateway[0] = 0;
        ui8Gateway[1] = 0;
        ui8Gateway[2] = 0;
        ui8Gateway[3] = 0;

        ui8IP[0] = 0;
        ui8IP[1] = 0;
        ui8IP[2] = 0;
        ui8IP[3] = 0;

    }
    else if(argc < 3)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 4)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }
    else
    {
        //
        // Handle IP Address
        //
        i32Check = DotDecimalDecoder(argv[1], &ui8IP[0], &ui8IP[1], &ui8IP[2],
                                     &ui8IP[3]);
        if(i32Check == COMMAND_FAIL)
        {
            UARTprintf("    Invalid IP Address\n");
            return(CMDLINE_INVALID_ARG);
        }

        //
        // Handle Default Gateway
        //
        i32Check = DotDecimalDecoder(argv[2], &ui8Gateway[0], &ui8Gateway[1],
                                     &ui8Gateway[2], &ui8Gateway[3]);
        if(i32Check == COMMAND_FAIL)
        {
            UARTprintf("    Invalid Gateway Address\n");
            return(CMDLINE_INVALID_ARG);
        }

        //
        // Handle Network Mask
        //
        if(argc == 4)
        {
            //
            // If provided, fill out Network Mask
            //
            i32Check = DotDecimalDecoder(argv[3], &ui8NetMask[0],
                                         &ui8NetMask[1], &ui8NetMask[2],
                                         &ui8NetMask[3]);

            if(i32Check == COMMAND_FAIL)
            {
                UARTprintf("    Invalid Network Mask\n");
                return(CMDLINE_INVALID_ARG);
            }
        }
        else
        {
            //
            // No Network Mask given, default to 255.255.255.0
            //
            ui8NetMask[0] = 255;
            ui8NetMask[1] = 255;
            ui8NetMask[2] = 255;
            ui8NetMask[3] = 0;
        }
    }

    //
    // Fill the variables
    //
    ui32IP = ((ui8IP[3] << 24) + (ui8IP[2] << 16) +
              (ui8IP[1] << 8 ) + (ui8IP[0] << 0 ));

    ui32Gateway = ((ui8Gateway[3] << 24) + (ui8Gateway[2] << 16) +
                   (ui8Gateway[1] <<  8) + (ui8Gateway[0] <<  0));

    ui32NetMask = ((ui8NetMask[3] << 24) + (ui8NetMask[2] << 16) +
                   (ui8NetMask[1] <<  8) + (ui8NetMask[0] <<  0));

    //
    // API Call
    //
    i32Check = netapp_dhcp( (unsigned long *)&ui32IP,
                            (unsigned long *)&ui32NetMask,
                            (unsigned long *)&ui32Gateway,
                            (unsigned long *)&ui32DNS);

    //
    // Validate return for success / failure.
    //
    if(i32Check == COMMAND_SUCCESS)
    {
        UARTprintf("    IPConfig completed Successfully.\n");
    }
    else
    {
        UARTprintf("    IPConfig Failed. netapp_dhcp() returned code '%d'\n",
                    i32Check);
    }

    return(0);
}

//*****************************************************************************
//
// Disconnect from Access Point.
// Arguments: None
//
//*****************************************************************************
int
CMD_disconnect(int argc, char **argv)
{
    int32_t i32Check;

    //
    // Call API to disconnect
    //
    i32Check = wlan_disconnect();

    //
    // Check disconnection
    //
    if(i32Check == COMMAND_SUCCESS)
    {
        UARTprintf("    CC3000 Disconnected Successfully.\n");
    }
    else
    {
        UARTprintf("    CC3000 already disconnected.\n");
    }

    //
    // Turn on LED1.
    //
    turnLedOn(LED1);

    return(0);
}

//*****************************************************************************
//
// Remove the automatic connection policy from the CC3000. On reset it will no
// longer attempt to automatically connect to the Access Point.
// Arguments: None
//
//*****************************************************************************
int
CMD_deletePolicy(int argc, char **argv)
{
    int32_t i32Check;

    //
    // API Call to Disconnect
    //
    i32Check = wlan_ioctl_set_connection_policy(DISABLE, DISABLE, DISABLE);

    if(i32Check == COMMAND_SUCCESS)
    {
        UARTprintf("    Policy Deleted Successfully.\n");
    }
    else
    {
        UARTprintf("    Delete Policy failed with error code '%d'\n",i32Check);
    }

    return(0);
}

//*****************************************************************************
//
// Send an mDNS query packet.
// Arguments:
// [1] (Optional) specify the name to advertise with.
//
//*****************************************************************************
int
CMD_mdnsadvertise(int argc, char **argv)
{
    uint32_t ui32x = 0;
    int32_t i32Check = 0;

    //
    // Validate input.
    //
    if(argc < 1)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 2)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }

    //
    // Choose mDNS Name.
    //
    if(1 == argc)
    {
        //
        // Use hard coded name.
        //
        //
        UARTprintf("    mdns advertising as: '%s'...\n",g_pcdevice_name);
        for(ui32x = 0; ((ui32x < 100) && (i32Check != 0)); ui32x++)
        {
            i32Check = mdnsAdvertiser(1, g_pcdevice_name,
                                      sizeof(g_pcdevice_name));
        }

    }
    else
    {
        //
        // Use argument as name.
        //
        UARTprintf("    mdns advertising as: '%s'...\n",argv[1]);
        for(ui32x = 0; ((ui32x < 100) && (i32Check != 0)); ui32x++)
        {
            i32Check = mdnsAdvertiser(1, argv[1], strlen(argv[1]));
        }

    }
    //
    // Check return code.
    //
    if(i32Check == COMMAND_SUCCESS)
    {
        UARTprintf("    mDNS Advertised successfully.\n");
    }
    else
    {
        UARTprintf("    mDNS Advertising failed with error code '%d'\n",
                    i32Check);
    }

    return(0);
}

//*****************************************************************************
//
// Stops and then starts the CC3000 unit. Necessary to apply profiles. Useful
// for restarting sockets and other things.
//
//*****************************************************************************
int
CMD_cc3000reset(int argc, char **argv)
{

    //
    // Reset Sockets, Ports, and all connections
    //
    g_ui32BindFlag = SENTINEL_EMPTY;
    g_ui32Socket = SENTINEL_EMPTY;
    g_ui32SocketType = SENTINEL_EMPTY;

    //
    // Stop the CC3000. No return value provided.
    //
    wlan_stop();

    //
    // Wait a bit
    //
    ROM_SysCtlDelay(100000);

    //
    // Turn off LED3 and turn on LED1.
    //
    turnLedOff(LED3);
    turnLedOn(LED1);

    //
    // Start the CC3000.
    //
    wlan_start(0);
    return(0);
}

//*****************************************************************************
//
// Ping an IP Address
// Arguments:
//  [1] IP address to ping
//  [2] (optional) max number of tries
//  [3] (optional) timeout in milliseconds
//
//*****************************************************************************
int
CMD_ping(int argc, char **argv)
{
    int32_t i32Check = 0;
    uint32_t ui32Tries = 0, ui32Timeout = 0, ui32IP = 0;
    uint8_t ui8IPBlock1, ui8IPBlock2, ui8IPBlock3, ui8IPBlock4;

    //
    // Validate input.
    //
    if(argc <= 1)
    {
        return(CMDLINE_TOO_FEW_ARGS);
    }
    else if(argc > 4)
    {
        return(CMDLINE_TOO_MANY_ARGS);
    }

    //
    // Extract IP address.
    //
    i32Check = DotDecimalDecoder(argv[1], &ui8IPBlock1, &ui8IPBlock2,
                                 &ui8IPBlock3, &ui8IPBlock4);

    //
    // Validate IP address.
    //
    if(i32Check == COMMAND_FAIL)
    {
        UARTprintf("    Invalid IP Address, Please try again.\n");
        return(CMDLINE_INVALID_ARG);
    }
    else
    {
        //
        // Concatenate IP blocks together.
        //
        ui32IP = (ui8IPBlock4 << 24) + (ui8IPBlock3 << 16) +
                 ( ui8IPBlock2 << 8) + (ui8IPBlock1 << 0);

    }

    //
    // Extract the maximum number of tries.
    //
    if(argc >= 3)
    {
        ui32Tries = ustrtoul(argv[2], 0, 10);
    }
    else
    {
        ui32Tries = 4;
    }

    //
    // Extract the timeout.
    //
    if(argc == 4)
    {
        ui32Timeout = ustrtoul(argv[3], 0, 10);
    }
    else
    {
        ui32Timeout = 500;
    }

    //
    // Notify user of settings.
    //
    UARTprintf("    Pinging %d.%d.%d.%d, 0x%x, Max Tries: %d, Timeout: %dms...",
                ui8IPBlock1, ui8IPBlock2, ui8IPBlock3, ui8IPBlock4, ui32IP,
                ui32Tries, ui32Timeout);

    //
    // Send Ping request to CC3000. Set buffer length to 255.
    //
    i32Check = netapp_ping_send((unsigned long *)&ui32IP, ui32Tries, 255,
                                ui32Timeout);

    //
    // Validate that the Ping completed successfully
    //
    if(i32Check != 0)
    {
        UARTprintf("    Ping request failed with error code: '%d'\n", i32Check);
    }

    return(0);
}

//*****************************************************************************
//
// main loop
//
//*****************************************************************************
int
main(void)
{
    int32_t i32CommandStatus;

    g_ui32CC3000DHCP = 0;
    g_ui32CC3000Connected = 0;
    g_ui32Socket = SENTINEL_EMPTY;
    g_ui32BindFlag = SENTINEL_EMPTY;
    g_ui32SmartConfigFinished = 0;

    //
    // Initialize all board specific components.
    //
    initDriver();

    //
    // Loop forever waiting  for commands from PC...
    //
    while(1)
    {
        //
        // Print prompt for user.
        //
        UARTprintf("\n>");

        //
        // Peek to see if a full command is ready for processing
        //
        while(UARTPeek('\r') == -1)
        {
            //
            // 1 millisecond delay.
            //
            ROM_SysCtlDelay(g_ui32SysClock / 3000);
        }

        //
        // A '\r' was detected get the line of text from the user.
        //
        UARTgets(g_cInput,sizeof(g_cInput));

        //
        // Pass the line from the user to the command processor.
        // It will be parsed and valid commands executed.
        //
        i32CommandStatus = CmdLineProcess(g_cInput);

        //
        // Handle the case of bad command.
        //
        if(i32CommandStatus == CMDLINE_BAD_CMD)
        {
            UARTprintf("    Bad command. Try again.\n");
        }
        //
        // Handle the case of too many arguments.
        //
        else if(i32CommandStatus == CMDLINE_TOO_MANY_ARGS)
        {
            UARTprintf("    Too many arguments for command. Try again.\n");
        }
        //
        // Handle the case of too few arguments.
        //
        else if(i32CommandStatus == CMDLINE_TOO_FEW_ARGS)
        {
            UARTprintf("    Too few arguments for command. Try again.\n");
        }
        //
        // Handle the case of too few arguments.
        //
        else if(i32CommandStatus == CMDLINE_INVALID_ARG)
        {
            UARTprintf("    Invalid command argument(s). Try again.\n");
        }

        //
        // Complete smart config process:
        // 1. if smart config is done
        // 2. CC3000 established AP connection
        // 3. DHCP IP is configured
        // then send mDNS packet to stop external SmartConfig application
        //
        if((g_ui8StopSmartConfig == 1) && (g_ui32CC3000DHCP == 1) &&
           (g_ui32CC3000Connected == 1))
        {
            unsigned char loop_index = 0;

            while(loop_index < 3)
            {
                mdnsAdvertiser(1, g_pcdevice_name, sizeof(g_pcdevice_name));
                loop_index++;
            }

            g_ui8StopSmartConfig = 0;
        }
    }
}
