CC3000 WiFi Access Point SSID Scanning Example

This example requires a CC3000 WiFi BoosterPack attached to the
EK-TM4C123GXL LaunchPad.  After booting and initializing the CC3000, the
application initiates a WiFi scan for access points.  When the scan
completes, the SSID, BSSID and security protocol supported by each detected
access point are output on the UART0 connection available over the virtual
COM port connection provided by the board's ICDI debug interface.

To view output from the application, set your host system's serial terminal
to use 115200bps, 8-N-1.

For information on the CC3000 software stack and API, please consult the
wiki at http://processors.wiki.ti.com/index.php/CC3000.

-------------------------------------------------------------------------------

Copyright (c) 2013-2015 Texas Instruments Incorporated.  All rights reserved.
Software License Agreement

Texas Instruments (TI) is supplying this software for use solely and
exclusively on TI's microcontroller products. The software is owned by
TI and/or its suppliers, and is protected under applicable copyright
laws. You may not combine this software with "viral" open-source
software in order to form a larger program.

THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, FOR ANY REASON WHATSOEVER.

This is part of revision 2.1.1.71 of the EK-TM4C123GXL Firmware Package.
