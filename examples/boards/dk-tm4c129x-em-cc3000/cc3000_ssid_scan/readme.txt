CC3000 WiFi Access Point SSID Scanning Example

This example requires a CC3000 WiFi BoosterPack attached to the
the DK-TM4C129X Development Kit.  After booting and initializing the
CC3000, the application initiates a WiFi scan for access points. When
the scan completes, the SSID, BSSID and security protocol supported by
each detected access point are shown.  Another scan can be started by
pressing the ``Scan'' button on the touchscreen display.

By default, the application is built to support a CC3000 Evaluation
Module connected to the EM connector of the DK-TM4C129X board.

The application may be rebuilt to support a CC3000 BoosterPack connected to
the board's BoosterPack 1 or BoosterPack 2 connector or a CC3000 EM
connected to the board's EM connectors by setting one of the labels
``CC3000_USE_BOOSTERPACK1'',  ``CC3000_USE_BOOSTERPACK2'' or
``CC3000_USE_EM'' in the build environment.  When connecting the CC3000 to
BoosterPack2, jumpers J16 and J17 must be moved to ensure that SPI signals
(rather than I2C) are routed to the appropriate BoosterPack connector pins.

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

This is part of revision 2.1.1.71 of the DK-TM4C129X Firmware Package.
