CC3000 Firmware Patch Programmer

This is the Patch Programmer tool for the CC3000 BoosterPack running on an
DK-TM4C129X Development Kit.  Run the application to download new firmware
and driver patches to the CC3000 processor.  Status is output on the LCD
display and also via UART0 which is available via the virtual COM port
provided by the ICDI debug interface.

Two patches are downloaded using this tool with the patch data is linked
directly into the application binary.  The driver patch can be found
in an array named ``wlan_drv_patch'' and the firmware patch can be found
in ``fw_patch''.  When new patches are available, these arrays must be
replaced with versions containing those new patches and then the
application rebuilt and run to apply the patches to the CC3000 hardware.

To view UART0 output from the application, set your host system's serial
terminal to use 115200bps, 8-N-1.

By default, the application is built to support a CC3000 Evaluation
Module connected to the EM connector of the DK-TM4C129X board.

The application may be rebuilt to support a CC3000 BoosterPack connected to
the board's BoosterPack 1 or BoosterPack 2 connector, or for a CC3000
Evaluation Module (EM) connected to the board's EM connectors by setting
one of the labels ``CC3000_USE_BOOSTERPACK1'',  ``CC3000_USE_BOOSTERPACK2''
or ``CC3000_USE_EM'' in the build environment.  When connecting the CC3000
to BoosterPack2, jumpers J16 and J17 must be moved to ensure that SPI
signals (rather than I2C) are routed to the appropriate BoosterPack
connector pins.

For information on the CC3000 software stack and API, please consult the
wiki at http://processors.wiki.ti.com/index.php/CC3000.

-------------------------------------------------------------------------------

Copyright (c) 2014-2015 Texas Instruments Incorporated.  All rights reserved.
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
