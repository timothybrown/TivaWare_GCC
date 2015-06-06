CC3000 Firmware Patch Programmer

This is the Patch Programmer tool for the CC3000 BoosterPack running on an
EK-TM4C123GXL LaunchPad.  Run the application to download new firmware
and driver patches to the CC3000 processor.  When run the LED on the board
will glow red for up to 10 seconds, then the led will turn blue when the
firmware update is complete.  Status is also output via UART0 which
is available via the virtual COM port provided by the ICDI debug interface.

Two patches are downloaded using this tool with the patch data is linked
directly into the application binary.  The driver patch can be found
in an array named ``wlan_drv_patch'' and the firmware patch can be found
in ``fw_patch''.  When new patches are available, these arrays must be
replaced with versions containing those new patches and then the
application rebuilt and run to apply the patches to the CC3000 hardware.

To view output from the application, set your host system's serial terminal
to use 115200bps, 8-N-1.

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

This is part of revision 2.1.1.71 of the EK-TM4C123GXL Firmware Package.
