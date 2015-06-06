# TivaWare_GCC
The full TivaWare C Series 2.1.1.71 (May 2015) library, ready to use on OS X, BSD and Linux platforms. Just add a compiler (arm-none-eabi-gcc) and a flashing tool (lmflash, dfu-util).

This package is the same as what's available from TI, only with all CCS, Keil, Sourcery and other non-GCC platform files removed. GCC build files were added were there were none, or changed where they were broken.
Windows-specific drivers and tools have also been removed and CLI tool source modified to build on *NIX systems.
Tested under OS X Yosemite and verified working!
