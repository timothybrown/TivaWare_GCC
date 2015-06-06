//*****************************************************************************
//
// cc3000lib.c - CC3000 library includes.
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
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __CC3000LIB_C__
#define __CC3000LIB_C__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
//
//
//
//*****************************************************************************
#include "board.h"

//*****************************************************************************
//
// Host Driver Includes -
//
//*****************************************************************************
#include "cc3000/src/host_driver/core_driver/cc3000_common.c"
#include "cc3000/src/host_driver/core_driver/evnt_handler.c"
#include "cc3000/src/host_driver/core_driver/hci.c"
#include "cc3000/src/host_driver/core_driver/netapp.c"
#include "cc3000/src/host_driver/core_driver/nvmem.c"
#include "cc3000/src/host_driver/core_driver/security.c"
#include "cc3000/src/host_driver/core_driver/socket.c"
#include "cc3000/src/host_driver/core_driver/wlan.c"

//*****************************************************************************
//
// SPI Includes
//
//*****************************************************************************
#ifdef CC3000_TM4C123_SPI
#include "cc3000/src/spi/spi-tm4c123.c"
#else
#ifdef CC3000_TM4C129_SPI
#include "cc3000/src/spi/spi-tm4c129.c"
#else
#error One of CC3000_TM4C129_SPI or CC3000_TM4C123_SPI must be defined!
#endif
#endif
#include "cc3000/src/spi/spi.h"
#include "cc3000/src/spi/spi_version.h"

//*****************************************************************************
//
// Dispatcher (UART) includes
//
//*****************************************************************************
#include "cc3000/src/uart/dispatcher.c"
#include "cc3000/src/uart/dispatcher.h"

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif
#endif // __CC3000LIB_C__
