//*****************************************************************************
//
// bl_ssi.h - Definitions for the SSI transport functions.
//
// Copyright (c) 2006-2015 Texas Instruments Incorporated.  All rights reserved.
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

#ifndef __BL_SSI_H__
#define __BL_SSI_H__

//*****************************************************************************
//
// This is the number of bits per transfer for SSI.  This is a constant and
// cannot be changed without corresponding code changes.
//
//*****************************************************************************
#define DATA_BITS_SSI           8

//*****************************************************************************
//
// This defines the SSI chip select pin that is being used by the boot loader.
//
//*****************************************************************************
#define SSI_CS                  (1 << 3)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define SSI_CS_PCTL             (0xF << 8)
#else
#define SSI_CS_PCTL             (0x2 << 8)
#endif

//*****************************************************************************
//
// This defines the SSI clock pin that is being used by the boot loader.
//
//*****************************************************************************
#define SSI_CLK                 (1 << 2)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define SSI_CLK_PCTL            (0xF << 8)
#else
#define SSI_CLK_PCTL            (0x2 << 8)
#endif


//*****************************************************************************
//
// This defines the SSI transmit pin that is being used by the boot loader.
//
//*****************************************************************************
#define SSI_TX                  (1 << 5)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define SSI_TX_PCTL             (0xF << 20)
#else
#define SSI_TX_PCTL             (0x2 << 20)
#endif

//*****************************************************************************
//
// This defines the SSI receive pin that is being used by the boot loader.
//
//*****************************************************************************
#define SSI_RX                  (1 << 4)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define SSI_RX_PCTL             (0xF << 16)
#else
#define SSI_RX_PCTL             (0x2 << 16)
#endif

//*****************************************************************************
//
// This defines the combination of pins used to implement the SSI port used by
// the boot loader.
//
//*****************************************************************************
#define SSI_PINS                (SSI_CLK | SSI_TX | SSI_RX | SSI_CS)
#define SSI_PINS_PCTL           (SSI_CLK_PCTL | SSI_TX_PCTL | SSI_RX_PCTL | SSI_CS_PCTL)

//*****************************************************************************
//
// SSI Transport APIs
//
//*****************************************************************************
extern void SSISend(const uint8_t *pui8Data, uint32_t ui32Size);
extern void SSIReceive(uint8_t *pui8Data, uint32_t ui32Size);
extern void SSIFlush(void);

//*****************************************************************************
//
// Define the transport functions if the SSI port is being used.
//
//*****************************************************************************
#ifdef SSI_ENABLE_UPDATE
#define SendData                SSISend
#define FlushData               SSIFlush
#define ReceiveData             SSIReceive
#endif

#endif // __BL_SSI_H__
