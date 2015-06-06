//*****************************************************************************
//
// bl_i2c.h - Definitions for the I2C transport functions.
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

#ifndef __BL_I2C_H__
#define __BL_I2C_H__

//*****************************************************************************
//
// This defines the I2C clock pin that is being used by the boot loader.
//
//*****************************************************************************
#define I2C_CLK                 (1 << 2)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define I2C_CLK_PCTL            (0x2 << 8)
#else
#define I2C_CLK_PCTL            (0x3 << 8)
#endif

//*****************************************************************************
//
// This defines the I2C data pin that is being used by the boot loader.
//
//*****************************************************************************
#define I2C_DATA                (1 << 3)
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
	defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
#define I2C_DATA_PCTL           (0x2 << 12)
#else
#define I2C_DATA_PCTL           (0x3 << 12)
#endif

//*****************************************************************************
//
// This defines the combination of pins used to implement the I2C port used by
// the boot loader.
//
//*****************************************************************************
#define I2C_PINS                (I2C_CLK | I2C_DATA)
#define I2C_PINS_PCTL           (I2C_CLK_PCTL | I2C_DATA_PCTL)

//*****************************************************************************
//
// I2C Transport APIs
//
//*****************************************************************************
extern void I2CSend(const uint8_t *pui8Data, uint32_t ui32Size);
extern void I2CReceive(uint8_t *pui8Data, uint32_t ui32Size);
extern void I2CFlush(void);

//*****************************************************************************
//
// Define the transport functions if the I2C port is being used.
//
//*****************************************************************************
#ifdef I2C_ENABLE_UPDATE
#define SendData                I2CSend
#define FlushData               I2CFlush
#define ReceiveData             I2CReceive
#endif

#endif // __BL_I2C_H__
