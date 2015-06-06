//*****************************************************************************
//
// board.h - board definitions for CC3000 on DK-TM4C129X
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
// This is part of revision 2.1.1.71 of the Tiva Firmware Development Package.
//
//*****************************************************************************
#ifndef __BOARD_H__
#define __BOARD_H__

//*****************************************************************************
//
// Global holding the system clock rate in Hz.
//
//*****************************************************************************
extern uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICK_PER_SECOND            10

//*****************************************************************************
//
// The desired system clock rate.
//
//*****************************************************************************
#define SYSCLOCK_RATE_HZ        60000000

//*****************************************************************************
//
// The parameter to pass to SysCtlDelay() to cause a 50uS delay.
//
//*****************************************************************************
#define DELAY_50_MICROSECONDS   ((SYSCLOCK_RATE_HZ / 20000) / 3)

//*****************************************************************************
//
// Make sure one or other boosterpack header has been selected in the build
// environment.
//
//*****************************************************************************
#if (!(defined CC3000_USE_BOOSTERPACK1) && !(defined CC3000_USE_BOOSTERPACK2) \
    && !(defined CC3000_USE_EM))
#error Please define one of CC3000_USE_BOOSTERPACK1 or CC3000_USE_BOOSTERPACK2 \
        or CC3000_USE_EM
#endif

//*****************************************************************************
//
// SPI General Defines & Macros
//
//*****************************************************************************
#define SPI_WINDOW_SIZE         DMA_WINDOW_SIZE
#define FWT_DELAY               4000
#define DMA_WINDOW_SIZE         1024
#define ASSERT_CS()          (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0))
#define DEASSERT_CS()        (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0xFF))

//*****************************************************************************
//
// SPI Board Specific Defines. To select which booster pack will be used
// make sure to define one of these in your project deffinitions.
//
//*****************************************************************************

//
// Use EM headers.
//
#if defined(CC3000_USE_EM)
//
// IRQ settings
//
#define SYSCTL_PERIPH_IRQ_PORT      SYSCTL_PERIPH_GPIOD
#define SPI_GPIO_IRQ_BASE           GPIO_PORTD_BASE
#define SPI_IRQ_PIN                 GPIO_PIN_3
#define INT_GPIO_SPI                INT_GPIOD
#define INT_SPI                     INT_SSI0

//
// SW EN settings
//
#define SYSCTL_PERIPH_SW_EN_PORT    SYSCTL_PERIPH_GPIOD
#define SPI_GPIO_SW_EN_BASE         GPIO_PORTD_BASE
#define SPI_EN_PIN                  GPIO_PIN_2

//
// CS settings
//
#define SPI_CS_PORT                 GPIO_PORTA_BASE
#define SYSCTL_PERIPH_SPI_PORT      SYSCTL_PERIPH_GPIOA
#define SPI_CS_PIN                  GPIO_PIN_3

//
// SPI Hardware Abstraction layer
//
#define SPI_BASE                    SSI0_BASE
#define SYSCTL_PERIPH_SPI           SYSCTL_PERIPH_SSI0
#define SYSCTL_PERIPH_SPI_BASE      SYSCTL_PERIPH_GPIOA
#define SPI_PORT                    GPIO_PORTA_BASE
#define SPI_CLK_PIN                 GPIO_PIN_2
#define SPI_CLK_MUX_SEL             GPIO_PA2_SSI0CLK
#define SPI_RX_PIN                  GPIO_PIN_5
#define SPI_RX_MUX_SEL              GPIO_PA5_SSI0XDAT1
#define SPI_TX_PIN                  GPIO_PIN_4
#define SPI_TX_MUX_SEL              GPIO_PA4_SSI0XDAT0
#define SPI_UDMA_RX_CHANNEL         UDMA_CH10_SSI0RX
#define SPI_UDMA_TX_CHANNEL         UDMA_CH11_SSI0TX

#else
//
// Use BoosterPack 1 Headers.
//
#if defined(CC3000_USE_BOOSTERPACK1)

//
// IRQ settings
//
#define SYSCTL_PERIPH_IRQ_PORT      SYSCTL_PERIPH_GPIOS
#define SPI_GPIO_IRQ_BASE           GPIO_PORTS_BASE
#define SPI_IRQ_PIN                 GPIO_PIN_2
#define INT_GPIO_SPI                INT_GPIOS
#define INT_SPI                     INT_SSI2

//
// SW EN settings
//
#define SYSCTL_PERIPH_SW_EN_PORT    SYSCTL_PERIPH_GPIOE
#define SPI_GPIO_SW_EN_BASE         GPIO_PORTE_BASE
#define SPI_EN_PIN                  GPIO_PIN_2

//
// CS settings
//
#define SPI_CS_PORT                 GPIO_PORTQ_BASE
#define SYSCTL_PERIPH_SPI_PORT      SYSCTL_PERIPH_GPIOQ
#define SPI_CS_PIN                  GPIO_PIN_7

//
// SPI Hardware Abstraction layer
//
#define SPI_BASE                    SSI2_BASE
#define SYSCTL_PERIPH_SPI           SYSCTL_PERIPH_SSI2
#define SYSCTL_PERIPH_SPI_BASE      SYSCTL_PERIPH_GPIOG
#define SPI_PORT                    GPIO_PORTG_BASE
#define SPI_CLK_PIN                 GPIO_PIN_7
#define SPI_CLK_MUX_SEL             GPIO_PG7_SSI2CLK
#define SPI_RX_PIN                  GPIO_PIN_4
#define SPI_RX_MUX_SEL              GPIO_PG4_SSI2XDAT1
#define SPI_TX_PIN                  GPIO_PIN_5
#define SPI_TX_MUX_SEL              GPIO_PG5_SSI2XDAT0
#define SPI_UDMA_RX_CHANNEL         UDMA_CH12_SSI2RX
#define SPI_UDMA_TX_CHANNEL         UDMA_CH13_SSI2TX

#else
//
// Use BoosterPack 2 Headers.
//
#if defined(CC3000_USE_BOOSTERPACK2)
//
// IRQ settings
//
#define SYSCTL_PERIPH_IRQ_PORT      SYSCTL_PERIPH_GPIOD
#define SPI_GPIO_IRQ_BASE           GPIO_PORTD_BASE
#define SPI_IRQ_PIN                 GPIO_PIN_1
#define INT_GPIO_SPI                INT_GPIOD
#define INT_SPI                     INT_SSI0

//
// SW EN settings
//
#define SYSCTL_PERIPH_SW_EN_PORT    SYSCTL_PERIPH_GPIOD
#define SPI_GPIO_SW_EN_BASE         GPIO_PORTD_BASE
#define SPI_EN_PIN                  GPIO_PIN_0

//
// CS settings
//
#define SPI_CS_PORT                 GPIO_PORTJ_BASE
#define SYSCTL_PERIPH_SPI_PORT      SYSCTL_PERIPH_GPIOJ
#define SPI_CS_PIN                  GPIO_PIN_3

//
// SPI Hardware Abstraction layer
//
#define SPI_BASE                    SSI0_BASE
#define SYSCTL_PERIPH_SPI           SYSCTL_PERIPH_SSI0
#define SYSCTL_PERIPH_SPI_BASE      SYSCTL_PERIPH_GPIOA
#define SPI_PORT                    GPIO_PORTA_BASE
#define SPI_CLK_PIN                 GPIO_PIN_2
#define SPI_CLK_MUX_SEL             GPIO_PA2_SSI0CLK
#define SPI_RX_PIN                  GPIO_PIN_5
#define SPI_RX_MUX_SEL              GPIO_PA5_SSI0XDAT1
#define SPI_TX_PIN                  GPIO_PIN_4
#define SPI_TX_MUX_SEL              GPIO_PA4_SSI0XDAT0
#define SPI_UDMA_RX_CHANNEL         UDMA_CH10_SSI0RX
#define SPI_UDMA_TX_CHANNEL         UDMA_CH11_SSI0TX

#endif
#endif
#endif //end of CC3000_USE_BOOSTERPACKX

//*****************************************************************************
//
// UART defines
//
//*****************************************************************************
#define CC3000_UART_BASE                UART0_BASE
#define CC3000_UART_SYSCTL_PERIPH_GPIO  SYSCTL_PERIPH_GPIOA
#define CC3000_UART_SYSCTL_PERIPH_UART  SYSCTL_PERIPH_UART0
#define CC3000_UART_GPIO_RX             GPIO_PA0_U0RX
#define CC3000_UART_GPIO_TX             GPIO_PA1_U0TX
#define CC3000_UART_GPIO_PORT_BASE      GPIO_PORTA_BASE
#define CC3000_UART_GPIO_PINS           (GPIO_PIN_0 | GPIO_PIN_1)
#define CC3000_UART_INT                 INT_UART0

//*****************************************************************************
//
// LED defines. Note that on DK-TM4C129X, we can't use the green LED in the
// tricolor LED when using the BoosterPack1 connector because this GPIO is
// shared with the CS line on the XL connector.  In this case, we use the
// green Ethernet LED on PK6 instead.
//
//*****************************************************************************
#define CC3000_LED_RED_SYSCTL_PERIPH    SYSCTL_PERIPH_GPION
#define CC3000_LED_RED_PORT             GPIO_PORTN_BASE
#define CC3000_LED_RED_PIN              GPIO_PIN_5

#define CC3000_LED_BLUE_SYSCTL_PERIPH   SYSCTL_PERIPH_GPIOQ
#define CC3000_LED_BLUE_PORT            GPIO_PORTQ_BASE
#define CC3000_LED_BLUE_PIN             GPIO_PIN_4

#if defined(CC3000_USE_BOOSTERPACK1)
#define CC3000_LED_GREEN_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOK
#define CC3000_LED_GREEN_PORT           GPIO_PORTK_BASE
#define CC3000_LED_GREEN_PIN            GPIO_PIN_6
#else
#define CC3000_LED_GREEN_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOQ
#define CC3000_LED_GREEN_PORT           GPIO_PORTQ_BASE
#define CC3000_LED_GREEN_PIN            GPIO_PIN_7
#endif

typedef enum
{
    RED_LED,
    GREEN_LED,
    BLUE_LED
}
tBoardLED;

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
extern void pio_init(void);
extern void initLEDs(void);
extern long ReadWlanInterruptPin(void);
extern void WlanInterruptEnable(void);
extern void WlanInterruptDisable(void);
extern void WriteWlanPin( unsigned char val );
extern void InitSysTick(void);
extern void SysTickHandler(void);
extern void initClk(void);
extern void turnLedOn(tBoardLED eLED);
extern void turnLedOff(tBoardLED eLED);

#endif //__BOARD_H__
