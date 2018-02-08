//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2011-2016 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.3.156 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************
// This is a changed line!!!!!!!!
// This is another changed line!
// Base includes with the timers Examples
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"

// Necessary for UART
#include "driverlib/uart.h"

// Necessary to write data to a temporary character buffer to ultimately display
#include <stdio.h>

// Necessary for ADC
#include "driverlib/adc.h"


#define ADC_SEQUENCE_3 3

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator on the display.
//
//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the CSTN display.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
tContext g_sContext;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ui32Flags, 0) ^= 1;

    //
    // Update the interrupt status on the display.
    //
    IntMasterDisable();
    GrStringDraw(&g_sContext, (HWREGBITW(&g_ui32Flags, 0) ? "1" : "0"), -1, 68,
                 26, 1);
    IntMasterEnable();
}

//*****************************************************************************
//
// The interrupt handler for the second timer interrupt.
//
//*****************************************************************************
void
Timer1IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the second timer.
    //
    HWREGBITW(&g_ui32Flags, 1) ^= 1;

    //
    // Update the interrupt status on the display.
    //
    IntMasterDisable();
    GrStringDraw(&g_sContext, (HWREGBITW(&g_ui32Flags, 1) ? "1" : "0"), -1, 68,
                 36, 1);
    IntMasterEnable();
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int
main(void)
{
    tRectangle sRect;

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();

    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //************************************************************************
    // Disabling all peripherals
    //************************************************************************
    IntMasterDisable();

    // Initialize the display driver.
    CFAL96x64x16Init();

    // Initialize the graphics context and find the middle X coordinate.
    GrContextInit(&g_sContext, &g_sCFAL96x64x16);

    //************************************************************************
    // Enabling the peripherals
    //************************************************************************
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);      // For LED

    //*************************************************************************
    // Checking if the peripheral is turned on
    //*************************************************************************
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1) ||
            !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG));

    //*************************************************************************
    // Configuration
    //*************************************************************************
    // UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    // Configuring the ADC to read from the ADC0_BASE, 3rd sequence,
    // trigger processing, and to take priority
    ADCSequenceConfigure(ADC0_BASE, ADC_SEQUENCE_3, ADC_TRIGGER_PROCESSOR, 0);

    // Configuring the ADCSequence to read from the 0 step
    ADCSequenceStepConfigure(ADC0_BASE, ADC_SEQUENCE_3, 0, ADC_CTL_CH7 |
                                              ADC_CTL_IE | ADC_CTL_END);

    // Enable the GPIO pin for the LED (PG2).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);

    // Configuring the two 32-bit periodic timers.
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / 2);

    //************************************************************************
    // Clearing Interrupt Flags and triggering ADC sequence conversion
    //************************************************************************
    ADCIntClear(ADC0_BASE, ADC_SEQUENCE_3);
    ADCProcessorTrigger(ADC0_BASE, ADC_SEQUENCE_3);

    //************************************************************************
    // Enable processor interrupts.
    //************************************************************************
    IntMasterEnable();


    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);

    //
    // Loop forever while the timers run.
    //
    while(1)
    {
    }
}
