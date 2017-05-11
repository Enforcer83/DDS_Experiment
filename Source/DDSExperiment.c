//************************************************************************************
//
// Title:               Modular Test Program for DDSExperiment
// Author:              Jacob Putz
// Filename:            DDSExperimentTest.c
//
// Description:     This program is to be used to incrementally compile/test/run
//                      each stage of the master DDSExperiment.c
//
// Current Revision:    0.1.1
//
// TivaWare:            2.1.4.178
//
// MIT License
// Copyright (c)    2017    Integrated Microsystem Electronics, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in the
// Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
// PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
// SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Revision History:
//
// 0.1.1    -       Implement SysTick timer, interrupt and millis() function.
//
// 0.1.0    -       Example project “blinky” with minor modifications.
//
//************************************************************************************

// Includes
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

// Defines
//
// GPIO Base Defines
#define     PORTF               GPIO_PORTF_BASE
#define     PORTN               GPIO_PORTN_BASE

// GPIO Defines
#define     LED1                GPIO_PIN_1  //Port N Pin 1
#define     LED2                GPIO_PIN_0  //Port N Pin 0
#define     LED3                GPIO_PIN_4  //Port F Pin 4
#define     LED4                GPIO_PIN_0  //Port F Pin 0

// Miscellaneous Defines
#define     LHALF               0x0F
#define     UHALF               0xF0
#define     ALL                 0xFF
#define     OFF                 0x00
#define     HIGH                0x1
#define     LOW                 0x0

// Global Constants
const uint32_t SYS_CLK_REQ = 0x07270E00;    // 120 MHz

// Global Variables
uint32_t SYS_CLK_ACT = 0;           // Actual clock frequency obtained by PLL

volatile uint64_t COUNT = 0;        // Stores how many times Interrupt handler called

//************************************************************************************
//
// Notes
//
//  Binary      Hex         Binary      Hex
//
//  0000        0x0         1000        0x8
//  0001        0x1         1001        0x9
//  0010        0x2         1010        0xA
//  0011        0x3         1011        0xB
//  0100        0x4         1100        0xC
//  0101        0x5         1101        0xD
//  0110        0x6         1110        0xE
//  0111        0x7         1111        0xF
//
//    signed char         int8_t;
//  unsigned char        uint8_t;
//           short       int16_t;
//  unsigned short      uint16_t;
//           int         int32_t;
//  unsigned int        uint32_t;
//           long long   int64_t;
//  unsigned long long  uint64_t;
//
//************************************************************************************

void SysTickHandler() {

    //
    // The SysTick Timer is configured as a millisecond timer.  An unsigned 64 bit
    // number will have a maximum value of 18,446,744,073,709,551,615.  It will take
    // just over 584,542,046 years (2^64/(1000*60*60*24*365.25)) of constant running to
    // fill this variable and nearly 50 days of constant running to fill a 32 bit
    // number.
    //
    COUNT++;

};

uint64_t millis () {

    return COUNT;

};

void main() {

    //
    // Local Variables
    //
    uint64_t led1OnTime = 750, led1OffTime = 750, led2OnTime = 100, led2ShortOff = 100;
    uint64_t led2LongOff = 700, led3OnTime = 250, led3OffTime = 750;

    volatile uint32_t pulse = 0;

    volatile uint64_t led1PreviousMillis = 0, led2PreviousMillis = 0;
    volatile uint64_t led3PreviousMillis = 0;

    volatile bool led1On = false, led2On = false, led3On = false;

    //
    // Use external 25MHz Precision Oscillator to Generate an 120MHz System
    // Clock using the PLL
    //
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    SYS_CLK_ACT = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                        SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, SYS_CLK_REQ);

    //
    // Enable Peripherals
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    //
    // Configure the SysTick Timer.
    //
    SysTickPeriodSet(SYS_CLK_ACT / 1000);
    SysTickIntRegister(SysTickHandler);
    SysTickEnable();
    SysTickIntEnable();

    if(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {

        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {

            // Wait for the peripheral to become ready.

        }

    }

    //
    // Configure GPIO Type
    //
    GPIOPinTypeGPIOOutput(PORTF, LED3 | LED4);
    GPIOPinTypeGPIOOutput(PORTN, LED1 | LED2);

    //
    // Set GPIO Direction
    //
    GPIODirModeSet(PORTF, LED3 | LED4, GPIO_DIR_MODE_OUT);
    GPIODirModeSet(PORTN, LED1 | LED2, GPIO_DIR_MODE_OUT);

    //
    // Configure GPIO Pad Properties
    //
    GPIOPadConfigSet(PORTF, LED3 | LED4, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(PORTN, LED1 | LED2, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    //
    // Enter forever loop
    //
    while(1) {

        uint64_t currentMillis = millis();

        if (led1On && (currentMillis - led1PreviousMillis >= led1OnTime)) {

            led1On = false;
            led1PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED1, 0x0);

        }

        else if (!led1On && (currentMillis - led1PreviousMillis >= led1OffTime)) {

            led1On = true;
            led1PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED1, LED1);

        }

        if (led3On && (currentMillis - led3PreviousMillis >= led3OnTime)) {

            led3On = false;
            led3PreviousMillis = currentMillis;
            GPIOPinWrite(PORTF, LED3, 0x0);

        }

        else if (!led3On && (currentMillis - led3PreviousMillis >= led3OffTime)) {

            led3On = true;
            led3PreviousMillis = currentMillis;
            GPIOPinWrite(PORTF, LED3, LED3);

        }

        if (led2On && (pulse == 1) && (currentMillis - led2PreviousMillis >= led2OnTime)) {

            led2On = false;
            led2PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED2, 0x0);

        }

        else if (led2On && (pulse == 2) && (currentMillis - led2PreviousMillis >= led2OnTime)) {

            led2On = false;
            pulse = 0;
            led2PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED2, 0x0);

        }

        else if (!led2On && (pulse == 0) && (currentMillis - led2PreviousMillis >= led2LongOff)) {

            led2On = true;
            pulse = 1;
            led2PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED2, LED2);

        }

        else if (!led2On && (pulse == 1) && (currentMillis - led2PreviousMillis >= led2ShortOff)) {

            led2On = true;
            pulse = 2;
            led2PreviousMillis = currentMillis;
            GPIOPinWrite(PORTN, LED2, LED2);

        }

    }

}
