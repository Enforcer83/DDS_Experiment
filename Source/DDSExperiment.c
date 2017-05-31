//**********************************************************************************************************
//
// Title:               Modular Test Program for DDSExperiment
// Author:          Jacob Putz
// Filename:            DDSExperimentTest.c
//
// Description:     This program is to be used to incrementally compile/test/run each stage of the master
//                  DDSExperiment.c
//
// Current Revision:    0.1.1
//
// TivaWare:            2.1.4.178
//
// MIT License
// Copyright (c)    2017    Integrated Microsystem Electronics, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
// following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
// LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO
// EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
// THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Revision History:
//
// 0.1.2    -   Implement Revision 0.1.1 as a C++ Object Oriented Program using class definitions
//
// 0.1.1    -   Implement SysTick timer, interrupt and millis() function.
//
// 0.1.0    -   Example project “blinky” with minor modifications.
//
//**********************************************************************************************************

// Includes
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/rom.h"
#include "driverlib/ssi.h"
#include "driverlib/fpu.h"
//#include "driverlib/rom_map.h"
//#include  ""

#ifdef __cplusplus
extern “C”{
#endif

// Defines
//
// GPIO Base Defines
#define PORTD		GPIO_PORTD_AHB_BASE
#define PORTF       GPIO_PORTF_AHB_BASE
#define PORTK       GPIO_PORTK_BASE
#define PORTM       GPIO_PORTM_BASE
#define PORTN       GPIO_PORTN_BASE

// GPIO Defines
#define	LED1		GPIO_PIN_1			// Port N Pin 1	O
#define	LED2		GPIO_PIN_0			// Port N Pin 0	O
#define	DRST0		GPIO_PIN_0			// Port K Pin 0	O
#define	SLEEP0		GPIO_PIN_1			// Port K Pin 1	O
#define	FSEL0		GPIO_PIN_2			// Port K Pin 2	O
#define	PSEL0		GPIO_PIN_3			// Port K Pin 3	O
#define	NCS_SEL0	GPIO_PIN_0			// Port M Pin 0	O
#define	NCS_SEL1	GPIO_PIN_1			// Port M Pin 1	O
#define	NCS_SEL2	GPIO_PIN_2			// Port M Pin 2	O
#define	NCS_INH1	GPIO_PIN_3			// Port M Pin 3	O
#define	NCS_SEL3	GPIO_PIN_4			// Port M Pin 4	O
#define	NCS_SEL4	GPIO_PIN_5			// Port M Pin 5	O
#define	NCS_SEL5	GPIO_PIN_6			// Port M Pin 6	O
#define	NCS_INH2	GPIO_PIN_7			// Port M Pin 7	O

// SSI Defines
#define	MISO3		GPIO_PIN_0			// Port F Pin 0	SSI3
#define	MOSI3		GPIO_PIN_1			// Port F Pin 1	SSI3
#define	NCS			GPIO_PIN_2			// Port F Pin 2	SSI3
#define	SCLK3		GPIO_PIN_3			// Port F Pin 3	SSI3

// Miscellaneous Defines
#define	LHALF		0x0F
#define UHALF       0xF0
#define ALL         0xFF
#define OFF         0x00
#define HIGH        0x1
#define LOW         0x0

#define PI					3.1415926535897932384626433832795
#define HALF_PI				1.5707963267948966192313216916398
#define TWO_PI				6.283185307179586476925286766559
#define DEG_TO_RAD			0.017453292519943295769236907684886
#define RAD_TO_DEG			57.295779513082320876798154814105
#define	GPIO_LOCK_KEY_DD	0x4C4F434B
#define	POW2_28				268435456
#define	POW2_12				4096

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(int64_t)((x)+0.5):(int64_t)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#define highByte32(w)	((uint8_t) ((w) >> 24))
#define highMidByte32(w) ((uint8_t) ((w) >> 16))
#define lowMidByte32(w) ((uint8_t) ((w) >> 8))
#define highByte(w) ((uint8_t) ((w) >> 8))
#define lowByte(w) ((uint8_t) ((w) & 0xff))

#ifdef __cplusplus
} // extern “C”
#endif

// Global Constants
const uint32_t SYS_CLK_REQ = 120000000; 		// Requested clock frequency

// Global Enumerations
enum CHIP_SELECT 
{
CHA_VGA_NCS = 0x80, CHB_VGA_NCS, CHC_VGA_NCS, CHD_VGA_NCS, DDS0_NCS, DDS1_NCS, 
OFF_NCS, TRIM1_NCS, CSD, TRIM2_NCS = 0x08, DDS1_VGA_NCS = 0x18, OUT_GAIN1_NCS = 0x28, BIAS_NCS = 0x38/*, CS12 = 0x48, CS13 = 0x58, CS14 = 0x68, CS15 = 0x78*/
};

enum DDS_VGA_GAIN
{
DDSdB16P = 0x00, DDSdB15P, DDSdB14P, DDSdB13P, DDSdB12P, DDSdB11P, DDSdB10P, DDSdB09P, DDSdB08P, DDSdB07P, DDSdB06P, DDSdB05P, DDSdB04P, DDSdB03P, DDSdB02P, DDSdB01P, DDSdB00, DDSdB01M, DDSdB02M, DDSdB03M, DDSdB04M, DDSdB05M, DDSdB06M, DDSdB07M, DDSdB08M, DDSdB09M, DDSdB10M, DDSdB11M, DDSdB12M, DDSdB13M, DDSdB14M, DDSdB15M, DDSdB16M
};

enum VGA_GAIN
{
VGAdB26P = 0x00, VGAdB25P, VGAdB24P, VGAdB23P, VGAdB22P, VGAdB21P, VGAdB20P, VGAdB19P, VGAdB18P, VGAdB17P, VGAdB16P, VGAdB15P, VGAdB14P, VGAdB13P, VGAdB12P, VGAdB11P, VGAdB10P, VGAdB09P, VGAdB08P, VGAdB07P, VGAdB06P, VGAdB05P, VGAdB04P, VGAdB03P, VGAdB02P, VGAdB01P, VGAdB00, VGAdB01M, VGAdB02M, VGAdB03M, VGAdB04M, VGAdB05M, VGAdB06M
};

// Global Variables
uint32_t SYS_CLK_ACT = 0;					// Actual clock frequency obtained by PLL
uint32_t DDS_CLK = 75000000;

volatile uint32_t COUNT = 0;					// Stores how many times Interrupt handler called
volatile uint32_t CURRENT_FREQ = 0; 
volatile uint32_t CURRENT_PH = 0;

//**********************************************************************************************************
//
// Notes
//
//  Binary  Hex         Binary  Hex
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
//**********************************************************************************************************

void SysTickHandler() {

    //
    // The SysTick Timer is configured as a millisecond timer.  An unsigned 32 bit number will have a
    // maximum value of 18,446,744,073,709,551,615.  It will take nearly 50 days
    // (2^32/(1000*60*60*24*365.25)) of constant running to fill this variable.
    //
    COUNT++;

}

uint32_t millis () {

    return COUNT;

}

void chgDDS (uint32_t fRequested, uint32_t phRequested, bool *ddsReg0)
{

	bool update;

	uint8_t fRegSel = 0x00, phRegSel = 0x00,fData0 = 0x00, fData1 = 0x00, fData2 = 0x00, fData3 = 0x00;
	uint8_t pData0 = 0x00, pData1 = 0x00;
	uint32_t freqReg = 0, phaseReg = 0;

	if ((fRequested != CURRENT_FREQ) && (phRequested != CURRENT_PH))
	{
		
		freqReg = round((POW2_28 * fRequested) / DDS_CLK);
		phaseReg = round((POW2_12 * phRequested) / TWO_PI);
		CURRENT_FREQ = fRequested;
		CURRENT_PH = phRequested;
		update = true;
	
	}

	else if ((fRequested != CURRENT_FREQ) && (phRequested == CURRENT_PH))
	{
		
		freqReg = round((POW2_28 * fRequested) / DDS_CLK);
		phaseReg = round((POW2_12 * CURRENT_PH) / TWO_PI);
		CURRENT_FREQ = fRequested;
		update = true;
		
	}

	else if ((fRequested == CURRENT_FREQ) && (phRequested != CURRENT_PH))
	{
		
		freqReg = round((POW2_28 * CURRENT_FREQ) / DDS_CLK);
		phaseReg = round((POW2_12 * phRequested) / TWO_PI);
		CURRENT_PH = phRequested;
		update = true;
		
	}

	fData0 = ((uint8_t) (freqReg & 0xFF));
	fData1 = ((uint8_t) ((freqReg >> 8) & 0x3F));
	fData2 = ((uint8_t) ((freqReg >> 14) & 0xFF));
	fData3 = ((uint8_t) ((freqReg >> 22) & 0x3F));
	pData0 = ((uint8_t) (phaseReg & 0xFF));
	pData1 = ((uint8_t) ((phaseReg >> 8) & 0x0F));
	
	if (update)
	{
		
		SSIDisable(SSI3_BASE);
		SSIAdvModeSet(SSI3_BASE, SSI_ADV_MODE_WRITE);
		SSIEnable(SSI3_BASE);
		GPIOPinWrite(PORTM, ALL, DDS0_NCS);			// Select DDS0 Chip Select on MUX
		GPIOPinWrite(PORTK, ALL, DRST0 | !SLEEP0);		// Place DDS in reset state via pin

		if (*ddsReg0)
		{
			
			SSIDataPut(SSI3_BASE, (0xBF & fData3));	// DDS FREQ1 Register MSB
			SSIAdvDataPutFrameEnd(SSI3_BASE, fData2);
			SSIDataPut(SSI3_BASE, (0xBF & fData1));	// DDS FREQ1 Register LSB
			SSIAdvDataPutFrameEnd(SSI3_BASE, fData0);
			SSIDataPut(SSI3_BASE, (0xEF & pData1));	// DDS PHASE0 Register
			SSIAdvDataPutFrameEnd(SSI3_BASE, pData0);
			*ddsReg0 = false;
			fRegSel = FSEL0;
			phRegSel = PSEL0;
			
		}
		
		else
		{
			
			SSIDataPut(SSI3_BASE, (0x7F & fData3));	// DDS FREQ1 Register MSB
			SSIAdvDataPutFrameEnd(SSI3_BASE, fData2);
			SSIDataPut(SSI3_BASE, (0x7F & fData1));	// DDS FREQ1 Register LSB
			SSIAdvDataPutFrameEnd(SSI3_BASE, fData0);
			SSIDataPut(SSI3_BASE, (0xCF & pData1));		// DDS PHASE0 Register
			SSIAdvDataPutFrameEnd(SSI3_BASE, pData0);
			*ddsReg0 = true;
			fRegSel = !FSEL0;
			phRegSel = !PSEL0;
			
		}
		
		while (SSIBusy(SSI3_BASE))
		{
		} 
		
		GPIOPinWrite(PORTM, ALL, CSD);		// Deselect DDS0 Chip Select on MUX
		SSIAdvModeSet(SSI3_BASE, SSI_ADV_MODE_READ_WRITE);
		GPIOPinWrite(PORTK, ALL, !DRST0 | fRegSel | phRegSel);	// Lift Reset and change Freq/Phase Reg
		
		SysCtlDelay(5);					// Delay 15 MCU Clock cycles (125ns @ 120 MHz)
										// DDS propagation delay is 107 nanoseconds @ 75 MHz

	}

}

void main()
{

    //
    // Enable the Floating-Point Unit (FPU)
    //
    FPUEnable();
    FPULazyStackingEnable();

    //
    // Local Variables
    //
    const uint32_t MAX_CODE_VGA = 0x00000020, MAX_CODE_VR = 0x000003FF;
    const uint32_t MIN_CODE = 0x00000000;

    uint32_t led1OnTime = 500, led1OffTime = 500, led2OnTime = 100, led2ShortOff = 100, led2LongOff = 700;

    volatile bool led1On = false, led2On = false;

    volatile uint32_t gainCodeVGA_DDS = DDSdB00, gainCodeOut = 0x00000000, led1PreviousMillis = 0;
    volatile uint32_t led2PreviousMillis = 0, pulse = 0, led1State = LOW, led2State = LOW;

    #ifdef DEBUG
    volatile rollover = 0;

    volatile uint64_t loop = 0;
    #endif

    const float RI = 10000, RF1 = 33000, DIG_VR_FS = 10000, RS = 99.8;
    const float IOUT_MAX = 0.003 * (1023 / 1024);
    const float RF_MIN = RF1 + (((0 / 1024) * DIG_VR_FS) + 70);
    const float RF_MAX = RF1 + (((1023 / 1024) * DIG_VR_FS) + 70);
    const float RIN = (((152.2) * 69.8) / (152.2 + 69.8)) + 52.2, RF = 105.5;
    const float GAIN_OUT_MIN_DB = 20*logf(RF_MIN/RI), ATTEN_DB = 10;
    const float GAIN_OUT_MAX_DB = 20*logf(RF_MAX/RI);
    const float VIN_VGA = ((IOUT_MAX * RF) * (RIN / (RIN + RS))) * powf(10, (-ATTEN_DB / 20));

    volatile float rF = RF1 + (((gainCodeOut / 1024) * DIG_VR_FS) + 70);
    volatile float gainOut_dB = 20 * logf(rF / RI);

    //
    // Use external 25MHz Precision Oscillator to Generate an 120MHz System
    // Clock using the PLL
    //
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);
    SYS_CLK_ACT = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                    SYSCTL_CFG_VCO_480, SYS_CLK_REQ);

    //
    // Enable Peripherals
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	
    //
    // Configure the SysTick Timer.
    //
    SysTickPeriodSet(SYS_CLK_ACT / 1000);
    SysTickIntRegister(SysTickHandler);
    SysTickEnable();
    SysTickIntEnable();

    //
    // Unlock NMI (Direct Register Access)
    //
    GPIO_PORTD_AHB_LOCK_R = GPIO_LOCK_KEY_DD;   // Unlock Port D Commit Register
    GPIO_PORTD_AHB_CR_R = 0x00000080; 			// Allow NMI Pin to Be Written
    GPIO_PORTD_AHB_LOCK_R = 0x00000000;			// Relock Port D Commit Register

    //
    // Enable GPIO AHB
    //
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOF);

    //
	// Configure GPIO Type
	//
	// Output
	//
	GPIOPinTypeGPIOOutput(PORTF, ALL);
	GPIOPinTypeGPIOOutput(PORTK, ALL);
	GPIOPinTypeGPIOOutput(PORTM, ALL);
	GPIOPinTypeGPIOOutput(PORTN, ALL);
	
	// Input
	//
	GPIOPinTypeGPIOInput(PORTD, ALL);

	//
	// Set GPIO Direction
	//
	// Output
	//
	GPIODirModeSet(PORTF, ALL, GPIO_DIR_MODE_OUT);
	GPIODirModeSet(PORTK, ALL, GPIO_DIR_MODE_OUT);
	GPIODirModeSet(PORTM, ALL, GPIO_DIR_MODE_OUT);
	GPIODirModeSet(PORTN, ALL, GPIO_DIR_MODE_OUT);

	// Input
	//
	

    //
	// Configure GPIO Pad Properties
	//
	// Standard
	//
	GPIOPadConfigSet(PORTD, ALL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(PORTF, ALL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(PORTK, ALL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(PORTM, ALL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(PORTN, ALL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	//
	// Configure Selected Pins Alternate Function
	//
	GPIOPinConfigure(GPIO_PF3_SSI3CLK);
	GPIOPinConfigure(GPIO_PF2_SSI3FSS);
	GPIOPinConfigure(GPIO_PF1_SSI3XDAT0);
	GPIOPinConfigure(GPIO_PF0_SSI3XDAT1);

	//
	// Select the analog ADC function for selected pins
	//
	GPIOPinTypeADC(PORTD, ALL);

	//
	// Assign Selected Pins to SSI Peripheral
	//
	GPIOPinTypeSSI(PORTF, SCLK3 | NCS | MOSI3 | MISO3);

	//
	// Setup SSI communications.
	//
	SSIConfigSetExpClk(SSI3_BASE, SYS_CLK_ACT, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 2000000, 8);
	SSIAdvModeSet(SSI3_BASE, SSI_ADV_MODE_WRITE);
	SSIAdvFrameHoldEnable(SSI3_BASE);

	//
	// Enable SSI
	//
	SSIEnable(SSI3_BASE);

	//
	// Initialize DDS waveform generators (Place in reset and program Freq0, Freq1, 
	// Phase0 and Phase1 to 0 
	//
	GPIOPinWrite(PORTM, ALL, DDS0_NCS);			// Select DDS0 Chip Select on MUX
	SSIDataPut(SSI3_BASE, 0x23);				// configures DDS for 2 write words and pin control
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);		// and enabling reset via register
	GPIOPinWrite(PORTK, ALL, DRST0);			// Place DDS in reset state via hardware
	SSIDataPut(SSI3_BASE, 0x40);				// DDS FREQ0 Register outputs a frequency of 0 Hz MSB
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0x40);				// DDS FREQ0 Register outputs a frequency of 0 Hz LSB
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0x80);				// DDS FREQ1 Register outputs a frequency of 0 Hz MSB
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0x80);				// DDS FREQ1 Register outputs a frequency of 0 Hz LSB
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0xC0);				// DDS PHASE0 Register set to 0 degrees
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0xE0);				// DDS PHASE1 Register set to 0 degrees
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	SSIDataPut(SSI3_BASE, 0x22);				// configures DDS for 2 write words and pin control
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);
	GPIOPinWrite(PORTK, ALL, 0x00);				// Release from Reset

	SysCtlDelay(5);								// Delay 15 MCU Clock cycles (125ns @ 120 MHz)
												// DDS propagation delay is 107 nanoseconds @ 75 MHz
	
	GPIOPinWrite(PORTK, ALL, DRST0 | SLEEP0);	// Place DDS back into reset and
												// sleep until needed

	//
	// Wait for the SSI bus to become free before continuing.
	//
	while (SSIBusy(SSI3_BASE))
	{
	}

	//
	// Deselect all chip selects.
	//
	GPIOPinWrite(PORTM, ALL, CSD);

	//
	// Select the VGA and change the Gain to 26dB which equates to 16dB gain due to the
	// 10dB attenuator preceding the inputs to the VGA.
	//
	gainCodeVGA_DDS = DDSdB16P;
	GPIOPinWrite(PORTM, ALL, DDS1_VGA_NCS);
	SSIDataPut(SSI3_BASE, 0x02);
	SSIAdvDataPutFrameEnd(SSI3_BASE, gainCodeVGA_DDS);

	//
	// Wait for the SSI bus to become free before continuing.
	//
	while (SSIBusy(SSI3_BASE))
	{
	}

	//
	// Set the output amplification stage to a gain of approximately 10.39dB.  This
	// will show up as an approximately 3.3 V peak signal when the DDS is operational.  
	// This will remain true as long as the VGA gain is set for 26dB.
	//
	GPIOPinWrite(PORTM, ALL, OUT_GAIN1_NCS);
	SSIDataPut(SSI3_BASE, 0x01);				// Variable resistor number 1
	SSIDataPut(SSI3_BASE, 0x00);				// Minimum Gain, value 0 ...
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00);	// sent to Variable resistor number 1.
	SSIDataPut(SSI3_BASE, 0x02);				// Variable resistor number 2
	SSIDataPut(SSI3_BASE, 0x00); 			// Minimum Gain, value 0 ...
	SSIAdvDataPutFrameEnd(SSI3_BASE, 0x00); 	// sent to Variable resistor number 2.

	//
	// Wait for the SSI bus to become free before continuing.
	//
	while (SSIBusy(SSI3_BASE))
	{
	}

	//
	// Deselect all chip selects.
	//
	GPIOPinWrite(PORTM, ALL, CSD);

	//
	// Configure SSI3 to receive data instead of just transmitting.
	//
	SSIDisable(SSI3_BASE);
	SSIAdvModeSet(SSI3_BASE, SSI_ADV_MODE_READ_WRITE);
	SSIEnable(SSI3_BASE);

	//
	// Enter forever loop
	//
    while(1)
	{

        uint32_t currentMillis = millis();

        //
        // The below is used for debug only
        //
        #ifdef DEBUG
        if (loop == 0xFFFFFFFFFFFFFFFF)
        {

            rollover++;
            loop = 0;

         }

         else
         {

            loop++;

         }
         #endif

        if (led1On && (currentMillis - led1PreviousMillis >= led1OnTime))
		{

            led1On = false;
            led1PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED1, 0x0);

        }

        else if (!led1On && (currentMillis - led1PreviousMillis >= led1OffTime))
		{

            led1On = true;
            led1PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED1, LED1);

        }

        if (led2On && (pulse == 1) && (currentMillis - led2PreviousMillis >= led2OnTime))
		{

            led2On = false;
            led2PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED2, 0x0);

        }

        else if (led2On && (pulse == 2) && (currentMillis - led2PreviousMillis >= led2OnTime))
		{

            led2On = false;
            pulse = 0;
            led2PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED2, 0x0);

        }

        else if (!led2On && (pulse == 0) && (currentMillis - led2PreviousMillis >= led2LongOff))
		{

            led2On = true;
            pulse = 1;
            led2PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED2, LED2);

        }

        else if (!led2On && (pulse == 1) && (currentMillis - led2PreviousMillis >= led2ShortOff))
		{

            led2On = true;
            pulse = 2;
            led2PreviousMillis = currentMillis;
            ROM_GPIOPinWrite(PORTN, LED2, LED2);

        }

    }

}
