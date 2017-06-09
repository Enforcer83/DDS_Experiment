#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "keypad.h"

#define	KEYPAD_ROW_0	GPIO_PIN_0	// The row beginning with 1.
#define KEYPAD_ROW_1	GPIO_PIN_1	// The row beginning with 4.
#define KEYPAD_ROW_2	GPIO_PIN_2	// The row beginning with 7.
#define KEYPAD_ROW_3	GPIO_PIN_3	// The row beginning with P(*).
#define KEYPAD_ROWS		(KEYPAD_ROW_0|KEYPAD_ROW_1|KEYPAD_ROW_2|KEYPAD_ROW_3)	// Handy for when you want to reference every pin connected to a row.

#define KEYPAD_COL_0	GPIO_PIN_4	// The column beginning with 1.
#define KEYPAD_COL_1	GPIO_PIN_5	// The column beginning with 2.
#define KEYPAD_COL_2	GPIO_PIN_6	// The column beginning with 3.
#define KEYPAD_COL_3	GPIO_PIN_7	// The column beginning with H(A).
#define KEYPAD_COLS		(KEYPAD_COL_0|KEYPAD_COL_1|KEYPAD_COL_2|KEYPAD_COL_3)	// Handy for when you want to reference every pin connected to a column.

#define keypadRow   4
#define keypadCol   4

bool NEW_KEY_PRESSED = false;

uint32_t KEYPAD_ROW_PORT = 0;
uint32_t KEYPAD_COL_PORT = 0;

int32_t LAST_KEY_PRESSED = -2;

const uint8_t keys[keypadRow][keypadCol] =
{{'1','2','3','h'},
 {'4','5','6','k'},
 {'7','8','9','m'},
 {'p','0','f','e'}};

void initKeypad (uint32_t rowPort, uint32_t colPort)
{

	//
    // Configure GPIO Type
    //
    // Output
    //
	GPIOPinTypeGPIOOutput(colPort, KEYPAD_COLS);
	
    // Input
    //
	GPIOPinTypeGPIOInput(rowPort, KEYPAD_ROWS);

	//
    // Set GPIO Direction
    //
    // Output
    //
	GPIODirModeSet(colPort, KEYPAD_COLS, GPIO_DIR_MODE_OUT);
	
	// Input
    //
	GPIODirModeSet(rowPort, KEYPAD_ROWS, GPIO_DIR_MODE_IN);
	
	//
    // Configure GPIO Pad Properties
    //
    // Standard
    //
	GPIOPadConfigSet(colPort, KEYPAD_COLS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(rowPort, KEYPAD_ROWS, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);

	//
	// Configure the GPIO Interrupt
	//
	GPIOIntTypeSet(rowPort, KEYPAD_ROWS, GPIO_HIGH_LEVEL);
	GPIOIntRegister(rowPort, keyPressISR);

	GPIOPinWrite(colPort, KEYPAD_COLS, KEYPAD_COLS); // turn on every column

	//
	// Store the passed port to the global variable
	//
	KEYPAD_ROW_PORT = rowPort;
	KEYPAD_COL_PORT = colPort;
	
	//
	// Enable the GPIO Interrupt
	//
	GPIOIntEnable(KEYPAD_ROW_PORT, KEYPAD_ROWS);

}

int32_t checkKeypadCol (uint8_t checkCol)
{

	uint8_t rowPins;

	GPIOPinWrite(KEYPAD_COL_PORT, KEYPAD_COLS, checkCol);

	rowPins = GPIOPinRead(KEYPAD_ROW_PORT, KEYPAD_ROWS);

	if (rowPins & KEYPAD_ROW_0)
	{

		return 0;

	}
	
	else if (rowPins & KEYPAD_ROW_1)
	{

		return 1;

	}

	else if (rowPins & KEYPAD_ROW_2)
	{

		return 2;

	}

	else if (rowPins & KEYPAD_ROW_3)
	{

		return 3;

	}

	return -1;

}

int32_t checkKeypad (void) {
	
	int32_t row = 0, col = 0;

	row = checkKeypadCol(KEYPAD_COL_0);

	if (row != -1)
	{

		col = 0;
		return keys[row][col];

	}

	row = checkKeypadCol(KEYPAD_COL_1);

	if (row != -1)
	{

		col = 1;
		return keys[row][col];

	}

	row = checkKeypadCol(KEYPAD_COL_2);

	if (row != -1)
	{

		col = 2;
		return keys[row][col];

	}

	row = checkKeypadCol(KEYPAD_COL_3);

	if (row != -1)
	{
	
		col = 3;
		return keys[row][col];

	}

	return -1;

}

int32_t getLastKeyPressed(void)
{

	if (!NEW_KEY_PRESSED)
	{
		
		return -1;
		
	}
	
	NEW_KEY_PRESSED = false;
	return LAST_KEY_PRESSED;

}

void keyPressISR(void)
{

	LAST_KEY_PRESSED = checkKeypad(); //poll the keypad
	NEW_KEY_PRESSED = true;

	GPIOPinWrite(KEYPAD_COL_PORT, KEYPAD_COLS, KEYPAD_COLS);
	
	GPIOIntClear(KEYPAD_ROW_PORT, KEYPAD_ROWS);

}
