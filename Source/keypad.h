#ifndef KEYPAD_H_
#define KEYPAD_H_

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

void initKeypad(uint32_t rowPort, uint32_t colPort);

int32_t checkKeypadCol(uint8_t checkCol);

int32_t checkKeypad(void);

void keyPressISR(void);

#endif /* KEYPAD_H_ */
