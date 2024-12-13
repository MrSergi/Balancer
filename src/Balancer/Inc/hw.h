#ifndef HW_H_
#define HW_H_

#include "f1_gpio.h"
#include "f1_i2c.h"
#include "f1_uart.h"


enum {
	GPIO_LED_ERR 	 = 0,
	GPIO_LED_STA 	 = 1,
	GPIO_M_RIGHT_DIR = 2,
	GPIO_M_LEFT_DIR  = 3,
	HW_GPIO_COUNT
};

#ifdef __cplusplus
extern "C" {
#endif

void hwInit(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif

//******************************************************************************
//  ENF OF FILE
//******************************************************************************

