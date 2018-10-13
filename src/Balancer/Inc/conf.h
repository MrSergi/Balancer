/*
 * conf.h
 *
 *  Created on: 6 ���. 2017 �.
 *      Author: Sergey
 */

#ifndef CONF_H_
#define CONF_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "adc.h"
//#include "i2c.h"
#include "uart.h"

//#define DEBUG                            // Comment this out for final version

//typedef enum
//{
//  FALSE = 0, TRUE  = !FALSE
//}
//bool;

// Sensors

uint32_t micros(void);
void Error_Handler(void);

#endif /* CONF_H_ */
