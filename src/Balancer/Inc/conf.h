/*
 * conf.h
 *
 *  Created on: 6 рту. 2017 у.
 *      Author: Sergey
 */

#ifndef CONF_H_
#define CONF_H_

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


#include "adc.h"
#include "i2c.h"

//typedef enum
//{
//  FALSE = 0, TRUE  = !FALSE
//}
//bool;

// Sensors


void Error_Handler(void);

#endif /* CONF_H_ */
