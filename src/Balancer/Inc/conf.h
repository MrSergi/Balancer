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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "adc.h"

// Sensors
void sensorInit(void);
int16_t cpuTemp(void);

#endif /* CONF_H_ */
