#ifndef CONF_H_
#define CONF_H_

//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
//#include "i2c.h"


uint32_t micros(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* CONF_H_ */
