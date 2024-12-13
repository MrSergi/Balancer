#ifndef F1_GPIO_H_
#define F1_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "conf.h"


typedef struct {
	GPIO_TypeDef 	 *gpio;
	uint16_t 		  Pin;
	uint32_t          Mode;
	uint32_t          Pull;
	uint32_t          Speed;
} gpio_config_t;

void gpioInit(const gpio_config_t * cfg, uint8_t num);

void gpioClockEnable(GPIO_TypeDef  *GPIOx);

void gpioHi(uint8_t gpio_id);
void gpioLo(uint8_t gpio_id);
void gpioToggle(uint8_t gpio_id);

#ifdef __cplusplus
}
#endif

#endif


