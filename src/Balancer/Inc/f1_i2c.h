#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include "conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	I2C_TypeDef  	   *regs;				/* I2C port */
	GPIO_TypeDef 	   *scl_gpio_port;
	uint16_t			scl_gpio_pin;
	GPIO_TypeDef 	   *sda_gpio_port;
	uint16_t			sda_gpio_pin;
	uint32_t			ClkSpeed;
} i2c_cfg_t;


void i2cClockEnable(I2C_TypeDef  *I2Cx);
int32_t i2cInit(uint8_t i2c_id);
void i2cConfig(const i2c_cfg_t * cfg, uint8_t num);
uint8_t i2cRead(uint8_t Addr, uint8_t Reg, uint8_t len, uint8_t* Data);
uint8_t i2cWrite(uint16_t Addr, uint8_t Reg, uint8_t Data);

#ifdef __cplusplus
}
#endif

#endif

//******************************************************************************
//  ENF OF FILE
//******************************************************************************

