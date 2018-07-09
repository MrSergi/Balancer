/*
 * i2c.h
 *
 *  Created on: 23 окт. 2017 г.
 *      Author: SYSTEM
 */

#ifndef I2C_H_
#define I2C_H_

void MX_I2C2_Init(void);
uint8_t i2cRead(uint8_t Addr, uint8_t Reg, uint8_t len, uint8_t* Data);
uint8_t i2cWrite(uint16_t Addr, uint8_t Reg, uint8_t Data);
void Accel_Ini(void);

#endif /* I2C_H_ */
