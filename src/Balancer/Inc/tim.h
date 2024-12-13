#ifndef TIM_H_
#define TIM_H_

#include "conf.h"

#ifdef __cplusplus
extern "C" {
#endif

void MX_TIM1_Init(void);
void MX_TIM4_Init(void);

void TimPWMSetPulseValue(uint8_t Channel, uint16_t PulseValue);

#ifdef __cplusplus
}
#endif

#endif

//******************************************************************************
//  ENF OF FILE
//******************************************************************************

