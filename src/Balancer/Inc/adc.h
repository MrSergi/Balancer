/*
 * adc.h
 *
 *  Created on: 10 окт. 2017 г.
 *      Author: SYSTEM
 */

#ifndef ADC_H_
#define ADC_H_

enum {
	ADC_VIDEO_BATTERY = 0,
	ADC_VOLTAGE_SENSOR,
	ADC_CURRENT_SENSOR,
	ADC_TEMP_SENSOR,
    ADC_NUM_CHANNELS
};

void MX_ADC1_Init(void);
uint16_t adcGetChannel(uint8_t channel);

#endif /* ADC_H_ */
