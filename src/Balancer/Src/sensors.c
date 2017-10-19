#include "conf.h"

/*
 * Return CPU temperature in 0.1 degrees celsius
 */
int16_t cpuTemp(void)
{
	float Temp_Voltage = adcGetChannel(ADC_TEMP_SENSOR) * 3.3 / 4095.0;
	const float STM32_TEMP_V25 = 1.43; 			/* V */
	const float STM32_TEMP_AVG_SLOPE = 4.3; 	/* mV/C */
	return ((STM32_TEMP_V25 - Temp_Voltage) * 1000.0 / STM32_TEMP_AVG_SLOPE + 25.0) * 10;
}

void sensorInit(void)
{

}


