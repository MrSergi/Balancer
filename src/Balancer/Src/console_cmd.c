#include "conf.h"
#include "console.h"
#include "adc.h"
#include "sensors.h"

/*****************************************************************************
 * Function:		ftoa
 * ---------------------------------------------------------------------------
 * description:		Преобразование дробного числа в строку
 * parameters:		x - дробное число; floatString - преобразованная строка
 * on return:		преобразованная строка
 ****************************************************************************/
char *ftoa(float x, char *floatString)
{
    int32_t value;
    char intString1[12];
    char intString2[12] = { 0, };
    char *decimalPoint = ".";
    uint8_t dpLocation;

    if (x > 0)                  // Rounding for x.xxx display format
        x += 0.0005f;
    else
        x -= 0.0005f;

    value = (int32_t) (x * 1000.0f);    // Convert float * 1000 to an integer

    itoa(abs(value), intString1, 10);   // Create string from abs of integer value

    if (value >= 0)
        intString2[0] = ' ';    // Positive number, add a pad space
    else
        intString2[0] = '-';    // Negative number, add a negative sign

    if (strlen(intString1) == 1) {
        intString2[1] = '0';
        intString2[2] = '0';
        intString2[3] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 2) {
        intString2[1] = '0';
        intString2[2] = '0';
        strcat(intString2, intString1);
    } else if (strlen(intString1) == 3) {
        intString2[1] = '0';
        strcat(intString2, intString1);
    } else {
        strcat(intString2, intString1);
    }

    dpLocation = strlen(intString2) - 3;

    strncpy(floatString, intString2, dpLocation);
    floatString[dpLocation] = '\0';
    strcat(floatString, decimalPoint);
    strcat(floatString, intString2 + dpLocation);

    return floatString;
}

/*****************************************************************************
 * Function:		cmdStatus
 * ---------------------------------------------------------------------------
 * description:		Выводим различную информацию о плате
 * parameters:
 * on return:		void
 ****************************************************************************/
void cmdStatus(int argc, const char * const * argv)
{
	char string[30];

	fc_printf("MCU clock        %u MHz\r\n", (uint16_t) (SystemCoreClock / 1000000));

	uint16_t sz;
	sz = *(__IO uint16_t*)(0x1FFFF7E0);
	fc_printf("MCU flash size   %u kB\r\n", sz);

	uint32_t ds0, ds1, ds2;
	ds0 = *(__IO uint32_t*)(0x1FFFF7E8);
	ds1 = *(__IO uint32_t*)(0x1FFFF7EC);
	ds2 = *(__IO uint32_t*)(0x1FFFF7F0);
	fc_printf("MCU unique ID    %08lX%08lX%08lX\r\n", ds0, ds1, ds2);

	float adc1 = (((float)adcGetChannel(0) * 3.309) / 4095.0) * 6.0;
	ftoa(adc1, string);
	fc_printf("ADC1:            %s V\r\n", string);

	float adc2 = (((float)adcGetChannel(1) * 3.309) / 4095.0) * 6.0;
	ftoa(adc2, string);
	fc_printf("ADC2:            %s V\r\n", string);

	float adc3 = (((float)adcGetChannel(2) * 3.309) / 4095.0) * 2.0;
	ftoa(adc3, string);
	fc_printf("ADC3:            %s V\r\n", string);

	ftoa(SensGetTempCPU() / 10.0f, string);
	fc_printf("CPU Temp:        %s deg C\r\n", string);

	ftoa(AccSensor.Temp, string);
	fc_printf("ACC Temp:        %s deg C\r\n", string);
}

/*****************************************************************************
 * Function:		cmdClear
 * ---------------------------------------------------------------------------
 * description:		Очищаем экран консоли
 * parameters:
 * on return:		void
 ****************************************************************************/

void cmdClear(int argc, const char * const * argv)
{
	fc_printf ("\033[2J\r\n");    // ESC seq for clear entire screen
	fc_printf ("\033[H\r\n");     // ESC seq for move cursor at left-top corner
}


