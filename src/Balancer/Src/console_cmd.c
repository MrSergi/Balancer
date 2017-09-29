#include "conf.h"
#include "console.h"

/*****************************************************************************
 * Function:		cmdStatus
 * ---------------------------------------------------------------------------
 * description:		Выводим информацию о микроконтроллере:
 * 					частоту тактирования, размер флэш-памяти, уникальный идентификатор
 * parameters:
 * on return:		void
 ****************************************************************************/
void cmdStatus(int argc, const char * const * argv)
{
	fc_printf("MCU clock          %u MHz\r\n", (uint16_t) (SystemCoreClock / 1000000));

	uint16_t sz;
	sz = *(__IO uint16_t*)(0x1FFFF7E0);
	fc_printf("MCU flash size     %u kB\r\n", sz);

	uint32_t ds0, ds1, ds2;
	ds0 = *(__IO uint32_t*)(0x1FFFF7E8);
	ds1 = *(__IO uint32_t*)(0x1FFFF7EC);
	ds2 = *(__IO uint32_t*)(0x1FFFF7F0);
	fc_printf("MCU unique ID      %08lX%08lX%08lX\r\n", ds0, ds1, ds2);
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


