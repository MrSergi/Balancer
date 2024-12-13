#ifndef MICRORL_FUNC_H_
#define MICRORL_FUNC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void consoleInit(void);
void consoleInput(uint8_t Buf);
//void consoleInput(uint8_t* Buf, uint32_t Len);
int fc_printf(const char * fmt, ...);

//Define this macros with correct write/read terminal functions
//#define microrl_getChar			USB_GetChar
//#define microrl_sendString		USB_SendString

void microrl_terminalInit();
void microrl_terminalProcess();
void microrl_printString(const char *str);
void microrl_printStringWithEndl(const char *str);
void microrl_printEndl();

// Команды консоли
void cmdStatus(int argc, const char * const * argv);
void cmdClear(int argc, const char * const * argv);
void cmdHelp(int argc, const char * const * argv);
void cmdPIDSetup(int argc, const char * const * argv);

/* Таблица команд консоли */
typedef struct _console_cmd_t
{
	char * name;				/* Имя команды */
	void   (*console_cmd)(); 	/* Указатель на функцию команды консоли */
} console_cmd_t;


#ifdef __cplusplus
}
#endif

#endif
