/**************************************************************************//*****
 * @file     syscalls.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
//#include "usbd_cdc_vcp.h"
#include "usbd_cdc_if.h"

#undef errno
extern int errno;
extern int  _end;

#if USE_LIBC_TIME
#include <sys/time.h>

uint64_t micros(void);

int _gettimeofday( struct timeval *tv, void *tzvp )
{
    uint64_t t = micros();  	// get uptime in microseconds
    tv->tv_sec = t / 1000000;  	// convert to seconds
    tv->tv_usec = t % 1000000;  // get remaining microseconds
    return 0;  					// return non-zero for error
} // end _gettimeofday()
#endif

caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}


int link(char *old, char *new)
{
	return -1;
}

int _close(int file)
{
	return -1;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _read(int file, char *ptr, int len)
{
	return 0;
}

int _write(int file, char *ptr, int len)
{
//	VCP_DataTx((uint8_t *) ptr, len);
	volatile uint32_t counter = 0;

	CDC_Transmit_FS((uint8_t *) ptr, len);

	for(counter = 0; counter < 1000000; counter ++) // временная задержка
		continue;

	return len;
}

void abort(void)
{
	/* Abort called */
	while (1)
		;
}

/* --------------------------------- End Of File ------------------------------ */
