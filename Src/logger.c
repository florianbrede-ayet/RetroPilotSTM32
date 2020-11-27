#include "core.h"
#include "uart.h"
#include "logger.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

bool disable_ts = false;

char logbuffer[1024];
char logbuf2[32];

void logger( const char* fmt, ... )
{
  va_list arglist;
  va_start( arglist, fmt );
  vsnprintf(logbuffer, sizeof(logbuffer), fmt, arglist);
  va_end( arglist );

  if (!disable_ts && (strlen(fmt)>1 || fmt[0]!='\n')) {
    double i = millis()/1000.0;
    int32_t d = floor(i);
    uint32_t s = (i*1E3-d*1E3);

    snprintf(logbuf2, sizeof(logbuf2), "%4ld.%03lu [LOG] ", d, s);
    
    serial_debug_puts(logbuf2);
  }
  else
  {
    disable_ts = false;
  }
  

  serial_debug_puts(logbuffer);
}

void logger_raw( const char* fmt, ... )
{
  va_list arglist;
  va_start( arglist, fmt );
  vsnprintf(logbuffer, sizeof(logbuffer), fmt, arglist);
  va_end( arglist );
  serial_debug_puts(logbuffer);
}


void logger_e( const char* fmt, ... )
{
  va_list arglist;
  va_start( arglist, fmt );
  vsnprintf(logbuffer, sizeof(logbuffer), fmt, arglist);
  va_end( arglist );

  if (!disable_ts && (strlen(fmt)>1 || fmt[0]!='\n')) {
    double i = millis()/1000.0;
    int32_t d = floor(i);
    uint32_t s = (i*1E3-d*1E3);

    snprintf(logbuf2, sizeof(logbuf2), "%4ld.%03lu [ERR] ", d, s);
    
    serial_debug_puts(logbuf2);
  }
  else
  {
    disable_ts = false;
  }
  

  serial_debug_puts(logbuffer);
}


