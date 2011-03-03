/***************************************************************************
                        Edebug.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : functions for debugging (implementation)
 ***************************************************************************/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

static int enable_debug = 0;
static const char *dbgfile;
static int dbgline;

void EDBG_ENABLE()
{
  enable_debug = 1;
}

void EDBG_DISABLE()
{
  enable_debug = 0;
}

void Edbginfo(int line, const char *file)
{
  dbgfile = file;
  dbgline = line;
}

void Edebug(const char *fmt, ...)
{
  char *ptr;
  va_list argp;

  if(!enable_debug) return;

  ptr = strrchr(dbgfile, '/');
  if(ptr == NULL)
    fprintf(stderr,"%s:%i: ", dbgfile, dbgline);
  else
    fprintf(stderr,"%s:%i: ", ptr+1, dbgline);

  va_start(argp, fmt);
  vfprintf(stderr, fmt, argp);
  va_end(argp);
  fprintf(stderr, "\n");
}

void EPRINT(const char *format, ...)
{
  va_list ap;
  char p[1024] = "";

  /* Create string to append */
  va_start(ap, format);
  vsprintf(p, format, ap);
  va_end(ap);

  fprintf(stdout,"%s",p);
}

void EERR(const char *format, ...)
{
  va_list ap;
  char p[1024] = "";

  /* Create string to append */
  va_start(ap, format);
  vsprintf(p, format, ap);
  va_end(ap);

  fprint(stderr, "%s", p);
}

#ifdef __cplusplus
}
#endif
