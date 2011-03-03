/***************************************************************************
                        Etime.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : functions for time handling. No support for negative time!
 ***************************************************************************/

#ifndef ETIME_H
#define ETIME_H

#include <sys/time.h>
#include <sys/timeb.h>
#include <time.h>
#include <stdio.h>
#include "Etypes.h"

#ifdef __cplusplus
extern "C" {
#endif

#define USEC_PER_SEC 1000000


typedef enum {TIME_WATCH_STOP, TIME_WATCH_START} TIME_WATCH_TYPE;
typedef enum {TIME_COMP_SMALLER=-1, TIME_COMP_EQUAL=0, TIME_COMP_GREATER=1} TIME_COMP_TYPE;

void    Time_gettimeofday(TIMEVAL *tv);
int     Time_Equal(double a, double b);
void    Time_CopyFromTimeval(TIMEVAL *dest, struct timeval src);
void    Time_CopyToTimeval(struct timeval *dest, TIMEVAL src);
void    Time_Normalize(TIMEVAL *tv);
TIMEVAL Time_Add(TIMEVAL a, TIMEVAL b);
TIMEVAL Time_Diff(TIMEVAL a, TIMEVAL b);
double  Time_DiffDbl(TIMEVAL a, TIMEVAL b);
TIMEVAL Time_FromDbl(double d);
double  Time_FromTimeval(TIMEVAL tv);
TIMEVAL Time_From_us(unsigned int us);
TIMEVAL Time_From_ms(unsigned int ms);
unsigned int Time_To_ms(TIMEVAL tv);
void    Time_SleepDbl(double seconds);
void    Time_Sleep_ms(unsigned int ms);
void    Time_Print(FILE *ostrm, TIMEVAL tv, char *explr);
double  Time_StopWatch (TIME_WATCH_TYPE start);
void    Time_InitVoid(TIMEVAL *tv);
void    Time_InitZero(TIMEVAL *tv);
int     Time_Compare(TIMEVAL a, TIMEVAL b);
int     Time_CheckCloseDbl(double a, double b, double eps);
int     Time_CheckClose(TIMEVAL a, TIMEVAL b, double eps);
EBOOL   Time_DateFromTimeval(TIMEVAL tv, struct tm *date);
void    Time_PrintDate(FILE *ostrm, TIMEVAL tv);
double  Time_GeomagDate(TIMEVAL tv);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif
