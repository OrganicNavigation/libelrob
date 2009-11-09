/***************************************************************************
                        Etime.c  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : implementation of the functions for time handling
 ***************************************************************************/

#define _ISOC99_SOURCE
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include "Etime.h"
#include "Edebug.h"

inline void Time_CopyFromTimeval(TIMEVAL *dest, struct timeval src)
{
  dest->tv_sec  = src.tv_sec;
  dest->tv_usec = src.tv_usec;
}

inline void Time_CopyToTimeval(struct timeval *dest, TIMEVAL src)
{
  dest->tv_sec  = src.tv_sec;
  dest->tv_usec = src.tv_usec;
}

void Time_InitVoid(TIMEVAL *tv)
{
  tv->tv_sec  = 123456;
  tv->tv_usec = 654321;
}

void Time_InitZero(TIMEVAL *tv)
{
  tv->tv_sec  = 0;
  tv->tv_usec = 0;
}

inline void Time_gettimeofday(TIMEVAL *tv)
{
  struct timeval ntv;

  gettimeofday(&ntv,0);
  Time_CopyFromTimeval(tv, ntv);
}

/* Compare a and b. If a-b y epsilon return 1 else return 0 */
int Time_Equal(double a, double b)
{
  if(fabs(a-b) < 1e-6)
    return 1;
  else
    return 0;
}

/* Normalize a timeval */
inline void Time_Normalize(TIMEVAL *tv)
{
  tv->tv_sec += tv->tv_usec / USEC_PER_SEC;
  tv->tv_usec = tv->tv_usec % USEC_PER_SEC;
}

inline TIMEVAL Time_FromDbl(double d)
{
  TIMEVAL tv;

#ifndef NDEBUG
  /*assert(d >= 0.0);*/
#endif

  tv.tv_sec  = (long) d;
  tv.tv_usec = (long) (round(1000000.0 * (d - (double) tv.tv_sec)));

 /* printf("round %.20f\n", ceil(1000000.0 * (d - (double) tv.tv_sec)) );
  Time_Print(stdout, tv, "test");*/
  return tv;
}

inline double Time_FromTimeval(TIMEVAL tv)
{
  return (tv.tv_sec + 1e-6 * tv.tv_usec);
}

inline TIMEVAL Time_From_us(unsigned int us)
{
  TIMEVAL tv;

  tv.tv_sec  = us / USEC_PER_SEC;
  tv.tv_usec = us % USEC_PER_SEC;

  return tv;
}

inline TIMEVAL Time_From_ms(unsigned int ms)
{
  return Time_From_us(1000 * ms);
}

unsigned int Time_To_ms(TIMEVAL tv)
{
  return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

inline TIMEVAL Time_Add(TIMEVAL a, TIMEVAL b)
{
  TIMEVAL tv;

  tv.tv_sec  = a.tv_sec  + b.tv_sec;
  tv.tv_usec = a.tv_usec + b.tv_usec;
  Time_Normalize(&tv);

  return tv;
}

/* return 0 if equal, 1 if a > b and -1 if a < b */
int Time_Compare(TIMEVAL a, TIMEVAL b)
{
  /* Everything is equal */
  if(a.tv_sec == b.tv_sec && a.tv_usec == b.tv_usec)
    return TIME_COMP_EQUAL;

  if(a.tv_sec > b.tv_sec)
    return TIME_COMP_GREATER;
  else
  {
    if(a.tv_sec < b.tv_sec)
      return TIME_COMP_SMALLER;
    else /* seconds == -> use micro-seconds to discriminate */
    {
      if(a.tv_usec > b.tv_usec)
        return TIME_COMP_GREATER;
      else
        return TIME_COMP_SMALLER;
    }
  }
}

/* a - b make sur b <= a*/
inline TIMEVAL Time_Diff(TIMEVAL a, TIMEVAL b)
{
  TIMEVAL tv;

#ifndef NDEBUG
  assert(a.tv_sec >= b.tv_sec);
#endif

  /* seconds */
  tv.tv_sec = a.tv_sec - b.tv_sec;

  /* micro-seconds */
  if(a.tv_usec >= b.tv_usec)
    tv.tv_usec = a.tv_usec - b.tv_usec;
  else{
    tv.tv_sec -= 1;

#ifndef NDEBUG
    assert(tv.tv_sec >= 0);
#endif
    tv.tv_usec = USEC_PER_SEC - (b.tv_usec - a.tv_usec);
  }

  return tv;
}

/* a - b */
inline double Time_DiffDbl(TIMEVAL a, TIMEVAL b)
{
  return Time_FromTimeval(Time_Diff(a,b));
}

/* Return the average between a and b (return (a+b)/2), precond b > a*/
inline TIMEVAL Time_Average(TIMEVAL a, TIMEVAL b)
{
  TIMEVAL avg;
  unsigned int carry;

  avg.tv_sec  = a.tv_sec/2  + b.tv_sec/2;
  carry = a.tv_sec % 2 + b.tv_sec %2;
  carry *= 500000;
  avg.tv_usec = a.tv_usec/2 + b.tv_usec/2 + carry;
  carry = a.tv_usec % 2 + b.tv_usec %2;
  if(carry == 2) avg.tv_usec += 1;

  /* Normalize */
  avg.tv_sec += avg.tv_usec / 1000000;
  avg.tv_usec = avg.tv_usec % 1000000;

  return avg;
}

/* return 0 if a-b >= eps */
inline int Time_CheckCloseDbl(double a, double b, double eps)
{
  return fabs(a-b) < eps;
}

inline int Time_CheckClose(TIMEVAL a, TIMEVAL b, double eps)
{
  return Time_CheckCloseDbl(Time_FromTimeval(a), Time_FromTimeval(b), eps);
}

void Time_SleepDbl(double seconds)
{
  struct timeval tv;
  TIMEVAL period;

#ifndef NDEBUG
  assert(seconds >= 0.0);
#endif

  if (seconds == 0.0) return;

  period = Time_FromDbl(seconds);
  Time_CopyToTimeval(&tv, period);
  select(0,0,0,0,&tv);
}

void Time_Sleep_ms(unsigned int ms)
{
  struct timeval tv;

  if (ms == 0) return;
  Time_CopyToTimeval(&tv, Time_From_ms(ms));
  select(0,0,0,0,&tv);
}

/* Return the elapsed time between two calls of this function (in seconds) is start == 0
// Else return -1 */
double Time_StopWatch (TIME_WATCH_TYPE start)
{
  static TIMEVAL start_tv;
  TIMEVAL    stop_tv;

  if (start)  {
    Time_gettimeofday(&start_tv);
    return (-1.0);
  }
  else {
    Time_gettimeofday(&stop_tv);
    return Time_DiffDbl (stop_tv, start_tv);
  }

}
void Time_Print(FILE *ostrm, TIMEVAL tv, char *explr)
{
  fprintf (ostrm, "%s [%li, %.6li] (%.6f s)", explr, tv.tv_sec, tv.tv_usec, Time_FromTimeval(tv));
}

/** Return the date given the elapsed second for the epoch */
EBOOL Time_DateFromTimeval(TIMEVAL tv, struct tm *date)
{
  struct tm  *date_tmp;
  time_t     tv_time = tv.tv_sec;
  date_tmp = gmtime_r(&tv_time, date);

  if(date_tmp != date) {
    EERR("gmtime_r -> failure\n");
    return 0;
  }

  return 1;
}

void Time_PrintDate(FILE *ostrm, TIMEVAL tv)
{
  struct tm date;
  Time_DateFromTimeval(tv, &date);
  fprintf(ostrm, asctime(&date));
}

/** Return a date compatible with the geomag library. -1 in case of failure */
double Time_GeomagDate(TIMEVAL tv)
{
  struct tm date;

  if(Time_DateFromTimeval(tv, &date))
    return date.tm_year + 1900 + (double) (date.tm_yday / 365.0);
  else
    return -1.0;
}

