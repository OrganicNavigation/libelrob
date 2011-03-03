/***************************************************************************
                        Emacros.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : common macros for ELROB
 ***************************************************************************/

#ifndef EMACROS_H
#define EMACROS_H

#include <math.h>

#ifdef FALSE
#if FALSE != 0
#error "FALSE is not zero"
#endif
#endif

#define FALSE 0
#define TRUE  1

/* Conversion macros */
#ifndef M_2PI
#define M_2PI                       (2.0*M_PI)
#endif

#define UPDATE_AVERAGE(n,new_data,old_data)  ((n * (old_data) + (new_data)) / (n+1))
#define UPDATE_VARIANCE(n,new_data,new_avg,old_avg,old_var) (((n)*((old_var)+SQR(old_avg))+SQR((new_data)))/((n)+1)- SQR(new_avg))

#define DEG2RAD(deg)                ((deg)*M_PI/180.0)
#define RAD2DEG(rad)                ((rad)*180.0/M_PI)
#define CELSIUS_TO_KELVIN(cel)      ((cel) + 273.15)
#define KELVIN_TO_CELSIUS(kel)      ((kel) - 273.15)
#define MM_TO_METERS(mm)            ((mm) / 1000.0)
#define KNOTS_TO_METERPERSEC(knots) (0.5148 * (knots))

#define DUMP_HOSTNAME(fd,hn) (fprintf(fd,"#!%s\n",hn))
#define SQR(a) ((a)*(a))
#define NORM2D(a,b)   sqrt(SQR(a)+SQR(b))
#define NORM3D(a,b,c) sqrt(SQR(a)+SQR(b)+SQR(c))
#define SIGMA_TO_VAR(s) (SQR(s))
#define VAR_TO_SIGMA(v) (sqrt(v))
#define TWO_SIGMA_TO_VAR(two_s) (SQR((two_s))/4.0)
#define THREE_SIGMA_TO_VAR(three_s) (SQR((three_s))/9.0)

#endif
