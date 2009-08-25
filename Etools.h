/***************************************************************************
                        Etools.h  -  description
                            -------------------
  begin                : March 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : tool functions definitions
 ***************************************************************************/
#ifndef ETOOLS_H
#define ETOOLS_H

#include <math.h>
#include "Etypes.h"

#ifndef M_2PI
#define M_2PI (2.0 * M_PI)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** This is the code if you want to get an angle in the range ]-Pi, Pi]*/
double E_ToMinusPiPlusPi(double input);
double E_ToZeroTwoPi(double input);
long   E_SkipHeader(FILE *input);
int    E_ReadDataType(FILE *input);
int    E_PointOnNextMsgType(FILE *f, MODULENAME type);
int    E_SkipRecord(FILE *input);
int    E_ReadTypeAndTimeStamp(FILE *input, double *stamp);
int    E_GetCronNextData(FILE **descr, size_t size, int *idx, int *msg_type);
size_t E_ReadField(FILE *input, char *field);
void   E_DumpPoint(FILE *f, EPOINT3D pt);
void   E_PrintPoint(EPOINT3D pt);
double E_DegreesToRadians(double input);
double E_RadiansToDegrees(double input);

#ifdef __cplusplus
}
#endif

#endif

