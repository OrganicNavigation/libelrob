/***************************************************************************
                        Etools.c  -  description
                            -------------------
  begin                : March 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : tool functions implementation
 ***************************************************************************/

#include <math.h>
#include <stdio.h>
#include <assert.h>
#include "Etools.h"

#define ESEP '\t'
#define E_MAX_LINE_LENGTH 1024

#ifdef __cplusplus
extern "C" {
#endif

const char E_INT_FMT[] = "%i\t";

/* [km] conversion */

double E_DegreesToRadians(double input)
{
  return ( input/180.*M_PI );
}

double E_RadiansToDegrees(double input)
{
  return ( input/M_PI*180 );
}


/* Convert an angle into the [0, M_2PI[ interval */
inline double E_ToZeroTwoPi(double input)
{
  int    mod;
  double res = 0.0;

  if(fabs(input) == M_2PI) return 0.0;

  if(input < 0.0) /* negative turn (clockwise) */
  {
    mod = (int) (input / M_2PI);  /* Compute the number of the M_2PI, mod is <= 0 */
    res = input - mod * M_2PI;    /* Now this is in ]0, -M_2PI] */
    res = M_2PI + res;
  }
  else
  if(input > 0.0) /* positive turn (anti-clockwise) */
  {
    mod = (int) (input / M_2PI);  /* Compute the number of the M_2PI, mod is <= 0 */
    res = input - mod * M_2PI;   /* Now this is in ]0, M_2PI] */
  }

  /*assert(res >= 0.0 && res < M_2PI);*/
  return res;
}

/* Convert an angle into the ]-M_PI, M_PI] interval */
inline double E_ToMinusPiPlusPi(double input)
{
  double zero_2pi;
  double res = 0.0;

  zero_2pi = E_ToZeroTwoPi(input); /* zero_2pi in [0, M_2PI[ */

  if (zero_2pi > M_PI) res = -M_2PI + zero_2pi;
  else res = zero_2pi;

  /*assert(res <= M_PI && res > -M_PI);*/
  return res;

 /* if (input < 0.0)
  {
  if (input < -M_PI) res = M_2PI + input;
  else res = input;
}
  else if (input > M_PI) res = input - M_2PI;
  else res = input;*/

  return res;
}

long E_SkipHeader(FILE *input)
{
  char  buf[E_MAX_LINE_LENGTH];
  long  offset = 0;
  char  *ptr = NULL;

  rewind(input);

  do
  {
    ptr = fgets(buf, E_MAX_LINE_LENGTH, input);
    if(buf[0] != '#') {
      fseek(input,offset,SEEK_SET);
      return offset;
    }
    offset = ftell(input);
  }
  while(!feof(input));

  return 0;
}

/* return 1 if ok else 0 */
int E_PointOnNextMsgType(FILE *f, MODULENAME type)
{
  char line[E_MAX_LINE_LENGTH] = "";
  int  tmp;

  while(1)
  {
    if(feof(f)) return 0;
    if((tmp = E_ReadDataType(f)) == type)
      return 1;
    else
      if(tmp == -1) return 0;
    else
      if(fgets(line,E_MAX_LINE_LENGTH,f) == NULL) return 0;
  }
}

/* -1 if error else type */
int E_ReadDataType(FILE *input)
{
  int     type, items;
  long    pos;
  char    type_str[256] = "";
  int     p = 0;

  if(feof(input))
    return -1;

  /* Store current position */
  pos = ftell(input);

  /* Read the file until we get the first char not in [0,9]*/
  while(!feof(input) && fread(&type_str[p],1,1,input) == 1 && type_str[p] >= '0' && type_str[p] <= '9') {
    p += 1;
  }

  /* check if anything went wrong */
  if(p == 0){
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Convert char to int */
  type_str[p+1] = '\n';
  items = sscanf(type_str, "%i", &type);

  if(items != 1)
  {
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Put the file pointer back in place */
  fseek(input, pos, SEEK_SET);
  return type;
}

/* return -2 if file is null, -1 if error and 0 if ok */
int E_ReadTypeAndTimeStamp(FILE *input, double *stamp)
{
  int     type, items;
  long    pos;
  char    type_str[256] = "";
  int     p = 0;

  if(input == NULL) return -2;
  if(feof(input)) return -1;

  /* Store current position */
  pos = ftell(input);

  /* Read the file until we get the first char not in [0,9]*/
  while(!feof(input) && (fread(&type_str[p],1,1,input) == 1) && type_str[p] >= '0' && type_str[p] <= '9') {
    p += 1;
  }

  /* check if anything went wrong */
  if(p == 0){
    fseek(input, pos, SEEK_SET);
    sleep(1);
    return -1;
  }

  /* Convert char to int */
  type_str[p+1] = '\n';
  items = sscanf(type_str, "%i", &type);

  if(items != 1)
  {
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  p = 0;

  /* Read the file until we get the first char not in [0,9]*/
  while(((!feof(input)) && (fread(&type_str[p],1,1,input) == 1) && (type_str[p] >= '0') && (type_str[p] <= '9')) || (type_str[p] == '.')) {
    p += 1;
  }

  /* check if anything went wrong */
  if(p == 0){
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Convert char to double */
  type_str[p+1] = '\n';
  items = sscanf(type_str, "%lf", stamp);

  if(items != 1)
  {
    fseek(input, pos, SEEK_SET);
    return -1;
  }

  /* Put the file pointer back in place */
  fseek(input, pos, SEEK_SET);
  return type;
}

int E_SkipRecord(FILE *input)
{
  char line[E_MAX_LINE_LENGTH] = "";

  if(feof(input)) return 0;
  if(fgets(line,E_MAX_LINE_LENGTH,input) != NULL) return 1;
  else return 0;
}

/* return 0 if error else 1
   idx contains the index in descr
   msg_type the type of data that can be read from descr[idx]
*/
int E_GetCronNextData(FILE **descr, size_t size, int *idx, int *msg_type)
{
  double mini_stamp = -1.0;
  double stamp;
  int    i, type;
  int    first = 1;

  *idx      = -1;
  *msg_type = -1;

  for(i = 0; i < size; i++)
  {
    if((type = E_ReadTypeAndTimeStamp(descr[i], &stamp)) == -1){
      /*EERR("err %i\n",i);*/
      return 0;
    }

    if(stamp < mini_stamp || first) {
      mini_stamp = stamp;
      *idx = i;
      *msg_type = type;
      first = 0;
    }
  }
  return 1;
}

size_t E_ReadField(FILE *input, char *field)
{
  int p = 0;

  /* Read the file until we get the first char not in [0,9]*/
  while(((!feof(input)) && (fread(&field[p],1,1,input) == 1) && (field[p] >= '0') && (field[p] <= '9')) || (field[p] == '.') || (field[p] == '-')) {
    p += 1;
  }
  field[p] = '\0';
  return p;
}

void E_DumpPoint(FILE *f, EPOINT3D pt)
{
  fprintf(f,"%f %f %f", pt.x, pt.y, pt.z);
}

void E_PrintPoint(EPOINT3D pt)
{
  printf("x: %e y: %e z: %e\n", pt.x, pt.y, pt.z);
}


#ifdef __cplusplus
}
#endif
