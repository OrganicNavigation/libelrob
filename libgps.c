/***************************************************************************
                        libgps.c  -  description
                            -------------------
  begin                : April 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          :gps stuff
 ***************************************************************************/
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "libgps.h"
#include "Edebug.h"
#include "Emacros.h"

#ifndef E_MAP_POINT
#define E_MAP_POINT(mat,point) (mat).nRow = 3; (mat).nCol = 1; (mat).Element = (double *) (&point)
#endif /*E_MAP_POINT*/

#define USE_ENU

#ifdef __cplusplus
extern "C" {
#endif

const GPS_REAL e_square   = 2.0 * flattening - flattening * flattening;

void GPS_PrintGPS(GPS_Struct *strct)
{
  EPRINT("Latitude:\t\t%.5f [deg] (%.5f rad)\n",RAD2DEG(strct->latitude), strct->latitude);
  EPRINT("Longitude:\t\t%.5f [deg] (%.5f rad)\n",RAD2DEG(strct->longitude), strct->longitude);
  EPRINT("Altitude:\t\t%.5f [m]\n",strct->altitude);
}

/* [km] rmc signal provides tha latitude && longitude,
   altitude is set a-priori to 0.0 ;) */
inline GPS_Struct GPS_GetCoordFromRMC(GPS_rmc rmc)
{
  GPS_Struct gps;
  gps.latitude  = rmc.latitude;
  gps.longitude = rmc.longitude;
  gps.altitude  = 0.0;
  return gps;
}

/* [km] the differential information latitude & longitude & altitude */
inline GPS_Struct GPS_GetCoordFromGGA(GPS_gga gga)
{
  GPS_Struct gps;
  gps.latitude  = gga.latitude;
  gps.longitude = gga.longitude;
  gps.altitude  = gga.altitude;
  return gps;
}

void GPS_GPSClear(GPS_Struct *gps)
{
  gps->latitude  = 0.0;
  gps->longitude = 0.0;
  gps->altitude  = 0.0;
}

/* --- Statistics functions */
void GPS_UpdateGPSMean(unsigned long n, GPS_Struct *new_gps, GPS_Struct *old_gps)
{
  old_gps->latitude  = UPDATE_AVERAGE(n, new_gps->latitude , old_gps->latitude);
  old_gps->longitude = UPDATE_AVERAGE(n, new_gps->longitude, old_gps->longitude);
  old_gps->altitude  = UPDATE_AVERAGE(n, new_gps->altitude , old_gps->altitude);
}

void GPS_UpdateGPSVariance(unsigned long n,
                           GPS_Struct *new_gps,
                           GPS_Struct *gps_avg,
                           GPS_Struct *old_avg,
                           GPS_Struct *gps_var_upd)
{
  gps_var_upd->latitude  = UPDATE_VARIANCE(n, new_gps->latitude  , gps_avg->latitude,  old_avg->latitude,  gps_var_upd->latitude);
  gps_var_upd->longitude = UPDATE_VARIANCE(n, new_gps->longitude , gps_avg->longitude, old_avg->longitude, gps_var_upd->longitude);
  gps_var_upd->altitude  = UPDATE_VARIANCE(n, new_gps->altitude  , gps_avg->altitude,  old_avg->altitude,  gps_var_upd->altitude);
}

inline GPS_REAL GPS_ComputeEarthRadius(GPS_REAL sin_lat)
{
  return a_major/(sqrt(1.0 - e_square * sin_lat * sin_lat));
}

/* [km] interesting how the local xyz coordinates are calculated
   -> the latitude/longitude are expressed in radians, so 
   the x,y are calculated as the mean radius (with regard to dAlt
   l=r\cdot\phi */
EPOINT3D GPS_GPSToLocalXYZ(GPS_Struct *gps, GPS_Struct *origin, GPS_REAL earth_radius)
{
  EPOINT3D xyz;
  GPS_REAL dLong, dLat, dAlt;

  dLat  = -(gps->latitude - origin->latitude);
  dLong = gps->longitude -  origin->longitude;
  dAlt  = gps->altitude  -  origin->altitude;

  xyz.x = (earth_radius+dAlt) * dLat;
  xyz.y = (earth_radius+dAlt) * dLong;
  xyz.z = dAlt;

  return xyz;
}


/* convenience function [km] */
void GPS_CompLocalCoordinates(GPS_WorldOriginStruct *m_origin, 
			      GPS_Struct *gps,
			      double *x, double *y, double *z)
{
  EPOINT3D pxyz_earth;
  EPOINT3D pxyz_local;
  KMatrix xyz_earth;
  KMatrix xyz_local;

  E_MAP_POINT(xyz_earth,pxyz_earth);
  E_MAP_POINT(xyz_local,pxyz_local);

  if( !m_origin->initialized ){
    fprintf(stderr, " Error: world origin not initialized! %s %d ", __FILE__, __LINE__);
    return;
  }

  pxyz_earth = GPS_ConvertToEarthXYZ(gps);
  pxyz_earth.x -= m_origin->earth_xyz_origin.x;
  pxyz_earth.y -= m_origin->earth_xyz_origin.y;
  pxyz_earth.z -= m_origin->earth_xyz_origin.z;

  Y_Eq_AX (&(m_origin->local_rot_mat), &xyz_earth, &xyz_local);

#ifndef USE_ENU
  pxyz_local.x = -pxyz_local.x;
  pxyz_local.z = -pxyz_local.z;
#endif
  
  *x = pxyz_local.x;
  *y = pxyz_local.y;
  *z = pxyz_local.z;

  return;
}


/* this function initializes a GPS_WorldOriginStruct based on
   basic GPS_Struct data for further calculations (transforms
   to the local world frame) [km] */
void GPS_InitWorldOriginStruct( GPS_Struct *orig, GPS_WorldOriginStruct *m_origin)
{

  double sin_lat, cos_lat, sin_long, cos_long;
  
  m_origin->gps = *orig;

  m_origin->radius = GPS_ComputeEarthRadius(sin(m_origin->gps.latitude));/* + m_origin.gps.altitude;*/
  
  m_origin->earth_xyz_origin = GPS_ConvertToEarthXYZ(&(m_origin->gps));

  if(!m_origin->initialized)
  {
    AllocMatrix(&(m_origin->local_rot_mat),3,3);
    
  }

  sin_lat  = sin(m_origin->gps.latitude);
  cos_lat  = cos(m_origin->gps.latitude);
  sin_long = sin(m_origin->gps.longitude);
  cos_long = cos(m_origin->gps.longitude);

#ifdef USE_ENU
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,0,-sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,1, cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,2, 0.0);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,0,-sin_lat*cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,1,-sin_lat*sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,2, cos_lat);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,0, cos_lat*cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,1, cos_lat*sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,2, sin_lat);
#else
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,0,-sin_lat * cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,1,-sin_lat * sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),0,2,cos_lat);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,0,-sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,1,cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),1,2,0.0);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,0,-cos_lat * cos_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,1,-cos_lat * sin_long);
  SET_MATRIX_ELEMENT(&(m_origin->local_rot_mat),2,2,-sin_lat);
#endif

  m_origin->initialized = ETRUE;
}

/* cleanup [km] */
void GPS_ReleaseWorldOriginStruct( GPS_WorldOriginStruct *m_origin)
{
  if(m_origin->initialized)
  {
    DeleteMatrix(&(m_origin->local_rot_mat));
  }
}


/* See http://www.colorado.edu/geography/gcraft/notes/gps/gps.html
 * http://www.colorado.edu/geography/gcraft/notes/gps/gif/llhxyz.gif */

/* [km] this seems to be a more correct way to compute the coordinates */
EPOINT3D GPS_ConvertToEarthXYZ(GPS_Struct *gps)
{
  GPS_REAL N,sin_lat,cos_lat,cos_long,sin_long;
  EPOINT3D xyz;

  sin_lat  = sin(gps->latitude);   cos_lat  = cos(gps->latitude);
  sin_long = sin(gps->longitude);  cos_long = cos(gps->longitude);

  N = GPS_ComputeEarthRadius(sin_lat);

  xyz.x = (N+gps->altitude)*cos_lat*cos_long;
  xyz.y = (N+gps->altitude)*cos_lat*sin_long;
  xyz.z = (N*(1.0-e_square)+gps->altitude)*sin_lat;

  return xyz;
}

/* Convert a string ddmm.mmmmm into an angle expressed in radians
 * orient can be N/s or E/W */

/* [km] gps like coordinates (angles) converted to radians */
double ConvDegMin_To_Radians(char *str, char orient)
{
  char   tmp[16] = "",*ptr;
  double deg;
  const size_t l_min = 8;

  ptr = str;

  switch(orient)
  {
    case 'N':{ /* North latitude is negative */
      strncpy(tmp,ptr,2); tmp[2] = '\0';
      deg = (double) atoi(tmp);
      ptr += 2;
      strncpy(tmp,ptr,l_min); tmp[l_min] = '\0';
      deg += atof(tmp) / 60.0;
      deg = -deg;
    }
    break;

    case 'S':{ /* South latitude is positive */
      strncpy(tmp,ptr,2); tmp[2] = '\0';
      deg = atoi(tmp);
      ptr += 2;
      strncpy(tmp,ptr,l_min); tmp[l_min] = '\0';
      deg += atof(tmp) / 60.0;
    }
    break;

    case 'E':{ /* East latitude is negative */
      strncpy(tmp,ptr,3); tmp[3] = '\0';
      deg = atoi(tmp);
      ptr += 3;
      strncpy(tmp,ptr,l_min); tmp[l_min] = '\0';
      deg += atof(tmp) / 60.0;
      deg = -deg;
    }
    break;

    case 'W':{ /* West latitude is positive */
      strncpy(tmp,ptr,3); tmp[3] = '\0';
      deg = atoi(tmp);
      ptr += 3;
      strncpy(tmp,ptr,l_min); tmp[l_min] = '\0';
      deg += atof(tmp) / 60.0;
    }
    break;

    default: {
      fprintf(stderr, "orient not valid!\n");
      exit(EXIT_FAILURE);
    }
  }

  return DEG2RAD(deg);
}

/* Convert a string hhmmss.ss into a TIMEVAL (elapsed seconds from midnight) */
/* [km] conversion to current time-stamp with respect to midnight */
TIMEVAL ConvUTC_To_Timeval(char *ptr)
{
  char tmp[10];
  TIMEVAL tv;

  strncpy(tmp,ptr,2);
  tv.tv_sec =  atoi(tmp) * 3600;
  strncpy(tmp,ptr+2,2);
  tv.tv_sec += atoi(tmp) * 60;
  strncpy(tmp,ptr+4,2);
  tv.tv_sec += atoi(tmp);

  strncpy(tmp,ptr+7,2);
  tv.tv_usec = 10000 * atoi(tmp);

  return tv;
}

void GPS_DumpGPSHeader(FILE *strm, char *hostname)
{
  if(hostname != NULL)
    DUMP_HOSTNAME(strm,hostname);

  fprintf(strm,"# Header for GPS log\n");
  fprintf(strm,"# This file contains GPGGA, GPRMC, GPGST, GPGSA structures (we skip some field) \n");
  fprintf(strm,"# GGA format is : \n");
  fprintf(strm,"# 1:  data row number \n");
  fprintf(strm,"# 2:  measured time [s]\n");
  fprintf(strm,"# 3:  available time [s]\n");
  fprintf(strm,"# 4:  G (means that the row is GPGGA)\n");
  fprintf(strm,"# 5:  validity\n");
  fprintf(strm,"# 6:  measurement id\n");
  fprintf(strm,"# 7:  UTC [s]\n");
  fprintf(strm,"# 8:  latitude [rad]\n");
  fprintf(strm,"# 9:  longitude [rad]\n");
  fprintf(strm,"# 10: number of satellites\n");
  fprintf(strm,"# 11: HDOP\n");
  fprintf(strm,"# 12: quality\n");
  fprintf(strm,"# 13: altitude [m]\n");
  fprintf(strm,"# 14: geo seal level\n");
  fprintf(strm,"# 15: differential data age\n");
  fprintf(strm,"# 16: diff station id\n#\n");

  fprintf(strm,"# RMC format is : \n");
  fprintf(strm,"# 1:  data row number \n");
  fprintf(strm,"# 2:  measured time [s]\n");
  fprintf(strm,"# 3:  available time [s]\n");
  fprintf(strm,"# 4:  R (means that the row is GPRMC)\n");
  fprintf(strm,"# 5:  validity\n");
  fprintf(strm,"# 6:  status\n");
  fprintf(strm,"# 7:  measurement id\n");
  fprintf(strm,"# 8:  UTC [s]\n");
  fprintf(strm,"# 9:  latitude [rad]\n");
  fprintf(strm,"# 10:  longitude [rad]\n");
  fprintf(strm,"# 11: Speed [m/s]\n");
  fprintf(strm,"# 12: True course [rad]\n");
  fprintf(strm,"# 13: UT Date\n");
  fprintf(strm,"# 14: Magnetic variation\n");
  fprintf(strm,"# 15: Magnetic dir\n");

  fprintf(strm,"# GST format is : \n");
  fprintf(strm,"# 1:  data row number \n");
  fprintf(strm,"# 2:  measured time [s]\n");
  fprintf(strm,"# 3:  available time [s]\n");
  fprintf(strm,"# 4:  T (means that the row is GPGST)\n");
  fprintf(strm,"# 5:  validity \n");
  fprintf(strm,"# 6:  measurement id \n");
  fprintf(strm,"# 7:  utc time \n");
  fprintf(strm,"# 8:  rms \n");
  fprintf(strm,"# 9:  sigma_major \n");
  fprintf(strm,"# 10: sigma_minor  \n");
  fprintf(strm,"# 11: orientation \n");
  fprintf(strm,"# 12: sigma_lat  \n");
  fprintf(strm,"# 13: sigma_long  \n");
  fprintf(strm,"# 14: sigma_alt  \n");
}



#ifdef __cplusplus
}
#endif



