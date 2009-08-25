/***************************************************************************
                        libgps.h  -  description
                            -------------------
  begin                : April 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          :gps stuff
 ***************************************************************************/

#ifndef LIBGPS_H
#define LIBGPS_H

#include "Etypes.h"
#include "libgps_struct.h"
#include "gps-nmea.h"

#ifdef __cplusplus
extern "C" {
#endif

#define a_major 6378137.0                             /* equatorial radius (major)*/
#define b_minor 6356752.3142                          /* axial radius (minor) */
#define flattening ((a_major-b_minor)/a_major)        /* flattening */

typedef double GPS_REAL;

void GPS_PrintGPS(GPS_Struct *strct);
void GPS_GPSClear(GPS_Struct *gps);
EPOINT3D GPS_ConvertToEarthXYZ(GPS_Struct *gps);
EPOINT3D GPS_GPSToLocalXYZ(GPS_Struct *gps, GPS_Struct *origin, GPS_REAL earth_radius);

/** Convert latitude, longitude and altitude in X,Y,Z earth frame
 * z pointing towards true north (// to the rotation axis),
 * x-z in the equator plane */
GPS_REAL GPS_ComputeEarthRadius(GPS_REAL sin_lat);

double ConvDegMin_To_Radians(char *str, char orient);
TIMEVAL ConvUTC_To_Timeval(char *ptr);
void GPS_DumpGPSHeader(FILE *strm, char *hostname);
GPS_Struct GPS_GetCoordFromRMC(GPS_rmc rmc);
GPS_Struct GPS_GetCoordFromGGA(GPS_gga gga);
void GPS_UpdateGPSMean(unsigned long n, GPS_Struct *new_gps, GPS_Struct *old_gps);

void GPS_UpdateGPSVariance(unsigned long n,
                           GPS_Struct *new_gps,
                           GPS_Struct *gps_avg,
                           GPS_Struct *old_avg,
                           GPS_Struct *gps_var_upd);

/* [km] */
void GPS_CompLocalCoordinates(GPS_WorldOriginStruct *m_origin, 
			      GPS_Struct *gps, double * x, double * y, double *z);
/* [km] */
void GPS_InitWorldOriginStruct( GPS_Struct *orig, GPS_WorldOriginStruct *m_origin);

/* [km] */
void GPS_ReleaseWorldOriginStruct( GPS_WorldOriginStruct *m_origin);

#ifdef __cplusplus
}
#endif


#endif

