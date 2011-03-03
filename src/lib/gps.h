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
#include "Kmatrix.h"

#ifdef __cplusplus
extern "C" {
#endif

#define a_major 6378137.0                             /* equatorial radius (major)*/
#define b_minor 6356752.3142                          /* axial radius (minor) */
#define flattening ((a_major-b_minor)/a_major)        /* flattening */

/** The structure containing the GPS coordinates */
typedef struct{
  double latitude;  /* north = negative, south = positive */
  double longitude; /* east  = negative, west = positive */
  double altitude;
}GPS_Struct;

/* [km] the gga structure refers to the differential gps signal,
   the differential reference station ID is included in the message */
typedef struct GPS_gga {
  int                 valid;          /* tells if the gps row has been correctly
parsed */
  int                 nr;             /* number of the gps unit */
  int                 meas_id;        /* measurement id */
  TIMEVAL             utc;            /* Universal Time Coordinated (UTC)
seconds from midnight */
  double              latitude;
  char                lat_orient;     /* N or S (North or South) */
  double              longitude;
  char                long_orient;    /* E or W (East or West) */
  int                 gps_quality;    /* GPS Quality Indicator,
  0 - fix not available,
  1 - GPS fix,
  2 - Differential GPS fix */
  int                 num_satellites; /* Number of satellites in view, 00-12 */
  double              hdop;           /* Horizontal Dilution of precision */

  double              altitude;       /* Units of antenna altitude, meters */
  char                altitude_unit;       /* Altitude unit */
  double              geo_sea_level;  /* Geoidal separation, the difference
  between the WGS-84 earth ellipsoid and
  mean-sea-level (geoid), "-" means
  mean-sea-level below ellipsoid */

  char                geo_sep_unit;        /* Units of geoidal separation,
meters */
  int                 data_age;       /* Age of differential GPS data, time
  in seconds since last SC104 type 1 or
  9 update, null field when DGPS is not
  used */
  int                 diff_station;  /* Differential Reference Station ID*/
  TIMEVAL             meas_time;
  TIMEVAL             avail_time;
} GPS_gga;

/* This is not compatible with GPS_gga structure !!! */
/*#define CARMEN_GPS_GPGGA_MESSAGE_FMT "{int,
double,double,char,double,char,int,int,double,double,double,double,double,int,
double,string}"
#define CARMEN_GPS_GPGGA_MESSAGE_NAME "carmen_gps_nmea_gpgga"
*/

/* [km] the rmc signal gives the position, latitude, longitude,
   speed and true heading/course to north (not magnetic) */
typedef struct _GPS_rmc{
  int                 valid;          /* tells if the gps row has been correctly
parsed */
  int                 nr;             /* number of the gps unit */
  int                 meas_id;        /* measurement id */
  TIMEVAL             utc;            /* Universal Time Coordinated (UTC) */
  int                 status;         /* A: valid V: Warning*/
  double              latitude;
  char                lat_orient;     /* N or S (North or South) */
  double              longitude;
  char                long_orient;    /* E or W (East or West) */

  double              speed;          /* Speed over ground in m/s in the
direction of true_course */
  double              true_course;    /* heading to north (in rads) */
  int                 date;           /* UT Date  */
  double              magnetic_variation;
  char                magnetic_dir;   /* E or W */
  char                mode;           /* A D or N */

  TIMEVAL             meas_time;
  TIMEVAL             avail_time;
} GPS_rmc;

/* This is not compatible with GPS_rmc structure !!! */
/*#define CARMEN_GPS_GPRMC_MESSAGE_FMT  "{int,
char,double,double,char,double,char,double,double,int,double,char,double,string}
"
#define CARMEN_GPS_GPRMC_MESSAGE_NAME "carmen_gps_nmea_gprmc"
*/

/* PDOP = sqrt(var_x+var_y+var_z)
   VDOP = sqrt(var_z)
   HDOP = sqrt(var_x+var_y)
PDOP2 = HDOP2+VDOP2 */
typedef struct GPS_gsa {
  int      valid;          /* tells if the gps row has been correctly parsed */
  int      nr;             /* number of the gps unit */
  int      meas_id;        /* measurement id */
  char     mode;           /* M = manual, A = automatic */
  int      gps_quality;    /* GPS Quality Indicator,
  1 - fix not available,
  2 - GPS fix,
  3 - 3D fix */
  int      PRN[12];        /* PRN umbers of the satellites */
  double   pdop;           /* Position Dilution of precision */
  double   hdop;           /* Horizontal Dilution of precision */
  double   vdop;           /* Vertical Dilution of precision */

  TIMEVAL  meas_time;
  TIMEVAL  avail_time;
} GPS_gsa;

/* GPS Pseudorange Noise Statistics */

/* [km] GST -> measurement noise statistics */
typedef struct GPS_gst {
  int        valid;          /* tells if the gps row has been correctly parsed
*/
  int        nr;             /* number of the gps unit */
  int        meas_id;        /* measurement id */
  TIMEVAL    utc;
  double     rms;            /* of the standart deviation */
  double     sigma_major;    /* in meters */
  double     sigma_minor;
  double     orientation;
  double     sigma_lat;      /* standard deviation of latitude in m  */
  double     sigma_long;     /* standard deviation of longitude in m */
  double     sigma_alt;      /* standard deviation of altitude in m  */
  TIMEVAL    meas_time;
  TIMEVAL    avail_time;
} GPS_gst;


/** The structure containing the initial state of the local map */
typedef struct {
  GPS_Struct    gps;             /* initial GPS coordinates (lat, long in rad
and alt in m) */
  double        radius;          /* do not modify by hand, use SetLocalOrigin
instead */
  EPOINT3D      earth_xyz_origin;/* origin of the local frame in xyz earth
coordinates */
  KMatrix       local_rot_mat;   /* rotation matrix set with local origin gps
coordinates */
  EBOOL         initialized;     /* tells if the structure has been initialized
or not */
} GPS_WorldOriginStruct;

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

