/***************************************************************************
                        libgps_struct.h  -  description
                            -------------------
  begin                : April 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          :gps structures definitions
 ***************************************************************************/

#ifndef LIBGPS_STRUCT_H
#define LIBGPS_STRUCT_H

#include "Kmatrix.h"
#include "Etypes.h"


#ifdef __cplusplus
extern "C" {
#endif

/** The structure containing the GPS coordinates */
typedef struct{
  double latitude;  /* north = negative, south = positive */
  double longitude; /* east  = negative, west = positive */
  double altitude;
}GPS_Struct;


/* [km] the gga structure refers to the differential gps signal,
   the differential reference station ID is included in the message */
typedef struct GPS_gga {
  int                 valid;          /* tells if the gps row has been correctly parsed */
  int                 nr;             /* number of the gps unit */
  int                 meas_id;        /* measurement id */
  TIMEVAL             utc;            /* Universal Time Coordinated (UTC) seconds from midnight */
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

  char                geo_sep_unit;        /* Units of geoidal separation, meters */
  int                 data_age;       /* Age of differential GPS data, time
  in seconds since last SC104 type 1 or
  9 update, null field when DGPS is not
  used */
  int                 diff_station;  /* Differential Reference Station ID*/
  TIMEVAL             meas_time;
  TIMEVAL             avail_time;
} GPS_gga;

/* This is not compatible with GPS_gga structure !!! */
/*#define CARMEN_GPS_GPGGA_MESSAGE_FMT "{int, double,double,char,double,char,int,int,double,double,double,double,double,int,double,string}"
#define CARMEN_GPS_GPGGA_MESSAGE_NAME "carmen_gps_nmea_gpgga"
*/

/* [km] the rmc signal gives the position, latitude, longitude,
   speed and true heading/course to north (not magnetic) */
typedef struct _GPS_rmc{
  int                 valid;          /* tells if the gps row has been correctly parsed */
  int                 nr;             /* number of the gps unit */
  int                 meas_id;        /* measurement id */
  TIMEVAL             utc;            /* Universal Time Coordinated (UTC) */
  int                 status;         /* A: valid V: Warning*/
  double              latitude;
  char                lat_orient;     /* N or S (North or South) */
  double              longitude;
  char                long_orient;    /* E or W (East or West) */

  double              speed;          /* Speed over ground in m/s in the direction of true_course */
  double              true_course;    /* heading to north (in rads) */
  int                 date;           /* UT Date  */
  double              magnetic_variation;
  char                magnetic_dir;   /* E or W */
  char                mode;           /* A D or N */

  TIMEVAL             meas_time;
  TIMEVAL             avail_time;
} GPS_rmc;

/* This is not compatible with GPS_rmc structure !!! */
/*#define CARMEN_GPS_GPRMC_MESSAGE_FMT  "{int, char,double,double,char,double,char,double,double,int,double,char,double,string}"
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
  int        valid;          /* tells if the gps row has been correctly parsed */
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
  GPS_Struct    gps;             /* initial GPS coordinates (lat, long in rad and alt in m) */
  double        radius;          /* do not modify by hand, use SetLocalOrigin instead */
  EPOINT3D      earth_xyz_origin;/* origin of the local frame in xyz earth coordinates */
  KMatrix       local_rot_mat;   /* rotation matrix set with local origin gps coordinates */
  EBOOL         initialized;     /* tells if the structure has been initialized or not */
} GPS_WorldOriginStruct;


#ifdef __cplusplus
}
#endif

#endif

