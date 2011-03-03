/***************************************************************************
                        Etypes.h  -  description
                            -------------------
  begin                : December 2005
  copyright            : (C) 2005 by Pierre Lamon, Sascha Kolski
  email                : pierre.lamon@epfl.ch

  description          : common types for ELROB
 ***************************************************************************/
#ifndef ETYPES_H
#define ETYPES_H

typedef struct _TIMEVAL{
  long tv_sec;   /* seconds */
  long tv_usec;  /* microseconds */
}TIMEVAL;

/*! \brief Boolean Type for ELROB */
typedef enum _EBOOL{ EFALSE, ETRUE } EBOOL;

/*! \brief type for file- or devicenames */
typedef struct _EFILENAME {
  char name[1024];
}EFILENAME;

/*! \brief Enumeration for modules

These identifiers are used in the logfiles to specify the origin of a given dataset
one unique id for each data row type */
typedef enum _MODULENAME {
  MODULE_NAME_NAV420_SCALED,
  MODULE_NAME_NAV420_ANGLE,
  MODULE_NAME_NAV420_NAV,
  MODULE_NAME_NMEA_RMC,
  MODULE_NAME_NMEA_GGA,
  MODULE_NAME_SMART,
  MODULE_NAME_FIRECAM,
  MODULE_NAME_MULTISICK,
  MODULE_NAME_ROTATINGSICK,
  MODULE_NAME_CALIBRATION = 69,
  MODULE_NAME_OMNIHPPOS,
  MODULE_NAME_NMEA_GSA,
  MODULE_NAME_NMEA_GST,
  MODULE_NAME_DSP3000_RATE,
  MODULE_NAME_DSP3000_INC,
  MODULE_NAME_DSP3000_INT
}MODULENAME;

typedef enum _PROCESSED_DATA{PROC_SMART = 0, PROC_NAV, PROC_RMC, PROC_HPPOS, PROC_GGA, PROC_GSA, PROC_GST, PROC_DSP}PROCESSED_DATA;
typedef enum _FILTER_DATA{DATA_NOT_AVAILABLE = -1, DATA_DISCARDED = -2}FILTER_DATA;

typedef struct _EPOINT2D{
  double x;
  double y;
}EPOINT2D;

typedef struct _EPOINT2D_INT{
  int x;
  int y;
}EPOINT2D_INT;

typedef struct _EPOINT3D{
  double x;
  double y;
  double z;
}EPOINT3D;

typedef struct _EGLOBAL_VEHICLE_STATE {
  double xpos;
  double ypos;
  double zpos;
  double yaw_angle;
  double pitch_angle;
  double roll_angle;
  double xvel;
  double yvel;
  double zvel;
  double yaw_rate;
  unsigned int car_cruising;
  TIMEVAL timestamp;
} EGLOBAL_VEHICLE_STATE;

typedef struct  ELOCAL_VEHICLE_STATE {
  double v;
  double yaw_rate;
  double pitch_rate;
  double roll_rate;
  double steering_angle;
  TIMEVAL timestamp;
}  ELOCAL_VEHICLE_STATE;

typedef struct _EBOUNDING_BOX
{
  double lx;
  double ux;
  double ly;
  double uy;
}EBOUNDING_BOX;

#endif
