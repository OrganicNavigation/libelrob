/***************************************************************************
                        Esmart.h  -  description
                            -------------------
  begin                : March 2005
  copyright            : (C) 2005 by Pierre Lamon
  email                : pierre.lamon@epfl.ch

  description          : constants for the SmartTer
 ***************************************************************************/
#ifndef ESMART_H
#define ESMART_H

#define DIM_REAR_FRONT_AXIS    1.812    /* distance between the rear axis and the front axis */
#define WHEEL_STEER_RATIO      28.55    /* ratio between steering wheel angle and wheels angle */

#define ENCODER_LOW_THRESH     0.764    /* encoder becomes active if the car drives faster than this threshold */

//offroad wheels
//#define ODO_SCALE_FACTOR       1.107     /* 1.15 scale factor due to wheel diameter change (to set it compare gps traj and pure odometry) */

#define ODO_SCALE_FACTOR       1.0    
#define ODO_YAW_OFFSET         0.0      /* heading angle correction for the odometry (the car does not move perfectly straight) */

#define IMU_PITCH_OFFSET       (0.011)  /* to calibrate this make a loop and compute pure odometry (with IMU_PITCH_OFFSET set to zero). compute the path length and see the z coord at the end pitch_offset = asin(dz/plength) */

#define SMART_WHEEL_RADIUS     (0.32)
#define RMC_SPEED_VALID_THRESH 0.25      /* speed over this are considered as valid */

#define DGPS_ANTENNA_POS_X     0.35     /* position of the DGPS antenna in the car frame */
#define DGPS_ANTENNA_POS_Y     0.4
#define DGPS_ANTENNA_POS_Z     1.845

#endif
