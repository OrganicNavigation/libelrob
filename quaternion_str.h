#ifndef QUATERNION_STR_H
#define QUATERNION_STR_H
 
typedef  struct _Q_Quaternion{
  double w;
  double x;
  double y;
  double z;
}Q_Quaternion;

typedef struct _Q_Euler{
  double yaw;
  double pitch;
  double roll;
}Q_Euler;

typedef struct _Q_Axis{
  double x;
  double y;
  double z;
}Q_Axis;

typedef struct _Q_point{
  double x;
  double y;
  double z;
}Q_point;

typedef double Q_Matrix44[16];
#endif
