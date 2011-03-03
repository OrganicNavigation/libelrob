#ifndef QUATERNION_H
#define QUATERNION_H

#define Q_DUMP_QUATERNION(q) (printf("Quaternion (w,x,y,z): %f %f %f %f\n", q.w, q.x, q.y, q.z))
#define Q_DUMP_EULER(e) (printf("Euler angles: (yaw, pitch, roll)  %f %f %f\n", e.yaw, e.pitch, e.roll))
#define Q_DUMP_AXIS(a) (printf("Axis (x,y,z): %f %f %f\n", a.x, a.y, a.z))

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

void Q_mat_four_by_four_from_three_by_three(double *mat3, double *mat4);
void Q_quaternion_from_axis_angle(Q_Axis a, double angle, Q_Quaternion *q);
void Q_axis_angle_from_quaternion(Q_Quaternion, Q_Axis *a, double *angle);
void Q_quaternion_from_rotation_matrix(double *mat, Q_Quaternion *q);
void Q_multiply(Q_Quaternion a, Q_Quaternion b, Q_Quaternion *c);
void Q_normalize(Q_Quaternion *q);
void Q_normalize_axis(Q_Axis *a);
void Q_rotate_point(Q_Quaternion q, Q_point *p);
void Q_get_quaternion(double yaw, double pitch, double roll, Q_Quaternion *q);

#endif
