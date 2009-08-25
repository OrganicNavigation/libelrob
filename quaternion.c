/*! \brief Quaternion library
  \author Sascha Kolski, ASL

  Defines functions ad types to realize rotations using quaternions. The big advantage of using quaternions for rotations is that there are no singularities or gimbal lock.
The easiest way to use this library is to call the function Q_get_quarternion with the yaw, pitch and roll angle of you object. This will give you a quarternion wich you can use to rotate points using the function Q_rotate_point with the point to rotate and the quarternion describing the rotation.
*/
#include <math.h>
#include "stdio.h"
#include "quaternion.h"

/*! \brief Creates a quaternion from an axis-angle representation (the rotation axis and the angle you want to turn arround)*/
void Q_quaternion_from_axis_angle(Q_Axis a, double angle, Q_Quaternion *q)
{
  q->w =  cos(angle/2);
  q->x = a.x*sin(angle/2);
  q->y = a.y*sin(angle/2);
  q->z = a.z*sin(angle/2);
}

/*! \brief Returns the axis-angle representation of the rotation realized by the quaternion */
void Q_axis_angle_from_quaternion(Q_Quaternion q, Q_Axis *a, double *angle)
{
  if ((q.w == 0)||(q.w == 1)){
    printf("Warning: Axis not defined with angle 0, setting to [1,0,0]\n");
    a->x = 1;
    a->y = 0;
    a->z = 0;
    angle = 0;
  }
  else{
    *angle = 2 *acos(q.w);
    double sqw =  sqrt(1- q.w*q.w);
    a->x = q.x / sqw;
    a->y = q.y / sqw;
    a->z = q.z / sqw;
  }
}
void Q_mat_four_by_four_from_three_by_three(double *mat3, double *mat4){
  mat4[0]=mat3[0];
  mat4[1]=mat3[3];
  mat4[2]=mat3[6];
  mat4[3]=0;
  mat4[4]=mat3[1];
  mat4[5]=mat3[4];
  mat4[6]=mat3[7];
  mat4[7]=0;
  mat4[8]=mat3[2];
  mat4[9]=mat3[5];
  mat4[10]=mat3[8];
  mat4[11]=0;
  mat4[12]=0;
  mat4[13]=0;
  mat4[14]=0;
  mat4[15]=1;
}

/*! \brief Quaternion from rotation matrix */
void Q_quaternion_from_rotation_matrix(double *mat, Q_Quaternion *q)
{
  //calculate trace
  double S;
  double T = 1 + mat[0] + mat[5] + mat[10];

  if ( T > 0.00000001 ){ //to avoid large distortions!
    S = sqrt(T) * 2;
    q->x = ( mat[9] - mat[6] ) / S;
    q->y = ( mat[2] - mat[8] ) / S;
    q->z = ( mat[4] - mat[1] ) / S;
    q->w = 0.25 * S;
  }
  else
    {
      if ( mat[0] > mat[5] && mat[0] > mat[10] )  {	// Column 0: 
        S  = sqrt( 1.0 + mat[0] - mat[5] - mat[10] ) * 2;
        q->x = 0.25 * S;
        q->y = (mat[4] + mat[1] ) / S;
        q->z = (mat[2] + mat[8] ) / S;
        q->w = (mat[9] - mat[6] ) / S;
      } else if ( mat[5] > mat[10] ) {			// Column 1: 
        S  = sqrt( 1.0 + mat[5] - mat[0] - mat[10] ) * 2;
        q->x = (mat[4] + mat[1] ) / S;
        q->y = 0.25 * S;
        q->z = (mat[9] + mat[6] ) / S;
        q->w = (mat[2] - mat[8] ) / S;
      } else {						// Column 2:
        S  = sqrt( 1.0 + mat[10] - mat[0] - mat[5] ) * 2;
        q->x = (mat[2] + mat[8] ) / S;
        q->y = (mat[9] + mat[6] ) / S;
        q->z = 0.25 * S;
        q->w = (mat[4] - mat[1] ) / S;
      }
    }
 
}

/*! \brief Get a rotation matrix from Quaternion */ 
void Q_rotation_matrix_from_quaternion(Q_Quaternion *q, double *mat){
  double xx      = q->x * q->x;
  double xy      = q->x * q->y;
  double xz      = q->x * q->z;
  double xw      = q->x * q->w;
  double yy      = q->y * q->y;
  double yz      = q->y * q->z;
  double yw      = q->y * q->w;
  double zz      = q->z * q->z;
  double zw      = q->z * q->w;

  mat[0]  = 1 - 2 * ( yy + zz );
  mat[1]  =     2 * ( xy - zw );
  mat[2]  =     2 * ( xz + yw );
  mat[4]  =     2 * ( xy + zw );
  mat[5]  = 1 - 2 * ( xx + zz );
  mat[6]  =     2 * ( yz - xw );
  mat[8]  =     2 * ( xz - yw );
  mat[9]  =     2 * ( yz + xw );
  mat[10] = 1 - 2 * ( xx + yy );
  mat[3]  = mat[7] = mat[11] = mat[12] = mat[13] = mat[14] = 0;
  mat[15] = 1;
  
  //  The resulting matrix uses the following positions:
  //    ¦ mat[0]  mat[4] mat[ 8] mat[12] ¦
  //M = ¦ mat[1]  mat[5] mat[ 9] mat[13] ¦
  //    ¦ mat[2]  mat[6] mat[10] mat[14] ¦
  //    ¦ mat[3]  mat[7] mat[11] mat[15] ¦
}


/* \brief Multiplies to quaternions */
void Q_multiply(Q_Quaternion a, Q_Quaternion b, Q_Quaternion *q){

  q->w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  q->x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  q->y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  q->z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

}

/*! \brief Normalizes a quaternion to a unit-quaternion */
void Q_normalize(Q_Quaternion *q){
  double magnitude = sqrt(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  printf("magnitude is %f \n", magnitude);
  q->w = q->w/magnitude;
  q->x = q->x/magnitude;
  q->y = q->y/magnitude;
  q->z = q->z/magnitude;
}

/*! \brief creates a quaternion from the commonly used (yaw, pitch, roll) description of a rotation */
void Q_get_quaternion(double yaw, double pitch, double roll, Q_Quaternion *q)
{
  Q_Quaternion q_yaw, q_pitch, q_roll, q_return, q1;

  //Initialize Axis
  Q_Axis a;

  a.x = 0;
  a.y = 1;
  a.z = 0;

  Q_quaternion_from_axis_angle(a, pitch, &q_pitch);

  a.x = 0;
  a.y = 0;
  a.z = 1;

  Q_quaternion_from_axis_angle(a, yaw, &q_yaw);

  a.x = 1;
  a.y = 0;
  a.z = 0;

  Q_quaternion_from_axis_angle(a, roll, &q_roll);

  Q_multiply(q_yaw, q_pitch, &q1);
  Q_multiply(q1, q_roll, &q_return);

  *q = q_return;
}
/*! \brief Conjugates (inverse) a quaternion */
void Q_conjugate(Q_Quaternion *q)
{
  q->w = q->w;
  q->x = -q->x;
  q->y = -q->y;
  q->z = -q->z;
}

/*! \brief Rotates a point as described by the quaternion */
void Q_rotate_point(Q_Quaternion q, Q_point *p)
{
  Q_Quaternion qcon, qconcon, qv, q1, q2;

  qcon = q;
  Q_conjugate(&qcon);
  qconcon=qcon;
  Q_conjugate(&qconcon);

  /* See the point as a quaternion */
  qv.w = 0;
  qv.x = p->x;
  qv.y = p->y;
  qv.z = p->z;

  Q_multiply(qv, qcon, &q1);
  Q_multiply(qconcon, q1, &q2);

  p->x = q2.x;
  p->y = q2.y;
  p->z = q2.z;
}


