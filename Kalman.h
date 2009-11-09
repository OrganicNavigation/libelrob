/***************************************************************************
                          Kalman.h  -  description
                             -------------------
    begin                : Fri Oct 25 2002
    copyright            : (C) 2002 by Pierre Lamon
    email                : plamon@lsa1pc29
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _KALMAN_H
#define _KALMAN_H

#include "Kmatrix.h"
#include "Emacros.h"

#ifndef Max
#define Max(x, y) (( (x) > (y) ) ? (x) : (y))
#endif
#ifndef Min
#define Min(x, y) (( (x) < (y) ) ? (x) : (y))
#endif

/*#define VERBOSE	1*/
#define MAX_SENSORS 8
#define USE_SENSOR(sensor, mask) ((1<<(sensor)) & (mask))

typedef enum {FM_STOP, FM_ACCEL, FM_CST_SPEED, FM_DECEL, FM_INIT, FM_STOP_END} FM_MODES;

typedef struct _FilterStorageElement {
	struct timeval TimeStamp;
    KMATRIX_TYPE	*pMatricesArray;
}FilterStorageElement;

/* WARNING !!! Never use nCol and nRow of the Tmp matrix for setting lda, ldb and ldc matrices. Tmp is only used for its Element field*/

typedef struct _KFilter
{
  int	nx;   	/* Size of the state vector (x) */
  int	nu;   	/* Size of the input vector (u) */
  int	nz;      	/* Size of the measurment vector (z) */
  int	nmax;		/* Max dimension nmax = max (nx, nz, nu);*/
  int useG;	      /* Says if there is an input */

  int	isEKF;	/* Says if it is an Extended Kalman Filer*/
  void	*EKF_Params;		/* Pointer to a structure containing all parameters needed for computing EKF F and H matrices*/
  void (*ComputeEKF_F_Matrix)  (struct _KFilter *);   /* A pointer to the function used for computing the F matrix for the EKF case*/
  void (*ComputeEKF_H_Matrix)  (struct _KFilter *);  	/* A pointer to the function used for computing the H matrix for the EKF case*/
  void (*StatePred)  (struct _KFilter *);  		/* A pointer to the function used for computing the nonlinear state prediction*/
  void (*MeasurementPred)  (struct _KFilter *);  /* A pointer to the function used for computing the nonlinear measurement prediction*/

  KMatrix u;   	/* The known input vector u(k)		: (nu x 1)*/
  KMatrix z;  	/* The measurement vector z(k+1) also temporary used for storing nu(k+1)	: (nz x 1)*/
  KMatrix pz;  	/* The measurement prediction vector z^(k+1|k)	: (nz x 1)*/
  KMatrix ex;   	/* The estimated state vector x^(k|k)		: (nx x 1)*/
  KMatrix px;  	/* The state prediction vector x^(k+1|k) 	: (nx x 1)*/

  KMatrix F;   	/* F(k)		State transition matrix (nx x nx)*/
  KMatrix G;   	/* G(k)		G matrix (nx x nu)*/
  KMatrix H;     	/* H(k+1)  		Observation matrix (nz x nx)*/
  KMatrix P;     	/* P(k|k)   	State covariance matrix (nx x nx)*/
  KMatrix pP;	/* P(k+1|k)		State prediction cov P(k+1|k)	: (nx x nx)*/
  KMatrix pPOnly;	/* Reflects the state prediction covariance only, not cumulated*/
  KMatrix Q; 	/* Q(k)    		Process noise (nx x nx)*/
  KMatrix R;     	/* R(k)    		Measurement noise (nz x nz)*/
  KMatrix S;     	/* S(k+1)    	Innovation matrix (nz x nz)*/
  KMatrix iS;	/* iS(k+1)		Inverse of S*/
  KMatrix W;     	/* W(k+1)   	Filter Gain (nx x nz)*/
  KMatrix	Tmp;	/* Temporary matrix for internal use only (max(nx,nu,nz) x max(nx,nu,nz))*/

  struct timeval	TimeStamp;
}KFilter;

typedef struct _InformationFilter
{
  int	ny;   	/* Size of the state vector (y). nx = ny*/
  int	nz;      	/* Size of the measurment vector (z)*/
  int	nmax;		/* Max dimension nmax = max (nx, nz, nu);*/

  int	isEIF;	/* Says if it is a non-linear Information Filer*/
  int   isRStatic;  /* If isRStatic == 0 then !R is computed at each iteration else !R is computed once at the initialization*/
  void		*EIF_Params;		/* Pointer to a structure containing all parameters needed for computing EKF F and H matrices*/
  void (*ComputeEIF_F_Matrix)  (struct _InformationFilter *);    /* A pointer to the function used for computing the F matrix for the EIF case*/ /* F-state prediction matrix */
  void (*ComputeEIF_HandZprim)  (struct _InformationFilter *);  /* A pointer to the function used for computing the H matrix for the EIF case*/ /* H - output matrix */
  void (*StatePred)  (struct _InformationFilter *);  	/* A pointer to the function used for computing the nonlinear state prediction*/ /* the process model */
  void (*MeasurementPred)  (struct _InformationFilter *);  	/* A pointer to the function used for computing the nonlinear measurement prediction*/ /*the nonlinear output map */

  KMatrix z;  	/* The measurement vector z(k) : (nz x 1)*/
  KMatrix ey;   	/* The estimated information state vector y^(k|k)		: (ny x 1)*/
  KMatrix ex;   	/* The estimated state vector x^(k|k)=P(k|k) y^(k|k)		: (ny x 1)*/
  KMatrix py;  	/* The information state prediction vector y^(k+1|k) 	: (ny x 1)*/
  KMatrix px;   	/* The state vector prediction x^(k|k-1)= P(k|k-1) y^(k|k-1)		: (ny x 1)*/

  /* [km] the state can be calculated from the information matrix, or the covariance matrix as well */

  KMatrix F;   	/* F(k)	  State transition matrix (ny x ny)*/
  KMatrix H;     	/* H(k)   	  Observation matrix (nz x ny)*/
  KMatrix P;     	/* P(k|k)     State covariance matrix (ny x ny), covariance of x*/
  KMatrix iP;     /* !P(k|k)    Inverse State covariance matrix (ny x ny), covariance of y*/ /*[km] information matrix */
  KMatrix pP;	/* P(k|k-1)	  State prediction cov P(k|k-1)	: (ny x ny) (information vector) */
  KMatrix ipP;    /* !P(k|k-1)  Inverse of pP */
  KMatrix Q; 	/* Q(k)    	  Process noise (ny x ny)*/
  KMatrix R;     	/* R(k)    	  Measurement noise (nz x nz)*/
  KMatrix iR;     /* !R(k)    	  Inverse of the measurement noise (nz x nz)*/
  KMatrix Tmp;	/* Temporary matrix for internal use only (max(ny,nz) x max(ny,nz))*/
}InformationFilter;

/* km, the sensor information is actually containing the data
   and the variance structures (R,H). The measurement itself
   can be found in the z KMatrix.

   The SensorInformation contains structures that can be computed
   a-priori in an off-line fashion.

   Question: why is the measurement vector z included in this
   process? */
typedef struct _SensorInformation {
  int		nz;
  int		isNonLinear;
  int		isRStatic;	/* if isRStatic != 0 then compute R and iR only once*/
  KMatrix	z;
  KMatrix	R;
  KMatrix	iR;
  KMatrix	H;
  KMatrix	Ht_iR;	/* Result of Ht !R, for internal purpose*/
  KMatrix	zp;		/* z prime*/
  int (*Compute_H_R_zp) (void *pFilter);	/* For computing Ht, !R and z_prime*/
  char	name[256];       /* Name of the sensor */
  char      meas_descr[256]; /* measurements description*/
}SensorInformation;


/* [km] the MultisensorInfoFilter is the workhorse of data fusion
   -> also contains pointers to functions calculating the specific structures.
   The whole concept is based on information filter type */
typedef struct _MultiSensorInfoFilter
{
  int	sensorNr;					/* Number of sensors*/
  SensorInformation	Sensors[MAX_SENSORS];	/* Array containing sensors structures*/

  int	 ny;   		/* Size of the state vector (y). nx = ny*/
  int  useG;            /* if we use Q = G q G'*/
  int  nu;              /* size of q, number of inputs */
  void *Params;		/* Pointer to a structure containing all specific parameters needed for the filter*/
  void (*Compute_F_Matrix)  (struct _MultiSensorInfoFilter *);    	/* A pointer to the function used for computing the F(k) matrix*/
  void (*Compute_Q_Matrix)  (struct _MultiSensorInfoFilter *);    	/* A pointer to the function used for computing the Q(k) matrix*/
  void (*Compute_NonLinear_Prediction)  (struct _MultiSensorInfoFilter *, KMatrix *predx);

  KMatrix ey;   	/* The estimated information state vector y^(k|k)	                   : (ny x 1)*/
  KMatrix ex;   	/* The estimated state vector x^(k|k) = P(k|k) y^(k|k)       : (ny x 1)*/
  KMatrix py;  	/* The information state prediction vector y^(k|k-1) 	       : (ny x 1)*/
  KMatrix px;   	/* The state vector prediction x^(k|k-1)= P(k|k-1) y^(k|k-1) : (ny x 1)*/

  KMatrix F;   	/* F(k)	  State transition matrix (ny x ny)*/
  KMatrix P;     	/* P(k|k)     State covariance matrix (ny x ny)*/
  KMatrix iP;     /* !P(k|k)    Inverse State covariance matrix (ny x ny)*/
  KMatrix pP;     /* P(k|k-1)	  State prediction cov P(k|k-1)	: (ny x ny)*/
  KMatrix ipP;    /* !P(k|k-1)  Inverse of pP*/
  KMatrix G;      /* G(k)       Map the noise G q G' (ny x nu) */
  KMatrix q;      /* q(k)       Map the noise G q G' (nu x nu) */
  KMatrix Q; 	/* Q(k)    	  Process noise (ny x ny)*/
  KMatrix upd_y;	/* sum of [Ht(j) !R(j) zp(j)], (ny x 1),  j = 0..sensorNr*/
  KMatrix upd_invP;	/* sum of [Ht(j) !R(j) H(j)], (ny x ny)*/
  KMatrix Tmp;		/* Temporary matrix for internal use only (ny x ny)*/
}MultiSensorInfoFilter;

/* Storage functions for the KalmanFilter. It is used to store the Kalman filter state
 in order to sync data. The reprocess uses a stored state as a starting point*/
int	KalmanCreateStorageBuffer (int maxSize, KFilter	*pFilter);
void	KalmanFreeStorageBuffer();
int	KalmanSaveFilter (KFilter	*pFilter);
int	KalmanRestoreFilter (int idx, KFilter	*pFilter);
int	KalmanReadNextFilter (KFilter	*pFilter);
int	KalmanGetNumberOfStoredElement ();
int	KalmanGetCurrentStorageIndex ();
void	KalmanRewindStorageBuffer ();
void	KalmanResetStorageBuffer ();
int 	KalmanGetClosestFilterIndex (struct timeval *timestamp);
void	KalmanTestStorageFunctions();
void  KalmanPrintAll (KFilter *pFilter);
void  Compute_Q_GaussMarkov (double variances[6], double var, double beta, double h);

/* Main functions for the Kalman filter*/
int KalmanInit (KFilter	*pFilter, int dx, int du, int dz, int useG, int isEKF, void (*pF_function)  (struct _KFilter *), void (*stateP_function)  (struct _KFilter *), void (*pH_function)  (struct _KFilter *), void (*measP_function)  (struct _KFilter *), void *EKF_params);
void KalmanFree (KFilter *pFilter);
void KalmanZeropPOnly (KFilter *pFilter);
void KalmanStatePredictionCovOnly (KFilter *pFilter);
void KalmanStatePrediction(KFilter	*pFilter);
void KalmanStateUpdate(KFilter	*pFilter);

/* Main functions for the information filter*/
void InformationFilterPrintAll (InformationFilter *pFilter);
int InformationFilterInit (InformationFilter	*pFilter, int dy, int dz, int isRStatic, int isEIF, void (*pF_function)  (struct _InformationFilter *), void (*stateP_function)  (struct _InformationFilter *), void (*pH_function)  (struct _InformationFilter *), void (*measP_function)  (struct _InformationFilter *), void *EIF_params);
void InformationFilterFree (InformationFilter	*pFilter);
void InformationFilterStatePrediction(InformationFilter	*pFilter);
void InformationFilterStateUpdate(InformationFilter	*pFilter);

/* MultiSensorInfoFilter functions*/
void PrintSensorInfo (SensorInformation *pInfo);
void SensorAllocMatrices (MultiSensorInfoFilter	*pFilter, int sensor_id);
void SensorFreeMatrices (MultiSensorInfoFilter	*pFilter, int sensor_id);
void MultiSensorFilterInit (MultiSensorInfoFilter	*pFilter, int ny, int sensorNr, int useG, int nu, void (*pF_function)  (MultiSensorInfoFilter *), void (*pQ_function)  (MultiSensorInfoFilter *), void (*pf_function)  (MultiSensorInfoFilter *, KMatrix *predx), void *params);
SensorInformation *GetSensorInfoPtr (MultiSensorInfoFilter	*pFilter, int idx);
void MultiSensorFilterFree (MultiSensorInfoFilter	*pFilter);
void MultiSensorFilterStatePrediction (MultiSensorInfoFilter *pFilter);
int MultiSensorFilterStateUpdate (MultiSensorInfoFilter *pFilter, unsigned char sensorMask);


#endif
