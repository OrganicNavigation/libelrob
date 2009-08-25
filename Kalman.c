/***************************************************************************
                          Kalman.c  -  description
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
#include "Kalman.h"
#include <string.h>

#define  TEST_VERBOSE 0

// Storage private variables
void	*mk_pStorage_buffer = NULL;  	// Array of element (filters)
int	mk_sizeofElement = 0;          			// Total size, in bytes, of one element of the array
int	mk_maxStorageBufferSize = 0;   		// Max number of steps that can be stored
int	mk_NumberOfElements = 0; 			// The actual number of stored elements
int	mk_currentStorageIndex = 0; 			// Actual index

void KalmanZeropPOnly (KFilter	*pFilter)
{
	ZeroMatrix (&pFilter->pPOnly);
}

void  KalmanFree (KFilter	*pFilter)
{
  DeleteMatrix (&pFilter->ex);
  DeleteMatrix (&pFilter->u);
  DeleteMatrix (&pFilter->z);
  DeleteMatrix (&pFilter->pz);
  DeleteMatrix (&pFilter->ex);
  DeleteMatrix (&pFilter->px);
  DeleteMatrix (&pFilter->F);
  DeleteMatrix (&pFilter->G);
  DeleteMatrix (&pFilter->H);
  DeleteMatrix (&pFilter->P);
  DeleteMatrix (&pFilter->pP);
  DeleteMatrix (&pFilter->pPOnly);
  DeleteMatrix (&pFilter->Q);
  DeleteMatrix (&pFilter->R);
  DeleteMatrix (&pFilter->S);
  DeleteMatrix (&pFilter->iS);
  DeleteMatrix (&pFilter->W);
  DeleteMatrix (&pFilter->Tmp);
#ifdef RTAI
    kfree (&pFilter->iPiv);
#else
	free (&pFilter->iPiv);
#endif
}

// Alloc all matrices and vectors for the Kalman filter. All values are set to zero
// KFilter	*pFilter	: a pointer on the structure you want to init
//	int dx					: dimension of the state vector
//	int du					: dimension of the known input vector
//	 int dz					: dimension of the measurment vector
//	 int useG			: says if the inputs are used
//  int isEKF  			: says if it is an Extended Kalman Filter
//  pF_function		: pointer on the function which will compute the F matrix (only if isEKF = 1)
//  stateP_function :  pointer to the function that does non linear state prediction. If NULL then linear system assumed
//  pH_function		: same for H
//  measP_function : same for measurement prediction
//  void *EKF_params	: pointer on a structure which contains all needed specific parameters for computing H and F in the EKF case
int KalmanInit (KFilter	*pFilter, int dx, int du, int dz, int useG, int isEKF, void (*pF_function)  (struct _KFilter *), void (*stateP_function)  (struct _KFilter *), void (*pH_function)  (struct _KFilter *), void (*measP_function)  (struct _KFilter *), void *EKF_params)
{
	// Init the main dimensions of the system
	pFilter->nx = dx;		pFilter->nu = du;		pFilter->nz = dz;
	pFilter->nmax = Max (pFilter->nx, pFilter->nz);
	pFilter->nmax = Max (pFilter->nmax, pFilter->nu);
	pFilter->isEKF = isEKF;
	pFilter->useG = useG;

	pFilter->EKF_Params = EKF_params;

	if (pFilter->isEKF)	{
		pFilter->ComputeEKF_F_Matrix =  pF_function;
		pFilter->ComputeEKF_H_Matrix =  pH_function;
		pFilter->StatePred = stateP_function;
		pFilter->MeasurementPred = measP_function;
	}
	else	{
	 	pFilter->ComputeEKF_F_Matrix =  NULL;
		pFilter->ComputeEKF_H_Matrix =  NULL;
		pFilter->StatePred = NULL;
		pFilter->MeasurementPred = NULL;
	}

	// Alloc vectors
	AllocMatrix (&pFilter->ex, pFilter->nx, 1);     					// Init the state vector
	AllocMatrix (&pFilter->px, pFilter->nx, 1);     					// Init the state prediction vector
	AllocMatrix (&pFilter->u, pFilter->nu, 1); 						// Init the known input vector
	AllocMatrix (&pFilter->z, pFilter->nz, 1); 						// Init the measurement vector
	AllocMatrix (&pFilter->pz, pFilter->nz, 1); 						// Init the measurement prediction vector

#ifdef RTAI
	pFilter->iPiv = (int *) kmalloc (pFilter->nmax, sizeof (int));
#else
	pFilter->iPiv = (int *) calloc (pFilter->nmax, sizeof (int));

	if (pFilter->iPiv == NULL)	{
  		fprintf (stderr, "KalmanAllocMatrix : Not enough memory !\n");
		exit (-1);
	}
#endif

	// Alloc matrices
	AllocMatrix (&pFilter->F, pFilter->nx, pFilter->nx);				// Init the state transition matrix
	AllocMatrix (&pFilter->H, pFilter->nz, pFilter->nx); 				// Init the observation matrix
	AllocMatrix (&pFilter->P, pFilter->nx, pFilter->nx);   			// Init the state cov matrix
	AllocMatrix (&pFilter->pP, pFilter->nx, pFilter->nx);   			// Init the state prediction cov matrix
	AllocMatrix (&pFilter->pPOnly, pFilter->nx, pFilter->nx);
	AllocMatrix (&pFilter->Q, pFilter->nx, pFilter->nx);  				// Init the process noise matrix
	AllocMatrix (&pFilter->R, pFilter->nz, pFilter->nz);				// Init the measurement noise matrix
	AllocMatrix (&pFilter->S, pFilter->nz, pFilter->nz);  				// Init the Innovation matrix
	AllocMatrix (&pFilter->iS, pFilter->nz, pFilter->nz);  			// Init the inverse matrix
	AllocMatrix (&pFilter->W, pFilter->nx, pFilter->nz); 				// Init the Kalman gain matrix
	AllocMatrix (&pFilter->Tmp, pFilter->nmax, pFilter->nmax);		// Init the Tmp matrix

	// Init the G matrix if necessary
	if (pFilter->useG) AllocMatrix (&pFilter->G, pFilter->nx, pFilter->nu);
	else	{
		pFilter->G.nRow	= 0;		pFilter->G.nCol	= 0;
		pFilter->G.Element = NULL;
	}
 	return	0;
}

// ------------------- Storage functions for the KalmanFilter -----------------------------------------
// H(k+1), R(k), P(k), P(k+1|k), z (k+1|k), x(k+1|k) , pPOnly are saved
// (nz x nx), (nz x nz), (nx x nx), (nx x nx), nz, nx
// maxSize : max number of element available in the buffer
// pFilter : pointer on a Kalman Filter
// return -1 on success else 0
int	KalmanCreateStorageBuffer (int maxSize, KFilter	*pFilter)
{
	if (mk_pStorage_buffer != NULL) return 0; 	// The buffer has already been allocated

	mk_maxStorageBufferSize = maxSize;
	mk_NumberOfElements = 0;
	mk_currentStorageIndex = 0;
	mk_sizeofElement = sizeof (FilterStorageElement) +  (pFilter->nz * pFilter->nx + pFilter->nz * pFilter->nz + 3 * pFilter->nx * pFilter->nx + pFilter->nz + pFilter->nx) * sizeof(KMATRIX_TYPE);

	mk_pStorage_buffer = malloc (mk_maxStorageBufferSize * mk_sizeofElement);
	if (mk_pStorage_buffer == NULL) perror ("KalmanCreateStorageBuffer -> Not enough memory m_pStorage_buffer\n");
	return -1;
}

// Frees allocated buffer
void	KalmanFreeStorageBuffer ()
{
 	if (mk_pStorage_buffer != NULL) {
 		free (mk_pStorage_buffer);
 		mk_pStorage_buffer = NULL;
 		mk_maxStorageBufferSize = 0;
		mk_NumberOfElements = 0;
		mk_currentStorageIndex = 0;
		mk_sizeofElement = 0;
	}
}

// Copy the matrix element array at the first free space of the storage buffer
// Return the size of the matrix (used for computing the next available free chunk of memory)
int	KalmanSaveMatrix (KMatrix *pMatrix, KMATRIX_TYPE *pMatricesArray, int offset)
{
	int	increment = pMatrix->nRow * pMatrix->nCol;   // Number of bytes to copy

	memcpy (&pMatricesArray[offset], pMatrix->Element, increment  * sizeof(KMATRIX_TYPE));
	return increment;
}

// Restore the matrix element array  (reads from the storage buffer)
// idx : element index (filter index)
// offset : offset from the first byte of the idx'the element
// pMatrix : pointer on the matrix we want to restore
int	KalmanRestoreMatrix (int idx, KMATRIX_TYPE *pMatricesArray, int offset, KMatrix *pMatrix)
{
	int	increment = pMatrix->nRow * pMatrix->nCol;

	memcpy (pMatrix->Element, &pMatricesArray[offset], increment * sizeof(KMATRIX_TYPE));
	return increment;
}

void Bin_Map_Element (int idx, FilterStorageElement **pElement)
{
	// Find the beginning of the structure
	*pElement =  (FilterStorageElement *) ( (unsigned long) mk_pStorage_buffer + idx * mk_sizeofElement);
	//printf (" *pElement rout : %i, idx : %i, mk_pStorage_buffer : %i\n", (unsigned long)  *pElement , idx, mk_pStorage_buffer);
	(*pElement)->pMatricesArray =  (KMATRIX_TYPE *) ( (unsigned long) (*pElement) + sizeof (FilterStorageElement));
}

// Store the filter at the first available position in the storage buffer
// return -1 if ok else return 0
int 	KalmanSaveFilter (KFilter	*pFilter)
{
	int offset = 0;
	FilterStorageElement *pElement;

	if (mk_NumberOfElements > mk_maxStorageBufferSize-1) {
		printf ("KalmanSaveFilter -> Buffer not large enough !\n");
		return 0;
	}

	Bin_Map_Element (mk_NumberOfElements, &pElement);
	pElement->TimeStamp = pFilter->TimeStamp;

	offset += KalmanSaveMatrix (&pFilter->H, pElement->pMatricesArray, 0);
	offset += KalmanSaveMatrix (&pFilter->R, pElement->pMatricesArray, offset);
	offset += KalmanSaveMatrix (&pFilter->P, pElement->pMatricesArray, offset);
	offset += KalmanSaveMatrix (&pFilter->pP, pElement->pMatricesArray, offset);
	offset += KalmanSaveMatrix (&pFilter->pz, pElement->pMatricesArray, offset);
	offset += KalmanSaveMatrix (&pFilter->px, pElement->pMatricesArray, offset);
	offset += KalmanSaveMatrix (&pFilter->pPOnly, pElement->pMatricesArray, offset);

	mk_NumberOfElements++;
	//mk_currentStorageIndex = mk_NumberOfElements;
	return -1;
}

// Reads the idx th element from the storage buffer
// return -1 if ok else return 0
int	KalmanRestoreFilter (int idx, KFilter	*pFilter)
{
	int offset = 0;
	FilterStorageElement *pElement;

	if ((idx > mk_NumberOfElements-1) || (idx < 0)) {
		return 0;
	}

	Bin_Map_Element (idx, &pElement);
	pFilter->TimeStamp = pElement->TimeStamp;

	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray, 0, &pFilter->H);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->R);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->P);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->pP);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->pz);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->px);
	offset += KalmanRestoreMatrix (idx, pElement->pMatricesArray,offset, &pFilter->pPOnly);

	return -1;
}

int KalmanGetTimeStamp (int idx, struct timeval *pTstamp)
{
 	FilterStorageElement *pElement;

	if (idx > mk_NumberOfElements-1) {
		return 0;
	}

	Bin_Map_Element (idx, &pElement);
	*pTstamp = pElement->TimeStamp;
	//printf ("KalmanGetTimeStamp ->Timestamp [%i, %i], idx : %i\n", (*pTstamp).tv_sec, (*pTstamp).tv_usec, idx);
	//printf ("DoubleOf (*pTstamp) %f\n", DoubleOf(pElement->TimeStamp) );
	return -1;
}

// Read the next element in the storage buffer
// return -1 if ok else return 0
int	KalmanRestoreNextFilter (KFilter	*pFilter)
{
  return KalmanRestoreFilter (mk_currentStorageIndex++, pFilter);
}

// Return the number of elements that have been stored until now
int	KalmanGetNumberOfStoredElement () {
	return mk_NumberOfElements;
}

// Return the current index
int	KalmanGetCurrentStorageIndex () {
 	return  mk_currentStorageIndex;
}

// Set the current index to point at the beginning of the buffer
void	KalmanRewindStorageBuffer () {
 	mk_currentStorageIndex = 0;
}

// Set the current index to point at the beginning of the buffer
// and empty the buffer (mk_NumberOfElements = 0)
void	KalmanResetStorageBuffer () {
 	mk_currentStorageIndex = 0;
 	mk_NumberOfElements = 0;
}

// Check which filter is chronologically closest to refTimeStamp
// return the index or mk_NumberOfElements all timestamps are higher than refTimeStamp and -1 if there is an error
/*int KalmanGetClosestFilterIndex (struct timeval *refTimeStamp)
{
	struct timeval tmp;
	//double	refTimeStamp
	int	idx = 0;
	// Start from the beginning
	KalmanRewindStorageBuffer();
	//Print_TimeVal (refTimeStamp, "refTimeStamp\n");

	while (KalmanGetTimeStamp(idx, &tmp)) {
		printf ("idx : %i, DoubleOf(tmp) %f, DoubleOf(refTimeStamp) %f\n", idx, DoubleOf(tmp), DoubleOf(*refTimeStamp));
		//printf ("idx ");		Print_TimeVal (&tmp, "tmp\n");
		if (DoubleOf(tmp) > DoubleOf(*refTimeStamp)) return (idx -1);
		idx++;
	}
	return mk_NumberOfElements-1;
}*/

// Only print the matrices that have been stored
void KalmanPrint_Stored_Matrices (KFilter *pFilter)
{
	printf ("TimeStamp [%li, %li]\n", pFilter->TimeStamp.tv_sec, pFilter->TimeStamp.tv_usec);
 	PrintMatrix(&pFilter->H, "H");
 	PrintMatrix(&pFilter->R, "R");
 	PrintMatrix(&pFilter->P, "P");
 	PrintMatrix(&pFilter->pP, "pP");
 	PrintMatrix(&pFilter->pz, "pz");
 	PrintMatrix(&pFilter->px, "px");
 	PrintMatrix(&pFilter->px, "pPOnly");
}

// Debug the storage functions
/*void	KalmanTestStorageFunctions()
{
#define	NN_X	2
#define	NN_Z	2

// H(k+1), R(k), P(k), P(k+1|k), z^(k+1|k), x^(k+1|k) are saved
// (nz x nx), (nz x nz), (nx x nx), (nx x nx), nz, nx

	KFilter		KalmanFilter;
	double  H[NN_Z * NN_X] = {1.0,2.0,3.0,4.0};
	double  R[NN_Z * NN_Z] = {5.0,6.0,7.0,8.0};
 	double  P[NN_X * NN_X] = {9.0,10.0,11.0,12.0};
  	double  pP[NN_X * NN_X] = {13.0,14.0,15.0,16.0};
   double  pz[NN_Z] = {17.0,18.0};
   double	  px[NN_X] = {19.0,20.0};
   double	  dtv0 = -1.0;
   struct timeval tv0 = {12, 56};
   int		closestIdx;

	KalmanInit (&KalmanFilter, NN_X, 0, NN_Z, 0, 0, NULL, NULL, NULL, NULL, NULL);

	// Creates the buffer (size 3)
	KalmanCreateStorageBuffer (3, &KalmanFilter);

	SetMatrix (&KalmanFilter.H, H);		SetMatrix (&KalmanFilter.R, R);
	SetMatrix (&KalmanFilter.P, P);		SetMatrix (&KalmanFilter.pP, pP);
	SetMatrix (&KalmanFilter.pz, pz);		SetMatrix (&KalmanFilter.px, px);
	gettimeofday (&KalmanFilter.TimeStamp, 0);
	KalmanPrint_Stored_Matrices (&KalmanFilter);
 	if (KalmanSaveFilter (&KalmanFilter)) 	printf ("Packed 0 written\n");
 	else  printf ("Could not write packet 0\n");

 	MultiplyMatrix (&KalmanFilter.H, 2.0);			MultiplyMatrix (&KalmanFilter.R, 2.0);
	MultiplyMatrix (&KalmanFilter.P, 2.0);			MultiplyMatrix (&KalmanFilter.pP, 2.0);
	MultiplyMatrix (&KalmanFilter.pz, 2.0);		MultiplyMatrix (&KalmanFilter.px, 2.0);
	gettimeofday (&KalmanFilter.TimeStamp, 0);
	gettimeofday (&tv0, 0);
	KalmanPrint_Stored_Matrices (&KalmanFilter);
 	if (KalmanSaveFilter (&KalmanFilter)) 	printf ("Packed 1 written\n");
 	else  printf ("Could not write packet 1\n");

 	SetIdentityMatrix (&KalmanFilter.H);			ZeroMatrix (&KalmanFilter.R);
	ZeroMatrix (&KalmanFilter.P);			ZeroMatrix (&KalmanFilter.pP);
	ZeroMatrix (&KalmanFilter.pz);			ZeroMatrix (&KalmanFilter.px);
	gettimeofday (&KalmanFilter.TimeStamp, 0);
	KalmanPrint_Stored_Matrices (&KalmanFilter);
 	if (KalmanSaveFilter (&KalmanFilter)) printf ("Packed 2 written\n");
 	else  printf ("Could not write packet 2\n");

 	// Idx 0
	if (KalmanRestoreFilter (0, &KalmanFilter)) {
		printf ("Read packet 0\n");
		KalmanPrint_Stored_Matrices (&KalmanFilter);
	} else printf ("Could not read packet 0\n");

	// Idx 1
	if (KalmanRestoreFilter (1, &KalmanFilter)) {
		printf ("Read packet 1\n");
		KalmanPrint_Stored_Matrices (&KalmanFilter);
	} else printf ("Could not read packet 1\n");

	// Idx 2
	if (KalmanRestoreFilter (2, &KalmanFilter)) {
		printf ("Read packet 2\n");
		KalmanPrint_Stored_Matrices (&KalmanFilter);
	} else printf ("Could not read packet 2\n");

	printf  ("Rewind buffer\n");
	KalmanRewindStorageBuffer ();
	printf ("Total number of stored item : %i\n", KalmanGetNumberOfStoredElement());
	printf ("Current idx : %i\n", KalmanGetCurrentStorageIndex());
	while (KalmanRestoreNextFilter(&KalmanFilter)) {
		KalmanPrint_Stored_Matrices (&KalmanFilter);
		printf ("Current idx : %i\n", KalmanGetCurrentStorageIndex());
	}

	closestIdx = KalmanGetClosestFilterIndex (&tv0);
	printf ("The closest filter index is : %i\n", closestIdx);

	printf ("Free storage buffer\n");
	KalmanFreeStorageBuffer ();
}*/

void KalmanPrintAll (KFilter *pFilter)
{
 	PrintMatrix(&pFilter->F, "F");
	PrintMatrix(&pFilter->G, "G");
	PrintMatrix(&pFilter->H, "H");
	PrintMatrix(&pFilter->pP, "pP");
	PrintMatrix(&pFilter->P, "P");
	PrintMatrix(&pFilter->Q, "Q");
	PrintMatrix(&pFilter->R, "R");
	PrintMatrix(&pFilter->S, "S");
	PrintMatrix(&pFilter->W, "W");
	PrintMatrix(&pFilter->u, "u");
	PrintMatrix(&pFilter->ex, "ex");
	PrintMatrix(&pFilter->px, "px");
	PrintMatrix(&pFilter->z, "z");
	PrintMatrix(&pFilter->pz, "pz");
}

// Compute the State Prediction covariance matrix with F, Q and P
// P is updated with FPFt + Q
void	KalmanStatePredictionCovOnly (KFilter	*pFilter)
{
	ABAt_Plus_C (&pFilter->F, &pFilter->pPOnly, &pFilter->Q, &pFilter->pPOnly, &pFilter->Tmp, 'p');
}

void KalmanStatePrediction(KFilter	*pFilter)
{
		// State prediction	---
		 if ( (pFilter->isEKF) && (pFilter->StatePred != NULL) ) (*pFilter->StatePred) (pFilter);    // Use non-linear function for state prediction
		else
			if(pFilter->useG)       // control given
			{
				// x^(k+1|k) = F(k) * x^(k|k) + G(k) * u(k) 	<-> px = F * ex + G * u
				Y_Eq_AX (&pFilter->F, &pFilter->ex, &pFilter->Tmp);											// Tmp = F * ex
			 	Z_Eq_AX_Plus_Y (&pFilter->G, &pFilter->u, &pFilter->Tmp, &pFilter->px);		// px = Tmp + G * u
			}	else
			{
				  // x^(k+1|k) = F * x^(k|k)		<-> px = F * ex
				 Y_Eq_AX (&pFilter->F, &pFilter->ex, &pFilter->px);
			}

		// Compute EKF F matrix if the process is non-linear !!! Before updating ex
  		if (pFilter->isEKF && (pFilter->ComputeEKF_F_Matrix != NULL)) (*pFilter->ComputeEKF_F_Matrix) (pFilter);

		// Compute EKF H matrix if the measurement is non-linear (px = x^(k+1|k) must be computed before calling this function)
		if (pFilter->isEKF && (pFilter->ComputeEKF_H_Matrix != NULL)) (*pFilter->ComputeEKF_H_Matrix) (pFilter);

		// Measurement prediction	---
		// z^(k+1|k) = H(k+1) * x^(k+1|k) <-> pz = H * px
		 if (!pFilter->isEKF) Y_Eq_AX (&pFilter->H, &pFilter->px, &pFilter->pz);
		else if (pFilter->MeasurementPred != NULL) (*pFilter->MeasurementPred) (pFilter);

		// State prediction covariance	---
		// pP = F * P * Ft + Q
		ABAt_Plus_C (&pFilter->F, &pFilter->P, &pFilter->Q, &pFilter->pP, &pFilter->Tmp, 'p');
}

void KalmanStateUpdate(KFilter	 *pFilter)
{
		// Residual covariance  ---
		// S = H * P * Ht + R
		ABAt_Plus_C (&pFilter->H, &pFilter->P, &pFilter->R, &pFilter->S, &pFilter->Tmp, 'p');

		// Filter Gain	---
		// W = eP * Ht * !S
		// ----------------------------------
		CopyRawMatrix (&pFilter->S, &pFilter->iS);  	// They have the same size then copy raw
		ComputeInverse (&pFilter->iS, pFilter->iPiv);

		/* Tmp = Ht * iS  */
		cblas_dgemm (CblasRowMajor, CblasTrans, CblasNoTrans,    // H is transposed
								pFilter->H.nCol, pFilter->iS.nCol, pFilter->H.nRow,
								1.0, pFilter->H.Element, pFilter->H.nCol,
								pFilter->iS.Element, pFilter->iS.nCol,
								0.0, pFilter->Tmp.Element, pFilter->iS.nCol); 			// Tmp is  H->nCol x iS->nCol

		/* W = eP * Temp */
		cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								pFilter->pP.nRow, pFilter->iS.nCol, pFilter->pP.nCol,
								1.0, pFilter->pP.Element, pFilter->pP.nCol,
								pFilter->Tmp.Element, pFilter->iS.nCol,
								0.0, pFilter->W.Element, pFilter->W.nCol);

		// ----------------------------------


    // Measurement residual	---
	// nu(k+1) = z(k+1) - z^(k+1|k) 	<-> z = z - pz
	X_Eq_X_Minus_Y (&pFilter->z, &pFilter->pz);			// At this stage we will use z(k+1) as nu(k+1)

	// Updated state estimate	---
	// x^(k+1|k+1) = x^(k+1|k) + W(k+1) * nu(k+1)   <-> ex = px + W * z
	Z_Eq_AX_Plus_Y (&pFilter->W, &pFilter->z, &pFilter->px, &pFilter->ex);

	// Updated state covariance	---
	// P(k+1|k+1) = P(k+1|k) - W(k+1) * S(k+1) * W(k+1)'   <-> P = ep - W * S * W'
	ABAt_Plus_C (&pFilter->W, &pFilter->S, &pFilter->pP, &pFilter->P, &pFilter->Tmp, 'm');
}

// -------------------------- Information filter specific --------------------------------------------
// Don't forget to call SetupRMatrix afterwards
void InformationFilterPrintAll (InformationFilter *pFilter)
{
 	PrintMatrix(&pFilter->F, "F");
	PrintMatrix(&pFilter->H, "H");
	PrintMatrix(&pFilter->pP, "pP");
	PrintMatrix(&pFilter->P, "P");
	PrintMatrix(&pFilter->Q, "Q");
	PrintMatrix(&pFilter->iR, "iR");
	PrintMatrix(&pFilter->ey, "ey");
	PrintMatrix(&pFilter->ex, "ex");
	PrintMatrix(&pFilter->ex, "ex");
	PrintMatrix(&pFilter->z, "z");
}

int InformationFilterInit (InformationFilter	*pFilter, int dy, int dz, int isRStatic, int isEIF, void (*pF_function)  (struct _InformationFilter *), void (*stateP_function)  (struct _InformationFilter *), void (*pH_function)  (struct _InformationFilter *), void (*measP_function)  (struct _InformationFilter *), void *EIF_params)
{
// Init the main dimensions of the system
	pFilter->ny = dy;	  pFilter->nz = dz;
	pFilter->nmax = Max (pFilter->ny, pFilter->nz);
	pFilter->isRStatic = isRStatic;
	pFilter->isEIF = isEIF;
	pFilter->EIF_Params = EIF_params;

	/* [km] the prediction/Jacobian computing is done only for the 
	   case of Extended Information Filter */
	if (pFilter->isEIF)	{
		pFilter->ComputeEIF_F_Matrix =  pF_function;
		pFilter->ComputeEIF_HandZprim =  pH_function;
		pFilter->StatePred = stateP_function;
		pFilter->MeasurementPred = measP_function;
	}
	else	{
	 	pFilter->ComputeEIF_F_Matrix =  NULL;
		pFilter->ComputeEIF_HandZprim =  NULL;
		pFilter->StatePred = NULL;
		pFilter->MeasurementPred = NULL;
	}

	// Alloc vectors
	AllocMatrix (&pFilter->ey, pFilter->ny, 1);     				// Init the Information state vector
	AllocMatrix (&pFilter->ex, pFilter->ny, 1);     				// Init the state vector
	AllocMatrix (&pFilter->py, pFilter->ny, 1);     					// Init the state prediction vector
	AllocMatrix (&pFilter->px, pFilter->ny, 1);
	AllocMatrix (&pFilter->z, pFilter->nz, 1); 						// Init the measurement vector

	/* [km] pivoting matrix for calculating the inverse (cblas) */
#ifdef RTAI
	pFilter->iPiv = (int *) kmalloc (pFilter->nmax, sizeof (int));
#else
	pFilter->iPiv = (int *) calloc (pFilter->nmax, sizeof (int));

	if (pFilter->iPiv == NULL)	{
  		fprintf (stderr, "InfoFilterAllocMatrix : Not enough memory !\n");
		exit (-1);
	}
#endif

	// Alloc matrices
	AllocMatrix (&pFilter->F, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->H, pFilter->nz, pFilter->ny);
	AllocMatrix (&pFilter->P, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->iP, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->pP, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->ipP, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->Q, pFilter->ny, pFilter->ny);
	AllocMatrix (&pFilter->R, pFilter->nz, pFilter->nz);
	AllocMatrix (&pFilter->iR, pFilter->nz, pFilter->nz);
	AllocMatrix (&pFilter->Tmp, pFilter->nmax, pFilter->nmax);
	return 0;
}

/* [km] the output noise covariance matrix is computed a-priori */
void SetupRMatrix (InformationFilter	*pFilter, KMATRIX_TYPE *pRarray)
{
  SetMatrix (&pFilter->R, pRarray);
  SetMatrix (&pFilter->iR, pRarray);
  ComputeInverse (&pFilter->iR, pFilter->iPiv);   // iR = !R
}

void  InformationFilterFree (InformationFilter	*pFilter)
{
  DeleteMatrix (&pFilter->F);
  DeleteMatrix (&pFilter->H);
  DeleteMatrix (&pFilter->P);
  DeleteMatrix (&pFilter->iP);
  DeleteMatrix (&pFilter->pP);
  DeleteMatrix (&pFilter->ipP);
  DeleteMatrix (&pFilter->Q);
  DeleteMatrix (&pFilter->R);
  DeleteMatrix (&pFilter->iR);
  DeleteMatrix (&pFilter->Tmp);
  DeleteMatrix (&pFilter->z);
  DeleteMatrix (&pFilter->ex);
  DeleteMatrix (&pFilter->ey);
  DeleteMatrix (&pFilter->py);
  DeleteMatrix (&pFilter->px);
#ifdef RTAI
    kfree (&pFilter->iPiv);
#else
	free (&pFilter->iPiv);
#endif
}

// Single sensor prediction
void InformationFilterStatePrediction(InformationFilter	*pFilter)
{
  // Inverse covariance prediction -------
  // P(k|k-1) = F(k) * P(k-1|k-1) * F(k)' + Q(k) <-> pP = F * P * Ft + Q
  // Compute EIF F matrix if the process is non-linear !!!

  /* [km] for the nonlinear case, the F matrix has to be updated on-line */
  if (pFilter->isEIF && (pFilter->ComputeEIF_F_Matrix != NULL)) (*pFilter->ComputeEIF_F_Matrix) (pFilter);
  /* [km] state covariance prediction is contained in pP */
  ABAt_Plus_C (&pFilter->F, &pFilter->P, &pFilter->Q, &pFilter->pP, &pFilter->Tmp, 'p');

  // State vector prediction -------
  // y^(k|k-1) = !P(k|k-1) * F(k) * P(k-1|k-1) * y^(k-1|k-1) <-> py = ipP * F * P * ey

  /* [km] WARNING: in the prediction step, a possible external input is not taken into account!, i.e. ut=0*/
  Y_Eq_AX (&pFilter->P, &pFilter->ey, &pFilter->py);  // py = P * ey
  Y_Eq_AX (&pFilter->F, &pFilter->py, &pFilter->py);  // py = F * py

  CopyRawMatrix (&pFilter->pP, &pFilter->ipP);  	// They have the same size then copy raw
  ComputeInverse (&pFilter->ipP, pFilter->iPiv);   // ipP = !P

  /* [km] got the information state prediction */
  Y_Eq_AX (&pFilter->ipP, &pFilter->py, &pFilter->py);  // py = !pP * py

  // Finally compute x^(k|k-1) 
  /* [km] obtained as x^(k|k-1)=pP*py; */
  Y_Eq_AX (&pFilter->pP, &pFilter->py, &pFilter->px);
}

// Single sensor Update
void InformationFilterStateUpdate(InformationFilter	*pFilter)
{
  // State vector update -------
  // if isEIF then z_prim(k) = z(k) - (h[k,x^(k|k-1)] - grad(h[k,x^(k|k-1)] x^(k|k-1)))
  // else z_prim(k) = z(k)
  // Compute EIF H and zprim if the measurement is non-linear (px = x^(k|k-1) must be computed before calling this function)
  /* [km] the state estimate must be computed before for the output mapping */
  if (pFilter->isEIF && (pFilter->ComputeEIF_HandZprim != NULL)) {
      (*pFilter->ComputeEIF_HandZprim) (pFilter);
  }

  /* [km] the measurement update of the information state */
   // y^(k) = y^(k|k-1) + H(k)' * !R(k) * z_prim(k) <-> ey = py + Ht * iR * z
  if (!pFilter->isRStatic){    // Compute the inverse at each step if R is not static
    CopyRawMatrix (&pFilter->R, &pFilter->iR);
    ComputeInverse (&pFilter->iR, pFilter->iPiv);   // iR = !R
  }

  Y_Eq_AX (&pFilter->iR, &pFilter->z, &pFilter->ey);  // ey = iR * z
  Y_Eq_ATranspX (&pFilter->H, &pFilter->ey, &pFilter->ey);  // ey = Ht * ey
  A_Eq_APlusB (&pFilter->ey, &pFilter->py); // ey = ey + py

  // Inverse cov update
  // !P(k) = !P(k|k-1) + H(k)' * !R(k) * H(k) <->
  AtBA_Plus_C (&pFilter->H, &pFilter->iR, &pFilter->ipP, &pFilter->iP, &pFilter->Tmp);

  // Compute x^(k) = P(k) * y^(k)
  CopyRawMatrix (&pFilter->iP, &pFilter->P);
  ComputeInverse (&pFilter->P, pFilter->iPiv);            // P = !iP

  /* [km] the state estimate is obtained based on the current
     state information filter and the covariance matrix */
  Y_Eq_AX (&pFilter->P, &pFilter->ey, &pFilter->ex);  // ex = P * ey
}


// ***********************
// Multi sensor information filter
// ***********************

void MultiSensorFilterInit (MultiSensorInfoFilter	*pFilter, int ny, int sensorNr, int useG, int nu, void (*pF_function)  (MultiSensorInfoFilter *), void (*pQ_function)  (MultiSensorInfoFilter *), void (*pf_function)  (MultiSensorInfoFilter *, KMatrix *predx), void *params)
{
	// Init MultiSensorInfoFilter fields
  pFilter->ny = ny;
  pFilter->useG = useG;
  pFilter->nu = nu;
  pFilter->sensorNr = sensorNr;
  pFilter->Params = params;
  /* the time variable F_matrix case, or pure linear stationary */
  pFilter->Compute_F_Matrix =  pF_function;
  /* [km] the function pointers for the non-linear case -> the state equation prediction */
  pFilter->Compute_NonLinear_Prediction =  pf_function;
  /* the process noise matrix */
  pFilter->Compute_Q_Matrix =  pQ_function;

  // Alloc matrices
  AllocMatrix (&pFilter->ey, pFilter->ny, 1);
  AllocMatrix (&pFilter->ex, pFilter->ny, 1);
  AllocMatrix (&pFilter->py, pFilter->ny, 1);
  AllocMatrix (&pFilter->px, pFilter->ny, 1);
  AllocMatrix (&pFilter->upd_y, pFilter->ny, 1);

  AllocMatrix (&pFilter->F, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->P, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->iP, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->pP, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->ipP, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->Q, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->upd_invP, pFilter->ny, pFilter->ny);
  AllocMatrix (&pFilter->Tmp, pFilter->ny, pFilter->ny);

  if(pFilter->useG)
    {
      AllocMatrix (&pFilter->G, pFilter->ny, pFilter->nu);
      AllocMatrix (&pFilter->q, pFilter->nu, pFilter->nu);
    }

#ifdef RTAI
  pFilter->iPiv = (int *) kmalloc (pFilter->ny, sizeof (int));
#else
  pFilter->iPiv = (int *) calloc (pFilter->ny, sizeof (int));
  
  if (pFilter->iPiv == NULL)	{
    fprintf (stderr, "MultiSensorFilterInit : Not enough memory !\n");
    exit (-1);
  }
#endif
}

void SensorAllocMatrices (MultiSensorInfoFilter *pFilter, int sensor_id)
{
  SensorInformation *pSensor = GetSensorInfoPtr (pFilter, sensor_id);

  AllocMatrix (&pSensor->z, pSensor->nz, 1);
  AllocMatrix (&pSensor->zp, pSensor->nz, 1);
  AllocMatrix (&pSensor->H, pSensor->nz, pFilter->ny);
  AllocMatrix (&pSensor->R, pSensor->nz, pSensor->nz);
  AllocMatrix (&pSensor->iR, pSensor->nz, pSensor->nz);
  AllocMatrix (&pSensor->Ht_iR, pFilter->ny, pSensor->nz);
}

void SensorFreeMatrices (MultiSensorInfoFilter  *pFilter, int sensor_id)
{
  SensorInformation *pSensor = GetSensorInfoPtr (pFilter, sensor_id);

  DeleteMatrix (&pSensor->z);
  DeleteMatrix (&pSensor->zp);
  DeleteMatrix (&pSensor->H);
  DeleteMatrix (&pSensor->R);
  DeleteMatrix (&pSensor->iR);
  DeleteMatrix (&pSensor->Ht_iR);
}

void  MultiSensorFilterFree (MultiSensorInfoFilter *pFilter)
{
  int i;

  DeleteMatrix (&pFilter->F);
  DeleteMatrix (&pFilter->P);
  DeleteMatrix (&pFilter->iP);
  DeleteMatrix (&pFilter->pP);
  DeleteMatrix (&pFilter->ipP);
  DeleteMatrix (&pFilter->Q);
  DeleteMatrix (&pFilter->upd_y);
  DeleteMatrix (&pFilter->upd_invP);
  DeleteMatrix (&pFilter->Tmp);
  DeleteMatrix (&pFilter->ex);
  DeleteMatrix (&pFilter->ey);
  DeleteMatrix (&pFilter->py);
  DeleteMatrix (&pFilter->px);
  if(pFilter->useG)
  {
    DeleteMatrix (&pFilter->G);
    DeleteMatrix (&pFilter->q);
  }
#ifdef RTAI
  kfree (&pFilter->iPiv);
#else
  if(pFilter->iPiv != NULL) free (pFilter->iPiv);
#endif
  for(i = 0; i < pFilter->sensorNr; i++) SensorFreeMatrices(pFilter,i);
}

void PrintSensorInfo (SensorInformation *pInfo)
{
	printf ("*** Sensor Information ***\n");
	printf ("Name : %s\n", pInfo->name);
	printf ("Measurement vector size : %i\n", pInfo->nz);
	printf ("isNonLinear : %i\n", pInfo->isNonLinear);
	printf ("isRStatic : %i\n", pInfo->isRStatic);
	PrintMatrix (&pInfo->z, "z");
	PrintMatrix (&pInfo->zp, "zp");
	PrintMatrix (&pInfo->H, "H");
	PrintMatrix (&pInfo->R, "R");
	PrintMatrix (&pInfo->iR, "iR");
}

SensorInformation *GetSensorInfoPtr (MultiSensorInfoFilter	*pFilter, int idx)
{
	return &(pFilter->Sensors[idx]);
}

#if 1
void MultiSensorFilterStatePrediction(MultiSensorInfoFilter *pFilter)
{

  //  int iPiv[100];
 

  /* PROCESS MATRICES CALCULATION , F(k)=F[(ex(k-1),u(k)], Q(k)=Q[ex(k-1),u(k)],
   Jacobians of the nonlinear state equations*/
  if (pFilter->Compute_F_Matrix != NULL) {
    //fprintf(stderr, "COMPUTING F MATRIX!\n");
    (*pFilter->Compute_F_Matrix) (pFilter);
    //PrintMatrixStderr(&pFilter->F, "F_MATRIX");
  }
  if (pFilter->Compute_Q_Matrix != NULL){
    //fprintf(stderr, "COMPUTING Q MATRIX!\n");
    (*pFilter->Compute_Q_Matrix) (pFilter);
    //PrintMatrixStderr(&pFilter->Q, "Q_MATRIX");
  }


  //  PrintMatrixStderr(&pFilter->ex, "EX_STATE_PREDICTION_BEGIN");


  /* ==== STATE PREDICTION -> ex(k|k-1) */
  //fprintf(stderr, "Selecting the state prediction type!\n");
  if (pFilter->Compute_NonLinear_Prediction == NULL)
  {
    /* use previous state prediction and multiply with the system gain */
    Y_Eq_AX (&pFilter->F, &pFilter->ex, &pFilter->px);   //ex(k|k-1)=Fe(x|x-1)
  }
  else { 
    //fprintf(stderr, "USING NON-LINEAR PREDICTION!\n");
    CopyRawMatrix (&pFilter->ex, &pFilter->px);     // They have the same size then copy raw
    (*pFilter->Compute_NonLinear_Prediction) (pFilter, &pFilter->px);      //ex(k|k-1)=f[k,ex(k-1|k-1)]
  }


  // PrintMatrixStderr(&pFilter->ex, "EX_STATE_PREDICTION_END");
  //  PrintMatrixStderr(&pFilter->px, "px_STATE_PREDICTION");
  /* =========== COVARIANCE PREDICTION -> P(k|k-1) */
  // P(k|k-1) = F(k) * P(k-1|k-1) * F(k)' + Q(k) <-> pP = F * P * Ft + Q
  // Compute EIF F matrix if the process is non-linear or time varying !!!
  //PrintMatrixStderr(&pFilter->F, "F");
  //PrintMatrixStderr(&pFilter->P, "P");
  //PrintMatrixStderr(&pFilter->Q, "Q");
  ABAt_Plus_C(&pFilter->F, &pFilter->P, &pFilter->Q, &pFilter->pP, &pFilter->Tmp, 'p');
  //PrintMatrixStderr(&pFilter->pP, "pP_AFTER_STATE_UPDATE");


  /*======== INFORMATION STATE PREDICTION -> y(k|k-1)=P(k|k-1)e(k|k-1) */
  CopyRawMatrix (&pFilter->pP, &pFilter->ipP);     // They have the same size then copy raw
  ComputeInverse (&pFilter->ipP, pFilter->iPiv);   // ipP = !pP
  //ComputeInverse (&pFilter->ipP, iPiv);   // ipP = !pP

  //PrintMatrixStderr(&pFilter->ipP, "ipP");

  /* just for testing...*/
  //  X_Eq_AX (&pFilter->ipP, &pFilter->pP, &pFilter->Tmp);  // py = ipP * py
  //Y_Eq_AX (&pFilter->ipP, &pFilter->pP, &pFilter->Tmp);  // py = ipP*e(k|k-1)
  //  C_Eq_AB (&pFilter->ipP, &pFilter->pP, &pFilter->Tmp);  // py = ipP*e(k|k-1)
  //PrintMatrixStderr(&pFilter->Tmp, "ipP*pP");
  
  
  /* [km] information state prediction */
  Y_Eq_AX (&pFilter->ipP, &pFilter->px, &pFilter->py);  // py = ipP*e(k|k-1)
  //PrintMatrixStderr(&pFilter->py, "py_STATE_PREDICTION");


}
#endif



#if 0 //km unnecessary steps in the State prediction 
void MultiSensorFilterStatePrediction(MultiSensorInfoFilter *pFilter)
{


  /* ==== COVARIANCE PREDICTION ===========*/
  /* hardcoding a value inside */
  //SET_MATRIX_ELEMENT(pMat,r,c,val) ((pMat)->Element[(c) + (r) *(pMat)->nCol] = val)
  //  SET_MATRIX_ELEMENT(&(pFilter->ex),0,8,-2.72);
  /* [km] printing the current state of the sytem (not information) */
  //  PrintMatrixStderr(&pFilter->ex, "EX_BEFORE");

  // Inverse covariance prediction -------
  // P(k|k-1) = F(k) * P(k-1|k-1) * F(k)' + Q(k) <-> pP = F * P * Ft + Q
  // Compute EIF F matrix if the process is non-linear or time varying !!!
  if (pFilter->Compute_F_Matrix != NULL) {
    //fprintf(stderr, "COMPUTING F MATRIX!\n");
    (*pFilter->Compute_F_Matrix) (pFilter);
    //    PrintMatrixStderr(&pFilter->F, "F_MATRIX");
  }
  if (pFilter->Compute_Q_Matrix != NULL){
    //fprintf(stderr, "COMPUTING Q MATRIX!\n");
    (*pFilter->Compute_Q_Matrix) (pFilter);
    // PrintMatrixStderr(&pFilter->Q, "Q_MATRIX");
  }

  /* [km] Q is the process noise */
  /* [km] covariance prediction */
     ABAt_Plus_C (&pFilter->F, &pFilter->P, &pFilter->Q, &pFilter->pP, &pFilter->Tmp, 'p');
     //PrintMatrixStderr(&pFilter->pP, "pP");
#if 0
     /* a test to see the state prediction by pure covariance multiplication */
     X_Eq_AX (&pFilter->F, &pFilter->ex, &pFilter->Tmp);
     //PrintMatrixStderr(&pFilter->ex, "EX_SIMPLE");
     PrintMatrixStderr(&pFilter->q, "q");
#endif

     //A_Eq_APlusB  (&pFilter->ex, &pFilter->q);
     //     PrintMatrixStderr(&pFilter->q, "EX_SIMPLE");
     

     /* ==== STATE PREDICTION 
	-> in linear case, the state ey from previous step can be used direclty */
  if (pFilter->Compute_NonLinear_Prediction == NULL)
  {
    //    fprintf(stderr, "USING LINEAR PREDICTION!\n");
  	// y^(k|k-1) = !P(k|k-1) * F(k) * P(k-1|k-1) * y^(k-1|k-1) <-> py = ipP * F * P * ey
  	Y_Eq_AX (&pFilter->P, &pFilter->ey, &pFilter->py);   // py = P * ey  (ex(k-1|k-1))
  	X_Eq_AX (&pFilter->F, &pFilter->py, &pFilter->Tmp);  // py = F * py  (ex(k|k-1))
  }
  else { // Non linear prediction py = f[k,ex(k-1|k-1)]
    //fprintf(stderr, "USING NON-LINEAR PREDICTION!\n");
    (*pFilter->Compute_NonLinear_Prediction) (pFilter, &pFilter->py);      //ex(k|k-1)=f[k,ex(k-1|k-1)]
  }

  //PrintMatrixStderr(&pFilter->pP, "pP");

  /* INFORMATION STATE PREDICTION */
  CopyRawMatrix (&pFilter->pP, &pFilter->ipP);     // They have the same size then copy raw
  ComputeInverse (&pFilter->ipP, pFilter->iPiv);   // ipP = !pP


  /* [km] information state prediction */
     X_Eq_AX (&pFilter->ipP, &pFilter->py, &pFilter->Tmp);  // py = ipP * py


  // Finally compute x^(k|k-1)
     /* [km} could be passed forward directly from the above equations before multiplying with ipP */
  Y_Eq_AX (&pFilter->pP, &pFilter->py, &pFilter->px);
  //  PrintMatrixStderr(&pFilter->px, "EX_PREDICITON_AFTER_MEASUREMENT_UPDATE");
}

#endif

// Update the state. sensorMask is used to specify wich sensors are to be considered at time k
// sensorMask is initialized as follow: a 1 means that the sensor will be used for the update
// Example : sensorMask = 1<<3 || 1<<0 means that sensor 3 and 0 will be considered

/* [km] state update based on the current measurement with is contained in z_prim (z_p) */

/* [km] the current measurement , z_p must is calculated within each sensor's update,
   here just the generic EIF update is performed */
int MultiSensorFilterStateUpdate(MultiSensorInfoFilter *pFilter, unsigned char sensorMask)
{
	int i;
      int res;
      int i_catch;


      // fprintf(stderr, "ENTERED THE MULTISENSOR MEASUREMENT UPDATE!\n");
	// Clear upd_y and upd_invP
      /* [km] the current innovations to the global filter state
	 are considered here "incremental", so the upd_y and upd_invP 
	 are set to zero in the start */
      /* [km] the idea behind this incrmental update is that 
	 we could update the global filter's state with more
	 then one sensor in the current function call (defined by the sensorMask) */

      ZeroMatrix (&pFilter->upd_y);
      ZeroMatrix (&pFilter->upd_invP);
      

      //  PrintMatrix(&pFilter->ex, "EX_MEASUREMENT_UPDATE_BEGIN");
	
	for (i = 0; i < pFilter->sensorNr; i++)
	{
	  /* [km] selecting out a specific sensor with the sensorMask */
		if (USE_SENSOR (i, sensorMask))
		{
#ifdef VERBOSE
			printf ("State update for : %s\n", pFilter->Sensors[i].name);
#endif
			  
			//		fprintf(stderr, "COMPUTING MEASUREMENT STRUCTURES!\n");
			res = (*pFilter->Sensors[i].Compute_H_R_zp) ((void *) pFilter); // Call the specific function for the sensor
                  /* Check if the measurement has been validated */
			if(res == 0) {
			  printf("Skip measurement sensor %i\n",i);
			  return 0;
			}

		  
                  // iR must be also computed once if it is static
			// Compute iR if R changes

		  /* [km] the R matrix can be static, if a constant process noise is assumed,
		     however, if not, the inverse of R, iR is computed on-line */
		  //fprintf(stderr, "IS STATIC!\n");

		  
			if (!pFilter->Sensors[i].isRStatic)
			  {
			    //		    PrintMatrixStderr(&pFilter->Sensors[i].R, "R");
			    //			    fprintf(stderr, "THE MEASUREMENT NOISE MATRIX IS NOT CONSTANT -> has to be computed on-line!\n");
			    CopyRawMatrix (&pFilter->Sensors[i].R, &pFilter->Sensors[i].iR);  	// They have the same size then copy raw
			    ComputeInverse (&pFilter->Sensors[i].iR, pFilter->iPiv);   // iR = !iR
			    //PrintMatrixStderr(&pFilter->Sensors[i].iR, "iR");
			  }
			
			/*
			PrintMatrixStderr(&pFilter->Sensors[i].zp, "ZP_BEFORE");
			PrintMatrixStderr(&pFilter->Sensors[i].R, "R");
			PrintMatrixStderr(&pFilter->Sensors[i].iR, "iR");
			PrintMatrixStderr(&pFilter->Sensors[i].H, "H");
			*/
			//PrintMatrixStderr(&pFilter->Sensors[i].zp, "ZP_BEFORE");
			//PrintMatrixStderr(&pFilter->Sensors[i].H, "H");
			i_catch=i;

			// Ht_iR = Ht iR
			C_Eq_AtB (&pFilter->Sensors[i].H, &pFilter->Sensors[i].iR, &pFilter->Sensors[i].Ht_iR);
			//PrintMatrixStderr(&pFilter->Sensors[i].Ht_iR, "Ht_iR");
			
			// upd_y = upd_y + Ht_iR zp
			Y_Eq_AX_Plus_Y (&pFilter->Sensors[i].Ht_iR, &pFilter->Sensors[i].zp, &pFilter->upd_y);
			
			
			//PrintMatrixStderr(&pFilter->upd_y, "upd_y");
			
			// upd_invP = upd_invP + Ht_iR H
			/* [km] updated information matrix */
			C_Eq_ABPlusC (&pFilter->Sensors[i].Ht_iR, &pFilter->Sensors[i].H, &pFilter->upd_invP);
			
			//PrintMatrixStderr(&pFilter->upd_invP, "upd_invP");
		}
	}

#if 1
	// y^(k|k) = y^(k|k-1) + sum ( Ht(j) !R(j) zp(j) ) <-> ey = py + upd_y
	/* [km] adding the sensor innovation from the current cycle
	   to the information state estimate of the global filter */

	//[km] ey=upd_y
  	CopyRawMatrix (&pFilter->upd_y, &pFilter->ey);
	//	PrintMatrixStderr (&pFilter->upd_y, "UPD_Y");


	//[km] ey=upd_y+py
	Y_Eq_X_Plus_Y (&pFilter->py, &pFilter->ey);
	//	PrintMatrixStderr(&pFilter->py, "py");
	//PrintMatrixStderr(&pFilter->ey, "ey");

	// !P(k|k) = !P(k|k-1) + sum ( Ht(j) !R(j) H(j) ) <-> iP = ipP + upd_invP

	/* [km] updating the information matrix (globally) */
	//[km] iP=upd_invP
	CopyRawMatrix (&pFilter->upd_invP, &pFilter->iP);
	//[km] iP=upd_invP+ipP
	A_Eq_APlusB (&pFilter->iP, &pFilter->ipP);

	// P(k|k) = !!P(k|k) <-> P = !iP
	CopyRawMatrix (&pFilter->iP, &pFilter->P);
	//PrintMatrixStderr(&pFilter->iP, "iP_MEASUREMENT_UPDATE");
	//P=i(iP)
	ComputeInverse (&pFilter->P, pFilter->iPiv);
	//PrintMatrixStderr(&pFilter->P, "P_MEASUREMENT_UPDATE");

	//	PrintMatrixStderr (&pFilter->ex, "EX_BEFORE");
	// Finally compute x^(k|k)
	/* [km] current global estimate */

	//[km] ex=P*ey
	Y_Eq_AX (&pFilter->P, &pFilter->ey, &pFilter->ex);
	//PrintMatrixStderr(&pFilter->ex, "ex_FUSED");

      CopyRawMatrix (&pFilter->ey, &pFilter->py);
      CopyRawMatrix (&pFilter->iP, &pFilter->ipP);

      // PrintMatrixStderr (&pFilter->Sensors[i_catch].zp, "ZP_AFTER");
      //PrintMatrixStderr (&pFilter->ex, "EX_MEASUREMENT_UPDATE_END");
#endif


     /* if(!IsPositiveDefinite(&pFilter->P)) {
      printf("P not positive definite\n");
}  */
  return 1;
}
