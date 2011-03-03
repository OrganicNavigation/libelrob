/***************************************************************************
                          Kmatrix.c  -  description
                             -------------------
    begin                : Mon May 26 2003
    copyright            : (C) 2003 by Pierre Lamon
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

#include <string.h>

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_cblas.h>

#include "Kmatrix.h"

 void AllocMatrix (KMatrix	*pMat, int nRow, int nCol)
{
	pMat->nRow	= nRow;
	pMat->nCol	= nCol;

#ifdef RTAI
    pMat->Element = (KMATRIX_TYPE *) kmalloc (pMat->nCol * pMat->nRow, sizeof (KMATRIX_TYPE));
#else
	pMat->Element = (KMATRIX_TYPE *) malloc (pMat->nCol * pMat->nRow * sizeof (KMATRIX_TYPE));
	if (pMat->Element == NULL)	{
  		fprintf (stderr, "AllocMatrix : Not enough memory !\n");
		exit (-1);
	}
	ZeroMatrix (pMat);
#endif
}

void DeleteMatrix (KMatrix	*pMat)
{
  if (pMat->Element != NULL)
#ifdef RTAI
   kfree (pMat->Element );
#else
 	free (pMat->Element);
#endif
	pMat->nRow = 0;
	pMat->nCol = 0;
}


// Copy a bloc of the src matrix to the destination matrix
// If the destination is bigger than the source then copy the whole src matrix into dest (considering src nCols and nRows mapping)
// else copy only a bloc mapped to the destination layout (considering dest nCols and nRows mapping)
void CopyBlockMatrix (KMatrix *src, KMatrix *dest)
{
	int	r,c;

	if ((dest->nCol * dest->nRow) >= (src->nCol * src->nRow))	{
 		for (r = 0; r < src->nRow; r++)
			for (c = 0; c < src->nCol; c++)
				dest->Element[c + r * dest->nCol] = src->Element[c + r * src->nCol];
	}
	else {                                                     	// If the source is bigger than the destination then copy the bloc
    	for (r = 0; r < dest->nRow; r++)
			for (c = 0; c < dest->nCol; c++)
				dest->Element[c + r * dest->nCol] = src->Element[c + r * src->nCol];
	}
}

// Copy a part of the src matrix (wherever) to the destination matrix
// The source is bigger than the destination. If the destination is bigger than the block down and right of the beginning point
// the matrix is completed by zeros.
void CopyPartMatrix (KMatrix *src, KMatrix *dest, int BeginRow, int BeginCol)
{
	int	r, c, stopr, stopc;

	if (BeginRow < 0 || BeginCol < 0) {
		fprintf(stderr,"The beginning Row and Col should be > 0.\n"); exit(-1);
	}
	else if ((dest->nRow > src->nRow-BeginRow) || (dest->nCol > src->nCol-BeginCol)) {	// thus must be completed by zeros
		//printf("a\n");
		ZeroMatrix (&*dest);

		if(src->nRow-BeginRow < dest->nRow) stopr = src->nRow-BeginRow;
		else stopr = dest->nRow;
		if(src->nCol-BeginCol < dest->nCol) stopc = src->nCol-BeginCol;
		else stopc = dest->nCol;

		for (r = 0; r < stopr; r++)
			for (c = 0; c < stopc; c++)
				dest->Element[c + r * dest->nCol] = src->Element[(c+BeginCol) + (r+BeginRow) * src->nCol];
	}
	else {
    	for (r = 0; r < dest->nRow; r++)
			for (c = 0; c < dest->nCol; c++)
				dest->Element[c + r * dest->nCol] = src->Element[(c+BeginCol) + (r+BeginRow) * src->nCol];
	}
}

// Set the content of data in dest matrix. The first element is (BeginRow, BeginCol) then the size is nRows x nCols
void	SetSubMatrix (KMatrix *dest, KMATRIX_TYPE *data, int BeginRow, int BeginCol, int nRows, int nCols)
{
	int r, c, i = 0;

	for (r = 0; r < nRows; r++)
		for (c = 0; c < nCols; c++)
			dest->Element[(BeginRow + r) * dest->nCol + BeginCol + c] = data[i++];
}

void CreateRPYMatrix(float roll, float pitch, float yaw, KMatrix *rot33)
{
        float cf,sf,cp,sp,ct,st;

        cf = cos (pitch);    sf = sin (pitch);
        cp = cos (yaw);     sp = sin (yaw);
        ct = cos (roll);       st = sin (roll);

        // First column
        rot33->Element[0] = cf * cp;
        rot33->Element[3] = cf * sp;
        rot33->Element[6] = -sf;

        // Second column
        rot33->Element[1] = st*sf*cp-ct*sp;
        rot33->Element[4] = st*sf*sp+ct*cp;
        rot33->Element[7] = st*cf;

        // Third column
        rot33->Element[2] = ct*sf*cp+st*sp;
        rot33->Element[5] = ct*sf*sp-st*cp;
        rot33->Element[8] = ct*cf;
}

void getRollPitchYaw(KMATRIX_TYPE *rpy, KMatrix	*homo)
{
	KMATRIX_TYPE	*mat = homo->Element;
	double		euler[3];

	double d = sqrt(mat[0]*mat[0] + mat[4]*mat[4]);
	if (fabs(d) > 1e-10) {
		euler[0] = atan2(mat[4], mat[0]);
		euler[2] = atan2(mat[9], mat[10]);
	} else {
		euler[0] = atan2(-mat[1], mat[5]);
		euler[2] = 0.0;
	}
	euler[1] = atan2(-mat[8], d);

	rpy[0]=euler[2];//asin(cos(euler[1])*sin(euler[2]));
	rpy[1]=euler[1];
	rpy[2]=euler[0];
}

void CreateHomogenMatrix (float roll, float pitch, float yaw, float x, float y, float z, KMatrix *h44)
{
	 float cf,sf,cp,sp,ct,st;

        cf = cos (pitch);    sf = sin (pitch);
        cp = cos (yaw);     sp = sin (yaw);
        ct = cos (roll);       st = sin (roll);

        // First column
        h44->Element[0] = cf * cp;
        h44->Element[4] = cf * sp;
        h44->Element[8] = -sf;
        h44->Element[12] = 0.0;

        // Second column
        h44->Element[1] = st*sf*cp-ct*sp;
        h44->Element[5] = st*sf*sp+ct*cp;
        h44->Element[9] = st*cf;
        h44->Element[13] = 0.0;

        // Third column
        h44->Element[2] = ct*sf*cp+st*sp;
        h44->Element[6] = ct*sf*sp-st*cp;
        h44->Element[10] = ct*cf;
        h44->Element[14] = 0.0;

        // Fourth column
        h44->Element[3] = x;
		 h44->Element[7] = y;
		 h44->Element[11] = z;
		 h44->Element[15] = 1.0;
}

// Paste a part of the src matrix (wherever) to a part of the destination matrix (wherever)
// the rest of the destination matrix remain untouched.
void PastePartMatrix (KMatrix *src, int CopyRow, int CopyCol, int CopyHeight, int CopyLength, KMatrix *dest, int PasteRow, int PasteCol)
{
	int r, c, stopr, stopc;

	if (CopyRow < 0 || CopyCol < 0 || CopyHeight < 0 || CopyLength < 0 || PasteRow < 0 || PasteCol < 0) {
		fprintf(stderr," : Some of the parameters are < 0. This is not allowed.\n"); exit(-1);
	}
	else if ((CopyLength+CopyCol > src->nCol) || (CopyHeight+CopyRow > src->nRow)) {
		fprintf(stderr," : From the starting point, the size to copy is greater than the rest of the matrix.\n"); exit(-1);
	}

	if ((dest->nRow-PasteRow) < CopyHeight)	stopr = dest->nRow-PasteRow;
	else	stopr = CopyHeight;

	if ((dest->nCol-PasteCol) < CopyLength)	stopc = dest->nCol-PasteCol;
	else	stopc = CopyLength;

	for (r=0 ; r<stopr ; r++)
		for (c=0 ; c<stopc ; c++)
			dest->Element[(c+PasteCol)+(r+PasteRow)*dest->nCol] = src->Element[(c+CopyCol) + (r+CopyRow) * src->nCol];
}

// Copy the Element of src to dest without considering the mapping
// Accounts for the desination size only
// If dest is to small the copy stops
void CopyRawMatrix (KMatrix *src, KMatrix *dest)
{
	memcpy (dest->Element, src->Element, dest->nRow * dest->nCol * sizeof (KMATRIX_TYPE));
}

double MatrixElement (KMatrix *pMat, int r, int c)
{
	return pMat->Element[c + r * pMat->nCol];
}

void SetMatrixElement (KMatrix *pMat, int r, int c, KMATRIX_TYPE val)
{
	pMat->Element[pMat->nCol * r + c] = val;
}

double *MatrixElPointer (KMatrix *pMat, int r, int c)
{
	return (&pMat->Element[c + r * pMat->nCol]);
}

void FillMatrix (KMatrix	*pMat, KMATRIX_TYPE val)
{
	int i;

	for (i = 0; i < pMat->nRow * pMat->nCol; i++) pMat->Element[i] = val;
}

/*void FillRandom (KMatrix	*pMat)
{
#ifndef RTAI
	int	r,c;

 	for (r = 0; r < pMat->nRow; r++)
			for (c = 0; c < pMat->nCol; c++) pMat->Element[c + r * pMat->nCol] = (double) GetRandomInt(5);
#endif
}*/

void CreateIdentityMatrix (KMatrix *Imat, int dim)
{
	int i;
	AllocMatrix (Imat, dim, dim);
	for (i = 0; i < Imat->nRow; i++) Imat->Element[i * (1 + Imat->nCol)] = 1.0;	 	// Init the diagonal elements
}

void SetIdentityMatrix (KMatrix *Imat)
{
	int i;
	if (Imat->nCol != Imat->nRow) {
		fprintf (stderr, "SetIdentityMatrix -> nRow != nCol");
		exit (-1);
	}
	ZeroMatrix (Imat);
	for (i = 0; i < Imat->nRow; i++) Imat->Element[i * (1 + Imat->nCol)] = 1.0;	 	// Init the diagonal elements
}

void SetDiagonalMatrix (KMatrix *Diagmat, KMATRIX_TYPE val)
{
	int i;
	if (Diagmat->nCol != Diagmat->nRow) {
		fprintf (stderr, "SetIdentityMatrix -> nRow != nCol");
		exit (-1);
	}
	//bzero ((void *) Imat->Element, Imat->nCol * Imat->nRow * sizeof (KMATRIX_TYPE));
	ZeroMatrix (Diagmat);
	for (i = 0; i < Diagmat->nRow; i++) Diagmat->Element[i * (1 + Diagmat->nCol)] = val;	 	// Init the diagonal elements
}

void MultiplyMatrix (KMatrix *mat, KMATRIX_TYPE val)
{
	int i;

	for (i = 0; i < mat->nRow * mat->nCol; i++) mat->Element[i] *= val;
}

void ZeroMatrix (KMatrix *mat)
{
	/*int i;
 	for (i = 0; i < mat->nRow * mat->nCol; i++) mat->Element[i] = (KMATRIX_TYPE) 0.0;	*/

 	memset(mat->Element, 0, mat->nRow * mat->nCol * sizeof(KMATRIX_TYPE));
}

// Return -1 if pMat is definite positiv if (x' A x >= 0)    A must be square !!!!
// else return 0
int  IsPositiveDefinite (KMatrix	*pMat)
{
	KMatrix		x, Tmp;
	KMATRIX_TYPE	Result;
	//int	lda =   pMat->nCol;

	AllocMatrix	(&x, pMat->nCol, 1);
	AllocMatrix	(&Tmp, pMat->nCol, 1);
	FillMatrix (&x, 1.0);

	// Compute Tmp = A * x
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								Tmp.nRow, Tmp.nCol, pMat->nCol,
								1.0, pMat->Element, pMat->nCol,
								x.Element, x.nCol,
								0.0, Tmp.Element, Tmp.nCol);

	// Compute Result = x' * Tmp   Tmp will change its dimension il will come a (C->nRow x C->nCol) matrix
	cblas_dgemm (CblasRowMajor, CblasTrans, CblasNoTrans,
								1, 1, x.nRow,
								1.0, x.Element, x.nCol,
								Tmp.Element, Tmp.nCol,
								0.0, &Result, 1);

	//printf ("Result : %f\n", Result);

	DeleteMatrix (&x);
	DeleteMatrix (&Tmp);

	if (Result >= 0.0) return -1;
	else return 0;
}

void	 PrintMatrix (KMatrix	*pMat, char *MatrixName)
{
#ifndef RTAI
	int r,c;

	if (MatrixName != NULL)		printf ("%s (%i x %i)\n\t", MatrixName, pMat->nRow, pMat->nCol);
	else printf ("(%i x %i)\n\t", pMat->nRow, pMat->nCol);

 	for (r = 0; r < pMat->nRow; r++)
	{
			for (c = 0; c < pMat->nCol; c++) printf ("%e\t", pMat->Element[c + r * pMat->nCol]);
    		printf ("\n\t");
	}
	//	printf ("\r");
	printf ("\n");
#endif
}

void	 PrintMatrixStderr (KMatrix	*pMat, char *MatrixName)
{
#ifndef RTAI
	int r,c;

	if (MatrixName != NULL)		fprintf (stderr, "%s (%i x %i)\n\t", MatrixName, pMat->nRow, pMat->nCol);
	else fprintf (stderr, "(%i x %i)\n\t", pMat->nRow, pMat->nCol);

 	for (r = 0; r < pMat->nRow; r++)
	{
			for (c = 0; c < pMat->nCol; c++) fprintf (stderr, "%e\t", pMat->Element[c + r * pMat->nCol]);
    		fprintf (stderr, "\n\t");
	}
	//fprintf (stderr, "\r");
	fprintf (stderr, "\n");
#endif
}

void A_Eq_kA (KMATRIX_TYPE k, KMatrix *A)
{
  int i;
  for (i = 0; i < A->nCol * A->nRow; i++) A->Element[i] = A->Element[i]*k;
}

void A_Eq_APlusB  (KMatrix *A, KMatrix *B)
{
  int i;
  if ((A->nCol != B->nCol) || (A->nRow != B->nRow)) {
    printf ("A_Eq_APlusB -> Bad parameters\n");
    exit (-1);
  }

  for (i = 0; i < A->nCol * A->nRow; i++) A->Element[i] = A->Element[i] + B->Element[i];
}

void A_Eq_AMinusB  (KMatrix *A, KMatrix *B)
{
 int i;
  if ((A->nCol != B->nCol) || (A->nRow != B->nRow)) {
    printf ("A_Eq_AMinusB -> Bad parameters\n");
    exit (-1);
  }

  for (i = 0; i < A->nCol * A->nRow; i++) A->Element[i] -= B->Element[i];
}

void A_Eq_MinusAPlusB  (KMatrix *A, KMatrix *B)
{
  int i;
  if ((A->nCol != B->nCol) || (A->nRow != B->nRow)) {
    printf ("A_Eq_MinusAPlusB -> Bad parameters\n");
    exit (-1);
  }

  for (i = 0; i < A->nCol * A->nRow; i++) A->Element[i] = B->Element[i]  - A->Element[i];
}

// Mat and Inverse will be modified
// Mat = inv(Mat)
// iPiv = P
void	 ComputeInverse (KMatrix *Mat)
{
    if (Mat->nRow != Mat->nCol) {
      printf ("ComputeInverse -> Not square\n");
      exit (-1);
    }

    int signum;
    gsl_permutation* perm = gsl_permutation_alloc(Mat->nRow);
    gsl_matrix* lu = gsl_matrix_alloc(Mat->nRow, Mat->nCol);
    gsl_matrix_view inverse = gsl_matrix_view_array(Mat->Element, Mat->nRow,
      Mat->nCol);

    gsl_matrix_memcpy(lu, &inverse.matrix);
    gsl_linalg_LU_decomp(lu, perm, &signum);
    gsl_linalg_LU_invert(lu, perm, &inverse.matrix);

    gsl_matrix_free(lu);
    gsl_permutation_free(perm);
}

/*                [ 1.0 2.0 3.0 ]
  -->  mat = [ 2.0 4.0 5.0 ]
                  [ 3.0 5.0 6.0 ]

  En fait, le stockage interne est :
  sym = [ 1.0 2.0 4.0 3.0 5.0 6.0 ] (pour compatibilite avec IMSL)*/
void FillSymmetricMatrix (KMatrix *mat, KMATRIX_TYPE *sym)
{
	int i,j,l,c;

	if (mat->nRow != mat->nCol) exit (-1);

	l = mat->nRow;
	c = mat->nCol;

	for (i = 0; i < l; i++)
    for (j = i; j < c; j++)
      {
      		mat->Element[i * mat->nCol + j] = sym[(j*(j+1))/2 + i];
      		mat->Element[j * mat->nCol + i] = sym[(j*(j+1))/2 + i];
      	}
  //  PrintMatrix (mat,"mat sym");
}

void FillSymmetricArray (KMATRIX_TYPE *sym, KMatrix *mat)
{
	int i,j,l,c;

	if (mat->nRow != mat->nCol) exit (-1);

	l = mat->nRow;
	c = mat->nCol;

	for (i = 0; i < l; i++)
    for (j = i; j < c; j++)
      {
      		sym[(j*(j+1))/2 + i] = mat->Element[i * mat->nCol + j];
      	}
     //PrintMatrix (mat,"mat sym");
}

// Tested only with same dimensions matrices
void ABAt (KMatrix *A, KMatrix *B, KMatrix *Result, KMatrix *Tmp)
{
	 int lda = A->nCol;			// Number of rows of the A matrix
	 int ldc = Result->nCol;

	// Compute Tmp = B * A'	which is a (B->nRow x A->nRow) matrix
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasTrans,    //	B not transposed, A transposed
								B->nRow, A->nRow, B->nCol, 								  //	M, N, K
								1.0, B->Element, B->nCol,
								A->Element, lda,
								0.0, Tmp->Element, A->nRow); 				  // Tmp is B->nRow x A->nRow

	// Compute Result = A * Tmp   Tmp will change its dimension il will come a (C->nRow x C->nCol) matrix
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								A->nRow, A->nRow, A->nCol,
								1.0, A->Element, lda,
								Tmp->Element, A->nRow,
								0.0, Result->Element, ldc);
}

void	ABAt_Plus_CDCt (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *D, KMatrix *Result)
{
	KMatrix	tmp,R1;

	AllocMatrix(&tmp, A->nCol, A->nRow);
	AllocMatrix(&R1, A->nCol, A->nRow);

	ABAt (A, B, Result, &tmp);
	ABAt (C, D, &R1, &tmp);
	A_Eq_APlusB(Result, &R1);

	DeleteMatrix(&R1);
	DeleteMatrix(&tmp);
}

// Do Result = A*B*A' + C  if plusOrMinus == 'p'	(B must be square)
// else Result = -A*B*A' + C
// Result has the same size as C
//	We have the following constraints	B->nCol = A->nCol = B->nRow and A->nRow = C->nRow
// Tmp must be max (dimension of A,B,C) x max(dimension of A,B,C)
void ABAt_Plus_C (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *Result, KMatrix *Tmp, char plusOrMinus)
{
	 int lda = A->nCol;			// Number of rows of the A matrix
	 int ldc = C->nCol;

	// Compute Tmp = B * A'	which is a (B->nRow x A->nRow) matrix
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasTrans,    //	B not transposed, A transposed
								B->nRow, A->nRow, B->nCol, 								  //	M, N, K
								1.0, B->Element, B->nCol,
								A->Element, lda,
								0.0, Tmp->Element, A->nRow); 				  // Tmp is B->nRow x A->nRow

	// Compute Result = A * Tmp   Tmp will change its dimension il will come a (C->nRow x C->nCol) matrix
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								A->nRow, A->nRow, A->nCol,
								1.0, A->Element, lda,
								Tmp->Element, A->nRow,
								0.0, Result->Element, ldc);

	// Compute Result = +/- Result + C
	// !!!!! I'm not sure BLAS is faster for that !
	if (plusOrMinus == 'p')
	    A_Eq_APlusB (Result, C);
		/*	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								C->nRow, C->nCol, C->nCol,
								1.0, C->Element, ldc,
								pFilter->I.Element, ldc,
								1.0, Result->Element, ldc);	*/
	else
	   A_Eq_MinusAPlusB  (Result, C);
		/*	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								C->nRow, C->nCol, C->nCol,
								1.0, C->Element, ldc,
								pFilter->I.Element, ldc,
								-1.0, Result->Element, ldc);	*/
}

// Do Result = A'*B*A + C
// Result has the same size as C
//	We have the following constraints	B->nCol = A->nCol = B->nRow and A->nRow = C->nRow
// Tmp must be max (dimension of A,B,C) x max(dimension of A,B,C)
void AtBA_Plus_C (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *Result, KMatrix *Tmp)
{
	 int lda = A->nCol;			// Number of rows of the A matrix
	 int ldc = C->nCol;

	// Compute Tmp = B * A	which is a (B->nRow x A->nRow) matrix
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,    //	B not transposed, A not transposed
								B->nRow, A->nRow, B->nCol, 								  //	M, N, K
								1.0, B->Element, B->nCol,
								A->Element, lda,
								0.0, Tmp->Element, A->nRow); 				  // Tmp is B->nRow x A->nRow

	// Compute Result = A' * Tmp   Tmp will change its dimension il will come a (C->nRow x C->nCol) matrix
	cblas_dgemm (CblasRowMajor, CblasTrans, CblasNoTrans,
								A->nRow, A->nRow, A->nCol,
								1.0, A->Element, lda,
								Tmp->Element, A->nRow,
								0.0, Result->Element, ldc);

	// Compute Result = Result + C
	// !!!!! I'm not sure BLAS is faster for that !
	A_Eq_APlusB (Result, C);
}

void C_Eq_ABPlusC (KMatrix *A, KMatrix *B, KMatrix *C)
{
	cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans,
								C->nRow, C->nCol, A->nCol,
								1.0, A->Element, A->nCol,
								B->Element, B->nCol,
								1.0, C->Element, C->nCol);
}

// Do x = x - y
// The pointers must be different
// x is modified by the procedure
// The vectors are 1 column vectors
void  X_Eq_X_Minus_Y(KMatrix *x, KMatrix *y)
{
	cblas_daxpy (x->nRow, -1.0, y->Element, 1, x->Element, 1);
}

void	Y_Eq_X_Plus_Y(KMatrix *x, KMatrix *y)
{
	cblas_daxpy (x->nRow, 1.0, x->Element, 1, y->Element, 1);
}

// Do y = A * x
// Does not take the dimensions of x into account
// You can't do x = A * x
void Y_Eq_AX (KMatrix *A, KMatrix *x, KMatrix *y)
{
	if (x != y) 	cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, 1.0, A->Element, A->nCol, x->Element, 1, 0.0, y->Element, 1);
	else {
		printf ("x = A x not allowed\n");
		exit (-1);
	}
}

// Vectorial product
void Z_Eq_XCrossY (KMatrix *x, KMatrix *y, KMatrix *z)
{
	z->Element[0] = x->Element[1] * y->Element[2] - x->Element[2] * y->Element[1];
	z->Element[1] = -x->Element[0] * y->Element[2] + x->Element[2] * y->Element[0];
	z->Element[2] = x->Element[0] * y->Element[1] - x->Element[1] * y->Element[0];
}

void SetVector (KMatrix *v, KMATRIX_TYPE x, KMATRIX_TYPE y, KMATRIX_TYPE z)
{
	v->Element[0] = x;
	v->Element[1] = y;
	v->Element[2] = z;
}

// Do x = A * x
// Does not take the dimensions of x into account
// tmp must have at least nx size
void X_Eq_AX (KMatrix *A, KMatrix *x, KMatrix *tmp)
{
	if (x != tmp) 	{
		cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, 1.0, A->Element, A->nCol, x->Element, 1, 0.0, tmp->Element, 1);
		CopyRawMatrix (tmp, x);
	}
	else {
		printf ("tmp == x !\n");
		exit (-1);
	}
}

// Do C = A * B
void C_Eq_AB (KMatrix *A, KMatrix *B, KMatrix *C)
{
/* c := alpha*op(a)*op(b) + beta*c,
where c is an m-by-n matrix,
op(a) is an m-by-k matrix,
op(b) is a k-by-n matrix.
dgemm (transa, transb, m, n, k, alpha, a, lda, b, ldb, beta, c, ldc)*/
  cblas_dgemm (CblasRowMajor, CblasNoTrans, CblasNoTrans, C->nRow, C->nCol, A->nCol, 1.0, A->Element, A->nCol, B->Element, B->nCol, 0.0, C->Element, C->nCol);
}

void C_Eq_AtB  (KMatrix *A, KMatrix *B, KMatrix *C)
{
	cblas_dgemm (CblasRowMajor, CblasTrans, CblasNoTrans, C->nRow, C->nCol, A->nRow, 1.0, A->Element, C->nRow, B->Element, B->nCol, 0.0, C->Element, C->nCol);
}

// Do y = A' * x
// Does not take the dimensions of x into account
void Y_Eq_ATranspX (KMatrix *A, KMatrix *x, KMatrix *y)
{
	cblas_dgemv (CblasRowMajor, CblasTrans, A->nRow, A->nCol, 1.0, A->Element, A->nCol, x->Element, 1, 0.0, y->Element, 1);
}

// Do z = A * x + y
// The x,y,z pointers must be different
// One doesn't consider the y size. It is determined by A * x
// y is modified by the procedure, it is set equal to z
void Z_Eq_AX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y, KMatrix *z)
{
	cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, 1.0, A->Element, A->nCol, x->Element, 1, 1.0, y->Element, 1);
	//CopyRawMatrix (y, z);		// The result must morf z
	cblas_dcopy (z->nRow, y->Element, 1, z->Element, 1);
}

// Do z = -A * x + y
// y = z !!!!
void Z_Eq_MinusAX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y, KMatrix *z)
{
	cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, -1.0, A->Element, A->nCol, x->Element, 1, 1.0, y->Element, 1);
	//CopyRawMatrix (y, z);		// The result should morf z
	cblas_dcopy (z->nRow, y->Element, 1, z->Element, 1);
}

// Do y = -A * x + y
void	Y_Eq_MinusAX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y)
{
	cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, -1.0, A->Element, A->nCol, x->Element, 1, 1.0, y->Element, 1);
}
// Do y = A * x + y
void	Y_Eq_AX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y)
{
	cblas_dgemv (CblasRowMajor, CblasNoTrans, A->nRow, A->nCol, 1.0, A->Element, A->nCol, x->Element, 1, 1.0, y->Element, 1);
}
// Fill matrix with pData
void SetMatrix (KMatrix *pMat, KMATRIX_TYPE	*pData)
{
	memcpy (pMat->Element, pData, pMat->nRow * pMat->nCol * sizeof (KMATRIX_TYPE));
}

void FillMatrixFromTriangle66(KMatrix *dest, KMATRIX_TYPE *triangle)
{
	int l, c, i = 0;

	for (l = 0; l < 6; l++)
	{
		for (c = l; c < 6; c++)
				dest->Element[c+6*l] = triangle[i++];
	}

	i = 0;
	for (c = 0; c < 6; c++)
	{
		for (l = c; l < 6; l++)
			dest->Element[c+6*l] = triangle[i++];
	}
}

int MatCompare (KMatrix *MatA, KMatrix *MatB)
{
  int i;
  if(MatA->nRow != MatB->nRow) return 0;
  if(MatA->nCol != MatB->nCol) return 0;

  for (i = 0; i < MatA->nRow * MatA->nCol; i++)
  {
    if(MatA->Element[i] != MatB->Element[i]) return 0;
  }

  return 1;
}






