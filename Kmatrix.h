/***************************************************************************
                          Kmatrix.h  -  description
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
#ifndef _KMATRIX_NEW_H
#define _KMATRIX_NEW_H

#ifdef RTAI
#include <linux/slab.h>
#else
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "random.h"
#endif

#include "atlas_enum.h"
#include "clapack.h"

typedef double KMATRIX_TYPE;

typedef struct _matrix
{
        int     nRow;
        int     nCol;
        KMATRIX_TYPE    *Element;
}KMatrix;

#ifndef NULL
#define NULL 0
#endif

#define   MATRIX_ELEMENT(pMat,r,c) ((pMat)->Element[(c) + (r) * (pMat)->nCol])
#define   SET_MATRIX_ELEMENT(pMat,r,c,val) ((pMat)->Element[(c) + (r) *(pMat)->nCol] = val)

void	AllocMatrix (KMatrix *pMat, int nRow, int nCol);
void	DeleteMatrix (KMatrix *pMat);
void	CopyBlockMatrix (KMatrix *src, KMatrix *dest);
void	CopyPartMatrix (KMatrix *src, KMatrix *dest, int BeginRow, int BeginCol);
void	SetSubMatrix (KMatrix *dest, KMATRIX_TYPE *data, int BeginRow, int BeginCol, int nRows, int nCols);
void	PastePartMatrix (KMatrix *src, int CopyRow, int CopyCol, int CopyHeight, int CopyLength, KMatrix *dest, int PasteRow, int PasteCol);
void	CopyRawMatrix (KMatrix *src, KMatrix *dest);
void CreateHomogenMatrix (float roll, float pitch, float yaw, float x, float y, float z, KMatrix *h44);
double	MatrixElement (KMatrix *pMat, int r, int c);
void	SetMatrixElement (KMatrix *pMat, int r, int c, KMATRIX_TYPE val);
double	*MatrixElPointer (KMatrix *pMat, int r, int c);
void	FillMatrix (KMatrix *pMat, KMATRIX_TYPE val);
void CreateRPYMatrix(float roll, float pitch, float yaw, KMatrix *rot33);
void	FillRandom (KMatrix *pMat);
void	CreateIdentityMatrix (KMatrix *Imat, int dim);
void	SetIdentityMatrix (KMatrix *Imat);
void	SetDiagonalMatrix (KMatrix *Diagmat, KMATRIX_TYPE val);
void	MultiplyMatrix (KMatrix *mat, KMATRIX_TYPE val);
void	ZeroMatrix (KMatrix *mat);
int	IsPositiveDefinite (KMatrix *pMat);
void FillSymmetricMatrix (KMatrix *mat, KMATRIX_TYPE *sym);
void	PrintMatrix (KMatrix *pMat, char *MatrixName);
void	PrintMatrixStderr (KMatrix *pMat, char *MatrixName);
void	A_Eq_kA (KMATRIX_TYPE k, KMatrix *A);
void	A_Eq_APlusB  (KMatrix *A, KMatrix *B);
void	A_Eq_AMinusB  (KMatrix *A, KMatrix *B);
void	A_Eq_MinusAPlusB  (KMatrix *A, KMatrix *B);
void  ABAt (KMatrix *A, KMatrix *B, KMatrix *Result, KMatrix *Tmp);
void	ABAt_Plus_CDCt (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *D, KMatrix *Result);
void	ABAt_Plus_C (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *Result, KMatrix *Tmp, char plusOrMinus);
void	AtBA_Plus_C (KMatrix *A, KMatrix *B, KMatrix *C, KMatrix *Result, KMatrix *Tmp);
void  C_Eq_ABPlusC (KMatrix *A, KMatrix *B, KMatrix *C);
void	ComputeInverse (KMatrix *Mat, int *iPiv);
int   MatCompare (KMatrix *MatA, KMatrix *MatB);
void	X_Eq_X_Minus_Y(KMatrix *x, KMatrix *y);
void	Y_Eq_X_Plus_Y(KMatrix *x, KMatrix *y);
void  SetVector (KMatrix *v, KMATRIX_TYPE x, KMATRIX_TYPE y, KMATRIX_TYPE z);
void  Z_Eq_XCrossY (KMatrix *x, KMatrix *y, KMatrix *z);
void	Y_Eq_AX (KMatrix *A, KMatrix *x, KMatrix *y);
void  X_Eq_AX (KMatrix *A, KMatrix *x, KMatrix *tmp);
void  C_Eq_AB (KMatrix *A, KMatrix *B, KMatrix *C);
void  C_Eq_AtB  (KMatrix *A, KMatrix *B, KMatrix *C);
void	Y_Eq_ATranspX (KMatrix *A, KMatrix *x, KMatrix *y);
void	Z_Eq_AX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y, KMatrix *z);
void	Z_Eq_MinusAX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y, KMatrix *z);
void	Y_Eq_MinusAX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y);
void	Y_Eq_AX_Plus_Y (KMatrix *A, KMatrix *x, KMatrix *y);
void	SetMatrix (KMatrix *pMat, KMATRIX_TYPE *pData);
void  getRollPitchYaw(KMATRIX_TYPE *rpy, KMatrix	*homo);
void  FillMatrixFromTriangle66(KMatrix *dest, KMATRIX_TYPE *triangle);

#endif
