/***************************************************************************
                          random.h  -  description
                             -------------------
    begin                : Sat Oct 26 2002
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

#define CUSTOM_RAND_MAX		(0xFFFFFFFF)

/* Combination of 3 tausworth generators -- assumes 32-bit integers */
void init_random_generator ();
unsigned int CustomRand();             /* returns a random 32 bit integer */
unsigned int GetRandomInt (unsigned int MaxRange);
void rand_seed( unsigned int, unsigned int, unsigned int );
double Gaussrand(double mean, double cov);
double GetRandomDouble (double MaxRange);
