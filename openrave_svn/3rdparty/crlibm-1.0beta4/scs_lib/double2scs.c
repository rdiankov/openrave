/** Conversion of floating-point double to SCS
@file double2scs.c

@author Defour David David.Defour@ens-lyon.fr
@author Florent de Dinechin Florent.de.Dinechin@ens-lyon.fr 

This file is part of the SCS library.
*/

/*
Copyright (C) 2002  David Defour and Florent de Dinechin

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */
#include "scs.h"
#include "scs_private.h"


/** Convert a double precision number in it SCS multiprecision
  representation
 */

void scs_set_d(scs_ptr result, double x){
  db_number nb, mantissa;
  int exponent, exponent_remainder;
  int ind, i;

  if(x>=0){R_SGN = 1;    nb.d = x;}
  else    {R_SGN = -1;   nb.d = -x;}

  exponent = nb.i[HI] & 0x7ff00000 ;

  if (exponent == 0x7ff00000)  {
    /*
     * x = +/- Inf, s/qNAN
     */
    R_EXP = x;  
    for(i=0; i<SCS_NB_WORDS; i++)
      R_HW[i] = 0;
 
    R_IND = 0;
    R_SGN = 1;
  } 

  else {    /* Normals,  denormals, +/- 0.  */

    /* This number is not an exception */
    R_EXP = 1;

#if 1

    if (exponent == 0){
      /* x is a denormal number : bring it back to the normal range */
      nb.d = nb.d * SCS_RADIX_TWO_DOUBLE;      /* 2^(2.SCS_NB_BITS) */
      exponent = nb.i[HI] & 0x7ff00000 ;
      R_IND = -2;
    }else {
      R_IND = 0;
    }

    exponent = exponent >> 20;  /* get the actual value */

    ind = ((exponent +((100*SCS_NB_BITS)-1023))/SCS_NB_BITS) - 100 ;  
    /* means : = (exponent -1023 + 100*SCS_NB_BITS)/SCS_NB_BITS -100 
     The business with 100*SCS_NB_BITS is to stay within the positive
     range for exponent_remainder between 1 and SCS_NB_BITS */

    exponent_remainder = exponent - 1022 - (SCS_NB_BITS*ind);

    R_IND += ind;

    /* now get the mantissa and add the implicit 1 in fp. format*/
    mantissa.l = (nb.l & ULL(000fffffffffffff)) | ULL(0010000000000000);


    /* and spread it over the structure
       Everything here is 64-bit arithmetic */
    R_HW[0] = (unsigned int) (mantissa.l >> (53 - exponent_remainder) );

    /* 11 = 64-53 */
    mantissa.l =  (mantissa.l << (exponent_remainder+11));
    R_HW[1] = (mantissa.i[HI] >> (32 - SCS_NB_BITS))& SCS_MASK_RADIX ;
    mantissa.l =  (mantissa.l << SCS_NB_BITS);
    R_HW[2] = (mantissa.i[HI] >> (32 - SCS_NB_BITS))& SCS_MASK_RADIX ;
#if SCS_NB_BITS < 27
    mantissa.l =  (mantissa.l << SCS_NB_BITS);
    R_HW[3] = (mantissa.i[HI] >> (32 - SCS_NB_BITS))& SCS_MASK_RADIX ;
#else
    R_HW[3] = 0 ;
#endif

#if (SCS_NB_WORDS==8)
      R_HW[4] = 0; R_HW[5] = 0; R_HW[6] = 0; R_HW[7] = 0;
#else
    for(i=4; i<SCS_NB_WORDS; i++)
      R_HW[i] = 0;
#endif



#else /* Other algorithm as in the research report. Slower */
    R_IND = 0;

    while(nb.d>SCS_RADIX_ONE_DOUBLE) {
	R_IND++;
	nb.d *= SCS_RADIX_MONE_DOUBLE;
      }

    while(nb.d<1)  {
	R_IND--;
	nb.d *= SCS_RADIX_ONE_DOUBLE;
      }

    i=0;
    while(nb.d != 0){
      R_HW[i] = (unsigned int) nb.d;
      nb.d = (nb.d - (double)R_HW[i]) * SCS_RADIX_ONE_DOUBLE;
      i++;
    }
    for(; i<SCS_NB_WORDS; i++)
      R_HW[i] = 0;

#endif

  } /* end if test NaN etc */

  return;
}


/**
  Convert an integer number in it scs multiprecision
  representation
 */
void scs_set_si(scs_ptr result, int x){
  unsigned int ux;
  int i;
  
  if(x>=0){R_SGN = 1;   ux = (unsigned int)x;}
  else    {R_SGN = -1;  ux = (unsigned int)-x;}
  

  if (ux > SCS_RADIX){
    R_IND   = 1;
    R_HW[0] = (ux - SCS_RADIX) >> SCS_NB_BITS;
    R_HW[1] =  ux - (R_HW[0] << SCS_NB_BITS);
  }else {
    R_IND   = 0;
    R_HW[0] = ux;
    R_HW[1] = 0;
  }

  for(i=2; i<SCS_NB_WORDS; i++)
    R_HW[i] = 0;

  if (x != 0)  R_EXP = 1;
  else         R_EXP = 0;
  
  return;
}

