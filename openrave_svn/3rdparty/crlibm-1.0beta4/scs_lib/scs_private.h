/** Various declarations and macros shared by
   several .c files, but useless to users of the library 

 @file scs_private.h

@author Defour David David.Defour@ens-lyon.fr
@author Florent de Dinechin Florent.de.Dinechin@ens-lyon.fr 
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
#ifndef SCS_PRIVATE_H
#define SCS_PRIVATE_H 1


#define SCS_RADIX   ((unsigned int)(1<<SCS_NB_BITS))

#define SCS_MASK_RADIX ((unsigned int)(SCS_RADIX-1))

#include "scs.h"

#ifdef WORDS_BIGENDIAN
 #define HI 0
 #define LO 1 
#else
 #define HI 1
 #define LO 0
#endif

/* An int such that SCS_MAX_RANGE * SCS_NB_BITS < 1024, 
   where 1024 is the max of the exponent of a double number.
   Used in scs2double.c along with radix_rng_double et al. 
   The value of 32 is OK for all practical values of SCS_NB_BITS */       
#define SCS_MAX_RANGE  32

/*
 * DEFINITION OF DOUBLE PRECISION FLOATING POINT NUMBER CONSTANTS 
 */
/* In all the following "radix" means 2^(SCS_NB_BITS),
   and radix_blah means radix^blah.
   (1023 + e)<<20 is the way to cast e into the exponent field of an IEEE-754 double
 */

 
 extern const db_number radix_one_double ; 
 extern const db_number radix_two_double ; 
 extern const db_number radix_mone_double; 
 extern const db_number radix_mtwo_double; 
 extern const db_number radix_rng_double ; 
 extern const db_number radix_mrng_double;
 extern const db_number max_double       ; 
 extern const db_number min_double       ; 


#define SCS_RADIX_ONE_DOUBLE     radix_one_double.d   /* 2^(SCS_NB_BITS)           */ 
#define SCS_RADIX_TWO_DOUBLE     radix_two_double.d   /* 2^(2.SCS_NB_BITS)         */
#define SCS_RADIX_MONE_DOUBLE    radix_mone_double.d  /* 2^-(SCS_NB_BITS)          */ 
#define SCS_RADIX_MTWO_DOUBLE    radix_mtwo_double.d  /* 2^-(2.SCS_NB_BITS)        */ 
#define SCS_RADIX_RNG_DOUBLE     radix_rng_double.d   /* 2^(SCS_NB_BITS.SCS_MAX_RANGE) */
#define SCS_RADIX_MRNG_DOUBLE    radix_mrng_double.d  /* 2^-(SCS_NB_BITS.SCS_MAX_RANGE)*/
#define SCS_MAX_DOUBLE           max_double.d         /* 2^1024-1              */
#define SCS_MIN_DOUBLE           min_double.d         /* 2^-1074             */






#define R_HW  result->h_word
#define R_SGN result->sign
#define R_IND result->index
#define R_EXP result->exception.d

#define X_HW  x->h_word
#define X_SGN x->sign
#define X_IND x->index
#define X_EXP x->exception.d

#define Y_HW  y->h_word
#define Y_SGN y->sign
#define Y_IND y->index
#define Y_EXP y->exception.d

#define Z_HW  z->h_word
#define Z_SGN z->sign
#define Z_IND z->index
#define Z_EXP z->exception.d

#define W_HW  w->h_word
#define W_SGN w->sign
#define W_IND w->index
#define W_EXP w->exception.d



/* A few additional defines for the case when we use floating-point
   multiplier  (OBSOLETE, NEVER USED ANYMORE but who knows, some day) */

#ifdef SCS_USE_FLT_MULT
/* There is a "53" below, which means that these constants won't do
   what we expect from them on x86 because of the double extended
   precision. We could put more ifdefs, but why care, nobody wants to use the
   FP muls on the x86. */
#ifdef WORDS_BIGENDIAN
 static const db_number scs_flt_trunc_cst = {{ ((1023+SCS_NB_BITS-1)<<20) ,           0x00000000 }}; 
 static const db_number scs_flt_shift_cst = {{ ((1023+SCS_NB_BITS+53)<<20),0x00000000}}; 
#else
 static const db_number scs_flt_trunc_cst = {{ 0x00000000, ((1023+SCS_NB_BITS-1)<<20) }}; 
 static const db_number scs_flt_shift_cst = {{ 0x00000000 ,((1023+SCS_NB_BITS+53)<<20)}}; 
#endif /*WORDS_BIGENDIAN*/

#define SCS_FLT_TRUNC_CST  scs_flt_trunc_cst.d    /* 2^(SCS_NB_BITS+53-1) */
#define SCS_FLT_SHIFT_CST  scs_flt_shift_cst.d    /* 2^(SCS_NB_BITS)(1+1/2) */
#endif /* SCS_USE_FLTMULT */

#endif /* SCS_PRIVATE_H */
