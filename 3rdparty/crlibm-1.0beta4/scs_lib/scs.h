/** This is the main header file of the SCS library, which defines the
    SCS data structure, and the functions that implement arithmetic on it.

@file scs.h

@author David Defour David.Defour@ens-lyon.fr
@author Florent de Dinechin Florent.de.Dinechin@ens-lyon.fr

This file is part of the SCS library.

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



/* Avoid loading the header twice */
#ifndef INCLUDE_SCS 
#define INCLUDE_SCS 1

#ifndef DOXYGEN_SHOULD_SKIP_THIS /* because it is not very clean */

#ifdef HAVE_CONFIG_H
#include "../crlibm_config.h"
#endif
#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif



/* 64 bit arithmetic may be standardised, but people still do want they want */
#ifdef HAVE_INTTYPES_H
#define ULL(bits) 0x##bits##uLL
#elif defined(WIN32) 
/* TODO insert Windows garbage there */
/* Default, hoping it works, hopefully less and less relevant */
#else
typedef long long int64_t;
typedef unsigned long long uint64_t; 
#define ULL(bits) 0x##bits##uLL
#endif

#ifndef SCS_DEF_INT64 
#define SCS_DEF_INT64
#ifdef SCS_TYPEOS_HPUX
#ifndef __LP64__ /* To solve the problem with 64 bits integer */
typedef long long int64_t;
typedef unsigned long long uint64_t;
#define ULL(bits) 0x##bits##uLL
#endif
#endif
#endif


#ifdef HAVE_GMP_H
 #include <gmp.h>
#endif

#ifdef HAVE_MPFR_H
 #include <mpfr.h>
#endif


#endif /* DOXYGEN_SHOULD_SKIP_THIS */


/** @internal An union to cast floats into doubles or the other way round. For
    internal purpose only */

typedef union {
  int32_t i[2]; /* Signed (may be useful) */                
  int64_t l;    /* Signed (may be useful) */
  double d;
} db_number;





/* ****************************************************************** */
/**@name SCS data-types */ /**@{*/

/** @struct scs 
The SCS data type. 

An SCS number is a a floating-point number in base 2^32.

- Its mantissa is formed of SCS_NB_WORDS digits (currently 32 bits by default)

- Its exponent is a 32-bit integer

- It also has a sign field, and an exception field used to store and
  propagate IEEE-754 exceptions.


The real number represented by a scs structure is equal to:
@f$
\displaystyle 
\sum_{i=0}^{\mathtt{SCS\_NB\_WORDS}} 
2^{(\mathtt{index} -i)\mathtt{SCS\_NB\_BITS}}
\times
\mathtt{h\_word}[i] 
@f$
*/

/*
  (verbatim-mode formula for the above eqation:) the number represented by a
  SCS structure is :

         i<SCS_NB_WORDS               (index - i).SCS_NB_BITS
 sign . (    sum       ( h_word[i] . 2^                        ) 
            i=0 
*/

struct scs {
  /** the digits, as 32 bits words */ 
  uint32_t h_word[SCS_NB_WORDS]; 
  /** Used to store Nan,+/-0, Inf, etc and then let the hardware handle them */
  db_number exception;   
  /** This corresponds to the exponent in an FP format, but here we are
     in base 2^32  */
  int index;        
  /** The sign equals 1 or -1*/
  int sign;              
};


typedef struct scs scs;



/** scs_ptr is a pointer on a SCS structure */
typedef struct scs * scs_ptr;




/** scs_t is an array of one SCS struct to lighten syntax : you may
 declare a scs_t object, and pass it to the scs functions (which
 expect pointers) without using ampersands.
*/
typedef struct scs scs_t[1];

/**@}*/ /* end doxygen group for SCS data-types */








/* ****************************************************************** */
/**@name Conversion and initialization functions  */ /**@{*/

/** Convert a SCS number to a double, rounding to the nearest */
void scs_get_d(double*, scs_ptr);

/** Convert a SCS number to a double, rounding towards minus infinity */
void scs_get_d_minf(double*, scs_ptr);

/** Convert a SCS number to a double, rounding towards plus infinity */
void scs_get_d_pinf(double*, scs_ptr);

/** Convert a SCS number to a double, rounding towards zero */
void scs_get_d_zero(double*, scs_ptr);

/** Convert a double into a SCS number (this is an exact operation) */
void scs_set_d(scs_ptr, double);

/** Convert a signed int into a SCS number (this is an exact operation) */
void scs_set_si(scs_ptr, signed int);


/** Print out a SCS number. Sorry for the strange name, we are mimicking GMP */
void scs_get_std(scs_ptr);


/** Copy a SCS number into another */
void scs_set(scs_ptr, scs_ptr); 


/** Set a SCS number to zero */
void scs_zero(scs_ptr);


/** Generate a random SCS number. 
   The index field of result will be between -expo_max and +expo_max.
   Example: to get a number in the double-precision floating-point range,
   expo_max should be smaller than 39.
   @warning No guarantee is made about the quality of the random algorithm
   used... */
void scs_rand(scs_ptr result, int expo_max);

/**@}*/ /* end doxygen group for conversion / initialisation functions*/




/* ****************************************************************** */
/**@name Addition and renormalisation functions  */ /**@{*/


/** Addition of two SCS numbers.  
 The arguments x, y and result may point to the same memory
 location. The result is a normalised SCS number.
*/
void scs_add(scs_ptr result, scs_ptr x, scs_ptr y);


/** Subtraction of two SCS numbers.  
 The arguments x, y and result may point to the same memory
 location. The result is a normalised SCS number.
 */
void scs_sub(scs_ptr result, scs_ptr x, scs_ptr y);


/** Addition without renormalisation, to be used for adding many
   numbers.
  @warning  In case of a cancellation, severe loss of precision could
   happen. Safe if the numbers are of the same sign.
 */
void scs_add_no_renorm(scs_ptr result, scs_ptr x, scs_ptr y);


/** Renormalisation (to be used after several scs_add_no_renorm).
This function removes the carry from each digit, and also shifts the
digits in case of a cancellation (so that if result != 0 then its
first digit is non-zero)
 
 @warning THIS FUNCTION HAS NEVER BEEN PROPERLY TESTED and is
 currently unused in the library: instead, specific renormalisation
 steps are fused within the code of the operations which require it.
  */

void scs_renorm(scs_ptr);


/** Renormalisation assuming no cancellation.  This function is useful
   for example when adding many numbers of the same sign */
void scs_renorm_no_cancel_check(scs_ptr);

/**@}*/ /* end doxygen group for addition and normalisation functions*/




/* ****************************************************************** */
/**@name Multiplication functions */ /**@{*/

/** Multiplication of two SCS numbers.  The arguments x, y and result
 may point to the same memory location. The result is a normalised SCS
 number.
 */
void scs_mul(scs_ptr result, const scs_ptr x, const scs_ptr y);

/** Multiplication of a SCS with an unsigned integer; result is
    returned in x. */
void scs_mul_ui(scs_ptr, const unsigned int);

/** Square. Result is normalised */
void scs_square(scs_ptr result, scs_ptr x);

/** Fused multiply-and-add (ab+c);   Result is normalised
\warning This function has not been tested thoroughly */
void scs_fma(scs_ptr result,  scs_ptr a,  scs_ptr b,  scs_ptr c);

/**@}*/ /* end doxygen group for Multiplication functions*/





/* ****************************************************************** */
/**@name Divisions */ /**@{*/

/** SCS inverse.  
Stores 1/x in result. Result is normalised 

@warning This function is known not to work for most precisions: it
performs a fixed number of Newton-Raphson iterations (two), starting
with a FP number (53 bits), so provides roughly 210 bits of
precision. It should be modified to perform more iterations if more
precision is needed.
*/
void scs_inv(scs_ptr result, scs_ptr x);

/** SCS division. Computes x/y. Result is normalised 
@warning This function is known not to work for most precisions: it
performs a fixed number of Newton-Raphson iterations (two), starting
with a FP number (53 bits), so provides roughly 210 bits of
precision. It should be modified to perform more iterations if more
precision is needed.
*/
void scs_div(scs_ptr result, scs_ptr x, scs_ptr y);


/** SCS division by 2. Computes x/2. Result is normalised */ 
void scs_div_2(scs_t x);

/**@}*/ /* end doxygen group for division functions*/





/* ****************************************************************** */
/**@name Functions for testing purpose */ /**@{*/


#ifdef HAVE_LIBGMP
/** Convert a SCS number into a GMP MPF (multiple precision,
   floating-point) number. Should be exact if the target number has
   more precision than the SCS number, otherwise the rounding is
   unspecified (the conversion uses MPF functions) */
void scs_get_mpf(scs_ptr, mpf_t);
#endif



#ifdef HAVE_MPFR_H 
/**  Convert a SCS number into a MPFR (multiple precision,
   floating-point) number. Should be exact if the target number has
   more precision than the SCS number, otherwise should be correctly
   rounded (the conversion uses MPFR functions). Not heavily tested
   though */
void scs_get_mpfr(scs_ptr, mpfr_t);
#endif


/**@}*/ /* end doxygen group for functions for testing purpose */

#endif /* INCLUDE_SCS */





