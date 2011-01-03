/*
 * Author  : David Defour, Catherine Daramy, Florent de Dinechin, Christoph Lauter
 * Contact : David.Defour@ens-lyon.fr, catherine_daramy@ens-lyon.fr
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  
 */

#ifndef CRLIBM_H
#define CRLIBM_H

#if defined (__cplusplus)
extern "C" {
#endif


/* An init function which sets FPU flags when needed (mostly on Intel
   architectures with default double extended) */
extern unsigned long long crlibm_init(void);

/* An exit function which restores FPU flags when needed (mostly on Intel
   architectures with default double extended) */
extern  void crlibm_exit(unsigned long long);


/* Finished functions */
/* These functions are computed in two steps and have an average
   execution time comparable to that of a standard libm
*/

/*  exponential  */
extern double exp_rn(double); /* to nearest  */
extern double exp_rd(double); /* toward -inf */ 
extern double exp_ru(double); /* toward +inf */ 
#define exp_rz exp_rd         /* toward zero */ 

/*  logarithm  */
extern double log_rn(double); /* to nearest  */
extern double log_rd(double); /* toward -inf */ 
extern double log_ru(double); /* toward +inf */ 
extern double log_rz(double); /* toward zero */ 

/*  cosine  */
extern double cos_rn(double); /* to nearest  */
extern double cos_rd(double); /* toward -inf */ 
extern double cos_ru(double); /* toward +inf */ 
extern double cos_rz(double); /* toward zero */ 

/*  sine  */
extern double sin_rn(double); /* to nearest  */
extern double sin_rd(double); /* toward -inf */ 
extern double sin_ru(double); /* toward +inf */ 
extern double sin_rz(double); /* toward zero */ 

/*  tangent  */
extern double tan_rn(double); /* to nearest  */
extern double tan_rd(double); /* toward -inf */ 
extern double tan_ru(double); /* toward +inf */
extern double tan_rz(double); /* toward zero */
 
/*  cosine of pi times x  */
extern double cospi_rn(double); /* to nearest  */
extern double cospi_rd(double); /* toward -inf */ 
extern double cospi_ru(double); /* toward +inf */ 
extern double cospi_rz(double); /* toward zero */ 

/*  sine  of pi times x */
extern double sinpi_rn(double); /* to nearest  */
extern double sinpi_rd(double); /* toward -inf */ 
extern double sinpi_ru(double); /* toward +inf */ 
extern double sinpi_rz(double); /* toward zero */ 

/*  tangent of pi times x */
extern double tanpi_rn(double); /* to nearest  */
extern double tanpi_rd(double); /* toward -inf */ 
extern double tanpi_ru(double); /* toward +inf */
extern double tanpi_rz(double); /* toward zero */
 



/* /\*  cotangent  *\/ */
/* extern double cotan_rn(double); /\* to nearest  *\/ */
/* extern double cotan_rd(double); /\* toward -inf *\/  */
/* extern double cotan_ru(double); /\* toward +inf *\/  */
/* extern double cotan_rz(double); /\* toward zero *\/  */

/*  arctangent  */
extern double atan_rn(double); /* to nearest  */
extern double atan_rd(double); /* toward -inf */ 
extern double atan_ru(double); /* toward +inf */ 
extern double atan_rz(double); /* toward zero */ 

/*  arctangentPi  */
extern double atanpi_rn(double); /* to nearest  */
extern double atanpi_rd(double); /* toward -inf */ 
extern double atanpi_ru(double); /* toward +inf */ 
extern double atanpi_rz(double); /* toward zero */ 

/*  hyperbolic cosine*/
extern double cosh_rn(double); /* to nearest */
extern double cosh_rd(double); /* toward -inf */ 
extern double cosh_ru(double); /* toward +inf */ 
extern double cosh_rz(double); /* toward zero */ 

/*  hyperbolic sine */
extern double sinh_rn(double); /* to nearest */
extern double sinh_rd(double); /* toward -inf */ 
extern double sinh_ru(double); /* toward +inf */ 
extern double sinh_rz(double); /* toward zero */ 


/* base 2 logarithm */
extern double log2_rn(double); /* to nearest  */
extern double log2_rd(double); /* toward -inf */ 
extern double log2_ru(double); /* toward +inf */ 
extern double log2_rz(double); /* towards zero */ 

/* base 10 logarithm */
extern double log10_rn(double); /* to nearest  */
extern double log10_rd(double); /* toward -inf */ 
extern double log10_ru(double); /* toward +inf */ 
extern double log10_rz(double); /* towards zero */ 

/* arcsine */
extern double asin_rn(double); /* to nearest */
extern double asin_rd(double); /* toward -inf */
extern double asin_ru(double); /* toward +inf */
extern double asin_rz(double); /* toward zero */

/* arccosine */
extern double acos_rn(double); /* to nearest */
extern double acos_rd(double); /* toward -inf */
extern double acos_ru(double); /* toward +inf */
#define acos_rz acos_rd        /* toward zero */ 

/* arcsine/PI */
extern double asinpi_rn(double); /* to nearest */
extern double asinpi_rd(double); /* toward -inf */
extern double asinpi_ru(double); /* toward +inf */
extern double asinpi_rz(double); /* toward zero */

/* arccosine/PI */
extern double acospi_rn(double); /* to nearest */
extern double acospi_rd(double); /* toward -inf */
extern double acospi_ru(double); /* toward +inf */
#define acospi_rz acospi_rd      /* toward zero */ 

/* expm1 = e^x -1 */
extern double expm1_rn(double); /* to nearest */
extern double expm1_rd(double); /* toward -inf */
extern double expm1_ru(double); /* toward +inf */
extern double expm1_rz(double); /* toward zero */

/* log1p = log(1 + x) */
extern double log1p_rn(double); /* to nearest */
extern double log1p_rd(double); /* toward -inf */
extern double log1p_ru(double); /* toward +inf */
extern double log1p_rz(double); /* toward zero */


/* Unfinished functions */
/* These functions provide correct rounding but are very slow
   (typically 100 times slower that the standard libm) */

extern double exp2_rn(double); /* to nearest  */
extern double exp2_rd(double); /* toward -inf */ 
extern double exp2_ru(double); /* toward +inf */ 

/*  pow */
/* ATTENTION: THIS FUNCTION IS UNDER DEVELOPMENT 
   AND CURRENTLY NOT PROVEN CORRECTLY ROUNDED FOR ALL CASES

   See the documentation

*/
extern double pow_rn(double, double); 


/* fi_lib-compatible interval functions (EXPERIMENTAL) */ 

#ifdef BUILD_INTERVAL_FUNCTIONS
#include "interval.h"
interval j_log(interval x);
interval j_exp(interval x);
#endif /* BUILD_INTERVAL_FUNCTIONS */

#if defined (__cplusplus)

}
#endif

#endif /* ifdef CRLIBM_H*/
