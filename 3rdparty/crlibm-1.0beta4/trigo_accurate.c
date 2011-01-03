/*
 * Correctly rounded trigonometric functions
 *
 * Author : Catherine Daramy, David Defour, Florent de Dinechin
 *
 * This file is part of the crlibm library developed by the Arenaire
 * project at Ecole Normale Superieure de Lyon
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
#include <stdio.h>
#include <stdlib.h>
#include "trigo_accurate.h"


/*
 *
 * 1) Range reduction if needed ... x in [-Pi/4, +Pi/4]
 *
 * 2) call cosine, sine or tan polynomial
 *
 * Polynomials are vastly too accurate.
 */   



extern int rem_pio2_scs(scs_ptr, scs_ptr);


/* Polynomial evaluation of sin(x) over [-Pi/4, +Pi/4] 	
   Approximation error lower than  2^(-133) */

static void scs_sin(scs_ptr x){
  scs_t res_scs;
  scs_t x2;
  int i;
 
  scs_square(x2, x);
  scs_mul(res_scs, sin_scs_poly_ptr[0], x2);

  for(i=1; i<(DEGREE_SIN_SCS-1)/2; i++){ /* Last coeff is one, not read from the file*/
    scs_add(res_scs, sin_scs_poly_ptr[i], res_scs);
    scs_mul(res_scs, res_scs, x2);
  } 
  scs_mul(res_scs, res_scs, x);
  scs_add(x, x, res_scs);

  return;
}


/* Polynomial evaluation of cos(x) over [-Pi/4, +Pi/4] 
   Approximation error lower than  2^(-128) */

static void scs_cos(scs_ptr x){
  scs_t res_scs;
  scs_t x2;
  int i;

  scs_square(x2, x);
  scs_mul(res_scs, cos_scs_poly_ptr[0], x2);
  for(i=1; i<DEGREE_COS_SCS/2; i++){
    scs_add(res_scs, cos_scs_poly_ptr[i], res_scs);
    scs_mul(res_scs, res_scs, x2);
  }
  /* The last coefficient is exactly one and is not read from the file */
  scs_add(x, res_scs, SCS_ONE);

  return ;
}








double scs_sin_rn(double x){  
  scs_t sc1, sc2;
  double resd;
  int N;


#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_sin(sc2);
    scs_get_d(&resd, sc2);
    return resd;
  case 1:
    scs_cos(sc2);
    scs_get_d(&resd, sc2);
    return resd;
  case 2:
    scs_sin(sc2);
    scs_get_d(&resd, sc2);
    return -resd;		
  case 3:
    scs_cos(sc2);
    scs_get_d(&resd, sc2);
    return -resd;
    default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_scs_sin \n", N);
    return 0.0;
  }
}




double scs_sin_rd(double x){  
  scs_t sc1, sc2;
  double resd;
  int N;
    
#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_sin(sc2);
    scs_get_d_minf(&resd, sc2);
    return resd;
  case 1:
    scs_cos(sc2);
    scs_get_d_minf(&resd, sc2);
    return resd;
  case 2:  
    scs_sin(sc2);
    scs_get_d_pinf(&resd, sc2);
    return -resd;
  case 3:
    scs_cos(sc2);
    scs_get_d_pinf(&resd, sc2);
    return -resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_scs_sin \n", N);
    exit(1);
  }
  return resd;
}




double scs_sin_ru(double x){  
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_sin(sc2);
    scs_get_d_pinf(&resd, sc2);
    return resd;
  case 1:
    scs_cos(sc2);
    scs_get_d_pinf(&resd, sc2);
    return resd;
   case 2:
    scs_sin(sc2);
    scs_get_d_minf(&resd, sc2);
    return -resd;
  case 3:
    scs_cos(sc2);
    scs_get_d_minf(&resd, sc2);
    return -resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_scs_sin \n", N);
    exit(1);
  }
  return resd;
}




double scs_sin_rz(double x){  
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_sin(sc2);
    scs_get_d_zero(&resd, sc2);
    return resd;
  case 1:
    scs_cos(sc2);
    scs_get_d_zero(&resd, sc2);
    return resd;
   case 2:
    scs_sin(sc2);
    scs_get_d_zero(&resd, sc2);
    return -resd;
  case 3:
    scs_cos(sc2);
    scs_get_d_zero(&resd, sc2);
    return -resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_scs_sin \n", N);
    exit(1);
  }
  return resd;
}







double scs_cos_rn(double x){ 
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_cos(sc2);
    scs_get_d(&resd, sc2);
    return resd;
  case 1:
    scs_sin(sc2);
    scs_get_d(&resd, sc2);
    return -resd;
  case 2:
    scs_cos(sc2);
    scs_get_d(&resd, sc2);
    return -resd;
  case 3:
    scs_sin(sc2);
    scs_get_d(&resd, sc2);
    return resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_cos \n", N);
    return 0.0;
  }
  
}


double scs_cos_rd(double x){ 
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_cos(sc2);
    scs_get_d_minf(&resd, sc2);
    return resd;
  case 1:
    scs_sin(sc2); 
    scs_get_d_pinf(&resd, sc2);
    return -resd;
  case 2:
    scs_cos(sc2); 
    scs_get_d_pinf(&resd, sc2);
    return -resd;
  case 3:
    scs_sin(sc2);
    scs_get_d_minf(&resd, sc2);
    return resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_cos \n", N);
    exit(1);
  }
  return resd;
}


double scs_cos_ru(double x){ 
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_cos(sc2);
    scs_get_d_pinf(&resd, sc2);
    return resd;
  case 1:
    scs_sin(sc2);
    scs_get_d_minf(&resd, sc2);
    return -resd;
  case 2:
    scs_cos(sc2);
    scs_get_d_minf(&resd, sc2);
    return -resd;
  case 3:
    scs_sin(sc2);
    scs_get_d_pinf(&resd, sc2);
    return resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_cos \n", N);
    exit(1);
  }
  return resd;
}



double scs_cos_rz(double x){ 
  scs_t sc1, sc2;
  double resd;
  int N;

#if EVAL_PERF
	crlibm_second_step_taken++;
#endif

  scs_set_d(sc1, x);
  N = rem_pio2_scs(sc2, sc1);
  N = N & 0x0000003;		/* extract the 2 last bits of  N */
  switch (N){
  case 0:
    scs_cos(sc2);
    scs_get_d_zero(&resd, sc2);
    return resd;
  case 1:
    scs_sin(sc2); 
    scs_get_d_zero(&resd, sc2);
    return -resd;
  case 2:
    scs_cos(sc2);
    scs_get_d_zero(&resd, sc2);
    return -resd;
  case 3:
    scs_sin(sc2);
    scs_get_d_zero(&resd, sc2);
    return resd;
  default:
    fprintf(stderr,"ERREUR: %d is not a valid value in s_cos \n", N);
    exit(1);
  }
  return resd;
}





/*************************************************************
 *                  Tangent                                  *
 *************************************************************/


/* The main function */

static void scs_tan(double x, scs_ptr res_scs){
  scs_t x_scs;
  scs_t x2;
  int i;
   scs_t y_scs;
  int N;

  scs_set_d(x_scs, x);
  

  N = rem_pio2_scs(y_scs, x_scs); 	/* x (=sc2) is in [-Pi/4,Pi/4] */ 
  N = N & 1;		/* extract the last bit of  N */
  scs_square(x2, y_scs);

  scs_mul(res_scs, tan_scs_poly_ptr[0], x2);
  
  for(i=1; i<(DEGREE_TAN_SCS-1)/2; i++){ /* The last coeff is not read from the file. */
    scs_add(res_scs, tan_scs_poly_ptr[i], res_scs);
    scs_mul(res_scs, res_scs, x2);
  }
  
  scs_mul(res_scs, res_scs, y_scs);
  scs_add(res_scs, y_scs, res_scs);
  
  if(N==1) {
    scs_inv(res_scs, res_scs);
    res_scs->sign = -res_scs->sign;
  }
}





double scs_tan_rn(double x){  
  scs_t res_scs;
  double resd;

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif

  scs_tan(x,res_scs);
  scs_get_d(&resd, res_scs);
  return resd;
}



double scs_tan_rd(double x){  
  scs_t res_scs;
  double resd;

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif

  scs_tan(x,res_scs);
  scs_get_d_minf(&resd, res_scs);
  return resd;
}



double scs_tan_ru(double x){  
  scs_t res_scs;
  double resd;

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif

  scs_tan(x,res_scs);
  scs_get_d_pinf(&resd, res_scs);
  return resd;
}



double scs_tan_rz(double x){  
  scs_t res_scs;
  double resd;

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif

  scs_tan(x,res_scs);
  scs_get_d_zero(&resd, res_scs);
  return resd;
}


