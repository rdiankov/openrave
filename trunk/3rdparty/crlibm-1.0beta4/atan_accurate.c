/*
 * Correctly rounded arctangent
 *
 * Author : Nicolas Gast (Ecole Normale Superieure), Florent de Dinechin
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
#include "crlibm_private.h"
#include "atan_accurate.h"
#include "atan_fast.h"


/*
 *  WHAT WE CAN DO :
 *
 * 1) Range reduction 
 *
 *	x > 0  because atan(-x) = - atan(x)
 *	
 *	we have built 50 intervals I(i), associated to a b(i) so that :
 *	
 *	For every x :
 *	
 *	we find the interval I(i) , as atan(x) = atan(b(i)) + atan( (x - b(i)) / (1 + x * b(i)) ) 
 *	
 *		so that X = (x - b(i)) / (1 + x * b(i))  be in interval [ -2^(-6) , 2^(-6) ] 
 *		There is no cancellation because :
 *		for every x in [ -2^(-6) , 2^(-6) ],
 *		
 *					     atan(x) <= 0.01562372862     in binary 0.000001111111111
 *		AND for the smallest b(i)    atan(b(i)) = 0.04687118592   in binary 0.00001011111111 	   
 *
 *
 * 2) Polynomial evaluation of atan(X), atan(b(i)) is tabulated.
 *                  
 *                                  (-???)  
 *   Approximation error: |err| < 2^ 
 *
 *
 * 3) Reconstruction:
 *
 *    atan(x) = atan(b(i)) + atan(X) 
 *
 *
 * 4) Rounding:
 *
 *    when |x| is too big, the result is always sign(x) * Pi/2,
 *    because Pi/2 is appromated by the biggest value smallest than Pi/2, 
 *    in order not to have an atan > Pi/2.
 */


 
 
static void scs_atan(scs_ptr res_scs, scs_ptr x){
  scs_t X_scs, denom1_scs, denom2_scs, poly_scs, X2;
  scs_t atanbhihi,atanbhilo, atanblo, atanbhi, atanb;
  scs_t bsc_ptr;
  db_number db;
  double test;
  int k, i=31;

  
  scs_get_d(&db.d, x);  
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  /* test if x as to be reduced */
  if (db.d > MIN_REDUCTION_NEEDED) {
    /* Compute i so that  x E [a[i],a[i+1]] */
    if (db.d < arctan_table[i][A].d) i-= 16;
    else i+=16;
    if (db.d < arctan_table[i][A].d) i-= 8;
    else i+= 8;
    if (db.d < arctan_table[i][A].d) i-= 4;
    else i+= 4;
    if (db.d < arctan_table[i][A].d) i-= 2;
    else i+= 2;
    if (db.d < arctan_table[i][A].d) i-= 1;
    else if (i<61) i+= 1;
    if (db.d < arctan_table[i][A].d) i-= 1;
    
    /* evaluate X = (x - b(i)) / (1 + x*b(i)) */
    scs_set_d(bsc_ptr, arctan_table[i][B].d);
    
    scs_mul(denom1_scs,bsc_ptr,x);
    scs_add(denom2_scs,denom1_scs,SCS_ONE);
    scs_sub(X_scs,x,bsc_ptr);
    scs_div(X_scs,X_scs,denom2_scs);
    
    scs_get_d(&test,X_scs);
    
    /* Polynomial evaluation of atan(X) , X = (x-b(i)) / (1+ x*b(i)) */
    scs_square(X2, X_scs);
    scs_set(res_scs, constant_poly_ptr[0]);
    for(k=1; k < 10; k++) {
      /* we use Horner expression */
      scs_mul(res_scs, res_scs, X2);
      scs_add(res_scs, constant_poly_ptr[k], res_scs);
    }
    scs_mul(poly_scs, res_scs, X_scs);
    
    /* reconstruction : */
    
    /* 1st we load atan ( b[i] ) in a scs*/ 
    scs_set_d( atanbhihi , arctan_table[i][ATAN_BHI].d);
    scs_set_d( atanbhilo , arctan_table[i][ATAN_BLO].d);
    scs_set_d( atanblo , atan_blolo[i].d);
    scs_add(atanbhi,atanbhihi,atanbhilo);
    scs_add(atanb,atanbhi,atanblo);
    scs_add(res_scs,atanb, poly_scs); 
    return;
  }
  
  else 
    { /* no reduction needed */
      /* Polynomial evaluation of atan(x) */
      scs_square(X2, x);
      scs_set(res_scs, constant_poly_ptr[0]);
      for(k=1; k < 10; k++) {
        /* we use Horner expression */
        scs_mul(res_scs, res_scs, X2);
        scs_add(res_scs, constant_poly_ptr[k], res_scs);
      }
      scs_mul(res_scs, res_scs, x);
      return;
    }
}



static void scs_atanpi(scs_ptr res, scs_ptr x){
  scs_t at;
  scs_atan(at, x);
  scs_mul(res, at, InvPiSCS_ptr);
}




double scs_atan_rn(double x){ 
  /* This function does NOT compute atan(x) correctly if it isn't 
   * called in atan_rn() 
   */
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign =1;
  
  res.d = x;

  if (x < 0){
    sign = -1;
    x *= -1;
  }
  scs_set_d(sc1, x);
  scs_atan(res_scs, sc1);
  scs_get_d(&res.d, res_scs);
  
  res.d *= sign;
  
  return res.d;
}








double scs_atan_rd(double x){ 
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign = 1;
   
  res.d = x;

  /* Filter cases */
  if (x < 0){
    sign = -1;
    x *= -1;
  }
  scs_set_d(sc1, x);
  scs_atan(res_scs, sc1);
  if (sign == -1){
    scs_get_d_pinf(&res.d, res_scs);
    res.d *= -1;
    return res.d;
  }
  else{
    scs_get_d_minf(&res.d, res_scs);		
    return res.d;
  }
}





double scs_atan_ru(double x){ 
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign = 1;
  
  res.d = x;

  /* Filter cases */
  if (x < 0){
    sign = -1;
    x *= -1;
  }
 
  scs_set_d(sc1, x);
  scs_atan(res_scs, sc1);
  if (sign == -1){
    scs_get_d_minf(&res.d, res_scs);
    res.d *= -1;
    return res.d;
  }
  else{
    scs_get_d_pinf(&res.d, res_scs);		
    return res.d;
  }
}





/************************************************************/
/********  AtanPi *******************************************/




double scs_atanpi_rn(double x){ 
  /* This function does NOT compute atanpi(x) correctly if it isn't 
   * called in atanpi_rn() 
   */
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign =1;
  
  res.d = x;

  if (x < 0){
    sign = -1;
    x *= -1;
  }
  scs_set_d(sc1, x);
  scs_atanpi(res_scs, sc1);
  scs_get_d(&res.d, res_scs);
  
  res.d *= sign;
  
  return res.d;
}


double scs_atanpi_rd(double x){ 
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign = 1;
   
  res.d = x;

  /* Filter cases */
  if (x < 0){
    sign = -1;
    x *= -1;
  }
  scs_set_d(sc1, x);
  scs_atanpi(res_scs, sc1);
  if (sign == -1){
    scs_get_d_pinf(&res.d, res_scs);
    res.d *= -1;
    return res.d;
  }
  else{
    scs_get_d_minf(&res.d, res_scs);		
    return res.d;
  }
}

/*************************************************************
 *************************************************************
 *               ROUNDED  TOWARD  +INFINITY
 *************************************************************
 *************************************************************/

double scs_atanpi_ru(double x){ 
  scs_t sc1;
  scs_t res_scs;
  db_number res;
  int sign = 1;
  
  res.d = x;

  /* Filter cases */
  if (x < 0){
    sign = -1;
    x *= -1;
  }
 
  scs_set_d(sc1, x);
  scs_atanpi(res_scs, sc1);
  if (sign == -1){
    scs_get_d_minf(&res.d, res_scs);
    res.d *= -1;
    return res.d;
  }
  else{
    scs_get_d_pinf(&res.d, res_scs);		
    return res.d;
  }
}

