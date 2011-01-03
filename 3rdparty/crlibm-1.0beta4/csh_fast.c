/*
 * Correctly rounded hyperbolic sine and cosine
 *
 * Author : Matthieu Gallet, Florent de Dinechin
 * 
 *  This file is part of the crlibm library, developed by the Arenaire
 * project at Ecole Normale Superieure de Lyon
 *
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
#include "crlibm.h"
#include "crlibm_private.h"
#include "csh_fast.h"
#include "triple-double.h"

void exp13(int *exponent, double *exp_h, double *exp_m, double *exp_l, double x);
void expm1_13(double *exp_h, double *exp_m, double *exp_l, double x);

/* switches on various printfs. Default 0 */
#define DEBUG 0
static const double largest_double = 0x1.fffffffffffffp1023;
static const double tiniest_double = 0x1.0p-1074;

enum{RN,RD,RU,RZ};
 
static void do_cosh(double x, double* preshi, double* preslo){
  int k;
  db_number y;
  double ch_hi, ch_lo, sh_hi, sh_lo;/* cosh(x) = (ch_hi + ch_lo)*(cosh(k*ln(2)) + (sh_hi + sh_lo)*(sinh(k*ln(2))) */
  db_number  table_index_float;
  int table_index;
  double temp_hi, temp_lo, temp;/* some temporary variables */
  double b_hi, b_lo,b_ca_hi, b_ca_lo, b_sa_hi, b_sa_lo;
  double ca_hi, ca_lo, sa_hi, sa_lo; /*will be the tabulated values */
  double tcb_hi, tsb_hi; /*results of polynomial approximations*/
  double square_b_hi;
  double ch_2_pk_hi, ch_2_pk_lo, ch_2_mk_hi, ch_2_mk_lo;
  double sh_2_pk_hi, sh_2_pk_lo, sh_2_mk_hi, sh_2_mk_lo;
  db_number two_p_plus_k, two_p_minus_k; /* 2^(k-1) + 2^(-k-1) */

  /* First range reduction*/
  DOUBLE2INT(k, x * inv_ln_2.d)
    if (k != 0){ /* b_hi+b_lo =  x - (ln2_hi + ln2_lo) * k */
      temp_hi = x - ln2_hi.d * k;                                         
      temp_lo = -ln2_lo.d * k;                                          
      Add12Cond(b_hi, b_lo, temp_hi, temp_lo); 
    }
    else {                                                         
      b_hi = x;  b_lo = 0.;
    }                                                               
  /*we'll construct 2 constants for the last reconstruction */
  two_p_plus_k.i[LO] = 0;
  two_p_plus_k.i[HI] = (k-1+1023) << 20;
  two_p_minus_k.i[LO] = 0;
  two_p_minus_k.i[HI] = (-k-1+1023) << 20;

  /* at this stage, we've done the first range reduction : we have b_hi + b_lo  between -ln(2)/2 and ln(2)/2 */
  /* now we can do the second range reduction */
  /* we'll get the 8 leading bits of b_hi */
  table_index_float.d = b_hi + two_43_44.d;
  /*this add do the float equivalent of a rotation to the right, since -0.5 <= b_hi <= 0.5*/
  table_index = table_index_float.i[LO];/* -89 <= table_index <= 89 */
  table_index_float.d -= two_43_44.d;
  table_index += bias; /* to have only positive values */
  b_hi -= table_index_float.d;/* to remove the 8 leading bits*/
  /* since b_hi was between -2^-1 and 2^1, we now have b_hi between -2^-9 and 2^-9 */

  
  y.d = b_hi;
  /*   first, y�  */
  square_b_hi = b_hi * b_hi;
  /* effective computation of the polynomial approximation */
  
  if (((y.i[HI])&(0x7FFFFFFF)) < (two_minus_30.i[HI])) {
    tcb_hi = 0;
    tsb_hi = 0;
  }
  else {
    /*   second, cosh(y) = y� * (1/2 + y� * (1/24 + y� * 1/720)) */
    tcb_hi = (square_b_hi)* (c2.d + square_b_hi * (c4.d + square_b_hi * c6.d));
    tsb_hi = square_b_hi * (s3.d + square_b_hi * (s5.d + square_b_hi * s7.d));
  }
 

  if( table_index != bias) {
    /* we get the tabulated the tabulated values */
    ca_hi = cosh_sinh_table[table_index][0].d;
    ca_lo = cosh_sinh_table[table_index][1].d;
    sa_hi = cosh_sinh_table[table_index][2].d;
    sa_lo = cosh_sinh_table[table_index][3].d;
    
    /* first reconstruction of the cosh (corresponding to the second range reduction) */
    Mul12(&b_sa_hi,&b_sa_lo, sa_hi, b_hi);
    temp =  ((((((ca_lo + (b_hi * sa_lo)) + b_lo * sa_hi) + b_sa_lo) + (b_sa_hi * tsb_hi)) + ca_hi * tcb_hi) + b_sa_hi);
    Add12Cond(ch_hi, ch_lo, ca_hi, temp);
      /* first reconstruction for the sinh (corresponding to the second range reduction) */
  }
  else {
    Add12Cond(ch_hi, ch_lo, (double) 1, tcb_hi);
  }

  
  if(k != 0) {
    if( table_index != bias) {
      /* first reconstruction for the sinh (corresponding to the second range reduction) */
      Mul12(&b_ca_hi , &b_ca_lo, ca_hi, b_hi);
      temp = (((((sa_lo + (b_lo * ca_hi)) + (b_hi * ca_lo)) + b_ca_lo) + (sa_hi*tcb_hi)) + (b_ca_hi * tsb_hi));
      Add12(temp_hi, temp_lo, b_ca_hi, temp);
      Add22Cond(&sh_hi, &sh_lo, sa_hi, (double) 0, temp_hi, temp_lo);
    }
    else {
      Add12Cond(sh_hi, sh_lo, b_hi, tsb_hi * b_hi + b_lo);
    }
    if((k < 35) && (k > -35) )
      {
	ch_2_pk_hi = ch_hi * two_p_plus_k.d;
	ch_2_pk_lo = ch_lo * two_p_plus_k.d;
	ch_2_mk_hi = ch_hi * two_p_minus_k.d;
	ch_2_mk_lo = ch_lo * two_p_minus_k.d;
	sh_2_pk_hi = sh_hi * two_p_plus_k.d;
	sh_2_pk_lo = sh_lo * two_p_plus_k.d;
	sh_2_mk_hi = - sh_hi * two_p_minus_k.d;
	sh_2_mk_lo = - sh_lo * two_p_minus_k.d;
	
	Add22Cond(preshi, preslo, ch_2_mk_hi, ch_2_mk_lo, sh_2_mk_hi, sh_2_mk_lo);
	Add22Cond(&ch_2_mk_hi, &ch_2_mk_lo , sh_2_pk_hi, sh_2_pk_lo, *preshi, *preslo);
	Add22Cond(preshi, preslo, ch_2_pk_hi, ch_2_pk_lo, ch_2_mk_hi, ch_2_mk_lo);
      } 
    else if (k >= 35) 
      {
	ch_2_pk_hi = ch_hi * two_p_plus_k.d;
	ch_2_pk_lo = ch_lo * two_p_plus_k.d;
	sh_2_pk_hi = sh_hi * two_p_plus_k.d;
	sh_2_pk_lo = sh_lo * two_p_plus_k.d;
	Add22Cond(preshi, preslo, ch_2_pk_hi, ch_2_pk_lo, sh_2_pk_hi, sh_2_pk_lo);
      }
    else /* if (k <= -35) */ 
      {
	ch_2_mk_hi = ch_hi * two_p_minus_k.d;
	ch_2_mk_lo = ch_lo * two_p_minus_k.d;
	sh_2_mk_hi = - sh_hi * two_p_minus_k.d;
	sh_2_mk_lo = - sh_lo * two_p_minus_k.d;
	Add22Cond(preshi, preslo, ch_2_mk_hi, ch_2_mk_lo, sh_2_mk_hi, sh_2_mk_lo);
      }
  }
  else {
    *preshi = ch_hi;
    *preslo = ch_lo;
  }

  return;
}



static void do_cosh_accurate(int* pexponent, 
			     double* presh, double* presm, double* presl, 
			     double x){
  double exph, expm, expl;
  double expph, exppm, exppl;
  int exponentm, deltaexponent;
  db_number  expmh, expmm, expml;

#if EVAL_PERF==1
  crlibm_second_step_taken++;
#endif

  if(x<0)
    x=-x;
  if (x > 40.0) {  /* then exp(-x) < 2^-118 exp(x) */
    exp13(pexponent, presh, presm, presl, x);
  }
  else { 
    exp13(pexponent, &expph, &exppm, &exppl, x);
    exp13(&exponentm, &(expmh.d), &(expmm.d), &(expml.d), -x);
    /* align the mantissas. 
       The exponent is increased but stays well below overflow threshold */
    deltaexponent =  exponentm - *pexponent ; 
    expmh.i[HI] += (deltaexponent) << 20;  
    expmm.i[HI] += (deltaexponent) << 20;  
    expml.i[HI] += (deltaexponent) << 20;  
    Add33(&exph, &expm, &expl, expph, exppm, exppl,   expmh.d, expmm.d, expml.d);
    Renormalize3(presh,presm,presl, exph, expm, expl);
  }
}



double cosh_rn(double x){ 
  db_number y;
  int hx;
  double rh, rl;
    
  y.d = x;
  hx = y.i[HI] & 0x7FFFFFFF; 

  /* Filter special cases */
  if (hx > max_input_csh.i[HI]) { /* strictly greater, implies x > max_input_csh */
    if (hx >= 0x7ff00000){  /* Infty or NaN */ 
      if (((hx&0x000fffff)|y.i[LO])!=0)
	return x+x;                                        /* Nan */ 
      else {/* otherwise the result should be +infty */
	y.i[HI] = 0x7FF00000; 
	return (y.d);
      }
    }
  }
  if (x >= max_input_csh.d || x <= -max_input_csh.d) 
    return largest_double * largest_double;     /* overflow  */ 
  if (hx<0x3e500000) {
    if(x==0) 
      return 1.0; /* exact */
    else 
      return (1.0+tiniest_double); /* to raise inexact flag */
  }

  do_cosh(x, &rh, &rl);

  
  if (rh == (rh + (rl * round_cst_csh))) return rh;
  else{
    int exponent;
    db_number res;
    double  resh, resm, resl;

    do_cosh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundToNearest3(&(res.d), resh, resm, resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  
}








double cosh_ru(double x){ 
  db_number y;
  int hx;
  double rh, rl;

  y.d = x;
  hx = y.i[HI] & 0x7FFFFFFF; 

  if (hx > max_input_csh.i[HI]) {
    /* if NaN, return it */
    if (((hx&0x7FF00000) == 0x7FF00000) && (((y.i[HI] & 0x000FFFFF)!=0) || (y.i[LO]!=0)) )
      return x;
    else {/* otherwise the result should be +infty */
      y.i[LO] = 0; y.i[HI] = 0x7FF00000; return (y.d);
    }
  }
  
  if (x >= max_input_csh.d || x <= -max_input_csh.d) 
    return largest_double * largest_double;     /* overflow  */ 

  if (hx<0x3e500000) { /* return the successor of 1 */
    if(x==0.) return 1.0;
    else{
      y.l = 0x3ff0000000000001LL;
      return y.d;
    }
  }

  do_cosh(x, &rh, &rl);

  TEST_AND_RETURN_RU(rh, rl, maxepsilon_csh);

  /* if the previous block didn't return a value, launch accurate phase */
  {
    int exponent;
    db_number res;
    double resh, resm, resl;

    do_cosh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundUpwards3(&(res.d), resh,resm,resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  
}



double cosh_rd(double x){ 
  db_number y;
  int hx;
  double rh, rl;

  y.d = x;
  hx = y.i[HI] & 0x7FFFFFFF; 


  if (hx > max_input_csh.i[HI]) {
    if (hx >= 0x7FF00000) {    /*particular cases : QNaN, SNaN, +- oo*/
      if (((hx&0x7FF00000) == 0x7FF00000) && (((y.i[HI] & 0x000FFFFF)!=0) || (y.i[LO]!=0)) )
	return x; /* NaN */
      else { /* infinity */ 
	y.i[HI] = hx;
	return (y.d);
      }
    }
  }

  if (y.d >= max_input_csh.d || y.d  <= - max_input_csh.d) { /* out of range */
    y.i[LO] = 0xFFFFFFFF; y.i[HI] = 0x7FEFFFFF ; return (y.d);
  }
    
  if (hx<0x3e500000)
    return (1.0); 

  do_cosh(x, &rh, &rl);

  TEST_AND_RETURN_RD(rh, rl, maxepsilon_csh);

  /* if the previous block didn't return a value, launch accurate phase */
  {
    int exponent;
    db_number res;
    double resh, resm, resl;

    do_cosh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundDownwards3(&(res.d), resh,resm,resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  
}



double cosh_rz(double x){ 
  return(cosh_rd(x));/* cosh is always positive, so rounding to -infinite is equivalent to rounding to zero */
}










static void do_sinh(double x, double* prh, double* prl){ 

  int k;
  db_number y;
  double temp1;
  double ch_hi, ch_lo, sh_hi, sh_lo;/* cosh(x) = (sh_hi + sh_lo)*(cosh(k*ln(2)) + (ch_hi + ch_lo)*(sinh(k*ln(2))) */
  db_number  table_index_float;
  int table_index;
  double ch_2_pk_hi, ch_2_pk_lo, ch_2_mk_hi, ch_2_mk_lo;
  double sh_2_pk_hi, sh_2_pk_lo, sh_2_mk_hi, sh_2_mk_lo;
  double b_hi, b_lo;
  double ca_b_hi, ca_b_lo, temp_hi, temp_lo, sa_b_hi, sa_b_lo;
  double ca_hi, ca_lo, sa_hi, sa_lo; /*tabulated values */
  double tcb_hi,  tsb_hi; /*results of polynomial approximations*/
  db_number two_p_plus_k, two_p_minus_k; /* 2^(k-1) + 2^(-k-1) */
  double square_y_hi;
  
  /* Now we can do the first range reduction*/
  DOUBLE2INT(k, x * inv_ln_2.d)
    if (k != 0){ /* b_hi + b_lo =  x - (ln2_hi + ln2_lo) * k */
      temp_hi = x - ln2_hi.d * k;                                         
      temp_lo = -ln2_lo.d * k;                                          
      Add12Cond(b_hi, b_lo, temp_hi, temp_lo); 
    }
    else {                                                         
      b_hi = x;  b_lo = 0.;
    }                                                               

  /*we'll construct 2 constants for the last reconstruction */
  two_p_plus_k.i[LO] = 0;
  two_p_plus_k.i[HI] = (k-1+1023) << 20;
  two_p_minus_k.i[LO] = 0;
  two_p_minus_k.i[HI] = (-k-1+1023) << 20;

  /* at this stage, we've done the first range reduction : we have b_hi + b_lo  between -ln(2)/2 and ln(2)/2 */
  /* now we can do the second range reduction */
  /* we'll get the 8 leading bits of r_hi */
  
  table_index_float.d = b_hi + two_43_44.d;
  /*this add do the float equivalent of a rotation to the right, since -0.5 <= b_hi <= 0.5*/
  table_index = table_index_float.i[LO];/* -89 <= table_index <= 89 */
  table_index_float.d -= two_43_44.d;
  table_index += bias; /* to have only positive values */
  b_hi -= table_index_float.d;/* to remove the 8 leading bits*/
  /* since b_hi was between -2^-1 and 2^1, we now have b_hi between -2^-9 and 2^-9 */
  
  y.d = b_hi;
  /*   first, y� = square_y_hi + square_y_lo  */
  square_y_hi = b_hi * b_hi;
  /* effective computation of the polyomial approximation */
  if (((y.i[HI])&(0x7FFFFFFF)) <= (two_minus_30.i[HI])) {
    tsb_hi = 0;
    tcb_hi = 0;
  }
  else {
    tsb_hi = square_y_hi * (s3.d + square_y_hi * (s5.d + square_y_hi * s7.d));
    /*   second, cosh(y) = y� * (1/2 + y� * (1/24 + y� * 1/720)) */
    tcb_hi = (square_y_hi)* (c2.d + square_y_hi * (c4.d + square_y_hi * c6.d));
  }
  
  if( table_index != bias) {
    /* we get the tabulated the tabulated values*/
    ca_hi = cosh_sinh_table[table_index][0].d;
    ca_lo = cosh_sinh_table[table_index][1].d;
    sa_hi = cosh_sinh_table[table_index][2].d;
    sa_lo = cosh_sinh_table[table_index][3].d;

    /* first reconstruction for the sinh (corresponding to the second range reduction) */
    temp1 = sa_lo;
    temp1 += b_lo * ca_hi;
    temp1 += b_hi * ca_lo;
    Mul12(&ca_b_hi, &ca_b_lo, ca_hi, b_hi);
    temp1 += ca_b_lo;
    temp1 += sa_hi * tcb_hi;
    temp1 += ca_b_hi * tsb_hi;
    Add12Cond(temp_hi, temp_lo, ca_b_hi, temp1);
    Add22Cond(&sh_hi, &sh_lo, sa_hi, (double) 0, temp_hi, temp_lo);
    /* first reconstruction of the cosh (corresponding to the second range reduction) */
    temp1 = ca_lo;
    Mul12(&sa_b_hi,&sa_b_lo, sa_hi, b_hi);
    temp1 += b_hi * sa_lo;
    temp1 += b_lo * sa_hi;
    temp1 += sa_b_lo;
    temp1 += sa_b_hi * tsb_hi;
    temp1 += ca_hi * tcb_hi;
    temp1 += sa_b_hi;
    Add12Cond(ch_hi, ch_lo, ca_hi, temp1);
  }
  else {
    Add12Cond(sh_hi, sh_lo, b_hi, tsb_hi * b_hi + b_lo);
    Add12Cond(ch_hi, ch_lo, (double) 1, tcb_hi);
  }
    
  if(k != 0) {
    if( (k < 35) && (k > -35) ) {
	ch_2_pk_hi = ch_hi * two_p_plus_k.d;
	ch_2_pk_lo = ch_lo * two_p_plus_k.d;
	ch_2_mk_hi = - ch_hi * two_p_minus_k.d;
	ch_2_mk_lo = - ch_lo * two_p_minus_k.d;
	sh_2_pk_hi = sh_hi * two_p_plus_k.d;
	sh_2_pk_lo = sh_lo * two_p_plus_k.d;
	sh_2_mk_hi = sh_hi * two_p_minus_k.d;
	sh_2_mk_lo = sh_lo * two_p_minus_k.d;

	Add22Cond(prh, prl, ch_2_mk_hi, ch_2_mk_lo, sh_2_mk_hi, sh_2_mk_lo);
	Add22Cond(&ch_2_mk_hi, &ch_2_mk_lo , sh_2_pk_hi, sh_2_pk_lo, *prh, *prl);
	Add22Cond(prh, prl, ch_2_pk_hi, ch_2_pk_lo, ch_2_mk_hi, ch_2_mk_lo);
    }
    else if (k >= 35) 
      {
	ch_2_pk_hi = ch_hi * two_p_plus_k.d;
	ch_2_pk_lo = ch_lo * two_p_plus_k.d;
	sh_2_pk_hi = sh_hi * two_p_plus_k.d;
	sh_2_pk_lo = sh_lo * two_p_plus_k.d;
	Add22Cond(prh, prl, ch_2_pk_hi, ch_2_pk_lo, sh_2_pk_hi, sh_2_pk_lo);
      }
    else 
      {
	ch_2_mk_hi = - ch_hi * two_p_minus_k.d;
	ch_2_mk_lo = - ch_lo * two_p_minus_k.d;
	sh_2_mk_hi = sh_hi * two_p_minus_k.d;
	sh_2_mk_lo = sh_lo * two_p_minus_k.d;
	Add22Cond(prh, prl, ch_2_mk_hi, ch_2_mk_lo, sh_2_mk_hi, sh_2_mk_lo);
      }
  }
  else {
    *prh = sh_hi;
    *prl = sh_lo;
  }
}





static void do_sinh_accurate(int* pexponent, 
			     double* presh, double* presm, double* presl, 
			     double x){
  double exph, expm, expl;
  double expph, exppm, exppl, expmh, expmm, expml;

#if EVAL_PERF==1
  crlibm_second_step_taken++;
#endif

  if(x > 40.0) { /* then exp(-x) < 2^-129 exp(x) */ 
    exp13(pexponent, presh, presm, presl, x);
    return;
  }
  if(x < -40.0) { /* then exp(x) < 2^-129 exp(-x) */ 
    exp13(pexponent, presh, presm, presl, -x);
    *presh = -*presh;
    *presm = -*presm;
    *presl = -*presl;
    return;
  }
  /* Otherwise we are between -40 and 40, and we also know that |x| > 2^-25 */
  if(x>0.0) {
    expm1_13(&expph, &exppm, &exppl, x);
    expm1_13(&expmh, &expmm, &expml, -x);
    /* The following is OK because expph and  -expmh have the same sign */
    Add33(&exph, &expm, &expl, expph, exppm, exppl,   -expmh, -expmm, -expml);
    Renormalize3(presh,presm,presl, exph, expm, expl);
    *pexponent=0;
    return;
  }
  else  { /* x<0 */
    expm1_13(&expph, &exppm, &exppl, x);
    expm1_13(&expmh, &expmm, &expml, -x);
    /* The following is OK because expph and  -expmh have the same sign */
    Add33(&exph, &expm, &expl,   -expmh, -expmm, -expml, expph, exppm, exppl);
    Renormalize3(presh,presm,presl, exph, expm, expl);
    *pexponent=0;
    return;
  }
}




double sinh_rn(double x){ 
  db_number y;
  int hx;
  double rh, rl;
    

  y.d = x;
  hx = y.i[HI] & 0x7FFFFFFF; 

  /* Filter special cases */
  if (hx > max_input_csh.i[HI]) { /* strictly greater, implies x > max_input_csh */
    if (hx >= 0x7ff00000){ /* infinity or NaN */
      if (((hx&0x000fffff)|y.i[LO])!=0)
	return x+x;                                        /* NaN */ 
      else {/* otherwise the result should be +infty */
	return (y.d);
      }
    }
    if (x > max_input_csh.d) 
      return largest_double * largest_double;     /* overflow  */ 
    if (x < -max_input_csh.d) 
      return -largest_double * largest_double;     /* overflow  */ 
  }

  if (hx<0x3e500000) {
      return x; /* exact, we should find some way of raising the inexact flag */
  }

  
  do_sinh(x, &rh, &rl);

  if (rh == (rh + (rl * round_cst_csh))) return rh;
  else{
    int exponent;
    db_number res;
    double  resh, resm, resl;

    do_sinh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundToNearest3(&(res.d), resh, resm, resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  

}



double sinh_ru(double x){ 
  db_number y;
  double rh, rl;


  y.d = x;
  y.i[HI] = y.i[HI] & 0x7FFFFFFF;     /* to get the absolute value of the input */
  if ((y.i[HI] & 0x7FF00000) >= (0x7FF00000)) {    /*particular cases : QNaN, SNaN, +- oo*/
   return (x);
  }
  if (y.d > max_input_csh.d) { /* out of range */
    if(x>0) {
      y.i[LO] = 0; y.i[HI] = 0x7FF00000; return (y.d);
    }
    else {
      y.i[LO] = 0xFFFFFFFF; y.i[HI] = 0xFFEFFFFF ; return (y.d);
    }
  }

  if(y.i[HI] < 0x3e500000) /* 2^(-26) */
    { /* Add one ulp if x positive */
      if(x>0) { 
	y.l++;
	return y.d;
      }
      else
	return x;
    }

  do_sinh(x, &rh, &rl);

  TEST_AND_RETURN_RU(rh, rl, maxepsilon_csh);

  /* if the previous block didn't return a value, launch accurate phase */
  {
    int exponent;
    db_number res;
    double resh, resm, resl;

    do_sinh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundUpwards3(&(res.d), resh,resm,resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  
}


double sinh_rd(double x){ 
  db_number y;
  double rh, rl;

  y.d = x;
  y.i[HI] = y.i[HI] & 0x7FFFFFFF;     /* to get the absolute value of the input */
  if ((y.i[HI] & 0x7FF00000) >= (0x7FF00000)) {    /*particular cases : QNaN, SNaN, +- oo*/
    y.d = x;
   return (y.d);
  }
  if (y.d > max_input_csh.d) { /* out of range */
    if(x>0) {
      y.i[LO] = 0xFFFFFFFF; y.i[HI] = 0x7FEFFFFF ; return (y.d);
    }
    else {
      y.i[LO] = 0; y.i[HI] = 0xFFF00000; return (y.d);
    }
  }
  if(y.i[HI] < 0x3e500000) /* 2^(-26) */
    { /* Add one ulp and restore the sign if x negative */
      if(x<0){
	y.l = (y.l+1); 
	return -y.d;
      } 
      else 
	return x;      
    }
  do_sinh(x, &rh, &rl);
  
  TEST_AND_RETURN_RD(rh, rl, maxepsilon_csh);

  /* if the previous block didn't return a value, launch accurate phase */
  {
    int exponent;
    db_number res;
    double resh, resm, resl;

    do_sinh_accurate(&exponent, &resh,&resm, &resl, x);
    RoundDownwards3(&(res.d), resh,resm,resl);

    /* Now we have to set the exponent of res as exponent -1 (division
       by 2). However, as res may sometimes end up infinite, we first
       set the exponent to exponent -11 and then multiply by 2^10,
       which will cater for overflow  */  
    res.i[HI] += (exponent-11) << 20;  
    return 1024. * res.d;
  }  
}




double sinh_rz(double x){ 
  if( x > 0) {
    return(sinh_rd(x));
  }
  else {
    return(sinh_ru(x));
  }
}

