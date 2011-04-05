 
#include <stdio.h>
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "pow.h"


/* Some macros for specific operations in power */

#define USE_BUILTINS 0

/* decompose

   Decomposes a positive double precision 
   variable x (normal or subnormal) such that

   2^(resE) * resm = x 

   where resm = 2 * k + 1 for integer k

   resE is an integer variable pointer
   resm is a double precision variable pointer
   x is a double precision variable.

*/

#if USE_BUILTINS
#define decompose(resm,resE,x)                                             \
{                                                                          \
   int __decompose_ex, __decompose_d;                                      \
   int64_t __decompose_temp;                                               \
   db_number __decompose_xdb;                                              \
                                                                           \
   __decompose_xdb.d = (x);                                                \
   __decompose_ex = 0;                                                     \
   if (!(__decompose_xdb.i[HI] & 0xfff00000)) {                            \
     __decompose_xdb.d = (x) * 0.18014398509481984e17;                     \
     __decompose_ex = -54;                                                 \
   }                                                                       \
   __decompose_ex += (__decompose_xdb.i[HI] >> 20) - 1023;                 \
   __decompose_xdb.i[HI] &= 0x000fffff;                                    \
   __decompose_temp = __decompose_xdb.l | 0x0010000000000000llu;           \
   __decompose_d = __builtin_ctz(__decompose_temp);                        \
   __decompose_xdb.i[HI] |= (1075 - __decompose_d) << 20;                  \
   *(resE) = __decompose_d - 52 + __decompose_ex;                          \
   *(resm) = __decompose_xdb.d;                                            \
}
#else
#define decompose(resm,resE,x)                                             \
{                                                                          \
   int __decompose_ex, __decompose_d;                                      \
   int64_t __decompose_temp;                                               \
   db_number __decompose_xdb, __decompose_tempdb;                          \
                                                                           \
   __decompose_xdb.d = (x);                                                \
   __decompose_ex = 0;                                                     \
   if (!(__decompose_xdb.i[HI] & 0xfff00000)) {                            \
     __decompose_xdb.d = (x) * 0.18014398509481984e17;                     \
     __decompose_ex = -54;                                                 \
   }                                                                       \
   __decompose_ex += (__decompose_xdb.i[HI] >> 20) - 1023;                 \
   __decompose_xdb.i[HI] &= 0x000fffff;                                    \
   __decompose_temp = __decompose_xdb.l | 0x0010000000000000llu;           \
   __decompose_temp = (~__decompose_temp & (__decompose_temp - 1)) + 1;    \
   __decompose_tempdb.d = (double) __decompose_temp;                       \
   __decompose_d = (__decompose_tempdb.i[HI] >> 20) - 1023;                \
   __decompose_xdb.i[HI] |= (1075 - __decompose_d) << 20;                  \
   *(resE) = __decompose_d - 52 + __decompose_ex;                          \
   *(resm) = __decompose_xdb.d;                                            \
}
#endif


/* isOddInteger 

   Determines if a given double precision number x is an odd integer 

*/

#define isOddInteger(x) (ABS(((ABS(x) + 0.9007199254740992e16) - 0.9007199254740992e16) - ABS(x)) == 1.0)

/* isInteger 

   Determines if a given double precision number x is integer 

*/

#define isInteger(x) ((ABS(x) >= 0.4503599627370496e16) || (((ABS(x) + 0.4503599627370496e16) - 0.4503599627370496e16) == ABS(x)))


/* log2_130

   Approximates 

   resh + resm + resl = (ed + log2(1 + (xh + xm)) + log2(r[index])) * (1 + eps)

   where ||eps|| <= 2^(-130)


*/
static inline void log2_130(double *resh, double *resm, double *resl, 
	                    int index, double ed, double xh, double xm) {

  double p_t_1_0h;
  double p_t_2_0h;
  double p_t_3_0h;
  double p_t_4_0h;
  double p_t_5_0h;
  double p_t_6_0h;
  double p_t_7_0h;
  double p_t_8_0h;
  double p_t_9_0h, p_t_9_0m;
  double p_t_10_0h, p_t_10_0m;
  double p_t_11_0h, p_t_11_0m;
  double p_t_12_0h, p_t_12_0m;
  double p_t_13_0h, p_t_13_0m;
  double p_t_14_0h, p_t_14_0m;
  double p_t_15_0h, p_t_15_0m;
  double p_t_16_0h, p_t_16_0m, p_t_16_0l;
  double p_t_17_0h, p_t_17_0m, p_t_17_0l;
  double p_t_18_0h, p_t_18_0m, p_t_18_0l;
  double p_t_19_0h, p_t_19_0m, p_t_19_0l;
  double p_t_20_0h, p_t_20_0m, p_t_20_0l;
  double p_t_21_0h, p_t_21_0m, p_t_21_0l;
  double p_t_21_1h, p_t_21_1m, p_t_21_1l;
  double p_t_22_0h, p_t_22_0m, p_t_22_0l;
  double p_t_23_0h, p_t_23_0m, p_t_23_0l;
  double p_resh, p_resm, p_resl;
  double log2yh, log2ym, log2yl;
  double log2xh, log2xm, log2xl;
  double logih, logim, logil;


  p_t_1_0h = log2_130_p_coeff_13h;
  p_t_2_0h = p_t_1_0h * xh;
  p_t_3_0h = log2_130_p_coeff_12h + p_t_2_0h;
  p_t_4_0h = p_t_3_0h * xh;
  p_t_5_0h = log2_130_p_coeff_11h + p_t_4_0h;
  p_t_6_0h = p_t_5_0h * xh;
  p_t_7_0h = log2_130_p_coeff_10h + p_t_6_0h;
  p_t_8_0h = p_t_7_0h * xh;
  Add12(p_t_9_0h,p_t_9_0m,log2_130_p_coeff_9h,p_t_8_0h);
  Mul22(&p_t_10_0h,&p_t_10_0m,p_t_9_0h,p_t_9_0m,xh,xm);
  Add122(&p_t_11_0h,&p_t_11_0m,log2_130_p_coeff_8h,p_t_10_0h,p_t_10_0m);
  MulAdd22(&p_t_12_0h,&p_t_12_0m,log2_130_p_coeff_7h,log2_130_p_coeff_7m,xh,xm,p_t_11_0h,p_t_11_0m);
  MulAdd22(&p_t_13_0h,&p_t_13_0m,log2_130_p_coeff_6h,log2_130_p_coeff_6m,xh,xm,p_t_12_0h,p_t_12_0m);
  MulAdd22(&p_t_14_0h,&p_t_14_0m,log2_130_p_coeff_5h,log2_130_p_coeff_5m,xh,xm,p_t_13_0h,p_t_13_0m);
  Mul22(&p_t_15_0h,&p_t_15_0m,p_t_14_0h,p_t_14_0m,xh,xm);
  Add23(&p_t_16_0h,&p_t_16_0m,&p_t_16_0l,log2_130_p_coeff_4h,log2_130_p_coeff_4m,p_t_15_0h,p_t_15_0m);
  Mul233(&p_t_17_0h,&p_t_17_0m,&p_t_17_0l,xh,xm,p_t_16_0h,p_t_16_0m,p_t_16_0l);
  Add233(&p_t_18_0h,&p_t_18_0m,&p_t_18_0l,log2_130_p_coeff_3h,log2_130_p_coeff_3m,p_t_17_0h,p_t_17_0m,p_t_17_0l);
  Mul233(&p_t_19_0h,&p_t_19_0m,&p_t_19_0l,xh,xm,p_t_18_0h,p_t_18_0m,p_t_18_0l);
  Add33(&p_t_20_0h,&p_t_20_0m,&p_t_20_0l,log2_130_p_coeff_2h,log2_130_p_coeff_2m,log2_130_p_coeff_2l,p_t_19_0h,p_t_19_0m,p_t_19_0l);
  Mul233(&p_t_21_0h,&p_t_21_0m,&p_t_21_0l,xh,xm,p_t_20_0h,p_t_20_0m,p_t_20_0l);
  Renormalize3(&p_t_21_1h,&p_t_21_1m,&p_t_21_1l,p_t_21_0h,p_t_21_0m,p_t_21_0l);
  Add33(&p_t_22_0h,&p_t_22_0m,&p_t_22_0l,log2_130_p_coeff_1h,log2_130_p_coeff_1m,log2_130_p_coeff_1l,p_t_21_1h,p_t_21_1m,p_t_21_1l);
  Mul233(&p_t_23_0h,&p_t_23_0m,&p_t_23_0l,xh,xm,p_t_22_0h,p_t_22_0m,p_t_22_0l);
  p_resh = p_t_23_0h;
  p_resm = p_t_23_0m;
  p_resl = p_t_23_0l;

  logih = argredtable[index].logih;
  logim = argredtable[index].logim;
  logil = argredtable[index].logil;

  Add33(&log2yh,&log2ym,&log2yl,logih,logim,logil,p_resh,p_resm,p_resl);
  Add133(&log2xh,&log2xm,&log2xl,ed,log2yh,log2ym,log2yl);
  
  Renormalize3(resh,resm,resl,log2xh,log2xm,log2xl);

}

/* exp2_120

   Approximates 

   2^H * (resh + resm + resl) = 2^(xh + xm + xl) * (1 + eps)

   where ||eps|| <= 2^(-119.5)

*/
static inline void exp2_120(int *H, double *resh, double *resm, double *resl, 
			    double xh, double xm, double xl) {
  double xhMult2L, rhMult2L, r;
  int k, index1, index2;
  db_number shiftedxhMult2Ldb;
  double rh, rm, rl;
  double t1h, t1m, t1l, t2, t3;
  double p_t_1_0h;
  double p_t_2_0h;
  double p_t_3_0h;
  double p_t_4_0h;
  double p_t_5_0h, p_t_5_0m;
  double p_t_6_0h, p_t_6_0m;
  double p_t_7_0h, p_t_7_0m;
  double p_t_8_0h, p_t_8_0m;
  double p_t_9_0h, p_t_9_0m, p_t_9_0l;
  double p_t_10_0h, p_t_10_0m, p_t_10_0l;
  double p_t_11_0h, p_t_11_0m, p_t_11_0l;
  double p_resh, p_resm, p_resl;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;
  double tablesh, tablesm, tablesl;
  double exp2h, exp2m, exp2l;

  /* Argument reduction 

     Produce exactly

     2^H * 2^(i1/2^8) * 2^(i2/2^13) * 2^(rh + rm + rl)

  */

  xhMult2L = xh * two13;
  shiftedxhMult2Ldb.d = shiftConst + xhMult2L;
  rhMult2L = xhMult2L - (shiftedxhMult2Ldb.d - shiftConst);
  r = rhMult2L * twoM13;
  k = shiftedxhMult2Ldb.i[LO];
  *H = k >> 13;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 5;
  Add12Cond(t1h, t2, r, xm);
  Add12Cond(t1m, t1l, t2, xl);
  Add12(rh, t3, t1h, t1m);
  Add12(rm, rl, t3, t1l);

  /* Polynomial approximation of 2^(rh + rm + rl) */

  p_t_1_0h = exp2_120_p_coeff_6h;
  p_t_2_0h = p_t_1_0h * rh;
  p_t_3_0h = exp2_120_p_coeff_5h + p_t_2_0h;
  p_t_4_0h = p_t_3_0h * rh;
  Add12(p_t_5_0h,p_t_5_0m,exp2_120_p_coeff_4h,p_t_4_0h);
  MulAdd22(&p_t_6_0h,&p_t_6_0m,exp2_120_p_coeff_3h,exp2_120_p_coeff_3m,rh,rm,p_t_5_0h,p_t_5_0m);
  MulAdd22(&p_t_7_0h,&p_t_7_0m,exp2_120_p_coeff_2h,exp2_120_p_coeff_2m,rh,rm,p_t_6_0h,p_t_6_0m);
  Mul22(&p_t_8_0h,&p_t_8_0m,p_t_7_0h,p_t_7_0m,rh,rm);
  Add23(&p_t_9_0h,&p_t_9_0m,&p_t_9_0l,exp2_120_p_coeff_1h,exp2_120_p_coeff_1m,p_t_8_0h,p_t_8_0m);
  Mul33(&p_t_10_0h,&p_t_10_0m,&p_t_10_0l,rh,rm,rl,p_t_9_0h,p_t_9_0m,p_t_9_0l);
  Add133(&p_t_11_0h,&p_t_11_0m,&p_t_11_0l,exp2_120_p_coeff_0h,p_t_10_0h,p_t_10_0m,p_t_10_0l);
  Renormalize3(&p_resh,&p_resm,&p_resl,p_t_11_0h,p_t_11_0m,p_t_11_0l);

  /* Table access */

  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl1l = twoPowerIndex1[index1].lo;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;
  tbl2l = twoPowerIndex2[index2].lo;

  /* Reconstruction */

  Mul33(&tablesh,&tablesm,&tablesl,tbl1h,tbl1m,tbl1l,tbl2h,tbl2m,tbl2l);
  Mul33(&exp2h,&exp2m,&exp2l,tablesh,tablesm,tablesl,p_resh,p_resm,p_resl);

  Renormalize3(resh,resm,resl,exp2h,exp2m,exp2l);
  
}



/* pow_120

   Approximates 

   2^H * (resh + resm + resl) = 2^(y * (ed + log2(1 + (zh + zm)) + log2(r[index]))) * (1 + eps)

   where ||eps|| <= 2^(-118.5) if -1075 <= y * (ed + log2(1 + (zh + zm)) + log2(r[index])) <= 1024

   Approximates further (ed + log2(1 + (zh + zm)) + log2(r[index))) by log2xh
   where 

   log2xh = (ed + log2(1 + (zh + zm)) + log2(r[index))) * (1 + eps2) 

   where |eps2| <= 2^(-52) 
   and log2xh is exact if 2^ed * ((1 + (zh + zm)) * r[index]) is an integer power of 2.

*/
void pow_120(int *H, double *resh, double *resm, double *resl, double *log2xh,
	     double y, int index, double ed, double zh, double zm) {
  double ylog2xh, ylog2xm, ylog2xl;
  double log2xm, log2xl;

  /* Compute log2(x) */
  log2_130(log2xh,&log2xm,&log2xl,index,ed,zh,zm); 

  /* Compute y * log2(x) */
  Mul133(&ylog2xh,&ylog2xm,&ylog2xl,y,*log2xh,log2xm,log2xl);

  /* Compute 2^(y * log2(x)) */
  exp2_120(H,resh,resm,resl,ylog2xh,ylog2xm,ylog2xl);
  
}

/* pow_round_and_check_rn 

   Checks whether 

   2^H * (powh + powm + powl) 

   which is an approximate to x^y with a relative error or less than 2^(-118.5),
   can be rounded correctly to double precision in round-to-nearest-ties-to-even 
   mode or whether the Table Maker's Dilemma occurs or the case is exact.

   Returns 1 if rounding is possible and affects pow with the rounding
   Returns 0 if rounding is not possible or if the case is exact

   If the returned value is 0, it affects

   G, kh, kl 

   such that 

   2^G * (kh + kl) 
   
   is an approximate to x^y with an relative error of 2^(-117) 

   and such that rounding 

   2^G * kh 

   to double precision is an exact operation.

*/

static inline int pow_round_and_check_rn(double *pow, 
					 int H, double powh, double powm, double powl,
					 int *G, double *kh, double *kl) {
  double th, tm, tl;
  int K, K1, K2;
  db_number twodb, two2db;
  double twoH1074powh, twoH1074powm, shiftedpowh, delta;
  double scaledth;
  double t1m, t1l;
  
  /* We start by bringing H and powh + powm + powl 
     to a form such that

     1 <= powh + powm + powl < 2
  */
  if ((powh < 1.0) ||
      ((powh == 1.0) && (powm < 0.0))) {
    powh *= 2.0;
    powm *= 2.0;
    powl *= 2.0;
    H--;
  }
  if ((powh > 2.0) ||
      ((powh == 2.0) && (powm >= 0.0))) {
    powh *= 0.5;
    powm *= 0.5;
    powl *= 0.5;
    H++;
  }
  
  /* Check now whether we have normal or subnormal rounding 

     The rounding is subnormal iff H <= -1023

     In both cases, we bring H, powh + powm + powl to a form

     2^K * (th + tm + tl) = 2^H * (powh + powm + powm) * (1 + eps)

     where 
     
     (i)   |eps| <= 2^(-118 - 53) = 2^(-171)
     (ii)  2^(-K) * ulp(2^K * th) = 1
     (iii) the rounding of 2^K * th to double precision is exact
           (or produces +inf)

  */
  if (H <= -1023) {
    /* Subnormal rounding 

       In this case, we can neglect powl
       because the rounding bit of the RN rounding 
       is in powh

    */
    twodb.i[HI] = (H + (1074 + 1023)) << 20;
    twodb.i[LO] = 0;
    
    twoH1074powh = twodb.d * powh;
    twoH1074powm = twodb.d * powm;
    
    shiftedpowh = two52 + twoH1074powh;
    th = shiftedpowh - two52;
    delta = twoH1074powh - th;
    
    Add12Cond(tm,tl,delta,twoH1074powm);

    K = -1074;
  } else {
    /* Normal rounding 
       
       In this case, we have exactly:

       2^K * (th + tm + tl) = 2^H * (powh + powm + powm)

    */
    th = powh * two52;
    tm = powm * two52;
    tl = powl * two52;
    K = H - 52;
  }

  /* Return results for exactness case test */
  *G = K;
  *kh = th;
  *kl = tm;

  /* Compute now 

     delta = ABS(0.5 - ABS(tm + tl)) * (1 + eps)

     where |eps| <= 2^(-53)

     The addition 0.5 + (-ABS(tm)) is exact by Sterbenz' lemma

  */
  if (tm > 0.0) {
    t1m = - tm;
    t1l = - tl;
  } else {
    t1m = tm;
    t1l = tl;
  }
  delta = ABS((0.5 + t1m) - t1l);
 
  /* We cannot decide the rounding or have an exact case 
     iff 
     
     delta <= 2^(-118) * th
     
     We can see this in the following drawing:

     result = round(

            +-----------------+------+----------...----+------------
     2^K *  | th              | +/-1 | 0               | delta        )
            +-----------------+------+----------...----+------------
            | <------------- 118 bits -------------> |

  */

  scaledth = th * PRECISEROUNDCST;

  if (delta > scaledth) {
    /* We can round exactly to nearest 
       
       We must correct th to the rounding of 
       th + tm + tl iff tm is greater in 
       absolute value than 0.5 
       or if tm is equal to 0.5 in absolute value and 
       the sign of tm and tl 
       is equal:

                |                 |
       ...------|--------|--------|------...
                |------->-->      |
                ^   tm    tl      ^
                th                round(th + tm + tl)

       If tm is negative, we must correct th by decreasing it
       otherwise we must correct th by increasing it.

    */
    if (ABS(tm) >= 0.5) {
      if (ABS(tm) == 0.5) {
	if (tm < 0.0) {
	  if (tl < 0.0) {
	    /* The same sign of tm and tl, tm is negative */
	    th -= 1.0;
	  }
	} else {
	  if (tl > 0.0) {
	    /* The same sign of tm and tl, tm is positive */
	    th += 1.0;
	  }
	}
      } else {
	/* tm > 0.5 */
	if (tm < 0.0) {
	  th -= 1.0;
	} else {
	  th += 1.0;
	}
      }
    }

    /* Perform now the multiplication 2^K * th 
       
       Note that we must be able to produce 
      
       (i)   0 if we have total underflow
       (ii)  a subnormal result 
       (iii) a normal result
       (iv)  +inf if we have overflow in a TMD case

       We produce K1 + K2 = K with K1 = floor(K/2)
       and multiply in two steps.

    */
    K1 = K >> 1;
    K2 = K - K1;

    twodb.i[HI] = (K1 + 1023) << 20;
    twodb.i[LO] = 0;
    two2db.i[HI] = (K2 + 1023) << 20;
    two2db.i[LO] = 0;
    
    *pow = two2db.d * (twodb.d * th);
    
    return 1;
  } 
  /* Otherwise we return 0 because we cannot round */

  return 0;
}


/* pow_exact_case 

   Checks whether x^y is an exact or half-ulp case for rounding 
   into double precision.

   This means the procedure checks whether x^y can be written on
   not more than 54 bits.

   The procedure uses 2^G * (kh + kl) supposing the following properties:

   * kh + kl holds on at most 54 bits
   * 2^G * kh is representable in double precision (even in the subnormal range)
   * 2^G * (kh + kl) approximates x^y with an error eps less than 2^(-117), i.e. 
   
     2^G * (kh + kl) = x^y * (1 + eps) where |eps| <= 2^(-117)
     
   * log2xh approximates log2(x) with an accuracy equivalent to at least 52 bits
     In particular log2xh is exact if x is an integer power of 2

   Returns 1 if the case is exact or half-ulp
   Returns 0 otherwise

   If returning 1, affects pow with 2^G * (kh + kl) rounded to double precision

*/
int pow_exact_case(double *pow,
		   double x, double y,
		   int G, double kh, double kl, double log2xh) {
  db_number xdb, ydb, tempdb, shiftedEydb, temp2db;
  int E, F, G1, G2;
  double m, n, yh, yl, Eyh, Eyl, ed, Ey;
  double delta;
  double nearestEy;
  double value;

  /* For testing whether x^y is an exact or half-ulp case,
     we have two main cases: 

     (i)  x is an integer power of 2
     (ii) x is not an integer power of 2

     We start by testing if x is an integer power of 2.

  */

  xdb.d = x;
  if ((xdb.i[HI] & 0xfff00000) == 0) {
    /* x is subnormal, scale by 2^52 */
    xdb.d *= 0.4503599627370496e16;
  }
  if (((xdb.i[HI] & 0x000fffff) | xdb.i[LO]) == 0) {
    /* x is an integer power of 2 

       x^y is exact or midpoint iff log2(x) * y is integer

       Since we know that x is an integer power of 2, 
       log2xh is equal to this integer. Since the exponent
       of the double precision number is bounded by the
       exponent range, we know that log2xh is integer and
       bounded by 2^11. It is therefore possible to
       split y into two parts of 21 and 32 bits, to perform
       the multiplication componentwise. Since the result
       is bounded by 2^11, it suffices to compute the 
       nearest integer to the higher word product, and to
       compare the rounding difference to the low word product.
       This splitting is faster than the usual Dekker splitting.

    */
    ydb.d = y;
    ydb.i[LO] = 0;
    yh = ydb.d;
    yl = y - yh;
    Eyh = log2xh * yh;
    Eyl = log2xh * yl;
    delta = ((0.6755399441055744e16 + Eyh) - 0.6755399441055744e16) - Eyh; /* addition rounds, subtractions exact */
    
    if (delta != Eyl) return 0;

  } else {

    /* x is not an integer power of 2 
       
       We have clearly an inexact case if y is negative
       or if y is greater than 35
    
    */
  
    if ((y < 0.0) || (y > 35.0)) return 0;
    
    /* Decompose now y into 
       
       y = 2^F * n 

       Checking F and n, we can then already decide
       some cases using the fact that the worst-case
       accuracy for x^n, n in [|0;35|], is less (in bits) 
       than the accuracy of the approximation of x^y we have
       already in 2^G * (kh + kl).

    */
    decompose(&n,&F,y);
    
    if ((n > 35.0) || (F < -5)) return 0;
  
    if (F < 0) {
      /* Here, -5 <= F <= -1, 3 <= n <= 35, n an odd integer
	 
         We decompose x into 2^E * m where m is an odd integer 

	 Let H, sh and sl such that 2^H * (sh + sl) = 2^G * (kh + kl) and
	 sh + sl is an odd integer.

	 If we have E * 2^F * n = H, we can apply the worst case argument 
	 because we know the worst case for 
    
	 m^(2^F * n) with m odd integer, -5 <= F <= -1, 3 <= n <= 35
    
	 when rounding to 53 bits in both rounding modes.
    
	 E is bounded in magnitude by 2^11. 2^F * n is equal to y by 
	 construction. Since n <= 35, y contains at most 6 significant bits. 
	 The arithmetical multiplication E * y is therefore exact and less 
	 than or equal to 2^17.

	 We check first whether E * y is an integer. If this is the case,
	 we compute sh + sl = 2^(G - E * y) * (kh + kl). Finally, 
	 we check whether sh + sl is an odd integer.

      */

      decompose(&m,&E,x);
    
      ed = (double) E;
    
      Ey = ed * y; /* Exact */
    

      /* Check whether Ey is an integer using the simple shift technique
	 The addition rounds, the substraction is exact by Sterbenz' lemma.
	 If Ey is an integer, the low order word of shiftedEydb is equal to this
	 integer.
      */
      shiftedEydb.d = 0.6755399441055744e16 + Ey;
      nearestEy = shiftedEydb.d - 0.6755399441055744e16;

      if (nearestEy != Ey) return 0; 
      
      /* Here E * y is integer. 
	 Produce now 2^(G - E * y).
      */
      tempdb.i[HI] = (((G - shiftedEydb.i[LO]) + 1023) << 20);
      tempdb.i[LO] = 0;

      /* Check now if sh + sl = tempdb.d * (kh + kl) is an odd
	 integer.
       
	 Since kh and kl are not overlapped, we have two cases:

	 (i)  kl is equal to 0, in which case tempdb.d * kh must be an odd integer
	 (ii) kl is not equal to 0, in which case tempdb.d * kl must be an odd integer

      */
      if (kl == 0.0) value = kh; else value = kl;
    
      value *= tempdb.d; /* Exact because multiplication by positive integer power of 2 */

      if (!isOddInteger(value)) return 0;
    
      /* Here the case is exact by the worst-case argument */   
    }
  
    /* Here, we have either F >= 0 or an exact case 
       
       If F >= 0, we also have an exact case because
       2^F * n = y <= 35, y therefore integer and because
       we can apply the worst case argument.

    */       
  }

  /*
       Here, the case is exact, affect pow with 2^G * (kh + kl) rounded to
       nearest in double precision
       
       Since kh + kl holds on at most 54 bits, 2^G * kh produces
       never any rounding error and kh and kl are not overlapping, 
       2^G * kh is equal to the rounding if 
       (i)  kl is equal to 0
       (ii) kl is not equal to 0 and the mantissa of 2^G * kh is even
       
       If in condition (ii), kl is not equal to 0 and the mantissa of 
       2^G * kh is not even, we correct it depending on the sign of kl.
       
       Remark that G can be such that 2^G is no longer a normal.
       Produce therefore 2^(floor(G/2)) and 2^(G - floor(G/2)) and
       multiply in two steps.

  */

  G1 = G >> 1;
  G2 = G - G1;
  
  tempdb.i[HI] = (G1 + 1023) << 20;
  tempdb.i[LO] = 0;
  temp2db.i[HI] = (G2 + 1023) << 20;
  temp2db.i[LO] = 0;
  
  tempdb.d *= (kh * temp2db.d);
  
  if ((kl != 0.0) && ((tempdb.i[LO] & 1) != 0)) {
    /* We must correct the rounding to the rounding to nearest ties to even */
    if (kl > 0.0) {
      tempdb.l++;
    } else {
      tempdb.l++;
    }
  }
  *pow = tempdb.d;
  
  return 1;
}


double pow_exact_rn(double x, double y, double sign, 
		    int index, double ed, double zh, double zm) {
  int H, G;
  double powh, powm, powl;
  double pow;
  double kh, kl;
  double log2xh;

  pow_120(&H, &powh, &powm, &powl, &log2xh, y, index, ed, zh, zm);

  if (pow_round_and_check_rn(&pow,H,powh,powm,powl,&G,&kh,&kl)) 
    return sign * pow;

  if (pow_exact_case(&pow,x,y,G,kh,kl,log2xh)) 
    return sign * pow;

  //  printf("Could not decide the rounding to nearest of power.\n");

  return -5.0;
}


double pow_rn(double x, double y) {
  db_number xdb, ydb, yhdb, shiftedylog2xhMult2Ldb, powdb, twodb;
  double sign;
  int E, index;
  double log2FastApprox, ed, ylog2xFast, f;
  double yh, yl, ri, logih, logim, yrih, yril, th, zh, zm;
  double p_t_1_0h;
  double p_t_2_0h;
  double p_t_3_0h;
  double p_t_4_0h;
  double p_t_5_0h;
  double p_t_9_0h;
  double p_t_10_0h;
  double p_t_11_0h, p_t_11_0m;
  double p_t_12_0h, p_t_12_0m;
  double log2zh, log2zm;
  double log2yh, log2ym;
  double log2xh, log2xm;
  double ylog2xh, ylog2xm;
  double rh, r;
  int k, index1, index2, H;
  double tbl1, tbl2h, tbl2m;
  double ph;
  double powh, powm;
  double twoH1074powh, twoH1074powm;
  double shiftedpowh, nearestintpowh, delta, deltaint;
  double rest;
  double lowerTerms;
  double temp1;
  double zhSq, zhFour, p35, p46, p36, p7;
  double xSq;
  
  /* Fast rejection of special cases */
  xdb.d = x;
  ydb.d = y;

  if (((((xdb.i[HI] >> 20) + 1) & 0x3ff) <= 1) ||
      ((((ydb.i[HI] >> 20) + 1) & 0x3ff) <= 1)) {

    /* Handle special cases before handling NaNs and Infinities */
    if (x == 1.0)  return 1.0;
    if (y == 0.0)  return 1.0;
    if (y == 1.0)  return x;
    if (y == 2.0)  return x * x; /* Remark: may yield uncorrect rounding on x86 for subnormal results */
    if (y == -1.0) return 1 / x;
    
    if ((x == 0.0) && ((ydb.i[HI] & 0x7ff00000) != 0x7ff00000)) {
      /* x = +/-0 and y is neither NaN nor Infinity
	 
      We have four cases 
      (i)  y < 0:
      (a) y odd integer: return +/- Inf and raise divide-by-zero 
      (b) y not odd integer: return + Ind and raise divide-by-zero
      (ii) (a) y odd integer: return +/- 0
      (b) y not odd integer: return +0
      
      Note that y = 0.0 has already been filtered out.
      */
      if (y < 0.0) {
	if (isOddInteger(y)) 
	  return 1/x;
	else 
	  return 1/(x * x);
      } else {
	if (isOddInteger(y))
	  return x;
	else 
	  return x * x;
      }
    }
    
    /* Handle NaNs and Infinities
       
    Note: the cases (x,y) = (1,NaN) and (x,y) = (NaN,0) have already been handled.
    
    */
    if ((ydb.i[HI] & 0x7ff00000) == 0x7ff00000) {
      /* Here y is NaN or Inf */
      if (((ydb.i[HI] & 0x000fffff) | ydb.i[LO]) != 0) {
	/* Here y is NaN, we return NaN */
	return y;
      } 
      /* Here y is +/- Inf 
	 
      There are three main cases:
      (i)   x = -1: return 1
      (ii)  abs(x) > 1: 
      (a) y = +Inf: return +Inf
      (b) y = -Inf: return +0
      (iii) abs(x) < 1: 
      (a) y = +Inf: return +0
      (b) y = -Inf: return +Inf
      
      Note: the case x = 1 has already been filtered out 
      */
      if (x == -1.0) 
	return 1.0;
      
      /* Here x != 1, x != -1 */
      if ((ABS(x) > 1.0) ^ ((ydb.i[HI] & 0x80000000) == 0)) {
	/* abs(x) > 1 and y = -Inf or abs(x) < 1 and y = +Inf */
	return 0.0;
      } else {
	/* abs(x) > 1 and y = +Inf or abs(x) < 1 and y = -Inf */
	return ABS(y);
      }
    }
    
    /* Here y is neither Inf nor NaN */
    
    if ((xdb.i[HI] & 0x7ff00000) == 0x7ff00000) {
      /* Here x is NaN or Inf */
      if (((xdb.i[HI] & 0x000fffff) | xdb.i[LO]) != 0) {
	/* Here x is NaN, we return NaN */
	return x;
      } 
      /* Here x is +/- Inf 
	 
      There are two main cases:
      
      (i)  x is +Inf 
      (ii) x is -Inf
      */
      if ((xdb.i[HI] & 0x80000000) == 0) {
	/* x is +Inf 
	   
	(a) y > 0: return +Inf
	(b) y < 0: return +0
	
	Note: y = 0 has already been filtered out
	*/
	if (y > 0.0) {
	  return x;
	} else {
	  return 0.0;
	}
      } else {
	/* x is -Inf 
	   
	There are four cases:
	
	(a) y > 0: 
	(*)  y is an odd integer: return -Inf
	(**) y is not an odd integer: return +Inf 
	(b) y < 0:
	(*)  y is an odd integer: return -0
	(**) y is not an odd integer: return +0
	
	Note: y = 0 has already been filtered out
	*/
	if (y > 0.0) {
	  if (isOddInteger(y)) {
	    return x;
	  } else {
	    return -x;
	  }
	} else {
	  if (isOddInteger(y)) {
	    return -0.0;
	  } else {
	    return 0.0;
	  }
	}
      }
    } 
  }
  /* Here both x and y are finite numbers */
  
  /* Test now whether we have the case 

     (-x)^y = (-1)^y * x^y 

     where x is positive

  */

  sign = 1.0;
  if (x < 0.0) {
    /* x is negative
       x^y is defined only if y is integer
    */
    if (!isInteger(y)) {
      /* y is not integer 

         return NaN and raise invalid exception
      */
      return 0.0/0.0;
    }
    /* Here y is integer 
       
       Remove the sign of x and put (-1)^y in sign

    */
    x = -x; xdb.i[HI] &= 0x7fffffff;
    if (isOddInteger(y)) {
      sign = -sign;
    }
  }

  /* Here x is strictly positive and finite */

  /* Test now if abs(y) is in a range giving finite, non-trivial results x^y 

     We have 

     x^y = 2^(y * log2(x)) = 2^(y * log2(2^E * m)) = 2^(y * (E + log2(1 + f)))

     We have overflow iff x^y >= 2^(1024), i.e. y * (E + log2(1 + f)) >= 1024
     We have underflow (flush to zero) iff x^y <= 2^(-1076), i.e. y * (E + log2(1 + f)) <= - 1076
     We round to 1.0 iff abs(y * (E + log2(1 + f))) <= 2^(-54) 

     We approximate log2(1 + f) as

     log2(1 + f) = c * f * alpha(f)  

     where c is a constant and  0.853 < alpha(f) < 1.21

     So we have surely 
     (i)  overflow or underflow if abs(y * (E + c * f)) >= ceil(1076/0.853) = 1261
     (ii) trivial rounding to 1.0 if abs(y * (E + c * f)) <= 2^(-55) <= 2^(-54)/1.21 

  */

  /* We start the computation of the logarithm of x */

  E = 0;
  if ((xdb.i[HI] & 0xfff00000) == 0) {
    /* x is subnormal, make it normal */
    xdb.d *= two52;
    E = -52;
  }

  E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
  index = (xdb.i[HI] & 0x000fffff);
  xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
  index = (index + (1<<(20-L-1))) >> (20-L);
 
  /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
  if (index >= MAXINDEX) { /* corresponds to xdb>sqrt(2)*/
    xdb.i[HI] -= 0x00100000; 
    E++;
  }

  ed = (double) E;

  /* Continue with the exact range reduction of the logarithm of x */

  yhdb.i[HI] = xdb.i[HI];
  yhdb.i[LO] = 0;
  yh = yhdb.d;
  yl = xdb.d - yh;

  
  /* Special handling for y = 3 or y = 4 and x on not more than 21 bits (without subnormals) */
  if ((yl == 0) && ((y == 3.0) || (y == 4.0)) && (E > -255)) {
    if (y == 3.0) 
      return sign * (x * (x * x));
    else {
      xSq = x * x;
      return sign * (xSq * xSq);
    }
  }

  index = index & INDEXMASK;

  /* Now we have 

     log2(x) = E + log2(xdb.d)

     with sqrt(2)/2 < xdb.d < sqrt(2)

     Compute now f such that 1 + f = xdb.d and 
     approximate 

     log2(1 + f) = logFastCoeff * f

  */

  f = xdb.d - 1.0;
  log2FastApprox = ed + logFastCoeff * f;
  ylog2xFast = y * log2FastApprox;

  if (ABS(ylog2xFast) >= 1261.0) {
    if (ylog2xFast > 0.0) {
      /* y * log2(x) is positive, i.e. we overflow */
      return (sign * LARGEST) * LARGEST;
    } else {
      /* y * log2(x) is negative, i.e. we underflow */
      return (sign * SMALLEST) * SMALLEST;
    }    
  }

  if (ABS(ylog2xFast) <= twoM55) {
    /* abs(y * log2(x)) <= 2^(-55), 
       we return 1.0 and set the inexact flag 
    */
    return sign * (1.0 + SMALLEST);
  }

  /* Now, we may still overflow or underflow but on some inputs only */
  
  /* 
     Read tables:
     Read one float for ri
     Read the first two doubles for -log(r_i) (out of three)
     
     Organization of the table:
     
     one struct entry per index, the struct entry containing 
     r, logih, logim and logil in this order
  */
 
  ri = argredtable[index].ri;
  /* 
     Actually we don't need the logarithm entries now
     Move the following two lines to the eventual reconstruction
     As long as we don't have any if in the following code, we can overlap 
     memory access with calculations 
  */
  logih = argredtable[index].logih;
  logim = argredtable[index].logim;
  
  /* Do range reduction:
     
     zh + zm = y * ri - 1.0 correctly
  
     Correctness is assured by use of two part yh + yl and 21 bit ri and Add12
  
     Discard zl for higher monome degrees
  */
  
  yrih = yh * ri;
  yril = yl * ri;
  th = yrih - 1.0; 
  Add12Cond(zh, zm, th, yril); 
  
  /* Polynomial approximation of log2(1 + (zh + zm)) */

  zhSq = zh * zh;

  p35 = log2_70_p_coeff_3h + zhSq * log2_70_p_coeff_5h; 
  p46 = log2_70_p_coeff_4h + zhSq * log2_70_p_coeff_6h;
  zhFour = zhSq * zhSq;

  p36 = p35 + zh * p46;
  p7 = log2_70_p_coeff_7h * zhFour;

  p_t_9_0h = p36 + p7;

  p_t_10_0h = p_t_9_0h * zh;
  Add212(&p_t_11_0h,&p_t_11_0m,log2_70_p_coeff_2h,log2_70_p_coeff_2m,p_t_10_0h);
  MulAdd22(&p_t_12_0h,&p_t_12_0m,log2_70_p_coeff_1h,log2_70_p_coeff_1m,zh,zm,p_t_11_0h,p_t_11_0m);
  Mul22(&log2zh,&log2zm,p_t_12_0h,p_t_12_0m,zh,zm);

  /* Reconstruction */

  Add122(&log2yh,&log2ym,ed,logih,logim);
  Add22(&log2xh,&log2xm,log2yh,log2ym,log2zh,log2zm);

  /* Produce ylog2xh + ylog2xm approximating y * log2(x) 

     Note: neither y nor y * log2(x) may not be subnormal and non-zero 
     because of the fast overflow/underflow/trivial rounding test.

  */

  Mul12(&ylog2xh,&temp1,y,log2xh);
  ylog2xm = temp1 + y * log2xm;
 
  /* Approximate now 2^(y * log2(x)) 

     We have

     2^(ylog2xh + ylog2xm) 
          = 2^ylog2xh * 2^ylog2xm
          = 2^H * 2^(ylog2xh - H) * 2^ylog2xm
	  = 2^H * 2^(i2/2^8) * 2^(i1/2^13) * 2^(ylog2xh - H - i2/2^8 - i1/2^13) * 2^ylog2xm
	  = 2^H * tbl2[i2] * (1 + tbl1[i1]) * (1 + p(rh)) + delta
	  where rh \approx ylog2xh - H - i1/2^7 - i2/2^13 + ylog2xm
  */

  shiftedylog2xhMult2Ldb.d = shiftConstTwoM13 + ylog2xh;
  r = ylog2xh - (shiftedylog2xhMult2Ldb.d - shiftConstTwoM13);
  k = shiftedylog2xhMult2Ldb.i[LO];
  H = k >> 13;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 5;

  /* Renormalize reduced argument 

     This operation produces an error 
     
     2^(z * (1 + eps)) = 2^z * (1 + eps') 

     where |eps| <= 2^(-53) and |eps'| <= 2^(-65)

  */
  rh = r + ylog2xm;

  /* Table reads */
  tbl1 = twoPowerIndex1[index1].hiM1;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Polynomial approximation */
  p_t_1_0h = exp2_p_coeff_3h;
  p_t_2_0h = p_t_1_0h * rh;
  p_t_3_0h = exp2_p_coeff_2h + p_t_2_0h;
  p_t_4_0h = p_t_3_0h * rh;
  p_t_5_0h = exp2_p_coeff_1h + p_t_4_0h;
  ph = p_t_5_0h * rh;

  /* Reconstruction */
  
  lowerTerms = tbl1 + (ph + tbl1 * ph);

  Add212(&powh,&powm,tbl2h,tbl2m,tbl2h * lowerTerms);

  /* Here we have 

     2^H * (powh + powm) = x^y * (1 + eps) 

     with ||eps|| <= 2^(-60)

     Check first if we overflow or underflow.

     Check then if we can perform normal rounding or 
     if we must perhaps perform subnormal rounding.
     We are sure that

     0.5 <= powh + powm <= 4.0

  */

  if (H >= 1025) {
    /* Here, we surely overflow 
       Return sign * inf
    */
    return (sign * LARGEST) * LARGEST;
  }

  if (H <= -1077) {
    /* Here, we surely underflow 
       Return sign * 0 and set inexact flag
    */
    return (sign * SMALLEST) * SMALLEST;
  } 

  if (H > -1022) {
    /* We are sure to be able to perform normal rounding 
       We must still be aware of the fact that the final
       result may overflow 
    */
    if(powh == (powh + (powm * RNROUNDCST))) {
      powdb.d = powh;
      if (H < 1023) {
	powdb.i[HI] += H << 20;
	return sign * powdb.d;
      } else {
	/* May overflow: multiply by 2^H in two steps */
	powdb.i[HI] += (H - 3) << 20;
	return sign * powdb.d * 8.0;
      }
    }
  } else {
    /* We must perhaps perform subnormal rounding 

       We start by renormalizing the double-double
       powh + powm such that 

       1 <= powh + powm < 2

    */

    if ((powh < 1.0) || ((powh == 1.0) && (powm < 0.0))) {
      powh *= 2.0;
      powm *= 2.0;
      H--;
    }
    if ((powh > 2.0) || ((powh == 2.0) && (powm >= 0.0))) {
      powh *= 0.5;
      powm *= 0.5;
      H++;
    }

    /* Here we know that 1 <= powh + powm < 2 

       2^H * (powh + powm) is subnormal or equal to the least normal
       iff H <= -1023

    */
    if (H > -1023) {
      /* We have nevertheless normal rounding */
      if(powh == (powh + (powm * RNROUNDCST))) {
	powdb.d = powh;
	powdb.i[HI] += H << 20;
	return sign * powdb.d;
      }
      /* Here we have normal rounding but could not decide the rounding */
    } else {
      /* We have surely denormal rounding

         This means

	 round(2^H * (powh + powm)) = 2^(-1074) * nearestint(2^(+1074) * 2^H * (powh + powm))

	 We compute the rounding (and test its TMD) of 
	 
	 nearestint(2^(H + 1074) * (powh + powm))

	 by computing first 

	 nearestint(2^(H + 1074) * powh) 
	 
	 and then the absolute error

	 delta = 2^(H + 1074) * powh - nearestint(2^(H + 1074) * powh) + 2^(H + 1074) * powl

	 We can then refute or correct the rounding by tests on delta.

	 We know that 2^(H + 1074) <= 2^(51) and powh <= 2.0. Thus 2^(H + 1074) * powh <= 2^(52)
	 We can hence compute nearestint using the shift trick.
	 
	 We start by constructing 2^(H + 1074)
      */
      
      twodb.i[HI] = (H + (1074 + 1023)) << 20;
      twodb.i[LO] = 0;

      twoH1074powh = twodb.d * powh;
      twoH1074powm = twodb.d * powm;

      shiftedpowh = two52 + twoH1074powh;
      nearestintpowh = shiftedpowh - two52;
      deltaint = twoH1074powh - nearestintpowh;

      /* The next addition produces a very small 
	 rounding error we must account for in the rounding test.
      */
      delta = deltaint + twoH1074powm;

      /* Correct now the rounding */

      if (ABS(delta) > 0.5) {
	/* ABS(delta) > 0.5 */
	if (delta > 0.0) {
	  /* nearestintpowh is too small by 1.0 */
	  nearestintpowh += 1.0; /* exact */
	  delta -= 1.0; /* exact */
	} else {
	  /* nearestintpowh is too great by 1.0 */
	  nearestintpowh -= 1.0; /* exact */
	  delta += 1.0; /* exact */
	}
      }

      /* Perform now the rounding test 

         This test filters out also half-ulp exact cases.
	 Exact cases are not filtered out because C99 allows
	 setting the underflow and/or inexact flags also
	 for exact cases.

         Compute first 

	 rest = abs(0.5 - abs(delta)) * (1 + eps) 

	 where abs(eps) <= 2^(-53)

	 We cannot decide the rounding if 

	 rest <= 2^(H + 1074) * epsmax * 2

	 where epsmax is the bound for the approximating polynomials,
	 
	 here epsmax = 2^(-60)

	 as follows by the following drawing

	 2^52                2^(H + 1074)     2^0            2^(H + 1074) * epsmax
	 +-------------------+----------------+---+------... |
	 | 0 ...           0 | nearestintpowh | 1 |          rest
         +-------------------+----------------+---+------... |
                             | <---- 60 bits approx. ------->|

	 This means that we cannot decide the rounding if 
	 
	 rest * 1/epsmax * 0.5 <= 2^(H + 1074)

	 We store 1/epsmax * 0.5 on SUBNORMROUNDCST
      */
      
      rest = ABS(0.5 - ABS(delta));

      if (rest * SUBNORMROUNDCST > twodb.d) {
	/* Here we could decide the rounding 

	   The result to return is

	   sign * 2^(-1074) * nearestintpowh 

	   which is either subnormal or equal to the 
	   least normal.

	   Since we know that the multiplication
	   does not imply any rounding error, we can
	   perform it using the shift trick and a mask.

	   This trick yields to a 200 cycle speed-up on Athlon.

	*/

	twodb.d = nearestintpowh + two52;
	twodb.i[HI] = (twodb.i[HI] + (1 << 20)) & 0x001fffff;

	return sign * twodb.d;	
      }
    }
  }
  
  /* If we are here, we could not decide the rounding or are half-ulp 

     Launch now exacter phases and exact and half-ulp testing
  
  */

  return pow_exact_rn(x,y,sign,index,ed,zh,zm);

}

