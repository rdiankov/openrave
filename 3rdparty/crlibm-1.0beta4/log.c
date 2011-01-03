

#include <stdio.h>
#include <stdlib.h>
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "log.h"



void p_accu(double *p_resh, double *p_resm, double *p_resl, double xh, double xm) {




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
 

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif



p_t_1_0h = p_coeff_accu_12h;
p_t_2_0h = p_t_1_0h * xh;
p_t_3_0h = p_coeff_accu_11h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * xh;
p_t_5_0h = p_coeff_accu_10h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * xh;
p_t_7_0h = p_coeff_accu_9h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * xh;
Add12(p_t_9_0h,p_t_9_0m,p_coeff_accu_8h,p_t_8_0h);
MulAdd22(&p_t_10_0h,&p_t_10_0m,p_coeff_accu_7h,p_coeff_accu_7m,xh,xm,p_t_9_0h,p_t_9_0m);
MulAdd22(&p_t_11_0h,&p_t_11_0m,p_coeff_accu_6h,p_coeff_accu_6m,xh,xm,p_t_10_0h,p_t_10_0m);
MulAdd22(&p_t_12_0h,&p_t_12_0m,p_coeff_accu_5h,p_coeff_accu_5m,xh,xm,p_t_11_0h,p_t_11_0m);
Mul22(&p_t_13_0h,&p_t_13_0m,p_t_12_0h,p_t_12_0m,xh,xm);
Add122(&p_t_14_0h,&p_t_14_0m,p_coeff_accu_4h,p_t_13_0h,p_t_13_0m);
Mul22(&p_t_15_0h,&p_t_15_0m,p_t_14_0h,p_t_14_0m,xh,xm);
Add23(&p_t_16_0h,&p_t_16_0m,&p_t_16_0l,p_coeff_accu_3h,p_coeff_accu_3m,p_t_15_0h,p_t_15_0m);
Mul233(&p_t_17_0h,&p_t_17_0m,&p_t_17_0l,xh,xm,p_t_16_0h,p_t_16_0m,p_t_16_0l);
Add133(&p_t_18_0h,&p_t_18_0m,&p_t_18_0l,p_coeff_accu_2h,p_t_17_0h,p_t_17_0m,p_t_17_0l);
Mul233(&p_t_19_0h,&p_t_19_0m,&p_t_19_0l,xh,xm,p_t_18_0h,p_t_18_0m,p_t_18_0l);
Add133(&p_t_20_0h,&p_t_20_0m,&p_t_20_0l,p_coeff_accu_1h,p_t_19_0h,p_t_19_0m,p_t_19_0l);
Mul233(&p_t_21_0h,&p_t_21_0m,&p_t_21_0l,xh,xm,p_t_20_0h,p_t_20_0m,p_t_20_0l);
Renormalize3(p_resh,p_resm,p_resl,p_t_21_0h,p_t_21_0m,p_t_21_0l);


}



/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log_rn(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus exact
      The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log2m is smaller than log2h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log2h * ed, log2m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   if(logh == (logh + (logm * RNROUNDCST)))
     return logh;
   else 
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       log2edh = log2h * ed;
       log2edm = log2m * ed;
       log2edl = log2l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyh, logym, logyl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundToNearest3(logh, logm, logl);

     } /* Accurate phase launched */
}


/*************************************************************
 *************************************************************
 *               ROUNDED UPWARDS			     *
 *************************************************************
 *************************************************************/
 double log_ru(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;

   if (x == 1.0) return 0.0; /* This the only case in which the image under log of a double is a double. */

   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus exact
      The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log2m is smaller than log2h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log2h * ed, log2m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   TEST_AND_RETURN_RU(logh, logm, RDROUNDCST);

     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       log2edh = log2h * ed;
       log2edm = log2m * ed;
       log2edl = log2l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyh, logym, logyl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundUpwards3(logh, logm, logl);

     } /* Accurate phase launched */
}


/*************************************************************
 *************************************************************
 *               ROUNDED DOWNWARDS			     *
 *************************************************************
 *************************************************************/
 double log_rd(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;

   if (x == 1.0) return 0.0; /* This the only case in which the image under log of a double is a double. */

   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus exact
      The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log2m is smaller than log2h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log2h * ed, log2m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   TEST_AND_RETURN_RD(logh, logm, RDROUNDCST);

     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       log2edh = log2h * ed;
       log2edm = log2m * ed;
       log2edl = log2l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyh, logym, logyl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundDownwards3(logh, logm, logl);

     } /* Accurate phase launched */
}


/*************************************************************
 *************************************************************
 *               ROUNDED TOWARDS ZERO			     *
 *************************************************************
 *************************************************************/
 double log_rz(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;

   if (x == 1.0) return 0.0; /* This the only case in which the image under log of a double is a double. */

   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus exact
      The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log2m is smaller than log2h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log2h * ed, log2m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   TEST_AND_RETURN_RZ(logh, logm, RDROUNDCST);

     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       log2edh = log2h * ed;
       log2edm = log2m * ed;
       log2edl = log2l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyh, logym, logyl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundTowardsZero3(logh, logm, logl);

     } /* Accurate phase launched */
}

/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log2_rn(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log2TabPolyh, log2TabPolyl, log2yh, log2ym, log2yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log2(x) = E + 1/log(2) * (log(1+z) - log(ri))


      Carry out everything in double double precision

   */
   
   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(2) */

   Mul22(&log2TabPolyh,&log2TabPolyl, RECPRLOG2H, RECPRLOG2L, logTabPolyh, logTabPolyl);

   /* Add E */
   
   Add122(&logh, &logm, ed, log2TabPolyh, log2TabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   if(logh == (logh + (logm * RNROUNDCST)))
     return logh;
   else 
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul233(&log2yh,&log2ym,&log2yl,RECPRLOG2H,RECPRLOG2L,logyh,logym,logyl);
       
       Add133(&loghover,&logmover,&loglover,ed,log2yh,log2ym,log2yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundToNearest3(logh, logm, logl);

     } /* Accurate phase launched */
}

/*************************************************************
 *************************************************************
 *               ROUNDED  UPWARDS			     *
 *************************************************************
 *************************************************************/
 double log2_ru(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log2TabPolyh, log2TabPolyl, log2yh, log2ym, log2yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);

   /* Test now if the argument is an exact power of 2
      i.e. if the mantissa is exactly 1 (0x0..0 with the implicit bit) 
      This test is necessary for filtering out the cases where the final 
      rounding test cannot distinguish between an exact algebraic 
      number and a hard case to round 
   */

   if ((index | xdb.i[LO]) == 0) {
     /* Handle the "trivial" case for log2: 
	The argument is an exact power of 2, return thus
	just the exponant of the number 
     */

     return (double) E;

   }

   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log2(x) = E + 1/log(2) * (log(1+z) - log(ri))


      Carry out everything in double double precision

   */
   
   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(2) */

   Mul22(&log2TabPolyh,&log2TabPolyl, RECPRLOG2H, RECPRLOG2L, logTabPolyh, logTabPolyl);

   /* Add E */
   
   Add122(&logh, &logm, ed, log2TabPolyh, log2TabPolyl);

   /* Rounding test and eventual return or call to the accurate function */

   TEST_AND_RETURN_RU(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul233(&log2yh,&log2ym,&log2yl,RECPRLOG2H,RECPRLOG2L,logyh,logym,logyl);
       
       Add133(&loghover,&logmover,&loglover,ed,log2yh,log2ym,log2yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundUpwards3(logh, logm, logl);

     } /* Accurate phase launched */
}

/*************************************************************
 *************************************************************
 *               ROUNDED DOWNWARDS			     *
 *************************************************************
 *************************************************************/
 double log2_rd(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log2TabPolyh, log2TabPolyl, log2yh, log2ym, log2yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);

   /* Test now if the argument is an exact power of 2
      i.e. if the mantissa is exactly 1 (0x0..0 with the implicit bit) 
      This test is necessary for filtering out the cases where the final 
      rounding test cannot distinguish between an exact algebraic 
      number and a hard case to round 
   */

   if ((index | xdb.i[LO]) == 0) {
     /* Handle the "trivial" case for log2: 
	The argument is an exact power of 2, return thus
	just the exponant of the number 
     */

     return (double) E;

   }

   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log2(x) = E + 1/log(2) * (log(1+z) - log(ri))


      Carry out everything in double double precision

   */
   
   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(2) */

   Mul22(&log2TabPolyh,&log2TabPolyl, RECPRLOG2H, RECPRLOG2L, logTabPolyh, logTabPolyl);

   /* Add E */
   
   Add122(&logh, &logm, ed, log2TabPolyh, log2TabPolyl);

   /* Rounding test and eventual return or call to the accurate function */

   TEST_AND_RETURN_RD(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul233(&log2yh,&log2ym,&log2yl,RECPRLOG2H,RECPRLOG2L,logyh,logym,logyl);
       
       Add133(&loghover,&logmover,&loglover,ed,log2yh,log2ym,log2yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundDownwards3(logh, logm, logl);

     } /* Accurate phase launched */
}

/*************************************************************
 *************************************************************
 *               ROUNDED TOWARDS ZERO			     *
 *************************************************************
 *************************************************************/
 double log2_rz(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log2TabPolyh, log2TabPolyl, log2yh, log2ym, log2yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);

   /* Test now if the argument is an exact power of 2
      i.e. if the mantissa is exactly 1 (0x0..0 with the implicit bit) 
      This test is necessary for filtering out the cases where the final 
      rounding test cannot distinguish between an exact algebraic 
      number and a hard case to round 
   */

   if ((index | xdb.i[LO]) == 0) {
     /* Handle the "trivial" case for log2: 
	The argument is an exact power of 2, return thus
	just the exponant of the number 
     */

     return (double) E;

   }

   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log2(x) = E + 1/log(2) * (log(1+z) - log(ri))


      Carry out everything in double double precision

   */
   
   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(2) */

   Mul22(&log2TabPolyh,&log2TabPolyl, RECPRLOG2H, RECPRLOG2L, logTabPolyh, logTabPolyl);

   /* Add E */
   
   Add122(&logh, &logm, ed, log2TabPolyh, log2TabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   TEST_AND_RETURN_RZ(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul233(&log2yh,&log2ym,&log2yl,RECPRLOG2H,RECPRLOG2L,logyh,logym,logyl);
       
       Add133(&loghover,&logmover,&loglover,ed,log2yh,log2ym,log2yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundTowardsZero3(logh, logm, logl);

     } /* Accurate phase launched */
}

/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log10_rn(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log10TabPolyh, log10TabPolyl;
   double log10yh, log10ym, log10yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log_10(2) as log210h + log210m + log210l where log210h and log210m have 10 trailing zeros
      Multiplication of ed (double E) and log210h and m is thus exact
      The overall accuracy of log10h + log10m + log10l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log210m is smaller than log210h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log210h * ed, log210m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(10) */

   Mul22(&log10TabPolyh,&log10TabPolyl,RECPRLOG10H,RECPRLOG10M,logTabPolyh,logTabPolyl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, log10TabPolyh, log10TabPolyl);

   /* Rounding test and possible return or call to the accurate function */

   if(logh == (logh + (logm * RNROUNDCST)))
     return logh;
   else 
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul33(&log10yh,&log10ym,&log10yl,RECPRLOG10H,RECPRLOG10M,RECPRLOG10L,logyh,logym,logyl);

       log2edh = log210h * ed;
       log2edm = log210m * ed;
       log2edl = log210l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, log10yh, log10ym, log10yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);
                     
       ReturnRoundToNearest3(logh, logm, logl);

     } /* Accurate phase launched */
}


/*************************************************************
 *************************************************************
 *               ROUNDED UPWARDS			     *
 *************************************************************
 *************************************************************/
 double log10_ru(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log10TabPolyh, log10TabPolyl;
   double log10yh, log10ym, log10yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log_10(2) as log210h + log210m + log210l where log210h and log210m have 10 trailing zeros
      Multiplication of ed (double E) and log210h and m is thus exact
      The overall accuracy of log10h + log10m + log10l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log210m is smaller than log210h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log210h * ed, log210m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(10) */

   Mul22(&log10TabPolyh,&log10TabPolyl,RECPRLOG10H,RECPRLOG10M,logTabPolyh,logTabPolyl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, log10TabPolyh, log10TabPolyl);

   /* Rounding test and possible return or call to the accurate function */


   TEST_AND_RETURN_RU(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul33(&log10yh,&log10ym,&log10yl,RECPRLOG10H,RECPRLOG10M,RECPRLOG10L,logyh,logym,logyl);

       log2edh = log210h * ed;
       log2edm = log210m * ed;
       log2edl = log210l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, log10yh, log10ym, log10yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);

       ReturnRoundUpwards3Unfiltered(logh, logm, logl, WORSTCASEACCURACY);                     

     } /* Accurate phase launched */
}



/*************************************************************
 *************************************************************
 *               ROUNDED DOWNWARDS			     *
 *************************************************************
 *************************************************************/
 double log10_rd(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log10TabPolyh, log10TabPolyl;
   double log10yh, log10ym, log10yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log_10(2) as log210h + log210m + log210l where log210h and log210m have 10 trailing zeros
      Multiplication of ed (double E) and log210h and m is thus exact
      The overall accuracy of log10h + log10m + log10l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log210m is smaller than log210h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log210h * ed, log210m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(10) */

   Mul22(&log10TabPolyh,&log10TabPolyl,RECPRLOG10H,RECPRLOG10M,logTabPolyh,logTabPolyl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, log10TabPolyh, log10TabPolyl);

   /* Rounding test and possible return or call to the accurate function */


   TEST_AND_RETURN_RD(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul33(&log10yh,&log10ym,&log10yl,RECPRLOG10H,RECPRLOG10M,RECPRLOG10L,logyh,logym,logyl);

       log2edh = log210h * ed;
       log2edm = log210m * ed;
       log2edl = log210l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, log10yh, log10ym, log10yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);

       ReturnRoundDownwards3Unfiltered(logh, logm, logl, WORSTCASEACCURACY);                     

     } /* Accurate phase launched */
}


/*************************************************************
 *************************************************************
 *               ROUNDED TOWARDS ZERO			     *
 *************************************************************
 *************************************************************/
 double log10_rz(double x){ 
   db_number xdb, yhdb;
   double yh, yl, ed, ri, logih, logim, logil, yrih, yril, th, zh, zl;
   double ph, pl, pm, log2edh, log2edl, log2edm, logTabPolyh, logTabPolyl, logh, logm, logl;
   int E, index;
   double zhSquare, zhCube, zhSquareHalf;
   double p35, p46, p36;
   double pUpper;
   double zhSquareHalfPlusZl;
   double zhFour;
   double logyh, logym, logyl;
   double loghover, logmover, loglover;
   double log10TabPolyh, log10TabPolyl;
   double log10yh, log10ym, log10yl;


   E=0;
   xdb.d=x;

   /* Filter cases */
   if (xdb.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb.i[HI] & 0x7fffffff)|xdb.i[LO])==0){
       return -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if (xdb.i[HI] < 0){ 
       return (x-x)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E = -52; 		
     xdb.d *= two52; 	  /* make x a normal number    */ 
   }
    
   if (xdb.i[HI] >= 0x7ff00000){
     return  x+x;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E += (xdb.i[HI]>>20)-1023;             /* extract the exponent */
   index = (xdb.i[HI] & 0x000fffff);
   xdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
   index = (index + (1<<(20-L-1))) >> (20-L);
 
   /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
   if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
     xdb.i[HI] -= 0x00100000; 
     E++;
   }

   yhdb.i[HI] = xdb.i[HI];
   yhdb.i[LO] = 0;
   yh = yhdb.d;
   yl = xdb.d - yh;

   index = index & INDEXMASK;
   /* Cast integer E into double ed for multiplication later */
   ed = (double) E;

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

      zh + zl = y * ri - 1.0 exactly

      Exactness is assured by use of two part yh + yl and 21 bit ri and Add12

      Discard zl for higher monome degrees
   */

   yrih = yh * ri;
   yril = yl * ri;
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* Polynomial approximation */

  zhSquare = zh * zh; /* 1 */

  p35 = p_coeff_3h + zhSquare * p_coeff_5h; /* 3 */
  p46 = p_coeff_4h + zhSquare * p_coeff_6h; /* 3 */
  zhCube = zhSquare * zh;   /* 2 */
  zhSquareHalf = p_coeff_2h * zhSquare; /* 2 */
  zhFour = zhSquare * zhSquare; /* 2 */

  p36 = zhCube * p35 + zhFour * p46; /* 4 */
  zhSquareHalfPlusZl = zhSquareHalf + zl; /* 3 */

  pUpper = zhSquareHalfPlusZl + p36; /* 5 */
  
  Add12(ph,pl,zh,pUpper); /* 8 */

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log_10(2) as log210h + log210m + log210l where log210h and log210m have 10 trailing zeros
      Multiplication of ed (double E) and log210h and m is thus exact
      The overall accuracy of log10h + log10m + log10l is 53 * 3 - 24 = 135 which
      is enough for the accurate phase
      The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
      Nevertheless the storage with trailing zeros implies an overlap of the tabulated
      triple double values. We have to take it into account for the accurate phase 
      basic procedures for addition and multiplication
      The condition on the next Add12 is verified as log210m is smaller than log210h 
      and both are scaled by ed
   */

   Add12(log2edh, log2edl, log210h * ed, log210m * ed);

   /* Add logih and logim to ph and pl */

   Add22(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Multiply by 1/log(10) */

   Mul22(&log10TabPolyh,&log10TabPolyl,RECPRLOG10H,RECPRLOG10M,logTabPolyh,logTabPolyl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22(&logh, &logm, log2edh, log2edl, log10TabPolyh, log10TabPolyl);

   /* Rounding test and possible return or call to the accurate function */


   TEST_AND_RETURN_RZ(logh, logm, RDROUNDCST);
     {

       logil = argredtable[index].logil;

       p_accu(&ph, &pm, &pl, zh, zl);

       Add33(&logyh, &logym, &logyl, logih, logim, logil, ph, pm, pl);

       Mul33(&log10yh,&log10ym,&log10yl,RECPRLOG10H,RECPRLOG10M,RECPRLOG10L,logyh,logym,logyl);

       log2edh = log210h * ed;
       log2edm = log210m * ed;
       log2edl = log210l * ed;

       Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, log10yh, log10ym, log10yl);

       Renormalize3(&logh,&logm,&logl,loghover,logmover,loglover);

       ReturnRoundTowardsZero3Unfiltered(logh, logm, logl, WORSTCASEACCURACY);                     

     } /* Accurate phase launched */
}
