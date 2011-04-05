/* 
 * This function computes log2, correctly rounded, 
 * using experimental techniques based on triple double arithmetics

 THIS IS EXPERIMENTAL SOFTWARE
 
 *
 * Author :  Christoph Lauter
 * christoph.lauter at ens-lyon.fr
 *

 To have it replace the crlibm log2, do:

 gcc -DHAVE_CONFIG_H -I.  -fPIC  -O2 -c log2-td.c;   mv log2-td.o log2_accurate.o; make 

 
*/


#include <stdio.h>
#include <stdlib.h>
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "log2-td.h"
#ifdef BUILD_INTERVAL_FUNCTIONS
#include "interval.h"
#endif


#define AVOID_FMA 0


void log2_td_accurate(double *logb2h, double *logb2m, double *logb2l, int E, double ed, int index, double zh, double zl, double logih, double logim) {
  double highPoly, t1h, t1l, t2h, t2l, t3h, t3l, t4h, t4l, t5h, t5l, t6h, t6l, t7h, t7l, t8h, t8l, t9h, t9l, t10h, t10l, t11h, t11l;
  double t12h, t12l, t13h, t13l, t14h, t14l, zSquareh, zSquarem, zSquarel, zCubeh, zCubem, zCubel, higherPolyMultZh, higherPolyMultZm;
  double higherPolyMultZl, zSquareHalfh, zSquareHalfm, zSquareHalfl, polyWithSquareh, polyWithSquarem, polyWithSquarel;
  double polyh, polym, polyl, logil, logyh, logym, logyl, loghover, logmover, loglover, log2edhover, log2edmover, log2edlover;
  double log2edh, log2edm, log2edl, logb2hover, logb2mover, logb2lover;


#if EVAL_PERF
  crlibm_second_step_taken++;
#endif


  /* Accurate phase:

     Argument reduction is already done. 
     We must return logh, logm and logl representing the intermediate result in 118 bits precision.

     We use a 14 degree polynomial, computing the first 3 (the first is 0) coefficients in triple double,
     calculating the next 7 coefficients in double double arithmetics and the last in double.

     We must account for zl starting with the monome of degree 4 (7^3 + 53 - 7 >> 118); so 
     double double calculations won't account for it.

  */

  /* Start of the horner scheme */

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
  highPoly = FMA(FMA(FMA(FMA(accPolyC14,zh,accPolyC13),zh,accPolyC12),zh,accPolyC11),zh,accPolyC10);
#else
  highPoly = accPolyC10 + zh * (accPolyC11 + zh * (accPolyC12 + zh * (accPolyC13 + zh * accPolyC14)));
#endif
  
  /* We want to write 

     accPolyC3 + zh * (accPoly4 + zh * (accPoly5 + zh * (accPoly6 + zh * (accPoly7 + zh * (accPoly8 + zh * (accPoly9 + zh * highPoly))))));
     (        t14  t13         t12  t11         t10   t9          t8   t7          t6   t5          t4   t3          t2   t1  )

     with all additions and multiplications in double double arithmetics
     but we will produce intermediate results labelled t1h/t1l thru t14h/t14l
  */

  Mul12(&t1h, &t1l, zh, highPoly);
  Add22(&t2h, &t2l, accPolyC9h, accPolyC9l, t1h, t1l);
  Mul22(&t3h, &t3l, zh, zl, t2h, t2l);
  Add22(&t4h, &t4l, accPolyC8h, accPolyC8l, t3h, t3l);
  Mul22(&t5h, &t5l, zh, zl, t4h, t4l);
  Add22(&t6h, &t6l, accPolyC7h, accPolyC7l, t5h, t5l);
  Mul22(&t7h, &t7l, zh, zl, t6h, t6l);
  Add22(&t8h, &t8l, accPolyC6h, accPolyC6l, t7h, t7l);
  Mul22(&t9h, &t9l, zh, zl, t8h, t8l);
  Add22(&t10h, &t10l, accPolyC5h, accPolyC5l, t9h, t9l);
  Mul22(&t11h, &t11l, zh, zl, t10h, t10l);
  Add22(&t12h, &t12l, accPolyC4h, accPolyC4l, t11h, t11l);
  Mul22(&t13h, &t13l, zh, zl, t12h, t12l);
  Add22(&t14h, &t14l, accPolyC3h, accPolyC3l, t13h, t13l);

  /* We must now prepare (zh + zl)^2 and (zh + zl)^3 as triple doubles */

  Mul23(&zSquareh, &zSquarem, &zSquarel, zh, zl, zh, zl); 
  Mul233(&zCubeh, &zCubem, &zCubel, zh, zl, zSquareh, zSquarem, zSquarel); 
  
  /* We can now multiplicate the middle and higher polynomial by z^3 */

  Mul233(&higherPolyMultZh, &higherPolyMultZm, &higherPolyMultZl, t14h, t14l, zCubeh, zCubem, zCubel);
  
  /* Multiply now z^2 by -1/2 (exact op) and add to middle and higher polynomial */
  
  zSquareHalfh = zSquareh * -0.5;
  zSquareHalfm = zSquarem * -0.5;
  zSquareHalfl = zSquarel * -0.5;

  Add33(&polyWithSquareh, &polyWithSquarem, &polyWithSquarel, 
	zSquareHalfh, zSquareHalfm, zSquareHalfl, 
	higherPolyMultZh, higherPolyMultZm, higherPolyMultZl);

  /* Add now zh and zl to obtain the polynomial evaluation result */

  Add233(&polyh, &polym, &polyl, zh, zl, polyWithSquareh, polyWithSquarem, polyWithSquarel);

  /* Reconstruct now log(y) = log(1 + z) - log(ri) by adding logih, logim, logil
     logil has not been read to the time, do this first 
  */

  logil =  argredtable[index].logil;

  Add33(&logyh, &logym, &logyl, logih, logim, logil, polyh, polym, polyl);

  /* Multiply log2 with E, i.e. log2h, log2m, log2l by ed 
     ed is always less than 2^(12) and log2h and log2m are stored with at least 12 trailing zeros 
     So multiplying naively is correct (up to 134 bits at least)

     The final result is thus obtained by adding log2 * E to log(y)
  */

  log2edhover = log2h * ed;
  log2edmover = log2m * ed;
  log2edlover = log2l * ed;

  /* It may be necessary to renormalize the tabulated value (multiplied by ed) before adding
     the to the log(y)-result 

     If needed, uncomment the following Renormalize3-Statement and comment out the copies 
     following it.
  */

  /* Renormalize3(&log2edh, &log2edm, &log2edl, log2edhover, log2edmover, log2edlover); */

  log2edh = log2edhover;
  log2edm = log2edmover;
  log2edl = log2edlover;

  Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyh, logym, logyl);


  /* Change logarithm base from natural base to base 2 by multiplying */

  Mul233(&logb2hover, &logb2mover, &logb2lover, log2invh, log2invl, loghover, logmover, loglover);


  /* Since we can not guarantee in each addition and multiplication procedure that 
     the results are not overlapping, we must renormalize the result before handing
     it over to the final rounding
  */

  Renormalize3(logb2h,logb2m,logb2l,logb2hover,logb2mover,logb2lover);

}



/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log2_rn(double x){ 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb2h, logb2m, logb2l;
   int E, index;

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
   y = xdb.d;
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

      zh + zl = y * ri - 1.0 correctly

      Correctness is assured by use of Mul12 and Add12
      even if we don't force ri to have its' LSBs set to zero

      Discard zl for higher monome degrees
   */

   Mul12(&yrih, &yril, y, ri);
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
      using an ad hoc method

   */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
   polyHorner = FMA(FMA(FMA(FMA(c7,zh,c6),zh,c5),zh,c4),zh,c3);
#else
   polyHorner = c3 + zh * (c4 + zh * (c5 + zh * (c6 + zh * c7)));
#endif

   Mul12(&zhSquareh, &zhSquarel, zh, zh);
   polyUpper = polyHorner * (zh * zhSquareh);
   zhSquareHalfh = zhSquareh * -0.5;
   zhSquareHalfl = zhSquarel * -0.5;
   Add12(t1h, t1l, polyUpper, -1 * (zh * zl));
   Add22(&t2h, &t2l, zh, zl, zhSquareHalfh, zhSquareHalfl);
   Add22(&ph, &pl, t2h, t2l, t1h, t1l);

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus correct
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

   /* Add logih and logim to ph and pl 

      We must use conditioned Add22 as logih can move over ph
   */

   Add22Cond(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22Cond(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);



   /* Change logarithm base from natural base to base 2 by multiplying */

   Mul22(&logb2h, &logb2m, log2invh, log2invl, logh, logm);


   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = ROUNDCST1;
   else
      roundcst = ROUNDCST2;


   if(logb2h == (logb2h + (logb2m * roundcst)))
     return logb2h;
   else 
     {
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log2_td_accurate(&logb2h, &logb2m, &logb2l, E, ed, index, zh, zl, logih, logim); 
       
       ReturnRoundToNearest3(logb2h, logb2m, logb2l);

     } /* Accurate phase launched */
 }


/*************************************************************
 *************************************************************
 *               ROUNDED  UPWARDS			     *
 *************************************************************
 *************************************************************/
 double log2_ru(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb2h, logb2m, logb2l;
   int E, index;

  
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
   y = xdb.d;
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

      zh + zl = y * ri - 1.0 correctly

      Correctness is assured by use of Mul12 and Add12
      even if we don't force ri to have its' LSBs set to zero

      Discard zl for higher monome degrees
   */

   Mul12(&yrih, &yril, y, ri);
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
      using an ad hoc method

   */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
   polyHorner = FMA(FMA(FMA(FMA(c7,zh,c6),zh,c5),zh,c4),zh,c3);
#else
   polyHorner = c3 + zh * (c4 + zh * (c5 + zh * (c6 + zh * c7)));
#endif

   Mul12(&zhSquareh, &zhSquarel, zh, zh);
   polyUpper = polyHorner * (zh * zhSquareh);
   zhSquareHalfh = zhSquareh * -0.5;
   zhSquareHalfl = zhSquarel * -0.5;
   Add12(t1h, t1l, polyUpper, -1 * (zh * zl));
   Add22(&t2h, &t2l, zh, zl, zhSquareHalfh, zhSquareHalfl);
   Add22(&ph, &pl, t2h, t2l, t1h, t1l);

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus correct
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

   /* Add logih and logim to ph and pl 

      We must use conditioned Add22 as logih can move over ph
   */

   Add22Cond(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22Cond(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Change logarithm base from natural base to base 2 by multiplying */

   Mul22(&logb2h, &logb2m, log2invh, log2invl, logh, logm);

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RU(logb2h, logb2m, roundcst);

#if DEBUG
  printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log2_td_accurate(&logb2h, &logb2m, &logb2l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundUpwards3(logb2h, logb2m, logb2l);

 } 


/*************************************************************
 *************************************************************
 *               ROUNDED  DOWNWARDS			     *
 *************************************************************
 *************************************************************/
 double log2_rd(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb2h, logb2m, logb2l;
   int E, index;

  
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
   y = xdb.d;
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

      zh + zl = y * ri - 1.0 correctly

      Correctness is assured by use of Mul12 and Add12
      even if we don't force ri to have its' LSBs set to zero

      Discard zl for higher monome degrees
   */

   Mul12(&yrih, &yril, y, ri);
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
      using an ad hoc method

   */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
   polyHorner = FMA(FMA(FMA(FMA(c7,zh,c6),zh,c5),zh,c4),zh,c3);
#else
   polyHorner = c3 + zh * (c4 + zh * (c5 + zh * (c6 + zh * c7)));
#endif

   Mul12(&zhSquareh, &zhSquarel, zh, zh);
   polyUpper = polyHorner * (zh * zhSquareh);
   zhSquareHalfh = zhSquareh * -0.5;
   zhSquareHalfl = zhSquarel * -0.5;
   Add12(t1h, t1l, polyUpper, -1 * (zh * zl));
   Add22(&t2h, &t2l, zh, zl, zhSquareHalfh, zhSquareHalfl);
   Add22(&ph, &pl, t2h, t2l, t1h, t1l);

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus correct
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

   /* Add logih and logim to ph and pl 

      We must use conditioned Add22 as logih can move over ph
   */

   Add22Cond(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22Cond(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Change logarithm base from natural base to base 2 by multiplying */

   Mul22(&logb2h, &logb2m, log2invh, log2invl, logh, logm);

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RD(logb2h, logb2m, roundcst);

#if DEBUG
  printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log2_td_accurate(&logb2h, &logb2m, &logb2l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundDownwards3(logb2h, logb2m, logb2l);
 } 

/*************************************************************
 *************************************************************
 *               ROUNDED  TOWARDS ZERO			     *
 *************************************************************
 *************************************************************/
 double log2_rz(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb2h, logb2m, logb2l;
   int E, index;

  
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
   y = xdb.d;
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

      zh + zl = y * ri - 1.0 correctly

      Correctness is assured by use of Mul12 and Add12
      even if we don't force ri to have its' LSBs set to zero

      Discard zl for higher monome degrees
   */

   Mul12(&yrih, &yril, y, ri);
   th = yrih - 1.0; 
   Add12Cond(zh, zl, th, yril); 

   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
      using an ad hoc method

   */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
   polyHorner = FMA(FMA(FMA(FMA(c7,zh,c6),zh,c5),zh,c4),zh,c3);
#else
   polyHorner = c3 + zh * (c4 + zh * (c5 + zh * (c6 + zh * c7)));
#endif

   Mul12(&zhSquareh, &zhSquarel, zh, zh);
   polyUpper = polyHorner * (zh * zhSquareh);
   zhSquareHalfh = zhSquareh * -0.5;
   zhSquareHalfl = zhSquarel * -0.5;
   Add12(t1h, t1l, polyUpper, -1 * (zh * zl));
   Add22(&t2h, &t2l, zh, zl, zhSquareHalfh, zhSquareHalfl);
   Add22(&ph, &pl, t2h, t2l, t1h, t1l);

   /* Reconstruction 

      Read logih and logim in the tables (already done)
      
      Compute log(x) = E * log(2) + log(1+z) - log(ri)
      i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

      Carry out everything in double double precision

   */
   
   /* 
      We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
      Multiplication of ed (double E) and log2h is thus correct
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

   /* Add logih and logim to ph and pl 

      We must use conditioned Add22 as logih can move over ph
   */

   Add22Cond(&logTabPolyh, &logTabPolyl, logih, logim, ph, pl);

   /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

   Add22Cond(&logh, &logm, log2edh, log2edl, logTabPolyh, logTabPolyl);

   /* Change logarithm base from natural base to base 2 by multiplying */

   Mul22(&logb2h, &logb2m, log2invh, log2invl, logh, logm);

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RZ(logb2h, logb2m, roundcst);

#if DEBUG
  printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log2_td_accurate(&logb2h, &logb2m, &logb2l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundTowardsZero3(logb2h, logb2m, logb2l);
 } 

#ifdef BUILD_INTERVAL_FUNCTIONS
 interval j_log2(interval x) { 
   interval res;
   double res_inf, res_sup, restemp_inf, restemp_sup;
   int infDone, supDone;
   int roundable;
   db_number xdb_inf;
   double y_inf, ed_inf, ri_inf, logih_inf, logim_inf, yrih_inf, yril_inf, th_inf, zh_inf, zl_inf;
   double polyHorner_inf, zhSquareh_inf, zhSquarel_inf, polyUpper_inf, zhSquareHalfh_inf, zhSquareHalfl_inf;
   double t1h_inf, t1l_inf, t2h_inf, t2l_inf, ph_inf, pl_inf, log2edh_inf, log2edl_inf, logTabPolyh_inf, logTabPolyl_inf, logh_inf, logm_inf, roundcst_inf;
   double logb2h_inf, logb2m_inf, logb2l_inf;
   int E_inf, index_inf;

   db_number xdb_sup;
   double y_sup, ed_sup, ri_sup, logih_sup, logim_sup, yrih_sup, yril_sup, th_sup, zh_sup, zl_sup;
   double polyHorner_sup, zhSquareh_sup, zhSquarel_sup, polyUpper_sup, zhSquareHalfh_sup, zhSquareHalfl_sup;
   double t1h_sup, t1l_sup, t2h_sup, t2l_sup, ph_sup, pl_sup, log2edh_sup, log2edl_sup, logTabPolyh_sup, logTabPolyl_sup, logh_sup, logm_sup, roundcst_sup;
   double logb2h_sup, logb2m_sup, logb2l_sup;
   int E_sup, index_sup;
   
   double x_inf, x_sup;
   x_inf=LOW(x);
   x_sup=UP(x);
   infDone=0; supDone=0;

   E_inf=0;
   E_sup=0;
   xdb_inf.d=x_inf;
   xdb_sup.d=x_sup;

   /* Filter cases */
   if (xdb_inf.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb_inf.i[HI] & 0x7fffffff)|xdb_inf.i[LO])==0){
       infDone=1;
       restemp_inf = -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if ((xdb_inf.i[HI] < 0) && (infDone==0)){ 
       infDone=1;
       restemp_inf = (x_inf-x_inf)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E_inf = -52; 		
     xdb_inf.d *= two52; 	  /* make x a normal number    */ 
   }

   if ((xdb_inf.i[HI] >= 0x7ff00000) && (infDone==0)){
     infDone=1;
     restemp_inf =  x_inf+x_inf;				 /* Inf or Nan       */
   }

   /* Filter cases */
   if (xdb_sup.i[HI] < 0x00100000){        /* x < 2^(-1022)    */
     if (((xdb_sup.i[HI] & 0x7fffffff)|xdb_sup.i[LO])==0){
       supDone=1;
       restemp_sup = -1.0/0.0;     
     }                    		   /* log(+/-0) = -Inf */
     if ((xdb_sup.i[HI] < 0) && (supDone==0)){ 
       supDone=1;
       restemp_sup = (x_sup-x_sup)/0;                      /* log(-x) = Nan    */
     }
     /* Subnormal number */
     E_sup = -52; 		
     xdb_sup.d *= two52; 	  /* make x a normal number    */ 
   }

   if ((xdb_sup.i[HI] >= 0x7ff00000) && (supDone==0)){
     supDone=1;
     restemp_sup = x_sup+x_sup;				 /* Inf or Nan       */
   }
   
   
   /* Extract exponent and mantissa 
      Do range reduction,
      yielding to E holding the exponent and
      y the mantissa between sqrt(2)/2 and sqrt(2)
   */
   E_inf += (xdb_inf.i[HI]>>20)-1023;             /* extract the exponent */
   E_sup += (xdb_sup.i[HI]>>20)-1023;             /* extract the exponent */
   index_inf = (xdb_inf.i[HI] & 0x000fffff);
   index_sup = (xdb_sup.i[HI] & 0x000fffff);

   /* Test now if the argument is an exact power of 2
      i.e. if the mantissa is exactly 1 (0x0..0 with the implicit bit) 
      This test is necessary for filtering out the cases where the final 
      rounding test cannot distinguish between an exact algebraic 
      number and a hard case to round 
   */

   if (((index_inf | xdb_inf.i[LO]) == 0) && (infDone==0)) {
     /* Handle the "trivial" case for log2: 
	The argument is an exact power of 2, return thus
	just the exponant of the number 
     */

     supDone=1;
     restemp_inf = (double) E_inf;

   }
   if (((index_sup | xdb_sup.i[LO]) == 0) && (supDone==0)) {
     /* Handle the "trivial" case for log2: 
	The argument is an exact power of 2, return thus
	just the exponant of the number 
     */

     supDone=1;
     restemp_sup = (double) E_sup;

   }

   if((infDone==1) && (supDone==1))
   {
     ASSIGN_LOW(res,restemp_inf);
     ASSIGN_UP(res,restemp_sup);
     return res;
   }
   if((infDone==0) && (supDone==0))
   {
     xdb_inf.i[HI] =  index_inf | 0x3ff00000;	/* do exponent = 0 */
     xdb_sup.i[HI] =  index_sup | 0x3ff00000;	/* do exponent = 0 */
     index_inf = (index_inf + (1<<(20-L-1))) >> (20-L);
     index_sup = (index_sup + (1<<(20-L-1))) >> (20-L);

 
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index_inf >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       xdb_inf.i[HI] -= 0x00100000; 
       E_inf++;
     }
     if (index_sup >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       xdb_sup.i[HI] -= 0x00100000; 
       E_sup++;
     }
     y_inf = xdb_inf.d;
     y_sup = xdb_sup.d;
     index_inf = index_inf & INDEXMASK;
     index_sup = index_sup & INDEXMASK;
     /* Cast integer E into double ed for multiplication later */
     ed_inf = (double) E_inf;
     ed_sup = (double) E_sup;
     /* 
        Read tables:
        Read one float for ri
        Read the first two doubles for -log(r_i) (out of three)

        Organization of the table:

        one struct entry per index, the struct entry containing 
        r, logih, logim and logil in this order
     */
   
     ri_inf = argredtable[index_inf].ri;
     ri_sup = argredtable[index_sup].ri;

     /* 
        Actually we don't need the logarithm entries now
        Move the following two lines to the eventual reconstruction
        As long as we don't have any if in the following code, we can overlap 
        memory access with calculations 
     */
     logih_inf = argredtable[index_inf].logih;
     logih_sup = argredtable[index_sup].logih;
     logim_inf = argredtable[index_inf].logim;
     logim_sup = argredtable[index_sup].logim;

     /* Do range reduction:

        zh + zl = y * ri - 1.0 correctly

        Correctness is assured by use of Mul12 and Add12
        even if we don't force ri to have its' LSBs set to zero

        Discard zl for higher monome degrees
     */

     Mul12(&yrih_inf, &yril_inf, y_inf, ri_inf);
     Mul12(&yrih_sup, &yril_sup, y_sup, ri_sup);
     th_inf = yrih_inf - 1.0; 
     th_sup = yrih_sup - 1.0; 
     Add12Cond(zh_inf, zl_inf, th_inf, yril_inf); 
     Add12Cond(zh_sup, zl_sup, th_sup, yril_sup); 


     /* 
        Polynomial evaluation

        Use a 7 degree polynomial
        Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
        Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
        using an ad hoc method

     */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
     polyHorner_inf = FMA(FMA(FMA(FMA(c7_inf,zh_inf,c6),zh_inf,c5),zh_inf,c4),zh_inf,c3);
     polyHorner_sup = FMA(FMA(FMA(FMA(c7,zh_sup,c6),zh_sup,c5),zh_sup,c4),zh_sup,c3);
#else
     polyHorner_inf = c3 + zh_inf * (c4 + zh_inf * (c5 + zh_inf * (c6 + zh_inf * c7)));
     polyHorner_sup = c3 + zh_sup * (c4 + zh_sup * (c5 + zh_sup * (c6 + zh_sup * c7)));
#endif

     Mul12(&zhSquareh_inf, &zhSquarel_inf, zh_inf, zh_inf);
     Mul12(&zhSquareh_sup, &zhSquarel_sup, zh_sup, zh_sup);
     polyUpper_inf = polyHorner_inf * (zh_inf * zhSquareh_inf);
     polyUpper_sup = polyHorner_sup * (zh_sup * zhSquareh_sup);
     zhSquareHalfh_inf = zhSquareh_inf * -0.5;
     zhSquareHalfh_sup = zhSquareh_sup * -0.5;
     zhSquareHalfl_inf = zhSquarel_inf * -0.5;
     zhSquareHalfl_sup = zhSquarel_sup * -0.5;
     Add12(t1h_inf, t1l_inf, polyUpper_inf, -1 * (zh_inf * zl_inf));
     Add12(t1h_sup, t1l_sup, polyUpper_sup, -1 * (zh_sup * zl_sup));
     Add22(&t2h_inf, &t2l_inf, zh_inf, zl_inf, zhSquareHalfh_inf, zhSquareHalfl_inf);
     Add22(&t2h_sup, &t2l_sup, zh_sup, zl_sup, zhSquareHalfh_sup, zhSquareHalfl_sup);
     Add22(&ph_inf, &pl_inf, t2h_inf, t2l_inf, t1h_inf, t1l_inf);
     Add22(&ph_sup, &pl_sup, t2h_sup, t2l_sup, t1h_sup, t1l_sup);


     /* Reconstruction 

        Read logih and logim in the tables (already done)
      
        Compute log(x) = E * log(2) + log(1+z) - log(ri)
        i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

        Carry out everything in double double precision

     */
   
     /* 
        We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
        Multiplication of ed (double E) and log2h is thus correct
        The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
        is enough for the accurate phase
        The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
        Nevertheless the storage with trailing zeros implies an overlap of the tabulated
        triple double values. We have to take it into account for the accurate phase 
        basic procedures for addition and multiplication
        The condition on the next Add12 is verified as log2m is smaller than log2h 
        and both are scaled by ed
     */

     Add12(log2edh_inf, log2edl_inf, log2h * ed_inf, log2m * ed_inf);
     Add12(log2edh_sup, log2edl_sup, log2h * ed_sup, log2m * ed_sup);

     /* Add logih and logim to ph and pl 

        We must use conditioned Add22 as logih can move over ph
     */

     Add22Cond(&logTabPolyh_inf, &logTabPolyl_inf, logih_inf, logim_inf, ph_inf, pl_inf);
     Add22Cond(&logTabPolyh_sup, &logTabPolyl_sup, logih_sup, logim_sup, ph_sup, pl_sup);

     /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

     Add22Cond(&logh_inf, &logm_inf, log2edh_inf, log2edl_inf, logTabPolyh_inf, logTabPolyl_inf);
     Add22Cond(&logh_sup, &logm_sup, log2edh_sup, log2edl_sup, logTabPolyh_sup, logTabPolyl_sup);

     /* Change logarithm base from natural base to base 2 by multiplying */

     Mul22(&logb2h_inf, &logb2m_inf, log2invh, log2invl, logh_inf, logm_inf);
     Mul22(&logb2h_sup, &logb2m_sup, log2invh, log2invl, logh_sup, logm_sup);

     /* Rounding test and eventual return or call to the accurate function */

     if(E_inf==0)
        roundcst_inf = RDROUNDCST1;
     else
        roundcst_inf = RDROUNDCST2;

     if(E_sup==0)
      roundcst_sup = RDROUNDCST1;
     else
        roundcst_sup = RDROUNDCST2;

     TEST_AND_COPY_RDRU_LOG2(roundable, restemp_inf,logb2h_inf, logb2m_inf, restemp_sup, logb2h_sup, logb2m_sup, roundcst_inf, roundcst_sup);


#if DEBUG
     printf("Going for Accurate Phase for x=%1.50e\n",x_inf);
#endif

     if((roundable==2) || (roundable==0))
     {
       log2_td_accurate(&logb2h_inf, &logb2m_inf, &logb2l_inf, E_inf, ed_inf, index_inf, zh_inf, zl_inf, logih_inf, logim_inf); 
       RoundDownwards3(&restemp_inf,logb2h_inf, logb2m_inf, logb2l_inf);
     }
     if((roundable==1) || (roundable==0))
     {
       log2_td_accurate(&logb2h_sup, &logb2m_sup, &logb2l_sup, E_sup, ed_sup, index_sup, zh_sup, zl_sup, logih_sup, logim_sup); 
       RoundUpwards3(&restemp_sup,logb2h_sup, logb2m_sup, logb2l_sup);
     }
     ASSIGN_LOW(res,restemp_inf);
     ASSIGN_UP(res,restemp_sup);
     return res;
   }
   if((infDone==0))
   {
     xdb_inf.i[HI] =  index_inf | 0x3ff00000;	/* do exponent = 0 */
     index_inf = (index_inf + (1<<(20-L-1))) >> (20-L);
 
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index_inf >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       xdb_inf.i[HI] -= 0x00100000; 
       E_inf++;
     }
     y_inf = xdb_inf.d;
     index_inf = index_inf & INDEXMASK;
     /* Cast integer E into double ed for multiplication later */
     ed_inf = (double) E_inf;
     /* 
        Read tables:
        Read one float for ri
        Read the first two doubles for -log(r_i) (out of three)
        Organization of the table:
        one struct entry per index, the struct entry containing 
        r, logih, logim and logil in this order
     */
   
     ri_inf = argredtable[index_inf].ri;

     /* 
        Actually we don't need the logarithm entries now
        Move the following two lines to the eventual reconstruction
        As long as we don't have any if in the following code, we can overlap 
        memory access with calculations 
     */
     logih_inf = argredtable[index_inf].logih;
     logim_inf = argredtable[index_inf].logim;

     /* Do range reduction:

        zh + zl = y * ri - 1.0 correctly

        Correctness is assured by use of Mul12 and Add12
        even if we don't force ri to have its' LSBs set to zero

        Discard zl for higher monome degrees
     */

     Mul12(&yrih_inf, &yril_inf, y_inf, ri_inf);
     th_inf = yrih_inf - 1.0; 
     Add12Cond(zh_inf, zl_inf, th_inf, yril_inf); 


     /* 
        Polynomial evaluation

        Use a 7 degree polynomial
        Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
        Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
        using an ad hoc method

     */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
     polyHorner_inf = FMA(FMA(FMA(FMA(c7_inf,zh_inf,c6),zh_inf,c5),zh_inf,c4),zh_inf,c3);
#else
     polyHorner_inf = c3 + zh_inf * (c4 + zh_inf * (c5 + zh_inf * (c6 + zh_inf * c7)));
#endif

     Mul12(&zhSquareh_inf, &zhSquarel_inf, zh_inf, zh_inf);
     polyUpper_inf = polyHorner_inf * (zh_inf * zhSquareh_inf);
     zhSquareHalfh_inf = zhSquareh_inf * -0.5;
     zhSquareHalfl_inf = zhSquarel_inf * -0.5;
     Add12(t1h_inf, t1l_inf, polyUpper_inf, -1 * (zh_inf * zl_inf));
     Add22(&t2h_inf, &t2l_inf, zh_inf, zl_inf, zhSquareHalfh_inf, zhSquareHalfl_inf);
     Add22(&ph_inf, &pl_inf, t2h_inf, t2l_inf, t1h_inf, t1l_inf);


     /* Reconstruction 

        Read logih and logim in the tables (already done)
      
        Compute log(x) = E * log(2) + log(1+z) - log(ri)
        i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

        Carry out everything in double double precision

     */
   
     /* 
        We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
        Multiplication of ed (double E) and log2h is thus correct
        The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
        is enough for the accurate phase
        The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
        Nevertheless the storage with trailing zeros implies an overlap of the tabulated
        triple double values. We have to take it into account for the accurate phase 
        basic procedures for addition and multiplication
        The condition on the next Add12 is verified as log2m is smaller than log2h 
        and both are scaled by ed
     */

     Add12(log2edh_inf, log2edl_inf, log2h * ed_inf, log2m * ed_inf);

     /* Add logih and logim to ph and pl 

        We must use conditioned Add22 as logih can move over ph
     */

     Add22Cond(&logTabPolyh_inf, &logTabPolyl_inf, logih_inf, logim_inf, ph_inf, pl_inf);

     /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

     Add22Cond(&logh_inf, &logm_inf, log2edh_inf, log2edl_inf, logTabPolyh_inf, logTabPolyl_inf);

     /* Change logarithm base from natural base to base 2 by multiplying */

     Mul22(&logb2h_inf, &logb2m_inf, log2invh, log2invl, logh_inf, logm_inf);

     /* Rounding test and eventual return or call to the accurate function */

     if(E_inf==0)
        roundcst_inf = RDROUNDCST1;
     else
        roundcst_inf = RDROUNDCST2;


     TEST_AND_COPY_RD(roundable, restemp_inf,logb2h_inf, logb2m_inf, roundcst_inf);


#if DEBUG
     printf("Going for Accurate Phase for x=%1.50e\n",x_inf);
#endif

     if((roundable==0))
     {
       log2_td_accurate(&logb2h_inf, &logb2m_inf, &logb2l_inf, E_inf, ed_inf, index_inf, zh_inf, zl_inf, logih_inf, logim_inf); 
       RoundDownwards3(&restemp_inf,logb2h_inf, logb2m_inf, logb2l_inf);
     }
     ASSIGN_LOW(res,restemp_inf);
     ASSIGN_UP(res,restemp_sup);
     return res;
   }
   if((supDone==0))
   {
     xdb_sup.i[HI] =  index_sup | 0x3ff00000;	/* do exponent = 0 */
     index_sup = (index_sup + (1<<(20-L-1))) >> (20-L);

 
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index_sup >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       xdb_sup.i[HI] -= 0x00100000; 
       E_sup++;
     }
     y_sup = xdb_sup.d;
     index_sup = index_sup & INDEXMASK;
     /* Cast integer E into double ed for multiplication later */
     ed_sup = (double) E_sup;
     /* 
        Read tables:
        Read one float for ri
        Read the first two doubles for -log(r_i) (out of three)

        Organization of the table:

        one struct entry per index, the struct entry containing 
        r, logih, logim and logil in this order
     */
   
     ri_sup = argredtable[index_sup].ri;

     /* 
        Actually we don't need the logarithm entries now
        Move the following two lines to the eventual reconstruction
        As long as we don't have any if in the following code, we can overlap 
        memory access with calculations 
     */
     logih_sup = argredtable[index_sup].logih;
     logim_sup = argredtable[index_sup].logim;

     /* Do range reduction:

        zh + zl = y * ri - 1.0 correctly

        Correctness is assured by use of Mul12 and Add12
        even if we don't force ri to have its' LSBs set to zero

        Discard zl for higher monome degrees
     */

     Mul12(&yrih_sup, &yril_sup, y_sup, ri_sup);
     th_sup = yrih_sup - 1.0; 
     Add12Cond(zh_sup, zl_sup, th_sup, yril_sup); 


     /* 
        Polynomial evaluation

        Use a 7 degree polynomial
        Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
        Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zl
        using an ad hoc method

     */



#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
     polyHorner_sup = FMA(FMA(FMA(FMA(c7,zh_sup,c6),zh_sup,c5),zh_sup,c4),zh_sup,c3);
#else
     polyHorner_sup = c3 + zh_sup * (c4 + zh_sup * (c5 + zh_sup * (c6 + zh_sup * c7)));
#endif

     Mul12(&zhSquareh_sup, &zhSquarel_sup, zh_sup, zh_sup);
     polyUpper_sup = polyHorner_sup * (zh_sup * zhSquareh_sup);
     zhSquareHalfh_sup = zhSquareh_sup * -0.5;
     zhSquareHalfl_sup = zhSquarel_sup * -0.5;
     Add12(t1h_sup, t1l_sup, polyUpper_sup, -1 * (zh_sup * zl_sup));
     Add22(&t2h_sup, &t2l_sup, zh_sup, zl_sup, zhSquareHalfh_sup, zhSquareHalfl_sup);
     Add22(&ph_sup, &pl_sup, t2h_sup, t2l_sup, t1h_sup, t1l_sup);


     /* Reconstruction 

        Read logih and logim in the tables (already done)
      
        Compute log(x) = E * log(2) + log(1+z) - log(ri)
        i.e. log(x) = ed * (log2h + log2m) + (ph + pl) + (logih + logim) + delta

        Carry out everything in double double precision

     */
   
     /* 
        We store log2 as log2h + log2m + log2l where log2h and log2m have 12 trailing zeros
        Multiplication of ed (double E) and log2h is thus correct
        The overall accuracy of log2h + log2m + log2l is 53 * 3 - 24 = 135 which
        is enough for the accurate phase
        The accuracy suffices also for the quick phase: 53 * 2 - 24 = 82
        Nevertheless the storage with trailing zeros implies an overlap of the tabulated
        triple double values. We have to take it into account for the accurate phase 
        basic procedures for addition and multiplication
        The condition on the next Add12 is verified as log2m is smaller than log2h 
        and both are scaled by ed
     */

     Add12(log2edh_sup, log2edl_sup, log2h * ed_sup, log2m * ed_sup);

     /* Add logih and logim to ph and pl 

        We must use conditioned Add22 as logih can move over ph
     */

     Add22Cond(&logTabPolyh_sup, &logTabPolyl_sup, logih_sup, logim_sup, ph_sup, pl_sup);

     /* Add log2edh + log2edl to logTabPolyh + logTabPolyl */

     Add22Cond(&logh_sup, &logm_sup, log2edh_sup, log2edl_sup, logTabPolyh_sup, logTabPolyl_sup);

     /* Change logarithm base from natural base to base 2 by multiplying */

     Mul22(&logb2h_sup, &logb2m_sup, log2invh, log2invl, logh_sup, logm_sup);

     /* Rounding test and eventual return or call to the accurate function */

     if(E_sup==0)
      roundcst_sup = RDROUNDCST1;
     else
        roundcst_sup = RDROUNDCST2;

     TEST_AND_COPY_RU(roundable, restemp_sup,logb2h_sup, logb2m_sup, roundcst_sup);


#if DEBUG
     printf("Going for Accurate Phase for x=%1.50e\n",x_inf);
#endif

     if((roundable==0))
     {
       log2_td_accurate(&logb2h_sup, &logb2m_sup, &logb2l_sup, E_sup, ed_sup, index_sup, zh_sup, zl_sup, logih_sup, logim_sup); 
       RoundUpwards3(&restemp_sup,logb2h_sup, logb2m_sup, logb2l_sup);
     }
     ASSIGN_LOW(res,restemp_inf);
     ASSIGN_UP(res,restemp_sup);
     return res;
   }

 } 
#endif

