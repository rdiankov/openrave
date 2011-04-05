/* 
 * This function computes log10, correctly rounded, 
 * using experimental techniques based on triple double arithmetics

 THIS IS EXPERIMENTAL SOFTWARE
 
 *
 * Author :  Christoph Lauter
 * christoph.lauter at ens-lyon.fr
 *

 To have it replace the crlibm log10, do:

 gcc -DHAVE_CONFIG_H -I.  -fPIC  -O2 -c log10-td.c;   mv log10-td.o log10_accurate.o; make 


 **********************************************************
 *
 *  NOTE: THIS FUNCTION USES SOME BASIC SEQUENCES AND
 *  FURTHER FINAL ROUNDING SEQUENCES WITH SPECIAL CASE
 *  TESTS NO FORMAL PROOF IS AVAILABLE FOR IN THE MOMENT
 *
 **********************************************************

*/


#include <stdio.h>
#include <stdlib.h>
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "log10-td.h"

#define AVOID_FMA 0


void log10_td_accurate(double *logb10h, double *logb10m, double *logb10l, int E, double ed, int index, double zh, double zl, double logih, double logim) {
  double highPoly, t1h, t1l, t2h, t2l, t3h, t3l, t4h, t4l, t5h, t5l, t6h, t6l, t7h, t7l, t8h, t8l, t9h, t9l, t10h, t10l, t11h, t11l;
  double t12h, t12l, t13h, t13l, t14h, t14l, zSquareh, zSquarem, zSquarel, zCubeh, zCubem, zCubel, higherPolyMultZh, higherPolyMultZm;
  double higherPolyMultZl, zSquareHalfh, zSquareHalfm, zSquareHalfl, polyWithSquareh, polyWithSquarem, polyWithSquarel;
  double polyh, polym, polyl, logil, logyh, logym, logyl, loghover, logmover, loglover, log2edhover, log2edmover, log2edlover;
  double log2edh, log2edm, log2edl, logb10hover, logb10mover, logb10lover;
  double logyhnorm, logymnorm, logylnorm;





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


  /* Renormalize logyh, logym and logyl to a non-overlapping triple-double for winning some 
     accuracy in the final ln(x) result before multiplying with log10inv 

     THIS MAY NOT BE NECESSARY NOR SUFFICIENT

  */

  Renormalize3(&logyhnorm, &logymnorm, &logylnorm, logyh, logym, logyl);


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

  Add33(&loghover, &logmover, &loglover, log2edh, log2edm, log2edl, logyhnorm, logymnorm, logylnorm);


  /* Change logarithm base from natural base to base 10 by multiplying */

  Mul33(&logb10hover, &logb10mover, &logb10lover, log10invh, log10invm, log10invl, loghover, logmover, loglover);


  /* Since we can not guarantee in each addition and multiplication procedure that 
     the results are not overlapping, we must renormalize the result before handing
     it over to the final rounding
  */

  Renormalize3(logb10h,logb10m,logb10l,logb10hover,logb10mover,logb10lover);
}



/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log10_rn(double x){ 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb10h, logb10m, logb10l;
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



   /* Change logarithm base from natural base to base 10 by multiplying */

   Mul22(&logb10h, &logb10m, log10invh, log10invm, logh, logm);


   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = ROUNDCST1;
   else
      roundcst = ROUNDCST2;


   if(logb10h == (logb10h + (logb10m * roundcst)))
     return logb10h;
   else 
     {
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log10_td_accurate(&logb10h, &logb10m, &logb10l, E, ed, index, zh, zl, logih, logim); 
       
       ReturnRoundToNearest3(logb10h, logb10m, logb10l);

     } /* Accurate phase launched */
 }


/*************************************************************
 *************************************************************
 *               ROUNDED  UPWARDS			     *
 *************************************************************
 *************************************************************/
 double log10_ru(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb10h, logb10m, logb10l;
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



   /* Change logarithm base from natural base to base 10 by multiplying */

   Mul22(&logb10h, &logb10m, log10invh, log10invm, logh, logm);


   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RU(logb10h, logb10m, roundcst);

#if DEBUG  
   printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log10_td_accurate(&logb10h, &logb10m, &logb10l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundUpwards3Unfiltered(logb10h, logb10m, logb10l, WORSTCASEACCURACY);

 } 


/*************************************************************
 *************************************************************
 *               ROUNDED  DOWNWARDS			     *
 *************************************************************
 *************************************************************/
 double log10_rd(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb10h, logb10m, logb10l;
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



   /* Change logarithm base from natural base to base 10 by multiplying */

   Mul22(&logb10h, &logb10m, log10invh, log10invm, logh, logm);


   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RD(logb10h, logb10m, roundcst);

#if DEBUG
  printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log10_td_accurate(&logb10h, &logb10m, &logb10l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundDownwards3Unfiltered(logb10h, logb10m, logb10l, WORSTCASEACCURACY);
 } 

/*************************************************************
 *************************************************************
 *               ROUNDED  TOWARDS ZERO			     *
 *************************************************************
 *************************************************************/
 double log10_rz(double x) { 
   db_number xdb;
   double y, ed, ri, logih, logim, yrih, yril, th, zh, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, roundcst;
   double logb10h, logb10m, logb10l;
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



   /* Change logarithm base from natural base to base 10 by multiplying */

   Mul22(&logb10h, &logb10m, log10invh, log10invm, logh, logm);


   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RZ(logb10h, logb10m, roundcst);

#if DEBUG
  printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

    log10_td_accurate(&logb10h, &logb10m, &logb10l, E, ed, index, zh, zl, logih, logim); 

    ReturnRoundTowardsZero3Unfiltered(logb10h, logb10m, logb10l, WORSTCASEACCURACY);
 } 



