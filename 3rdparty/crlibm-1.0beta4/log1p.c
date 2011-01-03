/*
 * Correctly rounded log1p(x) = log(1 + x)
 *
 * Author : Christoph Lauter (ENS Lyon)
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
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "log-td.h"

#define AVOID_FMA 0



void log1p_td_accurate(double *logh, double *logm, double *logl, double ed, int index, 
		       double zh, double zm, double zl, double logih, double logim) {
  double highPoly, t1h, t1l, t2h, t2l, t3h, t3l, t4h, t4l, t5h, t5l, t6h, t6l, t7h, t7l, t8h, t8l, t9h, t9l, t10h, t10l, t11h, t11l;
  double t12h, t12l, t13h, t13l, t14h, t14l, zSquareh, zSquarem, zSquarel, zCubeh, zCubem, zCubel, higherPolyMultZh, higherPolyMultZm;
  double higherPolyMultZl, zSquareHalfh, zSquareHalfm, zSquareHalfl, polyWithSquareh, polyWithSquarem, polyWithSquarel;
  double polyh, polym, polyl, logil, logyh, logym, logyl, loghover, logmover, loglover, log2edhover, log2edmover, log2edlover;
  double log2edh, log2edm, log2edl;


#if EVAL_PERF
  crlibm_second_step_taken++;
#endif


  /* Accurate phase:

     Argument reduction is already done. 
     We must return logh, logm and logl representing the intermediate result in 118 bits precision.

     We use a 14 degree polynomial, computing the first 3 (the first is 0) coefficients in triple double,
     calculating the next 7 coefficients in double double arithmetics and the last in double.

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
  Mul22(&t3h, &t3l, zh, zm, t2h, t2l);
  Add22(&t4h, &t4l, accPolyC8h, accPolyC8l, t3h, t3l);
  Mul22(&t5h, &t5l, zh, zm, t4h, t4l);
  Add22(&t6h, &t6l, accPolyC7h, accPolyC7l, t5h, t5l);
  Mul22(&t7h, &t7l, zh, zm, t6h, t6l);
  Add22(&t8h, &t8l, accPolyC6h, accPolyC6l, t7h, t7l);
  Mul22(&t9h, &t9l, zh, zm, t8h, t8l);
  Add22(&t10h, &t10l, accPolyC5h, accPolyC5l, t9h, t9l);
  Mul22(&t11h, &t11l, zh, zm, t10h, t10l);
  Add22(&t12h, &t12l, accPolyC4h, accPolyC4l, t11h, t11l);
  Mul22(&t13h, &t13l, zh, zm, t12h, t12l);
  Add22(&t14h, &t14l, accPolyC3h, accPolyC3l, t13h, t13l);

  /* We must now prepare (zh + zm)^2 and (zh + zm)^3 as triple doubles */

  Mul33(&zSquareh, &zSquarem, &zSquarel, zh, zm, zl, zh, zm, zl); 
  Mul33(&zCubeh, &zCubem, &zCubel, zh, zm, zl, zSquareh, zSquarem, zSquarel); 
  
  /* We can now multiplicate the middle and higher polynomial by z^3 */

  Mul233(&higherPolyMultZh, &higherPolyMultZm, &higherPolyMultZl, t14h, t14l, zCubeh, zCubem, zCubel);
  
  /* Multiply now z^2 by -1/2 (exact op) and add to middle and higher polynomial */
  
  zSquareHalfh = zSquareh * -0.5;
  zSquareHalfm = zSquarem * -0.5;
  zSquareHalfl = zSquarel * -0.5;

  Add33(&polyWithSquareh, &polyWithSquarem, &polyWithSquarel, 
	zSquareHalfh, zSquareHalfm, zSquareHalfl, 
	higherPolyMultZh, higherPolyMultZm, higherPolyMultZl);

  /* Add now zh and zm to obtain the polynomial evaluation result */

  Add33(&polyh, &polym, &polyl, zh, zm, zl, polyWithSquareh, polyWithSquarem, polyWithSquarel);

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

  /* Since we can not guarantee in each addition and multiplication procedure that 
     the results are not overlapping, we must renormalize the result before handing
     it over to the final rounding
  */

  Renormalize3(logh,logm,logl,loghover,logmover,loglover);

}



/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
 double log1p_rn(double x){ 
   db_number xdb, shdb, scaledb;
   double yh, yl, ed, ri, logih, logim, yhrih, yhril, ylri, t1, t2, t3, t4, t5, t6, zh, zm, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, logl, roundcst;
   double sh, sl;
   int E, index;


   xdb.d=x;

   /* Filter cases */
   if ((xdb.i[HI] & 0x7fffffff) < 0x3c900000) {
     /* We are less than 2^(-54) and return simply an adjusted x 
	This captures also the algebraic case x = 0
     */
     return x;
   }

   if (((xdb.i[HI] & 0x80000000) != 0) && ((xdb.i[HI] & 0x7fffffff) >= 0x3ff00000)) {
     /* We are less or equal than -1 (-inf and NaN, too), 
	we return -inf for -1 and NaN otherwise 
     */
     if (x == -1.0) return x/0.0;

     
     return (x-x)/0.0;
   }

   if ((xdb.i[HI] & 0x7ff00000) == 0x7ff00000) {
     /* We are +inf or NaN 
	If +inf, we return +inf (x+x)
	If NaN, we return NaN (x+x)
     */
     return x+x;
   }

   /* Test if |x| < 2^(-8)
      
   If yes, short-circuit the range reduction 
   
   */
   
   if ((xdb.i[HI] & 0x7fffffff) < 0x3f700000) {
     /* Use the polynomial p(zh + zl) approximating log(1+zh+zl) directly 
	Set E and index to values that read 0.0 in the accurate phase.
     */
     logih = 0.0;
     logim = 0.0;
     index = 0;
     ed = 0.0;
     index = 0;
     zh = x;
     zm = 0.0;
     zl = 0.0;
   } else {
     /* If we are here, |x| >= 2^(-8) and we must perform range reduction */
     
     /* Compute first exactly
	
        sh + sl = 1 + x 
     
        x can move over 1, so use a conditional Add12
     */
     
     Add12Cond(sh,sl,1.0,x);
     
     /* Transform higher order double to integer */
     
     shdb.d = sh;

     /* Extract exponent and mantissa 
	Do range reduction,
	yielding to E holding the exponent and
	y the mantissa between sqrt(2)/2 and sqrt(2)
     */
     E = 0;
     E += (shdb.i[HI]>>20)-1023;             /* extract the exponent */
     index = (shdb.i[HI] & 0x000fffff);
     shdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
     index = (index + (1<<(20-L-1))) >> (20-L);
     
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       shdb.i[HI] -= 0x00100000; 
       E++;
     }
     
     
     /* Transform shdb to yh */
     yh = shdb.d;
     
    
     /* Compute the index to the table */
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
     
     /* Test if we have a simple range reduction or a complicated one 
	
        Simple range reduction for x < 0: x + 1 is exact, sl = 0 exactly
        Simple range reduction for x > 2^(125) (sh > 2^(125)): x + 1 is not exact but its error less than 2^(-125)
     
	Complicated range reduction: other cases 
	
     */

     
     if ((sl == 0.0) || (E > 125)) {
       /* Simple range reduction */
       
       Mul12(&yhrih, &yhril, yh, ri);
       t1 = yhrih - 1.0; 
       Add12Cond(zh, zm, t1, yhril); 
       zl = 0.0;
       
     } else {
       /* Complicated range reduction; E <= 125 */
       
       
       /* Scale sl accordingly to sh, from which the exponent was extracted 
	  
          We form first 2^(-E) and multiply sl with this value; this gives yl.
       */
       
       scaledb.i[HI] = (-E + 1023) << 20;
       scaledb.i[LO] = 0;
       
       yl = sl * scaledb.d;
       
       
       /* Do complicated range reduction:
	
          zh + zm + zl = (yh + yl) * ri - 1.0 


	  We use zh + zm in the quick phase and zh + zm + zl in the accurate phase
	  
	  The multiplication yl * ri is exact because yl contains at most 9 bits and
	  ri contains at most 24 bits.
	  
	  The substraction yhrih - 1.0 is exact as per Sterbenz' lemma.

       */
     
       Mul12(&yhrih,&yhril,yh,ri);
       ylri = yl * ri;
       
       t1 = yhrih - 1.0;
       
       /* The unnormalized triple-double t1 + yhril + ylri is equal to (yh + yl) * ri - 1.0 
	  As t1 can move over yhril and yhri can move over ylri, we normalize first these
	  values pairwise with Add12Conds. Then we renormalize the pairs by a 
	  "inverted" (A.E.) Renormalize3.
       */
       
       Add12Cond(t2,t3,yhril,ylri);
       Add12Cond(t4,t5,t1,t2);
       
       Add12Cond(t6,zl,t3,t5);
       Add12Cond(zh,zm,t4,t6);
       
     }
   }
   
   
   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zm
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
   Add12(t1h, t1l, polyUpper, -1 * (zh * zm));
   Add22(&t2h, &t2l, zh, zm, zhSquareHalfh, zhSquareHalfl);
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

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = ROUNDCST1;
   else
      roundcst = ROUNDCST2;


   if(logh == (logh + (logm * roundcst)))
     return logh;
   else 
     {
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log1p_td_accurate(&logh, &logm, &logl, ed, index, zh, zm, zl, logih, logim); 
       
       ReturnRoundToNearest3(logh, logm, logl);

     } /* Accurate phase launched */
 }




double log1p_ru(double x) {
   db_number xdb, shdb, scaledb;
   double yh, yl, ed, ri, logih, logim, yhrih, yhril, ylri, t1, t2, t3, t4, t5, t6, zh, zm, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, logl, roundcst;
   double sh, sl;
   int E, index;


   xdb.d=x;

   /* Filter cases */
   if ((xdb.i[HI] & 0x7fffffff) < 0x3c900000) {
     /* We are less than 2^(-54) and return simply an adjusted x 
	
        If x = 0, the result is algebraic and equal to 0.
	
	The series for log(1 + x) = x - 1/2 * x^2 + ... is alternated
	and converges in this interval. 
	The truncation rest -1/2 * x^2 + 1/3 * x^3 - ... is
	always negative, so log(1 + x) is always less than x but less than
	1 ulp of x away.
	We round up, so we return x.

     */
     return x;
   }

   if (((xdb.i[HI] & 0x80000000) != 0) && ((xdb.i[HI] & 0x7fffffff) >= 0x3ff00000)) {
     /* We are less or equal than -1 (-inf and NaN, too), 
	we return -inf for -1 and NaN otherwise 
     */
     if (x == -1.0) return x/0.0;

     
     return (x-x)/0.0;
   }

   if ((xdb.i[HI] & 0x7ff00000) == 0x7ff00000) {
     /* We are +inf or NaN 
	If +inf, we return +inf (x+x)
	If NaN, we return NaN (x+x)
     */
     return x+x;
   }

   /* Test if |x| < 2^(-8)
      
   If yes, short-circuit the range reduction 
   
   */
   
   if ((xdb.i[HI] & 0x7fffffff) < 0x3f700000) {
     /* Use the polynomial p(zh + zl) approximating log(1+zh+zl) directly 
	Set E and index to values that read 0.0 in the accurate phase.
     */
     logih = 0.0;
     logim = 0.0;
     index = 0;
     ed = 0.0;
     index = 0;
     zh = x;
     zm = 0.0;
     zl = 0.0;
   } else {
     /* If we are here, |x| >= 2^(-8) and we must perform range reduction */
     
     /* Compute first exactly
	
        sh + sl = 1 + x 
     
        x can move over 1, so use a conditional Add12
     */
     
     Add12Cond(sh,sl,1.0,x);
     
     /* Transform higher order double to integer */
     
     shdb.d = sh;

     /* Extract exponent and mantissa 
	Do range reduction,
	yielding to E holding the exponent and
	y the mantissa between sqrt(2)/2 and sqrt(2)
     */
     E = 0;
     E += (shdb.i[HI]>>20)-1023;             /* extract the exponent */
     index = (shdb.i[HI] & 0x000fffff);
     shdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
     index = (index + (1<<(20-L-1))) >> (20-L);
     
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       shdb.i[HI] -= 0x00100000; 
       E++;
     }
     
     
     /* Transform shdb to yh */
     yh = shdb.d;
     
    
     /* Compute the index to the table */
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
     
     /* Test if we have a simple range reduction or a complicated one 
	
        Simple range reduction for x < 0: x + 1 is exact, sl = 0 exactly
        Simple range reduction for x > 2^(125) (sh > 2^(125)): x + 1 is not exact but its error less than 2^(-125)
     
	Complicated range reduction: other cases 
	
     */

     
     if ((sl == 0.0) || (E > 125)) {
       /* Simple range reduction */
       
       Mul12(&yhrih, &yhril, yh, ri);
       t1 = yhrih - 1.0; 
       Add12Cond(zh, zm, t1, yhril); 
       zl = 0.0;
       
     } else {
       /* Complicated range reduction; E <= 125 */
       
       
       /* Scale sl accordingly to sh, from which the exponent was extracted 
	  
          We form first 2^(-E) and multiply sl with this value; this gives yl.
       */
       
       scaledb.i[HI] = (-E + 1023) << 20;
       scaledb.i[LO] = 0;
       
       yl = sl * scaledb.d;
       
       
       /* Do complicated range reduction:
	
          zh + zm + zl = (yh + yl) * ri - 1.0 


	  We use zh + zm in the quick phase and zh + zm + zl in the accurate phase
	  
	  The multiplication yl * ri is exact because yl contains at most 9 bits and
	  ri contains at most 24 bits.
	  
	  The substraction yhrih - 1.0 is exact as per Sterbenz' lemma.

       */
     
       Mul12(&yhrih,&yhril,yh,ri);
       ylri = yl * ri;
       
       t1 = yhrih - 1.0;
       
       /* The unnormalized triple-double t1 + yhril + ylri is equal to (yh + yl) * ri - 1.0 
	  As t1 can move over yhril and yhri can move over ylri, we normalize first these
	  values pairwise with Add12Conds. Then we renormalize the pairs by a 
	  "inverted" (A.E.) Renormalize3.
       */
       
       Add12Cond(t2,t3,yhril,ylri);
       Add12Cond(t4,t5,t1,t2);
       
       Add12Cond(t6,zl,t3,t5);
       Add12Cond(zh,zm,t4,t6);
       
     }
   }
   
   
   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zm
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
   Add12(t1h, t1l, polyUpper, -1 * (zh * zm));
   Add22(&t2h, &t2l, zh, zm, zhSquareHalfh, zhSquareHalfl);
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

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RU(logh, logm, roundcst);
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log1p_td_accurate(&logh, &logm, &logl, ed, index, zh, zm, zl, logih, logim); 
       
       ReturnRoundUpwards3(logh, logm, logl);
}

double log1p_rd(double x) {
   db_number xdb, shdb, scaledb;
   double yh, yl, ed, ri, logih, logim, yhrih, yhril, ylri, t1, t2, t3, t4, t5, t6, zh, zm, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, logl, roundcst;
   double sh, sl;
   int E, index;


   xdb.d=x;

   /* Filter cases */
   if ((xdb.i[HI] & 0x7fffffff) < 0x3c900000) {
     /* We are less than 2^(-54) and return simply an adjusted x 
	
        If x = 0, the result is algebraic and equal to 0.
	
	The series for log(1 + x) = x - 1/2 * x^2 + ... is alternated
	and converges in this interval. 
	The truncation rest -1/2 * x^2 + 1/3 * x^3 - ... is
	always negative, so log(1 + x) is always less than x but less than
	1 ulp of x away.
	We round down, so we return x - 1ulp;
	
     */

     if (x == 0.0) return x;

     if (x > 0) {
       xdb.l--;
     } else {
       xdb.l++;
     }
     return xdb.d;
   }

   if (((xdb.i[HI] & 0x80000000) != 0) && ((xdb.i[HI] & 0x7fffffff) >= 0x3ff00000)) {
     /* We are less or equal than -1 (-inf and NaN, too), 
	we return -inf for -1 and NaN otherwise 
     */
     if (x == -1.0) return x/0.0;

     
     return (x-x)/0.0;
   }

   if ((xdb.i[HI] & 0x7ff00000) == 0x7ff00000) {
     /* We are +inf or NaN 
	If +inf, we return +inf (x+x)
	If NaN, we return NaN (x+x)
     */
     return x+x;
   }

   /* Test if |x| < 2^(-8)
      
   If yes, short-circuit the range reduction 
   
   */
   
   if ((xdb.i[HI] & 0x7fffffff) < 0x3f700000) {
     /* Use the polynomial p(zh + zl) approximating log(1+zh+zl) directly 
	Set E and index to values that read 0.0 in the accurate phase.
     */
     logih = 0.0;
     logim = 0.0;
     index = 0;
     ed = 0.0;
     index = 0;
     zh = x;
     zm = 0.0;
     zl = 0.0;
   } else {
     /* If we are here, |x| >= 2^(-8) and we must perform range reduction */
     
     /* Compute first exactly
	
        sh + sl = 1 + x 
     
        x can move over 1, so use a conditional Add12
     */
     
     Add12Cond(sh,sl,1.0,x);
     
     /* Transform higher order double to integer */
     
     shdb.d = sh;

     /* Extract exponent and mantissa 
	Do range reduction,
	yielding to E holding the exponent and
	y the mantissa between sqrt(2)/2 and sqrt(2)
     */
     E = 0;
     E += (shdb.i[HI]>>20)-1023;             /* extract the exponent */
     index = (shdb.i[HI] & 0x000fffff);
     shdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
     index = (index + (1<<(20-L-1))) >> (20-L);
     
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       shdb.i[HI] -= 0x00100000; 
       E++;
     }
     
     
     /* Transform shdb to yh */
     yh = shdb.d;
     
    
     /* Compute the index to the table */
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
     
     /* Test if we have a simple range reduction or a complicated one 
	
        Simple range reduction for x < 0: x + 1 is exact, sl = 0 exactly
        Simple range reduction for x > 2^(125) (sh > 2^(125)): x + 1 is not exact but its error less than 2^(-125)
     
	Complicated range reduction: other cases 
	
     */

     
     if ((sl == 0.0) || (E > 125)) {
       /* Simple range reduction */
       
       Mul12(&yhrih, &yhril, yh, ri);
       t1 = yhrih - 1.0; 
       Add12Cond(zh, zm, t1, yhril); 
       zl = 0.0;
       
     } else {
       /* Complicated range reduction; E <= 125 */
       
       
       /* Scale sl accordingly to sh, from which the exponent was extracted 
	  
          We form first 2^(-E) and multiply sl with this value; this gives yl.
       */
       
       scaledb.i[HI] = (-E + 1023) << 20;
       scaledb.i[LO] = 0;
       
       yl = sl * scaledb.d;
       
       
       /* Do complicated range reduction:
	
          zh + zm + zl = (yh + yl) * ri - 1.0 


	  We use zh + zm in the quick phase and zh + zm + zl in the accurate phase
	  
	  The multiplication yl * ri is exact because yl contains at most 9 bits and
	  ri contains at most 24 bits.
	  
	  The substraction yhrih - 1.0 is exact as per Sterbenz' lemma.

       */
     
       Mul12(&yhrih,&yhril,yh,ri);
       ylri = yl * ri;
       
       t1 = yhrih - 1.0;
       
       /* The unnormalized triple-double t1 + yhril + ylri is equal to (yh + yl) * ri - 1.0 
	  As t1 can move over yhril and yhri can move over ylri, we normalize first these
	  values pairwise with Add12Conds. Then we renormalize the pairs by a 
	  "inverted" (A.E.) Renormalize3.
       */
       
       Add12Cond(t2,t3,yhril,ylri);
       Add12Cond(t4,t5,t1,t2);
       
       Add12Cond(t6,zl,t3,t5);
       Add12Cond(zh,zm,t4,t6);
       
     }
   }
   
   
   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zm
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
   Add12(t1h, t1l, polyUpper, -1 * (zh * zm));
   Add22(&t2h, &t2l, zh, zm, zhSquareHalfh, zhSquareHalfl);
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

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RD(logh, logm, roundcst);
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log1p_td_accurate(&logh, &logm, &logl, ed, index, zh, zm, zl, logih, logim); 
       
       ReturnRoundDownwards3(logh, logm, logl);
}

double log1p_rz(double x) {
   db_number xdb, shdb, scaledb;
   double yh, yl, ed, ri, logih, logim, yhrih, yhril, ylri, t1, t2, t3, t4, t5, t6, zh, zm, zl;
   double polyHorner, zhSquareh, zhSquarel, polyUpper, zhSquareHalfh, zhSquareHalfl;
   double t1h, t1l, t2h, t2l, ph, pl, log2edh, log2edl, logTabPolyh, logTabPolyl, logh, logm, logl, roundcst;
   double sh, sl;
   int E, index;


   xdb.d=x;

   /* Filter cases */
   if ((xdb.i[HI] & 0x7fffffff) < 0x3c900000) {
     /* We are less than 2^(-54) and return simply an adjusted x 
	
        If x = 0, the result is algebraic and equal to 0.
	
	The series for log(1 + x) = x - 1/2 * x^2 + ... is alternated
	and converges in this interval. 
	The truncation rest -1/2 * x^2 + 1/3 * x^3 - ... is
	always negative, so log(1 + x) is always less than x but less than
	1 ulp of x away.
	For x < 0, we have log(1 + x) < 0, so we round up and return x;
	For x > 0, we round down and return x - 1ulp

     */
     if (x > 0) {
       xdb.l--;
       return xdb.d;
     }

     /* Algebraic case x == 0.0 and round up */

     return x;
   }

   if (((xdb.i[HI] & 0x80000000) != 0) && ((xdb.i[HI] & 0x7fffffff) >= 0x3ff00000)) {
     /* We are less or equal than -1 (-inf and NaN, too), 
	we return -inf for -1 and NaN otherwise 
     */
     if (x == -1.0) return x/0.0;

     
     return (x-x)/0.0;
   }

   if ((xdb.i[HI] & 0x7ff00000) == 0x7ff00000) {
     /* We are +inf or NaN 
	If +inf, we return +inf (x+x)
	If NaN, we return NaN (x+x)
     */
     return x+x;
   }

   /* Test if |x| < 2^(-8)
      
   If yes, short-circuit the range reduction 
   
   */
   
   if ((xdb.i[HI] & 0x7fffffff) < 0x3f700000) {
     /* Use the polynomial p(zh + zl) approximating log(1+zh+zl) directly 
	Set E and index to values that read 0.0 in the accurate phase.
     */
     logih = 0.0;
     logim = 0.0;
     index = 0;
     ed = 0.0;
     index = 0;
     zh = x;
     zm = 0.0;
     zl = 0.0;
   } else {
     /* If we are here, |x| >= 2^(-8) and we must perform range reduction */
     
     /* Compute first exactly
	
        sh + sl = 1 + x 
     
        x can move over 1, so use a conditional Add12
     */
     
     Add12Cond(sh,sl,1.0,x);
     
     /* Transform higher order double to integer */
     
     shdb.d = sh;

     /* Extract exponent and mantissa 
	Do range reduction,
	yielding to E holding the exponent and
	y the mantissa between sqrt(2)/2 and sqrt(2)
     */
     E = 0;
     E += (shdb.i[HI]>>20)-1023;             /* extract the exponent */
     index = (shdb.i[HI] & 0x000fffff);
     shdb.i[HI] =  index | 0x3ff00000;	/* do exponent = 0 */
     index = (index + (1<<(20-L-1))) >> (20-L);
     
     /* reduce  such that sqrt(2)/2 < xdb.d < sqrt(2) */
     if (index >= MAXINDEX){ /* corresponds to xdb>sqrt(2)*/
       shdb.i[HI] -= 0x00100000; 
       E++;
     }
     
     
     /* Transform shdb to yh */
     yh = shdb.d;
     
    
     /* Compute the index to the table */
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
     
     /* Test if we have a simple range reduction or a complicated one 
	
        Simple range reduction for x < 0: x + 1 is exact, sl = 0 exactly
        Simple range reduction for x > 2^(125) (sh > 2^(125)): x + 1 is not exact but its error less than 2^(-125)
     
	Complicated range reduction: other cases 
	
     */

     
     if ((sl == 0.0) || (E > 125)) {
       /* Simple range reduction */
       
       Mul12(&yhrih, &yhril, yh, ri);
       t1 = yhrih - 1.0; 
       Add12Cond(zh, zm, t1, yhril); 
       zl = 0.0;
       
     } else {
       /* Complicated range reduction; E <= 125 */
       
       
       /* Scale sl accordingly to sh, from which the exponent was extracted 
	  
          We form first 2^(-E) and multiply sl with this value; this gives yl.
       */
       
       scaledb.i[HI] = (-E + 1023) << 20;
       scaledb.i[LO] = 0;
       
       yl = sl * scaledb.d;
       
       
       /* Do complicated range reduction:
	
          zh + zm + zl = (yh + yl) * ri - 1.0 


	  We use zh + zm in the quick phase and zh + zm + zl in the accurate phase
	  
	  The multiplication yl * ri is exact because yl contains at most 9 bits and
	  ri contains at most 24 bits.
	  
	  The substraction yhrih - 1.0 is exact as per Sterbenz' lemma.

       */
     
       Mul12(&yhrih,&yhril,yh,ri);
       ylri = yl * ri;
       
       t1 = yhrih - 1.0;
       
       /* The unnormalized triple-double t1 + yhril + ylri is equal to (yh + yl) * ri - 1.0 
	  As t1 can move over yhril and yhri can move over ylri, we normalize first these
	  values pairwise with Add12Conds. Then we renormalize the pairs by a 
	  "inverted" (A.E.) Renormalize3.
       */
       
       Add12Cond(t2,t3,yhril,ylri);
       Add12Cond(t4,t5,t1,t2);
       
       Add12Cond(t6,zl,t3,t5);
       Add12Cond(zh,zm,t4,t6);
       
     }
   }
   
   
   /* 
      Polynomial evaluation

      Use a 7 degree polynomial
      Evaluate the higher 5 terms in double precision (-7 * 3 = -21) using Horner's scheme
      Evaluate the lower 3 terms (the last is 0) in double double precision accounting also for zm
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
   Add12(t1h, t1l, polyUpper, -1 * (zh * zm));
   Add22(&t2h, &t2l, zh, zm, zhSquareHalfh, zhSquareHalfl);
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

   /* Rounding test and eventual return or call to the accurate function */

   if(E==0)
      roundcst = RDROUNDCST1;
   else
      roundcst = RDROUNDCST2;

   TEST_AND_RETURN_RZ(logh, logm, roundcst);
       
#if DEBUG
       printf("Going for Accurate Phase for x=%1.50e\n",x);
#endif

       log1p_td_accurate(&logh, &logm, &logl, ed, index, zh, zm, zl, logih, logim); 
       
       ReturnRoundTowardsZero3(logh, logm, logl);
}
