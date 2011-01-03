/* 
 * This function computes exp, correctly rounded, 
 * using experimental techniques based on triple double arithmetics

 THIS IS EXPERIMENTAL SOFTWARE
 
 *
 * Author :  Christoph Lauter
 * christoph.lauter at ens-lyon.fr
 *

 To have it replace the crlibm exp, do:

 gcc -DHAVE_CONFIG_H -I.  -fPIC  -O2 -c exp-td.c;   mv exp-td.o exp_fast.o; make 
 
*/


#include <stdio.h>
#include <stdlib.h>
#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "exp-td.h"
#ifdef BUILD_INTERVAL_FUNCTIONS
#include "interval.h"
#endif

#define AVOID_FMA 0
#define EVAL_PERF 1







void exp_td_accurate(double *polyTblh, double *polyTblm, double *polyTbll, 
		     double rh, double rm, double rl, 
		     double tbl1h, double tbl1m, double tbl1l,
		     double tbl2h, double tbl2m, double tbl2l) {
  double highPoly, highPolyMulth, highPolyMultm, highPolyMultl;
  double rhSquareh, rhSquarel, rhSquareHalfh, rhSquareHalfl;
  double rhCubeh, rhCubem, rhCubel;
  double t1h, t1l, t2h, t2l, t3h, t3l, t4h, t4l, t5, t6;
  double lowPolyh, lowPolym, lowPolyl;
  double ph, pm, pl, phnorm, pmnorm, rmlMultPh, rmlMultPl;
  double qh, ql, fullPolyh, fullPolym, fullPolyl;
  double polyWithTbl1h, polyWithTbl1m, polyWithTbl1l;
  double polyAddOneh,polyAddOnem,polyAddOnel;
  double polyWithTablesh, polyWithTablesm, polyWithTablesl;


#if EVAL_PERF
  crlibm_second_step_taken++;
#endif

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
  highPoly = FMA(FMA(accPolyC7,rh,accPolyC6),rh,accPolyC5);
#else
  highPoly = accPolyC5 + rh * (accPolyC6 + rh * accPolyC7);
#endif

  Mul12(&t1h,&t1l,rh,highPoly);
  Add22(&t2h,&t2l,accPolyC4h,accPolyC4l,t1h,t1l);
  Mul22(&t3h,&t3l,rh,0,t2h,t2l);
  Add22(&t4h,&t4l,accPolyC3h,accPolyC3l,t3h,t3l);

  Mul12(&rhSquareh,&rhSquarel,rh,rh);
  Mul23(&rhCubeh,&rhCubem,&rhCubel,rh,0,rhSquareh,rhSquarel);

  rhSquareHalfh = 0.5 * rhSquareh;
  rhSquareHalfl = 0.5 * rhSquarel;  

  Renormalize3(&lowPolyh,&lowPolym,&lowPolyl,rh,rhSquareHalfh,rhSquareHalfl);

  Mul233(&highPolyMulth,&highPolyMultm,&highPolyMultl,t4h,t4l,rhCubeh,rhCubem,rhCubel);

  Add33(&ph,&pm,&pl,lowPolyh,lowPolym,lowPolyl,highPolyMulth,highPolyMultm,highPolyMultl);

  Add12(phnorm,pmnorm,ph,pm);
  Mul22(&rmlMultPh,&rmlMultPl,rm,rl,phnorm,pmnorm);
  Add22(&qh,&ql,rm,rl,rmlMultPh,rmlMultPl);

  Add233Cond(&fullPolyh,&fullPolym,&fullPolyl,qh,ql,ph,pm,pl);

  Add12(polyAddOneh,t5,1,fullPolyh);
  Add12Cond(polyAddOnem,t6,t5,fullPolym);
  polyAddOnel = t6 + fullPolyl;
  Mul33(&polyWithTbl1h,&polyWithTbl1m,&polyWithTbl1l,tbl1h,tbl1m,tbl1l,polyAddOneh,polyAddOnem,polyAddOnel);
  Mul33(&polyWithTablesh,&polyWithTablesm,&polyWithTablesl,
	tbl2h,tbl2m,tbl2l,
	polyWithTbl1h,polyWithTbl1m,polyWithTbl1l);

  Renormalize3(polyTblh,polyTblm,polyTbll,polyWithTablesh,polyWithTablesm,polyWithTablesl);

}



/*************************************************************
 *************************************************************
 *               ROUNDED  TO NEAREST			     *
 *************************************************************
 *************************************************************/
double exp_rn(double x){ 
  double rh, rm, rl, tbl1h, tbl1m, tbl1l;
  double tbl2h, tbl2m, tbl2l;
  double xMultLog2InvMult2L, shiftedXMult, kd;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double t1, t2, t3, t4, polyTblh, polyTblm, polyTbll;
  db_number shiftedXMultdb, twoPowerMdb, xdb, t4db, t4db2, polyTblhdb, resdb;
  int k, M, index1, index2, xIntHi, mightBeDenorm;
  double t5, t6, t7, t8, t9, t10, t11, t12, t13;
  double rhSquare, rhSquareHalf, rhC3, rhFour, monomialCube;
  double highPoly, highPolyWithSquare, monomialFour;
  double tablesh, tablesl;
  double s1, s2, s3, s4, s5;
  double res;
   
  /* Argument reduction and filtering for special cases */

  /* Compute k as a double and as an int */
  xdb.d = x;
  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  
  /* Special cases tests */
  xIntHi = xdb.i[HI];
  mightBeDenorm = 0;
  /* Test if argument is a denormal or zero */
  if ((xIntHi & 0x7ff00000) == 0) {
    /* We are in the RN case, return 1.0 in all cases */
    return 1.0;
  }
 
  /* Test if argument is greater than approx. 709 in magnitude */
  if ((xIntHi & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) {
    /* If we are here, the result might be overflowed, underflowed, inf, or NaN */

    /* Test if +/- Inf or NaN */
    if ((xIntHi & 0x7fffffff) >= 0x7ff00000) {
      /* Either NaN or Inf in this case since exponent is maximal */

      /* Test if NaN: mantissa is not 0 */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* x = NaN, return NaN */
	return x + x;
      } else {
	/* +/- Inf */

	/* Test sign */
	if ((xIntHi & 0x80000000)==0) 
	  /* x = +Inf, return +Inf */
	  return x;
	else
	  /* x = -Inf, return 0 */
	  return 0;
      } /* End which in NaN, Inf */
    } /* End NaN or Inf ? */
    
    /* If we are here, we might be overflowed, denormalized or underflowed in the result 
       but there is no special case (NaN, Inf) left */

    /* Test if actually overflowed */
    if (x > OVRFLWBOUND) {
      /* We are actually overflowed in the result */
      return LARGEST * LARGEST;
    }

    /* Test if surely underflowed */
    if (x <= UNDERFLWBOUND) {
      /* We are actually sure to be underflowed and not denormalized any more 
	 So we return 0 and raise the inexact flag */
      return SMALLEST * SMALLEST;
    }
       
    /* Test if possibly denormalized */
    if (x <= DENORMBOUND) {
      /* We know now that we are not sure to be normalized in the result
	 We just set an internal flag for a further test 
      */
      mightBeDenorm = 1;
    }
  } /* End might be a special case */

  /* If we are here, we are sure to be neither +/- Inf nor NaN nor overflowed nor denormalized in the argument
     but we might be denormalized in the result 

     We continue the argument reduction for the quick phase and table reads for both phases
  */

  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  k = shiftedXMultdb.i[LO];
  M = k >> L;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> LHALF;

  /* Table reads */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Test now if it is sure to launch the quick phase because no denormalized result is possible */
  if (mightBeDenorm == 1) {
    /* The result might be denormalized, we launch the accurate phase in all cases */

    /* Rest of argument reduction for accurate phase */

    Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1 = x + msLog2Div2LMultKh;
    Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
    Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

    /* Table reads for accurate phase */
     tbl1l = twoPowerIndex1[index1].lo;
     tbl2l = twoPowerIndex2[index2].lo;

     /* Call accurate phase */
     exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

     /* Final rounding and multiplication with 2^M 

        We first multiply the highest significant byte by 2^M in two steps
	and adjust it then depending on the lower significant parts.

	We cannot multiply directly by 2^M since M is less than -1022.
	We first multiply by 2^(-1000) and then by 2^(M+1000).

     */
     
     t3 = polyTblh * twoPowerM1000;

     /* Form now twoPowerM with adjusted M */
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 2023) << 20;


     /* Multiply with the rest of M, the result will be denormalized */
     t4 = t3 * twoPowerMdb.d;

     /* For x86, force the compiler to pass through memory for having the right rounding */

     t4db.d = t4;   /* Do not #if-ify this line, we need the copy */
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
     t4db2.i[HI] = t4db.i[HI];
     t4db2.i[LO] = t4db.i[LO];
     t4 = t4db2.d;
#endif

     /* Remultiply by 2^(-M) for manipulating the rounding error and the lower significant parts */
     M *= -1;
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 23) << 20;
     t5 = t4 * twoPowerMdb.d;
     t6 = t5 * twoPower1000;
     t7 = polyTblh - t6;
     
     /* The rounding decision is made at 1/2 ulp of a denormal, i.e. at 2^(-1075)
	We construct this number and by comparing with it we get to know 
	whether we are in a difficult rounding case or not. If not we just return 
	the known result. Otherwise we continue with further tests.
     */

     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M - 52) << 20;

     if (ABS(t7) != twoPowerMdb.d) return t4;

     /* If we are here, we are in a difficult rounding case */
     
     /* We have to adjust the result iff the sign of the error on 
	rounding 2^M * polyTblh (which must be an ulp of a denormal) 
	and polyTblm +arith polyTbll is the same which means that 
	the error made was greater than an ulp of an denormal.
     */

     polyTblm = polyTblm + polyTbll;

     if (t7 > 0.0) {
       if (polyTblm > 0.0) {
	 t4db.l++;
	 return t4db.d;
       } else return t4;
     } else {
       if (polyTblm < 0.0) {
	 t4db.l--;
	 return t4db.d;
       } else return t4;
     }
  } /* End accurate phase launched as there might be a denormalized result */

  /* No more underflow nor denormal is possible. There may be the case where
     M is 1024 and the value 2^M is to be multiplied may be less than 1
     So the final result will be normalized and representable by the multiplication must be 
     made in 2 steps
  */

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = c3 * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = c4 * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;

  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblh,polyTblm,t11,t13);
  
  /* Rounding test 
     Since we know that the result of the final multiplication with 2^M 
     will always be representable, we can do the rounding test on the 
     factors and multiply only the final result.
     We implement the multiplication in integer computations to overcome
     the problem of the non-representability of 2^1024 if M = 1024
  */

  if(polyTblh == (polyTblh + (polyTblm * ROUNDCST))) {
    polyTblhdb.d = polyTblh;
    polyTblhdb.i[HI] += M << 20;
    return polyTblhdb.d;
  } else 
    {
      /* Rest of argument reduction for accurate phase */

      Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1 = x + msLog2Div2LMultKh;
      Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
      Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

      /* Table reads for accurate phase */
      tbl1l = twoPowerIndex1[index1].lo;
      tbl2l = twoPowerIndex2[index2].lo;
      
      /* Call accurate phase */
      exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

      /* Since the final multiplication is exact, we can do the final rounding before multiplying
	 We overcome this way also the cases where the final result is not underflowed whereas the
	 lower parts of the intermediate final result are.
      */
      
      RoundToNearest3(&res,polyTblh,polyTblm,polyTbll);

      /* Final multiplication with 2^M 
	 We implement the multiplication in integer computations to overcome
	 the problem of the non-representability of 2^1024 if M = 1024
      */

      resdb.d = res;
      resdb.i[HI] += M << 20;
      return resdb.d;
    } /* Accurate phase launched after rounding test*/
}


/*************************************************************
 *************************************************************
 *               ROUNDED  UPWARDS			     *
 *************************************************************
 *************************************************************/
double exp_ru(double x) { 
  double rh, rm, rl, tbl1h, tbl1m, tbl1l;
  double tbl2h, tbl2m, tbl2l;
  double xMultLog2InvMult2L, shiftedXMult, kd;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double t1, t2, t3, t4, polyTblh, polyTblm, polyTbll;
  db_number shiftedXMultdb, twoPowerMdb, xdb, t4db, t4db2, resdb;
  int k, M, index1, index2, xIntHi, mightBeDenorm, roundable;
  double t5, t6, t7, t8, t9, t10, t11, t12, t13;
  double rhSquare, rhSquareHalf, rhC3, rhFour, monomialCube;
  double highPoly, highPolyWithSquare, monomialFour;
  double tablesh, tablesl;
  double s1, s2, s3, s4, s5;
  double res;
 
  /* Argument reduction and filtering for special cases */

  /* Compute k as a double and as an int */
  xdb.d = x;
  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  
  /* Special cases tests */
  xIntHi = xdb.i[HI];
  mightBeDenorm = 0;
  /* Test if argument is a denormal or zero */
  if ((xIntHi & 0x7ff00000) == 0) {
    /* If the argument is exactly zero, we just return 1.0
       which is the mathematical image of the function
    */
    if (x == 0.0) return 1.0;

    /* If the argument is a negative denormal, we 
       must return 1.0 and raise the inexact flag.
    */

    if (x < 0.0) return 1.0 + SMALLEST;

    /* Otherwise, we return 1.0 + 1ulp since 
       exp(greatest denorm) < 1.0 + 1ulp
       We must do the addition dynamically for
       raising the inexact flag.
    */

    return 1.0 + twoM52;
  }
 
  /* Test if argument is greater than approx. 709 in magnitude */
  if ((xIntHi & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) {
    /* If we are here, the result might be overflowed, underflowed, inf, or NaN */

    /* Test if +/- Inf or NaN */
    if ((xIntHi & 0x7fffffff) >= 0x7ff00000) {
      /* Either NaN or Inf in this case since exponent is maximal */

      /* Test if NaN: mantissa is not 0 */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* x = NaN, return NaN */
	return x + x;
      } else {
	/* +/- Inf */

	/* Test sign */
	if ((xIntHi & 0x80000000)==0) 
	  /* x = +Inf, return +Inf */
	  return x;
	else
	  /* x = -Inf, return 0 (even in RU!) */
	  return 0;
      } /* End which in NaN, Inf */
    } /* End NaN or Inf ? */
    
    /* If we are here, we might be overflowed, denormalized or underflowed in the result 
       but there is no special case (NaN, Inf) left */

    /* Test if actually overflowed */
    if (x > OVRFLWBOUND) {
      /* We are actually overflowed in the result */
      return LARGEST * LARGEST;
    }

    /* Test if surely underflowed */
    if (x <= UNDERFLWBOUND) {
      /* We are actually sure to be underflowed and not denormalized any more 
	 (at least where computing makes sense); since we are in the round 
	 upwards case, we return the smallest denormal possible.
      */
      return SMALLEST;
    }
       
    /* Test if possibly denormalized */
    if (x <= DENORMBOUND) {
      /* We know now that we are not sure to be normalized in the result
	 We just set an internal flag for a further test 
      */
      mightBeDenorm = 1;
    }
  } /* End might be a special case */

  /* If we are here, we are sure to be neither +/- Inf nor NaN nor overflowed nor denormalized in the argument
     but we might be denormalized in the result 

     We continue the argument reduction for the quick phase and table reads for both phases
  */

  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  k = shiftedXMultdb.i[LO];
  M = k >> L;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> LHALF;

  /* Table reads */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Test now if it is sure to launch the quick phase because no denormalized result is possible */
  if (mightBeDenorm == 1) {
    /* The result might be denormalized, we launch the accurate phase in all cases */

    /* Rest of argument reduction for accurate phase */

    Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1 = x + msLog2Div2LMultKh;
    Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
    Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

    /* Table reads for accurate phase */
     tbl1l = twoPowerIndex1[index1].lo;
     tbl2l = twoPowerIndex2[index2].lo;

     /* Call accurate phase */
     exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

     /* Final rounding and multiplication with 2^M 

        We first multiply the highest significant byte by 2^M in two steps
	and adjust it then depending on the lower significant parts.

	We cannot multiply directly by 2^M since M is less than -1022.
	We first multiply by 2^(-1000) and then by 2^(M+1000).

     */
     
     t3 = polyTblh * twoPowerM1000;

     /* Form now twoPowerM with adjusted M */
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 2023) << 20;


     /* Multiply with the rest of M, the result will be denormalized */
     t4 = t3 * twoPowerMdb.d;

     /* For x86, force the compiler to pass through memory for having the right rounding */

     t4db.d = t4;   /* Do not #if-ify this line, we need the copy */
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
     t4db2.i[HI] = t4db.i[HI];
     t4db2.i[LO] = t4db.i[LO];
     t4 = t4db2.d;
#endif


     /* Remultiply by 2^(-M) for manipulating the rounding error and the lower significant parts */
     M *= -1;
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 23) << 20;
     t5 = t4 * twoPowerMdb.d;
     t6 = t5 * twoPower1000;
     t7 = polyTblh - t6;

     /* The rounding can be decided using the sign of the arithmetical sum of the
	round-to-nearest-error (i.e. t7) and the lower part(s) of the final result.
	We add first the lower parts and add the result to the error in t7. We have to 
	keep in mind that everything is scaled by 2^(-M).
	t8 can never be exactly 0 since we filter out the cases where the image of the 
	function is algebraic and the implementation is exacter than the TMD worst case.
     */
 
     polyTblm = polyTblm + polyTbll;
     t8 = t7 + polyTblm;

     /* Since we are rounding upwards, the round-to-nearest-rounding result in t4 is 
	equal to the final result if the rounding error (i.e. the error plus the lower parts)
	is negative, i.e. if the rounding-to-nearest was upwards.
     */
     
     if (t8 < 0.0) return t4;

     /* If we are here, we must adjust the final result by +1ulp 
	Relying on the fact that the exponential is always positive, we can simplify this
	adjustment 
     */

     t4db.l++;
     return t4db.d;
  } /* End accurate phase launched as there might be a denormalized result */

  /* No more underflow nor denormal is possible. There may be the case where
     M is 1024 and the value 2^M is to be multiplied may be less than 1
     So the final result will be normalized and representable by the multiplication must be 
     made in 2 steps
  */

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = c3 * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = c4 * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;

  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblh,polyTblm,t11,t13);
  
  /* Rounding test 
     Since we know that the result of the final multiplication with 2^M 
     will always be representable, we can do the rounding test on the 
     factors and multiply only the final result.
     We implement the multiplication in integer computations to overcome
     the problem of the non-representability of 2^1024 if M = 1024
  */

  TEST_AND_COPY_RU(roundable,res,polyTblh,polyTblm,RDROUNDCST);

  if (roundable) {
    resdb.d = res;
    resdb.i[HI] += M << 20;
    return resdb.d;
  } else 
    {
      /* Rest of argument reduction for accurate phase */

      Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1 = x + msLog2Div2LMultKh;
      Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
      Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

      /* Table reads for accurate phase */
      tbl1l = twoPowerIndex1[index1].lo;
      tbl2l = twoPowerIndex2[index2].lo;
      
      /* Call accurate phase */
      exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

      /* Since the final multiplication is exact, we can do the final rounding before multiplying
	 We overcome this way also the cases where the final result is not underflowed whereas the
	 lower parts of the intermediate final result are.
      */
      
      RoundUpwards3(&res,polyTblh,polyTblm,polyTbll);

      /* Final multiplication with 2^M 
	 We implement the multiplication in integer computations to overcome
	 the problem of the non-representability of 2^1024 if M = 1024
      */

      resdb.d = res;
      resdb.i[HI] += M << 20;
      return resdb.d;
    } /* Accurate phase launched after rounding test*/
} 


/*************************************************************
 *************************************************************
 *               ROUNDED  DOWNWARDS			     *
 *************************************************************
 *************************************************************/
double exp_rd(double x) { 
  double rh, rm, rl, tbl1h, tbl1m, tbl1l;
  double tbl2h, tbl2m, tbl2l;
  double xMultLog2InvMult2L, shiftedXMult, kd;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double t1, t2, t3, t4, polyTblh, polyTblm, polyTbll;
  db_number shiftedXMultdb, twoPowerMdb, xdb, t4db, t4db2, resdb;
  int k, M, index1, index2, xIntHi, mightBeDenorm, roundable;
  double t5, t6, t7, t8, t9, t10, t11, t12, t13;
  double rhSquare, rhSquareHalf, rhC3, rhFour, monomialCube;
  double highPoly, highPolyWithSquare, monomialFour;
  double tablesh, tablesl;
  double s1, s2, s3, s4, s5;
  double res;
 
  /* Argument reduction and filtering for special cases */

  /* Compute k as a double and as an int */
  xdb.d = x;
  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  
  /* Special cases tests */
  xIntHi = xdb.i[HI];
  mightBeDenorm = 0;
  /* Test if argument is a denormal or zero */
  if ((xIntHi & 0x7ff00000) == 0) {
    /* If the argument is exactly zero, we just return 1.0
       which is the mathematical image of the function
    */
    if (x == 0.0) return 1.0;

    /* If the argument is a positive denormal, we 
       must return 1.0 and raise the inexact flag.
    */
    
    if (x > 0.0) return 1.0 + SMALLEST;

    /* Otherwise, we return 1.0 - 1ulp since 
       exp(-greatest denorm) > 1.0 - 1ulp
       We must do the addition dynamically for
       raising the inexact flag.
    */
    
    return 1.0 + mTwoM53;

  }
 
  /* Test if argument is greater than approx. 709 in magnitude */
  if ((xIntHi & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) {
    /* If we are here, the result might be overflowed, underflowed, inf, or NaN */

    /* Test if +/- Inf or NaN */
    if ((xIntHi & 0x7fffffff) >= 0x7ff00000) {
      /* Either NaN or Inf in this case since exponent is maximal */

      /* Test if NaN: mantissa is not 0 */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* x = NaN, return NaN */
	return x + x;
      } else {
	/* +/- Inf */

	/* Test sign */
	if ((xIntHi & 0x80000000)==0) 
	  /* x = +Inf, return +Inf */
	  return x;
	else
	  /* x = -Inf, return 0 */
	  return 0;
      } /* End which in NaN, Inf */
    } /* End NaN or Inf ? */
    
    /* If we are here, we might be overflowed, denormalized or underflowed in the result 
       but there is no special case (NaN, Inf) left */

    /* Test if actually overflowed */
    if (x > OVRFLWBOUND) {
      /* We would be overflowed but as we are rounding downwards
	 the nearest number lesser than the exact result is the greatest 
	 normal. In any case, we must raise the inexact flag.
      */
      return LARGEST * (1.0 + SMALLEST);
    }

    /* Test if surely underflowed */
    if (x <= UNDERFLWBOUND) {
      /* We are actually sure to be underflowed and not denormalized any more 
	 (at least where computing makes sense); since we are in the round 
	 upwards case, we return the smallest denormal possible.
      */
      return SMALLEST * SMALLEST;
    }
       
    /* Test if possibly denormalized */
    if (x <= DENORMBOUND) {
      /* We know now that we are not sure to be normalized in the result
	 We just set an internal flag for a further test 
      */
      mightBeDenorm = 1;
    }
  } /* End might be a special case */

  /* If we are here, we are sure to be neither +/- Inf nor NaN nor overflowed nor denormalized in the argument
     but we might be denormalized in the result 

     We continue the argument reduction for the quick phase and table reads for both phases
  */

  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  k = shiftedXMultdb.i[LO];
  M = k >> L;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> LHALF;

  /* Table reads */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Test now if it is sure to launch the quick phase because no denormalized result is possible */
  if (mightBeDenorm == 1) {
    /* The result might be denormalized, we launch the accurate phase in all cases */

    /* Rest of argument reduction for accurate phase */

    Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1 = x + msLog2Div2LMultKh;
    Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
    Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

    /* Table reads for accurate phase */
     tbl1l = twoPowerIndex1[index1].lo;
     tbl2l = twoPowerIndex2[index2].lo;

     /* Call accurate phase */
     exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

     /* Final rounding and multiplication with 2^M 

        We first multiply the highest significant byte by 2^M in two steps
	and adjust it then depending on the lower significant parts.

	We cannot multiply directly by 2^M since M is less than -1022.
	We first multiply by 2^(-1000) and then by 2^(M+1000).

     */
     
     t3 = polyTblh * twoPowerM1000;

     /* Form now twoPowerM with adjusted M */
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 2023) << 20;


     /* Multiply with the rest of M, the result will be denormalized */
     t4 = t3 * twoPowerMdb.d;

     /* For x86, force the compiler to pass through memory for having the right rounding */

     t4db.d = t4;   /* Do not #if-ify this line, we need the copy */
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
     t4db2.i[HI] = t4db.i[HI];
     t4db2.i[LO] = t4db.i[LO];
     t4 = t4db2.d;
#endif

     /* Remultiply by 2^(-M) for manipulating the rounding error and the lower significant parts */
     M *= -1;
     twoPowerMdb.i[LO] = 0;
     twoPowerMdb.i[HI] = (M + 23) << 20;
     t5 = t4 * twoPowerMdb.d;
     t6 = t5 * twoPower1000;
     t7 = polyTblh - t6;

     /* The rounding can be decided using the sign of the arithmetical sum of the
	round-to-nearest-error (i.e. t7) and the lower part(s) of the final result.
	We add first the lower parts and add the result to the error in t7. We have to 
	keep in mind that everything is scaled by 2^(-M).
	t8 can never be exactly 0 since we filter out the cases where the image of the 
	function is algebraic and the implementation is exacter than the TMD worst case.
     */
 
     polyTblm = polyTblm + polyTbll;
     t8 = t7 + polyTblm;

     /* Since we are rounding downwards, the round-to-nearest-rounding result in t4 is 
	equal to the final result if the rounding error (i.e. the error plus the lower parts)
	is positive, i.e. if the rounding-to-nearest was downwards.
     */
     
     if (t8 > 0.0) return t4;

     /* If we are here, we must adjust the final result by +1ulp 
	Relying on the fact that the exponential is always positive, we can simplify this
	adjustment 
     */

     t4db.l--;
     return t4db.d;
  } /* End accurate phase launched as there might be a denormalized result */

  /* No more underflow nor denormal is possible. There may be the case where
     M is 1024 and the value 2^M is to be multiplied may be less than 1
     So the final result will be normalized and representable by the multiplication must be 
     made in 2 steps
  */

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = c3 * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = c4 * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;

  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblh,polyTblm,t11,t13);
  
  /* Rounding test 
     Since we know that the result of the final multiplication with 2^M 
     will always be representable, we can do the rounding test on the 
     factors and multiply only the final result.
     We implement the multiplication in integer computations to overcome
     the problem of the non-representability of 2^1024 if M = 1024
  */

  TEST_AND_COPY_RD(roundable,res,polyTblh,polyTblm,RDROUNDCST);

  if (roundable) {
    resdb.d = res;
    resdb.i[HI] += M << 20;
    return resdb.d;
  } else {      
      /* Rest of argument reduction for accurate phase */

      Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1 = x + msLog2Div2LMultKh;
      Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
      Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

      /* Table reads for accurate phase */
      tbl1l = twoPowerIndex1[index1].lo;
      tbl2l = twoPowerIndex2[index2].lo;
      
      /* Call accurate phase */
      exp_td_accurate(&polyTblh, &polyTblm, &polyTbll, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l); 

      /* Since the final multiplication is exact, we can do the final rounding before multiplying
	 We overcome this way also the cases where the final result is not underflowed whereas the
	 lower parts of the intermediate final result are.
      */
      
      RoundDownwards3(&res,polyTblh,polyTblm,polyTbll);

      /* Final multiplication with 2^M 
	 We implement the multiplication in integer computations to overcome
	 the problem of the non-representability of 2^1024 if M = 1024
      */

      resdb.d = res;
      resdb.i[HI] += M << 20;
      return resdb.d;
    } /* Accurate phase launched after rounding test*/
} 
 

#ifdef BUILD_INTERVAL_FUNCTIONS
interval j_exp(interval x)
{
  interval res;
  double x_inf, x_sup;
  double rh_sup, rm_sup, rl_sup, tbl1h_sup, tbl1m_sup, tbl1l_sup;
  double tbl2h_sup, tbl2m_sup, tbl2l_sup;
  double xMultLog2InvMult2L_sup, shiftedXMult_sup, kd_sup;
  double msLog2Div2LMultKh_sup, msLog2Div2LMultKm_sup, msLog2Div2LMultKl_sup;
  double t1_sup, t2_sup, polyTblh_sup, polyTblm_sup, polyTbll_sup;
  db_number shiftedXMultdb_sup, xdb_sup, resdb_sup;
  int k_sup, M_sup, index1_sup, index2_sup, xIntHi_sup, mightBeDenorm_sup, roundable;
  double t8_sup, t9_sup, t10_sup, t11_sup, t12_sup, t13_sup;
  double rhSquare_sup, rhSquareHalf_sup, rhC3_sup, rhFour_sup, monomialCube_sup;
  double highPoly_sup, highPolyWithSquare_sup, monomialFour_sup;
  double tablesh_sup, tablesl_sup;
  double s1_sup, s2_sup, s3_sup, s4_sup, s5_sup;
  double res_sup;

  double rh_inf, rm_inf, rl_inf, tbl1h_inf, tbl1m_inf, tbl1l_inf;
  double tbl2h_inf, tbl2m_inf, tbl2l_inf;
  double xMultLog2InvMult2L_inf, shiftedXMult_inf, kd_inf;
  double msLog2Div2LMultKh_inf, msLog2Div2LMultKm_inf, msLog2Div2LMultKl_inf;
  double t1_inf, t2_inf, polyTblh_inf, polyTblm_inf, polyTbll_inf;
  db_number shiftedXMultdb_inf, xdb_inf, resdb_inf;
  int k_inf, M_inf, index1_inf, index2_inf, xIntHi_inf, mightBeDenorm_inf;
  double t8_inf, t9_inf, t10_inf, t11_inf, t12_inf, t13_inf;
  double rhSquare_inf, rhSquareHalf_inf, rhC3_inf, rhFour_inf, monomialCube_inf;
  double highPoly_inf, highPolyWithSquare_inf, monomialFour_inf;
  double tablesh_inf, tablesl_inf;
  double s1_inf, s2_inf, s3_inf, s4_inf, s5_inf;
  double res_inf;

  double res_simple_inf, res_simple_sup;
  int infDone=0; int supDone=0;

  x_inf=LOW(x);
  x_sup=UP(x);

  /* Argument reduction and filtering for special cases */

  /* Compute k as a double and as an int */
  xdb_sup.d = x_sup;
  xdb_inf.d = x_inf;
  xMultLog2InvMult2L_sup = x_sup * log2InvMult2L;
  xMultLog2InvMult2L_inf = x_inf * log2InvMult2L;
  shiftedXMult_sup = xMultLog2InvMult2L_sup + shiftConst;
  shiftedXMult_inf = xMultLog2InvMult2L_inf + shiftConst;
  kd_sup = shiftedXMult_sup - shiftConst;
  kd_inf = shiftedXMult_inf - shiftConst;
  shiftedXMultdb_sup.d = shiftedXMult_sup;
  shiftedXMultdb_inf.d = shiftedXMult_inf;


  /* Special cases tests */
  xIntHi_sup = xdb_sup.i[HI];
  mightBeDenorm_sup = 0;

  /* Special cases tests */
  xIntHi_inf = xdb_inf.i[HI];
  mightBeDenorm_inf = 0;

  if ( __builtin_expect(
       ((xIntHi_sup & 0x7ff00000) == 0)
    || (((xIntHi_sup & 0x7ff00000) == 0)  && (x_sup == 0.0)) 
    || (((xIntHi_sup & 0x7ff00000) == 0)  && (x_sup < 0.0))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_sup & 0x7fffffff) >= 0x7ff00000))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_sup & 0x7fffffff) >= 0x7ff00000) && (((xIntHi_sup & 0x000fffff) | xdb_sup.i[LO]) != 0))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_sup & 0x7fffffff) >= 0x7ff00000) && ((xIntHi_sup & 0x80000000)==0))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_sup > OVRFLWBOUND))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_sup <= UNDERFLWBOUND))
    || (((xIntHi_sup & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_sup <= DENORMBOUND))
    || ((xIntHi_inf & 0x7ff00000) == 0)
    || (((xIntHi_inf & 0x7ff00000) == 0) && (x_inf == 0.0))
    || (((xIntHi_inf & 0x7ff00000) == 0) && (x_inf > 0.0))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_inf & 0x7fffffff) >= 0x7ff00000))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_inf & 0x7fffffff) >= 0x7ff00000) && (((xIntHi_inf & 0x000fffff) | xdb_inf.i[LO]) != 0))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && ((xIntHi_inf & 0x7fffffff) >= 0x7ff00000) && ((xIntHi_inf & 0x80000000)==0))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_inf > OVRFLWBOUND))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_inf <= UNDERFLWBOUND))
    || (((xIntHi_inf & 0x7fffffff) >= OVRUDRFLWSMPLBOUND) && (x_inf <= DENORMBOUND))
     ,FALSE))
  {
    ASSIGN_LOW(res,exp_rd(LOW(x)));
    ASSIGN_UP(res,exp_ru(UP(x)));
    return res;
  }

  /* Test if argument is a denormal or zero */
  /* If we are here, we are sure to be neither +/- Inf nor NaN nor overflowed nor denormalized in the argument
     but we might be denormalized in the result 

     We continue the argument reduction for the quick phase and table reads for both phases
  */

  Mul12(&s1_sup,&s2_sup,msLog2Div2Lh,kd_sup);
  Mul12(&s1_inf,&s2_inf,msLog2Div2Lh,kd_inf);
  s3_sup = kd_sup * msLog2Div2Lm;
  s3_inf = kd_inf * msLog2Div2Lm;
  s4_sup = s2_sup + s3_sup; 
  s4_inf = s2_inf + s3_inf; 
  s5_sup = x_sup + s1_sup;
  s5_inf = x_inf + s1_inf;
  Add12Cond(rh_sup,rm_sup,s5_sup,s4_sup);
  Add12Cond(rh_inf,rm_inf,s5_inf,s4_inf);
  k_sup = shiftedXMultdb_sup.i[LO];
  k_inf = shiftedXMultdb_inf.i[LO];
  M_sup = k_sup >> L;
  M_inf = k_inf >> L;
  index1_sup = k_sup & INDEXMASK1;
  index1_inf = k_inf & INDEXMASK1;
  index2_sup = (k_sup & INDEXMASK2) >> LHALF;
  index2_inf = (k_inf & INDEXMASK2) >> LHALF;

  /* Table reads */
  tbl1h_sup = twoPowerIndex1[index1_sup].hi;
  tbl1h_inf = twoPowerIndex1[index1_inf].hi;
  tbl1m_sup = twoPowerIndex1[index1_sup].mi;
  tbl1m_inf = twoPowerIndex1[index1_inf].mi;
  tbl2h_sup = twoPowerIndex2[index2_sup].hi;
  tbl2h_inf = twoPowerIndex2[index2_inf].hi;
  tbl2m_sup = twoPowerIndex2[index2_sup].mi;
  tbl2m_inf = twoPowerIndex2[index2_inf].mi;




  /* No more underflow nor denormal is possible. There may be the case where
     M is 1024 and the value 2^M is to be multiplied may be less than 1
     So the final result will be normalized and representable by the multiplication must be 
     made in 2 steps
  */

  /* Quick phase starts here */

  rhSquare_sup = rh_sup * rh_sup;
  rhSquare_inf = rh_inf * rh_inf;
  rhC3_sup = c3 * rh_sup;
  rhC3_inf = c3 * rh_inf;
  rhSquareHalf_sup = 0.5 * rhSquare_sup;
  rhSquareHalf_inf = 0.5 * rhSquare_inf;
  monomialCube_sup = rhC3_sup * rhSquare_sup;
  monomialCube_inf = rhC3_inf * rhSquare_inf;
  rhFour_sup = rhSquare_sup * rhSquare_sup;
  rhFour_inf = rhSquare_inf * rhSquare_inf;
  monomialFour_sup = c4 * rhFour_sup;
  monomialFour_inf = c4 * rhFour_inf;
  highPoly_sup = monomialCube_sup + monomialFour_sup;
  highPoly_inf = monomialCube_inf + monomialFour_inf;
  highPolyWithSquare_sup = rhSquareHalf_sup + highPoly_sup;
  highPolyWithSquare_inf = rhSquareHalf_inf + highPoly_inf;
  Mul22(&tablesh_sup,&tablesl_sup,tbl1h_sup,tbl1m_sup,tbl2h_sup,tbl2m_sup);
  Mul22(&tablesh_inf,&tablesl_inf,tbl1h_inf,tbl1m_inf,tbl2h_inf,tbl2m_inf);
  t8_sup = rm_sup + highPolyWithSquare_sup;
  t8_inf = rm_inf + highPolyWithSquare_inf;
  t9_sup = rh_sup + t8_sup;
  t9_inf = rh_inf + t8_inf;
  t10_sup = tablesh_sup * t9_sup;
  t10_inf = tablesh_inf * t9_inf;
  Add12(t11_sup,t12_sup,tablesh_sup,t10_sup);
  Add12(t11_inf,t12_inf,tablesh_inf,t10_inf);
  t13_sup = t12_sup + tablesl_sup;
  t13_inf = t12_inf + tablesl_inf;
  Add12(polyTblh_sup,polyTblm_sup,t11_sup,t13_sup);
  Add12(polyTblh_inf,polyTblm_inf,t11_inf,t13_inf);
  
  /* Rounding test 
     Since we know that the result of the final multiplication with 2^M 
     will always be representable, we can do the rounding test on the 
     factors and multiply only the final result.
     We implement the multiplication in integer computations to overcome
     the problem of the non-representability of 2^1024 if M = 1024
  */

  if (infDone==1) res_inf=res_simple_inf;
  if (supDone==1) res_sup=res_simple_sup;

//  TEST_AND_COPY_RDRU_EXP(roundable,infDone,supDone,res_inf,polyTblh_inf,polyTblm_inf,res_sup,polyTblh_sup,polyTblm_sup,RDROUNDCST);
  db_number yh_inf, yl_inf, u53_inf, yh_sup, yl_sup, u53_sup;
  int yh_inf_neg, yl_inf_neg, yh_sup_neg, yl_sup_neg;
  int rd_ok, ru_ok;
  double save_res_inf=res_inf;
  double save_res_sup=res_sup;
  yh_inf.d = polyTblh_inf;    yl_inf.d = polyTblm_inf;
  yh_inf_neg = (yh_inf.i[HI] & 0x80000000);
  yl_inf_neg = (yl_inf.i[HI] & 0x80000000);
  yh_inf.l = yh_inf.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/
  yl_inf.l = yl_inf.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/
  u53_inf.l     = (yh_inf.l & ULL(7ff0000000000000)) +  ULL(0010000000000000);
  yh_sup.d = polyTblh_sup;    yl_sup.d = polyTblm_sup;
  yh_sup_neg = (yh_sup.i[HI] & 0x80000000);
  yl_sup_neg = (yl_sup.i[HI] & 0x80000000);
  yh_sup.l = yh_sup.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/
  yl_sup.l = yl_sup.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/
  u53_sup.l     = (yh_sup.l & ULL(7ff0000000000000)) +  ULL(0010000000000000);
  roundable = 0;
  rd_ok=(yl_inf.d > RDROUNDCST * u53_inf.d);
  ru_ok=(yl_sup.d > RDROUNDCST * u53_sup.d);
     if(yl_inf_neg) {  /* The case yl==0 is filtered by the above test*/
      /* return next down */
       yh_inf.d = polyTblh_inf;
      if(yh_inf_neg) yh_inf.l++;  else yh_inf.l--; /* Beware: fails for zero */
      res_inf = yh_inf.d;
    }
    else {
      res_inf = polyTblh_inf;
    }
    if(!yl_sup_neg) {  /* The case yl==0 is filtered by the above test*/
      /* return next up */
      yh_sup.d = polyTblh_sup;
      if(yh_sup_neg) yh_sup.l--;  else yh_sup.l++; /* Beware: fails for zero */
      res_sup = yh_sup.d;
    }
    else {
      res_sup = polyTblh_sup;
    }
  if(infDone) res_inf=save_res_inf;
  if(supDone) res_sup=save_res_sup;
  if(rd_ok && ru_ok){
    roundable=3;
  }
  else if (rd_ok){
    roundable=1;
  }
  else if (ru_ok){
     roundable=2;
  }
  resdb_inf.d = res_inf;
  resdb_sup.d = res_sup;

  if (roundable==3)
  {
    if (infDone==0){
      resdb_inf.i[HI] += M_inf << 20;
    }
    ASSIGN_LOW(res,resdb_inf.d);
    if (supDone==0){
      resdb_sup.i[HI] += M_sup << 20;
    }
    ASSIGN_UP(res,resdb_sup.d);
    return res;
  }
  if(roundable==1)
  {
    if(infDone==0){
      resdb_inf.i[HI] += M_inf << 20;
    }
    ASSIGN_LOW(res,resdb_inf.d);
    if(supDone==0){
    /* Rest of argument reduction for accurate phase */
    Mul133(&msLog2Div2LMultKh_sup,&msLog2Div2LMultKm_sup,&msLog2Div2LMultKl_sup,kd_sup,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1_sup = x_sup + msLog2Div2LMultKh_sup;
    Add12Cond(rh_sup,t2_sup,t1_sup,msLog2Div2LMultKm_sup);
    Add12Cond(rm_sup,rl_sup,t2_sup,msLog2Div2LMultKl_sup);
    /* Table reads for accurate phase */
    tbl1l_sup = twoPowerIndex1[index1_sup].lo;
    tbl2l_sup = twoPowerIndex2[index2_sup].lo;
    /* Call accurate phase */
    exp_td_accurate(&polyTblh_sup, &polyTblm_sup, &polyTbll_sup, rh_sup, rm_sup, rl_sup, tbl1h_sup, tbl1m_sup, tbl1l_sup, tbl2h_sup, tbl2m_sup, tbl2l_sup); 
    /* Since the final multiplication is exact, we can do the final rounding before multiplying
       We overcome this way also the cases where the final result is not underflowed whereas the
       lower parts of the intermediate final result are.
    */
    RoundUpwards3(&res_sup,polyTblh_sup,polyTblm_sup,polyTbll_sup);
    /* Final multiplication with 2^M 
       We implement the multiplication in integer computations to overcome
       the problem of the non-representability of 2^1024 if M = 1024
    */
    resdb_sup.d = res_sup;
    resdb_sup.i[HI] += M_sup << 20;
    }
    ASSIGN_UP(res,resdb_sup.d);
    return res;
  } /* Accurate phase launched after rounding test*/
    
  if (roundable==2) {
    if (infDone==0){
    /* Rest of argument reduction for accurate phase */
    Mul133(&msLog2Div2LMultKh_inf,&msLog2Div2LMultKm_inf,&msLog2Div2LMultKl_inf,kd_inf,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1_inf = x_inf + msLog2Div2LMultKh_inf;
    Add12Cond(rh_inf,t2_inf,t1_inf,msLog2Div2LMultKm_inf);
    Add12Cond(rm_inf,rl_inf,t2_inf,msLog2Div2LMultKl_inf);
    /* Table reads for accurate phase */
    tbl1l_inf = twoPowerIndex1[index1_inf].lo;
    tbl2l_inf = twoPowerIndex2[index2_inf].lo;
    /* Call accurate phase */
    exp_td_accurate(&polyTblh_inf, &polyTblm_inf, &polyTbll_inf, rh_inf, rm_inf, rl_inf, tbl1h_inf, tbl1m_inf, tbl1l_inf, tbl2h_inf, tbl2m_inf, tbl2l_inf); 
    /* Since the final multiplication is exact, we can do the final rounding before multiplying
       We overcome this way also the cases where the final result is not underflowed whereas the
       lower parts of the intermediate final result are.
    */

    RoundDownwards3(&res_inf,polyTblh_inf,polyTblm_inf,polyTbll_inf);
    /* Final multiplication with 2^M 
       We implement the multiplication in integer computations to overcome
       the problem of the non-representability of 2^1024 if M = 1024
    */

    resdb_inf.d = res_inf;
    resdb_inf.i[HI] += M_inf << 20;
    }
    ASSIGN_LOW(res,resdb_inf.d);
    if(supDone==0){
      resdb_sup.i[HI] += M_sup << 20;
    }
    ASSIGN_UP(res,resdb_sup.d);    
    return res;
  } /* Accurate phase launched after rounding test*/
  if(roundable==0)
  {
    if(supDone==0){
    /* Rest of argument reduction for accurate phase */
    Mul133(&msLog2Div2LMultKh_sup,&msLog2Div2LMultKm_sup,&msLog2Div2LMultKl_sup,kd_sup,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1_sup = x_sup + msLog2Div2LMultKh_sup;
    Add12Cond(rh_sup,t2_sup,t1_sup,msLog2Div2LMultKm_sup);
    Add12Cond(rm_sup,rl_sup,t2_sup,msLog2Div2LMultKl_sup);
    /* Table reads for accurate phase */
    tbl1l_sup = twoPowerIndex1[index1_sup].lo;
    tbl2l_sup = twoPowerIndex2[index2_sup].lo;
    /* Call accurate phase */
    exp_td_accurate(&polyTblh_sup, &polyTblm_sup, &polyTbll_sup, rh_sup, rm_sup, rl_sup, tbl1h_sup, tbl1m_sup, tbl1l_sup, tbl2h_sup, tbl2m_sup, tbl2l_sup); 
    /* Since the final multiplication is exact, we can do the final rounding before multiplying
       We overcome this way also the cases where the final result is not underflowed whereas the
       lower parts of the intermediate final result are.
    */
    RoundUpwards3(&res_sup,polyTblh_sup,polyTblm_sup,polyTbll_sup);
    /* Final multiplication with 2^M 
       We implement the multiplication in integer computations to overcome
       the problem of the non-representability of 2^1024 if M = 1024
    */
    resdb_sup.d = res_sup;
    resdb_sup.i[HI] += M_sup << 20;
    }
    ASSIGN_UP(res,resdb_sup.d);
    if (infDone==0){
    /* Rest of argument reduction for accurate phase */
    Mul133(&msLog2Div2LMultKh_inf,&msLog2Div2LMultKm_inf,&msLog2Div2LMultKl_inf,kd_inf,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1_inf = x_inf + msLog2Div2LMultKh_inf;
    Add12Cond(rh_inf,t2_inf,t1_inf,msLog2Div2LMultKm_inf);
    Add12Cond(rm_inf,rl_inf,t2_inf,msLog2Div2LMultKl_inf);
    /* Table reads for accurate phase */
    tbl1l_inf = twoPowerIndex1[index1_inf].lo;
    tbl2l_inf = twoPowerIndex2[index2_inf].lo;
    /* Call accurate phase */
    exp_td_accurate(&polyTblh_inf, &polyTblm_inf, &polyTbll_inf, rh_inf, rm_inf, rl_inf, tbl1h_inf, tbl1m_inf, tbl1l_inf, tbl2h_inf, tbl2m_inf, tbl2l_inf); 
    /* Since the final multiplication is exact, we can do the final rounding before multiplying
       We overcome this way also the cases where the final result is not underflowed whereas the
       lower parts of the intermediate final result are.
    */

    RoundDownwards3(&res_inf,polyTblh_inf,polyTblm_inf,polyTbll_inf);
    /* Final multiplication with 2^M 
       We implement the multiplication in integer computations to overcome
       the problem of the non-representability of 2^1024 if M = 1024
    */

    resdb_inf.d = res_inf;
    resdb_inf.i[HI] += M_inf << 20;
    }
    ASSIGN_LOW(res,resdb_inf.d);
    return res;
  } /* Accurate phase launched after rounding test*/

  return res;
}
#endif

