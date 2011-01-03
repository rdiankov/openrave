/*
 * Correctly rounded expm1 = e^x - 1
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
#include "expm1.h"
#ifdef BUILD_INTERVAL_FUNCTIONS
#include "interval.h"
#endif



#define DEBUG 0


void expm1_direct_td(double *expm1h, double *expm1m, double *expm1l, 
		     double x, double xSqHalfh, double xSqHalfl, double xSqh, double xSql, int expoX) {
  double highPoly, tt1h, t1h, t1l, t2h, t2l, t3h, t3l, t4h, t4l, t5h, t5l, t6h, t6l;
  double tt6h, tt6m, tt6l, t7h, t7m, t7l, lowPolyh, lowPolym, lowPolyl;
  double fullHighPolyh, fullHighPolym, fullHighPolyl, polyh, polym, polyl;
  double xCubeh, xCubem, xCubel, tt7h, tt7m, tt7l, t8h, t8m, t8l;
  double expm1hover, expm1mover, expm1lover;
  double r1h, r1m, r1l, r2h, r2m, r2l, r3h, r3m, r3l;
  double rr1h, rr1m, rr1l, rr2h, rr2m, rr2l, rr3h, rr3m, rr3l;
  double fullHighPolyhover, fullHighPolymover, fullHighPolylover;

  /* Double precision evaluation steps */
#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    highPoly = FMA(FMA(FMA(FMA(accuDirectpolyC15h ,x,accuDirectpolyC14h),x,accuDirectpolyC13h),x,
                               accuDirectpolyC12h),x,accuDirectpolyC11h);
#else
    highPoly = accuDirectpolyC11h + x * (accuDirectpolyC12h + x * (accuDirectpolyC13h + x * (
	       accuDirectpolyC14h + x *  accuDirectpolyC15h)));
#endif

    tt1h = x * highPoly;

    /* Triple-double steps for x + x^2/2 and x^3*/

    Add123(&lowPolyh,&lowPolym,&lowPolyl,x,xSqHalfh,xSqHalfl);                     /* infty - 52/53 */
    Mul123(&xCubeh,&xCubem,&xCubel,x,xSqh,xSql);                                   /* 154 - 47/53 */


    /* Double-double evaluation steps */

    Add12(t1h,t1l,accuDirectpolyC10h,tt1h);
    
    MulAdd212(&t2h,&t2l,accuDirectpolyC9h,accuDirectpolyC9m,x,t1h,t1l);
    MulAdd212(&t3h,&t3l,accuDirectpolyC8h,accuDirectpolyC8m,x,t2h,t2l);
    MulAdd212(&t4h,&t4l,accuDirectpolyC7h,accuDirectpolyC7m,x,t3h,t3l);
    MulAdd212(&t5h,&t5l,accuDirectpolyC6h,accuDirectpolyC6m,x,t4h,t4l);
    MulAdd212(&t6h,&t6l,accuDirectpolyC5h,accuDirectpolyC5m,x,t5h,t5l);

    /* Triple-double evaluation steps */

    Mul123(&tt6h,&tt6m,&tt6l,x,t6h,t6l);                                          /* 154 - 47/53 */
    Add233(&t7h,&t7m,&t7l,accuDirectpolyC4h,accuDirectpolyC4m,tt6h,tt6m,tt6l);    /* 150 - 43/53 */
   
    Mul133(&tt7h,&tt7m,&tt7l,x,t7h,t7m,t7l);                                      /* 143 - 38/53 */
    Add33(&t8h,&t8m,&t8l,accuDirectpolyC3h,accuDirectpolyC3m,accuDirectpolyC3l,tt7h,tt7m,tt7l); /* 135 - 33/53 */

    Mul33(&fullHighPolyhover,&fullHighPolymover,&fullHighPolylover,xCubeh,xCubem,xCubel,t8h,t8m,t8l); /* 130 - 29/53 */

    Renormalize3(&fullHighPolyh,&fullHighPolym,&fullHighPolyl,
		 fullHighPolyhover,fullHighPolymover,fullHighPolylover);                     /* infty - 52/53 */

    Add33(&polyh,&polym,&polyl,lowPolyh,lowPolym,lowPolyl,fullHighPolyh,fullHighPolym,fullHighPolyl);
                                                                                             /* 149 - 47/53 */

    /* Reconstruction steps */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
                                                                             
      Add133(&r1h,&r1m,&r1l,2,polyh,polym,polyl);                             
      Mul33(&rr1h,&rr1m,&rr1l,r1h,r1m,r1l,polyh,polym,polyl);
      if (expoX >= 1) {

	/* Second reconstruction step */
	Add133(&r2h,&r2m,&r2l,2,rr1h,rr1m,rr1l);
	Mul33(&rr2h,&rr2m,&rr2l,r2h,r2m,r2l,rr1h,rr1m,rr1l);

	if (expoX >= 2) {

	  /* Third reconstruction step */
	  Add133(&r3h,&r3m,&r3l,2,rr2h,rr2m,rr2l);
	  Mul33(&rr3h,&rr3m,&rr3l,r3h,r3m,r3l,rr2h,rr2m,rr2l);

	  /* expoX may be maximally 2 */

	  expm1hover = rr3h;
	  expm1mover = rr3m;
	  expm1lover = rr3l;

	} else {
	  expm1hover = rr2h;
	  expm1mover = rr2m;
	  expm1lover = rr2l;
	}

      } else {
	expm1hover = rr1h;
	expm1mover = rr1m;
	expm1lover = rr1l;
      }

    } else {
      expm1hover = polyh;
      expm1mover = polym;
      expm1lover = polyl;
    }

    /* Renormalize before returning */

    Renormalize3(expm1h,expm1m,expm1l,expm1hover,expm1mover,expm1lover);
}

void expm1_common_td(double *expm1h, double *expm1m, double *expm1l, 
		     double rh, double rm, double rl, 
		     double tbl1h, double tbl1m, double tbl1l, 
		     double tbl2h, double tbl2m, double tbl2l, 
		     int M) {
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
  double exph, expm, expl, expm1hover, expm1mover, expm1lover;
  db_number polyWithTableshdb, polyWithTablesmdb, polyWithTablesldb;

  /* Polynomial approximation - double precision steps */

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
  highPoly = FMA(FMA(accuCommonpolyC7h,rh,accuCommonpolyC6h),rh,accuCommonpolyC5h);
#else
  highPoly = accuCommonpolyC5h + rh * (accuCommonpolyC6h + rh * accuCommonpolyC7h);
#endif

  /* Polynomial approximation - double-double precision steps */

  Mul12(&t1h,&t1l,rh,highPoly);
  Add22(&t2h,&t2l,accuCommonpolyC4h,accuCommonpolyC4m,t1h,t1l);
  Mul122(&t3h,&t3l,rh,t2h,t2l);
  Add22(&t4h,&t4l,accuCommonpolyC3h,accuCommonpolyC3m,t3h,t3l);

  Mul12(&rhSquareh,&rhSquarel,rh,rh);
  Mul123(&rhCubeh,&rhCubem,&rhCubel,rh,rhSquareh,rhSquarel);

  rhSquareHalfh = 0.5 * rhSquareh;
  rhSquareHalfl = 0.5 * rhSquarel;  

  /* Polynomial approximation - triple-double precision steps */

  Renormalize3(&lowPolyh,&lowPolym,&lowPolyl,rh,rhSquareHalfh,rhSquareHalfl);

  Mul233(&highPolyMulth,&highPolyMultm,&highPolyMultl,t4h,t4l,rhCubeh,rhCubem,rhCubel);

  Add33(&ph,&pm,&pl,lowPolyh,lowPolym,lowPolyl,highPolyMulth,highPolyMultm,highPolyMultl);

  /* Reconstruction */

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

  /* Multiplication by 2^(M) 

     We perform it in integer to overcome the non-representability of 2^(1024) 
     This case is possible for M = 1024 and polyWithTablesh < 1

     The overlap in the triple-double polyWithTables[hml] stays unchanged.

  */

  polyWithTableshdb.d = polyWithTablesh;
  polyWithTablesmdb.d = polyWithTablesm;
  polyWithTablesldb.d = polyWithTablesl;

  /* TODO FIXME probably at least the first of these tests is useless,
     but I leave this to Christoph to check it. Let us be
     conservative. Florent  */
  if(polyWithTableshdb.d!=0)
    polyWithTableshdb.i[HI] += M << 20;
  if(polyWithTablesmdb.d!=0)
    polyWithTablesmdb.i[HI] += M << 20;
  if(polyWithTablesldb.d!=0)
    polyWithTablesldb.i[HI] += M << 20;

  exph = polyWithTableshdb.d;
  expm = polyWithTablesmdb.d;
  expl = polyWithTablesldb.d;

  /* Substraction of -1 
     
     We use a conditional Add133 
  */

  Add133Cond(&expm1hover,&expm1mover,&expm1lover,-1,exph,expm,expl);

  /* Renormalization */    

  Renormalize3(expm1h,expm1m,expm1l,expm1hover,expm1mover,expm1lover);

}


double expm1_rn(double x) {
  db_number xdb, shiftedXMultdb, polyTblhdb, polyTblmdb;
  int xIntHi, expoX, k, M, index1, index2;
  double highPoly, tt1h, t1h, t1l, xSqh, xSql, xSqHalfh, xSqHalfl, xCubeh, xCubel, t2h, t2l, templ, tt3h, tt3l;
  double polyh, polyl, expm1h, expm1m, expm1l;
  double r1h, r1l, r1t, rr1h, rr1l;
  double r2h, r2l, r2t, rr2h, rr2l;
  double r3h, r3l, r3t, rr3h, rr3l;
  double xMultLog2InvMult2L, shiftedXMult, kd, s1, s2, s3, s4, s5, rh, rm, rl;
  double rhSquare, rhC3, rhSquareHalf, monomialCube, rhFour, monomialFour;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;
  double highPolyWithSquare, tablesh, tablesl, t8, t9, t10, t11, t12, t13;
  double exph, expm, t1, t2, t3;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double middlePoly, doublePoly;


  xdb.d = x; 

  /* Strip off the sign of x for the following tests */

  xIntHi = xdb.i[HI] & 0x7fffffff;

  /* Test if we are so small that we can return (a corrected) x as correct rounding */
  if (xIntHi < RETURNXBOUND) {
    return x;
  }


  /* Filter out special cases like overflow, -1 in result, infinities and NaNs 
     The filters are not sharp, we have positive arguments that flow through
  */
  if (xIntHi >= SIMPLEOVERFLOWBOUND) {
    /* Test if we are +/-inf or NaN */
    if (xIntHi >= 0x7ff00000) {
      /* Test if NaN */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* NaN */
	return x+x;  /* return NaN */
      }
      /* Test if +inf or -inf */
      if (xdb.i[HI] > 0) {
	/* +inf */
	return x+x;  /* return +inf */
      }
      
      /* If we are here, we are -inf */
      return -1.0;
    }

    /* If we are here, we are overflowed or a common case that flows through */

    /* Test if we are actually overflowed */
    if (x > OVERFLOWBOUND) {
      return LARGEST * LARGEST;  /* return +inf and set flag */
    }
  }
  
  /* Test if we know already that we are -1.0 (+ correction depending on rounding mode) in result */
  if (x < MINUSONEBOUND) {
    return -1.0;
  }

  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi < DIRECTINTERVALBOUND) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX = ((xIntHi & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb.i[HI] += (-expoX-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi = xdb.i[HI] & 0x7fffffff;
      x = xdb.d;
    }
    
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh,&xSql,x,x);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly = FMA(quickDirectpolyC5h,x,quickDirectpolyC4h);
#else
    middlePoly = quickDirectpolyC4h + x * quickDirectpolyC5h;
#endif

    doublePoly = middlePoly;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly = FMA(FMA(FMA(quickDirectpolyC9h ,x,quickDirectpolyC8h),x,
                             quickDirectpolyC7h),x,quickDirectpolyC6h);
#else
      highPoly = quickDirectpolyC6h + x * (quickDirectpolyC7h + x * (
	         quickDirectpolyC8h + x *  quickDirectpolyC9h));
#endif

      highPolyWithSquare = xSqh * highPoly;

      doublePoly = middlePoly + highPolyWithSquare;

    }
    
    /* Double-double evaluation steps */
    tt1h = x * doublePoly;

    xSqHalfh = 0.5 * xSqh;
    xSqHalfl = 0.5 * xSql;
    Add12(t2h,templ,x,xSqHalfh);
    t2l = templ + xSqHalfl;
    
    Add12(t1h,t1l,quickDirectpolyC3h,tt1h);
    Mul122(&xCubeh,&xCubel,x,xSqh,xSql);
    Mul22(&tt3h,&tt3l,xCubeh,xCubel,t1h,t1l);

    Add22(&polyh,&polyl,t2h,t2l,tt3h,tt3l);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h,r1t,2,polyh);
      r1l = r1t + polyl;
      Mul22(&rr1h,&rr1l,r1h,r1l,polyh,polyl);

      if (expoX >= 1) {

	/* Second reconstruction step */
	Add12(r2h,r2t,2,rr1h);
	r2l = r2t + rr1l;
	Mul22(&rr2h,&rr2l,r2h,r2l,rr1h,rr1l);

	if (expoX >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h,r3t,2,rr2h);
	  r3l = r3t + rr2l;
	  Mul22(&rr3h,&rr3l,r3h,r3l,rr2h,rr2l);
	  
	  /* expoX may be maximally 2 */

	  expm1h = rr3h;
	  expm1m = rr3l;

	} else {
	  expm1h = rr2h;
	  expm1m = rr2l;
	}

      } else {
	expm1h = rr1h;
	expm1m = rr1l;
      }

    } else {
      expm1h = polyh;
      expm1m = polyl;
    }

    /* Rounding test */
    if(expm1h == (expm1h + (expm1m * ROUNDCSTDIRECTRN)))
     return expm1h;
   else 
     {

#if DEBUG 
       printf("Launch accurate phase (direct interval)\n");
#endif

       expm1_direct_td(&expm1h, &expm1m, &expm1l, x, xSqHalfh, xSqHalfl, xSqh, xSql, expoX);
      
       ReturnRoundToNearest3(expm1h, expm1m, expm1l);

     } /* Accurate phase launched */

    /* We cannot be here, since we return in all cases before */
  }

  /* If we are here, we can use expm1(x) = exp(x) - 1 */

  /* Range reduction - exact part: compute k as double and as int */

  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  k = shiftedXMultdb.i[LO];
  M = k >> 12;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 6;

  /* Range reduction - part affected by error - must be redone in accurate phase */
  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  /* Table reads - read only two double-doubles by now */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = quickCommonpolyC3h * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = quickCommonpolyC4h * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;
  
  /* Reconstruction: integration of table values */
  
  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblhdb.d,polyTblmdb.d,t11,t13);
  
  /* Reconstruction: multiplication by 2^M */

  /* Implement the multiplication by multiplication to overcome the
     problem of the non-representability of 2^1024 (M = 1024)
     This case is possible if polyTblhdb.d < 1
  */
  
  polyTblhdb.i[HI] += M << 20;
  polyTblmdb.i[HI] += M << 20;

  exph = polyTblhdb.d;
  expm = polyTblmdb.d;

  /* Substraction of 1 

     Testing if the operation is necessary is more expensive than 
     performing it in any case.

     We may cancellate at most 2 bits in the subtraction for 
     arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
     We must therefore use conditional Add12s

     Since we perform a substraction, we may not have addition overflow towards +inf

  */

  Add12Cond(t1,t2,-1,exph);
  t3 = t2 + expm;
  Add12Cond(expm1h,expm1m,t1,t3);


  /* Rounding test */
  if(expm1h == (expm1h + (expm1m * ROUNDCSTCOMMONRN))) {
    return expm1h;
  } else {
    /* Rest of argument reduction for accurate phase */
    
    Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1 = x + msLog2Div2LMultKh;
    Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
    Add12Cond(rm,rl,t2,msLog2Div2LMultKl);
    
    /* Table reads for accurate phase */
    tbl1l = twoPowerIndex1[index1].lo;
    tbl2l = twoPowerIndex2[index2].lo;

#if DEBUG 
       printf("Launch accurate phase (common interval)\n");
#endif
    
    /* Call accurate phase */
    expm1_common_td(&expm1h, &expm1m, &expm1l, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l, M); 
    
    /* Final rounding */

    ReturnRoundToNearest3(expm1h, expm1m, expm1l);
  } /* Accurate phase launched */
  
  /* We cannot be here since we return before in any case */
}

double expm1_rd(double x) {
  db_number xdb, shiftedXMultdb, polyTblhdb, polyTblmdb;
  int xIntHi, expoX, k, M, index1, index2;
  double highPoly, tt1h, t1h, t1l, xSqh, xSql, xSqHalfh, xSqHalfl, xCubeh, xCubel, t2h, t2l, templ, tt3h, tt3l;
  double polyh, polyl, expm1h, expm1m, expm1l;
  double r1h, r1l, r1t, rr1h, rr1l;
  double r2h, r2l, r2t, rr2h, rr2l;
  double r3h, r3l, r3t, rr3h, rr3l;
  double xMultLog2InvMult2L, shiftedXMult, kd, s1, s2, s3, s4, s5, rh, rm, rl;
  double rhSquare, rhC3, rhSquareHalf, monomialCube, rhFour, monomialFour;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;
  double highPolyWithSquare, tablesh, tablesl, t8, t9, t10, t11, t12, t13;
  double exph, expm, t1, t2, t3;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double middlePoly, doublePoly;

  xdb.d = x; 

  /* Strip off the sign of x for the following tests */

  xIntHi = xdb.i[HI] & 0x7fffffff;

  /* Test if we are so small that we can return (a corrected) x as correct rounding */
  if (xIntHi < RETURNXBOUND) {
    /* The only algebraic result is 0 for x = +/- 0; in this case, we can return x = +/- 0
       The truncation rest x^2/2 + x^3/6 + ... is always positive 
       but less than 1 ulp in this case, so we round down by returning x
    */
    return x;
  }


  /* Filter out special cases like overflow, -1 in result, infinities and NaNs 
     The filters are not sharp, we have positive arguments that flow through
  */
  if (xIntHi >= SIMPLEOVERFLOWBOUND) {
    /* Test if we are +/-inf or NaN */
    if (xIntHi >= 0x7ff00000) {
      /* Test if NaN */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* NaN */
	return x+x;  /* return NaN */
      }
      /* Test if +inf or -inf */
      if (xdb.i[HI] > 0) {
	/* +inf */
	return x+x;  /* return +inf */
      }
      
      /* If we are here, we are -inf */
      return -1.0;
    }

    /* If we are here, we are overflowed or a common case that flows through */

    /* Test if we are actually overflowed */
    if (x > OVERFLOWBOUND) {
      /* We would be overflowed but as we are rounding downwards
	 the nearest number lesser than the exact result is the greatest 
	 normal. In any case, we must raise the inexact flag.
      */
      return LARGEST * (1.0 + SMALLEST);
    }
  }
  
  /* Test if we know already that we are -1.0 (+ correction depending on rounding mode) in result */
  if (x < MINUSONEBOUND) {
    /* We round down, so we are -1.0 */
    return -1.0;
  }

  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi < DIRECTINTERVALBOUND) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX = ((xIntHi & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb.i[HI] += (-expoX-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi = xdb.i[HI] & 0x7fffffff;
      x = xdb.d;
    }
    
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh,&xSql,x,x);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly = FMA(quickDirectpolyC5h,x,quickDirectpolyC4h);
#else
    middlePoly = quickDirectpolyC4h + x * quickDirectpolyC5h;
#endif

    doublePoly = middlePoly;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly = FMA(FMA(FMA(quickDirectpolyC9h ,x,quickDirectpolyC8h),x,
                             quickDirectpolyC7h),x,quickDirectpolyC6h);
#else
      highPoly = quickDirectpolyC6h + x * (quickDirectpolyC7h + x * (
	         quickDirectpolyC8h + x *  quickDirectpolyC9h));
#endif

      highPolyWithSquare = xSqh * highPoly;

      doublePoly = middlePoly + highPolyWithSquare;

    }
    
    /* Double-double evaluation steps */
    tt1h = x * doublePoly;

    xSqHalfh = 0.5 * xSqh;
    xSqHalfl = 0.5 * xSql;
    Add12(t2h,templ,x,xSqHalfh);
    t2l = templ + xSqHalfl;
    
    Add12(t1h,t1l,quickDirectpolyC3h,tt1h);
    Mul122(&xCubeh,&xCubel,x,xSqh,xSql);
    Mul22(&tt3h,&tt3l,xCubeh,xCubel,t1h,t1l);

    Add22(&polyh,&polyl,t2h,t2l,tt3h,tt3l);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h,r1t,2,polyh);
      r1l = r1t + polyl;
      Mul22(&rr1h,&rr1l,r1h,r1l,polyh,polyl);

      if (expoX >= 1) {

	/* Second reconstruction step */
	Add12(r2h,r2t,2,rr1h);
	r2l = r2t + rr1l;
	Mul22(&rr2h,&rr2l,r2h,r2l,rr1h,rr1l);

	if (expoX >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h,r3t,2,rr2h);
	  r3l = r3t + rr2l;
	  Mul22(&rr3h,&rr3l,r3h,r3l,rr2h,rr2l);
	  
	  /* expoX may be maximally 2 */

	  expm1h = rr3h;
	  expm1m = rr3l;

	} else {
	  expm1h = rr2h;
	  expm1m = rr2l;
	}

      } else {
	expm1h = rr1h;
	expm1m = rr1l;
      }

    } else {
      expm1h = polyh;
      expm1m = polyl;
    }

    /* Rounding test */
    TEST_AND_RETURN_RD(expm1h, expm1m, ROUNDCSTDIRECTRD);
    {
      expm1_direct_td(&expm1h, &expm1m, &expm1l, x, xSqHalfh, xSqHalfl, xSqh, xSql, expoX);
      
      ReturnRoundDownwards3(expm1h, expm1m, expm1l);
      
    } /* Accurate phase launched */

    /* We cannot be here, since we return in all cases before */
  }

  /* If we are here, we can use expm1(x) = exp(x) - 1 */

  /* Range reduction - exact part: compute k as double and as int */

  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  k = shiftedXMultdb.i[LO];
  M = k >> 12;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 6;

  /* Range reduction - part affected by error - must be redone in accurate phase */
  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  /* Table reads - read only two double-doubles by now */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = quickCommonpolyC3h * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = quickCommonpolyC4h * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;
  
  /* Reconstruction: integration of table values */
  
  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblhdb.d,polyTblmdb.d,t11,t13);
  
  /* Reconstruction: multiplication by 2^M */

  /* Implement the multiplication by multiplication to overcome the
     problem of the non-representability of 2^1024 (M = 1024)
     This case is possible if polyTblhdb.d < 1
  */
  
  polyTblhdb.i[HI] += M << 20;
  polyTblmdb.i[HI] += M << 20;

  exph = polyTblhdb.d;
  expm = polyTblmdb.d;

  /* Substraction of 1 

     Testing if the operation is necessary is more expensive than 
     performing it in any case.

     We may cancellate at most 2 bits in the subtraction for 
     arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
     We must therefore use conditional Add12s

     Since we perform a substraction, we may not have addition overflow towards +inf

  */

  Add12Cond(t1,t2,-1,exph);
  t3 = t2 + expm;
  Add12Cond(expm1h,expm1m,t1,t3);


  /* Rounding test */
  TEST_AND_RETURN_RD(expm1h, expm1m, ROUNDCSTCOMMONRD);
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
    expm1_common_td(&expm1h, &expm1m, &expm1l, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l, M); 
    
    /* Final rounding */

    ReturnRoundDownwards3(expm1h, expm1m, expm1l);
  } /* Accurate phase launched */
  
  /* We cannot be here since we return before in any case */
}

double expm1_ru(double x) {
  db_number xdb, shiftedXMultdb, polyTblhdb, polyTblmdb;
  int xIntHi, expoX, k, M, index1, index2;
  double highPoly, tt1h, t1h, t1l, xSqh, xSql, xSqHalfh, xSqHalfl, xCubeh, xCubel, t2h, t2l, templ, tt3h, tt3l;
  double polyh, polyl, expm1h, expm1m, expm1l;
  double r1h, r1l, r1t, rr1h, rr1l;
  double r2h, r2l, r2t, rr2h, rr2l;
  double r3h, r3l, r3t, rr3h, rr3l;
  double xMultLog2InvMult2L, shiftedXMult, kd, s1, s2, s3, s4, s5, rh, rm, rl;
  double rhSquare, rhC3, rhSquareHalf, monomialCube, rhFour, monomialFour;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;
  double highPolyWithSquare, tablesh, tablesl, t8, t9, t10, t11, t12, t13;
  double exph, expm, t1, t2, t3;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double middlePoly, doublePoly;

  xdb.d = x; 

  /* Strip off the sign of x for the following tests */

  xIntHi = xdb.i[HI] & 0x7fffffff;

  /* Test if we are so small that we can return (a corrected) x as correct rounding */
  if (xIntHi < RETURNXBOUND) {
    /* The only algebraic result is 0 for x = +/-0; in this case, we return x = +/-0
       The truncation rest x^2/2 + x^3/6 + ... is always positive 
       but less than 1 ulp in this case, so we round by adding 1 ulp 
    */
    
    if (x == 0.0) return x;

    if (xdb.i[HI] & 0x80000000) {
      /* x is negative 
	 We add 1 ulp by subtracting 1 in long
      */
      xdb.l--;
    } else {
      /* x is positive 
	 We add 1 ulp by adding 1 in long
      */
      xdb.l++;
    }
    return xdb.d;
  }


  /* Filter out special cases like overflow, -1 in result, infinities and NaNs 
     The filters are not sharp, we have positive arguments that flow through
  */
  if (xIntHi >= SIMPLEOVERFLOWBOUND) {
    /* Test if we are +/-inf or NaN */
    if (xIntHi >= 0x7ff00000) {
      /* Test if NaN */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* NaN */
	return x+x;  /* return NaN */
      }
      /* Test if +inf or -inf */
      if (xdb.i[HI] > 0) {
	/* +inf */
	return x+x;  /* return +inf */
      }
      
      /* If we are here, we are -inf */
      return -1.0;
    }

    /* If we are here, we are overflowed or a common case that flows through */

    /* Test if we are actually overflowed */
    if (x > OVERFLOWBOUND) {
      return LARGEST * LARGEST;  /* return +inf and set flag */
    }
  }
  
  /* Test if we know already that we are -1.0 (+ correction depending on rounding mode) in result */
  if (x < MINUSONEBOUND) {
    /* Round up so we are -1.0 + 1ulp */
    return MINUSONEPLUSONEULP;
  }

  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi < DIRECTINTERVALBOUND) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX = ((xIntHi & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb.i[HI] += (-expoX-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi = xdb.i[HI] & 0x7fffffff;
      x = xdb.d;
    }
    
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh,&xSql,x,x);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly = FMA(quickDirectpolyC5h,x,quickDirectpolyC4h);
#else
    middlePoly = quickDirectpolyC4h + x * quickDirectpolyC5h;
#endif

    doublePoly = middlePoly;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly = FMA(FMA(FMA(quickDirectpolyC9h ,x,quickDirectpolyC8h),x,
                             quickDirectpolyC7h),x,quickDirectpolyC6h);
#else
      highPoly = quickDirectpolyC6h + x * (quickDirectpolyC7h + x * (
	         quickDirectpolyC8h + x *  quickDirectpolyC9h));
#endif

      highPolyWithSquare = xSqh * highPoly;

      doublePoly = middlePoly + highPolyWithSquare;

    }
    
    /* Double-double evaluation steps */
    tt1h = x * doublePoly;

    xSqHalfh = 0.5 * xSqh;
    xSqHalfl = 0.5 * xSql;
    Add12(t2h,templ,x,xSqHalfh);
    t2l = templ + xSqHalfl;
    
    Add12(t1h,t1l,quickDirectpolyC3h,tt1h);
    Mul122(&xCubeh,&xCubel,x,xSqh,xSql);
    Mul22(&tt3h,&tt3l,xCubeh,xCubel,t1h,t1l);

    Add22(&polyh,&polyl,t2h,t2l,tt3h,tt3l);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h,r1t,2,polyh);
      r1l = r1t + polyl;
      Mul22(&rr1h,&rr1l,r1h,r1l,polyh,polyl);

      if (expoX >= 1) {

	/* Second reconstruction step */
	Add12(r2h,r2t,2,rr1h);
	r2l = r2t + rr1l;
	Mul22(&rr2h,&rr2l,r2h,r2l,rr1h,rr1l);

	if (expoX >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h,r3t,2,rr2h);
	  r3l = r3t + rr2l;
	  Mul22(&rr3h,&rr3l,r3h,r3l,rr2h,rr2l);
	  
	  /* expoX may be maximally 2 */

	  expm1h = rr3h;
	  expm1m = rr3l;

	} else {
	  expm1h = rr2h;
	  expm1m = rr2l;
	}

      } else {
	expm1h = rr1h;
	expm1m = rr1l;
      }

    } else {
      expm1h = polyh;
      expm1m = polyl;
    }

    /* Rounding test */
    TEST_AND_RETURN_RU(expm1h, expm1m, ROUNDCSTDIRECTRD);
    {
      expm1_direct_td(&expm1h, &expm1m, &expm1l, x, xSqHalfh, xSqHalfl, xSqh, xSql, expoX);
      
      ReturnRoundUpwards3(expm1h, expm1m, expm1l);

    } /* Accurate phase launched */

    /* We cannot be here, since we return in all cases before */
  }

  /* If we are here, we can use expm1(x) = exp(x) - 1 */

  /* Range reduction - exact part: compute k as double and as int */

  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  k = shiftedXMultdb.i[LO];
  M = k >> 12;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 6;

  /* Range reduction - part affected by error - must be redone in accurate phase */
  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  /* Table reads - read only two double-doubles by now */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = quickCommonpolyC3h * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = quickCommonpolyC4h * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;
  
  /* Reconstruction: integration of table values */
  
  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblhdb.d,polyTblmdb.d,t11,t13);
  
  /* Reconstruction: multiplication by 2^M */

  /* Implement the multiplication by multiplication to overcome the
     problem of the non-representability of 2^1024 (M = 1024)
     This case is possible if polyTblhdb.d < 1
  */
  
  polyTblhdb.i[HI] += M << 20;
  polyTblmdb.i[HI] += M << 20;

  exph = polyTblhdb.d;
  expm = polyTblmdb.d;

  /* Substraction of 1 

     Testing if the operation is necessary is more expensive than 
     performing it in any case.

     We may cancellate at most 2 bits in the subtraction for 
     arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
     We must therefore use conditional Add12s

     Since we perform a substraction, we may not have addition overflow towards +inf

  */

  Add12Cond(t1,t2,-1,exph);
  t3 = t2 + expm;
  Add12Cond(expm1h,expm1m,t1,t3);


  /* Rounding test */
  TEST_AND_RETURN_RU(expm1h, expm1m, ROUNDCSTCOMMONRD);
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
    expm1_common_td(&expm1h, &expm1m, &expm1l, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l, M); 
    
    /* Final rounding */

    ReturnRoundUpwards3(expm1h, expm1m, expm1l);
  } /* Accurate phase launched */
  
  /* We cannot be here since we return before in any case */
}
 
double expm1_rz(double x) {
  db_number xdb, shiftedXMultdb, polyTblhdb, polyTblmdb;
  int xIntHi, expoX, k, M, index1, index2;
  double highPoly, tt1h, t1h, t1l, xSqh, xSql, xSqHalfh, xSqHalfl, xCubeh, xCubel, t2h, t2l, templ, tt3h, tt3l;
  double polyh, polyl, expm1h, expm1m, expm1l;
  double r1h, r1l, r1t, rr1h, rr1l;
  double r2h, r2l, r2t, rr2h, rr2l;
  double r3h, r3l, r3t, rr3h, rr3l;
  double xMultLog2InvMult2L, shiftedXMult, kd, s1, s2, s3, s4, s5, rh, rm, rl;
  double rhSquare, rhC3, rhSquareHalf, monomialCube, rhFour, monomialFour;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;
  double highPolyWithSquare, tablesh, tablesl, t8, t9, t10, t11, t12, t13;
  double exph, expm, t1, t2, t3;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double middlePoly, doublePoly;

  xdb.d = x; 

  /* Strip off the sign of x for the following tests */

  xIntHi = xdb.i[HI] & 0x7fffffff;

  /* Test if we are so small that we can return (a corrected) x as correct rounding */
  if (xIntHi < RETURNXBOUND) {
    /* The only algebraic result is 0 for x = +/- 0; in this case, we can return x = +/- 0
       expm1 is positive for positive x, negative for negative x
       The truncation rest x^2/2 + x^3/6 + ... is always positive 
       but less than 1 ulp in this case, so we round as follows:
       
       - x is positive => expm1 is positive => round downwards => truncate by returning x
       - x is negative => expm1 is negative => round upwards => add 1 ulp
    */

    if (x == 0.0) return x;

    if (xdb.i[HI] & 0x80000000) {
      /* x is negative 
	 We add 1 ulp by subtracting 1 in long
      */
      xdb.l--;
      return xdb.d;
    } else {
      /* x is positive 
	 We do nothing (see above)
      */
      return x;
    }
  }


  /* Filter out special cases like overflow, -1 in result, infinities and NaNs 
     The filters are not sharp, we have positive arguments that flow through
  */
  if (xIntHi >= SIMPLEOVERFLOWBOUND) {
    /* Test if we are +/-inf or NaN */
    if (xIntHi >= 0x7ff00000) {
      /* Test if NaN */
      if (((xIntHi & 0x000fffff) | xdb.i[LO]) != 0) {
	/* NaN */
	return x+x;  /* return NaN */
      }
      /* Test if +inf or -inf */
      if (xdb.i[HI] > 0) {
	/* +inf */
	return x+x;  /* return +inf */
      }
      
      /* If we are here, we are -inf */
      return -1.0;
    }

    /* If we are here, we are overflowed or a common case that flows through */

    /* Test if we are actually overflowed */
    if (x > OVERFLOWBOUND) {
      /* We would be overflowed but as we are rounding towards zero, i.e. downwards,
	 the nearest number lesser than the exact result is the greatest 
	 normal. In any case, we must raise the inexact flag.
      */
      return LARGEST * (1.0 + SMALLEST);
    }
  }
  
  /* Test if we know already that we are -1.0 (+ correction depending on rounding mode) in result */
  if (x < MINUSONEBOUND) {
    /* We round towards zero, i.e. upwards, so we return -1.0+1ulp */
    return MINUSONEPLUSONEULP;
  }

  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi < DIRECTINTERVALBOUND) {

    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX = ((xIntHi & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb.i[HI] += (-expoX-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi = xdb.i[HI] & 0x7fffffff;
      x = xdb.d;
    }
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh,&xSql,x,x);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly = FMA(quickDirectpolyC5h,x,quickDirectpolyC4h);
#else
    middlePoly = quickDirectpolyC4h + x * quickDirectpolyC5h;
#endif

    doublePoly = middlePoly;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly = FMA(FMA(FMA(quickDirectpolyC9h ,x,quickDirectpolyC8h),x,
                             quickDirectpolyC7h),x,quickDirectpolyC6h);
#else
      highPoly = quickDirectpolyC6h + x * (quickDirectpolyC7h + x * (
	         quickDirectpolyC8h + x *  quickDirectpolyC9h));
#endif

      highPolyWithSquare = xSqh * highPoly;

      doublePoly = middlePoly + highPolyWithSquare;

    }
    
    /* Double-double evaluation steps */
    tt1h = x * doublePoly;

    xSqHalfh = 0.5 * xSqh;
    xSqHalfl = 0.5 * xSql;
    Add12(t2h,templ,x,xSqHalfh);
    t2l = templ + xSqHalfl;
    
    Add12(t1h,t1l,quickDirectpolyC3h,tt1h);
    Mul122(&xCubeh,&xCubel,x,xSqh,xSql);
    Mul22(&tt3h,&tt3l,xCubeh,xCubel,t1h,t1l);

    Add22(&polyh,&polyl,t2h,t2l,tt3h,tt3l);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h,r1t,2,polyh);
      r1l = r1t + polyl;
      Mul22(&rr1h,&rr1l,r1h,r1l,polyh,polyl);

      if (expoX >= 1) {

	/* Second reconstruction step */
	Add12(r2h,r2t,2,rr1h);
	r2l = r2t + rr1l;
	Mul22(&rr2h,&rr2l,r2h,r2l,rr1h,rr1l);

	if (expoX >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h,r3t,2,rr2h);
	  r3l = r3t + rr2l;
	  Mul22(&rr3h,&rr3l,r3h,r3l,rr2h,rr2l);
	  
	  /* expoX may be maximally 2 */

	  expm1h = rr3h;
	  expm1m = rr3l;

	} else {
	  expm1h = rr2h;
	  expm1m = rr2l;
	}

      } else {
	expm1h = rr1h;
	expm1m = rr1l;
      }

    } else {
      expm1h = polyh;
      expm1m = polyl;
    }

    /* Rounding test */
    TEST_AND_RETURN_RZ(expm1h, expm1m, ROUNDCSTDIRECTRD);
    {
      expm1_direct_td(&expm1h, &expm1m, &expm1l, x, xSqHalfh, xSqHalfl, xSqh, xSql, expoX);
      
      ReturnRoundTowardsZero3(expm1h, expm1m, expm1l);

    } /* Accurate phase launched */

    /* We cannot be here, since we return in all cases before */
  }

  /* If we are here, we can use expm1(x) = exp(x) - 1 */

  /* Range reduction - exact part: compute k as double and as int */

  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  k = shiftedXMultdb.i[LO];
  M = k >> 12;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> 6;

  /* Range reduction - part affected by error - must be redone in accurate phase */
  Mul12(&s1,&s2,msLog2Div2Lh,kd);
  s3 = kd * msLog2Div2Lm;
  s4 = s2 + s3; 
  s5 = x + s1;
  Add12Cond(rh,rm,s5,s4);

  /* Table reads - read only two double-doubles by now */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;

  /* Quick phase starts here */

  rhSquare = rh * rh;
  rhC3 = quickCommonpolyC3h * rh;

  rhSquareHalf = 0.5 * rhSquare;
  monomialCube = rhC3 * rhSquare;
  rhFour = rhSquare * rhSquare;

  monomialFour = quickCommonpolyC4h * rhFour;
  
  highPoly = monomialCube + monomialFour;

  highPolyWithSquare = rhSquareHalf + highPoly;
  
  /* Reconstruction: integration of table values */
  
  Mul22(&tablesh,&tablesl,tbl1h,tbl1m,tbl2h,tbl2m);

  t8 = rm + highPolyWithSquare;
  t9 = rh + t8;

  t10 = tablesh * t9;
  
  Add12(t11,t12,tablesh,t10);
  t13 = t12 + tablesl;
  Add12(polyTblhdb.d,polyTblmdb.d,t11,t13);
  
  /* Reconstruction: multiplication by 2^M */

  /* Implement the multiplication by multiplication to overcome the
     problem of the non-representability of 2^1024 (M = 1024)
     This case is possible if polyTblhdb.d < 1
  */
  
  polyTblhdb.i[HI] += M << 20;
  polyTblmdb.i[HI] += M << 20;

  exph = polyTblhdb.d;
  expm = polyTblmdb.d;

  /* Substraction of 1 

     Testing if the operation is necessary is more expensive than 
     performing it in any case.

     We may cancellate at most 2 bits in the subtraction for 
     arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
     We must therefore use conditional Add12s

     Since we perform a substraction, we may not have addition overflow towards +inf

  */

  Add12Cond(t1,t2,-1,exph);
  t3 = t2 + expm;
  Add12Cond(expm1h,expm1m,t1,t3);


  /* Rounding test */
  TEST_AND_RETURN_RZ(expm1h, expm1m, ROUNDCSTCOMMONRD);
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
    expm1_common_td(&expm1h, &expm1m, &expm1l, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l, M); 
    
    /* Final rounding */

    ReturnRoundTowardsZero3(expm1h, expm1m, expm1l);
  } /* Accurate phase launched */
  
  /* We cannot be here since we return before in any case */
}

#ifdef BUILD_INTERVAL_FUNCTIONS
interval j_expm1(interval x)
{
  interval res;
  db_number xdb_inf, shiftedXMultdb_inf, polyTblhdb_inf, polyTblmdb_inf;
  int xIntHi_inf, expoX_inf, k_inf, M_inf, index1_inf, index2_inf;
  double x_inf, x_sup;
  double res_inf,res_sup;
  double restemp_inf, restemp_sup;
  double highPoly_inf, tt1h_inf, t1h_inf, t1l_inf, xSqh_inf, xSql_inf, xSqHalfh_inf, xSqHalfl_inf, xCubeh_inf, xCubel_inf, t2h_inf, t2l_inf, templ_inf, tt3h_inf, tt3l_inf;
  double polyh_inf, polyl_inf, expm1h_inf, expm1m_inf, expm1l_inf;
  double r1h_inf, r1l_inf, r1t_inf, rr1h_inf, rr1l_inf;
  double r2h_inf, r2l_inf, r2t_inf, rr2h_inf, rr2l_inf;
  double r3h_inf, r3l_inf, r3t_inf, rr3h_inf, rr3l_inf;
  double xMultLog2InvMult2L_inf, shiftedXMult_inf, kd_inf, s1_inf, s2_inf, s3_inf, s4_inf, s5_inf, rh_inf, rm_inf, rl_inf;
  double rhSquare_inf, rhC3_inf, rhSquareHalf_inf, monomialCube_inf, rhFour_inf, monomialFour_inf;
  double tbl1h_inf, tbl1m_inf, tbl1l_inf, tbl2h_inf, tbl2m_inf, tbl2l_inf;
  double highPolyWithSquare_inf, tablesh_inf, tablesl_inf, t8_inf, t9_inf, t10_inf, t11_inf, t12_inf, t13_inf;
  double exph_inf, expm_inf, t1_inf, t2_inf, t3_inf;
  double msLog2Div2LMultKh_inf, msLog2Div2LMultKm_inf, msLog2Div2LMultKl_inf;
  double middlePoly_inf, doublePoly_inf;
  int roundable;
  int infDone, supDone;
  infDone=0; supDone=0;

  db_number xdb_sup, shiftedXMultdb_sup, polyTblhdb_sup, polyTblmdb_sup;
  int xIntHi_sup, expoX_sup, k_sup, M_sup, index1_sup, index2_sup;
  double highPoly_sup, tt1h_sup, t1h_sup, t1l_sup, xSqh_sup, xSql_sup, xSqHalfh_sup, xSqHalfl_sup, xCubeh_sup, xCubel_sup, t2h_sup, t2l_sup, templ_sup, tt3h_sup, tt3l_sup;
  double polyh_sup, polyl_sup, expm1h_sup, expm1m_sup, expm1l_sup;
  double r1h_sup, r1l_sup, r1t_sup, rr1h_sup, rr1l_sup;
  double r2h_sup, r2l_sup, r2t_sup, rr2h_sup, rr2l_sup;
  double r3h_sup, r3l_sup, r3t_sup, rr3h_sup, rr3l_sup;
  double xMultLog2InvMult2L_sup, shiftedXMult_sup, kd_sup, s1_sup, s2_sup, s3_sup, s4_sup, s5_sup, rh_sup, rm_sup, rl_sup;
  double rhSquare_sup, rhC3_sup, rhSquareHalf_sup, monomialCube_sup, rhFour_sup, monomialFour_sup;
  double tbl1h_sup, tbl1m_sup, tbl1l_sup, tbl2h_sup, tbl2m_sup, tbl2l_sup;
  double highPolyWithSquare_sup, tablesh_sup, tablesl_sup, t8_sup, t9_sup, t10_sup, t11_sup, t12_sup, t13_sup;
  double exph_sup, expm_sup, t1_sup, t2_sup, t3_sup;
  double msLog2Div2LMultKh_sup, msLog2Div2LMultKm_sup, msLog2Div2LMultKl_sup;
  double middlePoly_sup, doublePoly_sup;

  x_inf=LOW(x);
  x_sup=UP(x);

  xdb_inf.d = x_inf; 
  xdb_sup.d = x_sup; 


  /* Strip off the sign of x for the following tests */

  xIntHi_inf = xdb_inf.i[HI] & 0x7fffffff;
  xIntHi_sup = xdb_sup.i[HI] & 0x7fffffff;

  /* Test if we are so small that we can return (a corrected) x as correct rounding */

  if ( __builtin_expect(
      (xIntHi_inf < RETURNXBOUND) ||
      ((xIntHi_inf >= SIMPLEOVERFLOWBOUND) && (xIntHi_inf >= 0x7ff00000)) ||
      ((xIntHi_inf >= SIMPLEOVERFLOWBOUND) && (xIntHi_inf >= 0x7ff00000) && (((xIntHi_inf & 0x000fffff) | xdb_inf.i[LO]) != 0)) ||
      ((xIntHi_inf >= SIMPLEOVERFLOWBOUND) && (xIntHi_inf >= 0x7ff00000) && (xdb_inf.i[HI] > 0)) ||
      ((xIntHi_inf >= SIMPLEOVERFLOWBOUND) && (x_inf > OVERFLOWBOUND)) ||
      (x_inf < MINUSONEBOUND) ||
      (xIntHi_sup < RETURNXBOUND) ||
      ((xIntHi_sup < RETURNXBOUND) && (x_sup == 0.0)) ||
      ((xIntHi_sup >= SIMPLEOVERFLOWBOUND) && (xIntHi_sup >= 0x7ff00000)) ||
      ((xIntHi_sup >= SIMPLEOVERFLOWBOUND) && (xIntHi_sup >= 0x7ff00000) && (((xIntHi_sup & 0x000fffff) | xdb_sup.i[LO]) != 0)) ||
      ((xIntHi_sup >= SIMPLEOVERFLOWBOUND) && (xIntHi_sup >= 0x7ff00000) && (xdb_sup.i[HI] > 0)) ||
      ((xIntHi_sup >= SIMPLEOVERFLOWBOUND) && (x_sup > OVERFLOWBOUND)) ||
      (x_sup < MINUSONEBOUND)
     ,FALSE))
  {
    ASSIGN_LOW(res,expm1_rd(LOW(x)));
    ASSIGN_UP(res,expm1_ru(UP(x)));
    return res;
  }

  if (__builtin_expect((xIntHi_inf < DIRECTINTERVALBOUND) && (xIntHi_sup < DIRECTINTERVALBOUND),TRUE)) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX_inf = ((xIntHi_inf & 0x7ff00000) >> 20) - (1023 - 5);
    expoX_sup = ((xIntHi_sup & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX_inf >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb_inf.i[HI] += (-expoX_inf-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi_inf = xdb_inf.i[HI] & 0x7fffffff;
      x_inf = xdb_inf.d;
    }

    if (expoX_sup >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb_sup.i[HI] += (-expoX_sup-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi_sup = xdb_sup.i[HI] & 0x7fffffff;
      x_sup = xdb_sup.d;
    }

    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh_inf,&xSql_inf,x_inf,x_inf);
    Mul12(&xSqh_sup,&xSql_sup,x_sup,x_sup);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly_inf = FMA(quickDirectpolyC5h,x_inf,quickDirectpolyC4h);
    middlePoly_sup = FMA(quickDirectpolyC5h,x_sup,quickDirectpolyC4h);
#else
    middlePoly_inf = quickDirectpolyC4h + x_inf * quickDirectpolyC5h;
    middlePoly_sup = quickDirectpolyC4h + x_sup * quickDirectpolyC5h;
#endif

    doublePoly_inf = middlePoly_inf;
    doublePoly_sup = middlePoly_sup;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi_inf > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly_inf = FMA(FMA(FMA(quickDirectpolyC9h ,x_inf,quickDirectpolyC8h),x_inf,
                             quickDirectpolyC7h),x_inf,quickDirectpolyC6h);
#else
      highPoly_inf = quickDirectpolyC6h + x_inf * (quickDirectpolyC7h + x_inf * (
	         quickDirectpolyC8h + x_inf *  quickDirectpolyC9h));
#endif

      highPolyWithSquare_inf = xSqh_inf * highPoly_inf;

      doublePoly_inf = middlePoly_inf + highPolyWithSquare_inf;

    }
    if (xIntHi_sup > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly_sup = FMA(FMA(FMA(quickDirectpolyC9h ,x_sup,quickDirectpolyC8h),x_sup,
                             quickDirectpolyC7h),x_sup,quickDirectpolyC6h);
#else
      highPoly_sup = quickDirectpolyC6h + x_sup * (quickDirectpolyC7h + x_sup * (
	         quickDirectpolyC8h + x_sup *  quickDirectpolyC9h));
#endif

      highPolyWithSquare_sup = xSqh_sup * highPoly_sup;

      doublePoly_sup = middlePoly_sup + highPolyWithSquare_sup;

    }
    
    /* Double-double evaluation steps */
    tt1h_inf = x_inf * doublePoly_inf;
    tt1h_sup = x_sup * doublePoly_sup;

    xSqHalfh_inf = 0.5 * xSqh_inf;
    xSqHalfh_sup = 0.5 * xSqh_sup;
    xSqHalfl_inf = 0.5 * xSql_inf;
    xSqHalfl_sup = 0.5 * xSql_sup;
    Add12(t2h_inf,templ_inf,x_inf,xSqHalfh_inf);
    Add12(t2h_sup,templ_sup,x_sup,xSqHalfh_sup);
    t2l_inf = templ_inf + xSqHalfl_inf;
    t2l_sup = templ_sup + xSqHalfl_sup;
    
    Add12(t1h_inf,t1l_inf,quickDirectpolyC3h,tt1h_inf);
    Add12(t1h_sup,t1l_sup,quickDirectpolyC3h,tt1h_sup);
    Mul122(&xCubeh_inf,&xCubel_inf,x_inf,xSqh_inf,xSql_inf);
    Mul122(&xCubeh_sup,&xCubel_sup,x_sup,xSqh_sup,xSql_sup);
    Mul22(&tt3h_inf,&tt3l_inf,xCubeh_inf,xCubel_inf,t1h_inf,t1l_inf);
    Mul22(&tt3h_sup,&tt3l_sup,xCubeh_sup,xCubel_sup,t1h_sup,t1l_sup);

    Add22(&polyh_inf,&polyl_inf,t2h_inf,t2l_inf,tt3h_inf,tt3l_inf);
    Add22(&polyh_sup,&polyl_sup,t2h_sup,t2l_sup,tt3h_sup,tt3l_sup);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX_inf >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h_inf,r1t_inf,2,polyh_inf);
      r1l_inf = r1t_inf + polyl_inf;
      Mul22(&rr1h_inf,&rr1l_inf,r1h_inf,r1l_inf,polyh_inf,polyl_inf);

      if (expoX_inf >= 1) {

	/* Second reconstruction step */
	Add12(r2h_inf,r2t_inf,2,rr1h_inf);
	r2l_inf = r2t_inf + rr1l_inf;
	Mul22(&rr2h_inf,&rr2l_inf,r2h_inf,r2l_inf,rr1h_inf,rr1l_inf);

	if (expoX_inf >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h_inf,r3t_inf,2,rr2h_inf);
	  r3l_inf = r3t_inf + rr2l_inf;
	  Mul22(&rr3h_inf,&rr3l_inf,r3h_inf,r3l_inf,rr2h_inf,rr2l_inf);
	  
	  /* expoX may be maximally 2 */

	  expm1h_inf = rr3h_inf;
	  expm1m_inf = rr3l_inf;

	} else {
	  expm1h_inf = rr2h_inf;
	  expm1m_inf = rr2l_inf;
	}

      } else {
	expm1h_inf = rr1h_inf;
	expm1m_inf = rr1l_inf;
      }

    } else {
      expm1h_inf = polyh_inf;
      expm1m_inf = polyl_inf;
    }
    if (expoX_sup >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h_sup,r1t_sup,2,polyh_sup);
      r1l_sup = r1t_sup + polyl_sup;
      Mul22(&rr1h_sup,&rr1l_sup,r1h_sup,r1l_sup,polyh_sup,polyl_sup);

      if (expoX_sup >= 1) {

	/* Second reconstruction step */
	Add12(r2h_sup,r2t_sup,2,rr1h_sup);
	r2l_sup = r2t_sup + rr1l_sup;
	Mul22(&rr2h_sup,&rr2l_sup,r2h_sup,r2l_sup,rr1h_sup,rr1l_sup);

	if (expoX_sup >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h_sup,r3t_sup,2,rr2h_sup);
	  r3l_sup = r3t_sup + rr2l_sup;
	  Mul22(&rr3h_sup,&rr3l_sup,r3h_sup,r3l_sup,rr2h_sup,rr2l_sup);
	  
	  /* expoX may be maximally 2 */

	  expm1h_sup = rr3h_sup;
	  expm1m_sup = rr3l_sup;

	} else {
	  expm1h_sup = rr2h_sup;
	  expm1m_sup = rr2l_sup;
	}

      } else {
	expm1h_sup = rr1h_sup;
	expm1m_sup = rr1l_sup;
      }

    } else {
      expm1h_sup = polyh_sup;
      expm1m_sup = polyl_sup;
    }


    /* Rounding test */
    TEST_AND_COPY_RDRU_EXPM1(roundable,restemp_inf,expm1h_inf, expm1m_inf, restemp_sup,expm1h_sup, expm1m_sup, ROUNDCSTDIRECTRD);
    if((roundable==2) || (roundable==0))
    {
      expm1_direct_td(&expm1h_inf, &expm1m_inf, &expm1l_inf, x_inf, xSqHalfh_inf, xSqHalfl_inf, xSqh_inf, xSql_inf, expoX_inf);
      
      RoundDownwards3(&restemp_inf,expm1h_inf, expm1m_inf, expm1l_inf);
      
    } /* Accurate phase launched */
    if((roundable==1) || (roundable==0))
    {
      expm1_direct_td(&expm1h_sup, &expm1m_sup, &expm1l_sup, x_sup, xSqHalfh_sup, xSqHalfl_sup, xSqh_sup, xSql_sup, expoX_sup);
      
      RoundUpwards3(&restemp_sup,expm1h_sup, expm1m_sup, expm1l_sup);

    } /* Accurate phase launched */
    ASSIGN_LOW(res,restemp_inf);
    ASSIGN_UP(res,restemp_sup);
    return res;
  }


  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi_inf < DIRECTINTERVALBOUND) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX_inf = ((xIntHi_inf & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX_inf >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb_inf.i[HI] += (-expoX_inf-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi_inf = xdb_inf.i[HI] & 0x7fffffff;
      x_inf = xdb_inf.d;
    }
    
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh_inf,&xSql_inf,x_inf,x_inf);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly_inf = FMA(quickDirectpolyC5h,x_inf,quickDirectpolyC4h);
#else
    middlePoly_inf = quickDirectpolyC4h + x_inf * quickDirectpolyC5h;
#endif

    doublePoly_inf = middlePoly_inf;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi_inf > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly_inf = FMA(FMA(FMA(quickDirectpolyC9h ,x_inf,quickDirectpolyC8h),x_inf,
                             quickDirectpolyC7h),x_inf,quickDirectpolyC6h);
#else
      highPoly_inf = quickDirectpolyC6h + x_inf * (quickDirectpolyC7h + x_inf * (
	         quickDirectpolyC8h + x_inf *  quickDirectpolyC9h));
#endif

      highPolyWithSquare_inf = xSqh_inf * highPoly_inf;

      doublePoly_inf = middlePoly_inf + highPolyWithSquare_inf;

    }
    
    /* Double-double evaluation steps */
    tt1h_inf = x_inf * doublePoly_inf;

    xSqHalfh_inf = 0.5 * xSqh_inf;
    xSqHalfl_inf = 0.5 * xSql_inf;
    Add12(t2h_inf,templ_inf,x_inf,xSqHalfh_inf);
    t2l_inf = templ_inf + xSqHalfl_inf;
    
    Add12(t1h_inf,t1l_inf,quickDirectpolyC3h,tt1h_inf);
    Mul122(&xCubeh_inf,&xCubel_inf,x_inf,xSqh_inf,xSql_inf);
    Mul22(&tt3h_inf,&tt3l_inf,xCubeh_inf,xCubel_inf,t1h_inf,t1l_inf);

    Add22(&polyh_inf,&polyl_inf,t2h_inf,t2l_inf,tt3h_inf,tt3l_inf);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX_inf >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h_inf,r1t_inf,2,polyh_inf);
      r1l_inf = r1t_inf + polyl_inf;
      Mul22(&rr1h_inf,&rr1l_inf,r1h_inf,r1l_inf,polyh_inf,polyl_inf);

      if (expoX_inf >= 1) {

	/* Second reconstruction step */
	Add12(r2h_inf,r2t_inf,2,rr1h_inf);
	r2l_inf = r2t_inf + rr1l_inf;
	Mul22(&rr2h_inf,&rr2l_inf,r2h_inf,r2l_inf,rr1h_inf,rr1l_inf);

	if (expoX_inf >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h_inf,r3t_inf,2,rr2h_inf);
	  r3l_inf = r3t_inf + rr2l_inf;
	  Mul22(&rr3h_inf,&rr3l_inf,r3h_inf,r3l_inf,rr2h_inf,rr2l_inf);
	  
	  /* expoX may be maximally 2 */

	  expm1h_inf = rr3h_inf;
	  expm1m_inf = rr3l_inf;

	} else {
	  expm1h_inf = rr2h_inf;
	  expm1m_inf = rr2l_inf;
	}

      } else {
	expm1h_inf = rr1h_inf;
	expm1m_inf = rr1l_inf;
      }

    } else {
      expm1h_inf = polyh_inf;
      expm1m_inf = polyl_inf;
    }

    /* Rounding test */
    infDone=1;
    TEST_AND_COPY_RD(roundable,restemp_inf,expm1h_inf, expm1m_inf, ROUNDCSTDIRECTRD);
    if(roundable==0)
    {
      expm1_direct_td(&expm1h_inf, &expm1m_inf, &expm1l_inf, x_inf, xSqHalfh_inf, xSqHalfl_inf, xSqh_inf, xSql_inf, expoX_inf);
      
      RoundDownwards3(&restemp_inf,expm1h_inf, expm1m_inf, expm1l_inf);
      
    } /* Accurate phase launched */

  }

  /* Test if we have |x| <= 1/4-1/2ulp(1/4) for knowing if we use exp(x) or approximate directly */

  if (xIntHi_sup < DIRECTINTERVALBOUND) {
    /* We approximate expm1 directly after a range reduction as follows

       expm1(x) = (expm1(x/2) + 2) * expm1(x/2)

       We perform the range reduction in such a way that finally |x| < 1/32 
    */

    /* Extract the exponent of |x| and add 5 (2^5 = 32) */
    expoX_sup = ((xIntHi_sup & 0x7ff00000) >> 20) - (1023 - 5);
    
    /* If this particularily biased exponent expoX is negative, we are already less than 1/32 */
    if (expoX_sup >= 0) {
      /* If we are here, we must perform range reduction */


      /* We multiply x by 2^(-expoX-1) by bit manipulation 
	 x cannot be denormalized so there is no danger
      */
      xdb_sup.i[HI] += (-expoX_sup-1) << 20;

      /* We reassign the new x and maintain xIntHi */

      xIntHi_sup = xdb_sup.i[HI] & 0x7fffffff;
      x_sup = xdb_sup.d;
    }
    
    /* Here, we have always |x| < 1/32 */


    /* Double precision evaluation steps and one double-double step */

    Mul12(&xSqh_sup,&xSql_sup,x_sup,x_sup);

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
    middlePoly_sup = FMA(quickDirectpolyC5h,x_sup,quickDirectpolyC4h);
#else
    middlePoly_sup = quickDirectpolyC4h + x_sup * quickDirectpolyC5h;
#endif

    doublePoly_sup = middlePoly_sup;

    /* Special path: for small |x| we can truncate the polynomial */

    if (xIntHi_sup > SPECIALINTERVALBOUND) {

#if defined(PROCESSOR_HAS_FMA) && !defined(AVOID_FMA)
      highPoly_sup = FMA(FMA(FMA(quickDirectpolyC9h ,x_sup,quickDirectpolyC8h),x_sup,
                             quickDirectpolyC7h),x_sup,quickDirectpolyC6h);
#else
      highPoly_sup = quickDirectpolyC6h + x_sup * (quickDirectpolyC7h + x_sup * (
	         quickDirectpolyC8h + x_sup *  quickDirectpolyC9h));
#endif

      highPolyWithSquare_sup = xSqh_sup * highPoly_sup;

      doublePoly_sup = middlePoly_sup + highPolyWithSquare_sup;

    }
    
    /* Double-double evaluation steps */
    tt1h_sup = x_sup * doublePoly_sup;

    xSqHalfh_sup = 0.5 * xSqh_sup;
    xSqHalfl_sup = 0.5 * xSql_sup;
    Add12(t2h_sup,templ_sup,x_sup,xSqHalfh_sup);
    t2l_sup = templ_sup + xSqHalfl_sup;
    
    Add12(t1h_sup,t1l_sup,quickDirectpolyC3h,tt1h_sup);
    Mul122(&xCubeh_sup,&xCubel_sup,x_sup,xSqh_sup,xSql_sup);
    Mul22(&tt3h_sup,&tt3l_sup,xCubeh_sup,xCubel_sup,t1h_sup,t1l_sup);

    Add22(&polyh_sup,&polyl_sup,t2h_sup,t2l_sup,tt3h_sup,tt3l_sup);

    /* Reconstruction */

    /* If we have not performed any range reduction, we have no reconstruction to do */
    if (expoX_sup >= 0) {
      /* If we are here, we must perform reconstruction */

      /* First reconstruction step */
      Add12(r1h_sup,r1t_sup,2,polyh_sup);
      r1l_sup = r1t_sup + polyl_sup;
      Mul22(&rr1h_sup,&rr1l_sup,r1h_sup,r1l_sup,polyh_sup,polyl_sup);

      if (expoX_sup >= 1) {

	/* Second reconstruction step */
	Add12(r2h_sup,r2t_sup,2,rr1h_sup);
	r2l_sup = r2t_sup + rr1l_sup;
	Mul22(&rr2h_sup,&rr2l_sup,r2h_sup,r2l_sup,rr1h_sup,rr1l_sup);

	if (expoX_sup >= 2) {

	  /* Third reconstruction step */
	  Add12(r3h_sup,r3t_sup,2,rr2h_sup);
	  r3l_sup = r3t_sup + rr2l_sup;
	  Mul22(&rr3h_sup,&rr3l_sup,r3h_sup,r3l_sup,rr2h_sup,rr2l_sup);
	  
	  /* expoX may be maximally 2 */

	  expm1h_sup = rr3h_sup;
	  expm1m_sup = rr3l_sup;

	} else {
	  expm1h_sup = rr2h_sup;
	  expm1m_sup = rr2l_sup;
	}

      } else {
	expm1h_sup = rr1h_sup;
	expm1m_sup = rr1l_sup;
      }

    } else {
      expm1h_sup = polyh_sup;
      expm1m_sup = polyl_sup;
    }

    /* Rounding test */
    supDone=1;
    TEST_AND_COPY_RU(roundable,restemp_sup,expm1h_sup, expm1m_sup, ROUNDCSTDIRECTRD);
    if(roundable==0)
    {
      expm1_direct_td(&expm1h_sup, &expm1m_sup, &expm1l_sup, x_sup, xSqHalfh_sup, xSqHalfl_sup, xSqh_sup, xSql_sup, expoX_sup);
      
      RoundUpwards3(&restemp_sup,expm1h_sup, expm1m_sup, expm1l_sup);

    } /* Accurate phase launched */

  }
  if((infDone==0) && (supDone==0))
  {
    /* If we are here, we can use expm1(x) = exp(x) - 1 */

    /* Range reduction - exact part: compute k as double and as int */

    xMultLog2InvMult2L_inf = x_inf * log2InvMult2L;
    xMultLog2InvMult2L_sup = x_sup * log2InvMult2L;
    shiftedXMult_inf = xMultLog2InvMult2L_inf + shiftConst;
    shiftedXMult_sup = xMultLog2InvMult2L_sup + shiftConst;
    kd_inf = shiftedXMult_inf - shiftConst;
    kd_sup = shiftedXMult_sup - shiftConst;
    shiftedXMultdb_inf.d = shiftedXMult_inf;
    shiftedXMultdb_sup.d = shiftedXMult_sup;
    k_inf = shiftedXMultdb_inf.i[LO];
    k_sup = shiftedXMultdb_sup.i[LO];
    M_inf = k_inf >> 12;
    M_sup = k_sup >> 12;
    index1_inf = k_inf & INDEXMASK1;
    index1_sup = k_sup & INDEXMASK1;
    index2_inf = (k_inf & INDEXMASK2) >> 6;
    index2_sup = (k_sup & INDEXMASK2) >> 6;


    /* Range reduction - part affected by error - must be redone in accurate phase */
    Mul12(&s1_inf,&s2_inf,msLog2Div2Lh,kd_inf);
    Mul12(&s1_sup,&s2_sup,msLog2Div2Lh,kd_sup);
    s3_inf = kd_inf * msLog2Div2Lm;
    s3_sup = kd_sup * msLog2Div2Lm;
    s4_inf = s2_inf + s3_inf; 
    s4_sup = s2_sup + s3_sup; 
    s5_inf = x_inf + s1_inf;
    s5_sup = x_sup + s1_sup;
    Add12Cond(rh_inf,rm_inf,s5_inf,s4_inf);
    Add12Cond(rh_sup,rm_sup,s5_sup,s4_sup);


    /* Table reads - read only two double-doubles by now */
    tbl1h_inf = twoPowerIndex1[index1_inf].hi;
    tbl1h_sup = twoPowerIndex1[index1_sup].hi;
    tbl1m_inf = twoPowerIndex1[index1_inf].mi;
    tbl1m_sup = twoPowerIndex1[index1_sup].mi;
    tbl2h_inf = twoPowerIndex2[index2_inf].hi;
    tbl2h_sup = twoPowerIndex2[index2_sup].hi;
    tbl2m_inf = twoPowerIndex2[index2_inf].mi;
    tbl2m_sup = twoPowerIndex2[index2_sup].mi;

    /* Quick phase starts here */

    rhSquare_inf = rh_inf * rh_inf;
    rhSquare_sup = rh_sup * rh_sup;
    rhC3_inf = quickCommonpolyC3h * rh_inf;
    rhC3_sup = quickCommonpolyC3h * rh_sup;

    rhSquareHalf_inf = 0.5 * rhSquare_inf;
    rhSquareHalf_sup = 0.5 * rhSquare_sup;

    monomialCube_inf = rhC3_inf * rhSquare_inf;
    monomialCube_sup = rhC3_sup * rhSquare_sup;
    rhFour_inf = rhSquare_inf * rhSquare_inf;
    rhFour_sup = rhSquare_sup * rhSquare_sup;

    monomialFour_inf = quickCommonpolyC4h * rhFour_inf;
    monomialFour_sup = quickCommonpolyC4h * rhFour_sup;
    highPoly_inf = monomialCube_inf + monomialFour_inf;
    highPoly_sup = monomialCube_sup + monomialFour_sup;
    highPolyWithSquare_inf = rhSquareHalf_inf + highPoly_inf;
    highPolyWithSquare_sup = rhSquareHalf_sup + highPoly_sup;

    /* Reconstruction: integration of table values */
  
    Mul22(&tablesh_inf,&tablesl_inf,tbl1h_inf,tbl1m_inf,tbl2h_inf,tbl2m_inf);
    Mul22(&tablesh_sup,&tablesl_sup,tbl1h_sup,tbl1m_sup,tbl2h_sup,tbl2m_sup);

    t8_inf = rm_inf + highPolyWithSquare_inf;
    t8_sup = rm_sup + highPolyWithSquare_sup;
    t9_inf = rh_inf + t8_inf;
    t9_sup = rh_sup + t8_sup;

    t10_inf = tablesh_inf * t9_inf;
    t10_sup = tablesh_sup * t9_sup;

    Add12(t11_inf,t12_inf,tablesh_inf,t10_inf);
    Add12(t11_sup,t12_sup,tablesh_sup,t10_sup);
    t13_inf = t12_inf + tablesl_inf;
    t13_sup = t12_sup + tablesl_sup;
    Add12(polyTblhdb_inf.d,polyTblmdb_inf.d,t11_inf,t13_inf);
    Add12(polyTblhdb_sup.d,polyTblmdb_sup.d,t11_sup,t13_sup);

    /* Reconstruction: multiplication by 2^M */

    /* Implement the multiplication by multiplication to overcome the
       problem of the non-representability of 2^1024 (M = 1024)
       This case is possible if polyTblhdb.d < 1
    */
  
    polyTblhdb_inf.i[HI] += M_inf << 20;
    polyTblhdb_sup.i[HI] += M_sup << 20;
    polyTblmdb_inf.i[HI] += M_inf << 20;
    polyTblmdb_sup.i[HI] += M_sup << 20;

    exph_inf = polyTblhdb_inf.d;
    exph_sup = polyTblhdb_sup.d;
    expm_inf = polyTblmdb_inf.d;
    expm_sup = polyTblmdb_sup.d;

    /* Substraction of 1 

       Testing if the operation is necessary is more expensive than 
       performing it in any case.

       We may cancellate at most 2 bits in the subtraction for 
       arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
       We must therefore use conditional Add12s

       Since we perform a substraction, we may not have addition overflow towards +inf

    */

    Add12Cond(t1_inf,t2_inf,-1,exph_inf);
    Add12Cond(t1_sup,t2_sup,-1,exph_sup);
    t3_inf = t2_inf + expm_inf;
    t3_sup = t2_sup + expm_sup;
    Add12Cond(expm1h_inf,expm1m_inf,t1_inf,t3_inf);
    Add12Cond(expm1h_sup,expm1m_sup,t1_sup,t3_sup);


    /* Rounding test */
    TEST_AND_COPY_RDRU_EXPM1(roundable,res_inf,expm1h_inf, expm1m_inf, res_sup,expm1h_sup, expm1m_sup, ROUNDCSTCOMMONRD);
    if((roundable==2) || (roundable==0))
    {
      /* Rest of argument reduction for accurate phase */
    
      Mul133(&msLog2Div2LMultKh_inf,&msLog2Div2LMultKm_inf,&msLog2Div2LMultKl_inf,kd_inf,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1_inf = x_inf + msLog2Div2LMultKh_inf;
      Add12Cond(rh_inf,t2_inf,t1_inf,msLog2Div2LMultKm_inf);
      Add12Cond(rm_inf,rl_inf,t2_inf,msLog2Div2LMultKl_inf);
    
      /* Table reads for accurate phase */
      tbl1l_inf = twoPowerIndex1[index1_inf].lo;
      tbl2l_inf = twoPowerIndex2[index2_inf].lo;
    
      /* Call accurate phase */
      expm1_common_td(&expm1h_inf, &expm1m_inf, &expm1l_inf, rh_inf, rm_inf, rl_inf, tbl1h_inf, tbl1m_inf, tbl1l_inf, tbl2h_inf, tbl2m_inf, tbl2l_inf, M_inf); 
    
      /* Final rounding */

      RoundDownwards3(&res_inf,expm1h_inf, expm1m_inf, expm1l_inf);
    } /* Accurate phase launched */
    if((roundable==1) || (roundable==0))
    {
      /* Rest of argument reduction for accurate phase */
      Mul133(&msLog2Div2LMultKh_sup,&msLog2Div2LMultKm_sup,&msLog2Div2LMultKl_sup,kd_sup,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1_sup = x_sup + msLog2Div2LMultKh_sup;
      Add12Cond(rh_sup,t2_sup,t1_sup,msLog2Div2LMultKm_sup);
      Add12Cond(rm_sup,rl_sup,t2_sup,msLog2Div2LMultKl_sup);
    
      /* Table reads for accurate phase */
      tbl1l_sup = twoPowerIndex1[index1_sup].lo;
      tbl2l_sup = twoPowerIndex2[index2_sup].lo;
    
      /* Call accurate phase */
      expm1_common_td(&expm1h_sup, &expm1m_sup, &expm1l_sup, rh_sup, rm_sup, rl_sup, tbl1h_sup, tbl1m_sup, tbl1l_sup, tbl2h_sup, tbl2m_sup, tbl2l_sup, M_sup); 
    
      /* Final rounding */

      RoundUpwards3(&res_sup,expm1h_sup, expm1m_sup, expm1l_sup);
    } /* Accurate phase launched */

    ASSIGN_LOW(res,res_inf);
    ASSIGN_UP(res,res_sup);
    return res;
  }
  if((supDone==1))
  {
    /* If we are here, we can use expm1(x) = exp(x) - 1 */

    /* Range reduction - exact part: compute k as double and as int */

    xMultLog2InvMult2L_inf = x_inf * log2InvMult2L;
    shiftedXMult_inf = xMultLog2InvMult2L_inf + shiftConst;
    kd_inf = shiftedXMult_inf - shiftConst;
    shiftedXMultdb_inf.d = shiftedXMult_inf;
    k_inf = shiftedXMultdb_inf.i[LO];
    M_inf = k_inf >> 12;
    index1_inf = k_inf & INDEXMASK1;
    index2_inf = (k_inf & INDEXMASK2) >> 6;


    /* Range reduction - part affected by error - must be redone in accurate phase */
    Mul12(&s1_inf,&s2_inf,msLog2Div2Lh,kd_inf);
    s3_inf = kd_inf * msLog2Div2Lm;
    s4_inf = s2_inf + s3_inf; 
    s5_inf = x_inf + s1_inf;
    Add12Cond(rh_inf,rm_inf,s5_inf,s4_inf);


    /* Table reads - read only two double-doubles by now */
    tbl1h_inf = twoPowerIndex1[index1_inf].hi;
    tbl1m_inf = twoPowerIndex1[index1_inf].mi;
    tbl2h_inf = twoPowerIndex2[index2_inf].hi;
    tbl2m_inf = twoPowerIndex2[index2_inf].mi;

    /* Quick phase starts here */

    rhSquare_inf = rh_inf * rh_inf;
    rhC3_inf = quickCommonpolyC3h * rh_inf;

    rhSquareHalf_inf = 0.5 * rhSquare_inf;

    monomialCube_inf = rhC3_inf * rhSquare_inf;
    rhFour_inf = rhSquare_inf * rhSquare_inf;

    monomialFour_inf = quickCommonpolyC4h * rhFour_inf;
    highPoly_inf = monomialCube_inf + monomialFour_inf;
    highPolyWithSquare_inf = rhSquareHalf_inf + highPoly_inf;

    /* Reconstruction: integration of table values */
  
    Mul22(&tablesh_inf,&tablesl_inf,tbl1h_inf,tbl1m_inf,tbl2h_inf,tbl2m_inf);

    t8_inf = rm_inf + highPolyWithSquare_inf;
    t9_inf = rh_inf + t8_inf;

    t10_inf = tablesh_inf * t9_inf;

    Add12(t11_inf,t12_inf,tablesh_inf,t10_inf);
    t13_inf = t12_inf + tablesl_inf;
    Add12(polyTblhdb_inf.d,polyTblmdb_inf.d,t11_inf,t13_inf);

    /* Reconstruction: multiplication by 2^M */

    /* Implement the multiplication by multiplication to overcome the
       problem of the non-representability of 2^1024 (M = 1024)
       This case is possible if polyTblhdb.d < 1
    */
  
    polyTblhdb_inf.i[HI] += M_inf << 20;
    polyTblmdb_inf.i[HI] += M_inf << 20;

    exph_inf = polyTblhdb_inf.d;
    expm_inf = polyTblmdb_inf.d;

    /* Substraction of 1 

       Testing if the operation is necessary is more expensive than 
       performing it in any case.

       We may cancellate at most 2 bits in the subtraction for 
       arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
       We must therefore use conditional Add12s

       Since we perform a substraction, we may not have addition overflow towards +inf

    */

    Add12Cond(t1_inf,t2_inf,-1,exph_inf);
    t3_inf = t2_inf + expm_inf;
    Add12Cond(expm1h_inf,expm1m_inf,t1_inf,t3_inf);


    /* Rounding test */
    TEST_AND_COPY_RD(roundable,res_inf,expm1h_inf, expm1m_inf, ROUNDCSTCOMMONRD);
    if(roundable==0)
    {
      /* Rest of argument reduction for accurate phase */
    
      Mul133(&msLog2Div2LMultKh_inf,&msLog2Div2LMultKm_inf,&msLog2Div2LMultKl_inf,kd_inf,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1_inf = x_inf + msLog2Div2LMultKh_inf;
      Add12Cond(rh_inf,t2_inf,t1_inf,msLog2Div2LMultKm_inf);
      Add12Cond(rm_inf,rl_inf,t2_inf,msLog2Div2LMultKl_inf);
    
      /* Table reads for accurate phase */
      tbl1l_inf = twoPowerIndex1[index1_inf].lo;
      tbl2l_inf = twoPowerIndex2[index2_inf].lo;
    
      /* Call accurate phase */
      expm1_common_td(&expm1h_inf, &expm1m_inf, &expm1l_inf, rh_inf, rm_inf, rl_inf, tbl1h_inf, tbl1m_inf, tbl1l_inf, tbl2h_inf, tbl2m_inf, tbl2l_inf, M_inf); 
    
      /* Final rounding */

      RoundDownwards3(&res_inf,expm1h_inf, expm1m_inf, expm1l_inf);
    } /* Accurate phase launched */

  }
  if(infDone==1)
  {
    /* If we are here, we can use expm1(x) = exp(x) - 1 */

    /* Range reduction - exact part: compute k as double and as int */

    xMultLog2InvMult2L_sup = x_sup * log2InvMult2L;
    shiftedXMult_sup = xMultLog2InvMult2L_sup + shiftConst;
    kd_sup = shiftedXMult_sup - shiftConst;
    shiftedXMultdb_sup.d = shiftedXMult_sup;
    k_sup = shiftedXMultdb_sup.i[LO];
    M_sup = k_sup >> 12;
    index1_sup = k_sup & INDEXMASK1;
    index2_sup = (k_sup & INDEXMASK2) >> 6;


    /* Range reduction - part affected by error - must be redone in accurate phase */
    Mul12(&s1_sup,&s2_sup,msLog2Div2Lh,kd_sup);
    s3_sup = kd_sup * msLog2Div2Lm;
    s4_sup = s2_sup + s3_sup; 
    s5_sup = x_sup + s1_sup;
    Add12Cond(rh_sup,rm_sup,s5_sup,s4_sup);


    /* Table reads - read only two double-doubles by now */
    tbl1h_sup = twoPowerIndex1[index1_sup].hi;
    tbl1m_sup = twoPowerIndex1[index1_sup].mi;
    tbl2h_sup = twoPowerIndex2[index2_sup].hi;
    tbl2m_sup = twoPowerIndex2[index2_sup].mi;

    /* Quick phase starts here */

    rhSquare_sup = rh_sup * rh_sup;
    rhC3_sup = quickCommonpolyC3h * rh_sup;

    rhSquareHalf_sup = 0.5 * rhSquare_sup;

    monomialCube_sup = rhC3_sup * rhSquare_sup;
    rhFour_sup = rhSquare_sup * rhSquare_sup;

    monomialFour_sup = quickCommonpolyC4h * rhFour_sup;
    highPoly_sup = monomialCube_sup + monomialFour_sup;
    highPolyWithSquare_sup = rhSquareHalf_sup + highPoly_sup;

    /* Reconstruction: integration of table values */
  
    Mul22(&tablesh_sup,&tablesl_sup,tbl1h_sup,tbl1m_sup,tbl2h_sup,tbl2m_sup);

    t8_sup = rm_sup + highPolyWithSquare_sup;
    t9_sup = rh_sup + t8_sup;

    t10_sup = tablesh_sup * t9_sup;

    Add12(t11_sup,t12_sup,tablesh_sup,t10_sup);
    t13_sup = t12_sup + tablesl_sup;
    Add12(polyTblhdb_sup.d,polyTblmdb_sup.d,t11_sup,t13_sup);

    /* Reconstruction: multiplication by 2^M */

    /* Implement the multiplication by multiplication to overcome the
       problem of the non-representability of 2^1024 (M = 1024)
       This case is possible if polyTblhdb.d < 1
    */
  
    polyTblhdb_sup.i[HI] += M_sup << 20;
    polyTblmdb_sup.i[HI] += M_sup << 20;

    exph_sup = polyTblhdb_sup.d;
    expm_sup = polyTblmdb_sup.d;

    /* Substraction of 1 

       Testing if the operation is necessary is more expensive than 
       performing it in any case.

       We may cancellate at most 2 bits in the subtraction for 
       arguments 1/4 <= x <= ln(2) (0.25 <= x <= 0.69) 
       We must therefore use conditional Add12s

       Since we perform a substraction, we may not have addition overflow towards +inf

    */

    Add12Cond(t1_sup,t2_sup,-1,exph_sup);
    t3_sup = t2_sup + expm_sup;
    Add12Cond(expm1h_sup,expm1m_sup,t1_sup,t3_sup);

    /* Rounding test */
    TEST_AND_COPY_RU(roundable,res_sup,expm1h_sup, expm1m_sup, ROUNDCSTCOMMONRD);
    if(roundable==0)
    {
      /* Rest of argument reduction for accurate phase */
      Mul133(&msLog2Div2LMultKh_sup,&msLog2Div2LMultKm_sup,&msLog2Div2LMultKl_sup,kd_sup,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
      t1_sup = x_sup + msLog2Div2LMultKh_sup;
      Add12Cond(rh_sup,t2_sup,t1_sup,msLog2Div2LMultKm_sup);
      Add12Cond(rm_sup,rl_sup,t2_sup,msLog2Div2LMultKl_sup);
    
      /* Table reads for accurate phase */
      tbl1l_sup = twoPowerIndex1[index1_sup].lo;
      tbl2l_sup = twoPowerIndex2[index2_sup].lo;
    
      /* Call accurate phase */
      expm1_common_td(&expm1h_sup, &expm1m_sup, &expm1l_sup, rh_sup, rm_sup, rl_sup, tbl1h_sup, tbl1m_sup, tbl1l_sup, tbl2h_sup, tbl2m_sup, tbl2l_sup, M_sup); 
    
      /* Final rounding */

      RoundUpwards3(&res_sup,expm1h_sup, expm1m_sup, expm1l_sup);
    } /* Accurate phase launched */
  }

  if (infDone==1) res_inf=restemp_inf;
  if (supDone==1) res_sup=restemp_sup;
  ASSIGN_LOW(res,res_inf);
  ASSIGN_UP(res,res_sup);
  return res;
}
#endif
