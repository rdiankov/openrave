#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "expm1.h"

#define AVOID_FMA 0


extern void expm1_direct_td(double *expm1h, double *expm1m, double *expm1l, 
	         	    double x, double xSqHalfh, double xSqHalfl, double xSqh, double xSql, int expoX);

extern void expm1_common_td(double *expm1h, double *expm1m, double *expm1l, 
			    double rh, double rm, double rl, 
			    double tbl1h, double tbl1m, double tbl1l, 
			    double tbl2h, double tbl2m, double tbl2l, 
			    int M);


/* Function expm1_13

   Computes exp(x)-1 with an accuracy of 120 bits (128 for |x| <= 2^(-5)) as

   (expm1h + expm1m + expm1l) \approx exp(x) - 1

   There is no special case handling. 

   |x| is supposed to be greater than 2^(-53)

   x is supposed to be in the range -37...709

   The triple-double exph + expm + expl is non-overlapping.

*/

void expm1_13(double *expm1h, double *expm1m, double *expm1l, double x) {
  db_number xdb, shiftedXMultdb;
  int xIntHi, expoX, k, M, index1, index2;
  double xSqh, xSql, xSqHalfh, xSqHalfl;
  double t1, t2;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double xMultLog2InvMult2L, shiftedXMult, kd, rh, rm, rl;
  double tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l;

  
  xdb.d = x; 
  xIntHi = xdb.i[HI] & 0x7fffffff;

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
    xSqHalfh = 0.5 * xSqh;
    xSqHalfl = 0.5 * xSql;

    expm1_direct_td(expm1h, expm1m, expm1l, x, xSqHalfh, xSqHalfl, xSqh, xSql, expoX);
       
  } else {

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
    
    
    Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
    t1 = x + msLog2Div2LMultKh;
    Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
    Add12Cond(rm,rl,t2,msLog2Div2LMultKl);
    
    tbl1h = twoPowerIndex1[index1].hi;
    tbl1m = twoPowerIndex1[index1].mi;
    tbl2h = twoPowerIndex2[index2].hi;
    tbl2m = twoPowerIndex2[index2].mi;
    tbl1l = twoPowerIndex1[index1].lo;
    tbl2l = twoPowerIndex2[index2].lo;
    
    expm1_common_td(expm1h, expm1m, expm1l, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l, M); 
    
  }
}
