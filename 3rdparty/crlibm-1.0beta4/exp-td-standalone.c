#include "crlibm.h"
#include "crlibm_private.h"
#include "triple-double.h"
#include "exp-td.h"

#define AVOID_FMA 0


extern void exp_td_accurate(double *polyTblh, double *polyTblm, double *polyTbll, 
			    double rh, double rm, double rl, 
			    double tbl1h, double tbl1m, double tbl1l,
			    double tbl2h, double tbl2m, double tbl2l);


/* Function exp13

   Computes exp(x) with an accuracy of 113 bits as

   2^exponent * (exph + expm + expl) \approx exp(x)

   Unless the subnormal case for x, no special cases are 
   handled.

   The triple-double exph + expm + expl is non-overlapping.
   The domain for exph + expm + expl is 1/2..2
   The integer exponent is in the range -1024..1024. The 
   value 2^(exponent) may therefore be non-representable
   whereas 2^exponent * (exph + expm + expl) is.

*/


void exp13(int *exponent, double *exph, double *expm, double *expl, double x) { 
  double rh, rm, rl, tbl1h, tbl1m, tbl1l;
  double tbl2h, tbl2m, tbl2l;
  double xMultLog2InvMult2L, shiftedXMult, kd;
  double msLog2Div2LMultKh, msLog2Div2LMultKm, msLog2Div2LMultKl;
  double t1, t2;
  db_number shiftedXMultdb, xdb;
  int k, M, index1, index2, xIntHi;
   
  /* Argument reduction and filtering for special cases */

  /* Compute k as a double and as an int */
  xdb.d = x;
  xMultLog2InvMult2L = x * log2InvMult2L;
  shiftedXMult = xMultLog2InvMult2L + shiftConst;
  kd = shiftedXMult - shiftConst;
  shiftedXMultdb.d = shiftedXMult;
  
  /* Special cases tests */
  xIntHi = xdb.i[HI];
  /* Test if argument is a denormal or zero */
  if ((xIntHi & 0x7ff00000) == 0) {
    /* We are in the RN case, return 1.0 in all cases */
    *exph = 1.0;
    *expm = 0.0;
    *expl = 0.0;
    return;
  }
 
  k = shiftedXMultdb.i[LO];
  M = k >> L;
  index1 = k & INDEXMASK1;
  index2 = (k & INDEXMASK2) >> LHALF;

  /* Table reads */
  tbl1h = twoPowerIndex1[index1].hi;
  tbl1m = twoPowerIndex1[index1].mi;
  tbl2h = twoPowerIndex2[index2].hi;
  tbl2m = twoPowerIndex2[index2].mi;
  tbl1l = twoPowerIndex1[index1].lo;
  tbl2l = twoPowerIndex2[index2].lo;

  /* Argument reduction */

  Mul133(&msLog2Div2LMultKh,&msLog2Div2LMultKm,&msLog2Div2LMultKl,kd,msLog2Div2Lh,msLog2Div2Lm,msLog2Div2Ll);
  t1 = x + msLog2Div2LMultKh;
  Add12Cond(rh,t2,t1,msLog2Div2LMultKm);
  Add12Cond(rm,rl,t2,msLog2Div2LMultKl);

  /* Polynomial approximation and reconstruction: factorized code */
  
  exp_td_accurate(exph, expm, expl, rh, rm, rl, tbl1h, tbl1m, tbl1l, tbl2h, tbl2m, tbl2l);

  *exponent = M;
}
