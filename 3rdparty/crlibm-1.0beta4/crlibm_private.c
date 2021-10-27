/**
 * Variables and common functions shared by many functions
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


/* I wish I could use C99 fenv.h, but as of 2004 it doesn't specify
   anything about precision, only rounding direction. */

#ifdef HAVE_FENV_H
#include <fenv.h>
#endif

/* Tell the compiler that we're going to mess with FP status register */
#ifdef FENV_H
#pragma STDC FENV_ACCESS ON
#endif





/* TODO proper init and exit functions 

- for Itanium, sf0 is set   to RNDouble, sf1 to RNdoubleExt, 
 set sf2 and/or sf3 for
 directed functions, one should be kept for saving the fpsr when
 speculating, study operating systems)

- for PowerPC: nothing to do usually, however if for some reason the
  CPU was not in the default state then crlibm won't work

 */



/* An init function which sets FPU flags when needed */
unsigned long long crlibm_init() {
#ifndef CRLIBM_TYPEOS_BSD
#if defined(CRLIBM_HAS_FPU_CONTROL) && (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))
  unsigned short oldcw, cw;

#if 1 /* gcc */
  /* save old state */
  _FPU_GETCW(oldcw);
  /* Set FPU flags to use double, not double extended, 
     with rounding to nearest */  
  cw = (_FPU_DEFAULT & ~_FPU_EXTENDED)|_FPU_DOUBLE;
  _FPU_SETCW(cw);
  return (unsigned long long) oldcw;
#else  /* Sun Studio  */
  __asm__ ("movw    $639, -22(%ebp)");
  __asm__ ("fldcw -22(%ebp)");
#endif


#elif defined(CRLIBM_TYPECPU_ITANIUM) 
  /* On Itanium we assume that SF2 is used fo speculation, and use only SF3 */

  unsigned long long int  old_fpsr;

#if defined(__INTEL_COMPILER)
  _Asm_fsetc( 0x00, 0x28, 3 /*_SF3*/ ); /* sf3 = round up, double-precision */

  //  _Asm_mov_to_ar(40, 
  //	 (old_fpsr & 0xfd000000FFFFFFFFULL) || ((0x18ULL<<32) + (0x28ULL<<45)) );
#elif defined(__GNUC__)
  __asm__ ("fsetc.s3 0, 40\n");
#endif /* defined(__INTEL_COMPILER) */
  old_fpsr = 0 ; /* TODO */
  return old_fpsr;

#else
  return 0;
#endif /* CRLIBM_TYPECPU_X86 || CRLIBM_TYPECPU_AMD64 */
#else
  return 0;
#endif
}

/* An exit function which sets FPU flags to initial value */
void crlibm_exit(unsigned long long int oldcw) {
#ifndef CRLIBM_TYPEOS_BSD
#if defined(CRLIBM_HAS_FPU_CONTROL) && (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))
  /* Set FPU flags to use double, not double extended, 
     with rounding to nearest */
  unsigned short t = (unsigned short)oldcw;
  _FPU_SETCW(t);
#endif
#endif
}




#if ADD22_AS_FUNCTIONS
/*
 * computes double-double addition: zh+zl = xh+xl + yh+yl
 * relative error is smaller than 2^-103 
 */
  
void Add22Cond(double *zh, double *zl,
               double xh, double xl, double yh, double yl)
{
  double r,s;
  r = xh+yh;
  if ((ABS(xh)) > (ABS(yh))) 
    {s=   ((((xh-r)+yh)+yl)+xl); }
  else {s=((((yh-r)+xh)+xl)+yl);}
  *zh = r+s;
  *zl = r - (*zh) + s;
}

/*
 * computes double-double addition: zh+zl = xh+xl + yh+yl
 * knowing that xh>yh
 * relative error is smaller than 2^-103 
 */
  
void Add22(double *zh, double *zl, double xh, double xl, double yh, double yl)
{
double r,s;

r = xh+yh;
s = xh-r+yh+yl+xl;
*zh = r+s;
*zl = r - (*zh) + s;
}

#endif /*ADD22_AS_FUNCTIONS*/



#if  DEKKER_AS_FUNCTIONS && (!defined PROCESSOR_HAS_FMA)
/* else it is defined in crlibm_private.h */

/*
 * computes rh and rl such that rh + rl = a * b with rh = a @* b exactly
 * under the conditions : a < 2^970 et b < 2^970 
 */
void  Mul12(double *rh, double *rl, double u, double v){
  const double c = 134217729.;   /*  1+2^27 */ 
  double up, u1, u2, vp, v1, v2;

  up = u*c;        vp = v*c;
  u1 = (u-up)+up;  v1 = (v-vp)+vp;
  u2 = u-u1;       v2 = v-v1;
  
  *rh = u*v;
  *rl = (((u1*v1-*rh)+(u1*v2))+(u2*v1))+(u2*v2);
}

/*
 * Computes rh and rl such that rh + rl = a * b and rh = a @* b exactly
 */
void Mul12Cond(double *rh, double *rl, double a, double b){
  const double two_970 = 0.997920154767359905828186356518419283e292;
  const double two_em53 = 0.11102230246251565404236316680908203125e-15;
  const double two_e53  = 9007199254740992.;
  double u, v;

  if (a>two_970)  u = a*two_em53; 
  else            u = a;
  if (b>two_970)  v = b*two_em53; 
  else            v = b;

  Mul12(rh, rl, u, v);

  if (a>two_970) {*rh *= two_e53; *rl *= two_e53;} 
  if (b>two_970) {*rh *= two_e53; *rl *= two_e53;} 
}

  
/*
 * computes double-double multiplication: zh+zl = (xh+xl) *  (yh+yl)
 * under the conditions : xh < 2^970 et xl < 2^970 
 * relative error is smaller than 2^-102
 */

void Mul22(double *zh, double *zl, double xh, double xl, double yh, double yl)
{
double mh, ml;

  const double c = 134217729.;                /* 0x41A00000, 0x02000000 */ 
  double up, u1, u2, vp, v1, v2;

  up = xh*c;        vp = yh*c;
  u1 = (xh-up)+up;  v1 = (yh-vp)+vp;
  u2 = xh-u1;       v2 = yh-v1;
  
  mh = xh*yh;
  ml = (((u1*v1-mh)+(u1*v2))+(u2*v1))+(u2*v2);

  ml += xh*yl + xl*yh;
  *zh = mh+ml;
  *zl = mh - (*zh) + ml;
}


/*
 * computes double-double division: pzh+pzl = (xh+xl) /  (yh+yl)
 * relative error is smaller than 2^-104
 */

void Div22(double* pzh, double* pzl, double xh, double xl, double yh, double yl){
  double _ch,_cl,_uh,_ul;  
  _ch=(xh)/(yh);   Mul12(&_uh,&_ul,_ch,(yh));  
  _cl=(((((xh)-_uh)-_ul)+(xl))-_ch*(yl))/(yh);   
  *pzh=_ch+_cl;   *pzl=(_ch-(*pzh))+_cl;
}

#endif /* DEKKER_AS_FUNCTIONS && (!defined PROCESSOR_HAS_FMA)  */


#if SQRT_AS_FUNCTIONS 

/* 
   Computes sqrt(x) with a result in double-double precision 
   Should be provable to be exact to at least 100 bits.
   
   Only handles the following special cases:
   - x == 0
   - subnormal x 
   The following cases are not handled:
   - x < 0
   - x = +/-Infty, NaN
*/
void sqrt12(double *resh, double *resl, double x) {
  db_number xdb;
  int E;
  double m, r0, r1, r2, r3h, r3l, r4h, r4l, srtmh, srtml;
  double r2PHr2h, r2PHr2l, r2Sqh, r2Sql;
  double mMr2h, mMr2l, mMr2Ch, mMr2Cl;
  double MHmMr2Ch, MHmMr2Cl;
  double r3Sqh, r3Sql, mMr3Sqh, mMr3Sql;

  /* Special case x = 0 */
  if (x == 0) {
    *resh = x;
    *resl = 0;
  } else {

    E = 0;

    /* Convert to integer format */
    xdb.d = x;
      
    /* Handle subnormal case */
    if (xdb.i[HI] < 0x00100000) {
      E = -52;
      xdb.d *= ((db_number) ((double) SQRTTWO52)).d; 	  /* make x a normal number */ 
    }
    
    /* Extract exponent E and mantissa m */
    E += (xdb.i[HI]>>20)-1023; 
    xdb.i[HI] = (xdb.i[HI] & 0x000fffff) | 0x3ff00000;
    m = xdb.d;
    
    /* Make exponent even */
    if (E & 0x00000001) {
      E++;
      m *= 0.5;    /* Suppose now 1/2 <= m <= 2 */
    }
  
    /* Construct sqrt(2^E) = 2^(E/2) */
    xdb.i[HI] = (E/2 + 1023) << 20;
    xdb.i[LO] = 0;
    
    /* Compute initial approximation to r = 1/sqrt(m) */
    
    r0 = SQRTPOLYC0 + m * (SQRTPOLYC1 + m * (SQRTPOLYC2 + m * (SQRTPOLYC3 + m * SQRTPOLYC4)));
    
    /* Iterate two times on double precision */
    
    r1 = 0.5 * r0 * (3 - m * (r0 * r0));
    r2 = 0.5 * r1 * (3 - m * (r1 * r1));

    /* Iterate two times on double-double precision */

    Mul12(&r2Sqh, &r2Sql, r2, r2);    Add12(r2PHr2h, r2PHr2l, r2, 0.5 * r2);
    Mul12(&mMr2h, &mMr2l, m, r2);
    Mul22(&mMr2Ch, &mMr2Cl, mMr2h, mMr2l, r2Sqh, r2Sql);
    
    MHmMr2Ch = -0.5 * mMr2Ch;
    MHmMr2Cl = -0.5 * mMr2Cl;

    Add22(&r3h, &r3l, r2PHr2h, r2PHr2l, MHmMr2Ch, MHmMr2Cl);
 
    Mul22(&r3Sqh, &r3Sql, r3h, r3l, r3h, r3l);
    Mul22(&mMr3Sqh, &mMr3Sql, m, 0, r3Sqh, r3Sql);  /* To prove: mMr3Sqh = 1.0 in each case */ 
    
    Mul22(&r4h, &r4l, r3h, r3l, 1, -0.5 * mMr3Sql);

    /* Multiply obtained reciprocal square root by m */
    
    Mul22(&srtmh,&srtml,m,0,r4h,r4l);

    /* Multiply componentwise by sqrt(2^E), which is an integer power of 2 that may not produce a subnormal */
    
    *resh = xdb.d * srtmh;
    *resl = xdb.d * srtml;
    
  } /* End: special case 0 */
}

#endif /* SQRT_AS_FUNCTIONS */




  
#if EVAL_PERF==1
/* counter of calls to the second step (accurate step) */
int crlibm_second_step_taken;
#endif

/* A debug functions */

void printHexa(char* s, double x) {
  db_number xdb;

  xdb.d = x;
  printf("%s = %08x%08x (%1.8e) exponent = %d exponent of ulp = %d\n",
	 s,
	 xdb.i[HI],
	 xdb.i[LO],
	 x,
	 ((xdb.i[HI] & 0x7ff00000) >> 20) - 1023,
	 ((xdb.i[HI] & 0x7ff00000) >> 20) - 1023 - 52);
}




#ifdef SCS_TYPECPU_SPARC
 const scs
/* 0   */
   scs_zer ={{0x00000000, 0x00000000, 0x00000000, 0x00000000},
             {{0, 0}},  0,   1 },
/* 1/2 */
   scs_half={{0x02000000, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE, -1,   1 },
/*  1  */  
   scs_one ={{0x00000001, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE,  0,   1 },
/*  2  */
   scs_two ={{0x00000002, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE,  0,   1 },	

/* ~1.666667e-01 */ 
   scs_sixinv ={{0x0aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa},
	     DB_ONE,  -1,   1 };

#else
 const struct scs
/* 0   */
   scs_zer ={{0x00000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000},
             {{0, 0}},  0,   1 },
/* 1/2 */
   scs_half={{0x20000000, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE, -1,   1 },
/*  1  */  
   scs_one ={{0x00000001, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE,  0,   1 },
/*  2  */
   scs_two ={{0x00000002, 0x00000000, 0x00000000, 0x00000000,
             0x00000000, 0x00000000, 0x00000000, 0x00000000},
             DB_ONE,  0,   1 },
/* 0.166666*/
   scs_sixinv ={{0x0aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa, 
	     0x2aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa, 0x2aaaaaaa},
	     DB_ONE,  -1,   1 };

#endif
