/*
 *  crlibm_private.h
 *  
 * This file contains useful tools and data for the crlibm functions.
 *
 */

#ifndef CRLIBM_PRIVATE_H
#define CRLIBM_PRIVATE_H 1

#include "scs_lib/scs.h"
#include "scs_lib/scs_private.h"

#ifdef HAVE_CONFIG_H
#include "crlibm_config.h"
#endif
/* otherwise CMake is used, and defines all the useful variables using -D switch */

#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif



#if (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))
# ifdef CRLIBM_HAS_FPU_CONTROL
#  include <fpu_control.h>
#  ifndef _FPU_SETCW
#   define _FPU_SETCW(cw) __asm__ ("fldcw %0" : : "m" (*&cw))
#  endif
#  ifndef _FPU_GETCW
#   define _FPU_GETCW(cw) __asm__ ("fnstcw %0" : "=m" (*&cw))
#  endif
# endif
#endif

/* 64 bit arithmetic may be standardised, but people still do what they want */
#ifdef HAVE_INTTYPES_H
#define ULL(bits) 0x##bits##uLL
#elif defined(_WIN32) 
/*  Windows garbage there */
typedef long long int int64_t;
typedef unsigned long long int uint64_t;
#define ULL(bits) 0x##bits##i64
/* Default, hoping it works, hopefully less and less relevant */
#else
typedef long long int int64_t;
typedef unsigned long long int uint64_t;
#define ULL(bits) 0x##bits##uLL
#endif

#ifndef SCS_DEF_INT64
#define SCS_DEF_INT64
#ifdef CRLIBM_TYPEOS_HPUX
#ifndef __LP64__ /* To solve the problem with 64 bits integer on HPPA */
typedef long long int64_t;
typedef unsigned long long uint64_t;
#define ULL(bits) 0x##bits##uLL
#endif
#endif
#endif




/* The Add22 and Add22 functions, as well as double-double
multiplications of the Dekker family may be either defined as
functions, or as #defines.  Which one is better depends on the
processor/compiler/OS.  As #define has to be used with more care (not
type-safe), the two following variables should  be set to 1 in the
development/debugging phase, until no type warning remains.  

*/

#define ADD22_AS_FUNCTIONS 0
#define DEKKER_AS_FUNCTIONS 0
#define SQRT_AS_FUNCTIONS 0

/* The conditional version of the Add12 can be implemented either
   using 3 floating point additions, a absolute value test and 
   a branch or using 6 floating point additions but no branch.
   The Add22 sequence is similar. 
   The branchless versions might be faster on some systems.

   The function versions of Add12Cond and Add22Cond are not 
   implemented in branchless versions.
*/

#define AVOID_BRANCHES 1


/* setting the following variable adds variables and code for
   monitoring the performance.
   Note that sometimes only round to nearest is instrumented */
#define EVAL_PERF  1


#if EVAL_PERF==1
/* counter of calls to the second step (accurate step) */
extern int crlibm_second_step_taken;
#endif



/* The prototypes of the second steps */
/* extern void exp_SC(scs_ptr res_scs, double x);*/


 


/*
 * i = d in rounding to nearest
  The constant added is 2^52 + 2^51 
 */
#define DOUBLE2INT(_i, _d)       \
  {db_number _t;              \
   _t.d = (_d+6755399441055744.0);  \
   _i = _t.i[LO];}


/* Same idea but beware: works only for |_i| < 2^51 -1 */
#define DOUBLE2LONGINT(_i, _d)                                        \
  {                                                                   \
    db_number _t;                                                     \
    _t.d = (_d+6755399441055744.0);                                   \
    if (_d >= 0) /* sign extend */                                    \
      _i = _t.l & ULL(0007FFFFFFFFFFFF);                              \
    else                                                              \
      _i = (_t.l & ULL(0007FFFFFFFFFFFF)) |  (ULL(FFF8000000000000)); \
  }





/* Macros for the rounding tests in directed modes */
/* After Evgeny Gvozdev pointed out a bug in the rounding procedures I
   decided to centralize them here 

Note that these tests launch the accurate phase when yl=0, in
particular in the exceptional cases when the image of a double is a
double. See the chapter about the log for an example

All this does not work for denormals, of course
*/


#define TEST_AND_RETURN_RU(__yh__, __yl__, __eps__)                    \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  __yldb__.l = __yldb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  if(__yldb__.d > __eps__ * u53.d){                                          \
    if(!yl_neg) {  /* The case yl==0 is filtered by the above test*/   \
      /* return next up */                                             \
      __yhdb__.d = __yh__;                                                   \
      if(yh_neg) __yhdb__.l--;  else __yhdb__.l++; /* Beware: fails for zero */    \
      return __yhdb__.d ;                                                    \
    }                                                                  \
    else  return __yh__;                                               \
  }                                                                    \
}


#define TEST_AND_RETURN_RD(__yh__, __yl__, __eps__)                    \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  __yldb__.l = __yldb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  if(__yldb__.d > __eps__ * u53.d){                                          \
    if(yl_neg) {   /* The case yl==0 is filtered by the above test*/   \
      /* return next down */                                           \
      __yhdb__.d = __yh__;                                                   \
      if(yh_neg) __yhdb__.l++;  else __yhdb__.l--; /* Beware: fails for zero */    \
      return __yhdb__.d ;                                                    \
    }                                                                  \
    else  return __yh__;                                               \
  }                                                                    \
}



#define TEST_AND_RETURN_RZ(__yh__, __yl__, __eps__)                    \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & ULL(7fffffffffffffff);  /* compute the absolute value*/\
  __yldb__.l = __yldb__.l & ULL(7fffffffffffffff);  /* compute the absolute value*/\
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  if(__yldb__.d > __eps__ * u53.d){                                          \
    if(yl_neg!=yh_neg) {                                               \
      __yhdb__.d = __yh__;                                                   \
      __yhdb__.l--;                          /* Beware: fails for zero */    \
      return __yhdb__.d ;                                                    \
    }                                                                  \
    else  return __yh__;                                               \
  }                                                                    \
}



#define TEST_AND_COPY_RU(__cond__, __res__, __yh__, __yl__, __eps__)   \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  __yldb__.l = __yldb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  __cond__ = 0;                                                        \
  if(__yldb__.d > __eps__ * u53.d){                                          \
     __cond__ = 1;                                                     \
    if(!yl_neg) {  /* The case yl==0 is filtered by the above test*/   \
      /* return next up */                                             \
      __yhdb__.d = __yh__;                                                   \
      if(yh_neg) __yhdb__.l--;  else __yhdb__.l++; /* Beware: fails for zero */    \
      __res__ = __yhdb__.d ;                                                 \
    }                                                                  \
    else {                                                             \
      __res__ = __yh__;                                                \
    }                                                                  \
  }                                                                    \
}

#define TEST_AND_COPY_RD(__cond__, __res__, __yh__, __yl__, __eps__)   \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  __yldb__.l = __yldb__.l & 0x7fffffffffffffffLL;  /* compute the absolute value*/ \
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  __cond__ = 0;                                                        \
  if(__yldb__.d > __eps__ * u53.d){                                          \
    __cond__ = 1;                                                      \
    if(yl_neg) {  /* The case yl==0 is filtered by the above test*/    \
      /* return next down */                                           \
      __yhdb__.d = __yh__;                                                   \
      if(yh_neg) __yhdb__.l++;  else __yhdb__.l--; /* Beware: fails for zero */    \
      __res__ = __yhdb__.d ;                                                 \
    }                                                                  \
    else {                                                             \
      __res__ = __yh__;                                                \
    }                                                                  \
  }                                                                    \
}


#define TEST_AND_COPY_RZ(__cond__, __res__, __yh__, __yl__, __eps__)   \
{                                                                      \
  db_number __yhdb__, __yldb__, u53;  int yh_neg, yl_neg;                          \
  __yhdb__.d = __yh__;    __yldb__.d = __yl__;                                     \
  yh_neg = (__yhdb__.i[HI] & 0x80000000);                                    \
  yl_neg = (__yldb__.i[HI] & 0x80000000);                                    \
  __yhdb__.l = __yhdb__.l & ULL(7fffffffffffffff);  /* compute the absolute value*/\
  __yldb__.l = __yldb__.l & ULL(7fffffffffffffff);  /* compute the absolute value*/\
  u53.l     = (__yhdb__.l & ULL(7ff0000000000000)) +  ULL(0010000000000000); \
  __cond__ = 0;                                                        \
  if(__yldb__.d > __eps__ * u53.d){                                          \
    if(yl_neg!=yh_neg) {                                               \
      __yhdb__.d = __yh__;                                                   \
      __yhdb__.l--;                          /* Beware: fails for zero */    \
      __res__ = __yhdb__.d ;                                                 \
      __cond__ = 1;                                                    \
    }                                                                  \
    else {                                                             \
      __res__ = __yh__;                                                \
      __cond__ = 1;                                                    \
  }                                                                    \
}



/* If the processor has a FMA, use it !   **/

/* All this probably works only with gcc. 
   See Markstein book for the case of HP's compiler */

#if defined(CRLIBM_TYPECPU_POWERPC) && defined(__GNUC__)
#define PROCESSOR_HAS_FMA 1
#define FMA(a,b,c)  /* r = a*b + c*/                   \
({                                                     \
  double _a, _b,_c,_r;                                 \
  _a=a; _b=b;_c=c;                                     \
  __asm__ ("fmadd %0, %1, %2, %3\n ;;\n"               \
		       : "=f"(_r)                      \
		       : "f"(_a), "f"(_b), "f"(_c)     \
		       );                              \
 _r;                                                   \
})


#define FMS(a,b,c)   /* r = a*b - c*/                 \
({                                                    \
  double _a, _b,_c,_r;                                \
  _a=a; _b=b;_c=c;                                    \
  __asm__ ("fmsub %0, %1, %2, %3\n ;;\n"              \
		       : "=f"(_r)                     \
		       : "f"(_a), "f"(_b), "f"(_c)    \
		       );                             \
  _r;                                                 \
  })

#endif /* defined(CRLIBM_TYPECPU_POWERPC) && defined(__GCC__) */




/* On the Itanium 1 / gcc3.2 we lose 10 cycles when using the FMA !?! 
   It probably breaks the scheduling algorithms somehow... 
   To test again with higher gcc versions
*/ 

#if defined(CRLIBM_TYPECPU_ITANIUM) && defined(__GNUC__)  && !defined(__INTEL_COMPILER) && 0
#define PROCESSOR_HAS_FMA 1
#define FMA(a,b,c)  /* r = a*b + c*/                 \
({                                                   \
  double _a, _b,_c,_r;                               \
  _a=a; _b=b;_c=c;                                   \
  __asm__ ("fma %0 = %1, %2, %3\n ;;\n"              \
		       : "=f"(_r)                    \
		       : "f"(_a), "f"(_b), "f"(_c)   \
		       );                            \
  _r;                                                \
})


#define FMS(a,b,c)  /* r = a*b - c*/                 \
({                                                   \
  double _a, _b, _c, _r;                             \
  _a=a; _b=b;_c=c;                                   \
  __asm__ ("fms %0 = %1, %2, %3\n ;;\n"              \
		       : "=f"(_r)                    \
		       : "f"(_a), "f"(_b), "f"(_c)   \
		       );                            \
  _r;                                                \
  })
#endif /* defined(CRLIBM_TYPECPU_ITANIUM) && defined(__GCC__) && !defined(__INTEL_COMPILER) */




#if defined(CRLIBM_TYPECPU_ITANIUM) && defined(__INTEL_COMPILER) 
#define PROCESSOR_HAS_FMA 1
#if 0 /* Commented out because it shouldn't be there: There should be
	 a standard #include doing all this, but as of april 2005
	 it doesn't exist, say intel people). Leave
	 it as documentation, though, until it is replaced by #include
*/
/* Table 1-17: legal floating-point precision completers (.pc) */
typedef enum {
    _PC_S        = 1        /* single .s */
   ,_PC_D        = 2        /* double .d */
   ,_PC_NONE     = 3        /* dynamic   */
} _Asm_pc;

/* Table 1-22: legal getf/setf floating-point register access completers */
typedef enum {
    _FR_S        = 1        /* single form      .s   */
   ,_FR_D        = 2        /* double form      .d   */
   ,_FR_EXP      = 3        /* exponent form    .exp */
   ,_FR_SIG      = 4        /* significand form .sig */
} _Asm_fr_access;

/* Table 1-24: legal floating-point FPSR status field completers (.sf) */
typedef enum {
    _SF0         = 0        /* FPSR status field 0 .s0 */
   ,_SF1         = 1        /* FPSR status field 1 .s1 */
   ,_SF2         = 2        /* FPSR status field 2 .s2 */
   ,_SF3         = 3        /* FPSR status field 3 .s3 */
} _Asm_sf;
#endif

#define FMA(a,b,c)  /* r = a*b + c*/                 \
   _Asm_fma( 2/*_PC_D*/, a, b, c, 0/*_SF0*/ );              


#define FMS(a,b,c)  /* r = a*b - c*/                 \
   _Asm_fms( 2/*_PC_D*/, a, b, c, 0/*_SF0*/);              

#endif /*defined(CRLIBM_TYPECPU_ITANIUM) && defined(__INTEL_COMPILER)*/








#ifdef WORDS_BIGENDIAN
 #define DB_ONE    {{0x3ff00000, 0x00000000}}
#else
 #define DB_ONE    {{0x00000000 ,0x3ff00000}}
#endif






extern const scs scs_zer, scs_half, scs_one, scs_two, scs_sixinv;


#define SCS_ZERO    (scs_ptr)(&scs_zer)
#define SCS_HALF    (scs_ptr)(&scs_half)
#define SCS_ONE     (scs_ptr)(&scs_one)
#define SCS_TWO     (scs_ptr)(&scs_two)
#define SCS_SIXINV  (scs_ptr)(&scs_sixinv)




#if defined(__GNUC__)
#define ABS(x) (__builtin_fabs((x)))
#else
#define ABS(x) (((x)>0) ? (x) : (-(x)))
#endif




/*
 * In the following, when an operator is preceded by a '@' it means that we
 * are considering the IEEE-compliant machine operator, otherwise it
 * is the mathematical operator.
 *
 */


/*
 * computes s and r such that s + r = a + b,  with s = a @+ b exactly 
 */
#if AVOID_BRANCHES
#define Add12Cond(s, r, a, b)      \
{                                  \
    double _u1, _u2, _u3, _u4;     \
    double  _a=a, _b=b;            \
                                   \
    s = _a + _b;                   \
    _u1 = s - _a;                  \
    _u2 = s - _u1;                 \
    _u3 = _b - _u1;                \
    _u4 = _a - _u2;                \
    r = _u4 + _u3;                 \
}

#else
#define Add12Cond(s, r, a, b)      \
        {double _z, _a=a, _b=b;    \
         s = _a + _b;              \
         if (ABS(a) > ABS(b)){     \
           _z = s - _a;            \
           r = _b - _z;            \
         }else {                   \
           _z = s - _b;            \
           r = _a - _z;}}                          
#endif

/*
 *  computes s and r such that s + r = a + b,  with s = a @+ b exactly 
 * under the condition  a >= b
 */
#define Add12(s, r, a, b)          \
        {double _z, _a=a, _b=b;    \
         s = _a + _b;              \
         _z = s - _a;              \
         r = _b - _z;   }            


/*
 * computes r1, r2, r3 such that r1 + r2 + r3 = a + b + c exactly 
 */
#define Fast3Sum(r1, r2, r3, a, b,  c) \
        {double u, v, w;               \
         Fast2Sum(u, v, b, c);         \
         Fast2Sum(r1, w, a, u);        \
         Fast2Sum(r2, r3, w, v); }







/*
 * Functions to computes double-double addition: zh+zl = xh+xl + yh+yl
 * knowing that xh>yh
 * relative error is smaller than 2^-103 
 */


#if ADD22_AS_FUNCTIONS
extern void Add22(double *zh, double *zl, double xh, double xl, double yh, double yl);
extern void Add22Cond(double *zh, double *zl, double xh, double xl, double yh, double yl);

#else /* ADD22_AS_FUNCTIONS */

#if AVOID_BRANCHES
#define Add22Cond(zh,zl,xh,xl,yh,yl)                                                   \
do {                                                                                   \
  double _v1, _v2, _v3, _v4;                                                           \
                                                                                       \
  Add12Cond(_v1, _v2, (xh), (yh));                                                     \
  _v3 = (xl) + (yl);                                                                   \
  _v4 = _v2 + _v3;                                                                     \
  Add12((*(zh)),(*(zl)),_v1,_v4);                                                      \
} while (2+2==5) 
#else
#define Add22Cond(zh,zl,xh,xl,yh,yl)                                                   \
do {                                                                                   \
  double _r,_s;                                                                        \
  _r = (xh)+(yh);                                                                      \
  _s = ((ABS(xh)) > (ABS(yh)))? ((xh)-_r+(yh)+(yl)+(xl)) : ((yh)-_r+(xh)+(xl)+(yl));   \
  *zh = _r+_s;                                                                         \
  *zl = (_r - (*zh)) + _s;                                                             \
} while(2+2==5)
#endif
  

#define Add22(zh,zl,xh,xl,yh,yl)         \
do {                                     \
double _r,_s;                            \
_r = (xh)+(yh);                          \
_s = ((((xh)-_r) +(yh)) + (yl)) + (xl);  \
*zh = _r+_s;                             \
*zl = (_r - (*zh)) + _s;                 \
} while(0)

#endif /* ADD22_AS_FUNCTIONS */



#ifdef PROCESSOR_HAS_FMA
/* One of the nice things with the fused multiply-and-add is that it
   greatly simplifies the double-double multiplications : */
#define Mul12(rh,rl,u,v)                             \
{                                                    \
  *rh = u*v;                                         \
  *rl = FMS(u,v, *rh);                               \
}

#define Mul22(pzh,pzl, xh,xl, yh,yl)                  \
{                                                     \
double ph, pl;                                        \
  ph = xh*yh;                                         \
  pl = FMS(xh, yh,  ph);                              \
  pl = FMA(xh,yl, pl);                                \
  pl = FMA(xl,yh,pl);                                 \
  *pzh = ph+pl;					      \
  *pzl = ph - (*pzh);                                 \
  *pzl += pl;                                         \
}


/* besides we don't care anymore about overflows in the mult  */
#define Mul12Cond Mul12    
#define Mul22cond Mul22


#else /* ! PROCESSOR_HAS_FMA */


#if DEKKER_AS_FUNCTIONS
extern void Mul12(double *rh, double *rl, double u, double v);
extern void Mul12Cond(double *rh, double *rl, double a, double b);
extern void Mul22(double *zh, double *zl, double xh, double xl, double yh, double yl);
#else /* if DEKKER_AS_FUNCTIONS  */
/*
 * computes rh and rl such that rh + rl = a * b with rh = a @* b exactly
 * under the conditions : a < 2^970 et b < 2^970 
 */
#if 1
#define Mul12(rh,rl,u,v)                        \
{                                               \
  const double c  = 134217729.; /* 2^27 +1 */   \
  double up, u1, u2, vp, v1, v2;                \
  double _u=u, _v=v;                            \
  up = _u*c;        vp = _v*c;                  \
  u1 = (_u-up)+up;  v1 = (_v-vp)+vp;            \
  u2 = _u-u1;       v2 = _v-v1;                 \
                                                \
  *rh = _u*_v;                                  \
  *rl = (((u1*v1-*rh)+(u1*v2))+(u2*v1))+(u2*v2);\
}
#else 
/* This works but is much slower. Problem:
 SSE2 instructions are two-address, and intrinsincs are 3-address  */
#include<emmintrin.h>
#define Mul12(rh,rl,u,v)                        \
{                                               \
  const double c  = 134217729.; /* 2^27 +1 */   \
 __m128d _u_v = _mm_set_pd (u,v);           \
 __m128d c2=_mm_set1_pd(c);                    \
         c2 = _mm_mul_pd(c2, _u_v);          \
 __m128d u1v1 = _mm_sub_pd(_u_v, c2);        \
         u1v1 = _mm_add_pd(u1v1, c2);        \
 __m128d u2v2 = _mm_sub_pd(_u_v, u1v1);        \
 __m128d _v_u = _mm_shuffle_pd(_u_v, _u_v, _MM_SHUFFLE2 (0,1));      \
 __m128d rhrh = _mm_mul_pd(_v_u, _u_v);        \
 _mm_store_sd (rh, rhrh);                      \
 __m128d v2u2 = _mm_shuffle_pd(u2v2, u2v2, _MM_SHUFFLE2 (0,1));      \
 __m128d u1v2u2v1 = _mm_mul_pd(u1v1, v2u2);          \
 __m128d u2v1u1v2 = _mm_shuffle_pd(u1v2u2v1, u1v2u2v1, _MM_SHUFFLE2 (0,1));      \
 __m128d uvmed = _mm_add_pd(u1v2u2v1, u2v1u1v2);      \
 __m128d u1u2 = _mm_shuffle_pd(u1v1, u2v2, _MM_SHUFFLE2 (1,1));      \
 __m128d v1v2 = _mm_shuffle_pd(u1v1, u2v2, _MM_SHUFFLE2 (0,0));      \
 __m128d u1v1u2v2 = _mm_mul_pd(u1u2, v1v2);          \
 __m128d tmp = _mm_sub_pd(u1v1u2v2, rhrh);        \
         tmp = _mm_add_pd(tmp,  uvmed);        \
 __m128d u2v2u2v2 = _mm_mul_pd(u2v2, v2u2);          \
         tmp = _mm_add_pd(tmp,  u2v2u2v2);        \
 _mm_store_sd (rl, tmp);                      \
}
#endif

/*
  double _u =u, _v=v;                           \
 __m128d _u_v = _mm_set_pd(_u, _v);            \
*/                                                \
/*
 * Computes rh and rl such that rh + rl = a * b and rh = a @* b exactly
 */
#define Mul12Cond(rh, rl, a,  b) \
{\
  const double two_em53 = 1.1102230246251565404e-16; /* 0x3CA00000, 0x00000000 */\
  const double two_e53  = 9007199254740992.;         /* 0x43400000, 0x00000000 */\
  double u, v;                                               \
  db_number _a=a, _b=b;                                      \
                                                             \
  if (_a.i[HI]>0x7C900000) u = _a*two_em53;                  \
  else            u = _a;                                    \
  if (_b.i[HI]>0x7C900000) v = _b*two_em53;                  \
  else            v = _b;                                    \
                                                             \
  Mul12(rh, rl, u, v);                                       \
                                                             \
  if (_a.i[HI]>0x7C900000) {*rh *= two_e53; *rl *= two_e53;} \
  if (_b.i[HI]>0x7C900000) {*rh *= two_e53; *rl *= two_e53;} \
}



/*
 * computes double-double multiplication: zh+zl = (xh+xl) *  (yh+yl)
 * relative error is smaller than 2^-102
 */
  

  
#define Mul22(zh,zl,xh,xl,yh,yl)                      \
{                                                     \
double mh, ml;                                        \
						      \
  const double c = 134217729.;			      \
  double up, u1, u2, vp, v1, v2;		      \
						      \
  up = (xh)*c;        vp = (yh)*c;		      \
  u1 = ((xh)-up)+up;  v1 = ((yh)-vp)+vp;	      \
  u2 = (xh)-u1;       v2 = (yh)-v1;                   \
  						      \
  mh = (xh)*(yh);				      \
  ml = (((u1*v1-mh)+(u1*v2))+(u2*v1))+(u2*v2);	      \
						      \
  ml += (xh)*(yl) + (xl)*(yh);			      \
  *zh = mh+ml;					      \
  *zl = mh - (*zh) + ml;                              \
}



#endif /* DEKKER_AS_FUNCTIONS */

#endif /* PROCESSOR_HAS_FMA */

/* Additional double-double operators */

/* Eps Mul122 <= 2^-102 */
#define Mul122(resh,resl,a,bh,bl)                 \
{                                                 \
    double _t1, _t2, _t3, _t4;                    \
                                                  \
    Mul12(&_t1,&_t2,(a),(bh));                    \
    _t3 = (a) * (bl);                             \
    _t4 = _t2 + _t3;                              \
    Add12((*(resh)),(*(resl)),_t1,_t4);           \
}

/* Eps MulAdd212 <= 2^-100 for |a * (bh + bl)| <= 1/4 * |ch + cl| */
#define MulAdd212(resh,resl,ch,cl,a,bh,bl)           \
{                                                    \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8;   \
                                                     \
    Mul12(&_t1,&_t2,(a),(bh));                       \
    Add12(_t3,_t4,(ch),_t1);                         \
    _t5 = (bl) * (a);                                \
    _t6 = (cl) + _t2;                                \
    _t7 = _t5 + _t6;                                 \
    _t8 = _t7 + _t4;                                 \
    Add12((*(resh)),(*(resl)),_t3,_t8);              \
}

/* Eps MulAdd212 <= 2^-100 
   for |(ah + bh) * (bh + bl)| <= 1/4 * |ch + cl| 
*/
#define MulAdd22(resh,resl,ch,cl,ah,al,bh,bl)        \
{                                                    \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8;   \
    double _t9, _t10;                                \
                                                     \
    Mul12(&_t1,&_t2,(ah),(bh));                      \
    Add12(_t3,_t4,(ch),_t1);                         \
    _t5 = (ah) * (bl);                               \
    _t6 = (al) * (bh);                               \
    _t7 = _t2 + (cl);                                \
    _t8 = _t4 + _t7;                                 \
    _t9 = _t5 + _t6;                                 \
    _t10 = _t8 + _t9;                                \
    Add12((*(resh)),(*(resl)),_t3,_t10);             \
}

#define Add122(resh,resl,a,bh,bl)                    \
{                                                    \
    double _t1, _t2, _t3;                            \
                                                     \
    Add12(_t1,_t2,(a),(bh));                         \
    _t3 = _t2 + (bl);                                \
    Add12((*(resh)),(*(resl)),_t1,_t3);              \
}    

#define Add122Cond(resh,resl,a,bh,bl)                \
{                                                    \
    double _t1, _t2, _t3;                            \
                                                     \
    Add12Cond(_t1,_t2,(a),(bh));                     \
    _t3 = _t2 + (bl);                                \
    Add12((*(resh)),(*(resl)),_t1,_t3);              \
}    


#define Add212(resh,resl,ah,al,b)                    \
{                                                    \
    double _t1, _t2, _t3;                            \
                                                     \
    Add12(_t1,_t2,(ah),b);                           \
    _t3 = _t2 + (al);                                \
    Add12((*(resh)),(*(resl)),_t1,_t3);              \
}


/* In the following the one-line computation of _cl was split so that
   icc(8.1) would compile it properly. It's a bug of icc */

#if DEKKER_AS_FUNCTIONS
extern void Div22(double *z, double *zz, double x, double xx, double y, double yy);
#else
#define  Div22(pzh,pzl,xh,xl,yh,yl)  {           \
  double _ch,_cl,_uh,_ul;                        \
  _ch=(xh)/(yh);   Mul12(&_uh,&_ul,_ch,(yh));    \
  _cl=((xh)-_uh);                                \
  _cl -= _ul;                                    \
  _cl += (xl);                                   \
  _cl -= _ch*(yl);                               \
  _cl /= (yh);                                   \
  *pzh=_ch+_cl;   *pzl=(_ch-(*pzh))+_cl;         \
}
#endif /* DEKKER_AS_FUNCTIONS */



/* 
   Coefficients for 1/sqrt(m) with 1/2 < m < 2
   The corresponding relative polynomial approximation error is less than
   eps < 2^(-8.3127) (cf. Maple file)
   The Itanium instruction frsqrta is slightly more accurate; it can
   therefore easily replace the polynomial evaluation.
*/
   
#define SQRTPOLYC0 2.50385236695888790947606139525305479764938354492188e+00   
#define SQRTPOLYC1 -3.29763389114324168005509818613063544034957885742188e+00  
#define SQRTPOLYC2 2.75726076139124520736345402838196605443954467773438e+00   
#define SQRTPOLYC3 -1.15233725777933848632983426796272397041320800781250e+00  
#define SQRTPOLYC4 1.86900066679800969104974228685023263096809387207031e-01   
#define SQRTTWO52 4.50359962737049600000000000000000000000000000000000e+15

#if SQRT_AS_FUNCTIONS
extern void sqrt12(double *resh, double *resl, double x);
#else

/* Concerning special case handling see crlibm_private.h */
#define  sqrt12(resh, resl, x)  {                                                            \
  db_number _xdb;                                                                            \
  int _E;                                                                                    \
  double _m, _r0, _r1, _r2, _r3h, _r3l, _r4h, _r4l, _srtmh, _srtml;                          \
  double _r2PHr2h, _r2PHr2l, _r2Sqh, _r2Sql;                                                 \
  double _mMr2h, _mMr2l, _mMr2Ch, _mMr2Cl;                                                   \
  double _MHmMr2Ch, _MHmMr2Cl;                                                               \
  double _r3Sqh, _r3Sql, _mMr3Sqh, _mMr3Sql;                                                 \
  double _half;                                                                              \
                                                                                             \
  /* Special case x = 0 */                                                                   \
  if ((x) == 0.0) {                                                                          \
    (*(resh)) = (x);                                                                         \
    (*(resl)) = 0.0;                                                                         \
  } else {                                                                                   \
                                                                                             \
    _E = 0;                                                                                  \
                                                                                             \
    /* Convert to integer format */                                                          \
    _xdb.d = (x);                                                                            \
                                                                                             \
    /* Handle subnormal case */                                                              \
    if (_xdb.i[HI] < 0x00100000) {                                                           \
      _E = -52;                                                                              \
      _xdb.d *= ((db_number) ((double) SQRTTWO52)).d; 	                                     \
                      /* make x a normal number */                                           \
    }                                                                                        \
                                                                                             \
    /* Extract exponent E and mantissa m */                                                  \
    _E += (_xdb.i[HI]>>20)-1023;                                                             \
    _xdb.i[HI] = (_xdb.i[HI] & 0x000fffff) | 0x3ff00000;                                     \
    _m = _xdb.d;                                                                             \
                                                                                             \
    _half = 0.5;                                                                             \
    /* Make exponent even */                                                                 \
    if (_E & 0x00000001) {                                                                   \
      _E++;                                                                                  \
      _m *= _half;    /* Suppose now 1/2 <= m <= 2 */                                        \
    }                                                                                        \
                                                                                             \
    /* Construct sqrt(2^E) = 2^(E/2) */                                                      \
    _xdb.i[HI] = (_E/2 + 1023) << 20;                                                        \
    _xdb.i[LO] = 0;                                                                          \
                                                                                             \
    /* Compute initial approximation to r = 1/sqrt(m) */                                     \
                                                                                             \
    _r0 = SQRTPOLYC0 +                                                                       \
         _m * (SQRTPOLYC1 + _m * (SQRTPOLYC2 + _m * (SQRTPOLYC3 + _m * SQRTPOLYC4)));        \
                                                                                             \
    /* Iterate two times on double precision */                                              \
                                                                                             \
    _r1 = _half * _r0 * (3.0 - _m * (_r0 * _r0));                                            \
    _r2 = _half * _r1 * (3.0 - _m * (_r1 * _r1));                                            \
                                                                                             \
    /* Iterate two times on double-double precision */                                       \
                                                                                             \
    Mul12(&_r2Sqh, &_r2Sql, _r2, _r2);                                                       \
    Add12(_r2PHr2h, _r2PHr2l, _r2, (_half * _r2));                                           \
    Mul12(&_mMr2h, &_mMr2l, _m, _r2);                                                        \
    Mul22(&_mMr2Ch, &_mMr2Cl, _mMr2h, _mMr2l, _r2Sqh, _r2Sql);                               \
                                                                                             \
    _MHmMr2Ch = -_half * _mMr2Ch;                                                            \
    _MHmMr2Cl = -_half * _mMr2Cl;                                                            \
                                                                                             \
    Add22(&_r3h, &_r3l, _r2PHr2h, _r2PHr2l, _MHmMr2Ch, _MHmMr2Cl);                           \
                                                                                             \
    Mul22(&_r3Sqh, &_r3Sql, _r3h, _r3l, _r3h, _r3l);                                         \
    Mul22(&_mMr3Sqh, &_mMr3Sql, _m, 0.0, _r3Sqh, _r3Sql);                                    \
    /* To prove: mMr3Sqh = 1.0 in each case */                                               \
                                                                                             \
    Mul22(&_r4h, &_r4l, _r3h, _r3l, 1.0, (-_half * _mMr3Sql));                               \
                                                                                             \
    /* Multiply obtained reciprocal square root by m */                                      \
                                                                                             \
    Mul22(&_srtmh,&_srtml,_m,0.0,_r4h,_r4l);                                                 \
                                                                                             \
    /* Multiply componentwise by sqrt(2^E) */                                                \
    /* which is an integer power of 2 that may not produce a subnormal */                    \
                                                                                             \
    (*(resh)) = _xdb.d * _srtmh;                                                             \
    (*(resl)) = _xdb.d * _srtml;                                                             \
                                                                                             \
  } /* End: special case 0 */                                                                \
}


#define  sqrt12_64(resh, resl, x)  {                                                         \
  db_number _xdb;                                                                            \
  int _E;                                                                                    \
  double _m, _r0, _r1, _r2, _r3h, _r3l, _r4h, _r4l, _srtmh, _srtml;                          \
  double _r2PHr2h, _r2PHr2l, _r2Sqh, _r2Sql;                                                 \
  double _mMr2h, _mMr2l, _mMr2Ch, _mMr2Cl;                                                   \
  double _MHmMr2Ch, _MHmMr2Cl;                                                               \
  double _r3Sqh, _r3Sql, _mMr3Sqh, _mMr3Sql;                                                 \
  double _half;                                                                              \
                                                                                             \
  /* Special case x = 0 */                                                                   \
  if ((x) == 0.0) {                                                                          \
    (*(resh)) = (x);                                                                         \
    (*(resl)) = 0.0;                                                                         \
  } else {                                                                                   \
                                                                                             \
    _E = 0.0;                                                                                \
                                                                                             \
    /* Convert to integer format */                                                          \
    _xdb.d = (x);                                                                            \
                                                                                             \
    /* Handle subnormal case */                                                              \
    if (_xdb.i[HI] < 0x00100000) {                                                           \
      _E = -52;                                                                              \
      _xdb.d *= ((db_number) ((double) SQRTTWO52)).d; 	                                     \
                      /* make x a normal number */                                           \
    }                                                                                        \
                                                                                             \
    /* Extract exponent E and mantissa m */                                                  \
    _E += (_xdb.i[HI]>>20)-1023;                                                             \
    _xdb.i[HI] = (_xdb.i[HI] & 0x000fffff) | 0x3ff00000;                                     \
    _m = _xdb.d;                                                                             \
                                                                                             \
    _half = 0.5;                                                                             \
    /* Make exponent even */                                                                 \
    if (_E & 0x00000001) {                                                                   \
      _E++;                                                                                  \
      _m *= _half;    /* Suppose now 1/2 <= m <= 2 */                                        \
    }                                                                                        \
                                                                                             \
    /* Construct sqrt(2^E) = 2^(E/2) */                                                      \
    _xdb.i[HI] = (_E/2 + 1023) << 20;                                                        \
    _xdb.i[LO] = 0;                                                                          \
                                                                                             \
    /* Compute initial approximation to r = 1/sqrt(m) */                                     \
                                                                                             \
    _r0 = SQRTPOLYC0 +                                                                       \
         _m * (SQRTPOLYC1 + _m * (SQRTPOLYC2 + _m * (SQRTPOLYC3 + _m * SQRTPOLYC4)));        \
                                                                                             \
    /* Iterate two times on double precision */                                              \
                                                                                             \
    _r1 = _half * _r0 * (3.0 - _m * (_r0 * _r0));                                            \
    _r2 = _half * _r1 * (3.0 - _m * (_r1 * _r1));                                            \
                                                                                             \
    /* Iterate once on double-double precision */                                            \
                                                                                             \
    Mul12(&_r2Sqh, &_r2Sql, _r2, _r2);                                                       \
    Add12(_r2PHr2h, _r2PHr2l, _r2, (_half * _r2));                                           \
    Mul12(&_mMr2h, &_mMr2l, _m, _r2);                                                        \
    Mul22(&_mMr2Ch, &_mMr2Cl, _mMr2h, _mMr2l, _r2Sqh, _r2Sql);                               \
                                                                                             \
    _MHmMr2Ch = -_half * _mMr2Ch;                                                            \
    _MHmMr2Cl = -_half * _mMr2Cl;                                                            \
                                                                                             \
    Add22(&_r3h, &_r3l, _r2PHr2h, _r2PHr2l, _MHmMr2Ch, _MHmMr2Cl);                           \
                                                                                             \
    /* Multiply obtained reciprocal square root by m */                                      \
                                                                                             \
    Mul22(&_srtmh,&_srtml,_m,0.0,_r3h,_r3l);                                                 \
                                                                                             \
    /* Multiply componentwise by sqrt(2^E) */                                                \
    /* which is an integer power of 2 that may not produce a subnormal */                    \
                                                                                             \
    (*(resh)) = _xdb.d * _srtmh;                                                             \
    (*(resl)) = _xdb.d * _srtml;                                                             \
                                                                                             \
  } /* End: special case 0 */                                                                \
}

/* 
   sqrt12_64_unfiltered = sqrt(x) * (1 + eps) where abs(eps) <= 2^(-64) 
   
   if x is neither subnormal nor 0

*/
#define  sqrt12_64_unfiltered(resh, resl, x)  {                                              \
  db_number _xdb;                                                                            \
  int _E;                                                                                    \
  double _m, _r0, _r1, _r2, _r3h, _r3l, _srtmh, _srtml;                                      \
  double _r2PHr2h, _r2PHr2l, _r2Sqh, _r2Sql;                                                 \
  double _mMr2h, _mMr2l, _mMr2Ch, _mMr2Cl;                                                   \
  double _MHmMr2Ch, _MHmMr2Cl;                                                               \
  double _half;                                                                              \
                                                                                             \
                                                                                             \
                                                                                             \
    /* Convert to integer format */                                                          \
    _xdb.d = (x);                                                                            \
                                                                                             \
                                                                                             \
    /* Extract exponent E and mantissa m */                                                  \
    _E = (_xdb.i[HI]>>20)-1023;                                                              \
    _xdb.i[HI] = (_xdb.i[HI] & 0x000fffff) | 0x3ff00000;                                     \
    _m = _xdb.d;                                                                             \
                                                                                             \
    _half = 0.5;                                                                             \
    /* Make exponent even */                                                                 \
    if (_E & 0x00000001) {                                                                   \
      _E++;                                                                                  \
      _m *= _half;    /* Suppose now 1/2 <= m <= 2 */                                        \
    }                                                                                        \
                                                                                             \
    /* Construct sqrt(2^E) = 2^(E/2) */                                                      \
    _xdb.i[HI] = (_E/2 + 1023) << 20;                                                        \
    _xdb.i[LO] = 0;                                                                          \
                                                                                             \
    /* Compute initial approximation to r = 1/sqrt(m) */                                     \
                                                                                             \
    _r0 = SQRTPOLYC0 +                                                                       \
         _m * (SQRTPOLYC1 + _m * (SQRTPOLYC2 + _m * (SQRTPOLYC3 + _m * SQRTPOLYC4)));        \
                                                                                             \
    /* Iterate two times on double precision */                                              \
                                                                                             \
    _r1 = _half * _r0 * (3.0 - _m * (_r0 * _r0));                                            \
    _r2 = _half * _r1 * (3.0 - _m * (_r1 * _r1));                                            \
                                                                                             \
    /* Iterate once on double-double precision */                                            \
                                                                                             \
    Mul12(&_r2Sqh, &_r2Sql, _r2, _r2);                                                       \
    Add12(_r2PHr2h, _r2PHr2l, _r2, (_half * _r2));                                           \
    Mul12(&_mMr2h, &_mMr2l, _m, _r2);                                                        \
    Mul22(&_mMr2Ch, &_mMr2Cl, _mMr2h, _mMr2l, _r2Sqh, _r2Sql);                               \
                                                                                             \
    _MHmMr2Ch = -_half * _mMr2Ch;                                                            \
    _MHmMr2Cl = -_half * _mMr2Cl;                                                            \
                                                                                             \
    Add22(&_r3h, &_r3l, _r2PHr2h, _r2PHr2l, _MHmMr2Ch, _MHmMr2Cl);                           \
                                                                                             \
    /* Multiply obtained reciprocal square root by m */                                      \
                                                                                             \
    Mul122(&_srtmh,&_srtml,_m,_r3h,_r3l);                                                    \
                                                                                             \
    /* Multiply componentwise by sqrt(2^E) */                                                \
    /* which is an integer power of 2 that may not produce a subnormal */                    \
                                                                                             \
    (*(resh)) = _xdb.d * _srtmh;                                                             \
    (*(resl)) = _xdb.d * _srtml;                                                             \
                                                                                             \
}



#endif /*SQRT_AS_FUNCTIONS*/

/* Declaration of the debug function */

void printHexa(char* s, double x);


#endif /*CRLIBM_PRIVATE_H*/
