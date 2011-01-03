#ifndef __DOUBLE_EXTENDED_H
#define __DOUBLE_EXTENDED_H

#if (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))
#include <fpu_control.h>
typedef long double double_ext;
#endif
#if defined(CRLIBM_TYPECPU_ITANIUM) && defined(__INTEL_COMPILER)
typedef __fpreg double_ext;
#endif


/* For debugging */
typedef union {
  int i[3];                 
  long double d;
} db_ext_number;


#define DE_EXP 2
#define DE_MANTISSA_HI 1
#define DE_MANTISSA_LO 0

#define print_ext(_s, _y) {\
db_ext_number _yy; _yy.d=_y; \
printf("%s %04x %08x %08x \n",_s, 0xffff&_yy.i[DE_EXP], _yy.i[DE_MANTISSA_HI], _yy.i[DE_MANTISSA_LO]);  \
}

/**************************************************************************************/
/*********************************Rounding tests***************************************/
/**************************************************************************************/


/* These test work by observing the bits of your double-extended after the 53rd.

   mask should be  7ff   if you trust your 64 bits (hum)
                   7fe   if you trust 63 (if you have proven that maxepsilon<2^(-63) )
                   7fc                62
                   7f8                61
                   7f0                60   etc
 */

/* Mask constants for rounding test */ 

#define ACCURATE_TO_64_BITS 0x7ff
#define ACCURATE_TO_63_BITS 0x7fe
#define ACCURATE_TO_62_BITS 0x7fc
#define ACCURATE_TO_61_BITS 0x7f8
#define ACCURATE_TO_60_BITS 0x7f0
#define ACCURATE_TO_59_BITS 0x7e0


#if (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))

static const unsigned short RN_Double     = (_FPU_DEFAULT & ~_FPU_EXTENDED)|_FPU_DOUBLE;
static const unsigned short RN_DoubleDown = (_FPU_DEFAULT & ~_FPU_EXTENDED & ~_FPU_RC_NEAREST)|_FPU_DOUBLE | _FPU_RC_DOWN;
static const unsigned short RN_DoubleUp   = (_FPU_DEFAULT & ~_FPU_EXTENDED & ~_FPU_RC_NEAREST)|_FPU_DOUBLE | _FPU_RC_UP;
static const unsigned short RN_DoubleExt  = _FPU_DEFAULT;

#define DOUBLE_EXTENDED_MODE  _FPU_SETCW(RN_DoubleExt)
#define DOUBLE_UP_MODE        _FPU_SETCW(RN_DoubleUp)
#define DOUBLE_DOWN_MODE      _FPU_SETCW(RN_DoubleDown)
#define BACK_TO_DOUBLE_MODE   _FPU_SETCW(RN_Double)


/*  
Two rounding tests to the nearest.  On Pentium 3, gcc3.3, the second
is faster by 12 cycles (and also improves the worst-case time by 60
cycles since it doesn't switch processor rounding mode in this
case). However it uses a coarser error estimation.
*/

#define DE_TEST_AND_RETURN_RN_ZIV(y,rncst)  \
{ double yh, yl;                            \
  yh = (double) y;                          \
  yl = y-yh;                                \
  BACK_TO_DOUBLE_MODE;                      \
  if(yh==yh + yl*rncst)   return yh;        \
  DOUBLE_EXTENDED_MODE;                     \
}


#define DE_TEST_AND_RETURN_RN(_y, _mask)                    \
{                                                           \
  db_ext_number _z;   double _yd;                           \
  int _lo;                                                  \
  _z.d = _y;                                                \
  _yd = (double) _y;                                        \
  _lo = _z.i[DE_MANTISSA_LO] &(_mask);                      \
  if((_lo!=(0x3ff&(_mask))) && (_lo!= (0x400&(_mask)))) {   \
    BACK_TO_DOUBLE_MODE;                                    \
    return _yd;                                             \
  }                                                         \
}


#define DE_TEST_AND_RETURN_RD(_y, _mask)                                        \
{                                                           			\
  double _result; int _bits;                                                    \
  db_ext_number _z;                                     			\
  _z.d = _y;                                                			\
  DOUBLE_DOWN_MODE;                                                             \
  _bits = _z.i[DE_MANTISSA_LO] &(_mask);     			                \
  _result = (double)(_y);	                                                \
  if( (_bits != (0xfff&(_mask)))  && (_bits != (0x000&(_mask))) ) {             \
    BACK_TO_DOUBLE_MODE;	                                                \
    return _result;                                                             \
    }                                                                           \
  DOUBLE_EXTENDED_MODE;                                                         \
}
#define DE_TEST_AND_RETURN_RU(_y, _mask)                                        \
{                                                           			\
  double _result; int _bits;                                                    \
  db_ext_number _z;                                     			\
  _z.d = _y;                                                			\
  DOUBLE_UP_MODE;                                                               \
  _bits = _z.i[DE_MANTISSA_LO] &(_mask);     			                \
  _result = (double)(_y);	                                                \
  if( (_bits != (0xfff&(_mask)))  && (_bits != (0x000&(_mask))) ) {             \
    BACK_TO_DOUBLE_MODE;	                                                \
    return _result;                                                             \
    }                                                                           \
  DOUBLE_EXTENDED_MODE;                                                         \
}


/* Use this one if you want a final computation step to overlap with		
   the rounding test. Examples: multiplication by a sign or by a power of 2 */	
										
#define DE_TEST_AND_RETURN_RN2(_ytest, _yreturn, _mask)      \
{                                                            \
  db_ext_number _z;   double _y_return_d;                    \
  int _bits;                                                 \
  _z.d = _ytest;                                             \
  _y_return_d = (double) (_yreturn);                         \
  _bits = _z.i[DE_MANTISSA_LO] &(_mask);                     \
  if((_bits!=(0x3ff&(_mask))) && (_bits!= (0x400&(_mask)))) {\
    BACK_TO_DOUBLE_MODE;                                     \
    return _y_return_d;                                      \
  }                                                          \
}




/* Slow macros with two changes of precision, but who cares, they are
   used at the end of the second step */
#define RETURN_SUM_ROUNDED_DOWN(_rh, _rl)   {\
  double _result;        \
  DOUBLE_DOWN_MODE;      \
  _result = (_rh+_rl);	 \
  BACK_TO_DOUBLE_MODE;	 \
  return _result;        \
}

#define RETURN_SUM_ROUNDED_UP(_rh, _rl)   {\
  double _result;        \
  DOUBLE_UP_MODE;        \
  _result = (_rh+_rl);	 \
  BACK_TO_DOUBLE_MODE;	 \
  return _result;        \
}

#else /* defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64) */





#if !defined(CRLIBM_TYPECPU_ITANIUM)
#error "This file should be compiled only for IA32 or IA64 architecture "
#endif

/* TODO Add what it takes to compile under HP-UX */
#if !defined(__INTEL_COMPILER)
#error "Use icc, version 8.1 or higher to compile for IA64 architecture"
#endif


#define DOUBLE_EXTENDED_MODE {}
#define BACK_TO_DOUBLE_MODE {}





#define DE_TEST_AND_RETURN_RN(_y, _mask)                                     \
{   uint64_t _mantissa, _bits; double _yd;                                   \
    _yd = _Asm_fma(2/*_FR_D*/, 1.0, _y, 0.0, 0/*SF0*/);                      \
    _mantissa = _Asm_getf(4/*_FR_SIG*/, _y);                                 \
    _bits =  _mantissa & (0x7ff&(_mask));                                    \
    if(__builtin_expect(                                                     \
         (_bits!=(0x3ff&(_mask)))  && (_bits != (0x400&(_mask))),            \
         1+1==2))                                                            \
      return _yd;                                                            \
}


/* Slower by 5 cycles as of 2005... Let us keep it, you never know */
#define DE_TEST_AND_RETURN_RN_ZIV(y,rncst)  \
{ double yh, yl;                            \
  yh = (double) y;                          \
  yl = y-yh;                                \
  if(__builtin_expect(yh == yh + yl*rncst, 1+1==2))   return yh;        \
}

/* Use this one if you want a final computation step to overlap with
   the rounding test. Examples: multiplication by a sign or by a power of 2 */

#define DE_TEST_AND_RETURN_RN2(_ytest, _yreturn, _mask)                      \
{   uint64_t _mantissa, _bits;                                               \
    _mantissa = _Asm_getf(4/*_FR_SIG*/, _ytest);                             \
    _bits =  _mantissa & (0x7ff&(_mask));                                    \
    if(__builtin_expect(                                                     \
         (_bits!=(0x3ff&(_mask)))  && (_bits != (0x400&(_mask))),            \
         1+1==2))                                                            \
      return _yreturn;                                                       \
}


#define DE_TEST_AND_RETURN_RD(_y, _mask)                                     \
{   uint64_t _mantissa, _bits; double _yd;                                   \
    _yd = _Asm_fma(2/*_FR_D*/, -1.0, _y, 0.0, 3/*SF3*/);                     \
    _mantissa = _Asm_getf(4/*_FR_SIG*/, _y);                                 \
    _bits =  _mantissa & (0x7ff&(_mask));                                    \
    if(__builtin_expect(                                                     \
         (_bits!=(0x000&(_mask)))  && (_bits != (0x7ff&(_mask))),            \
         1+1==2))                                                            \
      return -_yd;                                                           \
}

#define DE_TEST_AND_RETURN_RU(_y, _mask)                                     \
{   uint64_t _mantissa, _bits; double _yd;                                   \
    _yd = _Asm_fma(2/*_FR_D*/, 1.0, _y, 0.0, 3/*SF3*/);                      \
    _mantissa = _Asm_getf(4/*_FR_SIG*/, _y);                                 \
    _bits =  _mantissa & (0x7ff&(_mask));                                    \
    if(__builtin_expect(                                                     \
         (_bits!=(0x000&(_mask)))  && (_bits != (0x7ff&(_mask))),            \
         1+1==2))                                                            \
      return _yd;                                                            \
}

#define RETURN_SUM_ROUNDED_DOWN(_rh, _rl) \
   return -_Asm_fma(2/*_FR_D*/, -1.0, _rh, -_rl, 3/*SF3*/);

#define RETURN_SUM_ROUNDED_UP(_rh, _rl) \
   return _Asm_fma(2/*_FR_D*/, 1.0, _rh, _rl, 3/*SF3*/);


#if 0

/* This test doesn'use SF2 and SF3  Kept as a model for Pentium implementation, to erase afterwards */

#define DE_TEST_AND_RETURN_RU(_y, _mask)                                     \
{                                                           		     \
  db_ext_number _z;   double    _yd;                        		     \
  unsigned long int _mantissa, _y_double, _isNegative ,_wasRoundedUp, _bits; \
  _yd=(double) _y;                                                           \
  _mantissa = _Asm_getf(4/*_FR_SIG*/, _y);                                   \
  _y_double = _Asm_getf(2/*_FR_D*/, _yd);                                    \
  _bits =  _mantissa & (0x7ff&(_mask));                                      \
  _wasRoundedUp = ((_mantissa >>11) & 1) != (_y_double & 1);       	     \
  _bits =  _mantissa & (0x7ff&(_mask));                                      \
  _isNegative = _y_double >> 63;                      		    	     \
  if(_isNegative) {    							     \
    if(_wasRoundedUp) { /* RN was RD */	                                     \
      if( _bits != (0x7ff&(_mask)) ) {                          	     \
        _y_double--;                                                         \
        return (double) _Asm_setf(2/*_FR_D*/, _y_double);                    \
      } /* else launch accurate phase */ 				     \
    }									     \
    else{ /* RN was RU, so need to check  */                    	     \
      if( _bits != (0x000&(_mask)) ) {                          	     \
        return    _yd;                                           	     \
      } /* else launch accurate phase */ 				     \
    }									     \
  }									     \
  else{ /* Positive number */                                                \
    if(_wasRoundedUp) { /* RN was RU */                                      \
      if( _bits != (0x7ff&(_mask)) ) {                          	     \
        return  _yd;                                            	     \
      } /* else launch accurate phase */ 				     \
    }									     \
    else{ /* RN was RD,  */                                                  \
      if( _bits != (0x000&(_mask)) ) {                          	     \
        _y_double++; /* beware, does not work for -0 */                      \
        return (double) _Asm_setf(2/*_FR_D*/, _y_double);                    \
      } /* else launch accurate phase */ 				     \
    }                                                                        \
  }									     \
}




#define DE_TEST_AND_RETURN_RD(_y, _mask)                                     \
{                                                           		     \
  db_ext_number _z;   double    _yd;                        		     \
  unsigned long int _mantissa, _y_double, _isNegative ,_wasRoundedUp, _bits; \
  _yd=(double) _y;                                                           \
  _mantissa = _Asm_getf(4/*_FR_SIG*/, _y);                                   \
  _y_double = _Asm_getf(2/*_FR_D*/, _yd);                                    \
  _bits =  _mantissa & (0x7ff&(_mask));                                      \
  _wasRoundedUp = ((_mantissa >>11) & 1) != (_y_double & 1);       	     \
  _bits =  _mantissa & (0x7ff&(_mask));                                      \
  _isNegative = _y_double >> 63;                      		    	     \
  if(_isNegative) {    							     \
    if(_wasRoundedUp) { /* RN was RD */	                                     \
      if( _bits != (0x7ff&(_mask)) ) {                          	     \
        return (double) _y;                                           	     \
      } /* else launch accurate phase */ 				     \
    }									     \
    else{ /* RN was RU  */                                       	     \
      if( _bits != (0x000&(_mask)) ) {                          	     \
        _y_double++;                                                         \
        return (double) _Asm_setf(2/*_FR_D*/, _y_double);                    \
      } /* else launch accurate phase */ 				     \
    }									     \
  }									     \
  else{ /* Positive number */                                                \
    if(_wasRoundedUp) { /* RN was RU */                                      \
      if( _bits != (0x7ff&(_mask)) ) {                          	     \
        _y_double--;                                                         \
        return (double) _Asm_setf(2/*_FR_D*/, _y_double);                    \
      } /* else launch accurate phase */ 				     \
    }									     \
    else{ /* RN was RD,  */                                                  \
      if( _bits != (0x000&(_mask)) ) {                          	     \
        return (double) _y;                                           	     \
      } /* else launch accurate phase */ 				     \
    }                                                                        \
  }									     \
}									     

#endif  /* 0 */



#endif /* defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64) */







/**************************************************************************************/
/************************Double double-extended arithmetic*****************************/
/**************************************************************************************/



#define Add12_ext(prh, prl, a, b)       \
{                                       \
  double_ext _z, _a, _b;                \
  _a = a;   _b = b;                     \
  *prh = _a + _b;                       \
  _z = *prh - _a;                       \
  *prl = _b - _z;                       \
}




#if (defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64))
#define Mul12_ext(prh,prl,u,v)                         \
{                                                      \
  const double_ext _c  = 4294967297.L; /* 2^32 +1 */   \
  double_ext _up, _u1, _u2, _vp, _v1, _v2;             \
  double_ext _u =u, _v=v;                              \
                                                       \
  _up = _u*_c;         _vp = _v*_c;                    \
  _u1 = (_u-_up)+_up;  _v1 = (_v-_vp)+_vp;             \
  _u2 = _u-_u1;        _v2 = _v-_v1;                   \
                                                       \
  *prh = _u*_v;                                        \
  *prl = _u1*_v1 - *prh;                               \
  *prl = *prl + _u1*_v2;                               \
  *prl = *prl + _u2*_v1;                               \
  *prl = *prl + _u2*_v2;                               \
}

#define Mul22_ext(prh,prl, ah,al, bh,bl)               \
{                                                      \
  double_ext mh, ml;                                  \
  Mul12_ext(&mh,&ml,(ah),(bh));		               \
  ml += (ah)*(bl) + (al)*(bh);			       \
  Add12_ext(prh,prl, mh,ml);                           \
}

#define FMA22_ext(prh,prl, ah,al, bh,bl, ch,cl)        \
{                                                      \
  Mul22_ext(prh,prl, (ah),(al), (bh),(bl));            \
  Add22_ext(prh,prl, ch,cl, *prh, *prl);               \
}



#else  /* defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64) */






#define Mul12_ext( prh,prl, a, b )                              \
    {                                                           \
      *prh = (a) * (b);                                         \
      *prl = _Asm_fms( 3/*_PC_NONE*/, (a), (b), *prh, 1 );      \
    }


#if 0 /* transcription of Alexey's */
#define Mul22_ext( prh,prl, ah,al, bh,bl ) \
    {                                                            \
        double_ext _t1,_t2,_t3;                                  \
        *prh = (ah) * (bh);                                      \
        _t1 = (ah)*(bl);                                         \
        _t2 = _Asm_fms( 3/*_PC_NONE*/, (ah), (bh), *prh, 1 );    \
        _t3 = (al) * (bh) + _t1;                                 \
        *prl = (_t2 + _t3);                                      \
    }
#else
#define Mul22_ext( prh,prl, ah,al, bh,bl )                \
{                                                         \
  double_ext ph, pl;                                      \
  ph = (ah)*(bh);                                         \
  pl = _Asm_fms( 3/*_PC_NONE*/, ah, bh, ph, 1/*_SF1*/ );  \
  pl = (ah)*(bl) + pl;                                    \
  pl = (al)*(bh) + pl;                                    \
  Add12_ext(prh,prl, ph,pl);                              \
}
#endif

/* res = a*b + c, assume |a*b| <= |c|
 *   res, a and b in X format
 *   c in L format
 */

#if 0
#define FMA22_ext(prh,prl, ah,al, bh,bl, ch,cl)        \
{                                                      \
  Mul22_ext(prh,prl, (ah),(al), (bh),(bl));            \
  Add22_ext(prh,prl, ch,cl, *prh, *prl);               \
}
#else
#define FMA22_ext( prh,prl, ah,al,  bh,bl, ch,cl) \
    {                                                                                      \
        double_ext __xfmagxxx_r_hi__,__xfmagxxx_r_lo__,                                    \
                __xfmagxxx_t1__,__xfmagxxx_t2__,                                           \
                __xfmagxxx_t3__,__xfmagxxx_t4__;                                           \
        __xfmagxxx_r_hi__ = ah * bh + ch;                                                  \
        __xfmagxxx_t1__ = al * bh + cl;                                                    \
        __xfmagxxx_t2__ = __xfmagxxx_r_hi__ - ch;                                          \
        __xfmagxxx_t3__ = ah * bl + __xfmagxxx_t1__;                                       \
        __xfmagxxx_t4__ = _Asm_fms( 3/*_PC_NONE*/, ah, bh, __xfmagxxx_t2__, 1/*_SF1*/ );   \
        __xfmagxxx_r_lo__ = (__xfmagxxx_t3__ + __xfmagxxx_t4__);                           \
        *prh = __xfmagxxx_r_hi__; *prl = __xfmagxxx_r_lo__;                                \
    }
#endif

#endif    /* defined(CRLIBM_TYPECPU_X86) || defined(CRLIBM_TYPECPU_AMD64) */








#define  Div22_ext(prh,prl,xh,xl,yh,yl)             \
{                                                   \
  double_ext ch,cl,uh,ul;                           \
  ch = (xh)/(yh);                                   \
  Mul12_ext(&uh,&ul,ch,(yh));                       \
  cl = (xh)-uh;                                     \
  cl = cl - ul;                                     \
  cl = cl + (xl);                                   \
  cl = cl - ch*(yl);                                \
  cl = cl / (yh);                                   \
  Add12(prh,prl, ch, cl) ;                          \
}


#define Add22_ext(prh,prl,xh,xl,yh,yl)   \
do {                                     \
  double_ext _r,_s;                      \
  _r = (xh)+(yh);                        \
  _s = (xh)-_r;                          \
  _s = _s + (yh);                        \
  _s = _s + (yl);                        \
  _s = _s + (xl);                        \
  Add12_ext(prh,prl,_r,_s);              \
} while(0)

#endif /* ifndef __DOUBLE_EXTENDED_H*/
