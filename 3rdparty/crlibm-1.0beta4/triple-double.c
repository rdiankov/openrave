/*
 *  triple_double.c
 *  
 * This file contains useful tools and data for triple double data representation.
 *
 */

#include "triple-double.h"
#include "crlibm_private.h"


#if TRIPLEDOUBLE_AS_FUNCTIONS

#if 0
void Renormalize3(double* resh, double* resm, double* resl, double ah, double am, double al)
{                                                      
  DoRenormalize3(resh, resm, resl, ah, am, al);
}
#endif


void Mul23(double* resh, double* resm, double* resl, double ah, double al, double bh, double bl)                
{
  DoMul23(resh, resm, resl, ah, al, bh, bl);
}

void Mul233(double* resh, double* resm, double* resl, double ah, double al, double bh, double bm, double bl)            
{
  DoMul233(resh, resm, resl, ah, al, bh, bm, bl);
}

void Mul33(double* resh, double* resm, double* resl, double ah, double am, double al, double bh, double bm, double bl)            
{
  DoMul33(resh, resm, resl, ah, am, al, bh, bm, bl);
}

void Mul133(double* resh, double* resm, double* resl, double a, double bh, double bm, double bl)
{
  DoMul133(resh, resm, resl, a, bh, bm, bl);
}

void Mul123(double* resh, double* resm, double* resl, double a, double bh,  double bl)
{
  DoMul123(resh, resm, resl, a, bh, bl);
}

void Sqrt13(double* resh, double* resm, double* resl, double x)
{
  DoSqrt13(resh, resm, resl , x);
}

void Recpr33(double* resh, double* resm, double* resl, double dh, double dm, double dl)
{
  DoRecpr33(resh, resm, resl, dh, dm, dl);
}

#endif /* TRIPLEDOUBLE_AS_FUNCTIONS*/
