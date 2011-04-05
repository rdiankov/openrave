/*
 *  triple_double.h
 *  
 * This file contains useful tools and data for triple double data representation.
 *
 */

#ifndef TRIPLE_DOUBLE_H
#define TRIPLE_DOUBLE_H 1

#include "scs_lib/scs.h"
#include "scs_lib/scs_private.h"

 /* undef all the variables that might have been defined in
    scs_lib/scs_private.h */
//#undef VERSION 
//#undef PACKAGE 
//#undef HAVE_GMP_H
//#undef HAVE_MPFR_H
//#undef HAVE_MATHLIB_H
///* then include the proper definitions  */
//#include "crlibm_config.h"

#ifdef HAVE_INTTYPES_H
#include <inttypes.h>
#endif

 /* Set to O for larger but faster functions.
    As it only impacts the second step, smaller is preferred */
#define TRIPLEDOUBLE_AS_FUNCTIONS 0


#define  Renormalize3(resh, resm, resl, ah, am, al)     DoRenormalize3(resh, resm, resl, ah, am, al)  
/*extern void Renormalize3(double* resh, double* resm, double* resl, double ah, double am, double al) ;*/

#if TRIPLEDOUBLE_AS_FUNCTIONS
extern void Mul23(double* resh, double* resm, double* resl, double ah, double al, double bh, double bl);
extern void Mul233(double* resh, double* resm, double* resl, double ah, double al, double bh, double bm, double bl);
extern void Mul33(double* resh, double* resm, double* resl, double ah, double am, double al, double bh, double bm, double bl);
extern void Mul133(double* resh, double* resm, double* resl, double a, double bh, double bm, double bl);
extern void Mul123(double* resh, double* resm, double* resl, double a, double bh, double bl);
extern void Sqrt13(double* resh, double* resm, double* resl, double x);
extern void Recpr33(double* resh, double* resm, double* resl, double dh, double dm, double dl);
#else
#define  Mul23(resh, resm, resl, ah, al, bh, bl)           DoMul23(resh, resm, resl, ah, al, bh, bl)
#define  Mul233(resh, resm, resl, ah, al, bh, bm, bl)      DoMul233(resh, resm, resl, ah, al, bh, bm, bl)
#define  Mul33(resh, resm, resl, ah, am, al, bh, bm, bl)   DoMul33(resh, resm, resl, ah, am, al, bh, bm, bl)
#define  Mul133(resh, resm, resl, a, bh, bm, bl)           DoMul133(resh, resm, resl, a, bh, bm, bl)
#define  Mul123(resh, resm, resl, a, bh, bl)               DoMul123(resh, resm, resl, a, bh, bl)    
#define  Sqrt13(resh, resm, resl , x)                      DoSqrt13(resh, resm, resl , x)
#define  Recpr33(resh, resm, resl, dh, dm, dl)             DoRecpr33(resh, resm, resl, dh, dm, dl)
#endif



/* Renormalize3

   Procedure for renormalizing a triple double number, i.e.
   computing exactly an equivalent sum of three non-overlapping
   double numbers


   Arguments:       a triple double number ah, am, al
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(ah) > abs(am) > abs(al)
                    ah and am are overlapping not more than 51 bits
                    am and al are overlapping not more than 51 bits

   Guarantees:      abs(resh) > abs(resm) > abs(resl)
                    resh and resm are non-overlapping
		    resm and resl are non-overlapping
		    resm = round-to-nearest(resm + resl)

   Details:         resh, resm and resl are considered to be pointers

*/
#define  DoRenormalize3(resh, resm, resl, ah, am, al)     \
{                                                      \
    double _t1h, _t1l, _t2l;                           \
                                                       \
    Add12(_t1h, _t1l, (am), (al));                     \
    Add12((*(resh)), _t2l, (ah), (_t1h));              \
    Add12((*(resm)), (*(resl)), _t2l, _t1l);           \
}


/* Mul23

   Procedure for multiplying two double double numbers resulting
   in a triple double number


   Arguments:       two double double numbers:
                    ah, al and
		    bh, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(ah) > abs(al) 
                    ah and al do not overlap
		    ah = round-to-nearest(ah + al)
		    abs(bh) > abs(bl) 
                    bh and bl do not overlap
		    bh = round-to-nearest(bh + bl)
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-49) * abs(resh)
		    resh+resm+resl = (ah+al) * (bh+bl) * (1 + eps)
		    where
		    abs(eps) <= 2^(-149)

   Details:         resh, resm and resl are considered to be pointers
*/
#define  DoMul23(resh, resm, resl, ah, al, bh, bl)                \
{                                                              \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8, _t9, _t10;  \
                                                               \
    Mul12((resh),&_t1,(ah),(bh));                              \
    Mul12(&_t2,&_t3,(ah),(bl));                                \
    Mul12(&_t4,&_t5,(al),(bh));                                \
    _t6 = (al) * (bl);                                         \
    Add22Cond(&_t7,&_t8,_t2,_t3,_t4,_t5);                      \
    Add12(_t9,_t10,_t1,_t6);                                   \
    Add22Cond((resm),(resl),_t7,_t8,_t9,_t10);                 \
}



/* Mul233

   Procedure for multiplying a double double number by 
   a triple double number resulting in a triple double number


   Arguments:       a double double number ah, al
                    a triple double number bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(ah) > abs(al)
                    ah and al do not overlap
		    ah = round-to-nearest(ah + al)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 1
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(\gamma) * abs(resh)
		    where
		    \gamma >= min(48,b_o-4,b_o+b_u-4)
		    resh+resm+resl=(ah+al) * (bh+bm+bl) * (1+eps)
		    where
		    abs(eps) <= 
                       (2^(-99-b_o) + 2^(-99-b_o-b_u) + 2^(-152)) / 
		         (1 - 2^(-53) - 2^(-b_o+1) - 2^(-b_o-b_u+1))

   Details:         resh, resm and resl are considered to be pointers
*/
#define  DoMul233(resh, resm, resl, ah, al, bh, bm, bl)            \
{                                                               \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8, _t9, _t10;   \
    double _t11, _t12, _t13, _t14, _t15, _t16, _t17, _t18;      \
                                                                \
    Mul12((resh),&_t1,(ah),(bh));                               \
    Mul12(&_t2,&_t3,(ah),(bm));                                 \
    Mul12(&_t4,&_t5,(ah),(bl));                                 \
    Mul12(&_t6,&_t7,(al),(bh));                                 \
    Mul12(&_t8,&_t9,(al),(bm));                                 \
    _t10 = (al) * (bl);                                         \
    Add22Cond(&_t11,&_t12,_t2,_t3,_t4,_t5);                     \
    Add22Cond(&_t13,&_t14,_t6,_t7,_t8,_t9);                     \
    Add22Cond(&_t15,&_t16,_t11,_t12,_t13,_t14);                 \
    Add12Cond(_t17,_t18,_t1,_t10);                              \
    Add22Cond((resm),(resl),_t17,_t18,_t15,_t16);               \
}




/* Add33

   Procedure for adding two triple double numbers resulting
   in a triple double number


   Arguments:       two triple double numbers:
                    ah, am, al and
		    bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bh) <= 0.75 * abs(ah)  OR  ( sign(bh) = sign(ah) AND abs(bh) <= abs(ah))  (i)
                    abs(am) <= 2^(-a_o) * abs(ah)
		    abs(al) <= 2^(-a_u) * abs(am)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= a_o >= 4
		    b_u >= a_u >= 4

		    Condition (i) may not be respected if 
		    one can assume in this case that ah=am=al=0
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-min(a_o,b_o) + 5) * abs(resh)
		    resh+resm+resl = (ah+am+al + bh+bm+bl) * (1+eps)
                    where 
		    abs(eps) <= 2^(-min(a_o+a_u,b_o+b_u)-47) + 2^(-min(a_o,a_u)-98)

   Details:         resh, resm and resl are considered to be pointers
*/

#define  Add33(resh, resm, resl, ah, am, al, bh, bm, bl)      \
{                                                            \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8;           \
                                                             \
    Add12((*(resh)),_t1,(ah),(bh));                          \
    Add12Cond(_t2,_t3,(am),(bm));                            \
    _t6 = (al) + (bl);                                       \
    Add12Cond(_t7,_t4,_t1,_t2);                              \
    _t5 = _t3 + _t4;                                         \
    _t8 = _t5 + _t6;                                         \
    Add12Cond((*(resm)),(*(resl)),_t7,_t8);                  \
}

/* Add33Cond

   Procedure for adding two triple double numbers resulting
   in a triple double number


   Arguments:       two triple double numbers:
                    ah, am, al and
		    bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(am) <= 2^(-a_o) * abs(ah)
		    abs(al) <= 2^(-a_u) * abs(am)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= a_o >= 4
		    b_u >= a_u >= 4

		    Condition (i) may not be respected if 
		    one can assume in this case that ah=am=al=0
		    
   Guarantees:      TODO
   Details:         resh, resm and resl are considered to be pointers
*/

#define  Add33Cond(resh, resm, resl, ah, am, al, bh, bm, bl)      \
{                                                            \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8;           \
                                                             \
    Add12Cond((*(resh)),_t1,(ah),(bh));                          \
    Add12Cond(_t2,_t3,(am),(bm));                            \
    _t6 = (al) + (bl);                                       \
    Add12Cond(_t7,_t4,_t1,_t2);                              \
    _t5 = _t3 + _t4;                                         \
    _t8 = _t5 + _t6;                                         \
    Add12Cond((*(resm)),(*(resl)),_t7,_t8);                  \
}



/* Add233

   Procedure for adding a double double number to a triple 
   double number resulting in a triple double number


   Arguments:       a double double number ah, al
                    a triple double number bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(ah) > abs(al)
                    ah and al do not overlap
		    ah = round-to-nearest(ah + al)
		    abs(bh) <= 2^(-2) * abs(ah)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 1
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(\gamma) * abs(resh)
		    where
		    \gamma >= min(45,b_o-4,b_o+b_u-2)
		    resh+resm+resl=((ah+al) + (bh+bm+bl)) * (1+eps)
		    where
		    abs(eps) <= 
                       <= 2^(-b_o-b_u-52) + 2^(-b_o-104) + 2^(-153)

   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add233(resh, resm, resl, ah, al, bh, bm, bl)            \
{                                                               \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7;                   \
                                                                \
    Add12((*(resh)),_t1,(ah),(bh));                             \
    Add12Cond(_t2,_t3,(al),(bm));                               \
    Add12Cond(_t4,_t5,_t1,_t2);                                 \
    _t6 = _t3 + (bl);                                           \
    _t7 = _t6 + _t5;                                            \
    Add12Cond((*(resm)),(*(resl)),_t4,_t7);                     \
}

/* Add123

   Procedure for adding a double number to a double 
   double number resulting in a triple double number


   Arguments:       a double number a 
                    a double double number bh, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bh) <= 2^(-2) * abs(a)
		    abs(bl) <= 2^(-53) * abs(bh)
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-\gamma) * abs(resh)
		    where
		    
		    \gamma >= 52

		    resh+resm+resl=(a + (bh+bm+bl)) exactly
		    

   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add123(resh, resm, resl, a, bh, bl)                     \
{                                                               \
    double _t1;                                                 \
                                                                \
    Add12((*(resh)),_t1,(a),(bh));                              \
    Add12((*(resm)),(*(resl)),_t1,(bl));                        \
}

/* Add213

   Procedure for adding a double double number to a double 
   number resulting in a triple double number


   Arguments:       a double double number ah, al 
                    a double number b
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(b) <= 2^(-2) * abs(ah)
		    abs(al) <= 2^(-53) * abs(ah)
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-\gamma) * abs(resh)
		    where
		    
		    \gamma >= 52

		    resh+resm+resl=(a + (bh+bm+bl)) exactly
		    

   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add213(resh, resm, resl, ah, al, b)                     \
{                                                               \
    double _t1;                                                 \
                                                                \
    Add12((*(resh)),_t1,(ah),(b));                              \
    Add12Cond((*(resm)),(*(resl)),(al),(b));                    \
}



/* Add23

   Procedure for adding a double-double number to a double-double 
   number resulting in a triple double number


   Arguments:       a double double number ah, al
                    a double double number bh, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bh) <= 2^(-2) * abs(ah)
                    abs(al) <= 2^(-53) * abs(ah)
		    abs(bl) <= 2^(-53) * abs(bh)
		    
   Guarantees:      TO DO
		    

   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add23(resh, resm, resl, ah, al, bh, bl)                 \
{                                                               \
    double _t1, _t2, _t3, _t4, _t5, _t6;                        \
                                                                \
    Add12((*(resh)),_t1,(ah),(bh));                             \
    Add12Cond(_t2,_t3,(al),(bl));                               \
    Add12Cond(_t4,_t5,_t1,_t2);                                 \
    _t6 = _t3 + _t5;                                            \
    Add12Cond((*(resm)),(*(resl)),_t4,_t6);                     \
}




/* Add133

   Procedure for adding a double number to a triple 
   double number resulting in a triple double number


   Arguments:       a double number a 
                    a triple double number bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bh) <= 2^(-2) * abs(a)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 1
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(\gamma) * abs(resh)
		    where
		    \gamma >= min(47,2-b_o,1-b_o-b_u)
		    resh+resm+resl=(a + (bh+bm+bl)) * (1+eps)
		    where
		    abs(eps) <= 
                       <= 2^(-52-b_o-b_u) + 2^(-154)


   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add133(resh, resm, resl, a, bh, bm, bl)                 \
{                                                               \
    double _t1, _t2, _t3, _t4;                                  \
                                                                \
    Add12((*(resh)),_t1,(a),(bh));                              \
    Add12Cond(_t2,_t3,_t1,(bm));                                \
    _t4 = _t3 + (bl);                                           \
    Add12Cond((*(resm)),(*(resl)),_t2,_t4);                     \
}

/* Add133Cond

   Procedure for adding a double number to a triple 
   double number resulting in a triple double number


   Arguments:       a double number a 
                    a triple double number bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 1
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(\gamma) * abs(resh)
		    where

		    TODO

		    resh+resm+resl=(a + (bh+bm+bl)) * (1+eps)
		    where
		    abs(eps) <= 

		    TODO


   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add133Cond(resh, resm, resl, a, bh, bm, bl)             \
{                                                               \
    double _t1, _t2, _t3, _t4;                                  \
                                                                \
    Add12Cond((*(resh)),_t1,(a),(bh));                          \
    Add12Cond(_t2,_t3,_t1,(bm));                                \
    _t4 = _t3 + (bl);                                           \
    Add12Cond((*(resm)),(*(resl)),_t2,_t4);                     \
}



/* Add233Cond

   Procedure for adding a double double number to a triple 
   double number resulting in a triple double number


   Arguments:       a double double number ah, al
                    a triple double number bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(ah) > abs(al)
                    ah and al do not overlap
		    ah = round-to-nearest(ah + al)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 1
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(\gamma) * abs(resh)
		    where
		    \gamma >= ????
		    resh+resm+resl=((ah+al) + (bh+bm+bl)) * (1+eps)
		    where
		    abs(eps) <= 
                       <= ????

   Details:         resh, resm and resl are considered to be pointers
*/
#define  Add233Cond(resh, resm, resl, ah, al, bh, bm, bl)        \
{                                                               \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7;                   \
                                                                \
    Add12Cond((*(resh)),_t1,(ah),(bh));                         \
    Add12Cond(_t2,_t3,(al),(bm));                               \
    Add12Cond(_t4,_t5,_t1,_t2);                                 \
    _t6 = _t3 + (bl);                                           \
    _t7 = _t6 + _t5;                                            \
    Add12Cond((*(resm)),(*(resl)),_t4,_t7);                     \
}




/* Mul33

   Procedure for multiplying two triple double numbers resulting
   in a triple double number


   Arguments:       two triple double numbers:
                    ah, am, al and
		    bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(am) <= 2^(-a_o) * abs(ah)
		    abs(al) <= 2^(-a_u) * abs(am)
		    abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o, a_o >= 5
		    b_u, a_u >= 5

		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-g_o) * abs(resh)
		    with
		    g_o > min(48,-4+a_o,-4+b_o,-4+a_o-b_o)
		    resh+resm+resl = (ah+am+al) * (bh+bm+bl) * (1+eps)
                    where 
		    abs(eps) <= 2^-151 + 2^-99-a_o + 2^-99-b_o +
		    + 2^-49-a_o-a_u + 2^-49-b_o-b_u + 2^50-a_o-b_o-b_u + 
		    + 2^50-a_o-b_o-b_u + 2^-101-a_o-b_o + 2^-52-a_o-a_u-b_o-b_u

   Details:         resh, resm and resl are considered to be pointers
*/

#define  DoMul33(resh, resm, resl, ah, am, al, bh, bm, bl)      \
{                                                            \
    double _t1, _t2, _t3, _t4, _t5, _t6, _t7, _t8, _t9;      \
    double _t10, _t11, _t12, _t13, _t14, _t15, _t16, _t17;   \
    double _t18, _t19, _t20, _t21, _t22;                     \
                                                             \
    Mul12((resh),&_t1,(ah),(bh));                            \
    Mul12(&_t2,&_t3,(ah),(bm));                              \
    Mul12(&_t4,&_t5,(am),(bh));                              \
    Mul12(&_t6,&_t7,(am),(bm));                              \
    _t8 = (ah) * (bl);                                       \
    _t9 = (al) * (bh);                                       \
    _t10 = (am) * (bl);                                      \
    _t11 = (al) * (bm);                                      \
    _t12 = _t8 + _t9;                                        \
    _t13 = _t10 + _t11;                                      \
    Add12Cond(_t14,_t15,_t1,_t6);                            \
    _t16 = _t7 + _t15;                                       \
    _t17 = _t12 + _t13;                                      \
    _t18 = _t16 + _t17;                                      \
    Add12Cond(_t19,_t20,_t14,_t18);                          \
    Add22Cond(&_t21,&_t22,_t2,_t3,_t4,_t5);                  \
    Add22Cond((resm),(resl),_t21,_t22,_t19,_t20);            \
}


/* Mul133

   Procedure for multiplying double by a triple double number resulting
   in a triple double number


   Arguments:       a double a
		    a triple double bh, bm, bl
   
   Results:         a triple double number resh, resm, resl

   Preconditions:   abs(bm) <= 2^(-b_o) * abs(bh)
		    abs(bl) <= 2^(-b_u) * abs(bm)
		    where
		    b_o >= 2
		    b_u >= 2

		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-g_o) * abs(resh)
		    with
		    g_o > min(47,-5-b_o,-5+b_o+b_u) 
		    resh+resm+resl = a * (bh+bm+bl) * (1+eps)
                    where 
		    abs(eps) <= 2^-49-b_o-b_u + 2^-101-b_o + 2^-156

   Details:         resh, resm and resl are considered to be pointers
*/
#define  DoMul133(resh, resm, resl, a, bh, bm, bl)            \
{                                                          \
    double _t2, _t3, _t4, _t5, _t7, _t8, _t9, _t10;        \
                                                           \
    Mul12((resh),&_t2,(a),(bh));                           \
    Mul12(&_t3,&_t4,(a),(bm));                             \
    _t5 = (a) * (bl);                                      \
    Add12Cond(_t9,_t7,_t2,_t3);                            \
    _t8 = _t4 + _t5;                                       \
    _t10 = _t7 + _t8;                                      \
    Add12Cond((*(resm)),(*(resl)),_t9,_t10);               \
}

/* Mul123

   Procedure for multiplying double by a double double number resulting
   in a triple double number


   Arguments:       a double a
		    a double double bh, bl
   
   Results:         a triple double number resh, resm, resl
		    
   Guarantees:      resm and resl are non-overlapping
                    resm = round-to-nearest(resm + resl)
		    abs(resm) <= 2^(-g_o) * abs(resh)
		    with
		    g_o > 47 
		    resh+resm+resl = a * (bh+bm) * (1+eps)
                    where 
		    abs(eps) <= 2^-154

   Details:         resh, resm and resl are considered to be pointers
*/
#define  DoMul123(resh, resm, resl, a, bh, bl)                \
{                                                          \
    double _t1, _t2, _t3, _t4, _t5, _t6;                   \
                                                           \
    Mul12((resh),&_t1,(a),(bh));                           \
    Mul12(&_t2,&_t3,(a),(bl));                             \
    Add12Cond(_t5,_t4,_t1,_t2);                            \
    _t6 = _t3 + _t4;                                       \
    Add12Cond((*(resm)),(*(resl)),_t5,_t6);                \
}



/* ReturnRoundToNearest3

   Procedure for rounding a triple to a double number
   in round-to-nearest-ties-to-even mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0
		    xl = 0 iff xm != +/- 0.5 * ulp(xh) (0.25 if xh = 2^e)
		    		    
   Guarantees:      xprime = RN(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundToNearest3(xh,xm,xl)                       \
{                                                             \
    double _t1, _t2, _t3, _t4, _t5, _t6;                      \
    db_number _xp, _xn;                                       \
                                                              \
    _xp.d = (xh);                                             \
    _xn.i[HI] = _xp.i[HI];                                    \
    _xn.i[LO] = _xp.i[LO];                                    \
    _xn.l--;                                                  \
    _t1 = _xn.d;                                              \
    _xp.l++;                                                  \
    _t4 = _xp.d;                                              \
    _t2 = (xh) - _t1;                                         \
    _t3 = _t2 * -0.5;                                         \
    _t5 = _t4 - (xh);                                         \
    _t6 = _t5 * 0.5;                                          \
    if (((xm) != _t3) && ((xm) != _t6)) return ((xh) + (xm)); \
    if ((xm) * (xl) > 0.0) {                                  \
      if ((xh) * (xl) > 0.0)                                  \
        return _t4;                                           \
      else                                                    \
        return _t1;                                           \
    } else return (xh);                                       \
}

/* ReturnRoundToNearest3Other

   ATTENTION: THIS CURRENTLY UNPROVEN CODE !!!

   Procedure for rounding a triple to a double number
   in round-to-nearest-ties-to-even mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   |xm + xl| <= 2^(-5) * |xh|
		    		    
   Guarantees:      xprime = RN(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundToNearest3Other(xh,xm,xl)                  \
{                                                             \
    double _t3, _t4;                                          \
    db_number _t3db;                                          \
                                                              \
    Add12(_t3,_t4,(xm),(xl));                                 \
    if (_t4 != 0.0) {                                         \
      _t3db.d = _t3;                                          \
      if (!(_t3db.i[LO] & 0x00000001)) {                      \
        if ((_t4 > 0.0) ^ ((_t3db.i[HI] & 0x80000000) != 0))  \
           _t3db.l++;                                         \
        else                                                  \
           _t3db.l--;                                         \
        _t3 = _t3db.d;                                        \
      }                                                       \
    }                                                         \
    return (xh) + _t3;                                        \
}



/* ReturnRoundUpwards3

   Procedure for rounding a triple to a double number
   in round-upwards mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RU(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundUpwards3(xh,xm,xl)                         \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t3 > 0.0) {                                          \
      if (_t1 > 0.0) {                                        \
         _tdb.d = _t1;                                        \
         _tdb.l++;                                            \
         return _tdb.d;                                       \
      } else {                                                \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         return _tdb.d;                                       \
      }                                                       \
    } else return _t1;                                        \
}


/* ReturnRoundDownwards3

   Procedure for rounding a triple to a double number
   in round-downwards mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RD(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundDownwards3(xh,xm,xl)                       \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t3 < 0.0) {                                          \
      if (_t1 > 0.0) {                                        \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         return _tdb.d;                                       \
      } else {                                                \
         _tdb.d = _t1;                                        \
         _tdb.l++;                                            \
         return _tdb.d;                                       \
      }                                                       \
    } else return _t1;                                        \
}


/* ReturnRoundTowardsZero3

   Procedure for rounding a triple to a double number
   in round-towards-zero mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RZ(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundTowardsZero3(xh,xm,xl)                     \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t1 > 0.0) {                                          \
       if (_t3 < 0.0) {                                       \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         return _tdb.d;                                       \
       } else return _t1;                                     \
    } else {                                                  \
       if (_t3 > 0.0) {                                       \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         return _tdb.d;                                       \
       } else return _t1;                                     \
    }                                                         \
}


/* ReturnRoundUpwards3Unfiltered

   Procedure for rounding a triple to a double number
   in round-upwards mode.


   Arguments:       a triple double number xh, xm, xl
                    a double constant wca representing 2^k
		    where 2^-k is Lefevre's worst case accuracy
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0
		    		    
   Guarantees:      xprime = RU(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundUpwards3Unfiltered(xh,xm,xl,wca)               \
{                                                                 \
    double _t1, _t2, _t3;                                         \
    db_number _tdb, _tdb2;                                        \
                                                                  \
    Add12(_t1,_t2,(xh),(xm));                                     \
    _t3 = _t2 + (xl);                                             \
    if (_t3 > 0.0) {                                              \
      _tdb2.d = wca * _t3;                                        \
      _tdb.d = _t1;                                               \
      if ((_tdb2.i[HI] & 0x7ff00000) < (_tdb.i[HI] & 0x7ff00000)) \
         return _t1;                                              \
      if (_t1 > 0.0) {                                            \
         _tdb.l++;                                                \
         return _tdb.d;                                           \
      } else {                                                    \
         _tdb.l--;                                                \
         return _tdb.d;                                           \
      }                                                           \
    } else return _t1;                                            \
}



/* ReturnRoundDownwards3Unfiltered

   Procedure for rounding a triple to a double number
   in round-downwards mode.


   Arguments:       a triple double number xh, xm, xl
                    a double constant wca representing 2^k
		    where 2^-k is Lefevre's worst case accuracy
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0
		    		    
   Guarantees:      xprime = RD(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundDownwards3Unfiltered(xh,xm,xl,wca)             \
{                                                                 \
    double _t1, _t2, _t3;                                         \
    db_number _tdb, _tdb2;                                        \
                                                                  \
    Add12(_t1,_t2,(xh),(xm));                                     \
    _t3 = _t2 + (xl);                                             \
    if (_t3 < 0.0) {                                              \
      _tdb2.d = wca * _t3;                                        \
      _tdb.d = _t1;                                               \
      if ((_tdb2.i[HI] & 0x7ff00000) < (_tdb.i[HI] & 0x7ff00000)) \
         return _t1;                                              \
      if (_t1 > 0.0) {                                            \
         _tdb.l--;                                                \
         return _tdb.d;                                           \
      } else {                                                    \
         _tdb.l++;                                                \
         return _tdb.d;                                           \
      }                                                           \
    } else return _t1;                                            \
}

/* ReturnRoundTowardsZero3Unfiltered

   Procedure for rounding a triple to a double number
   in round-towards-zero mode.


   Arguments:       a triple double number xh, xm, xl
                    a double constant wca representing 2^k
		    where 2^-k is Lefevre's worst case accuracy
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0
		    		    
   Guarantees:      xprime = RZ(xh + xm + xl)

   Sideeffects:     returns, i.e. leaves the function

*/
#define ReturnRoundTowardsZero3Unfiltered(xh,xm,xl,wca)       \
{                                                             \
    if ((xh) > 0)                                             \
      ReturnRoundDownwards3Unfiltered((xh),(xm),(xl),(wca))   \
    else                                                      \
      ReturnRoundUpwards3Unfiltered((xh),(xm),(xl),(wca))     \
}

/* RoundToNearest3

   Procedure for rounding a triple to a double number
   in round-to-nearest-ties-to-even mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0
		    xl = 0 iff xm != +/- 0.5 * ulp(xh) (0.25 if xh = 2^e)
		    		    
   Guarantees:      xprime = RN(xh + xm + xl)

   Details:         res is considered to be a pointer

*/
#define RoundToNearest3(res,xh,xm,xl)                         \
{                                                             \
    double _t1, _t2, _t3, _t4, _t5, _t6;                      \
    db_number _xp, _xn;                                       \
                                                              \
    _xp.d = (xh);                                             \
    _xn.i[HI] = _xp.i[HI];                                    \
    _xn.i[LO] = _xp.i[LO];                                    \
    _xn.l--;                                                  \
    _t1 = _xn.d;                                              \
    _xp.l++;                                                  \
    _t4 = _xp.d;                                              \
    _t2 = (xh) - _t1;                                         \
    _t3 = _t2 * -0.5;                                         \
    _t5 = _t4 - (xh);                                         \
    _t6 = _t5 * 0.5;                                          \
    if (((xm) != _t3) && ((xm) != _t6))                       \
      (*(res)) = ((xh) + (xm));                               \
    else {                                                    \
      if ((xm) * (xl) > 0.0) {                                \
        if ((xh) * (xl) > 0.0)                                \
          (*(res)) = _t4;                                     \
        else                                                  \
          (*(res)) = _t1;                                     \
      } else (*(res)) = (xh);                                 \
    }                                                         \
}

/* RoundUpwards3

   Procedure for rounding a triple to a double number
   in round-upwards mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RU(xh + xm + xl)

   Details:         res is considered to be a pointer

*/
#define RoundUpwards3(res,xh,xm,xl)                           \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t3 > 0.0) {                                          \
      if (_t1 > 0.0) {                                        \
         _tdb.d = _t1;                                        \
         _tdb.l++;                                            \
         (*(res)) = _tdb.d;                                   \
      } else {                                                \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         (*(res)) = _tdb.d;                                   \
      }                                                       \
    } else (*(res)) = _t1;                                    \
}


/* RoundDownwards3

   Procedure for rounding a triple to a double number
   in round-downwards mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RD(xh + xm + xl)

   Details:         res is considered to be a pointer

*/
#define RoundDownwards3(res,xh,xm,xl)                         \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t3 < 0.0) {                                          \
      if (_t1 > 0.0) {                                        \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         (*(res)) = _tdb.d;                                   \
      } else {                                                \
         _tdb.d = _t1;                                        \
         _tdb.l++;                                            \
         (*(res)) = _tdb.d;                                   \
      }                                                       \
    } else (*(res)) = _t1;                                    \
}


/* RoundTowardsZero3

   Procedure for rounding a triple to a double number
   in round-towards-zero mode.


   Arguments:       a triple double number xh, xm, xl
   
   Results:         a double number xprime 
                    returned by a return-statement

   Preconditions:   xh, xm and xl are non-overlapping
                    xm = RN(xm +math xl)
		    xh != 0, xm != 0

		    Exact algebraic images have already
		    been filtered out.
		    		    
   Guarantees:      xprime = RZ(xh + xm + xl)

   Details:         res is considered to be a pointer

*/
#define RoundTowardsZero3(res,xh,xm,xl)                       \
{                                                             \
    double _t1, _t2, _t3;                                     \
    db_number _tdb;                                           \
                                                              \
    Add12(_t1,_t2,(xh),(xm));                                 \
    _t3 = _t2 + (xl);                                         \
    if (_t1 > 0.0) {                                          \
       if (_t3 < 0.0) {                                       \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         (*(res)) = _tdb.d;                                   \
       } else (*(res)) = _t1;                                 \
    } else {                                                  \
       if (_t3 > 0.0) {                                       \
         _tdb.d = _t1;                                        \
         _tdb.l--;                                            \
         (*(res)) = _tdb.d;                                   \
       } else (*(res)) = _t1;                                 \
    }                                                         \
}

/* sqrt13

   Computes a triple-double approximation of sqrt(x)
   
   Should be provable to be exact to at least 140 bits.

   Only handles the following special cases:
   - x == 0
   - subnormal x 
   The following cases are not handled:
   - x < 0
   - x = +/-Infty, NaN

*/


#define  DoSqrt13(resh, resm, resl , x)                                                          \
{                                                                                             \
  db_number _xdb;                                                                             \
  int _E;                                                                                     \
  double _m, _r0, _r1, _r2, _r3h, _r3l, _r4h, _r4l;                                           \
  double _r5h, _r5m, _r5l, _srtmh, _srtml, _srtmm;                                            \
  double _r2PHr2h, _r2PHr2l, _r2Sqh, _r2Sql;                                                  \
  double _mMr2h, _mMr2l, _mMr2Ch, _mMr2Cl;                                                    \
  double _MHmMr2Ch, _MHmMr2Cl;                                                                \
  double _r3Sqh, _r3Sql, _mMr3Sqh, _mMr3Sql;                                                  \
  double _srtmhover,_srtmmover,_srtmlover;                                                    \
  double _HmMr4Sqm,_HmMr4Sql, _mMr4Sqhover, _mMr4Sqmover, _mMr4Sqlover;                       \
  double _mMr4Sqh, _mMr4Sqm, _mMr4Sql, _r4Sqh, _r4Sqm, _r4Sql;                                \
                                                                                              \
  /* Special case x = 0 */                                                                    \
  if ((x) == 0) {                                                                             \
    (*(resh)) = (x);                                                                          \
    (*(resm)) = 0;                                                                            \
    (*(resl)) = 0;                                                                            \
  } else {                                                                                    \
                                                                                              \
    _E = 0;                                                                                   \
                                                                                              \
    /* Convert to integer format */                                                           \
    _xdb.d = (x);                                                                             \
                                                                                              \
    /* Handle subnormal case */                                                               \
    if (_xdb.i[HI] < 0x00100000) {                                                            \
      _E = -52;                                                                               \
      _xdb.d *= ((db_number) ((double) SQRTTWO52)).d;                                         \
                        /* make x a normal number */                                          \
    }                                                                                         \
                                                                                              \
    /* Extract exponent E and mantissa m */                                                   \
    _E += (_xdb.i[HI]>>20)-1023;                                                              \
    _xdb.i[HI] = (_xdb.i[HI] & 0x000fffff) | 0x3ff00000;                                      \
    _m = _xdb.d;                                                                              \
                                                                                              \
    /* Make exponent even */                                                                  \
    if (_E & 0x00000001) {                                                                    \
      _E++;                                                                                   \
      _m *= 0.5;    /* Suppose now 1/2 <= m <= 2 */                                           \
    }                                                                                         \
                                                                                              \
    /* Construct sqrt(2^E) = 2^(E/2) */                                                       \
    _xdb.i[HI] = (_E/2 + 1023) << 20;                                                         \
    _xdb.i[LO] = 0;                                                                           \
                                                                                              \
    /* Compute initial approximation to r = 1/sqrt(m) */                                      \
                                                                                              \
    _r0 = SQRTPOLYC0 +                                                                        \
         _m * (SQRTPOLYC1 + _m * (SQRTPOLYC2 + _m * (SQRTPOLYC3 + _m * SQRTPOLYC4)));         \
                                                                                              \
    /* Iterate two times on double precision */                                               \
                                                                                              \
    _r1 = 0.5 * _r0 * (3 - _m * (_r0 * _r0));                                                 \
    _r2 = 0.5 * _r1 * (3 - _m * (_r1 * _r1));                                                 \
                                                                                              \
    /* Iterate two times on double-double precision */                                        \
                                                                                              \
    Mul12(&_r2Sqh, &_r2Sql, _r2, _r2);                                                        \
    Add12(_r2PHr2h, _r2PHr2l, _r2, (0.5 * _r2));                                              \
    Mul12(&_mMr2h, &_mMr2l, _m, _r2);                                                         \
    Mul22(&_mMr2Ch, &_mMr2Cl, _mMr2h, _mMr2l, _r2Sqh, _r2Sql);                                \
                                                                                              \
    _MHmMr2Ch = -0.5 * _mMr2Ch;                                                               \
    _MHmMr2Cl = -0.5 * _mMr2Cl;                                                               \
                                                                                              \
    Add22(&_r3h, &_r3l, _r2PHr2h, _r2PHr2l, _MHmMr2Ch, _MHmMr2Cl);                            \
                                                                                              \
    Mul22(&_r3Sqh, &_r3Sql, _r3h, _r3l, _r3h, _r3l);                                          \
    Mul22(&_mMr3Sqh, &_mMr3Sql, _m, 0.0, _r3Sqh, _r3Sql);                                       \
             /* To prove: mMr3Sqh = 1.0 in each case */                                       \
                                                                                              \
    Mul22(&_r4h, &_r4l, _r3h, _r3l, 1.0, (-0.5 * _mMr3Sql));                                    \
                                                                                              \
    /* Iterate once on triple-double precision */                                             \
                                                                                              \
    Mul23(&_r4Sqh, &_r4Sqm, &_r4Sql, _r4h, _r4l, _r4h, _r4l);                                 \
    Mul133(&_mMr4Sqhover, &_mMr4Sqmover, &_mMr4Sqlover, _m, _r4Sqh, _r4Sqm, _r4Sql);          \
    Renormalize3(&_mMr4Sqh, &_mMr4Sqm, &_mMr4Sql, _mMr4Sqhover, _mMr4Sqmover, _mMr4Sqlover);  \
    /* To prove: mMr4Sqh = 1.0 in each case */                                                \
                                                                                              \
    _HmMr4Sqm = -0.5 * _mMr4Sqm;                                                              \
    _HmMr4Sql = -0.5 * _mMr4Sql;                                                              \
                                                                                              \
    Mul233(&_r5h,&_r5m,&_r5l,_r4h,_r4l,1.0,_HmMr4Sqm,_HmMr4Sql);                                \
                                                                                              \
    /* Multiply obtained reciprocal square root by m */                                       \
                                                                                              \
    Mul133(&_srtmhover, &_srtmmover, &_srtmlover,_m,_r5h,_r5m,_r5l);                          \
                                                                                              \
    Renormalize3(&_srtmh,&_srtmm,&_srtml,_srtmhover,_srtmmover,_srtmlover);                   \
                                                                                              \
    /* Multiply componentwise by sqrt(2^E) */                                                 \
    /* which is an integer power of 2 that may not produce a subnormal */                     \
                                                                                              \
    (*(resh)) = _xdb.d * _srtmh;                                                              \
    (*(resm)) = _xdb.d * _srtmm;                                                              \
    (*(resl)) = _xdb.d * _srtml;                                                              \
                                                                                              \
  } /* End: special case 0 */                                                                 \
}


/* recpr33()

   Computes a triple-double reciprocal of a triple-double
   
   Should be provable to be exact to at least 140 bits

   No special case handling is done

   dh + dm + dl must be renormalized

   The result is renormalized

*/


#define  DoRecpr33(resh, resm, resl, dh, dm, dl)                                                 \
{                                                                                             \
    double _rec_r1, _rec_t1, _rec_t2, _rec_t3, _rec_t4, _rec_t5, _rec_t6, _rec_t7, _rec_t8, _rec_t9, _rec_t10, _rec_t11, _rec_t12, _rec_t13, _rec_t14;    \
    double _rec_r2h, _rec_r2l, _rec_t15, _rec_t16, _rec_t17, _rec_t18, _rec_t19, _rec_t20, _rec_t21, _rec_t22, _rec_t23;                  \
                                                                                              \
    _rec_r1 = 1.0 / (dh);                                                                         \
    Mul12(&_rec_t1,&_rec_t2,_rec_r1,(dh));                                                                \
    _rec_t3 = _rec_t1 - 1.0;                                                                          \
    Add12Cond(_rec_t4,_rec_t5,_rec_t3,_rec_t2);                                                               \
    Mul12(&_rec_t6,&_rec_t7,_rec_r1,(dm));                                                                \
    Add12(_rec_t8,_rec_t9,-1.0,_rec_t6);                                                                  \
    _rec_t10 = _rec_t9 + _rec_t7;                                                                         \
    Add12(_rec_t11,_rec_t12,_rec_t8,_rec_t10);                                                                \
    _rec_r1 = -_rec_r1;                                                                               \
    Add22Cond(&_rec_t13,&_rec_t14,_rec_t4,_rec_t5,_rec_t11,_rec_t12);                                                 \
    Mul122(&_rec_r2h,&_rec_r2l,_rec_r1,_rec_t13,_rec_t14);                                                        \
    Mul233(&_rec_t15,&_rec_t16,&_rec_t17,_rec_r2h,_rec_r2l,(dh),(dm),(dl));                                       \
    Renormalize3(&_rec_t18,&_rec_t19,&_rec_t20,_rec_t15,_rec_t16,_rec_t17);                                           \
    _rec_t18 = -1.0;                                                                              \
    Mul233(&_rec_t21,&_rec_t22,&_rec_t23,_rec_r2h,_rec_r2l,_rec_t18,_rec_t19,_rec_t20);                                       \
    _rec_t21 = -_rec_t21; _rec_t22 = -_rec_t22; _rec_t23 = -_rec_t23;                                                 \
    Renormalize3((resh),(resm),(resl),_rec_t21,_rec_t22,_rec_t23);                                        \
}



#endif /*TRIPLE_rec_DOUBLE_rec_H*/
