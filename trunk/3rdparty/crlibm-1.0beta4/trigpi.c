#include <stdio.h>
#include <stdlib.h>
#include "crlibm.h"
/*
 * Correctly rounded trigpi functions
 *
 * Authors : F. de Dinechin, S. Chevillard, C. Lauter (the latter two
 * didn't write a line of this file, but wrote a tool that wrote a
 * tool that wrote etc that wrote bits of code related to polynomial
 * evaluation.)
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
#include "crlibm_private.h"
#include "triple-double.h"
#include "trigpi.h"
 

/*   TODO


Use the symmetries of the tables

Actually, use the tables from the standard trigo.

Write decent quick steps. Or hammer Christoph and Sylvain to do so.

 */





/* This ugly bits of code in the beginning are polynomial evaluations
   automagically generated and proven by Chevillard and Lauter's
   tools 
*/


#define sinpiacc_coeff_1h 3.14159265358979311599796346854418516159057617187500000000000000000000000000000000e+00
#define sinpiacc_coeff_1m 1.22464679914735320717376402945839660462569212467758006379625612680683843791484833e-16
#define sinpiacc_coeff_1l -2.87889731599645993207191707893463395148177292198731390393739579574603302514349608e-33
#define sinpiacc_coeff_3h -5.16771278004997025590228076907806098461151123046875000000000000000000000000000000e+00
#define sinpiacc_coeff_3m 2.26656228257550136196266687046492287115561324595258696490418515168130397796630859e-16
#define sinpiacc_coeff_5h 2.55016403987734552316624103696085512638092041015625000000000000000000000000000000e+00
#define sinpiacc_coeff_5m -7.93098961936403945684716222915171282926664203267314023904077657789457589387893677e-17
#define sinpiacc_coeff_7h -5.99264529320792105338000510528218001127243041992187500000000000000000000000000000e-01
#define sinpiacc_coeff_9h 8.21458866130424236740026344705256633460521697998046875000000000000000000000000000e-02
#define sinpiacc_coeff_11h -7.37046804820839888960914976223648409359157085418701171875000000000000000000000000e-03


#define cospiacc_coeff_0h 1.00000000000000000000000000000000000000000000000000000000000000000000000000000000e+00
#define cospiacc_coeff_2h -4.93480220054467899615247006295248866081237792968750000000000000000000000000000000e+00
#define cospiacc_coeff_2m -3.13264775437072047133490817894057799839785556899468543790021612949203699827194214e-16
#define cospiacc_coeff_4h 4.05871212641676848420502210501581430435180664062500000000000000000000000000000000e+00
#define cospiacc_coeff_4m -2.66019969731660223662555032718185048048635055542576743903282476821914315223693848e-16
#define cospiacc_coeff_6h -1.33526276885458949905682857206556946039199829101562500000000000000000000000000000e+00
#define cospiacc_coeff_8h 2.35330630358513925859398341344785876572132110595703125000000000000000000000000000e-01
#define cospiacc_coeff_10h -2.58068327360992909313974763563237502239644527435302734375000000000000000000000000e-02

static void sincospiacc(double *sinpiacc_resh, double *sinpiacc_resm, double *sinpiacc_resl,
		 double *cospiacc_resh, double *cospiacc_resm, double *cospiacc_resl,
		 double x) {
  double x2h, x2m;
  double sinpiacc_t_1_0h;
  double sinpiacc_t_2_0h;
  double sinpiacc_t_3_0h;
  double sinpiacc_t_4_0h;
  double sinpiacc_t_5_0h, sinpiacc_t_5_0m;
  double sinpiacc_t_6_0h, sinpiacc_t_6_0m;
  double sinpiacc_t_7_0h, sinpiacc_t_7_0m;
  double sinpiacc_t_8_0h, sinpiacc_t_8_0m;
  double sinpiacc_t_9_0h, sinpiacc_t_9_0m, sinpiacc_t_9_0l;
  double sinpiacc_t_10_0h, sinpiacc_t_10_0m, sinpiacc_t_10_0l;
 
  double cospiacc_t_1_0h;
  double cospiacc_t_2_0h;
  double cospiacc_t_3_0h;
  double cospiacc_t_4_0h;
  double cospiacc_t_5_0h, cospiacc_t_5_0m;
  double cospiacc_t_6_0h, cospiacc_t_6_0m;
  double cospiacc_t_7_0h, cospiacc_t_7_0m;
  double cospiacc_t_8_0h, cospiacc_t_8_0m;
  double cospiacc_t_9_0h, cospiacc_t_9_0m, cospiacc_t_9_0l;

  Mul12(&x2h,&x2m,x,x);  

  sinpiacc_t_1_0h = sinpiacc_coeff_11h;
  sinpiacc_t_2_0h = sinpiacc_t_1_0h * x2h;
  sinpiacc_t_3_0h = sinpiacc_coeff_9h + sinpiacc_t_2_0h;
  sinpiacc_t_4_0h = sinpiacc_t_3_0h * x2h;
  Add12(sinpiacc_t_5_0h,sinpiacc_t_5_0m,sinpiacc_coeff_7h,sinpiacc_t_4_0h);
  MulAdd22(&sinpiacc_t_6_0h,&sinpiacc_t_6_0m,sinpiacc_coeff_5h,sinpiacc_coeff_5m,x2h,x2m,sinpiacc_t_5_0h,sinpiacc_t_5_0m);
  MulAdd22(&sinpiacc_t_7_0h,&sinpiacc_t_7_0m,sinpiacc_coeff_3h,sinpiacc_coeff_3m,x2h,x2m,sinpiacc_t_6_0h,sinpiacc_t_6_0m);
  Mul22(&sinpiacc_t_8_0h,&sinpiacc_t_8_0m,sinpiacc_t_7_0h,sinpiacc_t_7_0m,x2h,x2m);
  Add233Cond(&sinpiacc_t_9_0h,&sinpiacc_t_9_0m,&sinpiacc_t_9_0l,sinpiacc_t_8_0h,sinpiacc_t_8_0m,sinpiacc_coeff_1h,sinpiacc_coeff_1m,sinpiacc_coeff_1l);
  Mul133(&sinpiacc_t_10_0h,&sinpiacc_t_10_0m,&sinpiacc_t_10_0l,x,sinpiacc_t_9_0h,sinpiacc_t_9_0m,sinpiacc_t_9_0l);
  Renormalize3(sinpiacc_resh,sinpiacc_resm,sinpiacc_resl,sinpiacc_t_10_0h,sinpiacc_t_10_0m,sinpiacc_t_10_0l);

  
  cospiacc_t_1_0h = cospiacc_coeff_10h;
  cospiacc_t_2_0h = cospiacc_t_1_0h * x2h;
  cospiacc_t_3_0h = cospiacc_coeff_8h + cospiacc_t_2_0h;
  cospiacc_t_4_0h = cospiacc_t_3_0h * x2h;
  Add12(cospiacc_t_5_0h,cospiacc_t_5_0m,cospiacc_coeff_6h,cospiacc_t_4_0h);
  MulAdd22(&cospiacc_t_6_0h,&cospiacc_t_6_0m,cospiacc_coeff_4h,cospiacc_coeff_4m,x2h,x2m,cospiacc_t_5_0h,cospiacc_t_5_0m);
  MulAdd22(&cospiacc_t_7_0h,&cospiacc_t_7_0m,cospiacc_coeff_2h,cospiacc_coeff_2m,x2h,x2m,cospiacc_t_6_0h,cospiacc_t_6_0m);
  Mul22(&cospiacc_t_8_0h,&cospiacc_t_8_0m,cospiacc_t_7_0h,cospiacc_t_7_0m,x2h,x2m);
  Add123(&cospiacc_t_9_0h,&cospiacc_t_9_0m,&cospiacc_t_9_0l,cospiacc_coeff_0h,cospiacc_t_8_0h,cospiacc_t_8_0m);
  *cospiacc_resh = cospiacc_t_9_0h; *cospiacc_resm = cospiacc_t_9_0m; *cospiacc_resl = cospiacc_t_9_0l;

}











/* Comment on comparing sa, ca, sy and cy 
   either index=0, then sa=0 and ca=1, therefore t2=0, and the Add33 will be exact 
   or index !=0, and       
	-eps1 < sy < eps1  (but sy may be negative)
	sa > eps1 (sa>0)
	1-eps2 < cy < 1
	ca < 1-eps2
	therefore 
	sacy = t2 >0
	casy = t1 may be negative
	abs(t1) <=  abs(t2)
	Unfortunately we need a stronger condition to use the Add33 
*/




static void sinpi_accurate(double *rh, double *rm, double *rl,
			   double y, int index, int quadrant)
{
   double syh, sym, syl, cyh, cym, cyl, sah, sam, sal, cah, cam, cal;
   double t1h, t1m, t1l, t2h, t2m, t2l;

   sincospiacc(&syh, &sym, &syl, &cyh, &cym, &cyl, y);
   
   sah=sincosTable[index].sh;
   cah=sincosTable[index].ch;
   sam=sincosTable[index].sm;
   cam=sincosTable[index].cm;
   sal=sincosTable[index].sl;
   cal=sincosTable[index].cl;

   if(quadrant==0 || quadrant==2) {
     /* compute sy*ca+sa*cy   :    t1 = sy*ca,     t2 =  sa*cy*/
     Mul33(&t1h,&t1m,&t1l, syh,sym,syl, cah,cam,cal);
     Mul33(&t2h,&t2m,&t2l, sah,sam,sal, cyh,cym,cyl);
     Add33Cond(rh, rm, rl, t2h,t2m,t2l, t1h,t1m,t1l);
   }
   else {
     /* compute cy*ca - sa*sy : t1 = cy*ca,    t2 =  sa*sy */
     Mul33(&t1h,&t1m,&t1l, cyh,cym,cyl, cah,cam,cal);
     Mul33(&t2h,&t2m,&t2l, sah,sam,sal, syh,sym,syl);     
     Add33Cond(rh, rm, rl, t1h,t1m,t1l, -t2h,-t2m,-t2l);
   }

   if (quadrant>=2) {
     *rh = -*rh;
     *rm = -*rm;
     *rl = -*rl;
   }

};






#define sinpiquick_coeff_1h 3.14159265358979311599796346854418516159057617187500000000000000000000000000000000e+00
#define sinpiquick_coeff_1m 1.22464971683184787123862072310058851157814368464452070561776508839102461934089661e-16
#define sinpiquick_coeff_3h -5.16771278004997025590228076907806098461151123046875000000000000000000000000000000e+00
#define sinpiquick_coeff_5h 2.55016403989992213041659852024167776107788085937500000000000000000000000000000000e+00
#define sinpiquick_coeff_7h -5.99263913290728922333983064163476228713989257812500000000000000000000000000000000e-01
#define cospiquick_coeff_0h 1.00000000000000000000000000000000000000000000000000000000000000000000000000000000e+00
#define cospiquick_coeff_2h -4.93480220054467899615247006295248866081237792968750000000000000000000000000000000e+00
#define cospiquick_coeff_4h 4.05871212632582167856298838160000741481781005859375000000000000000000000000000000e+00
#define cospiquick_coeff_6h -1.33525456323720947970912220625905320048332214355468750000000000000000000000000000e+00



static void cospi_accurate(double *rh, double *rm, double *rl,
			   double y, int index, int quadrant)
{
   double syh, sym, syl, cyh, cym, cyl, sah, sam, sal, cah, cam, cal;
   double t1h, t1m, t1l, t2h, t2m, t2l;

   sincospiacc(&syh, &sym, &syl, &cyh, &cym, &cyl, y);
   
   sah=sincosTable[index].sh;
   cah=sincosTable[index].ch;
   sam=sincosTable[index].sm;
   cam=sincosTable[index].cm;
   sal=sincosTable[index].sl;
   cal=sincosTable[index].cl;

   if(quadrant==0 || quadrant==2) {
     /* compute cy*ca - sa*sy : t1 = cy*ca,    t2 =  sa*sy */
     Mul33(&t1h,&t1m,&t1l, cyh,cym,cyl, cah,cam,cal);
     Mul33(&t2h,&t2m,&t2l, sah,sam,sal, syh,sym,syl);     
     Add33Cond(rh, rm, rl, t1h,t1m,t1l, -t2h,-t2m,-t2l);
   }
   else {
     /* compute sy*ca+sa*cy   :    t1 = sy*ca,     t2 =  sa*cy*/
     Mul33(&t1h,&t1m,&t1l, syh,sym,syl, cah,cam,cal);
     Mul33(&t2h,&t2m,&t2l, sah,sam,sal, cyh,cym,cyl);
     Add33Cond(rh, rm, rl, t2h,t2m,t2l, t1h,t1m,t1l);
   }

   if (quadrant==1 || quadrant==2) {
     *rh = -*rh;
     *rm = -*rm;
     *rl = -*rl;
   }

};






/* This one can clearly be improved. It was set up in less than one hour */
void sinpiquick(double *rh, double *rm, double x, int index, int quadrant) {
  double x2h, x2m;
  double sinpiquick_t_1_0h;
  double sinpiquick_t_2_0h;
  double sinpiquick_t_3_0h;
  double sinpiquick_t_4_0h;
  double sinpiquick_t_5_0h, sinpiquick_t_5_0m;
  double sinpiquick_t_6_0h, sinpiquick_t_6_0m;
  double sinpiquick_t_7_0h, sinpiquick_t_7_0m;
  double syh, sym;
  double cospiquick_t_1_0h;
  double cospiquick_t_2_0h;
  double cospiquick_t_3_0h;
  double cospiquick_t_4_0h;
  double cospiquick_t_5_0h, cospiquick_t_5_0m;
  double cospiquick_t_6_0h, cospiquick_t_6_0m;
  double cospiquick_t_7_0h, cospiquick_t_7_0m;
  double cyh, cym;
  double t1h, t1m, t2h, t2m, sah, sam, cah,cam;

  Mul12(&x2h,&x2m,x,x);
  
  sah=sincosTable[index].sh;
  cah=sincosTable[index].ch;
  sam=sincosTable[index].sm;
  cam=sincosTable[index].cm;

  sinpiquick_t_1_0h = sinpiquick_coeff_7h;
  sinpiquick_t_2_0h = sinpiquick_t_1_0h * x2h;
  sinpiquick_t_3_0h = sinpiquick_coeff_5h + sinpiquick_t_2_0h;
  sinpiquick_t_4_0h = sinpiquick_t_3_0h * x2h;
  Add12(sinpiquick_t_5_0h,sinpiquick_t_5_0m,sinpiquick_coeff_3h,sinpiquick_t_4_0h);
  MulAdd22(&sinpiquick_t_6_0h,&sinpiquick_t_6_0m,sinpiquick_coeff_1h,sinpiquick_coeff_1m,x2h,x2m,sinpiquick_t_5_0h,sinpiquick_t_5_0m);
  Mul122(&sinpiquick_t_7_0h,&sinpiquick_t_7_0m,x,sinpiquick_t_6_0h,sinpiquick_t_6_0m);
  syh = sinpiquick_t_7_0h; sym = sinpiquick_t_7_0m;

  cospiquick_t_1_0h = cospiquick_coeff_6h;
  cospiquick_t_2_0h = cospiquick_t_1_0h * x2h;
  cospiquick_t_3_0h = cospiquick_coeff_4h + cospiquick_t_2_0h;
  cospiquick_t_4_0h = cospiquick_t_3_0h * x2h;
  Add12(cospiquick_t_5_0h,cospiquick_t_5_0m,cospiquick_coeff_2h,cospiquick_t_4_0h);
  Mul22(&cospiquick_t_6_0h,&cospiquick_t_6_0m,cospiquick_t_5_0h,cospiquick_t_5_0m,x2h,x2m);
  Add122(&cospiquick_t_7_0h,&cospiquick_t_7_0m,cospiquick_coeff_0h,cospiquick_t_6_0h,cospiquick_t_6_0m);
  cyh = cospiquick_t_7_0h; cym = cospiquick_t_7_0m;

  /* Here comes the hand-written, unproven yet code */
   if(quadrant==0 || quadrant==2) {
     /* compute sy*ca+sa*cy   :    t1 = sy*ca,     t2 =  sa*cy*/
     Mul22(&t1h,&t1m, syh,sym, cah,cam);
     Mul22(&t2h,&t2m, sah,sam, cyh,cym);
     Add22Cond(rh, rm, t2h,t2m, t1h,t1m);
   }
   else {
     /* compute cy*ca - sa*sy : t1 = cy*ca,    t2 =  sa*sy */
     Mul22(&t1h,&t1m, cyh,cym, cah,cam);
     Mul22(&t2h,&t2m, sah,sam, syh,sym);     
     Add22Cond(rh, rm, t1h,t1m, -t2h,-t2m);
   }

   if (quadrant>=2) {
     *rh = -*rh;
     *rm = -*rm;
   }

}







 double sinpi_rn(double x){
   double xs, y,u, rh, rm, rl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

   if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     if(rh == (rh + (rl * PIX_RNCST_SIN)))
       return rh;
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   sinpiquick(&rh, &rm,  y, index, quadrant);
   if (rh==rh+1.00001*rm) /* See trigpiquick.gappa. This first step is ridiculously too accurate */
     return rh;
   sinpi_accurate(&rh, &rm, &rl, y, index, quadrant);
   ReturnRoundToNearest3(rh,rm,rl);   
 }

 







 double sinpi_rd(double x){
   double xs, y,u, rh, rm, rl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return -0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

    if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_minf(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RD(rh,rl,PIX_EPS_SIN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   sinpi_accurate(&rh, &rm, &rl, y, index, quadrant);

   ReturnRoundDownwards3(rh,rm,rl);   
}; 



 double sinpi_ru(double x){
   double xs, y,u, rh, rm, rl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return +0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

    if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_pinf(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RU(rh,rl,PIX_EPS_SIN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   sinpi_accurate(&rh, &rm, &rl, y, index, quadrant);

   ReturnRoundUpwards3(rh,rm,rl);   
   
};  




 double sinpi_rz(double x){
   double xs, y,u, rh, rm, rl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

    if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_zero(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RZ(rh,rl,PIX_EPS_SIN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   sinpi_accurate(&rh, &rm, &rl, y, index, quadrant);

   ReturnRoundTowardsZero3(rh,rm,rl);   
};









 /* to nearest  */
 double cospi_rn(double x){
   double xs, y,u, rh, rm, rl, absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx =-x; else absx=x; 

   xdb.d=x;
   xs = x*128.0;

   /* argument reduction. 
      We do it before the special case tests for performance, 
      it might compute garbage for inf, very large inputs, etc */
   if(absx>  TWOTO42 ) {  /* 2^42, x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part, in which was the coma since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   y = y * INV128;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;

   /* SPECIAL CASES: x=(Nan, Inf) cos(pi*x)=Nan */

   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }

   if(absxih>=0x43400000) /* 2^53, which entails that x is an even integer */
     return 1.0; 

   if(index==0 && y==0. && ((quadrant&1)==1)) return +0.; 
   /* Always +0, inpired by LIA2; We do not have cos(x+pi) == - cos(x)
      in this case */

   if(index==0 && y==0. && quadrant==0) return 1.; 
   if(index==0 && y==0. && quadrant==2) return -1.; 

   if (absxih<0x3E26A09E) /* sqrt(2^-53)/4 */
     return 1.0;
   /* printf("\n\nint part = %f    frac part = %f     index=%d   quadrant=%d   \n", u, y, index, quadrant);
    */

   cospi_accurate(&rh, &rm, &rl, y, index, quadrant);
   ReturnRoundToNearest3(rh,rm,rl);   
};





 double cospi_rd(double x){
   double xs, y,u, rh, rm, rl, absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx =-x; else absx=x; 

   xdb.d=x;
   xs = x*128.0;


   /* argument reduction. 
      We do it before the special case tests for performance, 
      it might compute garbage for inf, very large inputs, etc */
   if(absx>  TWOTO42 ) {  /* 2^42, x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part, in which was the coma since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   y = y * INV128;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;


   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;

   /* SPECIAL CASES: x=(Nan, Inf) cos(pi*x)=Nan */

   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43400000) /* 2^53, which entails that x is an even integer */
     return 1.0; /*signed */

   if(index==0 && y==0. && ((quadrant&1)==1)) return -0.; 

   if(index==0 && y==0. && quadrant==0) return 1.; 
   if(index==0 && y==0. && quadrant==2) return -1.; 

   if (absxih<0x3E200000) /* 2^-29 */
     return 0.9999999999999998889776975374843459576368331909179687500; /* 1-2^-53 */
   /* Always +0, inpired by LIA2; We do not have cos(x+pi) == - cos(x)
      in this case */

   cospi_accurate(&rh, &rm, &rl, y, index, quadrant);
   ReturnRoundDownwards3(rh,rm,rl);  
 }; 



 
 double cospi_ru(double x){
   double xs, y,u, rh, rm, rl, absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx =-x; else absx=x; 

   xdb.d=x;
   xs = x*128.0;


   /* argument reduction. 
      We do it before the special case tests for performance, 
      it might compute garbage for inf, very large inputs, etc */
   if(absx>  TWOTO42 ) {  /* 2^42, x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part, in which was the coma since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   y = y * INV128;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;


   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;

   /* SPECIAL CASES: x=(Nan, Inf) cos(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43400000) /* 2^53, which entails that x is an even integer */
     return 1.0; /*signed */

   if(index==0 && y==0. && quadrant==0) return 1.; 
   if(index==0 && y==0. && quadrant==2) return -1.; 

   if(index==0 && y==0. && ((quadrant&1)==1)) return +0.; 
   /* Always +0, inpired by LIA2; We do not have cos(x+pi) == - cos(x)
      in this case */

   if (absxih<0x3E200000) /* 2^-29 */
     return 1;

   cospi_accurate(&rh, &rm, &rl, y, index, quadrant);
   ReturnRoundUpwards3(rh,rm,rl);  
}; 




double cospi_rz(double x){
   double xs, y,u, rh, rm, rl, absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx =-x; else absx=x; 

   xdb.d=x;
   xs = x*128.0;


   /* argument reduction. 
      We do it before the special case tests for performance, 
      it might compute garbage for inf, very large inputs, etc */
   if(absx>  TWOTO42 ) {  /* 2^42, x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part, in which was the coma since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   y = y * INV128;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;


   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;

   /* SPECIAL CASES: x=(Nan, Inf) cos(pi*x)=Nan */

   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43400000) /* 2^53, which entails that x is an even integer */
     return 1.0; /*signed */

   if(index==0 && y==0. && ((quadrant&1)==1)) return +0.; 
   /* Always +0, inpired by LIA2; We do not have cos(x+pi) == - cos(x)
      in this case */

   if(index==0 && y==0. && quadrant==0) return 1.; 
   if(index==0 && y==0. && quadrant==2) return -1.; 

   if (absxih<0x3E200000) /* 2^-29 */
     return 0.9999999999999998889776975374843459576368331909179687500; /* 1-2^-53 */

   cospi_accurate(&rh, &rm, &rl, y, index, quadrant);
   ReturnRoundTowardsZero3(rh,rm,rl);
  }; 






/*  tangent of pi times x */
 double tanpi_rn(double x){
   double xs, y,u, rh, rm, rl, ch,cm,cl, ich,icm,icl, sh,sm,sl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */
   /* TODO ? No test for Pi/4. Such a value will lauch the accurate phase. */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

   if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     if(rh == (rh + (rl * PIX_RNCST_TAN)))
       return rh;
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   cospi_accurate(&ch, &cm, &cl, y, index, quadrant);
   Recpr33(&ich, &icm, &icl, ch, cm, cl);
   sinpi_accurate(&sh, &sm, &sl, y, index, quadrant);
   Mul33(&rh,&rm,&rl, sh,sm,sl, ich,icm,icl);
   ReturnRoundToNearest3(rh,rm,rl);   

}; 




 double tanpi_rd(double x){
   double xs, y,u, rh, rm, rl, ch,cm,cl, ich,icm,icl, sh,sm,sl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

   if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_minf(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RD(rh,rl,PIX_EPS_SIN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   cospi_accurate(&ch, &cm, &cl, y, index, quadrant);
   Recpr33(&ich, &icm, &icl, ch, cm, cl);
   sinpi_accurate(&sh, &sm, &sl, y, index, quadrant);
   Mul33(&rh,&rm,&rl, sh,sm,sl, ich,icm,icl);
   ReturnRoundDownwards3(rh,rm,rl);   
};


 







 double tanpi_ru(double x){
   double xs, y,u, rh, rm, rl, ch,cm,cl, ich,icm,icl, sh,sm,sl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

   if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_pinf(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RU(rh,rl,PIX_EPS_TAN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   cospi_accurate(&ch, &cm, &cl, y, index, quadrant);
   Recpr33(&ich, &icm, &icl, ch, cm, cl);
   sinpi_accurate(&sh, &sm, &sl, y, index, quadrant);
   Mul33(&rh,&rm,&rl, sh,sm,sl, ich,icm,icl);
   ReturnRoundUpwards3(rh,rm,rl);   
};



 double tanpi_rz(double x){
   double xs, y,u, rh, rm, rl, ch,cm,cl, ich,icm,icl, sh,sm,sl, sign,absx;
   db_number xdb, t;
   int32_t xih, absxih, index, quadrant;
   
   if (x<0) absx = -x;   else absx = x; 

   xdb.d = x;

   xs = x*128.0;

   /* argument reduction */
   if(absx>  TWOTO42 ) {  /* x is very large, let us first subtract a large integer from it */
     t.d = xs;
     t.i[LO] =0; /* remove the low part. The point is somewhere there since x > 2^42. 
		    So what remains in t is an FP integer almost as large as x */
     xs = xs-t.d; /* we are going to throw away the int part anyway */ 
   }

   t.d = TWOTO5251 + xs;
   u = t.d - TWOTO5251;
   y = xs - u;
   index = t.i[LO] & 0x3f;
   quadrant = (t.i[LO] & 0xff) >>6;

   /* Special case tests come late because the conversion FP to int is slow */
   xih = xdb.i[HI];
   absxih = xih & 0x7fffffff;
   if (xih>>31)  sign=-1.;   else sign=1.; /* consider the sign bit */

   if(index==0 && y==0.0 && ((quadrant&1)==0)) return sign*0.0; /*signed, inspired by LIA-2 */

   y = y * INV128;

   /* SPECIAL CASES: x=(Nan, Inf) sin(pi*x)=Nan */
   if (absxih>=0x7ff00000) {
     xdb.l=0xfff8000000000000LL;
     return xdb.d - xdb.d; 
   }
      
   if(absxih>=0x43300000) /* 2^52, which entails that x is an integer */
     return sign*0.0; /*signed */

   if(absxih<=0x3E000000) /*2^{-31}*/ {
     if (absxih<0x01700000) { /* 2^{-1000} :  Get rid of possible subnormals  */
       /* in this case, SCS computation, accurate to 2^-210 which is provably enough */
       scs_t result;
       scs_set_d(result, x );
       scs_mul(result, PiSCS_ptr, result);
       scs_get_d_zero(&rh, result);
       return rh;
     }
     /* First step for Pi*x. TODO: FMA-based optimisation */
     const double DekkerConst  = 134217729.; /* 2^27 +1 */   
     double tt, xh, xl;                           
     /* Splitting of x. Both xh and xl have at least 26 consecutive LSB zeroes */
     tt = x*DekkerConst;     
     xh = (x-tt)+tt;
     xl = x-xh;   
     Add12(rh,rl, xh*PIHH, (xl*PIHH + xh*PIHM) + (xh*PIM + xl*PIHM) );               
     TEST_AND_RETURN_RZ(rh,rl,PIX_EPS_SIN);
   }
   /* Fall here either if we have a large input, or if we have a small
      input and the rounding test fails.  */
   cospi_accurate(&ch, &cm, &cl, y, index, quadrant);
   Recpr33(&ich, &icm, &icl, ch, cm, cl);
   sinpi_accurate(&sh, &sm, &sl, y, index, quadrant);
   Mul33(&rh,&rm,&rl, sh,sm,sl, ich,icm,icl);
   ReturnRoundTowardsZero3(rh,rm,rl);   
}; 


