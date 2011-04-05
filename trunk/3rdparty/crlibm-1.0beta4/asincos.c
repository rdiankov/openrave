/*
 * Correctly rounded arcsine and arccosine
 *
 * Copyright (c) 2007 Christoph Lauter (ENS Lyon), 
 *                    with Sylvain Chevillard (ENS Lyon) for the polynomials
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
#include "asincos.h"

static inline void p0_quick(double *p_resh, double *p_resm, double x, int32_t xhi) {
double p_x_0_pow2h, p_x_0_pow2m;





double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h;
double p_t_8_0h;
double p_t_9_0h;
double p_t_10_0h;
double p_t_11_0h;
double p_t_12_0h;
double p_t_13_0h;
double p_t_14_0h;
double p_t_15_0h;
double p_t_16_0h;
double p_t_17_0h, p_t_17_0m;
double p_t_18_0h, p_t_18_0m;
double p_t_19_0h, p_t_19_0m;
double p_t_20_0h, p_t_20_0m;

 if (xhi < EXTRABOUND2) {
   
   double t1, t2, t3;

   t2 = p0_quick_coeff_3h * x;
   t1 = x * x;
   t3 = t1 * t2;
   
   Add12(*p_resh,*p_resm,x,t3);
   

   return;
 } 

Mul12(&p_x_0_pow2h,&p_x_0_pow2m,x,x);

p_t_15_0h = p0_quick_coeff_5h;
if (xhi > EXTRABOUND) {

p_t_1_0h = p0_quick_coeff_19h;
p_t_2_0h = p_t_1_0h * p_x_0_pow2h;
p_t_3_0h = p0_quick_coeff_17h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * p_x_0_pow2h;
p_t_5_0h = p0_quick_coeff_15h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * p_x_0_pow2h;
p_t_7_0h = p0_quick_coeff_13h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * p_x_0_pow2h;
p_t_9_0h = p0_quick_coeff_11h + p_t_8_0h;
p_t_10_0h = p_t_9_0h * p_x_0_pow2h;
p_t_11_0h = p0_quick_coeff_9h + p_t_10_0h;
p_t_12_0h = p_t_11_0h * p_x_0_pow2h;
p_t_13_0h = p0_quick_coeff_7h + p_t_12_0h;
p_t_14_0h = p_t_13_0h * p_x_0_pow2h;
p_t_15_0h = p_t_15_0h + p_t_14_0h;
}

p_t_16_0h = p_t_15_0h * p_x_0_pow2h;
Add12(p_t_17_0h,p_t_17_0m,p0_quick_coeff_3h,p_t_16_0h);

 Mul122(&p_t_18_0h,&p_t_18_0m,x,p_x_0_pow2h,p_x_0_pow2m);
 Mul22(&p_t_19_0h,&p_t_19_0m,p_t_17_0h,p_t_17_0m,p_t_18_0h,p_t_18_0m);

 Add122(&p_t_20_0h,&p_t_20_0m,x,p_t_19_0h,p_t_19_0m);

*p_resh = p_t_20_0h; *p_resm = p_t_20_0m;


}

static inline void p_quick(double *p_resh, double *p_resm, double x, int index) {
  

double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h;
double p_t_8_0h;
double p_t_9_0h;
double p_t_10_0h;
double p_t_11_0h;
double p_t_12_0h;
double p_t_13_0h;
double p_t_14_0h;
double p_t_15_0h;
double p_t_16_0h;
double p_t_17_0h;
double p_t_18_0h;
double p_t_19_0h;
double p_t_20_0h;
double p_t_21_0h, p_t_21_0m;
double p_t_22_0h, p_t_22_0m;
double p_t_23_0h, p_t_23_0m;
 
p_t_1_0h = p_quick_coeff_12h;
p_t_2_0h = p_t_1_0h * x;
p_t_3_0h = p_quick_coeff_11h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * x;
p_t_5_0h = p_quick_coeff_10h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * x;
p_t_7_0h = p_quick_coeff_9h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * x;
p_t_9_0h = p_quick_coeff_8h + p_t_8_0h;
p_t_10_0h = p_t_9_0h * x;
p_t_11_0h = p_quick_coeff_7h + p_t_10_0h;
p_t_12_0h = p_t_11_0h * x;
p_t_13_0h = p_quick_coeff_6h + p_t_12_0h;
p_t_14_0h = p_t_13_0h * x;
p_t_15_0h = p_quick_coeff_5h + p_t_14_0h;
p_t_16_0h = p_t_15_0h * x;
p_t_17_0h = p_quick_coeff_4h + p_t_16_0h;
p_t_18_0h = p_t_17_0h * x;
p_t_19_0h = p_quick_coeff_3h + p_t_18_0h;
p_t_20_0h = p_t_19_0h * x;
Add12(p_t_21_0h,p_t_21_0m,p_quick_coeff_2h,p_t_20_0h);
MulAdd212(&p_t_22_0h,&p_t_22_0m,p_quick_coeff_1h,p_quick_coeff_1m,x,p_t_21_0h,p_t_21_0m);
MulAdd212(&p_t_23_0h,&p_t_23_0m,p_quick_coeff_0h,p_quick_coeff_0m,x,p_t_22_0h,p_t_22_0m);
*p_resh = p_t_23_0h; *p_resm = p_t_23_0m;

}



static inline void p9_quick(double *p_resh, double *p_resm, double x) {




double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h;
double p_t_8_0h;
double p_t_9_0h;
double p_t_10_0h;
double p_t_11_0h;
double p_t_12_0h;
double p_t_13_0h;
double p_t_14_0h;
double p_t_15_0h;
double p_t_16_0h;
double p_t_17_0h;
double p_t_18_0h;
double p_t_19_0h;
double p_t_20_0h;
double p_t_21_0h, p_t_21_0m;
double p_t_22_0h, p_t_22_0m;
double p_t_23_0h, p_t_23_0m;
 


p_t_1_0h = p9_quick_coeff_11h;
p_t_2_0h = p_t_1_0h * x;
p_t_3_0h = p9_quick_coeff_10h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * x;
p_t_5_0h = p9_quick_coeff_9h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * x;
p_t_7_0h = p9_quick_coeff_8h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * x;
p_t_9_0h = p9_quick_coeff_7h + p_t_8_0h;
p_t_10_0h = p_t_9_0h * x;
p_t_11_0h = p9_quick_coeff_6h + p_t_10_0h;
p_t_12_0h = p_t_11_0h * x;
p_t_13_0h = p9_quick_coeff_5h + p_t_12_0h;
p_t_14_0h = p_t_13_0h * x;
p_t_15_0h = p9_quick_coeff_4h + p_t_14_0h;
p_t_16_0h = p_t_15_0h * x;
p_t_17_0h = p9_quick_coeff_3h + p_t_16_0h;
p_t_18_0h = p_t_17_0h * x;
p_t_19_0h = p9_quick_coeff_2h + p_t_18_0h;
p_t_20_0h = p_t_19_0h * x;
Add12(p_t_21_0h,p_t_21_0m,p9_quick_coeff_1h,p_t_20_0h);
Mul122(&p_t_22_0h,&p_t_22_0m,x,p_t_21_0h,p_t_21_0m);
Add122(&p_t_23_0h,&p_t_23_0m,p9_quick_coeff_0h,p_t_22_0h,p_t_22_0m);
*p_resh = p_t_23_0h; *p_resm = p_t_23_0m;


}



static void p0_accu(double *p_resh, double *p_resm, double *p_resl, double x) {
double p_x_0_pow2h, p_x_0_pow2m;


Mul12(&p_x_0_pow2h,&p_x_0_pow2m,x,x);


double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h, p_t_7_0m;
double p_t_8_0h, p_t_8_0m;
double p_t_9_0h, p_t_9_0m;
double p_t_10_0h, p_t_10_0m;
double p_t_11_0h, p_t_11_0m;
double p_t_12_0h, p_t_12_0m;
double p_t_13_0h, p_t_13_0m;
double p_t_14_0h, p_t_14_0m;
double p_t_15_0h, p_t_15_0m;
double p_t_16_0h, p_t_16_0m;
double p_t_17_0h, p_t_17_0m;
double p_t_18_0h, p_t_18_0m;
double p_t_19_0h, p_t_19_0m;
double p_t_20_0h, p_t_20_0m;
double p_t_21_0h, p_t_21_0m;
double p_t_22_0h, p_t_22_0m;
double p_t_23_0h, p_t_23_0m;
double p_t_24_0h, p_t_24_0m;
double p_t_25_0h, p_t_25_0m;
double p_t_26_0h, p_t_26_0m;
double p_t_27_0h, p_t_27_0m;
double p_t_28_0h, p_t_28_0m, p_t_28_0l;
double p_t_29_0h, p_t_29_0m, p_t_29_0l;
double p_t_30_0h, p_t_30_0m, p_t_30_0l;
double p_t_31_0h, p_t_31_0m, p_t_31_0l;
double p_t_32_0h, p_t_32_0m, p_t_32_0l;
double p_t_33_0h, p_t_33_0m, p_t_33_0l;
double p_t_34_0h, p_t_34_0m, p_t_34_0l;
double p_t_35_0h, p_t_35_0m, p_t_35_0l;
 


p_t_1_0h = p0_accu_coeff_37h;
p_t_2_0h = p_t_1_0h * p_x_0_pow2h;
p_t_3_0h = p0_accu_coeff_35h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * p_x_0_pow2h;
p_t_5_0h = p0_accu_coeff_33h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * p_x_0_pow2h;
Add12(p_t_7_0h,p_t_7_0m,p0_accu_coeff_31h,p_t_6_0h);
Mul22(&p_t_8_0h,&p_t_8_0m,p_t_7_0h,p_t_7_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_9_0h,&p_t_9_0m,p0_accu_coeff_29h,p_t_8_0h,p_t_8_0m);
Mul22(&p_t_10_0h,&p_t_10_0m,p_t_9_0h,p_t_9_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_11_0h,&p_t_11_0m,p0_accu_coeff_27h,p_t_10_0h,p_t_10_0m);
Mul22(&p_t_12_0h,&p_t_12_0m,p_t_11_0h,p_t_11_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_13_0h,&p_t_13_0m,p0_accu_coeff_25h,p_t_12_0h,p_t_12_0m);
Mul22(&p_t_14_0h,&p_t_14_0m,p_t_13_0h,p_t_13_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_15_0h,&p_t_15_0m,p0_accu_coeff_23h,p_t_14_0h,p_t_14_0m);
Mul22(&p_t_16_0h,&p_t_16_0m,p_t_15_0h,p_t_15_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_17_0h,&p_t_17_0m,p0_accu_coeff_21h,p_t_16_0h,p_t_16_0m);
Mul22(&p_t_18_0h,&p_t_18_0m,p_t_17_0h,p_t_17_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_19_0h,&p_t_19_0m,p0_accu_coeff_19h,p_t_18_0h,p_t_18_0m);
Mul22(&p_t_20_0h,&p_t_20_0m,p_t_19_0h,p_t_19_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_21_0h,&p_t_21_0m,p0_accu_coeff_17h,p_t_20_0h,p_t_20_0m);
Mul22(&p_t_22_0h,&p_t_22_0m,p_t_21_0h,p_t_21_0m,p_x_0_pow2h,p_x_0_pow2m);
Add122(&p_t_23_0h,&p_t_23_0m,p0_accu_coeff_15h,p_t_22_0h,p_t_22_0m);
MulAdd22(&p_t_24_0h,&p_t_24_0m,p0_accu_coeff_13h,p0_accu_coeff_13m,p_x_0_pow2h,p_x_0_pow2m,p_t_23_0h,p_t_23_0m);
MulAdd22(&p_t_25_0h,&p_t_25_0m,p0_accu_coeff_11h,p0_accu_coeff_11m,p_x_0_pow2h,p_x_0_pow2m,p_t_24_0h,p_t_24_0m);
MulAdd22(&p_t_26_0h,&p_t_26_0m,p0_accu_coeff_9h,p0_accu_coeff_9m,p_x_0_pow2h,p_x_0_pow2m,p_t_25_0h,p_t_25_0m);
Mul22(&p_t_27_0h,&p_t_27_0m,p_t_26_0h,p_t_26_0m,p_x_0_pow2h,p_x_0_pow2m);
Add23(&p_t_28_0h,&p_t_28_0m,&p_t_28_0l,p0_accu_coeff_7h,p0_accu_coeff_7m,p_t_27_0h,p_t_27_0m);
Mul233(&p_t_29_0h,&p_t_29_0m,&p_t_29_0l,p_x_0_pow2h,p_x_0_pow2m,p_t_28_0h,p_t_28_0m,p_t_28_0l);
Add233(&p_t_30_0h,&p_t_30_0m,&p_t_30_0l,p0_accu_coeff_5h,p0_accu_coeff_5m,p_t_29_0h,p_t_29_0m,p_t_29_0l);
Mul233(&p_t_31_0h,&p_t_31_0m,&p_t_31_0l,p_x_0_pow2h,p_x_0_pow2m,p_t_30_0h,p_t_30_0m,p_t_30_0l);
Add233(&p_t_32_0h,&p_t_32_0m,&p_t_32_0l,p0_accu_coeff_3h,p0_accu_coeff_3m,p_t_31_0h,p_t_31_0m,p_t_31_0l);
Mul233(&p_t_33_0h,&p_t_33_0m,&p_t_33_0l,p_x_0_pow2h,p_x_0_pow2m,p_t_32_0h,p_t_32_0m,p_t_32_0l);
Add133(&p_t_34_0h,&p_t_34_0m,&p_t_34_0l,p0_accu_coeff_1h,p_t_33_0h,p_t_33_0m,p_t_33_0l);
Mul133(&p_t_35_0h,&p_t_35_0m,&p_t_35_0l,x,p_t_34_0h,p_t_34_0m,p_t_34_0l);
Renormalize3(p_resh,p_resm,p_resl,p_t_35_0h,p_t_35_0m,p_t_35_0l);


}


static void p_accu(double *p_resh, double *p_resm, double *p_resl, double x, int index) {
  

double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h;
double p_t_8_0h;
double p_t_9_0h;
double p_t_10_0h;
double p_t_11_0h;
double p_t_12_0h;
double p_t_13_0h;
double p_t_14_0h;
double p_t_15_0h;
double p_t_16_0h;
double p_t_17_0h, p_t_17_0m;
double p_t_18_0h, p_t_18_0m;
double p_t_19_0h, p_t_19_0m;
double p_t_20_0h, p_t_20_0m;
double p_t_21_0h, p_t_21_0m;
double p_t_22_0h, p_t_22_0m;
double p_t_23_0h, p_t_23_0m;
double p_t_24_0h, p_t_24_0m;
double p_t_25_0h, p_t_25_0m;
double p_t_26_0h, p_t_26_0m;
double p_t_27_0h, p_t_27_0m;
double p_t_28_0h, p_t_28_0m;
double p_t_29_0h, p_t_29_0m;
double p_t_30_0h, p_t_30_0m;
double p_t_31_0h, p_t_31_0m;
double p_t_32_0h, p_t_32_0m;
double p_t_33_0h, p_t_33_0m;
double p_t_34_0h, p_t_34_0m;
double p_t_35_0h, p_t_35_0m, p_t_35_0l;
double p_t_36_0h, p_t_36_0m, p_t_36_0l;
double p_t_37_0h, p_t_37_0m, p_t_37_0l;
double p_t_38_0h, p_t_38_0m, p_t_38_0l;
double p_t_39_0h, p_t_39_0m, p_t_39_0l;
 


p_t_1_0h = p_accu_coeff_22h;
p_t_2_0h = p_t_1_0h * x;
p_t_3_0h = p_accu_coeff_21h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * x;
p_t_5_0h = p_accu_coeff_20h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * x;
p_t_7_0h = p_accu_coeff_19h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * x;
p_t_9_0h = p_accu_coeff_18h + p_t_8_0h;
p_t_10_0h = p_t_9_0h * x;
p_t_11_0h = p_accu_coeff_17h + p_t_10_0h;
p_t_12_0h = p_t_11_0h * x;
p_t_13_0h = p_accu_coeff_16h + p_t_12_0h;
p_t_14_0h = p_t_13_0h * x;
p_t_15_0h = p_accu_coeff_15h + p_t_14_0h;
p_t_16_0h = p_t_15_0h * x;
Add12(p_t_17_0h,p_t_17_0m,p_accu_coeff_14h,p_t_16_0h);
Mul122(&p_t_18_0h,&p_t_18_0m,x,p_t_17_0h,p_t_17_0m);
Add122(&p_t_19_0h,&p_t_19_0m,p_accu_coeff_13h,p_t_18_0h,p_t_18_0m);
Mul122(&p_t_20_0h,&p_t_20_0m,x,p_t_19_0h,p_t_19_0m);
Add122(&p_t_21_0h,&p_t_21_0m,p_accu_coeff_12h,p_t_20_0h,p_t_20_0m);
Mul122(&p_t_22_0h,&p_t_22_0m,x,p_t_21_0h,p_t_21_0m);
Add122(&p_t_23_0h,&p_t_23_0m,p_accu_coeff_11h,p_t_22_0h,p_t_22_0m);
Mul122(&p_t_24_0h,&p_t_24_0m,x,p_t_23_0h,p_t_23_0m);
Add122(&p_t_25_0h,&p_t_25_0m,p_accu_coeff_10h,p_t_24_0h,p_t_24_0m);
Mul122(&p_t_26_0h,&p_t_26_0m,x,p_t_25_0h,p_t_25_0m);
Add122(&p_t_27_0h,&p_t_27_0m,p_accu_coeff_9h,p_t_26_0h,p_t_26_0m);
MulAdd212(&p_t_28_0h,&p_t_28_0m,p_accu_coeff_8h,p_accu_coeff_8m,x,p_t_27_0h,p_t_27_0m);
MulAdd212(&p_t_29_0h,&p_t_29_0m,p_accu_coeff_7h,p_accu_coeff_7m,x,p_t_28_0h,p_t_28_0m);
MulAdd212(&p_t_30_0h,&p_t_30_0m,p_accu_coeff_6h,p_accu_coeff_6m,x,p_t_29_0h,p_t_29_0m);
MulAdd212(&p_t_31_0h,&p_t_31_0m,p_accu_coeff_5h,p_accu_coeff_5m,x,p_t_30_0h,p_t_30_0m);
MulAdd212(&p_t_32_0h,&p_t_32_0m,p_accu_coeff_4h,p_accu_coeff_4m,x,p_t_31_0h,p_t_31_0m);
MulAdd212(&p_t_33_0h,&p_t_33_0m,p_accu_coeff_3h,p_accu_coeff_3m,x,p_t_32_0h,p_t_32_0m);
Mul122(&p_t_34_0h,&p_t_34_0m,x,p_t_33_0h,p_t_33_0m);
Add23(&p_t_35_0h,&p_t_35_0m,&p_t_35_0l,p_accu_coeff_2h,p_accu_coeff_2m,p_t_34_0h,p_t_34_0m);
Mul133(&p_t_36_0h,&p_t_36_0m,&p_t_36_0l,x,p_t_35_0h,p_t_35_0m,p_t_35_0l);
Add233(&p_t_37_0h,&p_t_37_0m,&p_t_37_0l,p_accu_coeff_1h,p_accu_coeff_1m,p_t_36_0h,p_t_36_0m,p_t_36_0l);
Mul133(&p_t_38_0h,&p_t_38_0m,&p_t_38_0l,x,p_t_37_0h,p_t_37_0m,p_t_37_0l);
Add233(&p_t_39_0h,&p_t_39_0m,&p_t_39_0l,p_accu_coeff_0h,p_accu_coeff_0m,p_t_38_0h,p_t_38_0m,p_t_38_0l);
Renormalize3(p_resh,p_resm,p_resl,p_t_39_0h,p_t_39_0m,p_t_39_0l);


}


static void p9_accu(double *p_resh, double *p_resm, double *p_resl, double x) {




double p_t_1_0h;
double p_t_2_0h;
double p_t_3_0h;
double p_t_4_0h;
double p_t_5_0h;
double p_t_6_0h;
double p_t_7_0h;
double p_t_8_0h;
double p_t_9_0h;
double p_t_10_0h;
double p_t_11_0h;
double p_t_12_0h;
double p_t_13_0h;
double p_t_14_0h;
double p_t_15_0h;
double p_t_16_0h;
double p_t_17_0h, p_t_17_0m;
double p_t_18_0h, p_t_18_0m;
double p_t_19_0h, p_t_19_0m;
double p_t_20_0h, p_t_20_0m;
double p_t_21_0h, p_t_21_0m;
double p_t_22_0h, p_t_22_0m;
double p_t_23_0h, p_t_23_0m;
double p_t_24_0h, p_t_24_0m;
double p_t_25_0h, p_t_25_0m;
double p_t_26_0h, p_t_26_0m;
double p_t_27_0h, p_t_27_0m;
double p_t_28_0h, p_t_28_0m;
double p_t_29_0h, p_t_29_0m;
double p_t_30_0h, p_t_30_0m;
double p_t_31_0h, p_t_31_0m;
double p_t_32_0h, p_t_32_0m;
double p_t_33_0h, p_t_33_0m, p_t_33_0l;
double p_t_34_0h, p_t_34_0m, p_t_34_0l;
double p_t_35_0h, p_t_35_0m, p_t_35_0l;
 


p_t_1_0h = p9_accu_coeff_20h;
p_t_2_0h = p_t_1_0h * x;
p_t_3_0h = p9_accu_coeff_19h + p_t_2_0h;
p_t_4_0h = p_t_3_0h * x;
p_t_5_0h = p9_accu_coeff_18h + p_t_4_0h;
p_t_6_0h = p_t_5_0h * x;
p_t_7_0h = p9_accu_coeff_17h + p_t_6_0h;
p_t_8_0h = p_t_7_0h * x;
p_t_9_0h = p9_accu_coeff_16h + p_t_8_0h;
p_t_10_0h = p_t_9_0h * x;
p_t_11_0h = p9_accu_coeff_15h + p_t_10_0h;
p_t_12_0h = p_t_11_0h * x;
p_t_13_0h = p9_accu_coeff_14h + p_t_12_0h;
p_t_14_0h = p_t_13_0h * x;
p_t_15_0h = p9_accu_coeff_13h + p_t_14_0h;
p_t_16_0h = p_t_15_0h * x;
Add12(p_t_17_0h,p_t_17_0m,p9_accu_coeff_12h,p_t_16_0h);
Mul122(&p_t_18_0h,&p_t_18_0m,x,p_t_17_0h,p_t_17_0m);
Add122(&p_t_19_0h,&p_t_19_0m,p9_accu_coeff_11h,p_t_18_0h,p_t_18_0m);
Mul122(&p_t_20_0h,&p_t_20_0m,x,p_t_19_0h,p_t_19_0m);
Add122(&p_t_21_0h,&p_t_21_0m,p9_accu_coeff_10h,p_t_20_0h,p_t_20_0m);
Mul122(&p_t_22_0h,&p_t_22_0m,x,p_t_21_0h,p_t_21_0m);
Add122(&p_t_23_0h,&p_t_23_0m,p9_accu_coeff_9h,p_t_22_0h,p_t_22_0m);
Mul122(&p_t_24_0h,&p_t_24_0m,x,p_t_23_0h,p_t_23_0m);
Add122(&p_t_25_0h,&p_t_25_0m,p9_accu_coeff_8h,p_t_24_0h,p_t_24_0m);
MulAdd212(&p_t_26_0h,&p_t_26_0m,p9_accu_coeff_7h,p9_accu_coeff_7m,x,p_t_25_0h,p_t_25_0m);
MulAdd212(&p_t_27_0h,&p_t_27_0m,p9_accu_coeff_6h,p9_accu_coeff_6m,x,p_t_26_0h,p_t_26_0m);
MulAdd212(&p_t_28_0h,&p_t_28_0m,p9_accu_coeff_5h,p9_accu_coeff_5m,x,p_t_27_0h,p_t_27_0m);
MulAdd212(&p_t_29_0h,&p_t_29_0m,p9_accu_coeff_4h,p9_accu_coeff_4m,x,p_t_28_0h,p_t_28_0m);
MulAdd212(&p_t_30_0h,&p_t_30_0m,p9_accu_coeff_3h,p9_accu_coeff_3m,x,p_t_29_0h,p_t_29_0m);
MulAdd212(&p_t_31_0h,&p_t_31_0m,p9_accu_coeff_2h,p9_accu_coeff_2m,x,p_t_30_0h,p_t_30_0m);
Mul122(&p_t_32_0h,&p_t_32_0m,x,p_t_31_0h,p_t_31_0m);
Add23(&p_t_33_0h,&p_t_33_0m,&p_t_33_0l,p9_accu_coeff_1h,p9_accu_coeff_1m,p_t_32_0h,p_t_32_0m);
Mul133(&p_t_34_0h,&p_t_34_0m,&p_t_34_0l,x,p_t_33_0h,p_t_33_0m,p_t_33_0l);
Add233(&p_t_35_0h,&p_t_35_0m,&p_t_35_0l,p9_accu_coeff_0h,p9_accu_coeff_0m,p_t_34_0h,p_t_34_0m,p_t_34_0l);
Renormalize3(p_resh,p_resm,p_resl,p_t_35_0h,p_t_35_0m,p_t_35_0l);


}




double asin_rn(double x) {
  db_number xdb, zdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinhover, asinmover, asinlover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double asin;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arcsin(x) = x * ( 1 + xi ) 

     with 0 <= xi < 2^(-55) 
          
     So we can decide the rounding without any computation 
  */
  if (xdb.i[HI] < ASINSIMPLEBOUND) {
    return x;
  }

  /* asin is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return PIHALFH;
    }
    if (x == -1.0) {
      return - PIHALFH;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Rounding test */

    if(asinh == (asinh + (asinm * RNROUNDCST))) 
      return asinh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Final rounding */

    RoundToNearest3(&asin,asinh,asinm,asinl);

    return asin;
  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Add22(&asinh,&asinm,PIHALFH,PIHALFM,t1h,t1m);

    /* Rounding test */

    if(asinh == (asinh + (asinm * RNROUNDCST))) 
      return sign * asinh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Add33(&asinhover,&asinmover,&asinlover,PIHALFH,PIHALFM,PIHALFL,t1h,t1m,t1l);

    Renormalize3(&asinh,&asinm,&asinl,asinhover,asinmover,asinlover);

    /* Final rounding */    

    RoundToNearest3(&asin,asinh,asinm,asinl);

    return sign * asin;

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);


  /* Rounding test */
  
  if(asinh == (asinh + (asinm * RNROUNDCST))) 
    return sign * asinh;
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);
  
  /* Final rounding */
  
  RoundToNearest3(&asin,asinh,asinm,asinl);
  
  return sign * asin;
  
}


double asin_ru(double x) {
  db_number xdb, zdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinhover, asinmover, asinlover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arcsin(x) = x * ( 1 + xi ) 

     with 0 <= xi < 2^(-55) 
          
     So we can decide the rounding without any computation 
  */
  if (xdb.i[HI] < ASINSIMPLEBOUND) {
    /* If x == 0 then we got the algebraic result arcsin(0) = 0
       If x < 0 then the truncation rest is negative but less than 
       1 ulp; we round upwards by returning x
    */
    if (x <= 0.0) return x;
    /* Otherwise the rest is positive, less than 1 ulp and the
       image is not algebraic 
       We return x + 1ulp
    */
    xdb.l++;
    return xdb.d;
  }

  /* asin is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return PIHALFRU;
    }
    if (x == -1.0) {
      return - PIHALFH;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Rounding test */

    TEST_AND_RETURN_RU(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Final rounding */

    ReturnRoundUpwards3(asinh,asinm,asinl);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Add22(&asinh,&asinm,PIHALFH,PIHALFM,t1h,t1m);

    /* Rounding test */

    asinh *= sign;
    asinm *= sign;

    TEST_AND_RETURN_RU(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Add33(&asinhover,&asinmover,&asinlover,PIHALFH,PIHALFM,PIHALFL,t1h,t1m,t1l);

    Renormalize3(&asinh,&asinm,&asinl,asinhover,asinmover,asinlover);

    /* Final rounding */    

    asinh *= sign;
    asinm *= sign;
    asinl *= sign;

    ReturnRoundUpwards3(asinh,asinm,asinl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);


  /* Rounding test */

  asinh *= sign;
  asinm *= sign;
  
  TEST_AND_RETURN_RU(asinh, asinm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  if (x == ASINBADCASEX) return ASINBADCASEYRU;

  p_accu(&asinh, &asinm, &asinl, z, index);
  
  /* Final rounding */

  asinh *= sign;
  asinm *= sign;
  asinl *= sign;
  
  ReturnRoundUpwards3(asinh,asinm,asinl);

}

double asin_rd(double x) {
  db_number xdb, zdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinhover, asinmover, asinlover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arcsin(x) = x * ( 1 + xi ) 

     with 0 <= xi < 2^(-55) 
          
     So we can decide the rounding without any computation 
  */
  if (xdb.i[HI] < ASINSIMPLEBOUND) {
    /* If x == 0 then we got the algebraic result arcsin(0) = 0
       If x > 0 then the truncation rest is positive but less than 
       1 ulp; we round downwards by returning x
    */
    if (x >= 0) return x;
    /* Otherwise the rest is negative, less than 1 ulp and the
       image is not algebraic 
       We return x - 1ulp
       We stripped off the sign, so we add 1 ulp to -x (in xdb.d) and multiply by -1
    */
    xdb.l++;
    return -1 * xdb.d;
  }

  /* asin is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return PIHALFH;
    }
    if (x == -1.0) {
      return - PIHALFRU;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Rounding test */

    TEST_AND_RETURN_RD(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Final rounding */

    ReturnRoundDownwards3(asinh,asinm,asinl);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Add22(&asinh,&asinm,PIHALFH,PIHALFM,t1h,t1m);

    /* Rounding test */

    asinh *= sign;
    asinm *= sign;

    TEST_AND_RETURN_RD(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Add33(&asinhover,&asinmover,&asinlover,PIHALFH,PIHALFM,PIHALFL,t1h,t1m,t1l);

    Renormalize3(&asinh,&asinm,&asinl,asinhover,asinmover,asinlover);

    /* Final rounding */    

    asinh *= sign;
    asinm *= sign;
    asinl *= sign;

    ReturnRoundDownwards3(asinh,asinm,asinl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);


  /* Rounding test */

  asinh *= sign;
  asinm *= sign;
  
  TEST_AND_RETURN_RD(asinh, asinm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  if (x == ASINBADCASEX) return ASINBADCASEYRD;

  p_accu(&asinh, &asinm, &asinl, z, index);
  
  /* Final rounding */

  asinh *= sign;
  asinm *= sign;
  asinl *= sign;
  
  ReturnRoundDownwards3(asinh,asinm,asinl);

}

double asin_rz(double x) {
  db_number xdb, zdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinhover, asinmover, asinlover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arcsin(x) = x * ( 1 + xi ) 

     with 0 <= xi < 2^(-55) 
          
     So we can decide the rounding without any computation 
  */
  if (xdb.i[HI] < ASINSIMPLEBOUND) {
    /* If x == 0 the result is algebraic and equal to 0
       If x < 0 the truncation rest is negative and less than 1 ulp, we return x
       If x > 0 the truncation rest is positive and less than 1 ulp, we return x
    */
    return x;
  }

  /* asin is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return PIHALFH;
    }
    if (x == -1.0) {
      return - PIHALFH;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Rounding test */

    TEST_AND_RETURN_RZ(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Final rounding */

    ReturnRoundTowardsZero3(asinh,asinm,asinl);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Add22(&asinh,&asinm,PIHALFH,PIHALFM,t1h,t1m);

    /* Rounding test */

    asinh *= sign;
    asinm *= sign;

    TEST_AND_RETURN_RZ(asinh, asinm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Add33(&asinhover,&asinmover,&asinlover,PIHALFH,PIHALFM,PIHALFL,t1h,t1m,t1l);

    Renormalize3(&asinh,&asinm,&asinl,asinhover,asinmover,asinlover);

    /* Final rounding */    

    asinh *= sign;
    asinm *= sign;
    asinl *= sign;

    ReturnRoundTowardsZero3(asinh,asinm,asinl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);


  /* Rounding test */

  asinh *= sign;
  asinm *= sign;
  
  TEST_AND_RETURN_RZ(asinh, asinm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  if (x == ASINBADCASEX) return ASINBADCASEYRD;

  p_accu(&asinh, &asinm, &asinl, z, index);
  
  /* Final rounding */

  asinh *= sign;
  asinm *= sign;
  asinl *= sign;
  
  ReturnRoundTowardsZero3(asinh,asinm,asinl);

}

double acos_rn(double x) {
  db_number xdb, zdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arccos(x) = (double-double(pi/2) - x) * ( 1 + xi ) 

     with 0 <= xi < 2^(-87) 
          
     From 2^(-27) we have no bad rounding case longer than 5 bits 
     more than the ulp of x, thus the approximation suffices.

  */
  if (xdb.i[HI] < ACOSSIMPLEBOUND) {
    Add212(&acosh,&acosm,PIHALFH,PIHALFM,-x);
    
    return acosh;
  }

  /* acos is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return PIH;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos 

       No cancellation possible because 

       |asin(x)| <= 0.264 for |x| <= 0.26

    */

    asinh = - asinh;
    asinm = - asinm;

    Add22(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

    /* Rounding test */

    if(acosh == (acosh + (acosm * RNROUNDCST))) 
      return acosh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos */

    asinh = -asinh;
    asinm = -asinm;
    asinl = -asinl;

    Add33(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundToNearest3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    if (x > 0.0) {
      acosh = t1h;
      acosm = t1m;
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;

      Add22(&acosh,&acosm,PIH,PIM,t1h,t1m);
    }

    /* Rounding test */

    if(acosh == (acosh + (acosm * RNROUNDCST))) 
      return acosh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t1h,t1m,t1l);
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;
      t1l = - t1l;

      Add33(&acoshover,&acosmover,&acoslover,PIH,PIM,PIL,t1h,t1m,t1l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundToNearest3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Add22Cond(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

  /* Rounding test */
  
  if(acosh == (acosh + (acosm * RNROUNDCST))) 
    return acosh;
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Add33Cond(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);


  /* Final rounding */
  
  ReturnRoundToNearest3(acosh,acosm,acosl);
  
  
}

double acos_ru(double x) {
  db_number xdb, zdb, acoshdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arccos(x) = (double-double(pi/2) - x) * ( 1 + xi ) 

     with 0 <= xi < 2^(-87) 
          
     From 2^(-27) we have no bad rounding case longer than 5 bits 
     more than the ulp of x, thus the approximation suffices.

  */
  if (xdb.i[HI] < ACOSSIMPLEBOUND) {
    Add212(&acosh,&acosm,PIHALFH,PIHALFM,-x);
    
    /* acosh is the round-to-nearest of acos(x) in this domain
       
       acosm is a at least 20 bit exact correction of the
       rounding error of this round-to-nearest rounding.

       If acosh is the rounded up result of acos(x), the
       correction is negative and vice-versa.

    */

    if (acosm < 0.0) 
      return acosh;
 
    /* Here the correction acosm is positive, acosh is 
       therefore the rounded down result of acos(x).
       We add thus one ulp.
    */

    acoshdb.d = acosh;
    acoshdb.l++;

    return acoshdb.d;
    
  }

  /* acos is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return PIRU;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos 

       No cancellation possible because 

       |asin(x)| <= 0.264 for |x| <= 0.26

    */

    asinh = - asinh;
    asinm = - asinm;

    Add22(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

    /* Rounding test */

    TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos */

    asinh = -asinh;
    asinm = -asinm;
    asinl = -asinl;

    Add33(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundUpwards3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    if (x > 0.0) {
      acosh = t1h;
      acosm = t1m;
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;

      Add22(&acosh,&acosm,PIH,PIM,t1h,t1m);
    }

    /* Rounding test */

    TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t1h,t1m,t1l);
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;
      t1l = - t1l;

      Add33(&acoshover,&acosmover,&acoslover,PIH,PIM,PIL,t1h,t1m,t1l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundUpwards3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Add22Cond(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

  /* Rounding test */
  
  TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Add33Cond(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);


  /* Final rounding */
  
  ReturnRoundUpwards3(acosh,acosm,acosl);
  
}


double acos_rd(double x) {
  db_number xdb, zdb, acoshdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-28) we have
     
     arccos(x) = (double-double(pi/2) - x) * ( 1 + xi ) 

     with 0 <= xi < 2^(-87) 
          
     From 2^(-27) we have no bad rounding case longer than 5 bits 
     more than the ulp of x, thus the approximation suffices.

  */
  if (xdb.i[HI] < ACOSSIMPLEBOUND) {
    Add212(&acosh,&acosm,PIHALFH,PIHALFM,-x);
    
    /* acosh is the round-to-nearest of acos(x) in this domain
       
       acosm is a at least 20 bit exact correction of the
       rounding error of this round-to-nearest rounding.

       If acosh is the rounded up result of acos(x), the
       correction is negative and vice-versa.

    */

    if (acosm > 0.0) 
      return acosh;
 
    /* Here the correction acosm is negative, acosh is 
       therefore the rounded up result of acos(x).
       We subtract thus one ulp.
    */

    acoshdb.d = acosh;
    acoshdb.l--;

    return acoshdb.d;
    
  }

  /* acos is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return PIH;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos 

       No cancellation possible because 

       |asin(x)| <= 0.264 for |x| <= 0.26

    */

    asinh = - asinh;
    asinm = - asinm;

    Add22(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

    /* Rounding test */

    TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos */

    asinh = -asinh;
    asinm = -asinm;
    asinl = -asinl;

    Add33(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundDownwards3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    if (x > 0.0) {
      acosh = t1h;
      acosm = t1m;
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;

      Add22(&acosh,&acosm,PIH,PIM,t1h,t1m);
    }

    /* Rounding test */

    TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t1h,t1m,t1l);
    } else {
      /* arccos(-x) = Pi - arccos(x) */
      t1h = - t1h;
      t1m = - t1m;
      t1l = - t1l;

      Add33(&acoshover,&acosmover,&acoslover,PIH,PIM,PIL,t1h,t1m,t1l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundDownwards3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Add22Cond(&acosh,&acosm,PIHALFH,PIHALFM,asinh,asinm);

  /* Rounding test */
  
  TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Add33Cond(&acoshover,&acosmover,&acoslover,PIHALFH,PIHALFM,PIHALFL,asinh,asinm,asinl);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

  /* Final rounding */
  
  ReturnRoundDownwards3(acosh,acosm,acosl);
  
}




double acospi_rn(double x) {
  db_number xdb, zdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-54) we have
     
     arccos(x)/pi = 1/2 * (1 + xi)

     with 0 <= xi < 2^(-54) 
          
     So arcospi(x) = 0.5 in this case.

  */
  if (xdb.i[HI] < ACOSPISIMPLEBOUND) {
    return 0.5;
  }

  /* acospi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return 1.0;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos/pi

       We have 

       arccos(x)/pi = 1/2 + (-1/pi) * arcsin(x)

       No cancellation possible because 

       |asin(x)/pi| <= 0.0837 for |x| <= 0.26

    */

    Mul22(&asinpih,&asinpim,MRECPRPIH,MRECPRPIM,asinh,asinm);
    
    Add122(&acosh,&acosm,0.5,asinpih,asinpim);

    /* Rounding test */

    if(acosh == (acosh + (acosm * RNROUNDCST))) 
      return acosh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos/pi */

    Mul33(&asinpih,&asinpim,&asinpil,MRECPRPIH,MRECPRPIM,MRECPRPIL,asinh,asinm,asinl);
    
    Add133(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundToNearest3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    Mul22(&t2h,&t2m,RECPRPIH,RECPRPIM,t1h,t1m);

    if (x > 0.0) {
      acosh = t2h;
      acosm = t2m;
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;

      Add122(&acosh,&acosm,1.0,t2h,t2m);
    }

    /* Rounding test */

    if(acosh == (acosh + (acosm * RNROUNDCST))) 
      return acosh;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    Mul33(&t2h,&t2m,&t2l,RECPRPIH,RECPRPIM,RECPRPIL,t1h,t1m,t1l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t2h,t2m,t2l);
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;
      t2l = - t2l;

      Add133(&acoshover,&acosmover,&acoslover,1.0,t2h,t2m,t2l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundToNearest3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x)/pi out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Mul22(&asinpih,&asinpim,RECPRPIH,RECPRPIM,asinh,asinm);

  Add122Cond(&acosh,&acosm,0.5,asinpih,asinpim);

  /* Rounding test */
  
  if(acosh == (acosh + (acosm * RNROUNDCST))) 
    return acosh;
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Mul33(&asinpih,&asinpim,&asinpil,RECPRPIH,RECPRPIM,RECPRPIL,asinh,asinm,asinl);

  Add133Cond(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

  /* Final rounding */
  
  ReturnRoundToNearest3(acosh,acosm,acosl);
    
}

double acospi_rd(double x) {
  db_number xdb, zdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-54) we have
     
     arccos(x)/pi = 1/2 * (1 + xi)

     with 0 <= xi < 2^(-54) 
          
     Thus we have

     acospi(x) = 

     (i)  0.5                if x <= 0
     (ii) 0.5 - 1/2 ulp(0.5) if x > 0

  */
  if (xdb.i[HI] < ACOSPISIMPLEBOUND) {
    if (x <= 0.0) 
      return 0.5;
    
    return HALFMINUSHALFULP;
  }

  /* acospi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return 1.0;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos/pi

       We have 

       arccos(x)/pi = 1/2 + (-1/pi) * arcsin(x)

       No cancellation possible because 

       |asin(x)/pi| <= 0.0837 for |x| <= 0.26

    */

    Mul22(&asinpih,&asinpim,MRECPRPIH,MRECPRPIM,asinh,asinm);
    
    Add122(&acosh,&acosm,0.5,asinpih,asinpim);

    /* Rounding test */

    TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos/pi */

    Mul33(&asinpih,&asinpim,&asinpil,MRECPRPIH,MRECPRPIM,MRECPRPIL,asinh,asinm,asinl);
    
    Add133(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundDownwards3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    Mul22(&t2h,&t2m,RECPRPIH,RECPRPIM,t1h,t1m);

    if (x > 0.0) {
      acosh = t2h;
      acosm = t2m;
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;

      Add122(&acosh,&acosm,1.0,t2h,t2m);
    }

    /* Rounding test */

    TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    Mul33(&t2h,&t2m,&t2l,RECPRPIH,RECPRPIM,RECPRPIL,t1h,t1m,t1l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t2h,t2m,t2l);
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;
      t2l = - t2l;

      Add133(&acoshover,&acosmover,&acoslover,1.0,t2h,t2m,t2l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundDownwards3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x)/pi out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Mul22(&asinpih,&asinpim,RECPRPIH,RECPRPIM,asinh,asinm);

  Add122Cond(&acosh,&acosm,0.5,asinpih,asinpim);

  /* Rounding test */
  
  TEST_AND_RETURN_RD(acosh, acosm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Mul33(&asinpih,&asinpim,&asinpil,RECPRPIH,RECPRPIM,RECPRPIL,asinh,asinm,asinl);

  Add133Cond(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

  /* Final rounding */
  
  ReturnRoundDownwards3(acosh,acosm,acosl);
    
}

double acospi_ru(double x) {
  db_number xdb, zdb;
  double z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double acosh, acosm, acosl;
  double acoshover, acosmover, acoslover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double xabs;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-54) we have
     
     arccos(x)/pi = 1/2 * (1 + xi)

     with 0 <= xi < 2^(-54) 
          
     Thus we have

     acospi(x) = 

     (i)  0.5              if x >= 0
     (ii) 0.5 + 1 ulp(0.5) if x < 0

  */
  if (xdb.i[HI] < ACOSPISIMPLEBOUND) {
    if (x >= 0.0) 
      return 0.5;
    
    return 0.50000000000000011102230246251565404236316680908203125;
  }

  /* acospi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.0;
    }
    if (x == -1.0) {
      return 1.0;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    /* Recompose acos/pi

       We have 

       arccos(x)/pi = 1/2 + (-1/pi) * arcsin(x)

       No cancellation possible because 

       |asin(x)/pi| <= 0.0837 for |x| <= 0.26

    */

    Mul22(&asinpih,&asinpim,MRECPRPIH,MRECPRPIM,asinh,asinm);
    
    Add122(&acosh,&acosm,0.5,asinpih,asinpim);

    /* Rounding test */

    TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    /* Recompose acos/pi */

    Mul33(&asinpih,&asinpim,&asinpil,MRECPRPIH,MRECPRPIM,MRECPRPIL,asinh,asinm,asinl);
    
    Add133(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);
    
    Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

    /* Final rounding */

    ReturnRoundUpwards3(acosh,acosm,acosl);

  } 
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    
    Mul22(&t2h,&t2m,RECPRPIH,RECPRPIM,t1h,t1m);

    if (x > 0.0) {
      acosh = t2h;
      acosm = t2m;
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;

      Add122(&acosh,&acosm,1.0,t2h,t2m);
    }

    /* Rounding test */

    TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);

    Mul33(&t2h,&t2m,&t2l,RECPRPIH,RECPRPIM,RECPRPIL,t1h,t1m,t1l);

    if (x > 0.0) {
      Renormalize3(&acosh,&acosm,&acosl,t2h,t2m,t2l);
    } else {
      /* arccos(-x)/pi = 1 - arccos(x)/pi */
      t2h = - t2h;
      t2m = - t2m;
      t2l = - t2l;

      Add133(&acoshover,&acosmover,&acoslover,1.0,t2h,t2m,t2l);

      Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);
    }

    /* Final rounding */    

    ReturnRoundUpwards3(acosh,acosm,acosl);

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  /* Recompose acos(x)/pi out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
  }

  Mul22(&asinpih,&asinpim,RECPRPIH,RECPRPIM,asinh,asinm);

  Add122Cond(&acosh,&acosm,0.5,asinpih,asinpim);

  /* Rounding test */
  
  TEST_AND_RETURN_RU(acosh, acosm, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);

  /* Recompose acos(x) out of asin(abs(x)) 

     In the case of a substraction, we will cancel 
     not more than 1 bit.

  */

  if (x > 0.0) {
    asinh = - asinh;
    asinm = - asinm;
    asinl = - asinl; 
  }

  Mul33(&asinpih,&asinpim,&asinpil,RECPRPIH,RECPRPIM,RECPRPIL,asinh,asinm,asinl);

  Add133Cond(&acoshover,&acosmover,&acoslover,0.5,asinpih,asinpim,asinpil);

  Renormalize3(&acosh,&acosm,&acosl,acoshover,acosmover,acoslover);

  /* Final rounding */
  
  ReturnRoundUpwards3(acosh,acosm,acosl);
    
}



double asinpi_rn(double x) {
  db_number xdb, zdb, asinhdb, tempdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double asinpihover, asinpimover, asinpilover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double asinpi;
  double xabs;
  double xScaled;
  double xPih, xPim, xPil;
  double xPihover, xPimover, xPilover;
  double deltatemp, deltah, deltal;
  double temp1, temp2h, temp2l, temp3;
  double miulp;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-60) we have
     
     arcsin(x)/pi = x * triple-double(1/pi) * ( 1 + xi ) 

     with 0 <= xi < 2^(-122)

     We have no bad case worser than 
     112 bits for asinpi(x) for |x| > 2^(-58)
     and the order 3 term of asinpi is still more
     far away. 
          
  */
  if (xdb.i[HI] < ASINPISIMPLEBOUND) {
    /* For a faster path for the exact case, 
       we check first if x = 0
    */

    if (x == 0.0) 
      return x;

    /* We want a relatively fast path for values where
       neither the input nor the output is subnormal 
       because subnormal rounding is expensive.

       Since 
       
       abs(double(asin(x)/pi)) <= 2^(-860) 
       
       for abs(x) <= 2^(-858), we filter for this (normal) value.

    */

    if (xdb.i[HI] >= ASINPINOSUBNORMALBOUND) {

      /* Here abs(x) >= 2^(-858).
	 The result is therefore clearly normal and
	 the double precision numbers in the triple-double
	 representation of TD(1/pi) * x are all normal, too.

	 For speed, we use a two-step approach.
	 We know that 

	 || x*DD(1/pi)/(asin(x)/pi) - 1 ||_[-2^(-60);2^(-60)]^\infty <= 2^(-107.8) <= 2^(-80) 

      */

      Mul122(&xPih,&xPim,x,RECPRPIH,RECPRPIM);

      if(xPih == (xPih + (xPim * RNROUNDCSTASINPI))) 
	return xPih;
      
      Mul133(&xPihover,&xPimover,&xPilover,x,RECPRPIH,RECPRPIM,RECPRPIL);

      Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);
      
      ReturnRoundToNearest3(xPih,xPim,xPil);

    } 

    /* Here abs(x) < 2^(-858)

       Because of subnormals and especially because
       of the fact that 1/pi < 1, we must scale x
       appropriately. We compute hence:

       asinpi(x) = round( ((x * 2^(1000)) * triple-double(1/pi)) * 2^(-1000))

       where the rounding procedure works temporarily on the scaled 
       intermediate.
    */

    xScaled = x * TWO1000;

    Mul133(&xPihover,&xPimover,&xPilover,xScaled,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);

    /* Produce a (possibly) subnormal intermediate rounding */

    asinhdb.d = xPih * TWOM1000;

#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
    tempdb.i[HI] = asinhdb.i[HI];
    tempdb.i[LO] = asinhdb.i[LO];
    asinhdb.i[LO] = tempdb.i[LO];
    asinhdb.i[HI] = tempdb.i[HI];
#else 
    tempdb.d = asinhdb.d;
#endif

    /* Rescale the result */

    temp1 = asinhdb.d * TWO1000;

    /* Compute the scaled error, the operation is exact by Sterbenz' lemma */

    deltatemp = xPih - temp1;

    /* Sum up the errors, representing them on a double-double
       
       This is exact for the normal rounding case and the error
       is neglectable in the subnormal rounding case.
    */

    Add12Cond(temp2h,temp2l,deltatemp,xPim);
    temp3 = temp2l + xPil;
    Add12(deltah,deltal,temp2h,temp3);

    /* Compute now a scaled 1/2 ulp of the intermediate result 
       in the direction of delta 
    */

    if ((x >= 0.0) ^ (deltah >= 0.0)) 
      tempdb.l--;
    else 
      tempdb.l++;

    miulp = TWO999 * (tempdb.d - asinhdb.d);

    /* We must correct the intermediate rounding 
       if the error on deltah + deltal is greater
       in absolute value than miulp = 1/2 ulp in the
       right direction.
    */

    if (ABS(deltah) < ABS(miulp)) {
      /* deltah is less than miulp, deltal is less than 
	 1/2 the ulp of deltah. Thus deltah + deltal is 
	 less than miulp. We do not need to correct 
	 the intermediate rounding.
      */
      return asinhdb.d;
    }

    if (ABS(deltah) > ABS(miulp)) {
      /* deltah is greater than miulp and deltal cannot correct it.
	 We must correct the rounding.

	 tempdb.d (= asinhdb.d +/- 1 ulp) is the correct rounding.
      */
      return tempdb.d;
    }

    /* Here deltah and miulp are equal in absolute value 

       We must correct the intermediate rounding iff the sign of deltal 
       and deltah are the same.
       
    */

    if ((deltah >= 0.0) ^ (deltal >= 0.0)) {
      /* Here the sign is different, we return the 
	 intermediate rounding asinhdb.d 
      */
      return asinhdb.d;
    }
    
    /* Return the corrected result tempdb.d */

    return tempdb.d;
  }

  /* asinpi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.5;
    }
    if (x == -1.0) {
      return - 0.5;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

    /* Rounding test */

    if(asinpih == (asinpih + (asinpim * RNROUNDCST))) 
      return asinpih;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */

    ReturnRoundToNearest3(asinpih,asinpim,asinpil);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Mul22(&t2h,&t2m,t1h,t1m,RECPRPIH,RECPRPIM);

    Add122(&asinpih,&asinpim,0.5,t2h,t2m);

    /* Rounding test */

    if(asinpih == (asinpih + (asinpim * RNROUNDCST))) 
      return sign * asinpih;

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Mul33(&t2h,&t2m,&t2l,t1h,t1m,t1l,RECPRPIH,RECPRPIM,RECPRPIL);

    Add133(&asinpihover,&asinpimover,&asinpilover,0.5,t2h,t2m,t2l);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */    

    RoundToNearest3(&asinpi,asinpih,asinpim,asinpil);

    return sign * asinpi;

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

  /* Rounding test */
  
  if(asinpih == (asinpih + (asinpim * RNROUNDCST))) 
    return sign * asinpih;
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);
  
  Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

  Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

  /* Final rounding */
  
  RoundToNearest3(&asinpi,asinpih,asinpim,asinpil);
  
  return sign * asinpi;
  
}

double asinpi_rd(double x) {
  db_number xdb, zdb, asinhdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double asinpihover, asinpimover, asinpilover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double asinpi;
  double xabs;
  double xScaled;
  double xPih, xPim, xPil;
  double xPihover, xPimover, xPilover;
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
  db_number tempdb;
#endif
  double deltatemp, deltah, deltal;
  double temp1, temp2h, temp2l, temp3;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-60) we have
     
     arcsin(x)/pi = x * triple-double(1/pi) * ( 1 + xi ) 

     with 0 <= xi < 2^(-122)

     We have no bad case worser than 
     112 bits for asinpi(x) for |x| > 2^(-58)
     and the order 3 term of asinpi is still more
     far away. 
          
  */
  if (xdb.i[HI] < ASINPISIMPLEBOUND) {
    /* For a faster path for the exact case, 
       we check first if x = 0
    */

    if (x == 0.0) 
      return x;

    /* We want a relatively fast path for values where
       neither the input nor the output is subnormal 
       because subnormal rounding is expensive.

       Since 
       
       abs(double(asin(x)/pi)) <= 2^(-860) 
       
       for abs(x) <= 2^(-858), we filter for this (normal) value.

    */

    if (xdb.i[HI] >= ASINPINOSUBNORMALBOUND) {

      /* Here abs(x) >= 2^(-858).
	 The result is therefore clearly normal and
	 the double precision numbers in the triple-double
	 representation of TD(1/pi) * x are all normal, too.

	 For speed, we use a two-step approach.
	 We know that 

	 || x*DD(1/pi)/(asin(x)/pi) - 1 ||_[-2^(-60);2^(-60)]^\infty <= 2^(-107.8) <= 2^(-80) 

      */

      Mul122(&xPih,&xPim,x,RECPRPIH,RECPRPIM);

      TEST_AND_RETURN_RD(xPih, xPim, RDROUNDCSTASINPI);
      
      Mul133(&xPihover,&xPimover,&xPilover,x,RECPRPIH,RECPRPIM,RECPRPIL);

      Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);
      
      ReturnRoundDownwards3(xPih,xPim,xPil);

    } 

    /* Here abs(x) < 2^(-858)

       Because of subnormals and especially because
       of the fact that 1/pi < 1, we must scale x
       appropriately. We compute hence:

       asinpi(x) = round( ((x * 2^(1000)) * triple-double(1/pi)) * 2^(-1000))

       where the rounding procedure works temporarily on the scaled 
       intermediate.
    */

    xScaled = x * TWO1000;

    Mul133(&xPihover,&xPimover,&xPilover,xScaled,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);

    /* Produce a (possibly) subnormal intermediate rounding */

    asinhdb.d = xPih * TWOM1000;

#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
    tempdb.i[HI] = asinhdb.i[HI];
    tempdb.i[LO] = asinhdb.i[LO];
    asinhdb.i[LO] = tempdb.i[LO];
    asinhdb.i[HI] = tempdb.i[HI];
#endif


    /* Rescale the result */

    temp1 = asinhdb.d * TWO1000;

    /* Compute the scaled error, the operation is exact by Sterbenz' lemma */

    deltatemp = xPih - temp1;

    /* Sum up the errors, representing them on a double-double
       
       This is exact for the normal rounding case and the error
       is neglectable in the subnormal rounding case.
    */

    Add12Cond(temp2h,temp2l,deltatemp,xPim);
    temp3 = temp2l + xPil;
    Add12(deltah,deltal,temp2h,temp3);

    /* We are doing directed rounding. Thus we must correct the rounding
       if the sign of the error is not correct 
       RD -> sign must be positive for correct rounding
       RU -> sign must be negative for correct rounding
       RZ -> sign must be positive for positive x and negative for negative x
    */

    if (deltah >= 0.0) {
      /* The sign is correct, return the intermediate rounding */
      return asinhdb.d;
    }
   
    /* The sign is not correct

       RD -> subtract 1 ulp
       RU -> add 1 ulp
       RZ -> subtract 1 ulp if x positive, add 1 ulp if x negative
    */

    if (x < 0.0) 
      asinhdb.l++;
    else 
      asinhdb.l--;

    return asinhdb.d;
  }

  /* asinpi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.5;
    }
    if (x == -1.0) {
      return - 0.5;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

    /* Rounding test */

    TEST_AND_RETURN_RD(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */

    ReturnRoundDownwards3(asinpih,asinpim,asinpil);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Mul22(&t2h,&t2m,t1h,t1m,RECPRPIH,RECPRPIM);

    Add122(&asinpih,&asinpim,0.5,t2h,t2m);

    /* Rounding test */

    asinpih *= sign;
    asinpim *= sign;

    TEST_AND_RETURN_RD(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Mul33(&t2h,&t2m,&t2l,t1h,t1m,t1l,RECPRPIH,RECPRPIM,RECPRPIL);

    Add133(&asinpihover,&asinpimover,&asinpilover,0.5,t2h,t2m,t2l);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */    

    RoundDownwards3(&asinpi,asinpih,asinpim,asinpil);

    return sign * asinpi;

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

  /* Rounding test */

  asinpih *= sign;
  asinpim *= sign;
  
  TEST_AND_RETURN_RD(asinpih, asinpim, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);
  
  Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

  Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

  /* Final rounding */
  
  RoundDownwards3(&asinpi,asinpih,asinpim,asinpil);
  
  return sign * asinpi;
  
}

double asinpi_ru(double x) {
  db_number xdb, zdb, asinhdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double asinpihover, asinpimover, asinpilover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double asinpi;
  double xabs;
  double xScaled;
  double xPih, xPim, xPil;
  double xPihover, xPimover, xPilover;
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
  db_number tempdb;
#endif
  double deltatemp, deltah, deltal;
  double temp1, temp2h, temp2l, temp3;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-60) we have
     
     arcsin(x)/pi = x * triple-double(1/pi) * ( 1 + xi ) 

     with 0 <= xi < 2^(-122)

     We have no bad case worser than 
     112 bits for asinpi(x) for |x| > 2^(-58)
     and the order 3 term of asinpi is still more
     far away. 
          
  */
  if (xdb.i[HI] < ASINPISIMPLEBOUND) {
    /* For a faster path for the exact case, 
       we check first if x = 0
    */

    if (x == 0.0) 
      return x;

    /* We want a relatively fast path for values where
       neither the input nor the output is subnormal 
       because subnormal rounding is expensive.

       Since 
       
       abs(double(asin(x)/pi)) <= 2^(-860) 
       
       for abs(x) <= 2^(-858), we filter for this (normal) value.

    */

    if (xdb.i[HI] >= ASINPINOSUBNORMALBOUND) {

      /* Here abs(x) >= 2^(-858).
	 The result is therefore clearly normal and
	 the double precision numbers in the triple-double
	 representation of TD(1/pi) * x are all normal, too.

	 For speed, we use a two-step approach.
	 We know that 

	 || x*DD(1/pi)/(asin(x)/pi) - 1 ||_[-2^(-60);2^(-60)]^\infty <= 2^(-107.8) <= 2^(-80) 

      */

      Mul122(&xPih,&xPim,x,RECPRPIH,RECPRPIM);

      TEST_AND_RETURN_RU(xPih, xPim, RDROUNDCSTASINPI);
      
      Mul133(&xPihover,&xPimover,&xPilover,x,RECPRPIH,RECPRPIM,RECPRPIL);

      Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);
      
      ReturnRoundUpwards3(xPih,xPim,xPil);

    } 

    /* Here abs(x) < 2^(-858)

       Because of subnormals and especially because
       of the fact that 1/pi < 1, we must scale x
       appropriately. We compute hence:

       asinpi(x) = round( ((x * 2^(1000)) * triple-double(1/pi)) * 2^(-1000))

       where the rounding procedure works temporarily on the scaled 
       intermediate.
    */

    xScaled = x * TWO1000;

    Mul133(&xPihover,&xPimover,&xPilover,xScaled,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);

    /* Produce a (possibly) subnormal intermediate rounding */

    asinhdb.d = xPih * TWOM1000;

#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
    tempdb.i[HI] = asinhdb.i[HI];
    tempdb.i[LO] = asinhdb.i[LO];
    asinhdb.i[LO] = tempdb.i[LO];
    asinhdb.i[HI] = tempdb.i[HI];
#endif

    /* Rescale the result */

    temp1 = asinhdb.d * TWO1000;

    /* Compute the scaled error, the operation is exact by Sterbenz' lemma */

    deltatemp = xPih - temp1;

    /* Sum up the errors, representing them on a double-double
       
       This is exact for the normal rounding case and the error
       is neglectable in the subnormal rounding case.
    */

    Add12Cond(temp2h,temp2l,deltatemp,xPim);
    temp3 = temp2l + xPil;
    Add12(deltah,deltal,temp2h,temp3);

    /* We are doing directed rounding. Thus we must correct the rounding
       if the sign of the error is not correct 
       RD -> sign must be positive for correct rounding
       RU -> sign must be negative for correct rounding
       RZ -> sign must be positive for positive x and negative for negative x
    */

    if (deltah <= 0.0) {
      /* The sign is correct, return the intermediate rounding */
      return asinhdb.d;
    }
   
    /* The sign is not correct

       RD -> subtract 1 ulp
       RU -> add 1 ulp
       RZ -> subtract 1 ulp if x positive, add 1 ulp if x negative
    */

    if (x < 0.0) 
      asinhdb.l--;
    else 
      asinhdb.l++;

    return asinhdb.d;
  }

  /* asinpi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.5;
    }
    if (x == -1.0) {
      return - 0.5;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

    /* Rounding test */

    TEST_AND_RETURN_RU(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */

    ReturnRoundUpwards3(asinpih,asinpim,asinpil);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Mul22(&t2h,&t2m,t1h,t1m,RECPRPIH,RECPRPIM);

    Add122(&asinpih,&asinpim,0.5,t2h,t2m);

    /* Rounding test */

    asinpih *= sign;
    asinpim *= sign;

    TEST_AND_RETURN_RU(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Mul33(&t2h,&t2m,&t2l,t1h,t1m,t1l,RECPRPIH,RECPRPIM,RECPRPIL);

    Add133(&asinpihover,&asinpimover,&asinpilover,0.5,t2h,t2m,t2l);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */    

    RoundUpwards3(&asinpi,asinpih,asinpim,asinpil);

    return sign * asinpi;

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

  /* Rounding test */

  asinpih *= sign;
  asinpim *= sign;
  
  TEST_AND_RETURN_RU(asinpih, asinpim, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);
  
  Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

  Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

  /* Final rounding */
  
  RoundUpwards3(&asinpi,asinpih,asinpim,asinpil);
  
  return sign * asinpi;
  
}

double asinpi_rz(double x) {
  db_number xdb, zdb, asinhdb;
  double sign, z, zp;
  int index;
  double asinh, asinm, asinl;
  double asinpih, asinpim, asinpil;
  double asinpihover, asinpimover, asinpilover;
  double p9h, p9m, p9l, sqrh, sqrm, sqrl;
  double t1h, t1m, t1l;
  double t2h, t2m, t2l;
  double asinpi;
  double xabs;
  double xScaled;
  double xPih, xPim, xPil;
  double xPihover, xPimover, xPilover;
#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
  db_number tempdb;
#endif
  double deltatemp, deltah, deltal;
  double temp1, temp2h, temp2l, temp3;

  /* Start already computations for argument reduction */

  zdb.d = 1.0 + x * x;

  xdb.d = x;

  /* Special case handling */
  
  /* Remove sign of x in floating-point */
  xabs = ABS(x);
  xdb.i[HI] &= 0x7fffffff;

  /* If |x| < 2^(-60) we have
     
     arcsin(x)/pi = x * triple-double(1/pi) * ( 1 + xi ) 

     with 0 <= xi < 2^(-122)

     We have no bad case worser than 
     112 bits for asinpi(x) for |x| > 2^(-58)
     and the order 3 term of asinpi is still more
     far away. 
          
  */
  if (xdb.i[HI] < ASINPISIMPLEBOUND) {
    /* For a faster path for the exact case, 
       we check first if x = 0
    */

    if (x == 0.0) 
      return x;

    /* We want a relatively fast path for values where
       neither the input nor the output is subnormal 
       because subnormal rounding is expensive.

       Since 
       
       abs(double(asin(x)/pi)) <= 2^(-860) 
       
       for abs(x) <= 2^(-858), we filter for this (normal) value.

    */

    if (xdb.i[HI] >= ASINPINOSUBNORMALBOUND) {

      /* Here abs(x) >= 2^(-858).
	 The result is therefore clearly normal and
	 the double precision numbers in the triple-double
	 representation of TD(1/pi) * x are all normal, too.

	 For speed, we use a two-step approach.
	 We know that 

	 || x*DD(1/pi)/(asin(x)/pi) - 1 ||_[-2^(-60);2^(-60)]^\infty <= 2^(-107.8) <= 2^(-80) 

      */

      Mul122(&xPih,&xPim,x,RECPRPIH,RECPRPIM);

      TEST_AND_RETURN_RZ(xPih, xPim, RDROUNDCSTASINPI);
      
      Mul133(&xPihover,&xPimover,&xPilover,x,RECPRPIH,RECPRPIM,RECPRPIL);

      Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);
      
      ReturnRoundTowardsZero3(xPih,xPim,xPil);

    } 

    /* Here abs(x) < 2^(-858)

       Because of subnormals and especially because
       of the fact that 1/pi < 1, we must scale x
       appropriately. We compute hence:

       asinpi(x) = round( ((x * 2^(1000)) * triple-double(1/pi)) * 2^(-1000))

       where the rounding procedure works temporarily on the scaled 
       intermediate.
    */

    xScaled = x * TWO1000;

    Mul133(&xPihover,&xPimover,&xPilover,xScaled,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&xPih,&xPim,&xPil,xPihover,xPimover,xPilover);

    /* Produce a (possibly) subnormal intermediate rounding */

    asinhdb.d = xPih * TWOM1000;

#if defined(CRLIBM_TYPECPU_AMD64) || defined(CRLIBM_TYPECPU_X86) 
    tempdb.i[HI] = asinhdb.i[HI];
    tempdb.i[LO] = asinhdb.i[LO];
    asinhdb.i[LO] = tempdb.i[LO];
    asinhdb.i[HI] = tempdb.i[HI];
#endif

    /* Rescale the result */

    temp1 = asinhdb.d * TWO1000;

    /* Compute the scaled error, the operation is exact by Sterbenz' lemma */

    deltatemp = xPih - temp1;

    /* Sum up the errors, representing them on a double-double
       
       This is exact for the normal rounding case and the error
       is neglectable in the subnormal rounding case.
    */

    Add12Cond(temp2h,temp2l,deltatemp,xPim);
    temp3 = temp2l + xPil;
    Add12(deltah,deltal,temp2h,temp3);

    /* We are doing directed rounding. Thus we must correct the rounding
       if the sign of the error is not correct 
       RD -> sign must be positive for correct rounding
       RU -> sign must be negative for correct rounding
       RZ -> sign must be positive for positive x and negative for negative x
    */

    if ((x > 0.0) ^ (deltah < 0.0)) {
      /* The sign is correct, return the intermediate rounding */
      return asinhdb.d;
    }
   
    /* The sign is not correct

       RD -> subtract 1 ulp
       RU -> add 1 ulp
       RZ -> subtract 1 ulp if x positive, add 1 ulp if x negative
    */

    asinhdb.l--;

    return asinhdb.d;
  }

  /* asinpi is defined on -1 <= x <= 1, elsewhere it is NaN */
  if (xdb.i[HI] >= 0x3ff00000) {
    if (x == 1.0) {
      return 0.5;
    }
    if (x == -1.0) {
      return - 0.5;
    }
    return (x-x)/0.0;    /* return NaN */
  }

  /* Argument reduction:

     We have 10 intervals and 3 paths:

     - interval 0   => path 1 using p0
     - interval 1-8 => path 2 using p
     - interval 9   => path 3 using p9

  */

  index = (0x000f0000 & zdb.i[HI]) >> 16;

  /* 0 <= index <= 15 

     index approximates roughly x^2 

     Map indexes to intervals as follows:

     0  -> 0 
     1  -> 1
     ... 
     8  -> 8
     9  -> 9
     ... 
     15 -> 9

     For this mapping, filter first the case 0 -> 0
     In consequence, 1 <= index <= 15, i.e. 
     0 <= index - 1 <= 14 with the mapping index - 1 -> interval as

     0  -> 1
     ... 
     7  -> 8
     8  -> 9
     ...
     15 -> 9

     Thus it suffices to check the 3rd bit of index - 1 after the first filter.
     
  */

  if (index == 0) {
    /* Path 1 using p0 */

    p0_quick(&asinh, &asinm, x, xdb.i[HI]);

    Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

    /* Rounding test */

    TEST_AND_RETURN_RZ(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p0_accu(&asinh, &asinm, &asinl, x);

    Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */

    ReturnRoundTowardsZero3(asinpih,asinpim,asinpil);

  } 

  /* Strip off the sign of argument x */
  sign = 1.0;
  if (x < 0.0) sign = -sign;
  
  index--;
  if ((index & 0x8) != 0) {
    /* Path 3 using p9 */

    /* Do argument reduction using a MI_9 as a midpoint value 
       for the polynomial and compute exactly zp = 2 * (1 - x) 
       for the asymptotical approximation using a square root.
    */

    z = xabs - MI_9;
    zp = 2.0 * (1.0 - xabs);

    /* Polynomial approximation and square root extraction */

    p9_quick(&p9h, &p9m, z);
    p9h = -p9h;
    p9m = -p9m;

    sqrt12_64_unfiltered(&sqrh,&sqrm,zp);

    /* Reconstruction */

    Mul22(&t1h,&t1m,sqrh,sqrm,p9h,p9m);
    Mul22(&t2h,&t2m,t1h,t1m,RECPRPIH,RECPRPIM);

    Add122(&asinpih,&asinpim,0.5,t2h,t2m);

    /* Rounding test */

    asinpih *= sign;
    asinpim *= sign;

    TEST_AND_RETURN_RZ(asinpih, asinpim, RDROUNDCST);

    /* Rounding test failed, launch accurate phase */

#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
    
    p9_accu(&p9h, &p9m, &p9l, z);
    p9h = -p9h;
    p9m = -p9m;
    p9l = -p9l;

    Sqrt13(&sqrh,&sqrm,&sqrl,zp);

    /* Reconstruction */

    Mul33(&t1h,&t1m,&t1l,sqrh,sqrm,sqrl,p9h,p9m,p9l);
    Mul33(&t2h,&t2m,&t2l,t1h,t1m,t1l,RECPRPIH,RECPRPIM,RECPRPIL);

    Add133(&asinpihover,&asinpimover,&asinpilover,0.5,t2h,t2m,t2l);

    Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

    /* Final rounding */    

    RoundTowardsZero3(&asinpi,asinpih,asinpim,asinpil);

    return sign * asinpi;

  }

  /* Path 2 using p */

  /* Do argument reduction using a table value for 
     the midpoint value 
  */

  z = xabs - mi_i;

  p_quick(&asinh, &asinm, z, index);

  Mul22(&asinpih,&asinpim,asinh,asinm,RECPRPIH,RECPRPIM);

  /* Rounding test */

  asinpih *= sign;
  asinpim *= sign;
  
  TEST_AND_RETURN_RZ(asinpih, asinpim, RDROUNDCST);
  
  /* Rounding test failed, launch accurate phase */
  
#if EVAL_PERF
  crlibm_second_step_taken++;
#endif
  
  p_accu(&asinh, &asinm, &asinl, z, index);
  
  Mul33(&asinpihover,&asinpimover,&asinpilover,asinh,asinm,asinl,RECPRPIH,RECPRPIM,RECPRPIL);

  Renormalize3(&asinpih,&asinpim,&asinpil,asinpihover,asinpimover,asinpilover);

  /* Final rounding */
  
  RoundTowardsZero3(&asinpi,asinpih,asinpim,asinpil);
  
  return sign * asinpi;
  
}

