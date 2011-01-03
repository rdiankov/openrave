/*
 * Correctly rounded arctangent
 *
 * Author : Nicolas Gast (Ecole Normale Superieure), Florent de Dinechin
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
#include "atan_fast.h"

extern double scs_atan_rn(double); 
extern double scs_atan_rd(double); 
extern double scs_atan_ru(double); 
extern double scs_atanpi_rn(double); 
extern double scs_atanpi_rd(double); 
extern double scs_atanpi_ru(double); 


static void atan_quick(double *atanhi,double *atanlo, int *index_of_e, double x) {

  double tmphi,tmplo, x0hi,x0lo;
  double q,Xred2,x2;
  double Xredhi,Xredlo;
  double xmBihi, xmBilo, tmphi2, tmplo2, atanlolo;
  
  int i;
  
  if (x > MIN_REDUCTION_NEEDED) /* test if reduction is necessary : */
    {
      /*
       * 1) Argument reduction : 
       * 
       *  tan(x) = tan( b(i) ) + tan ( (x-b(i)) / (1+x*b(i)))
       *
       *                                                     6.3
       * we choose 62 b(i) so that (x-b(i)) / (1+x*b(i)) < 2^
       */ 
      

      
      if (x > arctan_table[61][B].d) {
        i=61;
        Add12( xmBihi , xmBilo , x , -arctan_table[61][B].d);
      }
      else 
        {
          /* compute i so that a[i] < x < a[i+1] */
          i=31;
          if (x < arctan_table[i][A].d) i-= 16;
          else i+=16;
          if (x < arctan_table[i][A].d) i-= 8;
          else i+= 8;
          if (x < arctan_table[i][A].d) i-= 4;
          else i+= 4;
          if (x < arctan_table[i][A].d) i-= 2;
          else i+= 2;
          if (x < arctan_table[i][A].d) i-= 1;
          else i+= 1;
          if (x < arctan_table[i][A].d) i-= 1;     
          xmBihi = x-arctan_table[i][B].d;
          xmBilo = 0.0;
        }
        
      /* we now compute Xred = ( x-b[i] ) / ( 1 + x*b[i] )
       * 
       * def : x0 := 1+x*b[i]
       *
       * 1st we compute an approximation of y = 1/x0
       * then we compute a better approx x' = y*(2-x0*y)
       * we can proove that :
       * if y = 1/x0*(1+e) 
       *     then x' = 1/x0 * (1-e^2)
       *                   
       */
      
      Mul12(&tmphi,&tmplo, x, arctan_table[i][B].d);

      if (x > 1)
        Add22(&x0hi,&x0lo,tmphi,tmplo, 1.0,0.0);
      else {Add22( &x0hi , &x0lo , 1.0,0.0,tmphi,tmplo);}

      Div22( &Xredhi, &Xredlo, xmBihi , xmBilo , x0hi,x0lo);

      /* Polynomial evaluation : 
       *  
       *  1rt compute Q(x^2) = (1 - x^2/3 + ...)
       *      then P(x) = x * Q(x^2)
       *
       */

      Xred2 = Xredhi*Xredhi;
      
      q = Xred2*(coef_poly[3]+Xred2*
                 (coef_poly[2]+Xred2*
                  (coef_poly[1]+Xred2*
                   coef_poly[0]))) ;

      /* reconstruction : atan(x) = atan(b[i]) + atan(x) */
      atanlolo = (Xredlo + arctan_table[i][ATAN_BLO].d);
      atanlolo += Xredhi*q;
      Add12( tmphi2, tmplo2, arctan_table[i][ATAN_BHI].d, Xredhi);
      Add12( *atanhi, *atanlo, tmphi2, (tmplo2+atanlolo));
      
      if (i<10)
        *index_of_e = 0;
      else
        *index_of_e = 1;
    }
  else 
    // no reduction needed
    {
      /* Polynomial evaluation : 
       *  
       *  1rt compute Q(x^2) = (1 - x^2/3 + ...)
       *      then P(x) = x * Q(x^2)
       *
       */
      
      x2 = x*x;
      q = x2*(coef_poly[3]+x2*
              (coef_poly[2]+x2*
               (coef_poly[1]+x2*
                coef_poly[0]))) ;
      Add12(*atanhi,*atanlo, x , x*q);
      
      *index_of_e = 2;
    }

}







extern double atan_rn(double x) {
 
  double atanhi,atanlo;
  int index_of_e;
  double sign;
  db_number x_db;
  int absxhi;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7fffffff; 

  if(x_db.i[HI] & 0x80000000){
    x_db.i[HI] = absxhi;
    sign =-1;
  }
  else 
    sign=1;
  
  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else 
        return sign*HALFPI.d;           /* atan(+/-infty) = +/- Pi/2 */
    }
  if ( absxhi < 0x3E400000 )
      return x;                   /* x<2^-27 then atan(x) =~ x */
  
  atan_quick(&atanhi, &atanlo,&index_of_e , x_db.d);
  
  if (atanhi == (atanhi + (atanlo*rncst[index_of_e]))) 
    return sign*atanhi;
  else
    {
      /* more accuracy is needed , lauch accurate phase */ 
      return sign*scs_atan_rn(x_db.d);
    }
}







extern double atan_rd(double x) {
  double atanhi,atanlo;
  int index_of_e;
  double maxepsilon;
  db_number x_db;
  int absxhi;
  int sign;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7FFFFFFF; 

  if(x_db.i[HI] & 0x80000000){
    x_db.i[HI] = absxhi;
    sign =-1;
  }
  else 
    sign=1;

  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else{
	if (sign>0)
	  return HALFPI.d;
	else
	  return -HALFPI_TO_PLUS_INFINITY.d;           /* atan(x) = Pi/2 */
      }
    }
  else
    if ( absxhi < 0x3E400000 )
      {if (sign>0)
        {if(x==0)
	  return x;
        else
          x_db.l--;
        return x_db.d;
        }
      else
        return x;
      }
  
  atan_quick(&atanhi, &atanlo,&index_of_e, x_db.d);
  maxepsilon = epsilon[index_of_e];
  atanhi = sign*atanhi;
  atanlo = sign*atanlo;
  
  /* Rounding test to - infinity */ 
  
  TEST_AND_RETURN_RD(atanhi, atanlo, maxepsilon);

  /* if the previous block didn't return a value, launch accurate phase */
  return scs_atan_rd(sign*x_db.d);
}







extern double atan_ru(double x) {
  double atanhi,atanlo;
  int index_of_e;
  int sign;
  double maxepsilon;
  db_number x_db;
  int absxhi;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7FFFFFFF; 

  if (x_db.i[HI] & 0x80000000){
    sign = -1;
    x_db.i[HI] = absxhi;
  }
  else 
    sign = 1;
  
  
  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else
        {
          if (sign>0)
            return HALFPI_TO_PLUS_INFINITY.d;
        else
          return -HALFPI.d;           /* atan(x) = Pi/2 */
        }
    }
    
  if ( absxhi < 0x3E400000 ){
    if(x==0)
      return x;
    
    if (sign<0) {
      x_db.l--;
      return -x_db.d;
    }
    else
      return x;
  }                   /* x<2^-27 then atan(x) =~ x */
  
  atan_quick(&atanhi, &atanlo, &index_of_e, x_db.d);
  maxepsilon = epsilon[index_of_e];
  atanhi = sign*atanhi;
  atanlo = sign*atanlo;
  
  TEST_AND_RETURN_RU(atanhi, atanlo, maxepsilon);

  /* if the previous block didn't return a value, launch accurate phase */
  return scs_atan_ru(x);
}





extern double atan_rz(double x) {
  if (x>0)
    return atan_rd(x);
  else
    return atan_ru(x);
}




/*************************************************************
 *************************************************************
 *                       AtanPi  		             *
 *************************************************************
 *************************************************************/



extern double atanpi_rn(double x) {
 
  double atanhi,atanlo,atanpihi,atanpilo;
  int index_of_e;
  double sign;
  db_number x_db;
  int absxhi;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7fffffff; 

  if(x_db.i[HI] & 0x80000000){
    x_db.i[HI] = absxhi;
    sign =-1;
  }
  else 
    sign=1;
  
  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else 
        return sign*0.5;           /* atan(+/-infty) = +/- Pi/2 */
    }
  if ( absxhi < 0x3E400000 )
    return sign*scs_atanpi_rn(x_db.d); /* TODO optim here */
  
  atan_quick(&atanhi, &atanlo,&index_of_e , x_db.d);
  Mul22(&atanpihi,&atanpilo, INVPIH, INVPIL, atanhi,atanlo);

  if (atanpihi == (atanpihi + (atanpilo*rncst[index_of_e]))) 
    return sign*atanpihi;
  else
    /* more accuracy is needed , lauch accurate phase */ 
    return sign*scs_atanpi_rn(x_db.d);
}







extern double atanpi_rd(double x) {
  double atanhi,atanlo,atanpihi,atanpilo;
  int index_of_e;
  double maxepsilon;
  db_number x_db;
  int absxhi;
  int sign;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7FFFFFFF; 

  if(x_db.i[HI] & 0x80000000){
    x_db.i[HI] = absxhi;
    sign =-1;
  }
  else 
    sign=1;

  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else{
	if (sign>0)
	  return 0.5; 
	/* Or should it be 0.4999999999999999444888487687421729788184165954589843750; nextdown(0.5) */
	else
	  return -0.5;           /* atan(infty) = Pi/2 */
      }
    }
  else
    if ( absxhi < 0x3E400000 ) {
      if(x==0.0) 
	return x;  /* signed */
      else
	return scs_atanpi_rd(sign*x_db.d); /* TODO optim here */
    }
  atan_quick(&atanhi, &atanlo,&index_of_e, x_db.d);
  Mul22(&atanpihi,&atanpilo, INVPIH, INVPIL, atanhi,atanlo);
  maxepsilon = epsilon[index_of_e];
  atanpihi = sign*atanpihi;
  atanpilo = sign*atanpilo;
  
  /* Rounding test to - infinity */ 
  
  TEST_AND_RETURN_RD(atanpihi, atanpilo, maxepsilon);

  /* if the previous block didn't return a value, launch accurate phase */
  return scs_atanpi_rd(sign*x_db.d);
}







extern double atanpi_ru(double x) {
  double atanhi,atanlo,atanpihi,atanpilo;
  int index_of_e;
  int sign;
  double maxepsilon;
  db_number x_db;
  int absxhi;

  x_db.d = x;
  absxhi = x_db.i[HI] & 0x7FFFFFFF; 

  if (x_db.i[HI] & 0x80000000){
    sign = -1;
    x_db.i[HI] = absxhi;
  }
  else 
    sign = 1;
  
  
  /* Filter cases */
  if ( absxhi >= 0x43500000)           /* x >= 2^54 */
    {
      if ((absxhi > 0x7ff00000) || ((absxhi == 0x7ff00000) && (x_db.i[LO] != 0)))
        return x+x;                /* NaN */
      else
        {
          if (sign>0)
            return 0.5;
        else
          return - 0.5;           
        }
    }
    
    if ( absxhi < 0x3E400000 ) {
      if(x==0.0)
	return x; /* signed */
      else
	return scs_atanpi_ru(x);
    }  
  atan_quick(&atanhi, &atanlo, &index_of_e, x_db.d);
  Mul22(&atanpihi,&atanpilo, INVPIH, INVPIL, atanhi,atanlo);
  maxepsilon = epsilon[index_of_e];
  atanpihi = sign*atanpihi;
  atanpilo = sign*atanpilo;
  
  TEST_AND_RETURN_RU(atanpihi, atanpilo, maxepsilon);

  /* if the previous block didn't return a value, launch accurate phase */
  return scs_atanpi_ru(x);
}





extern double atanpi_rz(double x) {
  if (x>0)
    return atanpi_rd(x);
  else
    return atanpi_ru(x);
}

