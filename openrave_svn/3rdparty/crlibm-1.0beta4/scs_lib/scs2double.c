/** Conversion of SCS to floating-point double 
@file scs2double.c

@author Defour David David.Defour@ens-lyon.fr
@author Florent de Dinechin Florent.de.Dinechin@ens-lyon.fr 

This file is part of the SCS library.
*/

/*
Copyright (C) 2002  David Defour and Florent de Dinechin

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */
#include "scs.h"
#include "scs_private.h"


/** Convert a multiple precision number in scs format into a double
 precision number.

@warning  "x" need to be normalized
  */ 

/* TODO BUG scs_get_d doesn't do round-to nearest even */




/*  computes the exponent from the index */
/* in principle an inline function would be cleaner, but 
 this leads to faster and smaller code 
*/





void scs_get_d(double *result, scs_ptr x){ 
  db_number nb, rndcorr;
  uint64_t lowpart, roundbits, t1;
  int expo, expofinal, shift;
  double res;
  
  /* convert the MSB digit into a double, and store it in nb.d */
  nb.d = (double)X_HW[0]; 

  /* place the two next digits in lowpart */
  t1   = X_HW[1];
  lowpart  = (t1 << SCS_NB_BITS) + X_HW[2];
    /* there is at least one significant bit in nb, 
       and at least 2*SCS_NB_BITS in lowpart, 
       so provided SCS_NB_BITS >= 27
       together they include the 53+ guard bits to decide rounding 
    */

  /* test for  s/qNan, +/- Inf, +/- 0, placed here for obscure performance reasons */
  if (X_EXP != 1){
    *result = X_EXP; 
    return;
  }
  
  /* take the exponent of nb.d (will be in [0:SCS_NB_BITS])*/
  expo = ((nb.i[HI] & 0x7ff00000)>>20) - 1023; 

  /* compute the exponent of the result */
  expofinal = expo + SCS_NB_BITS*X_IND;

  /* Is the SCS number not too large for the IEEE exponent range ? */
  if (expofinal >  1023) {
    /* return an infinity */
    res = SCS_RADIX_RNG_DOUBLE*SCS_RADIX_RNG_DOUBLE;
  }

  /* Is our SCS number a denormal  ? */
  else if (expofinal >= -1022){		
    /* x is in the normal range */   
    shift=expo+2*SCS_NB_BITS-53;
    roundbits=lowpart<<(64-shift);
    /* align the rest of the mantissa to nb : shift by (2*SCS_NB_BITS)-53+expo */
    lowpart = lowpart >> shift;     
    /* Look at the last bit to decide rounding */

    if (lowpart & ULL(0000000000000001)){
      /* Test for the round to even case */
      if(roundbits==0)
	{ int i;
	  for (i=3; i<SCS_NB_WORDS; i++)
	    roundbits=roundbits | X_HW[i];
	}
      if(roundbits==0) {
	/* round to even mantissa */
	if (lowpart & ULL(0000000000000002)){
	  /* mantissa odd, need to add an half-ulp */
	  rndcorr.i[LO] = 0; 
	  rndcorr.i[HI] = (expo-52+1023)<<20;    /* 2^(exp-52) */ 
	}else
	  rndcorr.d = 0.0;
      }
      else { /* there are round bits need to add an half-ulp */
	rndcorr.i[LO] = 0; 
	rndcorr.i[HI] = (expo-52+1023)<<20;    /* 2^(exp-52) */ 
      }
    }else{
      /* need to add nothing*/
      rndcorr.d = 0.0;
    }
    
    lowpart = lowpart >> 1;
    nb.l = nb.l | lowpart;    /* Finish to fill the mantissa */
    res  = nb.d + rndcorr.d;  /* rounded to nearest   */
    
    /* now compute the exponent from the index :
       we need to multiply res by 2^(X_IND*SCS_NB_BITS)
       First check this number won't be a denormal itself */
    if((X_IND)*SCS_NB_BITS +1023 > 0) {
      /* build the double 2^(X_IND*SCS_NB_BITS)   */         		
      nb.i[HI] = ((X_IND)*SCS_NB_BITS +1023)  << 20;  		
      nb.i[LO] = 0;
      res *= nb.d;     /* exact multiplication */
    }
    else { /*offset the previous computation by 2^(2*SCS_NB_BITS) */
      /* build the double 2^(X_IND*SCS_NB_BITS)   */         		
      nb.i[HI] = ((X_IND)*SCS_NB_BITS +1023 + 2*SCS_NB_BITS)  << 20;  		
      nb.i[LO] = 0;                                 
      res *= SCS_RADIX_MTWO_DOUBLE;  /* exact multiplication */
      res *= nb.d;                  /* exact multiplication */
    }
  } 


  else { 
    /* the final number is a denormal with 52-(expfinal+1022)
     significant bits. */

    if (expofinal < -1022 - 53 ) {
      res = 0.0;
    }
    else {

      /* align the rest of the mantissa to nb */
      lowpart = lowpart >> (expo+(2*SCS_NB_BITS)-52);     
      /* Finish to fill the mantissa */
      nb.l = nb.l | lowpart; 

      /* this is still a normal number. 
	 Now remove its exponent and add back the implicit one */
      nb.l = (nb.l & ULL(000FFFFFFFFFFFFF)) | ULL(0010000000000000);
      
      /* keep only the significant bits */
      nb.l = nb.l >> (-1023 - expofinal);
      /* Look at the last bit to decide rounding */
      if (nb.i[LO] & 0x00000001){
	/* need to add an half-ulp */
	rndcorr.l = 1;    /* this is a full ulp but we multiply by 0.5 in the end */ 
      }else{
	/* need to add nothing*/
	rndcorr.d = 0.0;

      }
      res  = 0.5*(nb.d + rndcorr.d);  /* rounded to nearest   */
      
      /* the exponent field is already set to zero so that's all */
    }          
  } 

  /* sign management */                                                 
  if (X_SGN < 0)                                                        
    *result = - res;                                                    
  else                                                                  
    *result = res;
}





/* All the directed roundings boil down to the same computation, which
is: first build the truncated mantissa.  if the SCS number is exactly
a double precision number, return that. Otherwise, either return the
truncated mantissa, or return this mantissa plus an ulp, rounded to
the nearest. Plus handle the infinities and denormals.
*/

static void get_d_directed(double *result, scs_ptr x, int rndMantissaUp){ 
  db_number nb, rndcorr;
  uint64_t lowpart, t1;
  int expo,expofinal,i, not_null;
  double res;
  
  /* convert the MSB digit into a double, and store it in nb.d */
  nb.d = (double)X_HW[0]; 

  /* place the two next digits in lowpart */
  t1   = X_HW[1];
  lowpart  = (t1 << SCS_NB_BITS) + X_HW[2];

  /* test for  s/qNan, +/- Inf, +/- 0, placed here for obscure performance reasons */
  if (X_EXP != 1){
    *result = X_EXP; 
    return;
  }
  
  /* take the exponent of nb.d (will be in [0:SCS_NB_BITS])*/
  expo = ((nb.i[HI] & 0x7ff00000)>>20) - 1023; 
  not_null = ((lowpart << (64+52 - 2*SCS_NB_BITS - expo)) != 0 );      
  /* Test if we are not on an exact double precision number */        
  for (i=3; i<SCS_NB_WORDS; i++)                                      
    if (X_HW[i]!=0)  not_null = 1;                                  

  /* compute the exponent of the result */
  expofinal = expo + SCS_NB_BITS*X_IND;

  /* Is the SCS number not too large for the IEEE exponent range ? */
  if (expofinal >  1023) {
    if (rndMantissaUp) 
      /* return an infinity */
      res = SCS_RADIX_RNG_DOUBLE*SCS_RADIX_RNG_DOUBLE;
    else
      /* infinity, rounded down, is SCS_MAX_DOUBLE */
      res = SCS_MAX_DOUBLE;
  }

  /* Is our SCS number a denormal  ? */
  else if (expofinal >= -1022){		
    /* x is in the normal range */   

    /* align the rest of the mantissa to nb : shift by (2*SCS_NB_BITS)-53-exp */
    lowpart = lowpart >> (expo+(2*SCS_NB_BITS)-52);     
    /* Finish to fill the mantissa */                                   
    nb.l = nb.l | lowpart;                                              
    if (rndMantissaUp && (not_null)){                                   
      rndcorr.i[LO] = 0;                                         
      rndcorr.i[HI] = (expo-52+1023)<<20;    /* 2^(exp-52) */     
    } else {                                                            
      rndcorr.d = 0.0;                                                
    }                                                                   
    res  = nb.d + rndcorr.d;  /*  rounded to nearest   */         
    
    /* now compute the exponent from the index :
       we need to multiply res by 2^(X_IND*SCS_NB_BITS)
       First check this number won't be a denormal itself */
    if((X_IND)*SCS_NB_BITS +1023 > 0) {
      /* build the double 2^(X_IND*SCS_NB_BITS)   */         		
      nb.i[HI] = ((X_IND)*SCS_NB_BITS +1023)  << 20;  		
      nb.i[LO] = 0;
      res *= nb.d;     /* exact multiplication */
    }
    else { /*offset the previous computation by 2^(2*SCS_NB_BITS) */
      /* build the double 2^(X_IND*SCS_NB_BITS)   */         		
      nb.i[HI] = ((X_IND)*SCS_NB_BITS +1023 + 2*SCS_NB_BITS)  << 20;  		
      nb.i[LO] = 0;                                 
      res *= SCS_RADIX_MTWO_DOUBLE;  /* exact multiplication */
      res *= nb.d;                  /* exact multiplication */
    }
  } 
  

  else { 
    /* the final number is a denormal with 52-(expfinal+1022)
       significant bits. */

    if (expofinal < -1022 - 53 ) {
      if(rndMantissaUp)
	res = SCS_MIN_DOUBLE;
      else
	res = 0.0;
    }
    else {

      /* align the rest of the mantissa to nb */
      lowpart = lowpart >> (expo+(2*SCS_NB_BITS)-52);     
      /* Finish to fill the mantissa */
      nb.l = nb.l | lowpart; 

      /* this is still a normal number. 
	 Now remove its exponent and add back the implicit one */
      nb.l = (nb.l & ULL(000FFFFFFFFFFFFF)) | ULL(0010000000000000);
      
      if (rndMantissaUp && (not_null)){
	nb.l = nb.l >> (-1022 - expofinal);
	nb.l = nb.l +1; /* works even if we move back into the normals*/
      }
      else
	/* keep only the significant bits */
	nb.l = nb.l >> (-1022 - expofinal);

      res  = nb.d;
      
      /* the exponent field is already set to zero so that's all */
    }          
  } 

  /* sign management */                                                 
  if (X_SGN < 0)                                                        
    *result = - res;                                                    
  else                                                                  
    *result = res;
}

#if 0
void get_d_directed0(double *result, scs_ptr x,int rndMantissaUp)                                 
{                                                                     
  uint64_t lowpart, t1;                                 
  db_number nb, rndcorr;                                          
  int i, expo, not_null;                                               
  double res;                                                         
  /* convert the MSB digit into a double, and store it in nb.d */     
  nb.d = (double)X_HW[0];                                             
  /* place the two next digits in lowpart */                          
  t1   = X_HW[1];                                                     
  lowpart  = (t1 << SCS_NB_BITS) + X_HW[2];                           
  /* s/qNan, +/- Inf, +/- 0 */                                        
  if (X_EXP != 1){                                                    
    *result = X_EXP;                                                  
    return;                                                           
  }                                                                   
  /* take the exponent */                                             
  expo = ((nb.i[HI] & 0x7ff00000)>>20) - 1023;                  
  not_null = ((lowpart << (64+52 - 2*SCS_NB_BITS - expo)) != 0 );      
  /* align the rest of the mantissa  */                               
  lowpart = lowpart >> (expo + 2*SCS_NB_BITS - 52);                    
  /* Finish to fill the mantissa */                                   
  nb.l = nb.l | lowpart;                                              
  /* Test if we are not on an exact double precision number */        
  for (i=3; i<SCS_NB_WORDS; i++)                                      
      if (X_HW[i]!=0)  not_null = 1;                                  
  if (rndMantissaUp && (not_null)){                                   
    rndcorr.i[LO] = 0;                                         
    rndcorr.i[HI] = (expo-52+1023)<<20;    /* 2^(exp-52) */     
  } else {                                                            
      rndcorr.d = 0.0;                                                
  }                                                                   
  res  = nb.d + rndcorr.d;  /* make a rounded to nearest   */         
  if ((X_IND < SCS_MAX_RANGE) && (X_IND > -SCS_MAX_RANGE)){	      
    /* x is comfortably in the double-precision range */   	      
    /* build the double 2^(X_IND*SCS_NB_BITS)   */         	      
    nb.i[HI] = ((X_IND)*SCS_NB_BITS +1023)  << 20;  	      
    nb.i[LO] = 0;                                   	      
    res *= nb.d;                                           	      
  }else {                                                  	      
    /* x may end up being a denormal or overflow */        	      
    i    = X_IND;                                          	      
    nb.d = 0;                                              	      
    if (X_IND > 0){                                        	      
      /* one of the following computations may lead to an overflow */ 
      res *=SCS_RADIX_RNG_DOUBLE; /* 2^(SCS_NB_BITS.SCS_MAX_RANGE) */ 
      i   -= SCS_MAX_RANGE;                                           
      while((i-->0)&&(res <= SCS_MAX_DOUBLE)) {                           
	/* second test means: This loop stops on overflow */          
	res *= SCS_RADIX_ONE_DOUBLE;                                  
      }                                                               
    }else {                                                           
      /* One of the computations may lead to denormal/underflow */    
      res *=SCS_RADIX_MRNG_DOUBLE; /* 2^-(SCS_NB_BITS.SCS_MAX_RANGE)*/
      i   += SCS_MAX_RANGE;                                           
      while((i++<0)&&(res != 0)) {                                    
	res *=SCS_RADIX_MONE_DOUBLE;                                  
      }                                                               
    }	                                                              
  }                                                                   
  /* sign management */                                               
  if (X_SGN < 0)                                                      
    *result = - res;                                                  
  else                                                                
    *result = res;                                                    
}

#endif
/*
 * Rounded toward -Inf
 */
void scs_get_d_minf(double *result, scs_ptr x){ 

  /* round up the mantissa if negative  */
  get_d_directed(result, x, (int)(X_SGN<0));
}



/*
 * Rounded toward +Inf
 */
void scs_get_d_pinf(double *result, scs_ptr x){ 

  /* round up the mantissa if positive  */
  get_d_directed(result, x, (int)(X_SGN>=0));
}



/*
 * Rounded toward zero
 */
void scs_get_d_zero(double *result, scs_ptr x){ 
  /* never round up the mantissa  */
  get_d_directed(result, x, 0);
}
