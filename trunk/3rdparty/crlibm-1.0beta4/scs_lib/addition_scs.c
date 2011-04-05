/** Functions for SCS addition and subtraction 

@file addition_scs.c

@author Defour David David.Defour@ens-lyon.fr
@author Florent de Dinechin Florent.de.Dinechin@ens-lyon.fr 
 
This file is part of the SCS library.

Many functions come in two versions, selected by a @#if.

The reason is that we designed scslib library for internal use with
SCS_NB_WORDS==8, so we provide a version with manual optimizations for
this case.

These optimisations include loop unrolling, and sometimes replacing
temporary arrays of size 8 with 8 variables, which is more efficient
on all modern processors with many (renaming) registers.

Using gcc3.2 with the most aggressive optimization options for this
purpose (-funroll-loops -foptimize-register-move -frerun-loop-opt
-frerun-cse-after-loop) is still much slower. At some point in the
future, gcc should catch up with unrolling since our loops are so
simple, however the replacement of small arrays with variables is
not something we are aware of in the literature about compiler
optimization.
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

/**
This function copies a result into another. There is an unrolled
version for the case SCS_NB_WORDS==8.
*/
void scs_set(scs_ptr result, scs_ptr x){
 /* unsigned int i;*/
  
#if (SCS_NB_WORDS==8)
    R_HW[0] = X_HW[0]; R_HW[1] = X_HW[1]; 
    R_HW[2] = X_HW[2]; R_HW[3] = X_HW[3]; 
    R_HW[4] = X_HW[4]; R_HW[5] = X_HW[5]; 
    R_HW[6] = X_HW[6]; R_HW[7] = X_HW[7]; 
#else
  for(i=0; i<SCS_NB_WORDS; i++)
    R_HW[i] = X_HW[i];
#endif
  R_EXP = X_EXP;  
  R_IND = X_IND; 
  R_SGN = X_SGN;
}


/** renormalize a SCS number.  

This function removes the carry from each digit, and also shifts the
digits in case of a cancellation (so that if result != 0 then its
first digit is non-zero)
 
 @warning THIS FUNCTION HAS NEVER BEEN PROPERLY TESTED and is
 currently unused in the library: instead, specific renormalisation
 steps are fused within the code of the operations which require it.
 */

void scs_renorm(scs_ptr result){
  unsigned int c;
  int i, j, k;

  /*
   * Carry propagate
   */
  for(i=SCS_NB_WORDS-1; i>0; i--){
    c = R_HW[i] & ~SCS_MASK_RADIX;
    R_HW[i-1] += c >> SCS_NB_BITS;
    R_HW[i]    = R_HW[i] &  SCS_MASK_RADIX; 
  }

  if (R_HW[0] >= SCS_RADIX){
    /*  Carry out! Need to shift digits  */
    c = R_HW[0] & ~SCS_MASK_RADIX;
    c = c >> SCS_NB_BITS;
    for(i=SCS_NB_WORDS-1; i>1; i--)
      R_HW[i] = R_HW[i-1];

    R_HW[1] = R_HW[0] & SCS_MASK_RADIX;
    R_HW[0] = c;
    R_IND  += 1;

  }else{
    /* Was there a cancellation ? */
    if (R_HW[0] == 0){

      k = 1;
      while ((R_HW[k] == 0) && (k <= SCS_NB_WORDS))
	k++;
      
      R_IND -= k;

      for(j=k, i=0; j<SCS_NB_WORDS; j++, i++)
	R_HW[i] = R_HW[j];

      for(        ; i<SCS_NB_WORDS; i++)
	R_HW[i] = 0;     

    }
  }
}



/** Renormalization without cancellation check.

 This renormalization step is especially designed for the addition of
 several numbers with the same sign.  In this case, you know that there
 has been no cancellation, which allows simpler renormalisation.
*/

void scs_renorm_no_cancel_check(scs_ptr result){ 
  unsigned int carry, c0;
 /* int i;*/

  /* Carry propagate  */      
#if (SCS_NB_WORDS==8)
  carry = R_HW[7] >> SCS_NB_BITS;
  R_HW[6] += carry;  R_HW[7] = R_HW[7] & SCS_MASK_RADIX;
  carry = R_HW[6] >> SCS_NB_BITS;
  R_HW[5] += carry;  R_HW[6] = R_HW[6] & SCS_MASK_RADIX;
  carry = R_HW[5] >> SCS_NB_BITS;
  R_HW[4] += carry;  R_HW[5] = R_HW[5] & SCS_MASK_RADIX;
  carry = R_HW[4] >> SCS_NB_BITS;
  R_HW[3] += carry;  R_HW[4] = R_HW[4] & SCS_MASK_RADIX;
  carry = R_HW[3] >> SCS_NB_BITS;
  R_HW[2] += carry;  R_HW[3] = R_HW[3] & SCS_MASK_RADIX;
  carry = R_HW[2] >> SCS_NB_BITS;
  R_HW[1] += carry;  R_HW[2] = R_HW[2] & SCS_MASK_RADIX;
  carry = R_HW[1] >> SCS_NB_BITS;
  R_HW[0] += carry;  R_HW[1] = R_HW[1] & SCS_MASK_RADIX;
#else
  for(i=(SCS_NB_WORDS-1);i>0;i--){
      carry      = R_HW[i] >> SCS_NB_BITS;
      R_HW[i-1] += carry;
      R_HW[i]    = R_HW[i] & SCS_MASK_RADIX;
  }
#endif
    
  if (R_HW[0] >= SCS_RADIX){
    /* Carry out ! Need to shift digits */
    c0 = R_HW[0] >> SCS_NB_BITS;
    
#if (SCS_NB_WORDS==8)
    R_HW[7] = R_HW[6]; R_HW[6] = R_HW[5]; 
    R_HW[5] = R_HW[4]; R_HW[4] = R_HW[3]; 
    R_HW[3] = R_HW[2]; R_HW[2] = R_HW[1]; 
#else
    for(i=(SCS_NB_WORDS-1); i>1; i--)
      R_HW[i] =  R_HW[i-1];
#endif    
    R_HW[1] =  R_HW[0] & SCS_MASK_RADIX;
    R_HW[0] =  c0;
    R_IND  += 1;
  }
  return;
}




/* addition without renormalisation.


  Add two scs number x and y, the result is put into "result".
  Assumes x.sign == y.sign    x.index > y.index.  

   The result is not normalized.
 */

static void do_add_no_renorm(scs_ptr result, scs_ptr x, scs_ptr y){
  unsigned int RES[SCS_NB_WORDS];
  unsigned int i, j, Diff;

  if (x->exception.i[HI]==0){scs_set(result, y); return; }
  if (y->exception.i[HI]==0){scs_set(result, x); return; }  
  
  for (i=0; i<SCS_NB_WORDS; i++)
    RES[i] = X_HW[i];

  Diff  = (unsigned int)(X_IND - Y_IND);
  R_EXP = X_EXP + Y_EXP - 1; 
  R_IND = X_IND;
  R_SGN = X_SGN;

  for (i=Diff, j=0; i<SCS_NB_WORDS; i++, j++)
    RES[i] += Y_HW[j];

  for (i=0; i<SCS_NB_WORDS; i++)
    R_HW[i] = RES[i];

  return;
}


/*
 * Addition without renormalization. Assumes that x.sign == y.sign.
 */
void  scs_add_no_renorm(scs_ptr result, scs_ptr x, scs_ptr y)
{
  if (X_IND >= Y_IND)
    do_add_no_renorm(result,x,y);
  else
    do_add_no_renorm(result,y,x);
  return;
}
















/* The function that does the work in case of an addition

 do_add is the function that does the addition of two SCS numbers,
   assuming that x.sign == y.sign, X_IND > Y_IND, x and y both
   non-zero. 
 */

static void do_add(scs_ptr result, scs_ptr x, scs_ptr y)
{
#if (SCS_NB_WORDS==8)  /* in this case we unroll all the loops */
  int Diff;
  unsigned int carry; 
  unsigned int r0,r1,r2,r3,r4,r5,r6,r7;
  
  Diff  = X_IND - Y_IND;
  R_EXP = X_EXP + Y_EXP - 1; 
  R_IND = X_IND;
  R_SGN = X_SGN;
#if 0
  if(Diff<4)
    if(Diff<2)
      if(Diff==0)
	{
	//      case 0:
	  r0 = X_HW[0] + Y_HW[0]; r1 = X_HW[1] + Y_HW[1]; 
	  r2 = X_HW[2] + Y_HW[2]; r3 = X_HW[3] + Y_HW[3];
	  r4 = X_HW[4] + Y_HW[4]; r5 = X_HW[5] + Y_HW[5];
	  r6 = X_HW[6] + Y_HW[6]; r7 = X_HW[7] + Y_HW[7]; 
	}
      else {
	//  case 1:
	r0 = X_HW[0];           r1 = X_HW[1] + Y_HW[0];
	r2 = X_HW[2] + Y_HW[1]; r3 = X_HW[3] + Y_HW[2];
	r4 = X_HW[4] + Y_HW[3]; r5 = X_HW[5] + Y_HW[4];
	r6 = X_HW[6] + Y_HW[5]; r7 = X_HW[7] + Y_HW[6]; 
      }
    else if(Diff==2)
      {
	//case 2:
	r0 = X_HW[0];           r1 = X_HW[1];      
	r2 = X_HW[2] + Y_HW[0]; r3 = X_HW[3] + Y_HW[1];
	r4 = X_HW[4] + Y_HW[2]; r5 = X_HW[5] + Y_HW[3];
	r6 = X_HW[6] + Y_HW[4]; r7 = X_HW[7] + Y_HW[5];
      }
    else
      {
	//  case 3:
	r0 = X_HW[0];           r1 = X_HW[1];     
	r2 = X_HW[2];           r3 = X_HW[3] + Y_HW[0];
	r4 = X_HW[4] + Y_HW[1]; r5 = X_HW[5] + Y_HW[2];
	r6 = X_HW[6] + Y_HW[3]; r7 = X_HW[7] + Y_HW[4];
      }
  else if(Diff<6)
    if(Diff==4)
      {
	// case 4:
	r0 = X_HW[0];           r1 = X_HW[1];
	r2 = X_HW[2];           r3 = X_HW[3];
	r4 = X_HW[4] + Y_HW[0]; r5 = X_HW[5] + Y_HW[1]; 
	r6 = X_HW[6] + Y_HW[2]; r7 = X_HW[7] + Y_HW[3];
      }
    else {
      //  case 5:
      r0 = X_HW[0];           r1 = X_HW[1];
      r2 = X_HW[2];           r3 = X_HW[3];
      r4 = X_HW[4];           r5 = X_HW[5] + Y_HW[0];
      r6 = X_HW[6] + Y_HW[1]; r7 = X_HW[7] + Y_HW[2];
    }   
  else if(Diff<8)
    if(Diff==6)
      {
      // case 6:
	r0 = X_HW[0];           r1 = X_HW[1];
	r2 = X_HW[2];           r3 = X_HW[3];
	r4 = X_HW[4];           r5 = X_HW[5];
	r6 = X_HW[6] + Y_HW[0]; r7 = X_HW[7] + Y_HW[1];
      }
    else {
      // case 7:
      r0 = X_HW[0];           r1 = X_HW[1];
      r2 = X_HW[2];           r3 = X_HW[3];
      r4 = X_HW[4];           r5 = X_HW[5];
      r6 = X_HW[6];           r7 = X_HW[7] + Y_HW[0]; 
    }

  else
    {
      /* Diff >= 8*/
      R_HW[0] = X_HW[0]; R_HW[1] = X_HW[1];
      R_HW[2] = X_HW[2]; R_HW[3] = X_HW[3];
      R_HW[4] = X_HW[4]; R_HW[5] = X_HW[5];
      R_HW[6] = X_HW[6]; R_HW[7] = X_HW[7];    
      return;
    }
#else
  switch (Diff){
  case 0:
    r0 = X_HW[0] + Y_HW[0]; r1 = X_HW[1] + Y_HW[1];
    r2 = X_HW[2] + Y_HW[2]; r3 = X_HW[3] + Y_HW[3];
    r4 = X_HW[4] + Y_HW[4]; r5 = X_HW[5] + Y_HW[5];
    r6 = X_HW[6] + Y_HW[6]; r7 = X_HW[7] + Y_HW[7]; break;
  case 1:
    r0 = X_HW[0];           r1 = X_HW[1] + Y_HW[0];
    r2 = X_HW[2] + Y_HW[1]; r3 = X_HW[3] + Y_HW[2];
    r4 = X_HW[4] + Y_HW[3]; r5 = X_HW[5] + Y_HW[4];
    r6 = X_HW[6] + Y_HW[5]; r7 = X_HW[7] + Y_HW[6]; break;
  case 2:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2] + Y_HW[0]; r3 = X_HW[3] + Y_HW[1];
    r4 = X_HW[4] + Y_HW[2]; r5 = X_HW[5] + Y_HW[3];
    r6 = X_HW[6] + Y_HW[4]; r7 = X_HW[7] + Y_HW[5]; break;
  case 3:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2];           r3 = X_HW[3] + Y_HW[0];
    r4 = X_HW[4] + Y_HW[1]; r5 = X_HW[5] + Y_HW[2];
    r6 = X_HW[6] + Y_HW[3]; r7 = X_HW[7] + Y_HW[4]; break;
  case 4:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2];           r3 = X_HW[3];
    r4 = X_HW[4] + Y_HW[0]; r5 = X_HW[5] + Y_HW[1];
    r6 = X_HW[6] + Y_HW[2]; r7 = X_HW[7] + Y_HW[3]; break;
  case 5:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2];           r3 = X_HW[3];
    r4 = X_HW[4];           r5 = X_HW[5] + Y_HW[0];
    r6 = X_HW[6] + Y_HW[1]; r7 = X_HW[7] + Y_HW[2]; break;
  case 6:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2];           r3 = X_HW[3];
    r4 = X_HW[4];           r5 = X_HW[5];
    r6 = X_HW[6] + Y_HW[0]; r7 = X_HW[7] + Y_HW[1]; break;
  case 7:
    r0 = X_HW[0];           r1 = X_HW[1];
    r2 = X_HW[2];           r3 = X_HW[3];
    r4 = X_HW[4];           r5 = X_HW[5];
    r6 = X_HW[6];           r7 = X_HW[7] + Y_HW[0]; break;
  default:
    /* Diff >= 8*/
    R_HW[0] = X_HW[0]; R_HW[1] = X_HW[1];
    R_HW[2] = X_HW[2]; R_HW[3] = X_HW[3];
    R_HW[4] = X_HW[4]; R_HW[5] = X_HW[5];
    R_HW[6] = X_HW[6]; R_HW[7] = X_HW[7];    return;
 }
#endif

  /* Carry propagation */
  
  carry = r7 >> SCS_NB_BITS; r6 += carry;  r7 = r7 & SCS_MASK_RADIX;
  carry = r6 >> SCS_NB_BITS; r5 += carry;  r6 = r6 & SCS_MASK_RADIX;
  carry = r5 >> SCS_NB_BITS; r4 += carry;  r5 = r5 & SCS_MASK_RADIX;
  carry = r4 >> SCS_NB_BITS; r3 += carry;  r4 = r4 & SCS_MASK_RADIX;
  carry = r3 >> SCS_NB_BITS; r2 += carry;  r3 = r3 & SCS_MASK_RADIX;
  carry = r2 >> SCS_NB_BITS; r1 += carry;  r2 = r2 & SCS_MASK_RADIX;
  carry = r1 >> SCS_NB_BITS; r0 += carry;  r1 = r1 & SCS_MASK_RADIX;
  carry = r0 >> SCS_NB_BITS;
   
  if (carry!=0){
    R_HW[7] = r6; R_HW[6] = r5;  R_HW[5] = r4; R_HW[4] = r3; 
    R_HW[3] = r2; R_HW[2] = r1;  R_HW[1] = r0 & SCS_MASK_RADIX;
    R_HW[0] = 1 ;  
    R_IND  += 1;
  }
  else {
    R_HW[0] = r0; R_HW[1] = r1; R_HW[2] = r2; R_HW[3] = r3; 
    R_HW[4] = r4; R_HW[5] = r5; R_HW[6] = r6; R_HW[7] = r7; 
  }
  return;

#else /* #if SCS_NB_WORDS==8*/

/* This generic version is still written in such a way that
 it is unrollable at compile time
*/
  int i,j, s, carry, Diff;
  int res[SCS_NB_WORDS];
  
  Diff  = X_IND - Y_IND;
  R_EXP = X_EXP + Y_EXP - 1; 
  R_IND = X_IND;
  R_SGN = X_SGN;

  /* The easy case */
  if(Diff >= SCS_NB_WORDS){  
    scs_set(result, x); return;
  }

  /* 0 <= Diff <= (SCS_NB_WORDS-1) */

  carry=0;
  for(i=(SCS_NB_WORDS-1), j=((SCS_NB_WORDS-1)-Diff); i>=0 ; i--,j--){
    if (j>=0)
      s = X_HW[i] + Y_HW[j] + carry;
    else
      s = X_HW[i] + carry;
    carry = s >> SCS_NB_BITS;
    res[i] = s & SCS_MASK_RADIX;
  }

  if (carry){
    /* Carry out ! Need to shift digits */    
    for(i=(SCS_NB_WORDS-1); i>=1; i--)
      R_HW[i] =  res[i-1];
    
    R_HW[0] = 1 ;  
    R_IND  += 1;
  }
  else {
    for(i=0; i<SCS_NB_WORDS; i++)
      R_HW[i] =  res[i];
  }

  return;
#endif  /* #if SCS_NB_WORDS==8*/

} /* do_add*/




/*/////////////////////////////////////////////////////////////////
/////////////////////// SUBTRACTION //////////////////////////////
//////////////////////////////////////////////////////////////////
// This procedure assumes :
// -    X_IND >= Y_IND
// -   X_SIGN != Y_SIGN
//    neither x or y is zero
//  and result = x - y  
*/


static void do_sub(scs_ptr result, scs_ptr x, scs_ptr y){
  int s, carry;
  int Diff, i, j, cp;
  int res[SCS_NB_WORDS];

  R_EXP = X_EXP + Y_EXP - 1;
  Diff  = X_IND - Y_IND;
  R_IND = X_IND;
  
    /* The easy case */
  if(Diff >= SCS_NB_WORDS){  
    scs_set(result, x); return;
  }

  else { 
    /* 0 <= Diff <= (SCS_NB_WORDS-1) */
    carry = 0;
    if(Diff==0) { 

      i=0;
      while((X_HW[i] == Y_HW[i]) && (i<SCS_NB_WORDS)) i++;
      if (X_HW[i] > Y_HW[i]) cp=1;
      else if (X_HW[i] < Y_HW[i]) cp=-1;
      else cp=0;

      if (cp == 0) {
	/* Yet another easy case: result = 0 */
	scs_zero(result);
	return;
      }
      else { /* cp <> 0 */
	if (cp > 0){
	  /* x > y */

	  R_SGN = X_SGN; 
	  for(i=(SCS_NB_WORDS-1); i>=0 ;i--){
	    s = (int)(X_HW[i] - Y_HW[i] - carry);
	    carry = (int)((s&SCS_RADIX)>>SCS_NB_BITS);
	    res[i] = (int)((s&SCS_RADIX) + s);
	  }	  
	}
	else { /* cp < 0  */
	  /* x < y (change of sign) */

	  R_SGN = - X_SGN;
	  for(i=(SCS_NB_WORDS-1); i>=0 ;i--){
	    s = (int)(- X_HW[i] + Y_HW[i] - carry);
	    carry = (int)((s&SCS_RADIX)>>SCS_NB_BITS);
	    res[i] = (int)((s&SCS_RADIX) + s);
	  }
	}
      }
    }
    else {
      /* 1<=Diff<(SCS_NB_WORDS-1) Digits of x and y overlap but the
       * sign will be that of x */
      
      R_SGN = X_SGN; 
      for(i=(SCS_NB_WORDS-1), j=((SCS_NB_WORDS-1)-Diff); i>=0 ;i--,j--){
	if(j>=0)
	  s = (int)(X_HW[i] - Y_HW[j] - carry);
	else
	  s = (int)(X_HW[i] - carry);
	carry = (int)((s&SCS_RADIX)>>SCS_NB_BITS);
	res[i] = (int)((s&SCS_RADIX) + s);
      }
    }
    /* check for cancellations */
    i=0;
    while ((res[i]==0) && (i < SCS_NB_WORDS))  i++;

    if(i>0) { /* cancellation, shift result*/
      R_IND -= i;
      for(j=0; i<SCS_NB_WORDS; i++,j++)    R_HW[j] = (unsigned int)(res[i]);
      for(   ; j<SCS_NB_WORDS; j++) 	   R_HW[j] = 0;
    }
    else {
      for(i=0; i<SCS_NB_WORDS; i++)
	R_HW[i] =  (unsigned int)(res[i]);
    }
  }
  return;
}





 


/** SCS addition (result is a normalised SCS number).

 */
void  scs_add(scs_ptr result, scs_ptr x, scs_ptr y)
{
    
  if (x->exception.i[HI]==0){scs_set(result, y); return; }
  if (y->exception.i[HI]==0){scs_set(result, x); return; }  

  if (X_SGN == Y_SGN){
    if(X_IND >= Y_IND)
      do_add(result,x,y);
    else
      do_add(result,y,x);
  }else {  
    if(X_IND>=Y_IND){
      do_sub(result,x,y);
    }else { 
      do_sub(result,y,x); 
    } 
  } return; 
}

/** SCS subtraction (result is a normalised SCS number).

 The arguments x, y and result may point to the same memory
 location. 
 */
void  scs_sub(scs_ptr result, scs_ptr x, scs_ptr y)
{
  if (x->exception.i[HI]==0)
    { scs_set(result, y); R_SGN = -R_SGN; return; }
  if (y->exception.i[HI]==0)
    { scs_set(result, x); return; }

  if (X_SGN == Y_SGN) {
    /* Same sign, so it's a sub   */
    if(X_IND>=Y_IND)
      do_sub(result,x,y);
    else{
      do_sub(result,y,x);
      R_SGN = -R_SGN;
    }
  }else {
    if(X_IND>=Y_IND)
      do_add(result,x,y);
    else{
      do_add(result,y,x);
      R_SGN = -R_SGN; 
    }
  }
  return;
}


