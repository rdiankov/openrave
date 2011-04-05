/*
 * rem_pio2, used in correctly rounded trigonometric functions
 *
 * Author : Catherine Daramy, David Defour, Florent de Dinechin
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

/**
 * This file include functions to compute y=x-N*pi/2 and return the last two bits of N
 * in order to know which quadrant we are considering.
 *
 * We use an scs representation to compute it by Payne and Hanek methods. For more information
 * you can read K. C. Ng research report from Sun Microsystems:
 * "Argument reduction for huge argument: Good to the last bit" (July 13, 1992)
 *
 */
#include "rem_pio2_accurate.h"


/**
 * Case X_IND = -1:
 *               0 
 *           2 ^ 
 *  X    :   <> |--| |--| |--|   0   0 0 0 0
 *  2/Pi :   <> |--| |--| |--| |--| .....
 *
 *  Case X_IND = 0:
 *                   0 
 *                2 ^ 
 *  X    :   |--| <> |--| |--|   0   0 0 0 0
 *  2/Pi :        <> |--| |--| |--| |--| .....
 *
 *  Case X_IND = 1:
 *                       0 
 *                    2 ^ 
 *  X    :   |--| |--| <> |--| |--|   0   0 0 0 0
 *  2/Pi :             <> |--| |--| |--| |--| .....
 *
 *  Case ...
 * 
 *  Step 1:
 *
 *   Compute r = X . 2/Pi where: 
 *    - r[0] hold the integer part. (if x>0 or the once complement integer part if x<0 )
 *    - r[1] to r[SCS_NB_WORDS+2] hold the reduced part
 *      the 3 extra 30 bits are here to prevent possible 
 *      cancellation due to a number x too close to a
 *      multiple of Pi/2.
 *
 *  Step 2:
 *   Compute result = (r[1] ... r[SCS_NB_WORDS]) . Pi/2.
 *       
 * description of local variables :
 * - ind : where to start multiplying into 2opi table
 *
 */

/* TODO OPTIM 
   better 64-bit multiplication, see in scs_mult */

int rem_pio2_scs(scs_ptr result, const scs_ptr x){
  uint64_t r[SCS_NB_WORDS+3], tmp;
  unsigned int N;
  /* result r[0],...,r[10] could store till 300 bits of precision */
  /* that is really enough for computing the reduced argument */
  int sign, i, j, ind;
  int *two_over_pi_pt;

  if ((X_EXP != 1)||(X_IND < -1)){
    scs_set(result, x);
    return 0;
  }
  


  /* Compute the product |x| * 2/Pi */
  if ((X_IND == -1)){
    /* In this case we consider number between ]-1,+1[    */
    /* we may use simpler algorithm such as Cody And Waite */
    r[0] =  0;    r[1] =  0;
    r[2] =  (uint64_t)(two_over_pi[0]) * X_HW[0];
    r[3] = ((uint64_t)(two_over_pi[0]) * X_HW[1]
	   +(uint64_t)(two_over_pi[1]) * X_HW[0]);
    if(X_HW[2] == 0){
      for(i=4; i<(SCS_NB_WORDS+3); i++){   
	r[i] = ((uint64_t)(two_over_pi[i-3]) * X_HW[1]
	       +(uint64_t)(two_over_pi[i-2]) * X_HW[0]);
      }}else {
	for(i=4; i<(SCS_NB_WORDS+3); i++){   
	  r[i] = ((uint64_t)(two_over_pi[i-4]) * X_HW[2]
		 +(uint64_t)(two_over_pi[i-3]) * X_HW[1]
		 +(uint64_t)(two_over_pi[i-2]) * X_HW[0]);
	}
      }
  }else {
    if (X_IND == 0){
      r[0] =  0;
      r[1] =  (uint64_t)(two_over_pi[0]) * X_HW[0];
      r[2] = ((uint64_t)(two_over_pi[0]) * X_HW[1]
	     +(uint64_t)(two_over_pi[1]) * X_HW[0]);
      if(X_HW[2] == 0){
	for(i=3; i<(SCS_NB_WORDS+3); i++){   
	  r[i] = ((uint64_t)(two_over_pi[i-2]) * X_HW[1]
		 +(uint64_t)(two_over_pi[i-1]) * X_HW[0]);
	}}else {
	  for(i=3; i<(SCS_NB_WORDS+3); i++){   
	    r[i] = ((uint64_t)(two_over_pi[i-3]) * X_HW[2]
		   +(uint64_t)(two_over_pi[i-2]) * X_HW[1]
		   +(uint64_t)(two_over_pi[i-1]) * X_HW[0]);
	  }}
    }else {
      if (X_IND == 1){
	r[0] =  (uint64_t)(two_over_pi[0]) * X_HW[0];
	r[1] = ((uint64_t)(two_over_pi[0]) * X_HW[1]
	       +(uint64_t)(two_over_pi[1]) * X_HW[0]);
	if(X_HW[2] == 0){
	  for(i=2; i<(SCS_NB_WORDS+3); i++){   
	    r[i] = ((uint64_t)(two_over_pi[i-1]) * X_HW[1]
		   +(uint64_t)(two_over_pi[ i ]) * X_HW[0]);
	  }}else {
	    for(i=2; i<(SCS_NB_WORDS+3); i++){   
	      r[i] = ((uint64_t)(two_over_pi[i-2]) * X_HW[2]
		     +(uint64_t)(two_over_pi[i-1]) * X_HW[1]
		     +(uint64_t)(two_over_pi[ i ]) * X_HW[0]);
	    }}
      }else {
	if (X_IND == 2){
  	  r[0] = ((uint64_t)(two_over_pi[0]) * X_HW[1]
		 +(uint64_t)(two_over_pi[1]) * X_HW[0]);
	  if(X_HW[2] == 0){
	    for(i=1; i<(SCS_NB_WORDS+3); i++){   
	      r[i] = ((uint64_t)(two_over_pi[ i ]) * X_HW[1]
		     +(uint64_t)(two_over_pi[i+1]) * X_HW[0]);
	    }}else {
	      for(i=1; i<(SCS_NB_WORDS+3); i++){   
		r[i] = ((uint64_t)(two_over_pi[i-1]) * X_HW[2]
		       +(uint64_t)(two_over_pi[ i ]) * X_HW[1]
		       +(uint64_t)(two_over_pi[i+1]) * X_HW[0]);
	      }}
	}else {
	  ind = (X_IND - 3);
	  two_over_pi_pt = (int*)&(two_over_pi[ind]);
	  if(X_HW[2] == 0){
	    for(i=0; i<(SCS_NB_WORDS+3); i++){   
	      r[i] = ((uint64_t)(two_over_pi_pt[i+1]) * X_HW[1]
		     +(uint64_t)(two_over_pi_pt[i+2]) * X_HW[0]);
	    }}else {
	      for(i=0; i<(SCS_NB_WORDS+3); i++){   
		r[i] = ((uint64_t)(two_over_pi_pt[ i ]) * X_HW[2]
		       +(uint64_t)(two_over_pi_pt[i+1]) * X_HW[1]
		       +(uint64_t)(two_over_pi_pt[i+2]) * X_HW[0]);
	      }
	    }
	}
      }
    }
  }
      
  /* Carry propagate */
  r[SCS_NB_WORDS+1] += r[SCS_NB_WORDS+2]>>30;
  for(i=(SCS_NB_WORDS+1); i>0; i--) {tmp=r[i]>>30;   r[i-1] += tmp;  r[i] -= (tmp<<30);}  
      
  /* The integer part is in r[0] */
  N = (unsigned int)(r[0]);
#if 0
  printf("r[0] = %d\n", N);
#endif


  /* test if the reduced part is bigger than Pi/4 */
  if (r[1] > (uint64_t)(SCS_RADIX)/2){
    N += 1;
    sign = -1;
    for(i=1; i<(SCS_NB_WORDS+3); i++) { r[i]=((~(unsigned int)(r[i])) & 0x3fffffff);}
  } 
  else
    sign = 1; 


  /* Now we get the reduce argument and check for possible
   * cancellation By Kahan algorithm we will have at most 2 digits
   * of cancellations r[1] and r[2] in the worst case.
   */    
  if (r[1] == 0)
    if (r[2] == 0) i = 3;
    else           i = 2;
  else             i = 1;

  for(j=0; j<SCS_NB_WORDS; j++) { R_HW[j] = (unsigned int)(r[i+j]);}


  R_EXP   = 1;
  R_IND   = -i;
  R_SGN   = sign*X_SGN; 
  
  /* Last step :
   *   Multiplication by pi/2
   */
  scs_mul(result, Pio2_ptr, result);
  return X_SGN*N;
}
 

