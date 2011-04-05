/** Functions for SCS inverse and division

@file division_scs.c

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


void scs_div_2(scs_ptr num) {
  /* small function to divide by 2 any SCS number */
  unsigned int carry, mask, old_value;
  int i;
  carry = 0x00000000;
  mask = (unsigned int)(((0x1) << SCS_NB_BITS)-1);/*we now have a mask for the used bits in a word*/
  /* if it's a normal number, i.e. not zero nor NaN */
  if( (num->exception).d == (double) 1) {
    /* first, a loop to rotate all numbers to the right*/
    for(i = 0; i < SCS_NB_WORDS; i++) {
      old_value = num->h_word[i];
      num->h_word[i] = (old_value & !mask) | carry | ((old_value >> 1) & mask);
      carry = old_value & 0x00000001;/* it can be interesting to keep the last bit of each number =) */
      carry = carry << (SCS_NB_BITS-1);
    }
    /* In the SCS format, the first number can't be zero, so we must handle this particular case */
    if (num->h_word[0] == 0){
      num->index = num->index - 1;
      for(i = 1; i < SCS_NB_WORDS; i++) {
	num->h_word[i-1] = num->h_word[i];
      }
      num->h_word[SCS_NB_WORDS-1] = 0;
    }
  }
  else {
    (num->exception).d = (num->exception).d / 2;/* zero, NaN, ... */
  }
}




/*
 * Compute 1/x with a Newton scheme
 */
void scs_inv(scs_ptr result, scs_ptr x){
  scs_t tmp, res, res1, scstwo;
  double app_x, inv;
  
  scs_set(tmp, x);  tmp->index = 0;  scs_get_d(&app_x, tmp);

  scs_set_si(scstwo, 2);
  /* inv is a 53-bit approximation of 1/x */
  inv = 1/app_x;
  
  scs_set_d(res, inv);
  res->index -= x->index;

  /* First Newton Iteration */
  scs_mul(res1, x, res);
  scs_sub(res1, scstwo, res1);
  scs_mul(res, res, res1);

  /* Second Newton Iteration */
  scs_mul(res1, x, res);
  scs_sub(res1, scstwo, res1);
  scs_mul(result, res, res1); 

  return;
}

/*
 * Compute result = x/y; 
 */
void scs_div(scs_ptr result, scs_ptr x, scs_ptr y){ 
  scs_t res;
 
  if (X_EXP != 1){
    R_EXP = X_EXP / Y_EXP;
    return;
  }

  scs_inv(res, y);
  scs_mul(result, res, x);
  return;
}
