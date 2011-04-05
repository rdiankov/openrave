/*
 * Author  : Defour David
 * Contact : David.Defour@ens-lyon.fr
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
#include "scs.h"
#include "scs_private.h"


/*
 * used by the next function to write
 * bit of the integer in the right order ....
 */
static void print_order(unsigned int x, int nb, int b){
  if (nb<=0)
    return;
  print_order(x/b, nb-1, b);
  printf("%u",x%b);
  return;
}
/*
 * print nb digits of the chain x in base "b"
 * b must be between 1 and 10
 */ 
static void print_integer(unsigned int x, int b, int nb){
   
  if ((b < 2)||(b>16)){
    fprintf(stderr," ERROR: You musn't print number with a base larger than 10 or less than 2 \n");
    return;
  }
  print_order(x, nb, b);
  return;
}


/*
 * Convert a double precision number in it scs multiprecision
 * representation
 *
 * Rem. : We haven't tested all special cases yet.
 */

/*
 */
void scs_get_std( scs_ptr x){
  int i; 
  db_number d;

  scs_get_d(&d.d, x);
  printf("Exception : %e \n", X_EXP);
  printf("Index= %d   \n Sign=  %d \n Double value= %.30e   \n Hex mantissa= %x %x\n", 
	 X_IND, X_SGN, d.d, d.i[HI], d.i[LO]);
  for(i=0;i<SCS_NB_WORDS;i++){
    printf("  D %d :  %8x %20u \n",i, X_HW[i], X_HW[i]);
  }
}



