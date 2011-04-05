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



#include "scs.h"
#include "scs_private.h"

/* Compile only if mpfr is present */

#ifdef HAVE_MPFR_H


/*
 * Convert a scs number into a MPFR number rounded to nearest
 */
void scs_get_mpfr(scs_ptr x, mpfr_t rop){
    mpfr_t mp1;
    long int expo;
    int i;

    mpfr_set_ui(rop, 0, GMP_RNDN);

    /* mantissa */
    for (i=0; i<SCS_NB_WORDS; i++){
      mpfr_mul_2exp(rop, rop, SCS_NB_BITS, GMP_RNDN);
      mpfr_add_ui(rop, rop, X_HW[i], GMP_RNDN);
    }

    /* sign */
    if (X_SGN == -1) mpfr_neg(rop, rop, GMP_RNDN);

    /* exception */
    mpfr_init_set_d(mp1, X_EXP, GMP_RNDN); 
    mpfr_mul(rop, rop, mp1, GMP_RNDN);

    /* exponent */
    expo = (X_IND - SCS_NB_WORDS + 1) * SCS_NB_BITS;

    if (expo < 0)  mpfr_div_2exp(rop, rop, (unsigned int) -expo, GMP_RNDN);
    else           mpfr_mul_2exp(rop, rop, (unsigned int) expo, GMP_RNDN);

    mpfr_clear(mp1);
}
#endif /* HAVE_MPFR_H */
