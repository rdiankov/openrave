/** Function for setting an SCS number to zero 
@file zero_scs.c

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


/** Set a SCS number to zero.
There should be a few simple functions in this library.
 */



void inline scs_zero(scs_ptr result) {
  int i;

  for(i=0; i<SCS_NB_WORDS; i++)
    R_HW[i] = 0;

  R_EXP = 0;
  R_IND = 0;
  R_SGN = 1;
}



