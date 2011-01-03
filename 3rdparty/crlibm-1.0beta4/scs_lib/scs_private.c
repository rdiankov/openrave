#include "scs.h"
#include "scs_private.h"

#ifdef WORDS_BIGENDIAN
  const db_number radix_one_double  = {{((1023+SCS_NB_BITS)<<20) ,               0x00000000 }}; 
  const db_number radix_two_double  = {{((1023+2*SCS_NB_BITS)<<20) ,             0x00000000 }}; 
  const db_number radix_mone_double = {{((1023-SCS_NB_BITS)<<20) ,               0x00000000 }}; 
  const db_number radix_mtwo_double = {{((1023-2*SCS_NB_BITS)<<20) ,             0x00000000 }}; 
  const db_number radix_rng_double  = {{((1023+SCS_NB_BITS*SCS_MAX_RANGE)<<20) , 0x00000000 }}; 
  const db_number radix_mrng_double = {{((1023-SCS_NB_BITS*SCS_MAX_RANGE)<<20) , 0x00000000 }};
  const db_number max_double        = {{0x7FEFFFFF ,                             0xFFFFFFFF }}; 
  const db_number min_double        = {{0x00000000 ,                             0x00000001 }}; 
#else
  const db_number radix_one_double  = {{0x00000000 , ((1023+SCS_NB_BITS)<<20)               }}; 
  const db_number radix_two_double  = {{0x00000000 , ((1023+2*SCS_NB_BITS)<<20)             }}; 
  const db_number radix_mone_double = {{0x00000000 , ((1023-SCS_NB_BITS)<<20)               }}; 
  const db_number radix_mtwo_double = {{0x00000000 , ((1023-2*SCS_NB_BITS)<<20)             }}; 
  const db_number radix_rng_double  = {{0x00000000 , ((1023+SCS_NB_BITS*SCS_MAX_RANGE)<<20) }}; 
  const db_number radix_mrng_double = {{0x00000000 , ((1023-SCS_NB_BITS*SCS_MAX_RANGE)<<20) }}; 
  const db_number max_double        = {{0xFFFFFFFF ,                             0x7FEFFFFF }}; 
  const db_number min_double        = {{0x00000001 ,                             0x00000000 }}; 
#endif

