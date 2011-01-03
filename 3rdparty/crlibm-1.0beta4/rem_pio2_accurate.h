/*
 * 2oPi[] store in hexadecimal 48 digits, each keeping 30 bits of
 * 2/pi. 
 * We then store 1440 bits of 2/pi. which is for sure a bit too much ...
 */
 
#include "crlibm.h"
#include "crlibm_private.h"


static const int two_over_pi[]=
  {0x28be60db, 0x24e44152, 0x27f09d5f, 0x11f534dd,
   0x3036d8a5, 0x1993c439, 0x0107f945, 0x23abdebb,
   0x31586dc9, 0x06e3a424, 0x374b8019, 0x092eea09,
   0x3464873f, 0x21deb1cb, 0x04a69cfb, 0x288235f5,
   0x0baed121, 0x0e99c702, 0x1ad17df9, 0x013991d6,
   0x0e60d4ce, 0x1f49c845, 0x3e2ef7e4, 0x283b1ff8,
   0x25fff781, 0x1980fef2, 0x3c462d68, 0x0a6d1f6d,
   0x0d9fb3c9, 0x3cb09b74, 0x3d18fd9a, 0x1e5fea2d,
   0x1d49eeb1, 0x3ebe5f17, 0x2cf41ce7, 0x378a5292,
   0x3a9afed7, 0x3b11f8d5, 0x3421580c, 0x3046fc7b,
   0x1aeafc33, 0x3bc209af, 0x10d876a7, 0x2391615e,
   0x3986c219, 0x199855f1, 0x1281a102, 0x0dffd880};




/*
 * This scs number store 211 bits of pi/2 
 */
static const scs Pio2=
  {{0x00000001, 0x2487ed51, 0x042d1846, 0x26263314,
    0x1701b839, 0x28948127, 0x01114cf9, 0x23a0105d},
    DB_ONE,  0,   1 };

#define Pio2_ptr  (scs_ptr)(& Pio2)



#if 0

/*
 * This scs number store 211 bits of pi/4 
 */
static const scs Pio4=
/* ~7.853982e-01 */ 
{{0x3243f6a8, 0x22168c23, 0x1313198a, 0x0b80dc1c, 
0x344a4093, 0x2088a67c, 0x31d0082e, 0x3ea63b13},
DB_ONE,  -1,   1 };

#define Pio4_ptr  ((scs_ptr)(& Pio4))

#endif
