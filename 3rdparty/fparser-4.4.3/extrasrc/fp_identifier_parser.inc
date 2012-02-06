/* NOTE:
  Do not include this file in your project. The fparser.cc file #includes
this file internally and thus you don't need to do anything (other than keep
this file in the same directory as fparser.cc).

  Part of this file is generated code (by using the make_function_name_parser
utility, found in the development version of this library). It's not intended
to be modified by hand.
*/

        unsigned nameLength = 0;
        const unsigned maximumNameLength = 0x80000000U-8;
        /*
        Due to the manner the identifier lengths are returned from
        the readOpcode() function, the maximum supported length for
        identifiers is 0x7FFFFFFF bytes. We minus 8 here to add some
        buffer, because of the multibyteness of UTF-8.
        Function names are limited to 0xFFFF bytes instead, but because
        function names that long just are not defined, the point is moot.
        */
        const unsigned char* const uptr = (const unsigned char*) input;
        typedef signed char schar;
        while(likely(nameLength < maximumNameLength))
        {
            unsigned char byte = uptr[nameLength+0];
            /* Handle the common case of A-Za-z first */
            if(byte >= 0x40)
            {
                if(byte < 0x80) // 0x40..0x7F - most common case
                {
                    // Valid characters in 40..7F: A-Za-z_
                    // Valid bitmask for 40..5F: 01111111111111111111111111100001
                    // Valid bitmask for 60..7F: 01111111111111111111111111100000
                    if(sizeof(unsigned long) == 8)
                    {
                        const unsigned n = sizeof(unsigned long)*8-32;
                        // ^ avoids compiler warning when not 64-bit
                        unsigned long masklow6bits = 1UL << (byte & 0x3F);
                        if(masklow6bits & ~((1UL << 0) | (0x0FUL << (0x1B  ))
                                          | (1UL << n) | (0x1FUL << (0x1B+n))))
                            { ++nameLength; continue; }
                    }
                    else
                    {
                        unsigned masklow5bits = 1 << (byte & 0x1F);
                        if((masklow5bits & ~(1 | (0x1F << 0x1B))) || byte == '_')
                            { ++nameLength; continue; }
                    }
                    break;
                }
                if(byte < 0xF0)
                {
                    if(byte < 0xE0)
                    {
                        if(byte < 0xC2) break; // 0x80..0xC1
                        if(byte == 0xC2 && uptr[nameLength+1]==0xA0) break; // skip nbsp
                        // C2-DF - next common case when >= 0x40
                        // Valid sequence: C2-DF 80-BF
                        if(schar(uptr[nameLength+1]) > schar(0xBF)) break;
                        nameLength += 2;
                        continue;
                    }
                    if(byte == 0xE0) // E0
                    {
                        // Valid sequence: E0 A0-BF 80-BF
                        if((unsigned char)(uptr[nameLength+1] - 0xA0) > (0xBF-0xA0)) break;
                    }
                    else
                    {
                        if(byte == 0xED) break; // ED is invalid
                        // Valid sequence: E1-EC 80-BF 80-BF
                        //            And: EE-EF 80-BF 80-BF
                        if(byte == 0xE2)
                        {
                            // break on various space characters
                            if(uptr[nameLength+1] == 0x80
                            && (schar(uptr[nameLength+2]) <= schar(0x8B)
                            || (uptr[nameLength+2] == 0xAF))) break;
                            if(uptr[nameLength+1] == 0x81
                            && uptr[nameLength+2] == 0x9F) break;
                        } else
                        if(byte == 0xE3 && uptr[nameLength+1] == 0x80
                        && uptr[nameLength+2] == 0x80) break; // this too

                        if(schar(uptr[nameLength+1]) > schar(0xBF)) break;
                    }
                    if(schar(uptr[nameLength+2]) > schar(0xBF)) break;
                    nameLength += 3;
                    continue;
                }
                if(byte == 0xF0) // F0
                {
                    // Valid sequence: F0 90-BF 80-BF 80-BF
                    if((unsigned char)(uptr[nameLength+1] - 0x90) > (0xBF-0x90)) break;
                }
                else
                {
                    if(byte > 0xF4) break; // F5-FF are invalid
                    if(byte == 0xF4) // F4
                    {
                        // Valid sequence: F4 80-8F
                        if(schar(uptr[nameLength+1]) > schar(0x8F)) break;
                    }
                    else
                    {
                        // F1-F3
                        // Valid sequence: F1-F3 80-BF 80-BF 80-BF
                        if(schar(uptr[nameLength+1]) > schar(0xBF)) break;
                    }
                }
                if(schar(uptr[nameLength+2]) > schar(0xBF)) break;
                if(schar(uptr[nameLength+3]) > schar(0xBF)) break;
                nameLength += 4;
                continue;
            }
            if(nameLength > 0)
            {
                if(sizeof(unsigned long) == 8)
                {
                    // Valid bitmask for 00..1F: 00000000000000000000000000000000
                    // Valid bitmask for 20..3F: 00000000000000001111111111000000
                    const unsigned n = sizeof(unsigned long)*8-32;
                    // ^ avoids compiler warning when not 64-bit
                    unsigned long masklow6bits = 1UL << byte;
                    if(masklow6bits & (((1UL << 10)-1UL) << (16+n)))
                        { ++nameLength; continue; }
                }
                else
                {
                    if(byte >= '0' && byte <= '9')
                        { ++nameLength; continue; }
                }
            }
            break;
        }

        /* This function generated with make_function_name_parser.cc */
#define lO l1 lE
#define lN l4 lE
#define lM if('i'l5
#define lL 'n'l5
#define lK l2 3]={
#define lJ 0x80000003U;
#define lI )==0)l0(
#define lH l3 3 lI
#define lG 0x80000005U;
#define lF :l9 uptr[
#define lE case
#define lD l8 3;}lE
#define lC std::memcmp(uptr+
#define lB lF 1]){lE
#define lA 'a'lB
#define l9 switch(
#define l8 default:l0
#define l7 lG l0 5;}lE
#define l6 <<16)|
#define l5 ==uptr[
#define l4 lJ l0 3;
#define l3 lC 1,tmp,
#define l2 static const char tmp[
#define l1 0x80000004U;l0 4;
#define l0 return
l9
nameLength){lE
2:lM
0]&&'f'l5
1])l0(cIf
l6
0x80000002U;l0
2;lE
3
lF
0]){lE
lA'b':if('s'l5
2])l0(cAbs
l6
lN'r':if('g'l5
2])l0(cArg
l6
l4
lD'c'lB'o'lF
2]){lE's':l0(cCos
l6
lJ
lE't':l0(cCot
l6
lJ
lD's':if('c'l5
2])l0(cCsc
l6
l4
lD'e':if('x'l5
1]&&'p'l5
2])l0(cExp
l6
lN'i':if(lL
1]&&'t'l5
2])l0(cInt
l6
lN'l':if('o'l5
1]&&'g'l5
2])l0(cLog
l6
lN'm'lB'a':if('x'l5
2])l0(cMax
l6
lN'i':if(lL
2])l0(cMin
l6
l4
lD'p':if('o'l5
1]&&'w'l5
2])l0(cPow
l6
lN's'lB'e':if('c'l5
2])l0(cSec
l6
lN'i':if(lL
2])l0(cSin
l6
l4
lD't':if('a'l5
1]&&lL
2])l0(cTan
l6
l4
lD
4
lF
0]){lE
lA'c':if('o'l5
2]&&'s'l5
3])l0(cAcos
l6
lO's':lM
2]&&lL
3])l0(cAsin
l6
lO't':if('a'l5
2]&&lL
3])l0(cAtan
l6
l1
l8
4;}
lE'c'lB'b':if('r'l5
2]&&'t'l5
3])l0(cCbrt
l6
lO'e':lM
2]&&'l'l5
3])l0(cCeil
l6
lO'o'lF
2]){lE'n':if('j'l5
3])l0(cConj
l6
lO's':if('h'l5
3])l0(cCosh
l6
l1
l8
4;}
l8
4;}
lE'e'lB'v':if('a'l5
2]&&'l'l5
3])l0(cEval
l6
lO'x':if('p'l5
2]&&'2'l5
3])l0(cExp2
l6
l1
l8
4;}
lE'i':{lK'm','a','g'}
;if(lH
cImag
l6
l1}
lE'l':{lK'o','g','2'}
;if(lH
cLog2
l6
l1}
lE'r':{lK'e','a','l'}
;if(lH
cReal
l6
l1}
lE's'lB'i':if(lL
2]&&'h'l5
3])l0(cSinh
l6
lO'q':if('r'l5
2]&&'t'l5
3])l0(cSqrt
l6
l1
l8
4;}
lE't':{lK'a','n','h'}
;if(lH
cTanh
l6
l1}
l8
4;}
lE
5
lF
0]){lE
lA'c':{lK'o','s','h'}
;if(lC
2,tmp,3
lI
cAcosh
l6
l7's':{lK'i','n','h'}
;if(lC
2,tmp,3
lI
cAsinh
l6
l7't':if('a'l5
2]){if(lL
3]){l9
uptr[4]){lE'2':l0(cAtan2
l6
lG
lE'h':l0(cAtanh
l6
lG
l8
5;}
}
l0
5;}
l0
5;l8
5;}
lE'f':{l2
4]={'l','o','o','r'}
;if(l3
4
lI
cFloor
l6
l7'h':{l2
4]={'y','p','o','t'}
;if(l3
4
lI
cHypot
l6
l7'l':{l2
4]={'o','g','1','0'}
;if(l3
4
lI
cLog10
l6
l7'p':{l2
4]={'o','l','a','r'}
;if(l3
4
lI
cPolar
l6
l7't':{l2
4]={'r','u','n','c'}
;if(l3
4
lI
cTrunc
l6
lG
l0
5;}
l8
5;}
default:break;}
l0
nameLength;