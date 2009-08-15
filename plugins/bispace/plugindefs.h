// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.


//This file contains includes of necessary headers and several useful macros and functions
#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <assert.h>
#include <cstdio>
#include <cmath>

#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC FOREACH

#endif

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdint.h>

#ifdef _MSC_VER
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

using namespace std;

#include <sys/timeb.h>    // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline unsigned long timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (unsigned long)(t.time*1000+t.millitm);
}

inline uint64_t GetCPUTicks()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}


#ifndef ARRAYSIZE
#define ARRAYSIZE(x) (sizeof(x)/(sizeof( (x)[0] )))
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifdef _WIN32

#define WCSTOK(str, delim, ptr) wcstok(str, delim)

// define wcsicmp for MAC OS X
#elif defined(__APPLE_CC__)

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr);

#define strnicmp strncasecmp
#define stricmp strcasecmp

inline int wcsicmp(const wchar_t* s1, const wchar_t* s2)
{
  char str1[128], str2[128];
  sprintf(str1, "%S", s1);
  sprintf(str2, "%S", s2);
  return stricmp(str1, str2);
}


#else

#define WCSTOK(str, delim, ptr) wcstok(str, delim, ptr)

#define strnicmp strncasecmp
#define stricmp strcasecmp
#define wcsnicmp wcsncasecmp
#define wcsicmp wcscasecmp

#endif

#include <rave/rave.h>
using namespace OpenRAVE;

//#define RANDOM_FLOAT g_pEnviron->RandomFloat
//
//inline int RANDOM_INT(int maximum)
//{
//    return ((u32)g_pEnviron->RandomInt() % maximum);
//}

inline float RANDOM_FLOAT()
{
#if defined(__IRIX__)
    return drand48();
#else
    return rand()/((float)RAND_MAX);
#endif
}

inline int RANDOM_INT(int maximum)
{
#if defined(__IRIX__)
    return (random() % maximum);
#else
    return (rand() % maximum);
#endif
}

// sorted by increasing getvalue
template <class T, class S>
class BinarySearchTree
{
public:
    BinarySearchTree() { Reset(); }

    // other global definitions
    void Add(T& pex)
    {
        assert( pex != NULL );
        
        switch(blocks.size()) {
		    case 0:
                blocks.push_back(pex);
                return;
		    case 1:
                if( blocks.front()->getvalue() < pex->getvalue() ) {
                    blocks.push_back(pex);
                }
                else blocks.insert(blocks.begin(), pex);
                
                return;
                
		    default: {
                int imin = 0, imax = (int)blocks.size(), imid;
                
                while(imin < imax) {
                    imid = (imin+imax)>>1;
                    
                    if( blocks[imid]->getvalue() > pex->getvalue() ) imax = imid;
                    else imin = imid+1;
                }
                
                blocks.insert(blocks.begin()+imin, pex);
                return;
            }
        }
    }
    
    ///< returns the index into blocks
    int Get(S& s)
    {
        switch(blocks.size()) {
		    case 1: return 0;
		    case 2: return blocks.front()->getvalue() < s;    
		    default: {
                int imin = 0, imax = blocks.size()-1, imid;
                
                while(imin < imax) {
                    imid = (imin+imax)>>1;
                    
                    if( blocks[imid]->getvalue() > s ) imax = imid;
                    else if( blocks[imid]->getvalue() == s ) return imid;
                    else imin = imid+1;
                }
                
                return imin;
            }
        }
    }

    void Reset()
    {
        blocks.clear();
        blocks.reserve(1<<16);
    }
    
	vector<T> blocks;
};

#include "profiler.h"

#endif
