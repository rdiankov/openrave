// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#ifndef OPENRAVE_QTCOIN_H
#define OPENRAVE_QTCOIN_H

/// functions that allow plugins to program for the RAVE simulator
#include <assert.h>
#include <cstdio>
#include <cmath>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include <sys/timeb.h>    // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

//#pragma warning(disable:4996) // 'function': was declared deprecated
//#pragma warning(disable:4267) // conversion from 'size_t' to 'type', possible loss of data
//#pragma warning(disable:4018) // '<' : signed/unsigned mismatch

inline uint32_t timeGetTime()
{
#ifdef _WIN32
    _timeb t;
    _ftime(&t);
#else
    timeb t;
    ftime(&t);
#endif

    return (uint32_t)(t.time*1000+t.millitm);
}

inline uint64_t GetMicroTime()
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

#ifndef C_ASSERT
#define C_ASSERT(e) typedef char __C_ASSERT__[(e)?1:-1]
#endif

inline int RANDOM_INT(int maximum)
{
#if defined(__IRIX__)
    return (random() % maximum);
#else
    return (rand() % maximum);
#endif
}

inline float RANDOM_FLOAT()
{
#if defined(__IRIX__)
    return drand48();
#else
    return rand()/((float)RAND_MAX);
#endif
}

inline float RANDOM_FLOAT(float maximum)
{
#if defined(__IRIX__)
    return (drand48() * maximum);
#else
    return (RANDOM_FLOAT() * maximum);
#endif
}

#include <rave/rave.h>
#include <pthread.h>

using namespace OpenRAVE;
using namespace std;

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

inline char* _ravestrdup(const char* pstr)
{
    if( pstr == NULL ) return NULL;
    size_t len = strlen(pstr);
    char* p = new char[len+1];
    memcpy(p, pstr, len*sizeof(char));
    p[len] = 0;
    return p;
}

// Common functions and defines
inline wchar_t* _ravewcsdup(const wchar_t* pstr)
{
    if( pstr == NULL ) return NULL;
    size_t len = wcslen(pstr);
    wchar_t* p = new wchar_t[len+1];
    memcpy(p, pstr, len*sizeof(wchar_t));
    p[len] = 0;
    return p;
}

inline std::wstring _ravembstowcs(const char* pstr)
{
    size_t len = mbstowcs(NULL, pstr, 0);
    std::wstring w; w.resize(len);
    mbstowcs(&w[0], pstr, len);
    return w;
}

inline std::string _stdwcstombs(const wchar_t* pname)
{
    if( pname == NULL )
        return std::string();

    std::string s;
    size_t len = wcstombs(NULL, pname, 0);
    if( len != (size_t)-1 ) {
        s.resize(len);
        wcstombs(&s[0], pname, len);
    }

    return s;
}

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// declaring variables with stdcall can be a little complex
#ifdef _MSC_VER

#define DECLPTR_STDCALL(name, paramlist) (__stdcall *name)paramlist

#else

#ifdef __x86_64__
#define DECLPTR_STDCALL(name, paramlist) (*name) paramlist
#else
#define DECLPTR_STDCALL(name, paramlist) (__attribute__((stdcall)) *name) paramlist
#endif

#endif // _MSC_VER

#include <pthread.h>

class MutexLock
{
public:
    MutexLock(pthread_mutex_t* pmutex) : _pmutex(pmutex) { pthread_mutex_lock(_pmutex); }
    ~MutexLock() { pthread_mutex_unlock(_pmutex); }
    pthread_mutex_t* _pmutex;
};

#include <Inventor/SbColor.h>
#include <Inventor/SoDB.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/SoPath.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/draggers/SoTransformBoxDragger.h>
#include <Inventor/draggers/SoTrackballDragger.h>
#include <Inventor/draggers/SoScaleUniformDragger.h>
#include <Inventor/draggers/SoRotateCylindricalDragger.h>
#include <Inventor/draggers/SoTranslate2Dragger.h>
#include <Inventor/fields/SoSFTime.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoFont.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoIndexedNurbsCurve.h>
#include <Inventor/nodes/SoIndexedNurbsSurface.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoNurbsSurface.h>
#include <Inventor/nodes/SoNurbsCurve.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoTransparencyType.h>
#include <Inventor/nodes/SoVertexShape.h>

#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/sensors/SoTimerSensor.h>

#if QT_VERSION >= 0x040000 // check for qt4
#include <QtGui>
#endif

/// returns the Transform from a Coin3D SoTransform object
inline RaveTransform<float> GetRaveTransform(const SoTransform* ptrans)
{
    RaveTransform<float> t;
    assert( ptrans != NULL );
    const float* q = ptrans->rotation.getValue().getValue();
    t.rot = Vector(q[3], q[0], q[1], q[2]);
    SbVec3f v = ptrans->translation.getValue();
    t.trans = Vector(v[0], v[1], v[2]);
    return t;
}

/// sets the transform of a Coin3D SoTransform object
inline void SetSoTransform(SoTransform* ptrans, const RaveTransform<float>& t)
{
    ptrans->rotation.setValue(t.rot.y, t.rot.z, t.rot.w, t.rot.x);
    ptrans->translation.setValue(t.trans.x, t.trans.y, t.trans.z);
}

//@{ video recording
bool START_AVI(const char* file_name, int _frameRate, int width, int height, int bits);
bool ADD_FRAME_FROM_DIB_TO_AVI(void* pdata);
bool STOP_AVI();
//@}

class QtCoinViewer;

#include "Item.h"
#include "IvSelector.h"
#include "qtcoinviewer.h"

#endif
