// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/*! --------------------------------------------------------------------
\file   ravep.h
\brief  Defines the private headers that every source file used to
build openrave must include (used in place of rave.h). Precompiled header.
-------------------------------------------------------------------- */

#ifndef RAVE_RAVEP_H
#define RAVE_RAVEP_H

#define RAVE_PRIVATE

#include <cstdio>
#include <cmath>
#include <cstdlib>

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

#include <time.h>

#ifndef _WIN32
#if POSIX_TIMERS <= 0 && _POSIX_TIMERS <= 0
#include <sys/time.h>
#endif
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <sys/timeb.h>    // ftime(), struct timeb
#endif

#ifdef _WIN32
inline static uint32_t GetMilliTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (uint32_t)((count.QuadPart * 1000) / freq.QuadPart);
}

inline static uint64_t GetMicroTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
}

inline static uint64_t GetNanoTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000000) / freq.QuadPart;
}

#else

inline static void getWallTime(uint32_t& sec, uint32_t& nsec)
{
#if POSIX_TIMERS > 0 || _POSIX_TIMERS > 0
  struct timespec start;
  clock_gettime(CLOCK_REALTIME, &start);
  sec  = start.tv_sec;
  nsec = start.tv_nsec;
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  sec  = timeofday.tv_sec;
  nsec = timeofday.tv_usec * 1000;
#endif
}

inline static uint64_t GetNanoTime()
{
    uint32_t sec,nsec;
    getWallTime(sec,nsec);
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
}

inline static uint64_t GetMicroTime()
{
    uint32_t sec,nsec;
    getWallTime(sec,nsec);
    return (uint64_t)sec*1000000 + (uint64_t)nsec/1000;
}

inline static uint32_t GetMilliTime()
{
    uint32_t sec,nsec;
    getWallTime(sec,nsec);
    return (uint64_t)sec*1000 + (uint64_t)nsec/1000000;
}

#endif

// returns a lower case version of the string 
inline std::string tolowerstring(const std::string & s)
{
    std::string d = s;
    std::transform(d.begin(), d.end(), d.begin(), ::tolower);
    return d;
}

// need the prototypes in order to keep them free of the OpenRAVE namespace
namespace OpenRAVEXMLParser
{
    class InterfaceXMLReader;
    class KinBodyXMLReader;
    class LinkXMLReader;
    class JointXMLReader;
    class ManipulatorXMLReader;
    class AttachedSensorXMLReader;
    class RobotXMLReader;
    class EnvironmentXMLReader;
}

class Environment;
class RaveDatabase;
class ColladaReader;
class ColladaWriter;

#include "openrave-core.h"

using namespace OpenRAVE;
using namespace std;

bool ParseDirectories(const char* pdirs, std::vector<std::string>& vdirs);

#ifdef OPENRAVE_COIN3D

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/nodes/SoSeparator.h>

/// Triangles SoNode and fills a dTriMeshDataID structure
void CreateTriMeshData(SoNode* pnode, KinBody::Link::TRIMESH& tri);

#endif

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define strnicmp strncasecmp
#define stricmp strcasecmp
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assert.hpp>

template<class P>
struct smart_pointer_deleter
{
private:
    P p_;
    boost::function<void(void const*)> _deleterfn;
public:
smart_pointer_deleter(P const & p, const boost::function<void(void const*)>& deleterfn): p_(p), _deleterfn(deleterfn)
    {
    }

    void operator()(void const * x)
    {
        _deleterfn(x);
        p_.reset();
    }
    
    P const & get() const
    {
        return p_;
    }
};

bool RaveParseColladaFile(EnvironmentBasePtr penv, const std::string& filename);
bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename);
bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename);
bool RaveParseColladaData(EnvironmentBasePtr penv, const std::string& data);
bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data);
bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data);

bool RaveWriteColladaFile(EnvironmentBasePtr penv, const std::string& filename);
bool RaveWriteColladaFile(KinBodyPtr pbody, const std::string& filename);
bool RaveWriteColladaFile(RobotBasePtr probot, const std::string& filename);

#include "ivcon.h"
#include "xmlreaders.h"
#include "plugindatabase.h"
#include "environment.h"

#endif
