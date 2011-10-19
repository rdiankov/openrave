// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
/**
   \file   ravep.h
   \brief  Defines the private headers that every source file used to build openrave must include (used in place of rave.h).
 */

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

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); ++(it))
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); ++(it))
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

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
#include <windows.h>
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
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0)
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

#ifdef _WIN32
static const char s_filesep = '\\';
#else
static const char s_filesep = '/';
#endif

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

// exports from libopenrave.h
namespace OpenRAVE
{
OPENRAVE_API std::string GetMD5HashString(const std::string& s);
OPENRAVE_API std::string GetMD5HashString(const std::vector<uint8_t>& v);
OPENRAVE_API std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >& pairs);

RobotBasePtr CreateGenericRobot(EnvironmentBasePtr penv, std::istream& sinput);
TrajectoryBasePtr CreateGenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput);
PhysicsEngineBasePtr CreateGenericPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput);
CollisionCheckerBasePtr CreateGenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput);
}

using namespace OpenRAVE;
using namespace std;

namespace OpenRAVEXMLParser
{
class InterfaceXMLReadable : public XMLReadable
{
public:
    InterfaceXMLReadable(InterfaceBasePtr pinterface) : XMLReadable(pinterface->GetXMLId()), _pinterface(pinterface) {
    }
    virtual ~InterfaceXMLReadable() {
    }
    InterfaceBasePtr _pinterface;
};

int& GetXMLErrorCount();
void SetDataDirs(const std::vector<std::string>& vdatadirs);
bool ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename);
bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata);
BaseXMLReaderPtr CreateEnvironmentReader(EnvironmentBasePtr penv, const AttributesList& atts);
boost::shared_ptr<std::pair<std::string,std::string> > FindFile(const std::string& filename);
BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, InterfaceType type, InterfaceBasePtr& pinterface, const std::string& xmltag, const AttributesList& atts);
BaseXMLReaderPtr CreateInterfaceReader(EnvironmentBasePtr penv, const AttributesList& atts, bool bAddToEnvironment);
bool CreateTriMeshData(EnvironmentBasePtr, const std::string& filename, const Vector &vscale, KinBody::Link::TRIMESH& trimesh, RaveVector<float>&diffuseColor, RaveVector<float>&ambientColor, float &ftransparency);

bool CreateGeometries(EnvironmentBasePtr penv, const std::string& filename, const Vector& vscale, std::list<KinBody::Link::GEOMPROPERTIES>& listGeometries);

}

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define strnicmp strncasecmp
#define stricmp strcasecmp
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp
#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

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
#include <boost/version.hpp>

template<class P>
struct smart_pointer_deleter
{
private:
    P p_;
    boost::function<void(void const*)> _deleterfn;
public:
    smart_pointer_deleter(P const & p, const boost::function<void(void const*)>& deleterfn) : p_(p), _deleterfn(deleterfn)
    {
    }

    void operator() (void const * x)
    {
        _deleterfn(x);
        p_.reset();
    }

    P const & get() const
    {
        return p_;
    }
};

// if modifying check modify libopenrave.h too!
inline bool IsValidCharInName(char c) {
    return c < 0 || c >= 33; //isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/';
}
inline bool IsValidName(const std::string& s) {
    if( s.size() == 0 ) {
        return false;
    }
    return std::count_if(s.begin(), s.end(), IsValidCharInName) == (int)s.size();
}

inline std::string ConvertToOpenRAVEName(const std::string& name)
{
    if( IsValidName(name) ) {
        return name;
    }
    std::string newname = name;
    for(size_t i = 0; i < newname.size(); ++i) {
        if( !IsValidCharInName(newname[i]) ) {
            newname[i] = '_';
        }
    }
    RAVELOG_WARN(str(boost::format("name '%s' is not a valid OpenRAVE name, converting to '%s'")%name%newname));
    return newname;
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, const std::string& data,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data,const AttributesList& atts);

void RaveWriteColladaFile(EnvironmentBasePtr penv, const std::string& filename);
void RaveWriteColladaFile(KinBodyPtr pbody, const std::string& filename);
void RaveWriteColladaFile(RobotBasePtr probot, const std::string& filename);
void RaveWriteColladaFile(const std::list<KinBodyPtr>& listbodies, const std::list<RobotBasePtr>& listrobots, const std::string& filename);

#endif
