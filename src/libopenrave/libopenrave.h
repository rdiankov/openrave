// Copyright (C) 2006-2010 Carnegie Mellon University (rosen.diankov@gmail.com)
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

#ifndef RAVE_LIBOPENRAVE_H
#define RAVE_LIBOPENRAVE_H

#include <rave/rave.h> // should be included first in order to get boost throwing openrave exceptions

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
#include <algorithm>

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
#include <iomanip>

#ifdef USE_CRLIBM
#include <crlibm.h> // robust/accurate math
#endif

#include <sys/timeb.h>    // ftime(), struct timeb

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

inline static uint32_t timeGetTime()
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

inline static uint64_t GetMicroTime()
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

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define strnicmp strncasecmp
#define stricmp strcasecmp
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp

#endif

#include <boost/bind.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

namespace OpenRAVE {

struct null_deleter
{
    void operator()(void const *) const {}
};

template <class T> boost::shared_ptr<T> sptr_from(boost::weak_ptr<T> const& wpt)
{
    return boost::shared_ptr<T>(wpt); // throws on wpt.expired()
}


// returns a lower case version of the string 
inline std::string tolowerstring(const std::string & s)
{
    std::string d = s;
    std::transform(d.begin(), d.end(), d.begin(), ::tolower);
    return d;
}

std::string GetMD5HashString(const std::string& s);
std::string GetMD5HashString(const std::vector<uint8_t>& v);

#define SERIALIZATION_PRECISION 4
template<typename T>
inline T SerializationValue(T f)
{
    return ( f > -1e-4f && f < 1e-4f ) ? static_cast<T>(0) : f;
}

inline void SerializeRound(std::ostream& o, float f)
{
    o << SerializationValue(f) << " ";
}

inline void SerializeRound(std::ostream& o, double f)
{
    o << SerializationValue(f) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " " << SerializationValue(v.w) << " ";
}

template <class T>
inline void SerializeRound3(std::ostream& o, const RaveVector<T>& v)
{
    o << SerializationValue(v.x) << " " << SerializationValue(v.y) << " " << SerializationValue(v.z) << " ";
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransform<T>& t)
{
    SerializeRound(o,t.rot);
    SerializeRound(o,t.trans);
}

template <class T>
inline void SerializeRound(std::ostream& o, const RaveTransformMatrix<T>& t)
{
    o << SerializationValue(t.m[0]) << " " << SerializationValue(t.m[4]) << " " << SerializationValue(t.m[8]) << " "
      << SerializationValue(t.m[1]) << " " << SerializationValue(t.m[5]) << " " << SerializationValue(t.m[9]) << " "
      << SerializationValue(t.m[2]) << " " << SerializationValue(t.m[6]) << " " << SerializationValue(t.m[10]) << " ";
    SerializeRound(o,t.trans);
}

template <typename T>
inline T NORMALIZE_ANGLE(T theta, T min, T max)
{
    if (theta < min) {
        theta += T(2*PI);
        while (theta < min)
            theta += T(2*PI);
    }
    else if (theta > max) {
        theta -= T(2*PI);
        while (theta > max)
            theta -= T(2*PI);
    }
    return theta;
}

template <typename T>
inline T ANGLE_DIFF(T f0, T f1)
{
    return NORMALIZE_ANGLE(f0-f1, T(-PI), T(PI));
}

template <typename T>
inline T ANGLE_INTERPOLATION(T start, T end, T fraction, T lowerLimit, T upperLimit)
{
    return NORMALIZE_ANGLE(start + fraction * ANGLE_DIFF(end,start), lowerLimit, upperLimit);
}

inline static dReal TransformDistanceFast(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*e);
}

void subtractstates(std::vector<dReal>& q1, const std::vector<dReal>& q2);

inline bool IsValidCharInName(char c) { return isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/'; }
 inline bool IsValidName(const std::string& s) {
    if( s.size() == 0 )
        return false;
    return std::count_if(s.begin(), s.end(), IsValidCharInName) == (int)s.size();
}

template<class T> struct index_cmp {
index_cmp(const T arr) : arr(arr) {}
    bool operator()(const size_t a, const size_t b) const
    { return arr[a] < arr[b]; }
    const T arr;
};

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

using namespace OpenRAVE;
using namespace std;

#endif
