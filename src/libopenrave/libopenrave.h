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

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; ++(it))
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(), __itend__=(v).end(); it != __itend__; )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <algorithm>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(), __itend__=(v).end(); it != __itend__; )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

//template <typename T>
//class openraveconst_iteratorbegin : public T::const_iterator
//{
//public:
//    openraveconst_iteratorbegin(const T & v) : T::const_iterator(v.begin()), _v(v) {
//    }
//    const T & _v;
//};
//
//
//template <typename T>
//class openraveiteratorbegin : public T::iterator
//{
//public:
//    openraveiteratorbegin(const T & v) : T::iterator( const_cast<T&> (v).begin()), _v(v) {
//    }
//    const T & _v;
//};

//#define OPENRAVE_FOREACH(it,v) for( OpenRAVE::openraveiteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )
//#define OPENRAVE_FOREACHC(it,v) for( OpenRAVE::openraveconst_iteratorbegin<typeof(v)> (it) (v); (it) != (it)._v.end(); (it)++ )

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>

#ifdef USE_CRLIBM
#include <crlibm.h> // robust/accurate math
#endif

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

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); ++(it))

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define strnicmp strncasecmp
#define stricmp strcasecmp
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp

#endif

#include <boost/bind.hpp>
#include <boost/version.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

namespace OpenRAVE {

template <typename T>
class TransformSaver
{
public:
    TransformSaver(T plink) : _plink(plink) {
        _t = _plink->GetTransform();
    }
    ~TransformSaver() {
        _plink->SetTransform(_t);
    }
    const Transform& GetTransform() {
        return _t;
    }
private:
    T _plink;
    Transform _t;
};

struct null_deleter
{
    void operator()(void const *) const {
    }
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

OPENRAVE_API std::string GetMD5HashString(const std::string& s);
OPENRAVE_API std::string GetMD5HashString(const std::vector<uint8_t>& v);

/// \brief search and replace strings for all pairs. Internally first checks the longest strings before the shortest
///
/// \return returns a reference to the out string
OPENRAVE_API std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >& pairs);

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
        while (theta < min) {
            theta += T(2*PI);
        }
    }
    else if (theta > max) {
        theta -= T(2*PI);
        while (theta > max) {
            theta -= T(2*PI);
        }
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

// if modifying check modify ravep.h too!
inline bool IsValidCharInName(char c) {
    return c < 0 || c >= 33; //isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/';
}
inline bool IsValidName(const std::string& s) {
    if( s.size() == 0 )
        return false;
    return std::count_if(s.begin(), s.end(), IsValidCharInName) == (int)s.size();
}

template<typename T>
struct index_cmp
{
    index_cmp(const T arr) : arr(arr) {
    }
    bool operator()(const size_t a, const size_t b) const
    {
        return arr[a] < arr[b];
    }
    const T arr;
};

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

template <typename IKReal>
inline void polyroots2(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    IKReal det = rawcoeffs[1]*rawcoeffs[1]-4*rawcoeffs[0]*rawcoeffs[2];
    if( det < 0 ) {
        numroots=0;
    }
    else if( det == 0 ) {
        rawroots[0] = -0.5*rawcoeffs[1]/rawcoeffs[0];
        numroots = 1;
    }
    else {
        det = RaveSqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]); //rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}

/// \brief Durand-Kerner polynomial root finding method
template <typename IKReal, int D>
inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    BOOST_ASSERT(rawcoeffs[0] != 0);
    const IKReal tol = 128.0*std::numeric_limits<IKReal>::epsilon();
    complex<IKReal> coeffs[D];
    const int maxsteps = 50;
    for(int i = 0; i < D; ++i) {
        coeffs[i] = complex<IKReal>(rawcoeffs[i+1]/rawcoeffs[0]);
    }
    complex<IKReal> roots[D];
    IKReal err[D];
    roots[0] = complex<IKReal>(1,0);
    roots[1] = complex<IKReal>(0.4,0.9); // any complex number not a root of unity is works
    err[0] = 1.0;
    err[1] = 1.0;
    for(int i = 2; i < D; ++i) {
        roots[i] = roots[i-1]*roots[1];
        err[i] = 1.0;
    }
    for(int step = 0; step < maxsteps; ++step) {
        bool changed = false;
        for(int i = 0; i < D; ++i) {
            if ( err[i] >= tol ) {
                changed = true;
                // evaluate
                complex<IKReal> x = roots[i] + coeffs[0];
                for(int j = 1; j < D; ++j) {
                    x = roots[i] * x + coeffs[j];
                }
                for(int j = 0; j < D; ++j) {
                    if( i != j ) {
                        if( roots[i] != roots[j] ) {
                            x /= (roots[i] - roots[j]);
                        }
                    }
                }
                roots[i] -= x;
                err[i] = abs(x);
            }
        }
        if( !changed ) {
            break;
        }
    }
    numroots = 0;
    for(int i = 0; i < D; ++i) {
        if( RaveFabs(imag(roots[i])) < std::numeric_limits<IKReal>::epsilon() ) {
            rawroots[numroots++] = real(roots[i]);
        }
    }
}

class TrajectoryReader : public BaseXMLReader
{
public:
    TrajectoryReader(TrajectoryBasePtr ptraj);
    virtual ProcessElement startElement(const std::string& name, const AttributesList& atts);
    virtual bool endElement(const std::string& name);
    virtual void characters(const std::string& ch);

protected:
    TrajectoryBasePtr _ptraj;
    std::stringstream _ss;
    ConfigurationSpecification _spec;
    BaseXMLReaderPtr _pcurreader;
    int _datacount;
    std::vector<dReal> _vdata;
};

namespace LocalXML
{
bool ParseXMLData(BaseXMLReaderPtr preader, const char* buffer, int size);
}

#ifdef _WIN32
inline const char *strcasestr(const char *s, const char *find)
{
    register char c, sc;
    register size_t len;

    if ((c = *find++) != 0) {
        c = tolower((unsigned char)c);
        len = strlen(find);
        do {
            do {
                if ((sc = *s++) == 0) {
                    return (NULL);
                }
            } while ((char)tolower((unsigned char)sc) != c);
        } while (strnicmp(s, find, len) != 0);
        s--;
    }
    return ((char *) s);
}
#endif

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
