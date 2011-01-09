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

/// \brief search and replace all for all pairs
///
/// \return returns a reference to the out string
std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >& pairs);

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

// if modifying check modify ravep.h too!
inline bool IsValidCharInName(char c) { return isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/'; }
inline bool IsValidName(const std::string& s) {
    if( s.size() == 0 )
        return false;
    return std::count_if(s.begin(), s.end(), IsValidCharInName) == (int)s.size();
}

template<typename T>
struct index_cmp
{
    index_cmp(const T arr) : arr(arr) {}
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
        det = IKsqrt(det);
        rawroots[0] = (-rawcoeffs[1]+det)/(2*rawcoeffs[0]);
        rawroots[1] = (-rawcoeffs[1]-det)/(2*rawcoeffs[0]);//rawcoeffs[2]/(rawcoeffs[0]*rawroots[0]);
        numroots = 2;
    }
}

/// \brief Durand-Kerner polynomial root finding method
template <typename IKReal, int D>
inline void polyroots(const IKReal* rawcoeffs, IKReal* rawroots, int& numroots)
{
    using std::complex;
    IKFAST_ASSERT(rawcoeffs[0] != 0);
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
        if( IKabs(imag(roots[i])) < std::numeric_limits<IKReal>::epsilon() ) {
            rawroots[numroots++] = real(roots[i]);
        }
    }
}

/** \brief Intersection between a conic section and a unit circle

    The general method to solve an intersection of two conics C1 and C2 is to first
    note that any solution to their intersection is also a solution of
    x^T C x = 0, where C = t0*C0 + t1*C1
    for t0, t1 in Reals. Without loss of generality, we set t1 = 1, and find t0 when
    C becomes degenerate, ie det(C) = 0. This produces a cubic equation in t0,
    which gives 4 solutions. Gathering all the equations produced by the degenerate
    conic should give rise to two equations of lines. Intersect these lines with the simpler of the
    two conics, the unit circle: c^2+s^2-1 = 0
*/
template <typename IKReal>
inline void conicsolver(IKReal _C0[6], IKReal roots[4], int& numroots)
{
    numroots = 0;
    // have to normalize _C0
    IKReal maxval = IKabs(_C0[0]);
    for(int i = 1; i < 6; ++i) {
        if( maxval < IKabs(_C0[i]) ) {
            maxval = IKabs(_C0[i]);
        }
    }
    IKReal C0[6];
    for(int i = 0; i < 6; ++i) {
        C0[i]=_C0[i]/maxval;
    }
    IKReal rawcoeffs[4] = {-1,
                           C0[5] - C0[0] - C0[3],
                           C0[0]*C0[5] + C0[3]*C0[5] - C0[0]*C0[3] + C0[1]*C0[1] - C0[2]*C0[2] - C0[4]*C0[4],
                           C0[0]*C0[3]*C0[5] + 2*C0[1]*C0[2]*C0[4] - C0[0]*C0[4]*C0[4] - C0[3]*C0[2]*C0[2] - C0[5]*C0[1]*C0[1]};
    IKReal proots[3];
    int numproots, numyroots;
    polyroots<IKReal,3>(rawcoeffs,proots,numproots);
    if( numproots < 1 ) {
        return;
    }
    int iroot=0;
    IKReal a, b, c, d, e, f;
    a = C0[0]+proots[iroot]; b = C0[1]; c = C0[3]+proots[iroot]; d = C0[2]; e = C0[4]; f = C0[5]-proots[iroot];
    IKReal adjugate[9] = {c*f-e*e, -b*f+e*d, b*e-c*d, -b*f+d*e, a*f-d*d, -a*e+b*d, b*e-d*c, -a*e+d*b, a*c-b*b};
    // find the greatest absolute value of adjugate and take that column
    int maxindex = 0;
    IKReal val = IKabs(adjugate[maxindex]);
    for(int i = 1; i < 9; ++i) {
        IKReal newval = IKabs(adjugate[i]);
        if( val < newval ) {
            val = newval;
            maxindex = i;
        }
    }
    maxindex = maxindex%3;
    if( adjugate[0] > 0 || adjugate[4] > 0 || adjugate[8] > 0 || adjugate[4*maxindex] >= 0 ) {
        // according to the structure of the matrix, should be always negative if a solution exists...
        return;
    }
    IKReal bmult = 1.0/IKsqrt(-adjugate[4*maxindex]);
    IKReal p[3] = {adjugate[maxindex]*bmult, adjugate[3+maxindex]*bmult, adjugate[6+maxindex]*bmult}; // intersection point
    // C = C0 - [p_x] = 2gh^t, C is rank1
    IKReal C[9] = {a,b+p[2],d-p[1],b-p[2],c,e+p[0],d+p[1],e-p[0],f};
    maxindex = 0;
    val = IKabs(C[maxindex]);
    for(int i = 1; i < 9; ++i) {
        IKReal newval = IKabs(C[i]);
        if( val < newval ) {
            val = newval;
            maxindex = i;
        }
    }
    int row = maxindex/3;
    int col = maxindex%3;
    IKReal lineequation[3], coeffs[3], yintersections[2];
    for(int i = 0; i < 2; ++i) {
        if( i == 0 ) {
            lineequation[0] = C[3*row];
            lineequation[1] = C[3*row+1];
            lineequation[2] = C[3*row+2];
        }
        else {
            lineequation[0] = C[col];
            lineequation[1] = C[3+col];
            lineequation[2] = C[6+col];
        }

        if( IKabs(lineequation[0]) < std::numeric_limits<IKReal>::epsilon() ) {
            yintersections[0] = -lineequation[2]/lineequation[1];
            IKReal x = 1-yintersections[0]*yintersections[0];
            if( x <= 0 && x > -std::numeric_limits<IKReal>::epsilon() ) {
                roots[numroots++] = yintersections[0] > 0 ? M_PI_2 : -M_PI_2;
            }
            else {
                x = IKsqrt(x);
                roots[numroots++] = IKatan2(yintersections[0], x);
                roots[numroots] = M_PI - roots[numroots-1]; numroots++;
            }
        }
        else {
            coeffs[0] = lineequation[0]*lineequation[0]+lineequation[1]*lineequation[1];
            coeffs[1] = 2*lineequation[1]*lineequation[2];
            coeffs[2] = lineequation[2]*lineequation[2]-lineequation[0]*lineequation[0];
            polyroots2<IKReal>(coeffs,yintersections,numyroots);
            for(int j = 0; j < numyroots; ++j) {
                // the mathematical solution would be: IKatan2(yintersections[j],-(lineequation[1]*yintersections[j]+lineequation[2])/lineequation[0]);
                // however due to numerical imprecisions, it is better to compute sqrt(1-yintersections[j]*yintersections[j]) and choose sign 
                IKReal x = 1-yintersections[j]*yintersections[j];
                if( x <= 0 ) {
                    if( x > -std::numeric_limits<IKReal>::epsilon() ) {
                        roots[numroots++] = IKatan2(yintersections[j],-(lineequation[1]*yintersections[j]+lineequation[2])/lineequation[0]);
                    }
                }
                else {
                    x = IKsqrt(x);
                    if( (lineequation[1]*yintersections[j]+lineequation[2])/lineequation[0] > 0 ) {
                        x = -x;
                    }
                    roots[numroots++] = IKatan2(yintersections[j],x);
                }
            }
        }
    }
}

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
