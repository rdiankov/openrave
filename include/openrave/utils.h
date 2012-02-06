// -*- coding: utf-8 -*-
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

/** \file utils.h
    \brief Programming related utilities likes tokenizers, timers, name checkers, etc.

    This file is optional and not automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_UTILS_H
#define OPENRAVE_UTILS_H

#include <openrave/openrave.h>

#include <time.h>

#ifndef _WIN32
#if POSIX_TIMERS <= 0 && _POSIX_TIMERS <= 0
#include <sys/time.h>
#endif
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <sys/timeb.h>    // ftime(), struct timeb
#define usleep(microseconds) Sleep((microseconds+999)/1000)
#endif

#include <bitset>

namespace OpenRAVE {
namespace utils {

#ifdef _WIN32
inline uint32_t GetMilliTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (uint32_t)((count.QuadPart * 1000) / freq.QuadPart);
}

inline uint64_t GetMicroTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
}

inline uint64_t GetNanoTime()
{
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000000) / freq.QuadPart;
}

inline static uint64_t GetNanoPerformanceTime() {
    return GetNanoTime();
}

#else

inline void GetWallTime(uint32_t& sec, uint32_t& nsec)
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

inline uint64_t GetNanoTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
}

inline uint64_t GetMicroTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000000 + (uint64_t)nsec/1000;
}

inline uint32_t GetMilliTime()
{
    uint32_t sec,nsec;
    GetWallTime(sec,nsec);
    return (uint64_t)sec*1000 + (uint64_t)nsec/1000000;
}

inline static uint64_t GetNanoPerformanceTime()
{
#if defined(CLOCK_GETTIME_FOUND) && (POSIX_TIMERS > 0 || _POSIX_TIMERS > 0) && defined(_POSIX_MONOTONIC_CLOCK)
    struct timespec start;
    uint32_t sec, nsec;
    clock_gettime(CLOCK_MONOTONIC, &start);
    sec  = start.tv_sec;
    nsec = start.tv_nsec;
    return (uint64_t)sec*1000000000 + (uint64_t)nsec;
#else
    return GetNanoTime();
#endif
}

#endif

struct null_deleter
{
    void operator()(void const *) const {
    }
};

template <class T> boost::shared_ptr<T> sptr_from(boost::weak_ptr<T> const& wpt)
{
    return boost::shared_ptr<T>(wpt); // throws on wpt.expired()
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

/// \brief returns a lower case version of the string
inline std::string ConvertToLowerCase(const std::string & s)
{
    std::string d = s;
    std::transform(d.begin(), d.end(), d.begin(), ::tolower);
    return d;
}

/** \brief separates the directories from a string and returns them in a vector

    from http://stackoverflow.com/questions/5505965/fast-string-splitting-with-multiple-delimiters

    Usage:
   \code
   std::vector<std::string> vstrings;
   TokenizeString("0.141 0.51411", " \n\t", vstrings);

   // store begin and end positions (~8x faster)
   typedef boost::iterator_range< std::string::const_iterator > string_view;
   std::vector<string_view> vsv;
   TokenizeString("0.141 0.51411", " \n\t", vsv);
   \endcode

 */
template<typename C>
inline void TokenizeString(std::string const& s, char const* d, C& ret)
{
    C output;
    std::bitset<255> delims;
    while( *d ) {
        unsigned char code = *d++;
        delims[code] = true;
    }
    std::string::const_iterator beg;
    bool in_token = false;
    for( std::string::const_iterator it = s.begin(), end = s.end(); it != end; ++it ) {
        if( delims[*it] ) {
            if( in_token ) {
                output.push_back(typename C::value_type(beg, it));
                in_token = false;
            }
        }
        else if( !in_token ) {
            beg = it;
            in_token = true;
        }
    }
    if( in_token ) {
        output.push_back(typename C::value_type(beg, s.end()));
    }
    output.swap(ret);
}

/// \brief gets the string up the next separator and strips it of leading whitespace.
///
/// If separator is not present, will return entire string
OPENRAVE_API std::string GetFilenameUntilSeparator(std::istream& sinput, char separator);

/// \brief search and replace strings for all pairs. Internally first checks the longest strings before the shortest
///
/// \return returns a reference to the out string
OPENRAVE_API std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >& pairs);

/// \brief compute the md5 hash of a string
OPENRAVE_API std::string GetMD5HashString(const std::string& s);
/// \brief compute the md5 hash of an array
OPENRAVE_API std::string GetMD5HashString(const std::vector<uint8_t>& v);

template<class T>
inline T ClampOnRange(T value, T min, T max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

template <typename T>
inline T NormalizeCircularAngle(T theta, T min, T max)
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
inline T SubtractCircularAngle(T f0, T f1)
{
    return NormalizeCircularAngle(f0-f1, T(-PI), T(PI));
}

template <typename T>
inline T InterpolateCircularAngle(T start, T end, T fraction, T lowerLimit, T upperLimit)
{
    return NormalizeCircularAngle(start + fraction * SubtractCircularAngle(end,start), lowerLimit, upperLimit);
}

template <typename T>
inline T Sqr(T t) {
    return t*t;
}

/// \brief openrave valid characters to be used in names
inline bool IsValidCharInName(char c) {
    return c < 0 || c >= 33; //isalnum(c) || c == '_' || c == '-' || c == '.' || c == '/';
}

/// \brief openrave valid characters to be used in names
inline bool IsValidName(const std::string& s) {
    if( s.size() == 0 ) {
        return false;
    }
    return std::count_if(s.begin(), s.end(), IsValidCharInName) == (int)s.size();
}

/// \brief converts improper characters to _ so the entire name is valid
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
    RAVELOG_WARN(boost::str(boost::format("name '%s' is not a valid OpenRAVE name, converting to '%s'")%name%newname));
    return newname;
}

} // utils
} // OpenRAVE

#endif
