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
/*! --------------------------------------------------------------------
\file   ravep.h
\brief  Defines the private headers that every source file used to
build openrave must include (used in place of rave.h). Precompiled header.
-------------------------------------------------------------------- */

#ifndef RAVE_LIBOPENRAVE_H
#define RAVE_LIBOPENRAVE_H

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

#ifdef _MSC_VER
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

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

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#ifdef _WIN32
#elif defined(__APPLE_CC__)
#define strnicmp strncasecmp
#define stricmp strcasecmp
#else
#define strnicmp strncasecmp
#define stricmp strcasecmp

#endif

#include <boost/format.hpp>
#include <boost/bind.hpp>

struct null_deleter
{
    void operator()(void const *) const {}
};

template <class T> boost::shared_ptr<T> sptr_from(boost::weak_ptr<T> const& wpt)
{
    return boost::shared_ptr<T>(wpt); // throws on wpt.expired()
}

// need the prototypes in order to keep them free of the OpenRAVE namespace
class LinkXMLReader;
class KinBodyXMLReader;
class JointXMLReader;
class RobotXMLReader;
class ManipulatorXMLReader;
class AttachedSensorXMLReader;
class Environment;
class RaveDatabase;

#include <rave/rave.h>
using namespace OpenRAVE;
using namespace std;

#endif
