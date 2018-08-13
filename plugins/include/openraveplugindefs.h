// -*- coding: utf-8 --*
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions
#include <openrave/utils.h>
#include <openrave/plannerparameters.h>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v) ::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v) ::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST
#else

#include <string>
#include <vector>
#include <list>
#include <map>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin())it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace OpenRAVE;

static const dReal g_fEpsilonLinear = RavePow(g_fEpsilon,0.9);
static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.75);
static const dReal g_fEpsilonWorkSpaceLimitSqr = RavePow(g_fEpsilon,0.9);

/// Return the smallest multiple of step larger or equal to T
inline dReal ComputeStepSizeCeiling(dReal T,dReal step)
{
    dReal ratio = T/step;
    dReal ceilratio = RaveCeil(ratio-1e-8);
    return ceilratio*step;
}

/// Check whether T is a multiple of step
inline bool IsMultipleOfStepSize(dReal T,dReal step)
{
    return RaveFabs(T-ComputeStepSizeCeiling(T,step)) <= g_fEpsilonLinear;
}

inline dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
    dReal facos = min((t1.rot-t2.rot).lengthsqr4(),(t1.rot+t2.rot).lengthsqr4());
    return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos; //*facos;
}

inline std::ostream& SerializeValues(std::ostream& O, const std::vector<dReal>& values, char delim=',')
{
    for(size_t i = 0; i < values.size(); ++i) {
        if( i > 0 ) {
            O << delim;
        }
        O << values[i];
    }
    return O;
}

inline std::ostream& SerializeTransform(std::ostream& O, const Transform& t, char delim=',')
{
    O << t.rot.x << delim << t.rot.y << delim << t.rot.z << delim << t.rot.w << delim << t.trans.x << delim << t.trans.y << delim << t.trans.z;
    return O;
}

#endif
