// -*- coding: utf-8 --*
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
#ifndef OPENRAVE_PLUGINDEFS_H
#define OPENRAVE_PLUGINDEFS_H

#include <openrave/openrave.h> // should be included first in order to get boost throwing openrave exceptions

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

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>

using namespace std;

template<class T>
inline T CLAMP_ON_RANGE(T value, T min, T max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

// OpenRAVE includes a dReal typedef which could cause conflict with ODEs, so don't include it
using OpenRAVE::BaseXMLReader;
using OpenRAVE::BaseXMLReaderPtr;
using OpenRAVE::EnvironmentBase;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::KinBody;
using OpenRAVE::KinBodyPtr;
using OpenRAVE::KinBodyWeakPtr;
using OpenRAVE::KinBodyConstPtr;
using OpenRAVE::RobotBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::RobotBaseConstPtr;
using OpenRAVE::RaveTransform;
using OpenRAVE::RaveTransformMatrix;
using OpenRAVE::RaveVector;
using OpenRAVE::Vector;
using OpenRAVE::Transform;
using OpenRAVE::TransformConstPtr;
using OpenRAVE::TransformMatrix;
using OpenRAVE::CollisionReport;
using OpenRAVE::CollisionReportPtr;
using OpenRAVE::RAY;
using OpenRAVE::InterfaceType;
using OpenRAVE::InterfaceBase;
using OpenRAVE::InterfaceBasePtr;
using OpenRAVE::InterfaceBaseConstPtr;
using OpenRAVE::PLUGININFO;
using OpenRAVE::openrave_exception;
using OpenRAVE::EnvironmentMutex;
using OpenRAVE::RaveFabs;
using OpenRAVE::ControllerBase;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::TrajectoryBaseConstPtr;
using OpenRAVE::ControllerBase;
using OpenRAVE::AttributesList;

// define ODE_LIB for static linking
#include <ode/ode.h>

// needed for ODE 0.10+
#if defined(NEED_DTRIINDEX_TYPEDEF)
typedef int dTriIndex;
#endif

template <class T> boost::shared_ptr<T> sptr_from(boost::weak_ptr<T> const& wpt)
{
    return boost::shared_ptr<T>(wpt); // throws on wpt.expired()
}

#endif
