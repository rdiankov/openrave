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
#include <openrave/utils.h>

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

#include <stdint.h>
#include <fstream>
#include <iostream>

#include <boost/assert.hpp>
#include <boost/bind.hpp>

/*
// This section is commented because it may have been copied from an older
// version.  The active code following this was modeled from bulletrave and
// properly compiles.  TODO: Remove this commented code when validated

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave_plugins_mobyrave", msgid)

using namespace std;

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

#endif

//#endif
*/

using namespace std;
using namespace OpenRAVE;

static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);

#endif
