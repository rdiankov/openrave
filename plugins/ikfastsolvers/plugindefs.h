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
#define _USE_MATH_DEFINES
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
#include <cmath>

#include <boost/assert.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

using namespace std;
using namespace OpenRAVE;

static const dReal g_fEpsilonJointLimit = RavePow(g_fEpsilon,0.8);

#define IKFAST_ASSERT BOOST_ASSERT
#define IKFAST_REAL double
#define IKFAST_NO_MAIN

#include "ikfast.h"

IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<float> > ikfunctions, const std::vector<dReal>& vfreeinc);
IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<double> > ikfunctions, const std::vector<dReal>& vfreeinc);

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TEMPLATE(IkSingleDOFSolutionBase, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(IkSolution, 1)
BOOST_TYPEOF_REGISTER_TEMPLATE(IkSolutionList, 1)
#endif

#endif
