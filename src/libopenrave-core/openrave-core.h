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

#ifndef OPENRAVE_CORE_H
#define OPENRAVE_CORE_H

// public headers
#include <rave/rave.h>

#if defined(_MSC_VER) && defined(RAVE_CORE_USEDLL)
#ifdef RAVE_CORE_LIBBUILD
#define RAVE_CORE_API __declspec(dllexport)
#else
#define RAVE_CORE_API __declspec(dllimport)
#endif
#else
#define RAVE_CORE_API 
#endif

namespace OpenRAVE
{

/// Creates an OpenRAVE environment.
/// \param bLoadAllPlugins passed into \ref RaveInitialize
RAVE_CORE_API EnvironmentBasePtr RaveCreateEnvironment();

/// \deprecated (10/09/23) see \ref RaveCreateEnvironment
RAVE_CORE_API EnvironmentBasePtr CreateEnvironment(bool bLoadAllPlugins=true) RAVE_DEPRECATED;

} // end namespace OpenRAVE

#endif
