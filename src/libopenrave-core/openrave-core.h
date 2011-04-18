// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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

// public OpenRAVE header
#include <openrave/openrave.h>

#if defined(OPENRAVE_CORE_DLL)
  #ifdef OPENRAVE_CORE_DLL_EXPORTS
    #define OPENRAVE_CORE_API OPENRAVE_HELPER_DLL_EXPORT
  #else
    #define OPENRAVE_CORE_API OPENRAVE_HELPER_DLL_IMPORT
  #endif
  #define OPENRAVE_CORE_LOCAL OPENRAVE_HELPER_DLL_LOCAL
#else
  #define OPENRAVE_CORE_API
  #define OPENRAVE_CORE_LOCAL
#endif

namespace OpenRAVE
{

/// Creates an OpenRAVE environment.
/// \param bLoadAllPlugins passed into \ref RaveInitialize
OPENRAVE_CORE_API EnvironmentBasePtr RaveCreateEnvironment();

/// \deprecated (10/09/23) see \ref RaveCreateEnvironment
OPENRAVE_CORE_API EnvironmentBasePtr CreateEnvironment(bool bLoadAllPlugins=true) RAVE_DEPRECATED;

} // end namespace OpenRAVE

#endif
