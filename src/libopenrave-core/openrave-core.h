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

#include <pthread.h>

namespace OpenRAVE
{
    

/// Creates an OpenRAVE environment.
/// \param bLoadAllPlugins, if true will load all the openrave plugins automatically that can be found in the OPENRAVE_PLUGINS environment path
EnvironmentBase* CreateEnvironment(bool bLoadAllPlugins=true);
RaveServerBase* CreateSimpleTextServer(EnvironmentBase* penv);

/// count of errors since last GetXMLErrorCount() call
int GetXMLErrorCount();

} // end namespace OpenRAVE

#endif
