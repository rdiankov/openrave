// -*- coding: utf-8 -*-
// Copyright (c) 2015 James Taylor, Rosen Diankov
// This program is free software: you can redistribute it and/or modify
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

#include "mobyphysics.h"
using namespace OpenRAVE;

// create moby physics shared pointer
PhysicsEngineBasePtr CreateMobyPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new MobyPhysicsEngine(penv,sinput));
}



// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_PhysicsEngine && interfacename == "moby" ) {
      return CreateMobyPhysicsEngine(penv,sinput);
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_PhysicsEngine].push_back("moby");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

