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
#include "mobycontroller.h"
#include "mobyreplaycontroller.h"

using namespace OpenRAVE;

// create moby physics shared pointer
PhysicsEngineBasePtr CreateMobyPhysics(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new MobyPhysics(penv,sinput));
}

static std::list< OpenRAVE::UserDataPtr >* s_listRegisteredReaders = NULL; ///< have to make it a pointer in order to prevent static object

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( !s_listRegisteredReaders ) {
        s_listRegisteredReaders = new list< OpenRAVE::UserDataPtr >();
        s_listRegisteredReaders->push_back(RaveRegisterXMLReader(OpenRAVE::PT_PhysicsEngine,"mobyphysics",MobyPhysics::CreateXMLReader));
        s_listRegisteredReaders->push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobycontroller",MobyController::CreateXMLReader));
        s_listRegisteredReaders->push_back(RaveRegisterXMLReader(OpenRAVE::PT_Controller,"mobyreplaycontroller",MobyReplayController::CreateXMLReader));
    }

    switch( type ) {
    case PT_PhysicsEngine:
        if( interfacename == "moby" ) 
        {
            return CreateMobyPhysics(penv,sinput);
        }
        break;
    case OpenRAVE::PT_Controller:
        if( interfacename == "moby") {
            return InterfaceBasePtr(new MobyController(penv,sinput));
        }
        else if( interfacename == "mobyreplay") {
            return InterfaceBasePtr(new MobyReplayController(penv,sinput));
        }
        break;
    default:
        break;
    }
    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_PhysicsEngine].push_back("moby");
    info.interfacenames[OpenRAVE::PT_Controller].push_back("moby");
    info.interfacenames[OpenRAVE::PT_Controller].push_back("mobyreplay"); 
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    delete s_listRegisteredReaders;
    s_listRegisteredReaders = NULL;
}

