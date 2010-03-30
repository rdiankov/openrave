// Copyright (C) 2006-2010 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#include "plugindefs.h"
#include "genericrobot.h"
#include "collisionmaprobot.h"

static boost::shared_ptr<void> s_RegisteredReader;

RAVE_PLUGIN_API InterfaceBasePtr CreateInterface(PluginType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
{
    if( strcmp(pluginhash,RaveGetInterfaceHash(type)) ) {
        RAVELOG_WARNA("plugin type hash is wrong\n");
        throw openrave_exception("bad plugin hash");
    }
    if( !penv )
        return InterfaceBasePtr();

    if( !s_RegisteredReader ) {
        /// as long as this pointer is valid, the reader will remain registered
        s_RegisteredReader = penv->RegisterXMLReader(PT_Robot,"collisionmap",CollisionMapRobot::CreateXMLReader);
    }
    
    stringstream ss(name);
    string interfacename;
    ss >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);

    switch(type) {
    case PT_Robot:
        if( interfacename == "genericrobot")
            return InterfaceBasePtr(new GenericRobot(penv));
        else if( interfacename == "collisionmaprobot")
            return InterfaceBasePtr(new CollisionMapRobot(penv));
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

RAVE_PLUGIN_API bool GetPluginAttributes(PLUGININFO* pinfo, int size)
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->interfacenames[PT_Robot].push_back("GenericRobot");
    pinfo->interfacenames[PT_Robot].push_back("CollisionMapRobot");
    return true;
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    s_RegisteredReader.reset(); // unregister the reader
}
