// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
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
#include "plugindefs.h"
#include "baselaser.h"
#include "baseflashlidar3d.h"
#include "basecamera.h"

// need c linkage
extern "C" {

static list< boost::shared_ptr<void> > s_listRegisteredReaders;
InterfaceBasePtr CreateInterface(PluginType type, const std::string& name, const char* pluginhash, EnvironmentBasePtr penv)
{
    if( strcmp(pluginhash,RaveGetInterfaceHash(type)) ) {
        RAVELOG_WARNA("plugin type hash is wrong\n");
        throw openrave_exception("bad plugin hash");
    }
    if( !penv )
        return InterfaceBasePtr();
    
    if( s_listRegisteredReaders.size() == 0 ) {
        s_listRegisteredReaders.push_back(penv->RegisterXMLReader(PT_Sensor,"baselaser2d",BaseLaser2DSensor::CreateXMLReader));
        s_listRegisteredReaders.push_back(penv->RegisterXMLReader(PT_Sensor,"basespinninglaser2d",BaseSpinningLaser2DSensor::CreateXMLReader));
        s_listRegisteredReaders.push_back(penv->RegisterXMLReader(PT_Sensor,"baseflashlidar3d",BaseFlashLidar3DSensor::CreateXMLReader));
        s_listRegisteredReaders.push_back(penv->RegisterXMLReader(PT_Sensor,"basecamera",BaseCameraSensor::CreateXMLReader));
    }

    stringstream ss(name);
    string interfacename;
    ss >> interfacename;
    std::transform(interfacename.begin(), interfacename.end(), interfacename.begin(), ::tolower);

    switch(type) {
    case PT_Sensor:
        if( interfacename == "baselaser2d" )
            return InterfaceBasePtr(new BaseLaser2DSensor(penv));
        else if( interfacename == "basespinninglaser2d" )
            return InterfaceBasePtr(new BaseSpinningLaser2DSensor(penv));
        else if( interfacename == "baseflashlidar3d" )
            return InterfaceBasePtr(new BaseFlashLidar3DSensor(penv));
        else if( interfacename == "basecamera" )
            return InterfaceBasePtr(new BaseCameraSensor(penv));
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

bool GetPluginAttributes(PLUGININFO* pinfo, int size)
{
    if( pinfo == NULL ) return false;
    if( size != sizeof(PLUGININFO) ) {
        RAVELOG_ERRORA("bad plugin info sizes %d != %d\n", size, sizeof(PLUGININFO));
        return false;
    }

    // fill pinfo
    pinfo->interfacenames[OpenRAVE::PT_Sensor].push_back("BaseLaser2D");
    pinfo->interfacenames[OpenRAVE::PT_Sensor].push_back("BaseSpinningLaser2D");
    pinfo->interfacenames[OpenRAVE::PT_Sensor].push_back("BaseFlashLidar3D");
    pinfo->interfacenames[OpenRAVE::PT_Sensor].push_back("BaseCamera");
    return true;
}

void DestroyPlugin()
{
    s_listRegisteredReaders.clear();
}

}
