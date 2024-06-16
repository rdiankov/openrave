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
#include "basesensors.h"

#include "plugindefs.h"
#include "baselaser.h"
#include "baseflashlidar3d.h"
#include "basecamera.h"
#include "baseforce6d.h"

const std::string BaseSensorsPlugin::_pluginname = "BaseSensorsPlugin";

BaseSensorsPlugin::BaseSensorsPlugin()
{
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"baselaser2d",BaseLaser2DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"base_laser2d",BaseLaser2DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"basespinninglaser2d",BaseSpinningLaser2DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"baseflashlidar3d",BaseFlashLidar3DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"base_laser3d",BaseFlashLidar3DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"basecamera",BaseCameraSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"base_pinhole_camera",BaseCameraSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"baseforce6d",BaseForce6DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Sensor,"base_force6d",BaseForce6DSensor::CreateXMLReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"baselaser2d",BaseLaser2DSensor::CreateJSONReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"base_laser2d",BaseLaser2DSensor::CreateJSONReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"basecamera",BaseCameraSensor::CreateJSONReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"base_pinhole_camera",BaseCameraSensor::CreateJSONReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"baseforce6d",BaseForce6DSensor::CreateJSONReader));
    s_listRegisteredReaders.push_back(RaveRegisterJSONReader(PT_Sensor,"base_force6d",BaseForce6DSensor::CreateJSONReader));
    _interfaces[OpenRAVE::PT_Sensor].push_back("BaseLaser2D");
    _interfaces[OpenRAVE::PT_Sensor].push_back("base_laser2d");
    _interfaces[OpenRAVE::PT_Sensor].push_back("BaseSpinningLaser2D");
    _interfaces[OpenRAVE::PT_Sensor].push_back("BaseFlashLidar3D");
    _interfaces[OpenRAVE::PT_Sensor].push_back("base_laser3d");
    _interfaces[OpenRAVE::PT_Sensor].push_back("BaseCamera");
    _interfaces[OpenRAVE::PT_Sensor].push_back("base_pinhole_camera");
    _interfaces[OpenRAVE::PT_Sensor].push_back("BaseForce6D");
    _interfaces[OpenRAVE::PT_Sensor].push_back("base_force6d");
}

BaseSensorsPlugin::~BaseSensorsPlugin() {}

OpenRAVE::InterfaceBasePtr BaseSensorsPlugin::CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    switch(type) {
    case PT_Sensor:
        if((interfacename == "baselaser2d")||(interfacename == "base_laser2d")) {
            return boost::make_shared<BaseLaser2DSensor>(penv);
        }
        else if( interfacename == "basespinninglaser2d" ) {
            return boost::make_shared<BaseSpinningLaser2DSensor>(penv);
        }
        else if((interfacename == "baseflashlidar3d")||(interfacename == "base_laser3d")) {
            return boost::make_shared<BaseFlashLidar3DSensor>(penv);
        }
        else if((interfacename == "basecamera")||(interfacename == "base_pinhole_camera")) {
            return boost::make_shared<BaseCameraSensor>(penv);
        }
        else if((interfacename == "baseforce6d")||(interfacename == "base_force6d")) {
            return boost::make_shared<BaseForce6DSensor>(penv);
        }
        break;
    default:
        break;
    }
    return OpenRAVE::InterfaceBasePtr();
}

const RavePlugin::InterfaceMap& BaseSensorsPlugin::GetInterfaces() const
{
    return _interfaces;
}

const std::string& BaseSensorsPlugin::GetPluginName() const
{
    return _pluginname;
}

#if !OPENRAVE_STATIC_PLUGINS

OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin() {
    return new BaseSensorsPlugin();
}

#endif // OPENRAVE_STATIC_PLUGINS