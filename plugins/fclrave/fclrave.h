// -*- coding: utf-8 -*-
// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_PLUGIN_FCLRAVE_H
#define OPENRAVE_PLUGIN_FCLRAVE_H

#include "plugindefs.h"
#include "fclcollision.h"
#include <openrave/plugin.h>

struct FCLRavePlugin : public RavePlugin {
    FCLRavePlugin()
    {
        _interfaces[OpenRAVE::PT_CollisionChecker].push_back("fcl_");
    }

    ~FCLRavePlugin() override {}

    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv) override
    {
        if( type == OpenRAVE::PT_CollisionChecker && interfacename == "fcl_" ) {
            return boost::make_shared<fclrave::FCLCollisionChecker>(penv, sinput);
        }
        return OpenRAVE::InterfaceBasePtr();
    }

    const InterfaceMap& GetInterfaces() const override
    {
        return _interfaces;
    }

    const std::string& GetPluginName() const override
    {
        static std::string pluginname = "FCLRavePlugin";
        return pluginname;
    }

private:
    InterfaceMap _interfaces;
};

#endif // OPENRAVE_PLUGIN_FCLRAVE_H