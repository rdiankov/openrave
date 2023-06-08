// -*- coding: utf-8 -*-
// Copyright (C) 2022 Tan Li Boon
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

#if OPENRAVE_STATIC_PLUGINS

#include "plugindatabase_virtual.h"

#include "baserobots.h"
#include "basecontrollers.h"
#include "basesamplers.h"
#include "basesensors.h"
#include "fclrave.h"

namespace OpenRAVE {

void StaticRaveDatabase::Init()
{
    _vPlugins.emplace_back(boost::make_shared<BaseRobotsPlugin>());      _vPlugins.back()->SetPluginPath("__static__");
    _vPlugins.emplace_back(boost::make_shared<BaseControllersPlugin>()); _vPlugins.back()->SetPluginPath("__static__");
    _vPlugins.emplace_back(boost::make_shared<BaseSensorsPlugin>());     _vPlugins.back()->SetPluginPath("__static__");
    _vPlugins.emplace_back(boost::make_shared<BaseSamplersPlugin>());    _vPlugins.back()->SetPluginPath("__static__");
    _vPlugins.emplace_back(boost::make_shared<FCLRavePlugin>());         _vPlugins.back()->SetPluginPath("__static__");
}

bool StaticRaveDatabase::LoadPlugin(const std::string& libraryname)
{
    return true;
}


} // namespace OpenRAVE

#endif // OPENRAVE_STATIC_PLUGINS