// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file openrave_c.h
    \brief  Defines C public headers
 */
#ifndef OPENRAVE_C_H
#define OPENRAVE_C_H

#include <openrave/config.h>

#ifdef __cplusplus
extern "C" {
#endif

/// \brief Calls \ref RaveSetDebugLevel
OPENRAVE_API void ORCSetDebugLevel(int level);

OPENRAVE_API bool ORCEnvironmentLoad(void* env, const char* filename);

/// \brief Calls \ref EnvironmentBase::GetRobots
///
/// Have to release each of the robot pointers with \ref ORCRobotRelease.
/// If NULL, the function will not fill the robots list only return the number of robots available.
/// \brief env pointer to environment
/// \brief robots a pre-allocated list of robot pointers to fill the robot pointers with.
/// \return number of robots
OPENRAVE_API int ORCEnvironmentGetRobots(void* env, void** robots);

/// \brief Calls \ref EnvironmentBase::AddModule
OPENRAVE_API int ORCEnvironmentAddModule(void* env, void* module, const char* args);

/// \brief Calls \ref EnvironmentBase::Remove
OPENRAVE_API void ORCEnvironmentRemove(void* env, void* pinterface);

/// \brief Return the global environment mutex used to protect environment information access in multi-threaded environments.
///
/// Accessing environment body information and adding/removing bodies
/// or changing any type of scene property should have the environment lock acquired. Once the environment
/// is locked, the user is guaranteed that nnothing will change in the environment.
OPENRAVE_API void ORCEnvironmentLock(void* env);

/// \brief unlock an already locked mutex
OPENRAVE_API void ORCEnvironmentUnock(void* env);

/// \brief release the interfaces like robots, modules, etc.
OPENRAVE_API void ORCInterfaceRelease(void* pinterface);

/// \brief Calls \ref InterfaceBase::SendCommand
///
/// If command failed, will return NULL string.
/// \return The output string. It has to be released with free() by the user.
OPENRAVE_API char* ORCInterfaceSendCommand(void* pinterface, const char* command);

OPENRAVE_API const char* ORCRobotGetName(void* robot);

/// \brief Calls \ref RaveCreateModule
///
/// Have to release the module pointer with \ref ORCModuleRelease
OPENRAVE_API void* ORCModuleCreate(void* env, const char* modulename);

#ifdef __cplusplus
}
#endif

#endif
