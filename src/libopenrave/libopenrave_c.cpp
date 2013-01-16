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
#include "libopenrave.h"

namespace OpenRAVE {

inline EnvironmentBasePtr& GetEnvironment(void* env) {
    BOOST_ASSERT(!!env);
    return *static_cast<EnvironmentBasePtr*>(env);
}

inline InterfaceBasePtr GetInterface(void* pinterface)
{
    BOOST_ASSERT(!!pinterface);
    return *static_cast<InterfaceBasePtr*>(pinterface);
}

inline RobotBasePtr GetRobot(void* robot) {
    BOOST_ASSERT(!!robot);
    return RaveInterfaceCast<RobotBase>(*static_cast<InterfaceBasePtr*>(robot));
}

inline ModuleBasePtr GetModule(void* module) {
    BOOST_ASSERT(!!module);
    return RaveInterfaceCast<ModuleBase>(*static_cast<InterfaceBasePtr*>(module));
}

}

extern "C" {

void ORCSetDebugLevel(int level)
{
    RaveSetDebugLevel((DebugLevel)level);
}

bool ORCEnvironmentLoad(void* env, const char* filename)
{
    return GetEnvironment(env)->Load(filename);
}

int ORCEnvironmentGetRobots(void* env, void** robots)
{
    EnvironmentBasePtr penv = GetEnvironment(env);
    std::vector<RobotBasePtr> vrobots;
    penv->GetRobots(vrobots);
    if( !robots ) {
        return static_cast<int>(vrobots.size());
    }
    for(size_t i = 0; i < vrobots.size(); ++i) {
        robots[i] = new InterfaceBasePtr(vrobots[i]);
    }
    return vrobots.size();
}

void ORCEnvironmentLock(void* env)
{
#if BOOST_VERSION < 103500
    throw OPENRAVE_EXCEPTION_FORMAT0("locking with boost version < 1.35 is not supported", ORE_Failed);
#else
    GetEnvironment(env)->GetMutex().lock();
#endif
}

void ORCEnvironmentUnock(void* env)
{
#if BOOST_VERSION < 103500
    throw OPENRAVE_EXCEPTION_FORMAT0("unlocking with boost version < 1.35 is not supported", ORE_Failed);
#else
    GetEnvironment(env)->GetMutex().unlock();
#endif
}

void ORCInterfaceRelease(void* pinterface)
{
    if( !!pinterface ) {
        delete static_cast<InterfaceBasePtr*>(pinterface);
    }
}

const char* ORCRobotGetName(void* robot)
{
    return GetRobot(robot)->GetName().c_str();
}

void* ORCModuleCreate(void* env, const char* modulename)
{
    ModuleBasePtr module = RaveCreateModule(GetEnvironment(env), modulename);
    if( !module ) {
        return NULL;
    }
    return new InterfaceBasePtr(module);
}

int ORCEnvironmentAddModule(void* env, void* module, const char* args)
{
    return GetEnvironment(env)->AddModule(GetModule(module), args);
}

void ORCEnvironmentRemove(void* env, void* pinterface)
{
    GetEnvironment(env)->Remove(GetInterface(pinterface));
}

char* ORCInterfaceSendCommand(void* pinterface, const char* command)
{
    std::stringstream sout, sinput;
    sinput << command;
    bool bSuccess = GetInterface(pinterface)->SendCommand(sout, sinput);
    if( !bSuccess ) {
        return NULL;
    }
    stringstream::streampos posstart = sout.tellg();
    sout.seekg(0, ios_base::end);
    stringstream::streampos posend = sout.tellg();
    sout.seekg(posstart);
    BOOST_ASSERT(posstart<=posend);
    char* poutput = (char*)malloc(posend-posstart+1);
    sout.get(poutput, posend-posstart+1,'\0');
    return poutput;
}

}
