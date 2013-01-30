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

#include <boost/thread/condition.hpp>

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

inline KinBodyPtr GetBody(void* body) {
    BOOST_ASSERT(!!body);
    return RaveInterfaceCast<KinBody>(*static_cast<InterfaceBasePtr*>(body));
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

void ORCInitialize(bool bLoadAllPlugins, int level)
{
    RaveInitialize(bLoadAllPlugins,level);
}

void ORCDestroy()
{
    RaveDestroy();
}

// can only support one viewer per environment
typedef std::map<EnvironmentBasePtr, boost::shared_ptr<boost::thread> > VIEWERMAP;
static VIEWERMAP s_mapEnvironmentThreadViewers;
static boost::mutex s_mutexViewer;
static boost::condition s_conditionViewer;

void ORCEnvironmentDestroy(void* env)
{
    EnvironmentBasePtr penv = GetEnvironment(env);
    VIEWERMAP::iterator it = s_mapEnvironmentThreadViewers.find(penv);
    if( it != s_mapEnvironmentThreadViewers.end() ) {
        {
            // release the viewer
            ViewerBasePtr pviewer = penv->GetViewer();
            if( !!pviewer ) {
                pviewer->quitmainloop();
            }
        }
        if( !!it->second ) {
            it->second->join();
        }
        s_mapEnvironmentThreadViewers.erase(it);
    }
    penv->Destroy();
}

bool ORCEnvironmentLoad(void* env, const char* filename)
{
    return GetEnvironment(env)->Load(filename);
}

int ORCEnvironmentGetBodies(void* env, void** bodies)
{
    EnvironmentBasePtr penv = GetEnvironment(env);
    std::vector<KinBodyPtr> vbodies;
    penv->GetBodies(vbodies);
    if( !bodies ) {
        return static_cast<int>(vbodies.size());
    }
    for(size_t i = 0; i < vbodies.size(); ++i) {
        bodies[i] = new InterfaceBasePtr(vbodies[i]);
    }
    return vbodies.size();
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

void ORCEnvironmentAdd(void* env, void* pinterface)
{
    GetEnvironment(env)->Add(GetInterface(pinterface));
}

int ORCEnvironmentAddModule(void* env, void* module, const char* args)
{
    return GetEnvironment(env)->AddModule(GetModule(module), args);
}

void ORCEnvironmentRemove(void* env, void* pinterface)
{
    GetEnvironment(env)->Remove(GetInterface(pinterface));
}

void ORCEnvironmentLock(void* env)
{
#if BOOST_VERSION < 103500
    throw OPENRAVE_EXCEPTION_FORMAT0("locking with boost version < 1.35 is not supported", ORE_Failed);
#else
    GetEnvironment(env)->GetMutex().lock();
#endif
}

void ORCEnvironmentUnlock(void* env)
{
#if BOOST_VERSION < 103500
    throw OPENRAVE_EXCEPTION_FORMAT0("unlocking with boost version < 1.35 is not supported", ORE_Failed);
#else
    GetEnvironment(env)->GetMutex().unlock();
#endif
}

void CViewerThread(EnvironmentBasePtr penv, const string &strviewer, bool bShowViewer)
{
    ViewerBasePtr pviewer;
    {
        boost::mutex::scoped_lock lock(s_mutexViewer);
        pviewer = RaveCreateViewer(penv, strviewer);
        if( !!pviewer ) {
            penv->AddViewer(pviewer);
        }
        s_conditionViewer.notify_one();
    }

    if( !pviewer ) {
        return;
    }
    pviewer->main(bShowViewer);     // spin until quitfrommainloop is called
    penv->Remove(pviewer);
}

bool ORCEnvironmentSetViewer(void* env, const char* viewername)
{
    EnvironmentBasePtr penv = GetEnvironment(env);
    VIEWERMAP::iterator it = s_mapEnvironmentThreadViewers.find(penv);
    if( it != s_mapEnvironmentThreadViewers.end() ) {
        if( !!it->second ) {     // wait for the viewer
            it->second->join();
        }
        s_mapEnvironmentThreadViewers.erase(it);
    }

    if( !!viewername && strlen(viewername) > 0 ) {
        boost::mutex::scoped_lock lock(s_mutexViewer);
        boost::shared_ptr<boost::thread> threadviewer(new boost::thread(boost::bind(CViewerThread, penv, std::string(viewername), true)));
        s_mapEnvironmentThreadViewers[penv] = threadviewer;
        s_conditionViewer.wait(lock);
    }
    return true;
}

void ORCInterfaceRelease(void* pinterface)
{
    if( !!pinterface ) {
        delete static_cast<InterfaceBasePtr*>(pinterface);
    }
}

const char* ORCBodyGetName(void* body)
{
    return GetBody(body)->GetName().c_str();
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
