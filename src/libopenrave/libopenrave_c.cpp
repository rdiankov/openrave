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

inline EnvironmentBasePtr& GetEnvironment(void* env)
{
    BOOST_ASSERT(!!env);
    return *static_cast<EnvironmentBasePtr*>(env);
}

inline InterfaceBasePtr GetInterface(void* pinterface)
{
    BOOST_ASSERT(!!pinterface);
    return *static_cast<InterfaceBasePtr*>(pinterface);
}

inline KinBodyPtr GetBody(void* body)
{
    BOOST_ASSERT(!!body);
    return RaveInterfaceCast<KinBody>(*static_cast<InterfaceBasePtr*>(body));
}

inline KinBody::LinkPtr GetBodyLink(void* link)
{
    BOOST_ASSERT(!!link);
    return *static_cast<KinBody::LinkPtr*>(link);
}

inline KinBody::Link::GeometryPtr GetBodyGeometry(void* geometry)
{
    BOOST_ASSERT(!!geometry);
    return *static_cast<KinBody::Link::GeometryPtr*>(geometry);
}

inline RobotBasePtr GetRobot(void* robot)
{
    BOOST_ASSERT(!!robot);
    return RaveInterfaceCast<RobotBase>(*static_cast<InterfaceBasePtr*>(robot));
}

inline ModuleBasePtr GetModule(void* module)
{
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

void* ORCCreateKinBody(void* env, const char* name)
{
    KinBodyPtr pbody = RaveCreateKinBody(GetEnvironment(env), !name ? std::string() : std::string(name));
    if( !pbody ) {
        return NULL;
    }
    return new KinBodyPtr(pbody);
}

void* ORCCreateTriMesh(dReal* vertices, int numvertices, dReal* indices, int numtriangles)
{
    TriMesh* ptrimesh = new TriMesh();
    if( !!vertices && numvertices > 0 ) {
        ptrimesh->vertices.resize(numvertices);
        for(int i = 0; i < numvertices; ++i) {
            ptrimesh->vertices[i].x = vertices[3*i+0];
            ptrimesh->vertices[i].y = vertices[3*i+1];
            ptrimesh->vertices[i].z = vertices[3*i+2];
        }
    }
    if( !!indices && numtriangles > 0 ) {
        ptrimesh->indices.resize(numtriangles*3);
        std::copy(indices,indices+3*numtriangles,ptrimesh->indices.begin());
    }
    return ptrimesh;
}

void ORCTriMeshDestroy(void* trimesh)
{
    if( !!trimesh ) {
        delete static_cast<TriMesh*>(trimesh);
    }
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

void* ORCEnvironmentGetKinBody(void* env, const char* name)
{
    KinBodyPtr pbody = GetEnvironment(env)->GetKinBody(name);
    if( !pbody ) {
        return NULL;
    }
    return new InterfaceBasePtr(pbody);
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

unsigned long long ORCEnvironmentGetSimulationTime(void* env)
{
    return static_cast<unsigned long long>(GetEnvironment(env)->GetSimulationTime());
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

void ORCBodySetName(void* body, const char* name)
{
    return GetBody(body)->SetName(name);
}

int ORCBodyGetDOF(void* body)
{
    return GetBody(body)->GetDOF();
}

void ORCBodyGetDOFValues(void* body, dReal* values)
{
    std::vector<dReal> tempvalues;
    GetBody(body)->GetDOFValues(tempvalues);
    if( tempvalues.size() > 0 ) {
        std::copy(tempvalues.begin(), tempvalues.end(), values);
    }
}

void ORCBodySetDOFValues(void* body, const dReal* values)
{
    KinBodyPtr pbody = GetBody(body);
    std::vector<dReal> tempvalues;
    tempvalues.resize(pbody->GetDOF());
    std::copy(values,values+tempvalues.size(), tempvalues.begin());
    pbody->SetDOFValues(tempvalues);
}

int ORCBodyGetLinks(void* body, void** links)
{
    KinBodyPtr pbody = GetBody(body);
    if( !!links ) {
        for(size_t i = 0; i < pbody->GetLinks().size(); ++i) {
            links[i] = new KinBody::LinkPtr(pbody->GetLinks()[i]);
        }
    }
    return static_cast<int>(pbody->GetLinks().size());
}

void ORCBodySetTransform(void* body, const dReal* pose)
{
    Transform t;
    for(int i = 0; i < 4; ++i) {
        t.rot[i] = pose[i];
    }
    for(int i = 0; i < 3; ++i) {
        t.trans[i] = pose[4+i];
    }
    t.rot.normalize4();
    GetBody(body)->SetTransform(t);
}

void ORCBodySetTransformMatrix(void* body, const dReal* matrix)
{
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        t.m[4*i+0] = matrix[4*i+0];
        t.m[4*i+1] = matrix[4*i+1];
        t.m[4*i+2] = matrix[4*i+2];
        t.trans[i] = matrix[4*i+3];
    }
    GetBody(body)->SetTransform(t);
}

void ORCBodyGetTransform(void* body, dReal* pose)
{
    Transform t = GetBody(body)->GetTransform();
    for(int i = 0; i < 4; ++i) {
        pose[i] = t.rot[i];
    }
    for(int i = 0; i < 3; ++i) {
        pose[4+i] = t.trans[i];
    }
}

void ORCBodyGetTransformMatrix(void* body, dReal* matrix)
{
    TransformMatrix t(GetBody(body)->GetTransform());
    for(int i = 0; i < 3; ++i) {
        matrix[4*i+0] = t.m[4*i+0];
        matrix[4*i+1] = t.m[4*i+1];
        matrix[4*i+2] = t.m[4*i+2];
        matrix[4*i+3] = t.trans[i];
    }
}

bool ORCBodyInitFromTrimesh(void* body, void* trimesh, bool visible)
{
    TriMesh* ptrimesh = static_cast<TriMesh*>(trimesh);
    return GetBody(body)->InitFromTrimesh(*ptrimesh,visible);
}

int ORCBodyLinkGetGeometries(void* link, void** geometries)
{
    KinBody::LinkPtr plink = GetBodyLink(link);
    if( !!geometries ) {
        for(size_t i = 0; i < plink->GetGeometries().size(); ++i) {
            geometries[i] = new KinBody::Link::GeometryPtr(plink->GetGeometry(i));
        }
    }
    return static_cast<int>(plink->GetGeometries().size());
}

void ORCBodyLinkRelease(void* link)
{
    if( !!link ) {
        delete static_cast<KinBody::LinkPtr*>(link);
    }
}

void ORCBodyGeometrySetDiffuseColor(void* geometry, float red, float green, float blue)
{
    GetBodyGeometry(geometry)->SetDiffuseColor(RaveVector<float>(red,green,blue));
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

}
