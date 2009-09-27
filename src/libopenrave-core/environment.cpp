// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#include "ravep.h"

#include "environment.h"
#include "mt19937ar.h"
#include <errno.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define PLUGIN_EXT ".dll"
#else
#include <dlfcn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>

#ifdef __APPLE_CC__
#define PLUGIN_EXT ".dylib"
#else
#define PLUGIN_EXT ".so"
#endif

#endif

#define CHECK_COLLISION_BODY(body) { \
        if( (body)->GetCollisionData() == NULL ) { \
            RAVELOG_WARNA("body %S not added to enviornment!\n", (body)->GetName()); \
            return false; \
        } \
    }

static bool IsColladaFile(const char* filename)
{
    int len = strlen(filename);
    if( len < 4 )
        return false;
    return filename[len-4] == '.' && filename[len-3] == 'd' && filename[len-2] == 'a' && filename[len-1] == 'e';
}

RaveDatabase::PLUGIN::~PLUGIN()
{
    // do some more checking here, there still might be instances of robots, planners, and sensors out there
    if (plibrary) {
        RAVELOG_DEBUGA("RaveDatabase: closing plugin %s\n", ppluginname.c_str()); Sleep(10);
    
        if( pfnDestroyPlugin != NULL )
            pfnDestroyPlugin();

        RaveDatabase::SysCloseLibrary(plibrary);
    }
}

RaveDatabase::RaveDatabase()
{
    _mapinterfacenames[PT_Planner] = "Planner";
    _mapinterfacenames[PT_Robot] = "Robot";
    _mapinterfacenames[PT_SensorSystem] = "SensorSystem";
    _mapinterfacenames[PT_Controller] = "Controller";
    _mapinterfacenames[PT_ProblemInstance] = "ProblemInstance";
    _mapinterfacenames[PT_InverseKinematicsSolver] = "InverseKinematicsSolver";
    _mapinterfacenames[PT_KinBody] = "KinBody";
    _mapinterfacenames[PT_PhysicsEngine] = "PhysicsEngine";
    _mapinterfacenames[PT_Sensor] = "Sensor";
    _mapinterfacenames[PT_CollisionChecker] = "CollisionChecker";
    _mapinterfacenames[PT_Trajectory] = "Trajectory";
    _mapinterfacenames[PT_Viewer] = "Viewer";
    _mapinterfacenames[PT_Server] = "Server";
}

void RaveDatabase::Destroy()
{
    _listplugins.clear();
    vplugindirs.clear();
}

void RaveDatabase::ReloadPlugins(const wchar_t* pnewlocaldir)
{
    for(list<PLUGIN>::iterator it = _listplugins.begin(); it != _listplugins.end(); ++it) {
        if( it->plibrary != NULL ) {
            if( it->pfnDestroyPlugin != NULL )
                it->pfnDestroyPlugin();
            SysCloseLibrary(it->plibrary);
            it->plibrary = NULL;
        }
        _LoadPlugin(&(*it));
    }
}

bool RaveDatabase::GetInfo(RaveDatabase::PLUGIN* p)
{
    assert(p != NULL);
    
    if( !_LoadPlugin(p) )
        return false;
    if( !p->pfnGetPluginAttributes(&p->info, sizeof(p->info)) )
        RAVELOG_WARNA("%s: GetPluginAttributes failed\n", p->ppluginname.c_str());

    return true;
}

bool RaveDatabase::_LoadPlugin(RaveDatabase::PLUGIN* p)
{
    if( p->plibrary == NULL ) {
        p->plibrary = SysLoadLibrary(p->ppluginname.c_str());

        if( p->plibrary == NULL ) {
            // check if PLUGIN_EXT is missing
            if( strstr(p->ppluginname.c_str(), PLUGIN_EXT) == NULL ) {
                // try adding it
                p->ppluginname += PLUGIN_EXT;
                p->plibrary = SysLoadLibrary(p->ppluginname.c_str());
            }
        }

        if( p->plibrary == NULL ) {
	        RAVELOG_WARNA("failed to load: %s\n", p->ppluginname.c_str());
            return false;
        }

        p->pfnCreate = (CreateFn)SysLoadSym(p->plibrary, "ORCreate");
        p->pfnGetPluginAttributes = (GetPluginAttributesFn)SysLoadSym(p->plibrary, "GetPluginAttributes");
        p->pfnDestroyPlugin = (DestroyPluginFn)SysLoadSym(p->plibrary, "DestroyPlugin");

        if( p->pfnCreate == NULL || p->pfnGetPluginAttributes == NULL ) {
            RAVELOG_WARNA("%s: can't load ORCreate and GetPluginAttriutes functions (%x,%x)\n", p->ppluginname.c_str(), p->pfnCreate, p->pfnGetPluginAttributes);
            return false;
        }

        if( p->pfnDestroyPlugin == NULL )
            RAVELOG_WARNA("plugin %s does not have DestroyPlugin function\n", p->ppluginname.c_str());
        
#ifndef _WIN32
        Dl_info info;
        dladdr((void*)p->pfnCreate, &info);
        RAVELOG_DEBUGA("loading plugin: %s\n", info.dli_fname);
#endif
        
        //p->pfnOpenPlugin(NULL, &g_Environ);
        GetInfo(p);
    }

    return true;
}

RobotBase* RaveDatabase::CreateRobot(EnvironmentBase* penv, const wchar_t* pname)
{
    RobotBase* probot = (RobotBase*)Create(penv, PT_Robot, pname, OPENRAVE_ROBOT_HASH);
    if( probot != NULL ) {
        if( strcmp(probot->GetKinBodyHash(), OPENRAVE_KINBODY_HASH) ) {
            RAVELOG_FATALA("plugin interface Robot, name %s has invalid hash, might be compiled with stale openrave files\n", pname);
            delete probot; probot = NULL;
        }
    }

    assert( probot == NULL || probot->IsRobot() );
    return probot;
}

RobotBase* RaveDatabase::CreateRobot(EnvironmentBase* penv, const char* pname)
{
    RobotBase* probot = (RobotBase*)Create(penv, PT_Robot, pname, OPENRAVE_ROBOT_HASH);
    if( probot != NULL ) {
        if( strcmp(probot->GetKinBodyHash(), OPENRAVE_KINBODY_HASH) ) {
            RAVELOG_FATALA("plugin interface Robot, name %s has invalid hash, might be compiled with stale openrave files\n", pname);
            delete probot; probot = NULL;
        }
    }

    assert( probot == NULL || probot->IsRobot() );
    return probot;
}

void* RaveDatabase::Create(EnvironmentBase* penv, PluginType type, const char* pname)
{
    switch(type) {
    case PT_Planner: return Create(penv,type,pname,OPENRAVE_PLANNER_HASH);
    case PT_Robot: return Create(penv,type,pname,OPENRAVE_ROBOT_HASH);
    case PT_SensorSystem: return Create(penv,type,pname,OPENRAVE_SENSORSYSTEM_HASH);
    case PT_Controller: return Create(penv,type,pname,OPENRAVE_CONTROLLER_HASH);
    case PT_ProblemInstance: return Create(penv,type,pname,OPENRAVE_PROBLEM_HASH);
    case PT_InverseKinematicsSolver: return Create(penv,type,pname,OPENRAVE_IKSOLVER_HASH);
    case PT_KinBody: return Create(penv,type,pname,OPENRAVE_KINBODY_HASH);
    case PT_PhysicsEngine: return Create(penv,type,pname,OPENRAVE_PHYSICSENGINE_HASH);
    case PT_Sensor: return Create(penv,type,pname,OPENRAVE_SENSOR_HASH);
    case PT_CollisionChecker: return Create(penv,type,pname,OPENRAVE_COLLISIONCHECKER_HASH);
    case PT_Trajectory: return Create(penv,type,pname,OPENRAVE_TRAJECTORY_HASH);
    case PT_Viewer: return Create(penv,type,pname,OPENRAVE_VIEWER_HASH);
    case PT_Server: return Create(penv,type,pname,OPENRAVE_SERVER_HASH);
    }
    
    RAVELOG_WARNA("failed to find type %d:%s\n",type,pname);
    return NULL;
}

void* RaveDatabase::Create(EnvironmentBase* penv, PluginType type, const wchar_t* pname, const char* hash)
{
    if( pname == NULL || hash == NULL ) {
        RAVELOG_ERRORA("RaveDatabase::Create bad parameters");
        return NULL;
    }

    for(list<PLUGIN>::iterator itplugin = _listplugins.begin(); itplugin != _listplugins.end(); ++itplugin) {
        if( itplugin->pfnCreate != NULL ) {
            InterfaceBase* pointer = itplugin->pfnCreate(type, pname, penv);
            if( pointer != NULL ) {
                
                if( strcmp(pointer->GetHash(), hash) ) {
                    RAVELOG_FATALA("plugin interface name %S, %s has invalid hash, might be compiled with stale openrave files\n", pname, _mapinterfacenames[type].c_str());
                    delete pointer;
                    continue;
                }

                pointer->__strpluginname = itplugin->ppluginname;
                pointer->__strxmlid = _stdwcstombs(pname);
                return pointer;
            }
        }
    }

    if( pname == NULL || pname[0] == 0 ) {
        switch(type) {
        case PT_KinBody: return penv->CreateKinBody();
        case PT_Trajectory: return penv->CreateTrajectory(0);
        default:
            break;
        }
    }

    RAVELOG_WARNA("Failed to create name %S, interface %s\n", pname, _mapinterfacenames[type].c_str());    
    return NULL;
}

void* RaveDatabase::Create(EnvironmentBase* penv, PluginType type, const char* pname, const char* hash)
{
    if( pname == NULL || hash == NULL ) {
        RAVELOG_ERRORA("RaveDatabase::Create bad parameters");
        return NULL;
    }

    wstring wname = _ravembstowcs(pname);
    for(list<PLUGIN>::iterator itplugin = _listplugins.begin(); itplugin != _listplugins.end(); ++itplugin) {
        if( itplugin->pfnCreate != NULL ) {
            InterfaceBase* pointer = itplugin->pfnCreate(type, wname.c_str(), penv);
            if( pointer != NULL ) {

                if( strcmp(pointer->GetHash(), hash) ) {
                    RAVELOG_FATALA("plugin name %s, interface %s  has invalid hash, might be compiled with stale openrave files\n", pname, _mapinterfacenames[type].c_str());
                    delete pointer;
                    continue;
                }

                pointer->__strpluginname = itplugin->ppluginname;
                pointer->__strxmlid = pname;
                return pointer;
            }
        }
    }

    RAVELOG_WARNA("Failed to create name %s, interface %s\n", pname, _mapinterfacenames[type].c_str());
	return NULL;
}

void* RaveDatabase::SysLoadLibrary(const char* lib)
{
#ifdef _WIN32
    void* plib = LoadLibraryA(lib);
    if( plib == NULL ) {
        RAVELOG_WARNA("Failed to load %s\n", lib);
    }
#else
    void* plib = dlopen(lib, RTLD_NOW);
    if( plib == NULL ) {
        RAVELOG_WARNA("%s\n", dlerror());
    }
#endif
    return plib;
}

void* RaveDatabase::SysLoadSym(void* lib, const char* sym)
{
#ifdef _WIN32
    return GetProcAddress((HINSTANCE)lib, sym);
#else
    return dlsym(lib, sym);
#endif
}

void RaveDatabase::SysCloseLibrary(void* lib)
{
#ifdef _WIN32
    FreeLibrary((HINSTANCE)lib);
#else
    dlclose(lib);
#endif
}

bool RaveDatabase::AddDirectory(const char* pdir)
{
#ifdef _WIN32
    WIN32_FIND_DATAA FindFileData;
    HANDLE hFind;
    string strfind = pdir;
    strfind += "\\*";
    strfind += PLUGIN_EXT;

    hFind = FindFirstFileA(strfind.c_str(), &FindFileData);
    if (hFind == INVALID_HANDLE_VALUE) {
        RAVELOG_DEBUGA("No plugins in dir: %s (GetLastError reports %d)\n", pdir, GetLastError ());
        return false;
    } 
    else 
    {
        do {
            RAVELOG_DEBUGA("Adding plugin %s\n", FindFileData.cFileName);
            string strplugin = pdir;
            strplugin += "\\";
            strplugin += FindFileData.cFileName;
            AddPlugin(strplugin.c_str());
        } while (FindNextFileA(hFind, &FindFileData) != 0);

        FindClose(hFind);
    }
#else
    // linux
    DIR *dp;
    struct dirent *ep;

    dp = opendir (pdir);
    if (dp != NULL) {
        while ( (ep = readdir (dp)) != NULL ) {
            // check for a .so in every file
            if( strstr(ep->d_name, PLUGIN_EXT) != NULL ) {
                string strplugin = pdir;
                strplugin += "/";
                strplugin += ep->d_name;
                AddPlugin(strplugin.c_str());
            }
        }
        (void) closedir (dp);
    }
    else
        RAVELOG_WARNA("Couldn't open directory %s\n", pdir);
#endif

    return true;
}

bool RaveDatabase::AddPlugin(const char* pname)
{
    if( pname == NULL ) {
        RAVELOG_WARNA("plugin name is NULL\n");
        return false;
    }

    // first delete it
    DeletePlugin(pname);

    _listplugins.push_back(PLUGIN());
    PLUGIN& p = _listplugins.back();
    p.ppluginname = pname;
    if( !_LoadPlugin(&p) ) {
        _listplugins.pop_back();
        return false;
    }

    return true;
}

bool RaveDatabase::DeletePlugin(const char* pname)
{
    bool bFound = false;
    list<PLUGIN>::iterator it = _listplugins.begin();
    while(it != _listplugins.end()) {
        if( stricmp(pname, it->ppluginname.c_str()) == 0 ) {
            it = _listplugins.erase(it);
            bFound = true;
        }
        else ++it;
    }

    return bFound;
}

bool Environment::DummyPhysicsEngine::GetBodyVelocity(const KinBody* pbody, Vector& linearvel, Vector& angularvel, dReal* pJointVelocity)
{
    if( pbody == NULL )
        return false;
    linearvel = Vector(0,0,0,0);
    angularvel = Vector(0,0,0,0);
    
    if( pJointVelocity != NULL )
        memset(pJointVelocity, 0, sizeof(dReal)*pbody->GetDOF());
    return true;
}

bool Environment::DummyPhysicsEngine::GetBodyVelocity(KinBody* pbody, Vector* pLinearVelocities, Vector* pAngularVelocities)
{
    if( pbody == NULL )
        return false;
    if( pLinearVelocities != NULL )
        memset(pLinearVelocities, 0, sizeof(dReal)*3*pbody->GetLinks().size());
    if( pAngularVelocities != NULL )
        memset(pAngularVelocities, 0, sizeof(dReal)*3*pbody->GetLinks().size());
    return true;
}

bool Environment::DummyPhysicsEngine::GetJointVelocity(const KinBody::Joint* pjoint, dReal* pJointVelocity)
{
    if( pjoint == NULL || pJointVelocity == NULL )
        return false;
    memset(pJointVelocity, 0, sizeof(dReal)*pjoint->GetDOF());
    return true;
}

/////////////////
// Environment //
/////////////////
Environment::Environment(bool bLoadAllPlugins) : _dummyphysics(this), _dummychecker(this), _dummyviewer(this)
{
    char* phomedir = getenv("OPENRAVE_CACHEPATH");
    if( phomedir == NULL ) {
#ifndef _WIN32
        _homedirectory = string(getenv("HOME"))+string("/.openrave");
#else
        _homedirectory = string(getenv("HOMEPATH"))+string("\\.openrave");
#endif
    }
    else
        _homedirectory = phomedir;
#ifndef _WIN32
    mkdir(_homedirectory.c_str(),S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH | S_IRWXU);
#else
    //CreateDirectory(_homedirectory.c_str());
#endif
    RAVELOG_DEBUGA("setting openrave cache directory to %s\n",_homedirectory.c_str());

    _nBodiesModifiedStamp = 0;
    _bPublishBodiesAnytime = false;

    _pCurrentViewer = &_dummyviewer;
    _pserver = NULL;
    _fDeltaSimTime = 0.01f;
    _nCurSimTime = 0;
    _nStartSimTime = GetMicroTime();
    _bRealTime = true;

    _bPluginsLoaded = false;
    //_bSelfCollision = false;
    _pPhysicsEngine = &_dummyphysics;
    _bDestroying = false;
    _bDestroyed = false;
    _bEnableSimulation = true; // need to start by default

    nNetworkIndex = 1;
    _mapBodies.clear();
    pthread_mutex_init(&_mutexNetworkIds, NULL);
    pthread_mutex_init(&_mutexPhysics, NULL);
    pthread_mutex_init(&_mutexProblems, NULL);
    pthread_mutex_init(&_mutexBodies, NULL);
    pthread_mutex_init(&_mutexDestroy, NULL);
    pthread_mutex_init(&_mutexXML, NULL);
    
    srand(timeGetTime());
    init_genrand(timeGetTime());

    pCurrentChecker = &_dummychecker;

    _pdatabase.reset(new RaveDatabase());
    if( bLoadAllPlugins ) {
        if( !ParseDirectories(getenv("OPENRAVE_PLUGINS"), _vplugindirs) || _vplugindirs.size() == 0 ) {
            RAVELOG_INFOA("could not find OPENRAVE_PLUGINS variable, setting to %s\n", OPENRAVE_PLUGINS_INSTALL_DIR);
            _vplugindirs.push_back(OPENRAVE_PLUGINS_INSTALL_DIR);
        }
    }
    
    if( !ParseDirectories(getenv("OPENRAVE_DATA"), _vdatadirs) || _vdatadirs.size() == 0 ) {
        RAVELOG_INFOA("could not find OPENRAVE_DATA variable, setting to %s\n", OPENRAVE_DATA_INSTALL_DIR);
        _vdatadirs.push_back(OPENRAVE_DATA_INSTALL_DIR);
    }

    pthread_create(&_threadLoop, NULL, main, this);
}

Environment::Environment(const Environment& r, int options) : _dummyphysics(this), _dummychecker(this), _dummyviewer(this)
{
    _nBodiesModifiedStamp = 0;
    _bPublishBodiesAnytime = r._bPublishBodiesAnytime;
    _homedirectory = r._homedirectory;
    _pCurrentViewer = &_dummyviewer;
    _pserver = NULL;
    _fDeltaSimTime = r._fDeltaSimTime;
    _nStartSimTime = GetMicroTime();
    _nCurSimTime = 0;
    nNetworkIndex = r.nNetworkIndex;
    _bRealTime = r._bRealTime;

    _bPluginsLoaded = false; // not loading plugins from this environment
    _bDestroying = false;
    _bDestroyed = false;
    _bEnableSimulation = false;

    SetDebugLevel(r.GetDebugLevel());

    _mapBodies.clear();
    pthread_mutex_init(&_mutexNetworkIds, NULL);
    pthread_mutex_init(&_mutexPhysics, NULL);
    pthread_mutex_init(&_mutexProblems, NULL);
    pthread_mutex_init(&_mutexBodies, NULL);
    pthread_mutex_init(&_mutexDestroy, NULL);
    pthread_mutex_init(&_mutexXML, NULL);

    srand(timeGetTime());
    init_genrand(timeGetTime());

    _pPhysicsEngine = &_dummyphysics;
    pCurrentChecker = &_dummychecker;

    _pdatabase = r._pdatabase;
    pthread_create(&_threadLoop, NULL, main, this);

    WaitForPlugins();
    _vplugindirs = r._vplugindirs;
    _vdatadirs = r._vdatadirs;

    LockPhysics(true);

    // clone collision and physics
    if( r.GetCollisionChecker() != NULL ) {
        SetCollisionChecker(NULL);
        _localchecker.reset(CreateCollisionChecker(r.GetCollisionChecker()->GetXMLId()));
        SetCollisionChecker(_localchecker.get());
    }

    if( options & Clone_Bodies ) {
        MutexLock mbodies(&r._mutexBodies);
        FOREACHC(itrobot, r._vecrobots) {
            RobotBase* pnewrobot = _pdatabase->CreateRobot(this, (*itrobot)->GetXMLId());
            if( pnewrobot == NULL ) {
                RAVELOG_ERRORA("failed to create robot %s\n", (*itrobot)->GetXMLId());
                continue;
            }

            if( !pnewrobot->Clone(*itrobot, options)) {
                RAVELOG_ERRORA("failed to clone robot %S\n", (*itrobot)->GetName());
                delete pnewrobot;
                continue;
            }

            pnewrobot->networkid = (*itrobot)->GetNetworkId();
            pnewrobot->DestroyCallback = KinBodyDestroyCallback;

            // note that pointers will not be correct
            pnewrobot->_vGrabbedBodies = (*itrobot)->_vGrabbedBodies;
            pnewrobot->_setAttachedBodies = (*itrobot)->_setAttachedBodies;

            assert( _mapBodies.find(pnewrobot->GetNetworkId()) == _mapBodies.end() );
            _mapBodies[pnewrobot->GetNetworkId()] = pnewrobot;
            _vecbodies.push_back(pnewrobot);
            _vecrobots.push_back(pnewrobot);
        }
        FOREACHC(itbody, r._vecbodies) {
            if( _mapBodies.find((*itbody)->GetNetworkId()) != _mapBodies.end() )
                continue;
            KinBody* pnewbody = new KinBody(PT_KinBody,this);
            if( !pnewbody->Clone(*itbody,options) ) {
                RAVELOG_ERRORA("failed to clone body %S\n", (*itbody)->GetName());
                delete pnewbody;
                continue;
            }

            pnewbody->networkid = (*itbody)->GetNetworkId();
            pnewbody->DestroyCallback = KinBodyDestroyCallback;

            // note that pointers will not be correct
            pnewbody->_setAttachedBodies = (*itbody)->_setAttachedBodies;

            _mapBodies[pnewbody->GetNetworkId()] = pnewbody;
            _vecbodies.push_back(pnewbody);
        }

        // process the attached and grabbed bodies
        FOREACH(itbody, _vecbodies) {
            set<KinBody*> setnew;
            FOREACH(itatt, (*itbody)->_setAttachedBodies) {
                assert( _mapBodies.find((*itatt)->GetNetworkId()) != _mapBodies.end());
                setnew.insert(_mapBodies[(*itatt)->GetNetworkId()]);
            }
            (*itbody)->_setAttachedBodies = setnew;
        }

        FOREACH(itrobot, _vecrobots) {
            FOREACH(itgrab, (*itrobot)->_vGrabbedBodies) {
                assert( itgrab->pbody != NULL && _mapBodies.find(itgrab->pbody->GetNetworkId()) != _mapBodies.end());
                itgrab->pbody = _mapBodies[itgrab->pbody->GetNetworkId()];
                itgrab->plinkrobot = (*itrobot)->GetLinks()[itgrab->plinkrobot->GetIndex()];

                set<KinBody::Link*> setnew;
                FOREACH(itlink, itgrab->sValidColLinks)
                    setnew.insert((*itrobot)->_veclinks[(*itlink)->GetIndex()]);
                itgrab->sValidColLinks = setnew;
            }
        }

        FOREACH(itbody, _vecbodies) {
            GetCollisionChecker()->InitKinBody(*itbody);
            GetPhysicsEngine()->InitKinBody(*itbody);
        }
    }
    if( options & Clone_Viewer ) {
        if( r.GetViewer() != NULL ) {
            _localviewer.reset(CreateViewer(r.GetViewer()->GetXMLId()));
            AttachViewer(_localviewer.get());
        }
    }
    
    if( options & Clone_Simulation ) {
        if( r.GetPhysicsEngine() != NULL ) {
            SetPhysicsEngine(NULL);
            _localphysics.reset(CreatePhysicsEngine(r.GetPhysicsEngine()->GetXMLId()));
            SetPhysicsEngine(_localphysics.get());
        }
        
        _bEnableSimulation = r._bEnableSimulation;
        _nCurSimTime = r._nCurSimTime;
    }

    LockPhysics(false);
}

Environment::~Environment()
{
    Destroy();

    // destroy the thread
    _bEnableSimulation = false;
    _bDestroying = true;

    // dont' join, might not return
    RAVELOG_DEBUGA("Environment destructor\n");
    void* retvalue;
    pthread_join(_threadLoop, &retvalue);

    {
        MutexLock m(&_mutexDestroy);
        if( !_bDestroyed ) {
            if( _pserver )
                _pserver->Destroy();
            Destroy();
            _bDestroyed = true;
        }
    }

    {
        MutexLock m(&_mutexBodies);
        _CleanRemovedBodies();
    }

    _pPhysicsEngine->DestroyEnvironment();
    pCurrentChecker->DestroyEnvironment();

    RAVELOG_DEBUGA("destroying plugins\n");
    SetCollisionChecker(NULL);
    SetPhysicsEngine(NULL);
    AttachViewer(NULL);
    _pIKFastLoader.reset();
    _localchecker.reset();
    _localphysics.reset();
    _localviewer.reset();
    _pdatabase.reset();

    pthread_mutex_destroy(&_mutexPhysics);
    pthread_mutex_destroy(&_mutexProblems);
    pthread_mutex_destroy(&_mutexNetworkIds);
    pthread_mutex_destroy(&_mutexBodies);
    pthread_mutex_destroy(&_mutexDestroy);
}

void Environment::Destroy()
{
    bool bOldSim = _bEnableSimulation;
    _bEnableSimulation = false;
    Reset();
    RAVELOG_DEBUGA("destroy problems\n");
    {
        MutexLock m(&_mutexProblems);
        FOREACH(it, listProblems) delete *it;
        listProblems.clear();
    }

    _bEnableSimulation = bOldSim;
}

void Environment::Reset()
{
    if( _pserver )
        _pserver->Reset();
    
    RAVELOG_DEBUGA("resetting raveviewer\n");
    assert( _pCurrentViewer != NULL );
    _pCurrentViewer->deselect();
    _pCurrentViewer->Reset();
    
    LockPhysics(true);

    {
        MutexLock m(&_mutexBodies);
        
        _CleanRemovedBodies();
        
        FOREACH(it, _vecbodies) {
            //RAVELOG(L"deleting %S\n", (*it)->GetName());
            delete *it;
        }
        
        _vecbodies.clear();
        _vecrobots.clear();
        _vPublishedBodies.clear();
        _nBodiesModifiedStamp++;
    }

    static bool bfirst=false;
    if( !bfirst ) bfirst = true;

    _CleanRemovedBodies();
    _pPhysicsEngine->DestroyEnvironment();
    pCurrentChecker->DestroyEnvironment();

    nNetworkIndex = 1;
    _mapBodies.clear();

    pCurrentChecker->InitEnvironment();
    _pPhysicsEngine->InitEnvironment();

    LockPhysics(false);
}

void Environment::_CleanRemovedBodies()
{
    FOREACH(itbody, _listRemovedBodies) {
        _pPhysicsEngine->DestroyKinBody(*itbody);
        pCurrentChecker->DestroyKinBody(*itbody); // release collision info
        delete *itbody;
    }
    
    _listRemovedBodies.clear();
}

void Environment::GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins)
{
    WaitForPlugins();
    plugins.clear();
    FOREACHC(itplugin, GetDatabase().GetPlugins())
        plugins.push_back(pair<string,PLUGININFO>(itplugin->ppluginname,itplugin->info));
}

#define WRITE_PLUGINNAMES(vtype, infonames) {   \
        infonames.resize(0); \
        FOREACHC(itplugin, GetDatabase().GetPlugins()) { \
            FOREACHC(itnames, vtype) { \
                infonames.push_back(*itnames);           \
            } \
        } \
        sort(infonames.begin(), infonames.end()); \
    }

void Environment::GetLoadedInterfaces(PLUGININFO& info)
{
    WaitForPlugins(); 
    WRITE_PLUGINNAMES(itplugin->info.robots, info.robots);
    WRITE_PLUGINNAMES(itplugin->info.planners, info.planners);
    WRITE_PLUGINNAMES(itplugin->info.sensorsystems, info.sensorsystems);
    WRITE_PLUGINNAMES(itplugin->info.controllers, info.controllers);
    WRITE_PLUGINNAMES(itplugin->info.problems, info.problems);
    WRITE_PLUGINNAMES(itplugin->info.iksolvers, info.iksolvers);
    WRITE_PLUGINNAMES(itplugin->info.physicsengines, info.physicsengines);
    WRITE_PLUGINNAMES(itplugin->info.sensors, info.sensors);
    WRITE_PLUGINNAMES(itplugin->info.collisioncheckers, info.collisioncheckers);
    WRITE_PLUGINNAMES(itplugin->info.trajectories, info.trajectories);
    WRITE_PLUGINNAMES(itplugin->info.viewers, info.viewers);
    WRITE_PLUGINNAMES(itplugin->info.servers, info.servers);
}

void Environment::WaitForPlugins() const
{
    while(!_bPluginsLoaded)
        Sleep(1);
}

EnvironmentBase* Environment::CloneSelf(int options)
{
    return new Environment(*this, options);
}

InterfaceBase* Environment::CreateInterface(PluginType type,const char* pinterfacename)
{
    WaitForPlugins();
    InterfaceBase* pinterface = (InterfaceBase*)_pdatabase->Create(this,type,pinterfacename);
    switch(type) {
    case PT_KinBody:
        if( pinterface == NULL )
            return CreateKinBody();
        else {
            KinBody* pbody = (KinBody*)pinterface;
            SetUniqueNetworkId(pbody, &pbody->networkid);
            pbody->DestroyCallback = KinBodyDestroyCallback;
        }
        break;
    case PT_Robot:
        if( pinterface == NULL )
            return CreateRobot((char*)NULL);
        else {
            RobotBase* probot = (RobotBase*)pinterface;
            SetUniqueNetworkId(probot, &probot->networkid);
            probot->DestroyCallback = KinBodyDestroyCallback;
        }
        break;
    default:
        break;
    }
    return pinterface;
}

RobotBase* Environment::CreateRobot(const wchar_t* pname)
{
    RobotBase* probot;
    if( pname != NULL ) {
        WaitForPlugins();
        probot = _pdatabase->CreateRobot(this, pname);
    }
    else
        probot = new RobotBase(this);

    if( probot != NULL ) {
        SetUniqueNetworkId(probot, &probot->networkid);
        probot->DestroyCallback = KinBodyDestroyCallback;
    }
    return probot;
}

RobotBase* Environment::CreateRobot(const char* pname)
{
    RobotBase* probot;
    if( pname != NULL ) {
        WaitForPlugins();
        probot = _pdatabase->CreateRobot(this, pname);
    }
    else
        probot = new RobotBase(this);

    if( probot != NULL ) {
        SetUniqueNetworkId(probot, &probot->networkid);
        probot->DestroyCallback = KinBodyDestroyCallback;
    }
    return probot;
}

void Environment::AddIKSolvers()
{
    if( !_pIKFastLoader ) {
        _pIKFastLoader.reset(_pdatabase->CreateProblem(this,"IKFast"));
        if( !_pIKFastLoader )
            return;
    }

    string ikname, iklibrary, response;
    stringstream ss;
    // don't wait for plugins
    ifstream f((_homedirectory + "/ikfastsolvers").c_str());
    if( !!f ) {
        while(!f.eof()) {
            f >> ikname >> iklibrary;
            if( !f )
                break;
            ss.str("");
            ss << "AddIkLibrary " << ikname << " " << iklibrary;
            if( !_pIKFastLoader->SendCommand(ss.str().c_str(),response) )
                RAVELOG_WARNA("failed to load %s",iklibrary.c_str());
        }
    }

    vector<string> vikfastsolvers;
    if( ParseDirectories(getenv("OPENRAVE_IKFAST"), vikfastsolvers) ) {
        FOREACH(it,vikfastsolvers) {
            string::size_type pos = it->find('=');
            if( pos == string::npos ) {
                if( it->size() > 0 )
                    RAVELOG_WARNA("cannot extract name and file from OPENRAVE_IKFAST string %s (no =)\n",it->c_str());
            }
            else {
                ikname = it->substr(0,pos);
                iklibrary = it->substr(pos+1);
                ss.str("");
                ss << "AddIkLibrary " << ikname << " " << iklibrary;
                if( !_pIKFastLoader->SendCommand(ss.str().c_str(),response) )
                    RAVELOG_WARNA("failed to load %s",iklibrary.c_str());
            }
        }
    }
}

bool Environment::Load(const wchar_t *filename)
{
    return Load(_stdwcstombs(filename).c_str());
}

bool Environment::Load(const char *filename)
{
    WaitForPlugins();

    bool bSuccess;
    if( IsColladaFile(filename) ) {
        bSuccess = RaveParseColladaFile(this, filename);
    }
    else {
        EnvironmentXMLReader reader(this, NULL);
        bSuccess = ParseXMLFile(&reader, filename);
        reader.Release();
    }

    if( !bSuccess ) {
        RAVELOG_WARNA("load failed on file %s\n", filename);
        //Destroy();
    }
    
    return bSuccess;
}

bool Environment::Save(const char* filename)
{
    return RaveWriteColladaFile(this,filename);
}

int Environment::LoadProblem(ProblemInstance* prob, const char* cmdargs)
{
    if( prob == NULL )
        return -1;
 
    assert(prob->GetEnv()==this);

    WaitForPlugins();
    int ret = prob->main(cmdargs);
    if( ret != 0 )
        RAVELOG_WARNA("Error %d with executing problem\n", ret);
    else {
        MutexLock m(&_mutexProblems);
        listProblems.push_back(prob);
    }

    return ret;
}

bool Environment::RemoveProblem(ProblemInstance* prob)
{
    MutexLock m(&_mutexProblems);
    list<ProblemInstance*>::iterator itprob = find(listProblems.begin(), listProblems.end(), prob);
    if( itprob != listProblems.end() ) {
        listProblems.erase(itprob);
        return true;
    }

    return false;
}

bool Environment::AddKinBody(KinBody* pbody)
{
    if( pbody == NULL )
        return false;

    assert(pbody->GetEnv()==this);
    {
        MutexLock m(&_mutexBodies);
        _vecbodies.push_back(pbody);
        _nBodiesModifiedStamp++;
    }
    pCurrentChecker->InitKinBody(pbody);
    _pPhysicsEngine->InitKinBody(pbody);
    pbody->ComputeJointHierarchy();
    return true;
}

bool Environment::AddRobot(RobotBase* robot)
{
    if( robot == NULL )
        return false;

    assert(robot->GetEnv()==this);
    {
        MutexLock m(&_mutexBodies);
        _vecbodies.push_back(robot);
        _vecrobots.push_back(robot);
        _nBodiesModifiedStamp++;
    }
    pCurrentChecker->InitKinBody(robot);
    _pPhysicsEngine->InitKinBody(robot);
    robot->ComputeJointHierarchy();
    return true;
}

bool Environment::RemoveKinBody(KinBody* pbody, bool bDestroy)
{
    bool bSuccess = true;

    {
        MutexLock m(&_mutexBodies);
        
        vector<KinBody*>::iterator it = std::find(_vecbodies.begin(), _vecbodies.end(), pbody);
        if( it != _vecbodies.end() ) {
            
            // check robots also
            if( bDestroy ) {
                
                // before deleting, make sure no robots are grabbing it!!
                FOREACH(itrobot, _vecrobots) {
                    if( (*itrobot)->IsGrabbing(*it) ) {
                        RAVELOG_WARNA("destroy %S already grabbed by robot %S!\n", pbody->GetName(), (*itrobot)->GetName());
                        (*itrobot)->Release(pbody);
                    }
                }
            }

            if( IsPhysicsLocked() ) {
                _pPhysicsEngine->DestroyKinBody(*it);
                pCurrentChecker->DestroyKinBody(*it); // release collision info
            }
            else
                RAVELOG_WARNA("need to lock physics in order to remove from collision checker!\n");

            if( bDestroy ) {
                if( IsPhysicsLocked() ) {
                    delete *it;
                }
                else {
                    RAVELOG_ERRORA("removing kinbody without locking physics!\n");
                    (*it)->Enable(false);
                    _listRemovedBodies.push_back(*it);
                }
            }
            
            _vecbodies.erase(it);
            _nBodiesModifiedStamp++;
            
            vector<RobotBase*>::iterator itrobot = std::find(_vecrobots.begin(), _vecrobots.end(), pbody);
            if( itrobot != _vecrobots.end() )
                _vecrobots.erase(itrobot);
        }
        else bSuccess = false;
    }

    return bSuccess;
}

KinBody* Environment::GetKinBody(const wchar_t *pname)
{
    MutexLock m(&_mutexBodies);

    FOREACHC(it, _vecbodies) {
        if(wcsicmp((*it)->GetName(),pname)==0)
            return *it;
    }
    RAVELOG_VERBOSEA("Environment::GetKinBody - Error: Unknown body\n");
    return NULL;
}

KinBody* Environment::CreateKinBody()
{
    if( !IsPhysicsLocked() ) {
        RAVELOG_ERRORA("CreateKinBody physics needs to be locked! Ignoring lock...\n");
    }

    KinBody* pbody = new KinBody(PT_KinBody,this);
    SetUniqueNetworkId(pbody, &pbody->networkid);
    pbody->DestroyCallback = KinBodyDestroyCallback;

    return pbody;
}

void Environment::KinBodyDestroyCallback(EnvironmentBase* penv, KinBody* pbody)
{
    if( pbody != NULL )
        ((Environment*)penv)->RemoveUniqueNetworkId(pbody->GetNetworkId());
}

bool Environment::SetPhysicsEngine(PhysicsEngineBase* pengine)
{
    if( !IsPhysicsLocked() )
        RAVELOG_WARNA("Setting physics engine without locking physics\n");

    assert( _pPhysicsEngine != NULL );
    _pPhysicsEngine->DestroyEnvironment();
    _pPhysicsEngine = pengine;
    if( _pPhysicsEngine == NULL ) {
        RAVELOG_DEBUGA("disabling physics\n");
        _pPhysicsEngine = &_dummyphysics;
    }
    _pPhysicsEngine->InitEnvironment();
    return true;
}

PhysicsEngineBase* Environment::GetPhysicsEngine() const
{
    WaitForPlugins();
    return _pPhysicsEngine;
}

void Environment::GetBodies(std::vector<KinBody*>& bodies) const
{
    MutexLock m(&_mutexBodies);
    bodies = _vecbodies;
}

EnvironmentBase::EnvLock* Environment::GetLockedBodies(std::vector<KinBody*>& bodies) const
{
    EnvMutexLock *pm = new EnvMutexLock(&_mutexBodies);
    bodies = _vecbodies;
    return pm;
}

EnvironmentBase::EnvLock* Environment::GetLockedRobots(std::vector<RobotBase*>& robots) const
{
    EnvMutexLock *pm = new EnvMutexLock(&_mutexBodies);
    robots = _vecrobots;
    return pm;
}

void Environment::GetPublishedBodies(vector<BODYSTATE>& vbodies)
{
    if( _bPublishBodiesAnytime ) {
        LockPhysics(true);
        MutexLock m(&_mutexBodies);
        
        vbodies.resize(_vecbodies.size());
        
        vector<BODYSTATE>::iterator itstate = vbodies.begin();
        vector<Transform> vectrans;
        vector<dReal> jointvalues;
        FOREACH(itbody, _vecbodies) {
            itstate->pbody = *itbody;
            (*itbody)->GetBodyTransformations(vectrans);
            itstate->vectrans.resize(vectrans.size());
            for(size_t i = 0; i < vectrans.size(); ++i)
                itstate->vectrans[i] = vectrans[i];

            (*itbody)->GetJointValues(jointvalues);
            itstate->jointvalues.resize(jointvalues.size());
            for(size_t i = 0; i < jointvalues.size(); ++i)
                itstate->jointvalues[i] = jointvalues[i];

            itstate->strname =(*itbody)->GetName();
            itstate->pguidata = (*itbody)->GetGuiData();
            itstate->networkid = (*itbody)->GetNetworkId();
            ++itstate;
        }

        LockPhysics(false);
    }
    else {
        MutexLock m(&_mutexBodies);
        vbodies = _vPublishedBodies;
    }
}

bool Environment::SetCollisionChecker(CollisionCheckerBase* pchecker)
{
    if( pCurrentChecker == pchecker )
        return true;
    
    pCurrentChecker->DestroyEnvironment(); // delete all resources
    
    pCurrentChecker = pchecker;
    if( pCurrentChecker == NULL ) {
        RAVELOG_DEBUGA("disabling collisions\n");
        pCurrentChecker = &_dummychecker;
    }
    return pCurrentChecker->InitEnvironment();
}

CollisionCheckerBase* Environment::GetCollisionChecker() const
{
    WaitForPlugins();
    return pCurrentChecker;
}

bool Environment::SetCollisionOptions(int options)
{
    WaitForPlugins();
    return pCurrentChecker->SetCollisionOptions(options);
}

int Environment::GetCollisionOptions() const
{
    WaitForPlugins();
    return pCurrentChecker->GetCollisionOptions();
}

bool Environment::CheckCollision(const KinBody* pbody, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(pbody);
    return pCurrentChecker->CheckCollision(pbody, pReport);
}

bool Environment::CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(pbody1);
    CHECK_COLLISION_BODY(pbody2);
    return pCurrentChecker->CheckCollision(pbody1, pbody2, pReport);
}

bool Environment::CheckCollision(const KinBody::Link* plink, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(plink->GetParent());
    return pCurrentChecker->CheckCollision(plink, pReport);
}

bool Environment::CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(plink1->GetParent());
    CHECK_COLLISION_BODY(plink2->GetParent());
    return pCurrentChecker->CheckCollision(plink1, plink2, pReport);
}

bool Environment::CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(plink->GetParent());
    CHECK_COLLISION_BODY(pbody);
    return pCurrentChecker->CheckCollision(plink,pbody,pReport);
}

bool Environment::CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(plink->GetParent());
    return pCurrentChecker->CheckCollision(plink,vbodyexcluded,vlinkexcluded,pReport);
}

bool Environment::CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(pbody);
    return pCurrentChecker->CheckCollision(pbody,vbodyexcluded,vlinkexcluded,pReport);
} 

bool Environment::CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(plink->GetParent());
    return pCurrentChecker->CheckCollision(ray, plink, pReport);
}

bool Environment::CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport)
{
    CHECK_COLLISION_BODY(pbody);
    return pCurrentChecker->CheckCollision(ray, pbody, pReport);
}

bool Environment::CheckCollision(const RAY& ray, COLLISIONREPORT* pReport)
{
    return pCurrentChecker->CheckCollision(ray, pReport);
}


bool Environment::CheckCollision(const KinBody::Link* plink, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)
{
    CHECK_COLLISION_BODY(plink->GetParent());
    return pCurrentChecker->CheckCollision(plink,vbodyexcluded,vlinkexcluded,tolerance);
}

bool Environment::CheckCollision(const KinBody* pbody, const std::set<KinBody *>& vbodyexcluded, const std::set<KinBody::Link *>& vlinkexcluded, dReal tolerance)
{
    CHECK_COLLISION_BODY(pbody);
    return pCurrentChecker->CheckCollision(pbody,vbodyexcluded,vlinkexcluded,tolerance);
}

void Environment::SetCamera(const RaveTransform<float>& trans)
{
    _pCurrentViewer->SetCamera(trans.trans, trans.rot);
}

void Environment::SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat)
{
    _pCurrentViewer->SetCamera(pos, quat);
}

void Environment::SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup)
{
    _pCurrentViewer->SetCameraLookAt(lookat, campos, camup);
}

RaveTransform<float> Environment::GetCameraTransform()
{
    return _pCurrentViewer->GetCameraTransform();
}


bool Environment::LockPhysics(bool bLock, float timeout)
{
    int err = 0;
    if( bLock ) {
#ifndef HAVE_TIMED_LOCK
        if( timeout > 0 )
            RAVELOG_WARNA("openrave not compiled with timed locking");
#else
        if( timeout > 0 ) {
            struct timespec abs_time;

#if !defined(HAVE_CLOCK_GETTIME)
            uint64_t abs_timeout = GetMicroTime();
#else
            // abs_time = now + 3sec
            clock_gettime(CLOCK_REALTIME, &abs_time);
            uint64_t abs_timeout = (uint64_t)abs_time.tv_sec*(uint64_t)1000000000 + (uint64_t)abs_time.tv_nsec + (uint64_t)(timeout*1000000000.0f);
#endif
            abs_time.tv_nsec = (long)(abs_timeout%(uint64_t)1000000000);
            abs_time.tv_sec = (long)(abs_timeout/1000000000);
            err = pthread_mutex_timedlock(&_mutexPhysics,&abs_time);
        }
        else
#endif
        {
            err = pthread_mutex_lock(&_mutexPhysics);
        }
    }
    else
        err = pthread_mutex_unlock(&_mutexPhysics);
    return err == 0;
}

bool Environment::IsPhysicsLocked()
{
    int ret = pthread_mutex_trylock(&_mutexPhysics);
    if( ret == 0 )
        pthread_mutex_unlock(&_mutexPhysics);
    return ret == EBUSY;
}

RobotBase* Environment::ReadRobotXML(RobotBase* robot, const char* filename, const char** atts)
{
    assert(robot==NULL||robot->GetEnv()==this);

    if( IsColladaFile(filename) ) {
        if( !RaveParseColladaFile(this, &robot, filename) )
            return NULL;
    }
    else {
        boost::shared_ptr<InterfaceXMLReader> preader(CreateInterfaceReader(this, PT_Robot, robot, "robot", atts));
        bool bSuccess = ParseXMLFile(preader.get(), filename);
        if( !bSuccess ) {
            preader->Release();
            return NULL;
        }
        robot = (RobotBase*)preader->Release();
        robot->strXMLFilename = preader->_filename;
    }
    
    return robot;
}

KinBody* Environment::ReadKinBodyXML(KinBody* body, const char* filename, const char** atts)
{
    assert(body==NULL||body->GetEnv()==this);

    if( IsColladaFile(filename) ) {
        if( !RaveParseColladaFile(this, &body, filename) )
            return NULL;
    }
    else {
        boost::shared_ptr<InterfaceXMLReader> preader(CreateInterfaceReader(this, PT_KinBody, body, "kinbody", atts));
        bool bSuccess = ParseXMLFile(preader.get(), filename);
        if( !bSuccess ) {
            preader->Release();
            return NULL;
        }

        body = (KinBody*)preader->Release();
        body->strXMLFilename = preader->_filename;
    }

    return body;
}

void Environment::RegisterXMLReader(PluginType type, const char* xmltag, CreateXMLReaderFn pfn)
{
    ::RegisterXMLReader(type, xmltag, pfn);
}

void Environment::UnregisterXMLReader(PluginType type, const char* xmltag)
{
    ::UnregisterXMLReader(type, xmltag);
}

bool Environment::ParseXMLFile(BaseXMLReader* preader, const char* filename)
{
    MutexLock m(&_mutexXML);
    return RaveParseXMLFile(this,preader, filename);
}

bool Environment::ParseXMLData(BaseXMLReader* preader, const char* pdata, int len)
{
    MutexLock m(&_mutexXML);
    return RaveParseXMLData(this, preader, pdata, len);
}

void Environment::SetUniqueNetworkId(KinBody* pbody, int* pOutNetworkId)
{
    pthread_mutex_lock(&_mutexNetworkIds);
    int id = nNetworkIndex++;
    if( pOutNetworkId )
        *pOutNetworkId = id;
    assert( _mapBodies.find(id) == _mapBodies.end() );
    _mapBodies[id] = pbody;
    pthread_mutex_unlock(&_mutexNetworkIds);
}

void Environment::RemoveUniqueNetworkId(int networkid)
{
    pthread_mutex_lock(&_mutexNetworkIds);
    _mapBodies.erase(networkid);
    pthread_mutex_unlock(&_mutexNetworkIds);
}

KinBody* Environment::GetBodyFromNetworkId(int id)
{
    KinBody* pbody = NULL;
    pthread_mutex_lock(&_mutexNetworkIds);
    {
        MutexLock m(&_mutexBodies);
        map<int, KinBody*>::iterator it = _mapBodies.find(id);
        if( it != _mapBodies.end() )
            pbody = it->second;
    }
    pthread_mutex_unlock(&_mutexNetworkIds);
    return pbody;
}

bool Environment::Triangulate(KinBody::Link::TRIMESH& trimesh, const KinBody* pbody)
{
    if( pbody == NULL )
        return false;

    FOREACHC(it, pbody->GetLinks())
        trimesh.Append((*it)->GetCollisionData(), (*it)->GetTransform());
    
    return true;
}

bool Environment::TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts, const wchar_t* pName)
{
    MutexLock m(&_mutexBodies);
    FOREACH(itbody, _vecbodies) {
        
        RobotBase* robot = NULL;
        if( (*itbody)->IsRobot() )
            robot = (RobotBase*)(*itbody);
        
        switch(opts) {
            case TO_Obstacles:
                if( robot == NULL ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;

            case TO_Robots:
                if( robot != NULL ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;

            case TO_Everything:
                Triangulate(trimesh, (*itbody));
                break;

            case TO_Body:
                if( pName != NULL && wcscmp((*itbody)->GetName(), pName) == 0 ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;

            case TO_AllExceptBody:
                if( pName == NULL || wcscmp((*itbody)->GetName(), pName) != 0 ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;
        }
    }

    return true;
}

bool Environment::AttachViewer(RaveViewerBase* pnewviewer)
{
    if( _pCurrentViewer == pnewviewer )
        return true;

    if( _pCurrentViewer != NULL ) {
        // freezes because internal thread in viewer could have exited
        //_pCurrentViewer->Reset();
        _pCurrentViewer->quitmainloop();
    }

    _pCurrentViewer = pnewviewer;
    if( _pCurrentViewer == NULL )
        _pCurrentViewer = &_dummyviewer;

    assert(_pCurrentViewer != NULL && _pCurrentViewer->GetEnv() == this);
    return true;
}

RaveViewerBase* Environment::GetViewer() const
{
    return _pCurrentViewer;
}

void* Environment::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
{
    return _pCurrentViewer->plot3(ppoints, numPoints, stride, fPointSize, color, drawstyle);
}

void* Environment::plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle)
{
    return _pCurrentViewer->plot3(ppoints, numPoints, stride, fPointSize, colors, drawstyle);
}

void* Environment::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    return _pCurrentViewer->drawlinestrip(ppoints, numPoints, stride, fwidth,color);
}

void* Environment::drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    return _pCurrentViewer->drawlinestrip(ppoints, numPoints, stride, fwidth,colors);
}

void* Environment::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
{
    return _pCurrentViewer->drawlinelist(ppoints, numPoints, stride, fwidth,color);
}

void* Environment::drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
{
    return _pCurrentViewer->drawlinelist(ppoints, numPoints, stride, fwidth,colors);
}

void* Environment::drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
{
    return _pCurrentViewer->drawarrow(p1,p2,fwidth,color);
}

void* Environment::drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
{
    return _pCurrentViewer->drawbox(vpos, vextents);
}

void* Environment::drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
{
    return _pCurrentViewer->drawtrimesh(ppoints, stride, pIndices, numTriangles, color);
}


void Environment::closegraph(void* handle)
{
    if( handle != NULL )
        _pCurrentViewer->closegraph(handle);
}

bool Environment::GetFractionOccluded(KinBody* pbody, int width, int height, float nearPlane, float farPlane, const RaveTransform<float>& extrinsic, const float* pKK, double& fracOccluded)
{
    return _pCurrentViewer->GetFractionOccluded(pbody, width, height, nearPlane, farPlane, extrinsic, pKK, fracOccluded);
}

bool Environment::GetCameraImage(void* pMemory, int width, int height, const RaveTransform<float>& t, const float* pKK)
{
    return _pCurrentViewer->GetCameraImage(pMemory, width, height, t, pKK);
}

bool Environment::WriteCameraImage(int width, int height, const RaveTransform<float>& t, const float* pKK, const char* fileName, const char* extension)
{
    return _pCurrentViewer->WriteCameraImage(width, height, t, pKK, fileName, extension);
}  

void Environment::StartSimulation(dReal fDeltaTime, bool bRealTime)
{
    if( !IsPhysicsLocked() ) {
        RAVELOG_ERRORA("StartSimulation need to lock physics! Ignoring lock...\n");
    }
    _fDeltaSimTime = fDeltaTime;
    _bEnableSimulation = true;
    _bRealTime = bRealTime;
    _nCurSimTime = 0;
    _nStartSimTime = GetMicroTime();
}

void Environment::StopSimulation()
{
    if( !IsPhysicsLocked() ) {
        RAVELOG_ERRORA("StopSimulation need to lock physics! Ignoring lock...\n");
    }
    _bEnableSimulation = false;
    _fDeltaSimTime = 1.0f;
}

unsigned int Environment::RandomInt()
{
    return genrand_int32();
}

void Environment::RandomInt(unsigned int n, std::vector<int>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_int32();
}

float Environment::RandomFloat()
{
    return (float)genrand_real1();
}

void Environment::RandomFloat(unsigned int n, std::vector<float>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = (float)genrand_real1();
}

double Environment::RandomDouble()
{
    return genrand_res53();
}

void Environment::RandomDouble(unsigned int n, std::vector<double>& v)
{
    v.resize(n);
    FOREACH(it, v) *it = genrand_res53();
}

/// advance the simulation by the given time step
void Environment::StepSimulation(dReal fTimeStep)
{
    if( !IsPhysicsLocked() ) {
        RAVELOG_ERRORA("StepSimulation need to lock physics! Ignoring lock...\n");
        return;
    }

    uint64_t step = (uint64_t)ceilf(1000000.0 * (double)fTimeStep);
    fTimeStep = (dReal)((double)step * 0.000001);

    // call the physics first to get forces
    _pPhysicsEngine->SimulateStep(fTimeStep);

    vector<KinBody*> vecbodies;
    int nbodiesstamp;

    // make a copy instead of locking the mutex pointer since will be calling into user functions
    {
        MutexLock m(&_mutexBodies);
        vecbodies = _vecbodies; 
        nbodiesstamp = _nBodiesModifiedStamp;
    }
    
    FOREACH(it, vecbodies) {
        if( nbodiesstamp != _nBodiesModifiedStamp ) {
            // changed so have to check if current pointer is still active
            MutexLock m(&_mutexBodies);
            if( std::find(_vecbodies.begin(), _vecbodies.end(), *it) == _vecbodies.end() )
                continue;
        }
        (*it)->SimulationStep(fTimeStep);
    }

    {
        MutexLock m(&_mutexProblems);
        FOREACH(itprob, listProblems)
            (*itprob)->SimulationStep(fTimeStep);
    }
}

void* Environment::main(void* p)
{
    return ((Environment*)p)->_main();
}

void* Environment::_main()
{
    FOREACH(it, _vplugindirs) {
        if( it->size() > 0 )
            GetDatabase().AddDirectory(it->c_str());
    }

    // set a collision checker, don't call EnvironmentBase::CreateCollisionChecker
    const char* checker_prefs[] = {"ode", "bullet"}; // ode takes priority since bullet has some bugs with deleting bodies
    for(int i = 0; i < (int)ARRAYSIZE(checker_prefs); ++i) {
        _localchecker.reset(_pdatabase->CreateCollisionChecker(this, checker_prefs[i]));
        if( !!_localchecker )
            break;
    }

    if( !_localchecker ) { // take any collision checker
        PLUGININFO info;

        FOREACHC(itplugin, GetDatabase().GetPlugins()) {
            FOREACHC(itname, itplugin->info.collisioncheckers) {
                _localchecker.reset(_pdatabase->CreateCollisionChecker(this, itname->c_str()));
                if( !!_localchecker )
                    break;
            }

            if( !!_localchecker )
                break;
        }
    }

    if( !!_localchecker )
        RAVELOG_DEBUGA("using %s collision checker\n", _localchecker->GetXMLId());
    else
        RAVELOG_WARNA("failed to find any collision checker.\n");
    SetCollisionChecker(_localchecker.get());
    AddIKSolvers();
    _bPluginsLoaded = true;
    _nStartSimTime = GetMicroTime();

    uint64_t nLastSleptTime = GetMicroTime();
    uint64_t nMaxConsecutiveSimTime = 500000; // force reupdate every 500ms
    uint64_t nLastUpdateTime = GetMicroTime();

    while( !_bDestroying ) {

        if( _pserver != NULL && _pserver->IsInit() ) {
            uint64_t startwork = GetMicroTime();
            _pserver->Worker();
            _nStartSimTime += GetMicroTime()-startwork;
        }

        uint64_t curtime = GetMicroTime();
        uint64_t deltatime = (uint64_t)(1000000.0 * _fDeltaSimTime);
        bool bDoSimulation = !_bRealTime || curtime-_nStartSimTime >= _nCurSimTime+deltatime;
        if( bDoSimulation ) {
            if( _bEnableSimulation ) {
                LockPhysics(true);
                StepSimulation(_fDeltaSimTime);
                LockPhysics(false);
            }
            _nCurSimTime += deltatime;
        }

        if( !bDoSimulation || curtime-nLastSleptTime > nMaxConsecutiveSimTime ) {
            nLastSleptTime = curtime;
            Sleep(1);
        }

        if( GetMicroTime()-nLastUpdateTime > 10000 ) {
            LockPhysics(true);
            nLastUpdateTime = GetMicroTime();
            {
                MutexLock mbodies(&_mutexBodies);

                // remove destroyed bodies
                _CleanRemovedBodies();

                // updated the published bodies
                _vPublishedBodies.resize(_vecbodies.size());
            
                vector<BODYSTATE>::iterator itstate = _vPublishedBodies.begin();
                FOREACH(itbody, _vecbodies) {
                    itstate->pbody = *itbody;
                    (*itbody)->GetBodyTransformations(itstate->vectrans);
                    (*itbody)->GetJointValues(itstate->jointvalues);
                    itstate->strname =(*itbody)->GetName();
                    itstate->pguidata = (*itbody)->GetGuiData();
                    itstate->networkid = (*itbody)->GetNetworkId();
                    ++itstate;
                }
            }
            LockPhysics(false);
        }
    }

    {
        MutexLock m(&_mutexDestroy);
        if( !_bDestroyed ) {
            if( _pserver != NULL )
                _pserver->Destroy();
            Destroy();
            _bDestroyed = true;
        }
    }

    return NULL;
}

bool Environment::AttachServer(RaveServerBase* pserver)
{
    if( _pserver == pserver )
        return true;

    if( _pserver != NULL )
        _pserver->Destroy();

    _pserver = pserver;
    assert(_pserver == NULL || _pserver->GetEnv() == this);
    return true;
}
