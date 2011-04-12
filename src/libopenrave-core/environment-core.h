// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_ENVIRONMENT_H
#define RAVE_ENVIRONMENT_H

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#define CHECK_INTERFACE(pinterface) { \
        if( (pinterface)->GetEnv() != shared_from_this() ) \
            throw openrave_exception(str(boost::format("Interface %s:%s is from a different environment")%RaveGetInterfaceName((pinterface)->GetInterfaceType())%(pinterface)->GetXMLId()),ORE_InvalidArguments); \
    } \

#define CHECK_COLLISION_BODY(body) { \
        CHECK_INTERFACE(body); \
        if( !(body)->GetCollisionData() ) { \
            RAVELOG_WARN("body %s not added to enviornment!\n", (body)->GetName().c_str()); \
            return false; \
        } \
    }

class Environment : public EnvironmentBase
{
 public:
    Environment() : EnvironmentBase()
    {
        _homedirectory = RaveGetHomeDirectory();
        RAVELOG_DEBUG("setting openrave cache directory to %s\n",_homedirectory.c_str());

        _nBodiesModifiedStamp = 0;
        _nEnvironmentIndex = 1;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = GetMicroTime();
        _bRealTime = true;
        _bInit = false;
        _bEnableSimulation = true; // need to start by default
    
        {
            bool bExists=false;
            RaveParseDirectories(getenv("OPENRAVE_DATA"), _vdatadirs);
            string installdir = OPENRAVE_DATA_INSTALL_DIR;
#ifdef HAVE_BOOST_FILESYSTEM
            if( !boost::filesystem::is_directory(boost::filesystem::path(installdir)) ) {
#ifdef _WIN32
                HKEY hkey;
                if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, TEXT("Software\\OpenRAVE\\"OPENRAVE_VERSION_STRING), 0, KEY_QUERY_VALUE, &hkey) == ERROR_SUCCESS) {
                    DWORD dwType = REG_SZ;
                    CHAR szInstallRoot[4096]; // dont' take chances, it is windows
                    DWORD dwSize = sizeof(szInstallRoot);
                    RegQueryValueEx(hkey, TEXT("InstallRoot"), NULL, &dwType, (PBYTE)szInstallRoot, &dwSize);
                    RegCloseKey(hkey);
                    installdir.assign(szInstallRoot);
                    installdir += str(boost::format("%cshare%copenrave")%s_filesep%s_filesep);
                }
                else
#endif
                {
                    RAVELOG_WARN(str(boost::format("%s doesn't exist")%installdir));
                }
            }
            boost::filesystem::path datafilename = boost::filesystem::system_complete(boost::filesystem::path(installdir, boost::filesystem::native));
            FOREACH(itname, _vdatadirs) {
                if( datafilename == boost::filesystem::system_complete(boost::filesystem::path(*itname, boost::filesystem::native)) ) {
                    bExists = true;
                    break;
                }
            }
#else
            string datafilename=installdir;
            FOREACH(itname, _vdatadirs) {
                if( datafilename == installdir ) {
                    bExists = true;
                    break;
                }
            }
#endif
            if( !bExists ) {
                _vdatadirs.push_back(installdir);
            }
       }
    }

    virtual ~Environment() {
        Destroy();
    }

    virtual void Init()
    {
        boost::mutex::scoped_lock(_mutexInit);
        if( _bInit ) {
            RAVELOG_WARN("environment is already initialized, ignoring\n");
            return;
        }

        _nBodiesModifiedStamp = 0;
        _nEnvironmentIndex = 1;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = GetMicroTime();
        _bRealTime = true;
        _bEnableSimulation = true; // need to start by default

        if( !_pCurrentChecker ) {
            _pCurrentChecker.reset(new DummyCollisionChecker(shared_from_this()));
        }
        if( !_pCurrentViewer ) {
            _pCurrentViewer.reset(new DummyViewer(shared_from_this()));
        }
        if( !_pPhysicsEngine ) {
            _pPhysicsEngine.reset(new DummyPhysicsEngine(shared_from_this()));
        }

        // try to set init as early as possible since will be calling into user code
        _bInit = true;

        // set a collision checker, don't call EnvironmentBase::CreateCollisionChecker
        CollisionCheckerBasePtr localchecker;
        boost::array<string,3> checker_prefs = {{"ode", "bullet", "pqp"}}; // ode takes priority since bullet has some bugs with deleting bodies
        FOREACH(itchecker,checker_prefs) {
            localchecker = RaveCreateCollisionChecker(shared_from_this(), *itchecker);
            if( !!localchecker ) {
                break;
            }
        }

        if( !localchecker ) { // take any collision checker
            std::map<InterfaceType, std::vector<std::string> > interfacenames;
            RaveGetLoadedInterfaces(interfacenames);
            std::map<InterfaceType, std::vector<std::string> >::const_iterator itnames =interfacenames.find(PT_CollisionChecker);
            if( itnames != interfacenames.end() ) {
                FOREACHC(itname, itnames->second) {
                    localchecker = RaveCreateCollisionChecker(shared_from_this(), *itname);
                    if( !!localchecker ) {
                        break;
                    }
                }
            }
        }

        if( !!localchecker ) {
            RAVELOG_DEBUG("using %s collision checker\n", localchecker->GetXMLId().c_str());
            SetCollisionChecker(localchecker);
        }
        else {
            RAVELOG_WARN("failed to find any collision checker.\n");
        }
        if( !!_threadSimulation ) {
            _threadSimulation->join();
        }

        _threadSimulation.reset(new boost::thread(boost::bind(&Environment::_SimulationThread,this)));
    }

    virtual void Destroy()
    {
        boost::mutex::scoped_lock lockdestroy(_mutexInit);
        if( !_bInit ) {
            RAVELOG_VERBOSE("environment is already destroyed\n");
            return;
        }

        // destruction order is *very* important, don't touch it without consultation
        _bInit = false;

        RAVELOG_VERBOSE("Environment destructor\n");
        if( !!_threadSimulation ) {
            _threadSimulation->join(); // might not return?
        }
        _threadSimulation.reset();

        // destroy the problems (their destructors could attempt to lock environment, so have to do it before global lock)
        // however, do not clear the _listProblems yet
        RAVELOG_DEBUG("destroy problems\n");
        list<ProblemInstancePtr> listProblems;
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            listProblems = _listProblems;
        }
        FOREACH(itprob,listProblems) {
            (*itprob)->Destroy();
        }
        listProblems.clear();

        RAVELOG_DEBUG("resetting raveviewer\n");
        if( !!_pCurrentViewer ) {
            _pCurrentViewer->Reset();
            _pCurrentViewer->quitmainloop();
        }

        // lock the environment
        {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            _bEnableSimulation = false;    
            if( !!_pPhysicsEngine ) {
                _pPhysicsEngine->DestroyEnvironment();
            }
            if( !!_pCurrentChecker ) {
                _pCurrentChecker->DestroyEnvironment();
            }
            
            // clear internal interface lists
            {
                boost::mutex::scoped_lock lock(_mutexInterfaces);
                // release all grabbed
                FOREACH(itrobot,_vecrobots) {
                    (*itrobot)->ReleaseAllGrabbed();
                }
                FOREACH(itbody,_vecbodies) {
                    (*itbody)->Destroy();
                }
                _vecbodies.clear();
                FOREACH(itrobot,_vecrobots) {
                    (*itrobot)->Destroy();
                }
                _vecrobots.clear();
                _vPublishedBodies.clear();
                _nBodiesModifiedStamp++;
                FOREACH(itsensor,_listSensors) {
                    (*itsensor)->Configure(SensorBase::CC_PowerOff);
                    (*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
                }
                _listSensors.clear();
                _listProblems.clear();
                _listOwnedInterfaces.clear();
            }
        }

        // release all other interfaces, not necessary to hold a mutex?
        _pCurrentChecker.reset();
        _pPhysicsEngine.reset();
        _pCurrentViewer.reset();
        RAVELOG_VERBOSE("Environment destroyed\n");
    }

    virtual void Reset()
    {
        // destruction order is *very* important, don't touch it without consultation
        RAVELOG_DEBUG("resetting raveviewer\n");
        if( !!_pCurrentViewer ) {
            _pCurrentViewer->Reset();
        }
    
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment();
        }
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);

            FOREACH(itbody,_vecbodies) {
                (*itbody)->_environmentid=0;
                (*itbody)->Destroy();
            }
            _vecbodies.clear();
            FOREACH(itrobot,_vecrobots) {
                (*itrobot)->_environmentid=0;
                (*itrobot)->Destroy();
            }
            _vecrobots.clear();
            _vPublishedBodies.clear();
            _nBodiesModifiedStamp++;
            
            _mapBodies.clear();

            FOREACH(itsensor,_listSensors) {
                (*itsensor)->Configure(SensorBase::CC_PowerOff);
                (*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
            }
            _listSensors.clear();
        }

        list<ProblemInstancePtr> listProblems;
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            listProblems = _listProblems;
        }

        FOREACH(itproblem,listProblems) {
            (*itproblem)->Reset();
        }
        listProblems.clear();
        _listOwnedInterfaces.clear();

        if( !!_pCurrentChecker ) {
            _pCurrentChecker->InitEnvironment();
        }
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->InitEnvironment();
        }
    }

    virtual UserDataPtr GlobalState() { return RaveGlobalState(); }

    virtual void GetPluginInfo(std::list< std::pair<std::string, PLUGININFO> >& plugins) { RaveGetPluginInfo(plugins); }
    virtual void GetLoadedInterfaces(std::map<InterfaceType, std::vector<std::string> >& interfacenames) const { RaveGetLoadedInterfaces(interfacenames); }
    virtual bool LoadPlugin(const std::string& pname) { return RaveLoadPlugin(pname); }
    virtual void ReloadPlugins() { RaveReloadPlugins(); }
    virtual bool HasInterface(InterfaceType type, const string& interfacename) const { return RaveHasInterface(type,interfacename); }

    virtual InterfaceBasePtr CreateInterface(InterfaceType type,const std::string& pinterfacename) { return RaveCreateInterface(shared_from_this(),type,pinterfacename); }
    virtual RobotBasePtr CreateRobot(const std::string& pname) { return RaveCreateRobot(shared_from_this(), pname); }
    virtual KinBodyPtr CreateKinBody(const std::string& pname) { return RaveCreateKinBody(shared_from_this(), pname); }
    virtual TrajectoryBasePtr CreateTrajectory(int nDOF) { return RaveCreateTrajectory(shared_from_this(), nDOF); }
    virtual PlannerBasePtr CreatePlanner(const std::string& pname) { return RaveCreatePlanner(shared_from_this(),pname); }
    virtual SensorSystemBasePtr CreateSensorSystem(const std::string& pname) { return RaveCreateSensorSystem(shared_from_this(),pname); }
    virtual ControllerBasePtr CreateController(const std::string& pname) { return RaveCreateController(shared_from_this(),pname); }
    virtual ProblemInstancePtr CreateProblem(const std::string& pname) { return RaveCreateProblem(shared_from_this(),pname); }
    virtual IkSolverBasePtr CreateIkSolver(const std::string& pname) { return RaveCreateIkSolver(shared_from_this(),pname); }
    virtual PhysicsEngineBasePtr CreatePhysicsEngine(const std::string& pname) { return RaveCreatePhysicsEngine(shared_from_this(),pname); }
    virtual SensorBasePtr CreateSensor(const std::string& pname) { return RaveCreateSensor(shared_from_this(),pname); }
    virtual CollisionCheckerBasePtr CreateCollisionChecker(const std::string& pname) { return RaveCreateCollisionChecker(shared_from_this(),pname); }
    virtual ViewerBasePtr CreateViewer(const std::string& pname) { return RaveCreateViewer(shared_from_this(),pname); }

    virtual void OwnInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        _listOwnedInterfaces.push_back(pinterface);
    }
    virtual void DisownInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        _listOwnedInterfaces.remove(pinterface);
    }

    virtual EnvironmentBasePtr CloneSelf(int options)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::shared_ptr<Environment> penv(new Environment());
        penv->Clone(boost::static_pointer_cast<Environment const>(shared_from_this()),options);
        return penv;
    }
    
    virtual int LoadProblem(ProblemInstancePtr prob, const std::string& cmdargs)
    {
        CHECK_INTERFACE(prob);
        int ret = prob->main(cmdargs);
        if( ret != 0 ) {
            RAVELOG_WARN(str(boost::format("Error %d with executing problem %s\n")%ret%prob->GetXMLId()));
        }
        else {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            _listProblems.push_back(prob);
        }

        return ret;
    }

    bool RemoveProblem(ProblemInstancePtr prob)
    {
        return Remove(InterfaceBasePtr(prob));
    }

    boost::shared_ptr<void> GetLoadedProblems(std::list<ProblemInstancePtr>& listProblems) const
    {
        boost::shared_ptr<boost::mutex::scoped_lock> plock(new boost::mutex::scoped_lock(_mutexInterfaces));
        listProblems = _listProblems;
        return plock;
    }

    virtual bool Load(const std::string& filename)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        bool bSuccess;
        if( _IsColladaFile(filename) ) {
            OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
            bSuccess = RaveParseColladaFile(shared_from_this(), filename, AttributesList());
        }
        else {
            bSuccess = ParseXMLFile(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(),AttributesList()), filename);
        }

        if( !bSuccess ) {
            RAVELOG_WARN("load failed on file %s\n", filename.c_str());
        }
    
        return bSuccess;
    }

    virtual bool LoadXMLData(const std::string& data)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( !ParseXMLData(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(),AttributesList()),data) ) {
            RAVELOG_WARN("load failed environment data\n");
            return false;
        }
        return true;
    }
    
    virtual bool Save(const std::string& filename)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        try {
            RaveWriteColladaFile(shared_from_this(),filename);
            return true;
        }
        catch(const openrave_exception& ex) {
            RAVELOG_ERROR("save filed: %s\n",ex.what());
        }
        return false;
    }

    virtual bool AddKinBody(KinBodyPtr pbody, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pbody);
        if( !IsValidName(pbody->GetName()) ) {
            throw openrave_exception(str(boost::format("body name: \"%s\" is not valid")%pbody->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(pbody),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=pbody->GetName(),newname;
            for(int i = 0;;++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                pbody->SetName(newname);
                if( IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(pbody), false) ) {
                    break;
                }
            }
        }
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            _vecbodies.push_back(pbody);
            SetEnvironmentId(pbody);
            _nBodiesModifiedStamp++;
        }
        pbody->_ComputeInternalInformation();
        _pCurrentChecker->InitKinBody(pbody);
        _pPhysicsEngine->InitKinBody(pbody);
        return true;
    }

    virtual bool AddRobot(RobotBasePtr robot, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(robot);
        if( !robot->IsRobot() ) {
            throw openrave_exception(str(boost::format("body \"%s\" is not a robot")%robot->GetName()));
        }
        if( !IsValidName(robot->GetName()) ) {
            throw openrave_exception(str(boost::format("body name: \"%s\" is not valid")%robot->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(robot),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=robot->GetName(),newname;
            for(int i = 0;;++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                robot->SetName(newname);
                if( IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(robot),false) ) {
                    break;
                }
            }
        }
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            _vecbodies.push_back(robot);
            _vecrobots.push_back(robot);
            SetEnvironmentId(robot);
            _nBodiesModifiedStamp++;
        }
        robot->_ComputeInternalInformation();
        _pCurrentChecker->InitKinBody(robot);
        _pPhysicsEngine->InitKinBody(robot);
        return true;
    }

    virtual bool AddSensor(SensorBasePtr psensor, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(psensor);
        if( !IsValidName(psensor->GetName()) ) {
            throw openrave_exception(str(boost::format("sensor name: \"%s\" is not valid")%psensor->GetName()));
        }
        if( !_CheckUniqueName(SensorBaseConstPtr(psensor),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=psensor->GetName(),newname;
            for(int i = 0;;++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                psensor->SetName(newname);
                if( IsValidName(newname) && _CheckUniqueName(SensorBaseConstPtr(psensor),false) ) {
                    break;
                }
            }
        }
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            _listSensors.push_back(psensor);
        }
        psensor->Configure(SensorBase::CC_PowerOn);
        return true;
    }

    virtual bool RemoveKinBody(KinBodyPtr pbody)
    {
        return Remove(InterfaceBasePtr(pbody));
    }

    virtual bool Remove(InterfaceBasePtr interface)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        CHECK_INTERFACE(interface);
        switch(interface->GetInterfaceType()) {
        case PT_KinBody:
        case PT_Robot: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(interface);
            vector<KinBodyPtr>::iterator it = std::find(_vecbodies.begin(), _vecbodies.end(), pbody);
            if( it == _vecbodies.end() ) {
                return false;
            }
            // before deleting, make sure no robots are grabbing it!!
            FOREACH(itrobot, _vecrobots) {
                if( (*itrobot)->IsGrabbing(*it) ) {
                    RAVELOG_WARN("destroy %s already grabbed by robot %s!\n", pbody->GetName().c_str(), (*itrobot)->GetName().c_str());
                    (*itrobot)->Release(pbody);
                }
            }
            
            if( (*it)->IsRobot() ) {
                vector<RobotBasePtr>::iterator itrobot = std::find(_vecrobots.begin(), _vecrobots.end(), RaveInterfaceCast<RobotBase>(pbody));
                if( itrobot != _vecrobots.end() ) {
                    _vecrobots.erase(itrobot);
                }
            }            
            (*it)->SetPhysicsData(UserDataPtr());
            (*it)->SetCollisionData(UserDataPtr());
            RemoveEnvironmentId(pbody);
            _vecbodies.erase(it);
            _nBodiesModifiedStamp++;
            return true;
        }
        case PT_Sensor: {
            SensorBasePtr psensor = RaveInterfaceCast<SensorBase>(interface);
            list<SensorBasePtr>::iterator it = std::find(_listSensors.begin(), _listSensors.end(), psensor);
            if( it != _listSensors.end() ) {
                (*it)->Configure(SensorBase::CC_PowerOff);
                _listSensors.erase(it);
                return true;
            }
            break;
        }
        case PT_ProblemInstance: {
            ProblemInstancePtr prob = RaveInterfaceCast<ProblemInstance>(interface);
            list<ProblemInstancePtr>::iterator itprob = find(_listProblems.begin(), _listProblems.end(), prob);
            if( itprob != _listProblems.end() ) {
                (*itprob)->Destroy();
                _listProblems.erase(itprob);
                return true;
            }
            break;
        }
        default:
            RAVELOG_WARN(str(boost::format("unmanaged interfaces of type %s cannot be removed\n")%RaveGetInterfaceName(interface->GetInterfaceType())));
            break;
        }
        return false;
    }

    virtual KinBodyPtr GetKinBody(const std::string& pname)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecbodies) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        //RAVELOG_VERBOSE(str(boost::format("Environment::GetKinBody - Error: Unknown body %s\n")%pname));
        return KinBodyPtr();
    }

    virtual RobotBasePtr GetRobot(const std::string& pname)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecrobots) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        //RAVELOG_VERBOSE(str(boost::format("Environment::GetRobot - Error: Unknown body %s\n")%pname));
        return RobotBasePtr();
    }

    virtual SensorBasePtr GetSensor(const std::string& name)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(itrobot,_vecrobots) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor && psensor->GetName() == name) {
                    return psensor;
                }
            }
        }
        FOREACH(itsensor,_listSensors) {
            if( (*itsensor)->GetName() == name ) {
                return *itsensor;
            }
        }
        return SensorBasePtr();
    }

    virtual bool SetPhysicsEngine(PhysicsEngineBasePtr pengine)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        _pPhysicsEngine = pengine;
        if( !_pPhysicsEngine ) {
            RAVELOG_DEBUG("disabling physics\n");
            _pPhysicsEngine.reset(new DummyPhysicsEngine(shared_from_this()));
        }
        else {
            RAVELOG_DEBUG(str(boost::format("setting %s physics engine\n")%_pPhysicsEngine->GetXMLId()));
        }
        _pPhysicsEngine->InitEnvironment();
        return true;
    }

    virtual PhysicsEngineBasePtr GetPhysicsEngine() const { return _pPhysicsEngine; }

    static void __erase_collision_iterator(boost::weak_ptr<Environment> pweak, std::list<CollisionCallbackFn>::iterator* pit)
    {
        if( !!pit ) {
            boost::shared_ptr<Environment> penv = pweak.lock();
            if( !!penv ) {
                penv->_listRegisteredCollisionCallbacks.erase(*pit);
            }
            delete pit;
        }
    }

    virtual boost::shared_ptr<void> RegisterCollisionCallback(const CollisionCallbackFn& callback) {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        boost::shared_ptr<Environment> penv = boost::static_pointer_cast<Environment>(shared_from_this());
        return boost::shared_ptr<void>(new std::list<CollisionCallbackFn>::iterator(_listRegisteredCollisionCallbacks.insert(_listRegisteredCollisionCallbacks.end(),callback)), boost::bind(Environment::__erase_collision_iterator,boost::weak_ptr<Environment>(penv),_1));
    }
    virtual bool HasRegisteredCollisionCallbacks() const
    {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        return _listRegisteredCollisionCallbacks.size() > 0;
    }
    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>& listcallbacks) const {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        listcallbacks = _listRegisteredCollisionCallbacks;
    }

    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( _pCurrentChecker == pchecker ) {
            return true;
        }        
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment(); // delete all resources
        }
        _pCurrentChecker = pchecker;
        if( !_pCurrentChecker ) {
            RAVELOG_DEBUG("disabling collisions\n");
            _pCurrentChecker.reset(new DummyCollisionChecker(shared_from_this()));
        }
        else {
            RAVELOG_DEBUG(str(boost::format("setting %s collision checker\n")%_pCurrentChecker->GetXMLId()));
            FOREACH(itbody,_vecbodies) {
                (*itbody)->_ResetInternalCollisionCache();
            }
        }
        return _pCurrentChecker->InitEnvironment();
    }

    virtual CollisionCheckerBasePtr GetCollisionChecker() const { return _pCurrentChecker; }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        return _pCurrentChecker->CheckCollision(pbody1,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        CHECK_COLLISION_BODY(pbody2);
        return _pCurrentChecker->CheckCollision(pbody1,pbody2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report )
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(plink,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink1->GetParent());
        CHECK_COLLISION_BODY(plink2->GetParent());
        return _pCurrentChecker->CheckCollision(plink1,plink2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(plink,pbody,report);
    }
    
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(plink,vbodyexcluded,vlinkexcluded,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(pbody,vbodyexcluded,vlinkexcluded,report);
    }
    
    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(ray,plink,report);
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(ray,pbody,report);
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report)
    {
        return _pCurrentChecker->CheckCollision(ray,report);
    }

    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckSelfCollision(pbody,report);
    }

    virtual void StepSimulation(dReal fTimeStep)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        uint64_t step = (uint64_t)ceil(1000000.0 * (double)fTimeStep);
        fTimeStep = (dReal)((double)step * 0.000001);

        // call the physics first to get forces
        _pPhysicsEngine->SimulateStep(fTimeStep);

        // make a copy instead of locking the mutex pointer since will be calling into user functions
        vector<KinBodyPtr> vecbodies;
        vector<RobotBasePtr> vecrobots;
        list<SensorBasePtr> listSensors;
        list<ProblemInstancePtr> listProblems;
        {
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            vecbodies = _vecbodies;
            vecrobots = _vecrobots;
            listSensors = _listSensors;
            listProblems = _listProblems;
        }
        
        FOREACH(it, vecbodies) {
            if( (*it)->GetEnvironmentId() ) { // have to check if valid
                (*it)->SimulationStep(fTimeStep);
            }
        }
        FOREACH(itprob, listProblems) {
            (*itprob)->SimulationStep(fTimeStep);
        }

        // simulate the sensors last (ie, they always reflect the most recent bodies
        FOREACH(itsensor, listSensors) {
            (*itsensor)->SimulationStep(fTimeStep);
        }
        FOREACH(itrobot, vecrobots) {
            FOREACH(itsensor, (*itrobot)->GetAttachedSensors()) {
                if( !!(*itsensor)->GetSensor() ) {
                    (*itsensor)->GetSensor()->SimulationStep(fTimeStep);
                }
            }
        }
        _nCurSimTime += step;
    }

    virtual EnvironmentMutex& GetMutex() const { return _mutexEnvironment; }

    virtual void GetBodies(std::vector<KinBodyPtr>& bodies) const
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        bodies = _vecbodies;
    }

    virtual void GetRobots(std::vector<RobotBasePtr>& robots) const
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        robots = _vecrobots;
    }

    virtual void GetSensors(std::vector<SensorBasePtr>& vsensors) const
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        vsensors.resize(0);
        FOREACHC(itrobot,_vecrobots) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor ) {
                    vsensors.push_back(psensor);
                }
            }
        }
        vsensors.insert(vsensors.end(),_listSensors.begin(),_listSensors.end());
    }
    
    /// triangulation of the body including its current transformation. trimesh will be appended the new data.
    virtual bool Triangulate(KinBody::Link::TRIMESH& trimesh, KinBodyConstPtr pbody)
    {
        if( !pbody ) {
            return false;
        }
        EnvironmentMutex::scoped_lock lockenv(GetMutex()); // reading collision data, so don't want anyone modifying it
        FOREACHC(it, pbody->GetLinks()) {
            trimesh.Append((*it)->GetCollisionData(), (*it)->GetTransform());
        }
        return true;
    }

    /// general triangulation of the whole scene. trimesh will be appended the new data.
    virtual bool TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts,const std::string& name)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        FOREACH(itbody, _vecbodies) {
            RobotBasePtr robot;
            if( (*itbody)->IsRobot() ) {
                robot = RaveInterfaceCast<RobotBase>(*itbody);
            }
            switch(opts) {
            case TO_Obstacles:
                if( !robot ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;

            case TO_Robots:
                if( !!robot ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;
            case TO_Everything:
                Triangulate(trimesh, (*itbody));
                break;
            case TO_Body:
                if( (*itbody)->GetName() == name ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;
            case TO_AllExceptBody:
                if( (*itbody)->GetName() == name ) {
                    Triangulate(trimesh, (*itbody));
                }
                break;
            }
        }

        return true;
    }
    virtual bool TriangulateScene(KinBody::Link::TRIMESH& trimesh, TriangulateOptions opts)
    {
        return TriangulateScene(trimesh,opts,"");
    }

    virtual const std::string& GetHomeDirectory() const { return _homedirectory; }

    virtual RobotBasePtr ReadRobotXMLFile(const std::string& filename)
    {
        try {
            if( _IsColladaFile(filename) ) {
                RobotBasePtr robot;
                OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
                if( !RaveParseColladaFile(shared_from_this(), robot, filename, AttributesList()) ) {
                    return RobotBasePtr();
                }
                return robot;
            }
            else {
                InterfaceBasePtr pinterface = ReadInterfaceXMLFile(filename,AttributesList());
                BOOST_ASSERT(pinterface->GetInterfaceType() == PT_Robot );
                return RaveInterfaceCast<RobotBase>(pinterface);
            }
        }
        catch(const openrave_exception& ex) {
            RAVELOG_ERROR(str(boost::format("ReadRobotXMLFile exception: %s\n")%ex.what()));
        }
        return RobotBasePtr();
    }

    virtual RobotBasePtr ReadRobotXMLFile(RobotBasePtr robot, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) {
            robot->SetGuiData(UserDataPtr());
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            if( std::find(_vecrobots.begin(),_vecrobots.end(),robot) != _vecrobots.end() ) {
                throw openrave_exception(str(boost::format("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n")%robot->GetName()));
            }
        }

        if( _IsColladaFile(filename) ) {
            OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
            if( !RaveParseColladaFile(shared_from_this(), robot, filename, atts) ) {
                return RobotBasePtr();
            }
        }
        else {
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_Robot, pinterface, "robot", atts);
            if( !preader ) {
                return RobotBasePtr();
            }
            bool bSuccess = ParseXMLFile(preader, filename);
            preader->endElement("robot"); // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !bSuccess || !robot ) {
                return RobotBasePtr();
            }
            robot->__strxmlfilename = filename;
        }
    
        return robot;
    }

    virtual RobotBasePtr ReadRobotXMLData(RobotBasePtr robot, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) {
            robot->SetGuiData(UserDataPtr());
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            if( std::find(_vecrobots.begin(),_vecrobots.end(),robot) != _vecrobots.end() ) {
                throw openrave_exception(str(boost::format("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n")%robot->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else {
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_Robot, pinterface, "robot", atts);
            if( !preader ) {
                return RobotBasePtr();
            }
            bool bSuccess = ParseXMLData(preader, data);
            preader->endElement("robot"); // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !bSuccess || !robot ) {
                return RobotBasePtr();
            }
            robot->__strxmlfilename = preader->_filename;
        }
        return robot;
    }

    virtual KinBodyPtr ReadKinBodyXMLFile(const std::string& filename)
    {
        try {
            if( _IsColladaFile(filename) ) {
                KinBodyPtr body;
                OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
                if( !RaveParseColladaFile(shared_from_this(), body, filename, AttributesList()) ) {
                    return KinBodyPtr();
                }
                return body;
            }
            else {
                InterfaceBasePtr pinterface = ReadInterfaceXMLFile(filename,AttributesList());
                BOOST_ASSERT(pinterface->GetInterfaceType() == PT_KinBody );
                return RaveInterfaceCast<KinBody>(pinterface);
            }
        }
        catch(const openrave_exception& ex) {
            RAVELOG_ERROR(str(boost::format("ReadRobotXMLFile exception: %s\n")%ex.what()));
        }
        return KinBodyPtr();
    }

    virtual KinBodyPtr ReadKinBodyXMLFile(KinBodyPtr body, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body ) {
            body->SetGuiData(UserDataPtr());
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            if( std::find(_vecbodies.begin(),_vecbodies.end(),body) != _vecbodies.end() ) {
                throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%body->GetName()));
            }
        }

        if( _IsColladaFile(filename) ) {
            OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
            if( !RaveParseColladaFile(shared_from_this(), body, filename, atts) ) {
                return KinBodyPtr();
            }
        }
        else {
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) {
                return KinBodyPtr();
            }
            bool bSuccess = ParseXMLFile(preader, filename);
            preader->endElement("kinbody"); // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !bSuccess || !body ) {
                return KinBodyPtr();
            }
            body->__strxmlfilename = filename;
        }

        return body;
    }

    virtual KinBodyPtr ReadKinBodyXMLData(KinBodyPtr body, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body ) {
            body->SetGuiData(UserDataPtr());
            boost::mutex::scoped_lock lock(_mutexInterfaces);
            if( std::find(_vecbodies.begin(),_vecbodies.end(),body) != _vecbodies.end() ) {
                throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%body->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else {
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) {
                return KinBodyPtr();
            }
            bool bSuccess = ParseXMLData(preader, data);
            preader->endElement("kinbody"); // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !bSuccess || !body ) {
                return KinBodyPtr();
            }
            body->__strxmlfilename = preader->_filename;
        }
        return body;
    }

    virtual InterfaceBasePtr ReadInterfaceXMLFile(const std::string& filename, const AttributesList& atts)
    {
        try {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),atts);
            if( !preader ) {
                return InterfaceBasePtr();
            }
            bool bSuccess = ParseXMLFile(preader, filename);
            boost::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = boost::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !bSuccess || !preadable || !preadable->_pinterface) {
                return InterfaceBasePtr();
            }
            preader->endElement(RaveGetInterfaceName(preadable->_pinterface->GetInterfaceType())); // have to end the tag!
            preadable->_pinterface->__strxmlfilename = filename;
            return preadable->_pinterface;
        }
        catch(const openrave_exception& ex) {
            RAVELOG_ERROR(str(boost::format("ReadInterfaceXMLFile exception: %s\n")%ex.what()));
        }
        return InterfaceBasePtr();
    }

    virtual InterfaceBasePtr ReadInterfaceXMLFile(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( (type == PT_KinBody || type == PT_Robot) && _IsColladaFile(filename) ) {
            if( type == PT_KinBody ) {
                BOOST_ASSERT(!pinterface|| (pinterface->GetInterfaceType()==PT_KinBody||pinterface->GetInterfaceType()==PT_Robot));
                KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
                OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
                if( !RaveParseColladaFile(shared_from_this(), pbody, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = pbody;
            }
            else if( type == PT_Robot ) {
                BOOST_ASSERT(!pinterface||pinterface->GetInterfaceType()==PT_Robot);
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
                OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
                if( !RaveParseColladaFile(shared_from_this(), probot, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = probot;
            }
            else {
                return InterfaceBasePtr();
            }
            pinterface->__strxmlfilename = filename;
        }
        else {
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
            boost::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = boost::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !!preadable ) {
                if( !ParseXMLFile(preader, filename) ) {
                    return InterfaceBasePtr();
                }
                preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType())); // have to end the tag!
                pinterface = preadable->_pinterface;
            }
            else {
                pinterface = ReadInterfaceXMLFile(filename,AttributesList());
                if( !!pinterface && pinterface->GetInterfaceType() != type ) {
                    return InterfaceBasePtr();
                }
            }
            pinterface->__strxmlfilename = filename;
        }
        return pinterface;
    }

    virtual InterfaceBasePtr ReadInterfaceXMLData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        // check for collada?
        BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
        if( !preader ) {
            return InterfaceBasePtr();
        }
        bool bSuccess = ParseXMLData(preader, data);
        preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType())); // have to end the tag!
        if( !bSuccess ) {
            return InterfaceBasePtr();
        }
        pinterface->__strxmlfilename = preader->_filename;
        return pinterface;
    }

    virtual boost::shared_ptr<KinBody::Link::TRIMESH> ReadTrimeshFile(boost::shared_ptr<KinBody::Link::TRIMESH> ptrimesh, const std::string& filename, const AttributesList& atts) {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
        boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::FindFile(filename);
        if( !filedata ) {
            return boost::shared_ptr<KinBody::Link::TRIMESH>();
        }
        Vector vscale(1,1,1);
        RaveVector<float> diffuseColor, ambientColor;
        float ftransparency;
        if( !ptrimesh ) {
            ptrimesh.reset(new KinBody::Link::TRIMESH());
        }
        if( !OpenRAVEXMLParser::CreateTriMeshData(filedata->second, vscale, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
            ptrimesh.reset();
        }
        return ptrimesh;
    }

    virtual boost::shared_ptr<void> RegisterXMLReader(InterfaceType type, const std::string& xmltag, const CreateXMLReaderFn& fn)
    {
        return RaveRegisterXMLReader(type,xmltag,fn);
    }
    
    virtual bool ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
        return OpenRAVEXMLParser::ParseXMLFile(preader, filename);
    }

    virtual bool ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        OpenRAVEXMLParser::SetDataDirs(GetDataDirs());
        return OpenRAVEXMLParser::ParseXMLData(preader, pdata);
    }

    virtual bool AttachViewer(ViewerBasePtr pnewviewer)
    {
        if( _pCurrentViewer == pnewviewer ) {
            return true;
        }
        if( !!_pCurrentViewer ) {
            _pCurrentViewer->quitmainloop();
        }
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        
        _pCurrentViewer = pnewviewer;
        if( !_pCurrentViewer ) {
            _pCurrentViewer.reset(new DummyViewer(shared_from_this()));
        }
        if( _pCurrentViewer->GetEnv() != shared_from_this() ) {
            throw openrave_exception("Viewer needs to be created by the current environment");
        }
        return true;
    }

    virtual ViewerBasePtr GetViewer() const
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _pCurrentViewer;
    }

    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
    {
        return _pCurrentViewer->plot3(ppoints, numPoints, stride, fPointSize, color, drawstyle);
    }
    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
    {
        return _pCurrentViewer->plot3(ppoints, numPoints, stride, fPointSize, colors, drawstyle, bhasalpha);
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        return _pCurrentViewer->drawlinestrip(ppoints, numPoints, stride, fwidth,color);
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        return _pCurrentViewer->drawlinestrip(ppoints, numPoints, stride, fwidth,colors);
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        return _pCurrentViewer->drawlinelist(ppoints, numPoints, stride, fwidth,color);
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        return _pCurrentViewer->drawlinelist(ppoints, numPoints, stride, fwidth,colors);
    }
    virtual OpenRAVE::GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
    {
        return _pCurrentViewer->drawarrow(p1,p2,fwidth,color);
    }
    virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
    {
        return _pCurrentViewer->drawbox(vpos, vextents);
    }
    virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
    {
        return _pCurrentViewer->drawplane(tplane, vextents, vtexture);
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
    {
        return _pCurrentViewer->drawtrimesh(ppoints, stride, pIndices, numTriangles, color);
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
    {
        return _pCurrentViewer->drawtrimesh(ppoints, stride, pIndices, numTriangles, colors);
    }
    virtual KinBodyPtr GetBodyFromNetworkId(int id)
    {
        RAVELOG_INFO("GetBodyFromNetworkId is deprecated, use GetBodyFromEnvironmentId instead\n");
        return GetBodyFromEnvironmentId(id);
    }

    virtual KinBodyPtr GetBodyFromEnvironmentId(int id)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        boost::mutex::scoped_lock locknetwork(_mutexEnvironmentIds);
        map<int, KinBodyWeakPtr>::iterator it = _mapBodies.find(id);
        if( it != _mapBodies.end() ) {
            return KinBodyPtr(it->second);
        }
        return KinBodyPtr();
    }

    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _bEnableSimulation = true;
        _fDeltaSimTime = fDeltaTime;
        _bRealTime = bRealTime;
        _nCurSimTime = 0;
        _nSimStartTime = GetMicroTime();
    }

    virtual bool IsSimulationRunning() const { return _bEnableSimulation; }

    virtual void StopSimulation()
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _bEnableSimulation = false;
        _fDeltaSimTime = 1.0f;
    }
    virtual uint64_t GetSimulationTime() { return _nCurSimTime; }

    virtual void SetDebugLevel(DebugLevel level) { RaveSetDebugLevel(level); }
    virtual DebugLevel GetDebugLevel() const { return RaveGetDebugLevel(); }

    virtual void GetPublishedBodies(std::vector<KinBody::BodyState>& vbodies)
    {
        boost::mutex::scoped_lock lock(_mutexInterfaces);
        vbodies = _vPublishedBodies;
    }

    virtual void UpdatePublishedBodies()
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::mutex::scoped_lock lock(_mutexInterfaces);

        // updated the published bodies
        _vPublishedBodies.resize(_vecbodies.size());
            
        vector<KinBody::BodyState>::iterator itstate = _vPublishedBodies.begin();
        FOREACH(itbody, _vecbodies) {
            itstate->pbody = *itbody;
            (*itbody)->GetBodyTransformations(itstate->vectrans);
            (*itbody)->GetDOFValues(itstate->jointvalues);
            itstate->strname =(*itbody)->GetName();
            itstate->pguidata = (*itbody)->GetGuiData();
            itstate->environmentid = (*itbody)->GetEnvironmentId();
            ++itstate;
        }
    }

    const vector<string>& GetDataDirs() const { return _vdatadirs; }

protected:

    virtual void Clone(boost::shared_ptr<Environment const> r, int options)
    {
        Destroy();

        boost::mutex::scoped_lock(_mutexInit);
        AttachViewer(ViewerBasePtr());
        SetCollisionChecker(CollisionCheckerBasePtr());
        SetPhysicsEngine(PhysicsEngineBasePtr());

        _nBodiesModifiedStamp = r->_nBodiesModifiedStamp;
        _homedirectory = r->_homedirectory;
        _pCurrentViewer.reset(new DummyViewer(shared_from_this()));
        _fDeltaSimTime = r->_fDeltaSimTime;
        _nCurSimTime = 0;
        _nSimStartTime = GetMicroTime();
        _nEnvironmentIndex = r->_nEnvironmentIndex;
        _bRealTime = r->_bRealTime;

        _bInit = true;
        _bEnableSimulation = r->_bEnableSimulation;

        SetDebugLevel(r->GetDebugLevel());

        // a little tricky due to a deadlocking situation
        std::map<int, KinBodyWeakPtr> mapBodies;
        {
            boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
            mapBodies = _mapBodies;
            _mapBodies.clear();
        }
        mapBodies.clear();

        _vdatadirs = r->_vdatadirs;

        EnvironmentMutex::scoped_lock lock(GetMutex());
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);

        // clone collision and physics
        if( !!r->GetCollisionChecker() ) {
            SetCollisionChecker(CreateCollisionChecker(r->GetCollisionChecker()->GetXMLId()));
        }

        if( options & Clone_Bodies ) {
            boost::mutex::scoped_lock lock(r->_mutexInterfaces);
            FOREACHC(itrobot, r->_vecrobots) {
                try {
                    RobotBasePtr pnewrobot = RaveCreateRobot(shared_from_this(), (*itrobot)->GetXMLId());
                    pnewrobot->Clone(*itrobot, options);
                    pnewrobot->_environmentid = (*itrobot)->GetEnvironmentId();

                    // note that pointers will not be correct
                    pnewrobot->_vGrabbedBodies = (*itrobot)->_vGrabbedBodies;
                    pnewrobot->_listAttachedBodies = (*itrobot)->_listAttachedBodies;

                    BOOST_ASSERT( _mapBodies.find(pnewrobot->GetEnvironmentId()) == _mapBodies.end() );
                    _mapBodies[pnewrobot->GetEnvironmentId()] = pnewrobot;
                    _vecbodies.push_back(pnewrobot);
                    _vecrobots.push_back(pnewrobot);
                }
                catch(const openrave_exception& ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone robot %s: %s")%(*itrobot)->GetName()%ex.what()));
                }
            }
            FOREACHC(itbody, r->_vecbodies) {
                if( _mapBodies.find((*itbody)->GetEnvironmentId()) != _mapBodies.end() ) {
                    continue;
                }
                try {
                    KinBodyPtr pnewbody(new KinBody(PT_KinBody,shared_from_this()));
                    pnewbody->Clone(*itbody,options);
                    pnewbody->_environmentid = (*itbody)->GetEnvironmentId();
                    
                    // note that pointers will not be correct
                    pnewbody->_listAttachedBodies = (*itbody)->_listAttachedBodies;
                    
                    // note that pointers will not be correct
                    _mapBodies[pnewbody->GetEnvironmentId()] = pnewbody;
                    _vecbodies.push_back(pnewbody);
                }
                catch(const openrave_exception& ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone body %s: %s")%(*itbody)->GetName()%ex.what()));
                }
            }

            // process attached bodies
            FOREACH(itbody, _vecbodies) {
                list<KinBodyWeakPtr> listnew;
                FOREACH(itatt, (*itbody)->_listAttachedBodies) {
                    KinBodyPtr patt = itatt->lock();
                    if( !!patt ) {
                        BOOST_ASSERT( _mapBodies.find(patt->GetEnvironmentId()) != _mapBodies.end() );
                        listnew.push_back(_mapBodies[patt->GetEnvironmentId()]);
                    }
                }
                (*itbody)->_listAttachedBodies = listnew;
            }

            // process grabbed bodies
            FOREACH(itrobot, _vecrobots) {
                FOREACH(itgrab, (*itrobot)->_vGrabbedBodies) {
                    KinBodyPtr pbody(itgrab->pbody);
                    BOOST_ASSERT( !!pbody && _mapBodies.find(pbody->GetEnvironmentId()) != _mapBodies.end());
                    itgrab->pbody = _mapBodies[pbody->GetEnvironmentId()];
                    itgrab->plinkrobot = (*itrobot)->GetLinks().at(KinBody::LinkPtr(itgrab->plinkrobot)->GetIndex());

                    vector<KinBody::LinkConstPtr> vnew;
                    FOREACH(itlink, itgrab->vCollidingLinks) {
                        vnew.push_back((*itrobot)->_veclinks.at((*itlink)->GetIndex()));
                    }
                    itgrab->vCollidingLinks = vnew;
                    vnew.resize(0);
                    FOREACH(itlink, itgrab->vNonCollidingLinks) {
                        vnew.push_back((*itrobot)->_veclinks.at((*itlink)->GetIndex()));
                    }
                    itgrab->vNonCollidingLinks = vnew;
                }
            }
            FOREACH(itbody,_vecbodies) {
                (*itbody)->_ComputeInternalInformation();
            }
            FOREACH(itbody, _vecbodies) {
                GetCollisionChecker()->InitKinBody(*itbody);
                GetPhysicsEngine()->InitKinBody(*itbody);
            }
        }
        if( options & Clone_Sensors ) {
            boost::mutex::scoped_lock lock(r->_mutexInterfaces);
            FOREACHC(itsensor,r->_listSensors) {
                try {
                    SensorBasePtr pnewsensor = RaveCreateSensor(shared_from_this(), (*itsensor)->GetXMLId());
                    pnewsensor->Clone(*itsensor, options);
                    _listSensors.push_back(pnewsensor);
                }
                catch(const openrave_exception& ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone sensor %s: %s")%(*itsensor)->GetName()%ex.what()));
                }
            }
        }

        if( options & Clone_Viewer ) {
            if( !!r->GetViewer() ) {
                AttachViewer(CreateViewer(r->GetViewer()->GetXMLId()));
            }
        }
    
        if( options & Clone_Simulation ) {
            if( !!r->GetPhysicsEngine() ) {
                SetPhysicsEngine(CreatePhysicsEngine(r->GetPhysicsEngine()->GetXMLId()));
            }
            _bEnableSimulation = r->_bEnableSimulation;
            _nCurSimTime = r->_nCurSimTime;
            _nSimStartTime = r->_nSimStartTime;
        }

        if( !!_threadSimulation ) {
            _threadSimulation->join();
        }
        _threadSimulation.reset(new boost::thread(boost::bind(&Environment::_SimulationThread,this)));
    }

    virtual bool _CheckUniqueName(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        FOREACHC(itbody,_vecbodies) {
            if( *itbody != pbody && (*itbody)->GetName() == pbody->GetName() ) {
                if( bDoThrow ) {
                    throw openrave_exception(str(boost::format("body %s does not have unique name")%pbody->GetName()));
                }
                return false;
            }
        }
        return true;
    }
    virtual bool _CheckUniqueName(SensorBaseConstPtr psensor, bool bDoThrow=false) const
    {
        FOREACHC(itsensor,_listSensors) {
            if( *itsensor != psensor && (*itsensor)->GetName() == psensor->GetName() ) {
                if( bDoThrow ) {
                    throw openrave_exception(str(boost::format("sensor %s does not have unique name")%psensor->GetName()));
                }
                return false;
            }
        }
        return true;
    }

    class DummyPhysicsEngine : public PhysicsEngineBase
    {
        class PhysicsData : public UserData
        {
        public:
            PhysicsData(KinBodyPtr pbody) {
                linkvelocities.resize(pbody->GetLinks().size());
            }
            virtual ~PhysicsData() {}
            std::vector< std::pair<Vector, Vector> > linkvelocities;
        };

        boost::shared_ptr<PhysicsData> _GetData(KinBodyConstPtr pbody) { return boost::dynamic_pointer_cast<PhysicsData>(pbody->GetPhysicsData()); }

    public:
        DummyPhysicsEngine(EnvironmentBasePtr penv) : PhysicsEngineBase(penv) {}
        virtual bool SetPhysicsOptions(int physicsoptions) { return true; }
        virtual int GetPhysicsOptions() const { return 0; }
        
        virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) { return true; }

        virtual bool InitEnvironment() {
            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            FOREACH(itbody, vbodies) {
                InitKinBody(*itbody);
            }
            return true;
        }
        virtual void DestroyEnvironment()
        {
            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            FOREACH(itbody, vbodies) {
                DestroyKinBody(*itbody);
            }
        }
        
        virtual bool InitKinBody(KinBodyPtr pbody) { SetPhysicsData(pbody, UserDataPtr(new PhysicsData(pbody))); return true; } 
        virtual bool DestroyKinBody(KinBodyPtr pbody) { SetPhysicsData(pbody, UserDataPtr()); return true; }

        virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel) {
            std::pair<Vector, Vector> vel = _GetData(plink->GetParent())->linkvelocities.at(plink->GetIndex());
            linearvel = vel.first;
            angularvel = vel.second;
            return true;
        }
        bool GetLinkVelocities(KinBodyConstPtr body, std::vector<std::pair<Vector,Vector> >& velocities) {
            velocities = _GetData(body)->linkvelocities;
            return true;
        }

        virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
        {
            _GetData(plink->GetParent())->linkvelocities.at(plink->GetIndex()) = make_pair(linearvel,angularvel);
            return true;
        }
        bool SetLinkVelocities(KinBodyPtr body, const std::vector<std::pair<Vector,Vector> >& velocities)
        {
            _GetData(body)->linkvelocities = velocities;
            return true;
        }

        virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd) { return true; }
        virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd) { return true; }
        virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques) { return true; }

        virtual void SetGravity(const Vector& gravity) {}
        virtual Vector GetGravity() { return Vector(0,0,0); }

        virtual bool GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque) {
            force = Vector(0,0,0);
            torque = Vector(0,0,0);
            return true;
        }

        virtual void SimulateStep(dReal fTimeElapsed) { }
    };

    class DummyCollisionChecker : public CollisionCheckerBase
    {
    public:
        DummyCollisionChecker(EnvironmentBasePtr penv) : CollisionCheckerBase(penv) {}
        virtual ~DummyCollisionChecker() {}

        virtual bool InitEnvironment() { return true; }
        virtual void DestroyEnvironment() {}

        virtual bool InitKinBody(KinBodyPtr pbody) { SetCollisionData(pbody, UserDataPtr()); return true; }
        virtual bool DestroyKinBody(KinBodyPtr pbody) { SetCollisionData(pbody, UserDataPtr()); return true; }
        virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) { return true; }
        virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) { return true; }

        virtual bool SetCollisionOptions(int collisionoptions) { return true; }
        virtual int GetCollisionOptions() const { return 0; }

        virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) { return true; }

        virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr) { return false; }
    
        virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) { return false; }
    
        virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr) { return false; }
        virtual bool CheckCollision(const RAY& ray, CollisionReportPtr) { return false; }
        virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr) { return false; }

        virtual void SetTolerance(dReal tolerance) {}
    };

    /// Base class for the graphics and gui engine. Derive a class called
    /// Viewer for anything more specific
    class DummyViewer : public ViewerBase
    {
    public:
        DummyViewer(EnvironmentBasePtr penv) : ViewerBase(penv) {
            _bQuitMainLoop = true;
        }
        virtual ~DummyViewer() {}

        // dummy loop
        virtual int main(bool bShow) {
            boost::mutex::scoped_lock lock(_mutex);
            if(!_bQuitMainLoop) {
                return -1;
            }
            _bQuitMainLoop = false;
            _cond.wait(lock);
            return 0;
        }
        
        virtual void quitmainloop() {
            boost::mutex::scoped_lock lock(_mutex);
            _bQuitMainLoop = true;
            _cond.notify_all();
        }

        virtual void Reset() {}
        virtual void SetBkgndColor(const RaveVector<float>& color) {}
        
        virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) { return OpenRAVE::GraphHandlePtr(); }
        virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors) { return OpenRAVE::GraphHandlePtr(); }

        virtual void SetCamera(const RaveTransform<float>& trans, float focalDistance=0) {}
        virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK) { return false; }
    protected:
        boost::mutex _mutex;
        boost::condition _cond;
        bool _bQuitMainLoop;
    };

    virtual void SetEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
        int id = _nEnvironmentIndex++;
        BOOST_ASSERT( _mapBodies.find(id) == _mapBodies.end() );
        pbody->_environmentid=id;
        _mapBodies[id] = pbody;
    }

    virtual void RemoveEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
        _mapBodies.erase(pbody->_environmentid);
        pbody->_environmentid = 0;
    }

    void _SimulationThread()
    {
        uint64_t nLastUpdateTime = GetMicroTime();
        uint64_t nLastSleptTime = GetMicroTime();
        while( _bInit ) {
            bool bNeedSleep = true;

            if( _bEnableSimulation ) {
                bNeedSleep = false;
                EnvironmentMutex::scoped_lock lockenv(GetMutex());
                int64_t deltasimtime = (int64_t)(_fDeltaSimTime*1000000.0f);
                try {
                    StepSimulation(_fDeltaSimTime);
                }
                catch(const openrave_exception& ex) {
                    RAVELOG_ERROR("simulation thread exception: %s\n",ex.what());
                }
                uint64_t passedtime = GetMicroTime()-_nSimStartTime;
                int64_t sleeptime = _nCurSimTime-passedtime;
                if( _bRealTime ) {
                    if( sleeptime > 2*deltasimtime && sleeptime > 2000 ) {
                        lockenv.unlock();
                        // sleep for less time since sleep isn't accurate at all and we have a 7ms buffer
                        Sleep( max((int)(deltasimtime + (sleeptime-2*deltasimtime)/2)/1000,1) );
                        //RAVELOG_INFO("sleeping %d(%d), slept: %d\n",(int)(_nCurSimTime-passedtime),(int)((sleeptime-(deltasimtime/2))/1000));
                        nLastSleptTime = GetMicroTime();
                    }
                    else if( sleeptime < -3*deltasimtime ) {
                        // simulation is getting late, so catch up
                        //RAVELOG_INFO("sim catching up: %d\n",-(int)sleeptime);
                        _nSimStartTime += -sleeptime;//deltasimtime;
                    }
                }
                else {
                    nLastSleptTime = GetMicroTime();
                }
                
                //RAVELOG_INFOA("sim: %f, real: %f\n",_nCurSimTime*1e-6f,(GetMicroTime()-_nSimStartTime)*1e-6f);
            }

            if( GetMicroTime()-nLastSleptTime > 20000 ) { // 100000 freezes the environment
                Sleep(1); bNeedSleep = false;
                nLastSleptTime = GetMicroTime();
            }

            if( GetMicroTime()-nLastUpdateTime > 10000 ) {
                EnvironmentMutex::scoped_lock lockenv(GetMutex());
                nLastUpdateTime = GetMicroTime();
                UpdatePublishedBodies();
            }

            if( bNeedSleep ) {
                Sleep(1);
            }
        }
    }

    static bool _IsColladaFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 4 ) {
            return false;
        }
        if( filename[len-4] == '.' && ::tolower(filename[len-3]) == 'd' && ::tolower(filename[len-2]) == 'a' && ::tolower(filename[len-1]) == 'e' ) {
            return true;
        }
        if( filename[len-4] == '.' && ::tolower(filename[len-3]) == 'z' && ::tolower(filename[len-2]) == 'a' && ::tolower(filename[len-1]) == 'e' ) {
            return true;
        }
        return false;
    }
    static bool _IsColladaData(const std::string& data)
    {
        return data.find("<COLLADA") != std::string::npos;
    }

    std::vector<RobotBasePtr> _vecrobots;  ///< robots (possibly controlled)
    std::vector<KinBodyPtr> _vecbodies; ///< all objects that are collidable (includes robots)

    list<ProblemInstancePtr> _listProblems; ///< problems loaded in the environment
    list<SensorBasePtr> _listSensors; ///< sensors loaded in the environment

    dReal _fDeltaSimTime;                ///< delta time for simulate step
    uint64_t _nCurSimTime;                    ///< simulation time since the start of the environment
    uint64_t _nSimStartTime;
    int _nBodiesModifiedStamp; ///< incremented every tiem bodies vector is modified
    bool _bRealTime;

    CollisionCheckerBasePtr _pCurrentChecker;
    PhysicsEngineBasePtr _pPhysicsEngine;
    ViewerBasePtr _pCurrentViewer;
    
    int _nEnvironmentIndex;               ///< next network index
    std::map<int, KinBodyWeakPtr> _mapBodies; ///< a map of all the bodies in the environment. Controlled through the KinBody constructor and destructors
    
    boost::shared_ptr<boost::thread> _threadSimulation;                  ///< main loop for environment simulation

    mutable EnvironmentMutex _mutexEnvironment;      ///< protects internal data from multithreading issues
    mutable boost::mutex _mutexEnvironmentIds;  ///< protects _vecbodies/_vecrobots from multithreading issues
    mutable boost::mutex _mutexInterfaces; ///< lock when managing interfaces like _listOwnedInterfaces, _listProblems, _mapBodies
    mutable boost::mutex _mutexInit; ///< lock for destroying the environment

    vector<KinBody::BodyState> _vPublishedBodies;
    vector<string> _vdatadirs;
    string _homedirectory;

    list<InterfaceBasePtr> _listOwnedInterfaces;

    std::list<CollisionCallbackFn> _listRegisteredCollisionCallbacks; ///< see EnvironmentBase::RegisterCollisionCallback

    bool _bInit;               ///< environment is initialized
    bool _bEnableSimulation;        ///< enable simulation loop

    friend class EnvironmentXMLReader;
};

#endif
