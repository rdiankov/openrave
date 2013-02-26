// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

#include "colladaparser/colladacommon.h"

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#define CHECK_INTERFACE(pinterface) { \
        if( (pinterface)->GetEnv() != shared_from_this() ) \
            throw openrave_exception(str(boost::format("Interface %s:%s is from a different environment")%RaveGetInterfaceName((pinterface)->GetInterfaceType())%(pinterface)->GetXMLId()),ORE_InvalidArguments); \
} \

#define CHECK_COLLISION_BODY(body) { \
        CHECK_INTERFACE(body); \
}

class Environment : public EnvironmentBase
{
    class GraphHandleMulti : public GraphHandle
    {
public:
        GraphHandleMulti() {
        }
        virtual ~GraphHandleMulti() {
        }
        void SetTransform(const RaveTransform<float>& t)
        {
            FOREACH(it,listhandles) {
                (*it)->SetTransform(t);
            }
        }

        void SetShow(bool bshow)
        {
            FOREACH(it,listhandles) {
                (*it)->SetShow(bshow);
            }
        }

        void Add(OpenRAVE::GraphHandlePtr phandle) {
            if( !!phandle) {
                listhandles.push_back(phandle);
            }
        }

        list<OpenRAVE::GraphHandlePtr> listhandles;
    };
    typedef boost::shared_ptr<GraphHandleMulti> GraphHandleMultiPtr;

    class CollisionCallbackData : public UserData
    {
public:
        CollisionCallbackData(const CollisionCallbackFn& callback, boost::shared_ptr<Environment> penv) : _callback(callback), _pweakenv(penv) {
        }
        virtual ~CollisionCallbackData() {
            boost::shared_ptr<Environment> penv = _pweakenv.lock();
            if( !!penv ) {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex());
                penv->_listRegisteredCollisionCallbacks.erase(_iterator);
            }
        }

        list<UserDataWeakPtr>::iterator _iterator;
        CollisionCallbackFn _callback;
protected:
        boost::weak_ptr<Environment> _pweakenv;
    };
    friend class CollisionCallbackData;
    typedef boost::shared_ptr<CollisionCallbackData> CollisionCallbackDataPtr;

public:
    Environment() : EnvironmentBase()
    {
        _homedirectory = RaveGetHomeDirectory();
        RAVELOG_DEBUG(str(boost::format("setting openrave home directory to %s")%_homedirectory));

        _nBodiesModifiedStamp = 0;
        _nEnvironmentIndex = 1;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _bRealTime = true;
        _bInit = false;
        _bEnableSimulation = true;     // need to start by default

        _handlegenericrobot = RaveRegisterInterface(PT_Robot,"GenericRobot", RaveGetInterfaceHash(PT_Robot), GetHash(), CreateGenericRobot);
        _handlegenerictrajectory = RaveRegisterInterface(PT_Trajectory,"GenericTrajectory", RaveGetInterfaceHash(PT_Trajectory), GetHash(), CreateGenericTrajectory);
        _handlegenericphysicsengine = RaveRegisterInterface(PT_PhysicsEngine,"GenericPhysicsEngine", RaveGetInterfaceHash(PT_PhysicsEngine), GetHash(), CreateGenericPhysicsEngine);
        _handlegenericcollisionchecker = RaveRegisterInterface(PT_CollisionChecker,"GenericCollisionChecker", RaveGetInterfaceHash(PT_CollisionChecker), GetHash(), CreateGenericCollisionChecker);
    }

    virtual ~Environment()
    {
        Destroy();
    }

    virtual void Init()
    {
        boost::mutex::scoped_lock lockinit(_mutexInit);
        if( _bInit ) {
            RAVELOG_WARN("environment is already initialized, ignoring\n");
            return;
        }

        _nBodiesModifiedStamp = 0;
        _nEnvironmentIndex = 1;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _bRealTime = true;
        _bEnableSimulation = true;     // need to start by default

        if( !_pCurrentChecker ) {
            _pCurrentChecker = RaveCreateCollisionChecker(shared_from_this(), "GenericCollisionChecker");
        }
        if( !_pPhysicsEngine ) {
            _pPhysicsEngine = RaveCreatePhysicsEngine(shared_from_this(), "GenericPhysicsEngine");
            _SetDefaultGravity();
        }

        // try to set init as early as possible since will be calling into user code
        _bInit = true;

        // set a collision checker, don't call EnvironmentBase::CreateCollisionChecker
        CollisionCheckerBasePtr localchecker;
        boost::array<string,3> checker_prefs = { { "ode", "bullet", "pqp"}};     // ode takes priority since bullet has some bugs with deleting bodies
        FOREACH(itchecker,checker_prefs) {
            localchecker = RaveCreateCollisionChecker(shared_from_this(), *itchecker);
            if( !!localchecker ) {
                break;
            }
        }

        if( !localchecker ) {     // take any collision checker
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
            _threadSimulation->join();     // might not return?
        }
        _threadSimulation.reset();

        // destroy the modules (their destructors could attempt to lock environment, so have to do it before global lock)
        // however, do not clear the _listModules yet
        RAVELOG_DEBUG("destroy module\n");
        list<ModuleBasePtr> listModules;
        list<ViewerBasePtr> listViewers = _listViewers;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules = _listModules;
            listViewers = _listViewers;
        }
        FOREACH(itmodule,listModules) {
            (*itmodule)->Destroy();
        }
        listModules.clear();

        FOREACH(itviewer, listViewers) {
            // don't reset the viewer since it can already be dead
            // todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
            RAVELOG_DEBUG(str(boost::format("quitting viewer %s")%(*itviewer)->GetXMLId()));
            (*itviewer)->quitmainloop();
        }
        listViewers.clear();

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
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
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
                _listModules.clear();
                _listViewers.clear();
                _listOwnedInterfaces.clear();
            }
        }

        // release all other interfaces, not necessary to hold a mutex?
        _pCurrentChecker.reset();
        _pPhysicsEngine.reset();
        RAVELOG_VERBOSE("Environment destroyed\n");
    }

    virtual void Reset()
    {
        // destruction order is *very* important, don't touch it without consultation
        RAVELOG_DEBUG("resetting raveviewer\n");
        list<ViewerBasePtr> listViewers;
        GetViewers(listViewers);
        FOREACH(itviewer, listViewers) {
            (*itviewer)->Reset();
        }

        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment();
        }
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
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

        list<ModuleBasePtr> listModules;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules = _listModules;
        }

        FOREACH(itmodule,listModules) {
            (*itmodule)->Reset();
        }
        listModules.clear();
        _listOwnedInterfaces.clear();

        if( !!_pCurrentChecker ) {
            _pCurrentChecker->InitEnvironment();
        }
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->InitEnvironment();
        }
    }

    virtual UserDataPtr GlobalState() {
        return RaveGlobalState();
    }

    virtual void OwnInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        _listOwnedInterfaces.push_back(pinterface);
    }
    virtual void DisownInterface(InterfaceBasePtr pinterface)
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        _listOwnedInterfaces.remove(pinterface);
    }

    virtual EnvironmentBasePtr CloneSelf(int options)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::shared_ptr<Environment> penv(new Environment());
        penv->_Clone(boost::static_pointer_cast<Environment const>(shared_from_this()),options,false);
        return penv;
    }

    virtual void Clone(EnvironmentBaseConstPtr preference, int cloningoptions)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _Clone(boost::static_pointer_cast<Environment const>(preference),cloningoptions,true);
    }

    virtual int AddModule(ModuleBasePtr module, const std::string& cmdargs)
    {
        CHECK_INTERFACE(module);
        int ret = module->main(cmdargs);
        if( ret != 0 ) {
            RAVELOG_WARN(str(boost::format("Error %d with executing module %s\n")%ret%module->GetXMLId()));
        }
        else {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listModules.push_back(module);
        }

        return ret;
    }

    void GetModules(std::list<ModuleBasePtr>& listModules, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules = _listModules;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            listModules = _listModules;
        }
    }

    virtual bool LoadURI(const std::string& uri, const AttributesList& atts)
    {
        return RaveParseColladaURI(shared_from_this(), uri, atts);
    }

    virtual bool Load(const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        OpenRAVEXMLParser::GetXMLErrorCount() = 0;
        if( _IsColladaFile(filename) ) {
            if( RaveParseColladaFile(shared_from_this(), filename, atts) ) {
                return true;
            }
        }
        if( _IsXFile(filename) ) {
            RobotBasePtr robot;
            if( RaveParseXFile(shared_from_this(), robot, filename, atts) ) {
                _AddRobot(robot, true);
                return true;
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) {
            KinBodyPtr pbody = ReadKinBodyURI(KinBodyPtr(),filename,atts);
            if( !!pbody ) {
                _AddKinBody(pbody,true);
                return true;
            }
        }
        else {
            if( _ParseXMLFile(OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),atts,true), filename) ) {
                if( OpenRAVEXMLParser::GetXMLErrorCount() == 0 ) {
                    return true;
                }
            }
        }

        RAVELOG_WARN("load failed on file %s\n", filename.c_str());
        return false;
    }

    virtual bool LoadData(const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( _IsColladaData(data) ) {
            return RaveParseColladaData(shared_from_this(), data, atts);
        }
        return _ParseXMLData(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(),atts),data);
    }

    virtual void Save(const std::string& filename, SelectionOptions options, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            RaveWriteColladaFile(shared_from_this(),filename,atts);
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN(str(boost::format("failed to get body %s")%itatt->second));
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            FOREACH(itbody,_vecbodies) {
                if( !(*itbody)->IsRobot() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        case SO_Robots:
            FOREACH(itrobot,_vecrobots) {
                listbodies.push_back(*itrobot);
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            FOREACH(itbody,_vecbodies) {
                if( find(listignore.begin(),listignore.end(),(*itbody)->GetName()) == listignore.end() ) {
                    listbodies.push_back(*itbody);
                }
            }
            break;
        }
        }

        if( listbodies.size() == 1 ) {
            RaveWriteColladaFile(listbodies.front(),filename,atts);
        }
        else {
            RaveWriteColladaFile(listbodies,filename,atts);
        }
    }

    virtual void Add(InterfaceBasePtr pinterface, bool bAnonymous, const std::string& cmdargs)
    {
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_Robot: _AddRobot(RaveInterfaceCast<RobotBase>(pinterface),bAnonymous); break;
        case PT_KinBody: _AddKinBody(RaveInterfaceCast<KinBody>(pinterface),bAnonymous); break;
        case PT_Module: {
            int ret = AddModule(RaveInterfaceCast<ModuleBase>(pinterface),cmdargs);
            OPENRAVE_ASSERT_OP_FORMAT(ret,==,0,"module %s failed with args: %s",pinterface->GetXMLId()%cmdargs,ORE_InvalidArguments);
            break;
        }
        case PT_Viewer: _AddViewer(RaveInterfaceCast<ViewerBase>(pinterface)); break;
        case PT_Sensor: _AddSensor(RaveInterfaceCast<SensorBase>(pinterface),bAnonymous); break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT("Interface %d cannot be added to the environment",pinterface->GetInterfaceType(),ORE_InvalidArguments);
        }
    }

    virtual void _AddKinBody(KinBodyPtr pbody, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pbody);
        if( !utils::IsValidName(pbody->GetName()) ) {
            throw openrave_exception(str(boost::format("kinbody name: \"%s\" is not valid")%pbody->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(pbody),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=pbody->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                pbody->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(pbody), false) ) {
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _vecbodies.push_back(pbody);
            SetEnvironmentId(pbody);
            _nBodiesModifiedStamp++;
        }
        pbody->_ComputeInternalInformation();
        _pCurrentChecker->InitKinBody(pbody);
        _pPhysicsEngine->InitKinBody(pbody);
        // send all the changed callbacks of the body since anything could have changed
        pbody->_ParametersChanged(0xffffffff&~KinBody::Prop_JointMimic&~KinBody::Prop_LinkStatic);
    }

    virtual void _AddRobot(RobotBasePtr robot, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(robot);
        if( !robot->IsRobot() ) {
            throw openrave_exception(str(boost::format("kinbody \"%s\" is not a robot")%robot->GetName()));
        }
        if( !utils::IsValidName(robot->GetName()) ) {
            throw openrave_exception(str(boost::format("kinbody name: \"%s\" is not valid")%robot->GetName()));
        }
        if( !_CheckUniqueName(KinBodyConstPtr(robot),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=robot->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                robot->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(robot),false) ) {
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _vecbodies.push_back(robot);
            _vecrobots.push_back(robot);
            SetEnvironmentId(robot);
            _nBodiesModifiedStamp++;
        }
        robot->_ComputeInternalInformation();
        _pCurrentChecker->InitKinBody(robot);
        _pPhysicsEngine->InitKinBody(robot);
        // send all the changed callbacks of the body since anything could have changed
        robot->_ParametersChanged(0xffffffff&~KinBody::Prop_JointMimic&~KinBody::Prop_LinkStatic);
    }

    virtual void _AddSensor(SensorBasePtr psensor, bool bAnonymous)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(psensor);
        if( !utils::IsValidName(psensor->GetName()) ) {
            throw openrave_exception(str(boost::format("sensor name: \"%s\" is not valid")%psensor->GetName()));
        }
        if( !_CheckUniqueName(SensorBaseConstPtr(psensor),!bAnonymous) ) {
            // continue to add random numbers until a unique name is found
            string oldname=psensor->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                psensor->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(SensorBaseConstPtr(psensor),false) ) {
                    break;
                }
            }
        }
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listSensors.push_back(psensor);
        }
        psensor->Configure(SensorBase::CC_PowerOn);
    }

    virtual bool Remove(InterfaceBasePtr pinterface)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_KinBody:
        case PT_Robot: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
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
            if( !!_pCurrentChecker ) {
                _pCurrentChecker->RemoveKinBody(*it);
            }
            if( !!_pPhysicsEngine ) {
                _pPhysicsEngine->RemoveKinBody(*it);
            }
            RemoveEnvironmentId(pbody);
            _vecbodies.erase(it);
            _nBodiesModifiedStamp++;
            return true;
        }
        case PT_Sensor: {
            SensorBasePtr psensor = RaveInterfaceCast<SensorBase>(pinterface);
            list<SensorBasePtr>::iterator it = std::find(_listSensors.begin(), _listSensors.end(), psensor);
            if( it != _listSensors.end() ) {
                (*it)->Configure(SensorBase::CC_PowerOff);
                _listSensors.erase(it);
                return true;
            }
            break;
        }
        case PT_Module: {
            ModuleBasePtr prob = RaveInterfaceCast<ModuleBase>(pinterface);
            list<ModuleBasePtr>::iterator itmodule = find(_listModules.begin(), _listModules.end(), prob);
            if( itmodule != _listModules.end() ) {
                (*itmodule)->Destroy();
                _listModules.erase(itmodule);
                return true;
            }
            break;
        }
        case PT_Viewer: {
            ViewerBasePtr pviewer = RaveInterfaceCast<ViewerBase>(pinterface);
            list<ViewerBasePtr>::iterator itviewer = find(_listViewers.begin(), _listViewers.end(), pviewer);
            if( itviewer != _listViewers.end() ) {
                (*itviewer)->quitmainloop();
                _listViewers.erase(itviewer);
                return true;
            }
            break;
        }
        default:
            RAVELOG_WARN(str(boost::format("unmanaged interfaces of type %s cannot be removed\n")%RaveGetInterfaceName(pinterface->GetInterfaceType())));
            break;
        }
        return false;
    }

    virtual KinBodyPtr GetKinBody(const std::string& pname) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecbodies) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        //RAVELOG_VERBOSE(str(boost::format("Environment::GetKinBody - Error: Unknown body %s\n")%pname));
        return KinBodyPtr();
    }

    virtual RobotBasePtr GetRobot(const std::string& pname) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecrobots) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        //RAVELOG_VERBOSE(str(boost::format("Environment::GetRobot - Error: Unknown body %s\n")%pname));
        return RobotBasePtr();
    }

    virtual SensorBasePtr GetSensor(const std::string& name) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(itrobot,_vecrobots) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor &&( psensor->GetName() == name) ) {
                    return psensor;
                }
            }
        }
        FOREACHC(itsensor,_listSensors) {
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
            _pPhysicsEngine = RaveCreatePhysicsEngine(shared_from_this(),"GenericPhysicsEngine");
            _SetDefaultGravity();
        }
        else {
            RAVELOG_DEBUG(str(boost::format("setting %s physics engine\n")%_pPhysicsEngine->GetXMLId()));
        }
        _pPhysicsEngine->InitEnvironment();
        return true;
    }

    virtual PhysicsEngineBasePtr GetPhysicsEngine() const {
        return _pPhysicsEngine;
    }

    virtual UserDataPtr RegisterCollisionCallback(const CollisionCallbackFn& callback) {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        CollisionCallbackDataPtr pdata(new CollisionCallbackData(callback,boost::dynamic_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredCollisionCallbacks.insert(_listRegisteredCollisionCallbacks.end(),pdata);
        return pdata;
    }
    virtual bool HasRegisteredCollisionCallbacks() const
    {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        return _listRegisteredCollisionCallbacks.size() > 0;
    }

    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>& listcallbacks) const {
        EnvironmentMutex::scoped_lock lock(GetMutex());
        listcallbacks.clear();
        FOREACHC(it, _listRegisteredCollisionCallbacks) {
            CollisionCallbackDataPtr pdata = boost::dynamic_pointer_cast<CollisionCallbackData>(it->lock());
            listcallbacks.push_back(pdata->_callback);
        }
    }

    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( _pCurrentChecker == pchecker ) {
            return true;
        }
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment();     // delete all resources
        }
        _pCurrentChecker = pchecker;
        if( !_pCurrentChecker ) {
            RAVELOG_DEBUG("disabling collisions\n");
            _pCurrentChecker = RaveCreateCollisionChecker(shared_from_this(),"GenericCollisionChecker");
        }
        else {
            RAVELOG_DEBUG(str(boost::format("setting %s collision checker\n")%_pCurrentChecker->GetXMLId()));
            FOREACH(itbody,_vecbodies) {
                (*itbody)->_ResetInternalCollisionCache();
            }
        }
        return _pCurrentChecker->InitEnvironment();
    }

    virtual CollisionCheckerBasePtr GetCollisionChecker() const {
        return _pCurrentChecker;
    }

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
        list<ModuleBasePtr> listModules;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            vecbodies = _vecbodies;
            vecrobots = _vecrobots;
            listSensors = _listSensors;
            listModules = _listModules;
        }

        FOREACH(it, vecbodies) {
            if( (*it)->GetEnvironmentId() ) {     // have to check if valid
                (*it)->SimulationStep(fTimeStep);
            }
        }
        FOREACH(itmodule, listModules) {
            (*itmodule)->SimulationStep(fTimeStep);
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

    virtual EnvironmentMutex& GetMutex() const {
        return _mutexEnvironment;
    }

    virtual void GetBodies(std::vector<KinBodyPtr>& bodies, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            bodies = _vecbodies;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            bodies = _vecbodies;
        }
    }

    virtual void GetRobots(std::vector<RobotBasePtr>& robots, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            robots = _vecrobots;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            robots = _vecrobots;
        }
    }

    virtual void GetSensors(std::vector<SensorBasePtr>& vsensors, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _GetSensors(vsensors);
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            _GetSensors(vsensors);
        }
    }

    virtual void _GetSensors(std::vector<SensorBasePtr>& vsensors) const
    {
        vsensors.resize(0);
        FOREACHC(itrobot,_vecrobots) {
            FOREACHC(itsensor, (*itrobot)->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor ) {
                    vsensors.push_back(psensor);
                }
            }
        }
    }

    virtual void Triangulate(TriMesh& trimesh, KinBodyConstPtr pbody)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());     // reading collision data, so don't want anyone modifying it
        FOREACHC(it, pbody->GetLinks()) {
            trimesh.Append((*it)->GetCollisionData(), (*it)->GetTransform());
        }
    }

    virtual void TriangulateScene(TriMesh& trimesh, SelectionOptions options,const std::string& selectname)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        FOREACH(itbody, _vecbodies) {
            RobotBasePtr robot;
            if( (*itbody)->IsRobot() ) {
                robot = RaveInterfaceCast<RobotBase>(*itbody);
            }
            switch(options) {
            case SO_NoRobots:
                if( !robot ) {
                    Triangulate(trimesh, *itbody);
                }
                break;

            case SO_Robots:
                if( !!robot ) {
                    Triangulate(trimesh, *itbody);
                }
                break;
            case SO_Everything:
                Triangulate(trimesh, *itbody);
                break;
            case SO_Body:
                if( (*itbody)->GetName() == selectname ) {
                    Triangulate(trimesh, *itbody);
                }
                break;
            case SO_AllExceptBody:
                if( (*itbody)->GetName() != selectname ) {
                    Triangulate(trimesh, *itbody);
                }
                break;
//            case SO_BodyList:
//                if( find(listnames.begin(),listnames.end(),(*itbody)->GetName()) != listnames.end() ) {
//                    Triangulate(trimesh,*itbody);
//                }
            }
        }
    }

    virtual void TriangulateScene(TriMesh& trimesh, TriangulateOptions options)
    {
        TriangulateScene(trimesh,options,"");
    }

    virtual RobotBasePtr ReadRobotURI(RobotBasePtr robot, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            FOREACH(itviewer, _listViewers) {
                (*itviewer)->RemoveKinBody(robot);
            }
            if( std::find(_vecrobots.begin(),_vecrobots.end(),robot) != _vecrobots.end() ) {
                throw openrave_exception(str(boost::format("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n")%robot->GetName()));
            }
        }

        if( _IsColladaFile(filename) ) {
            if( !RaveParseColladaFile(shared_from_this(), robot, filename, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsXFile(filename) ) {
            if( !RaveParseXFile(shared_from_this(), robot, filename, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) {
            if( !robot ) {
                robot = RaveCreateRobot(shared_from_this(),"GenericRobot");
            }
            if( !robot ) {
                robot = RaveCreateRobot(shared_from_this(),"");
            }
            if( !!robot ) {
                std::list<KinBody::GeometryInfo> listGeometries;
                std::string fullfilename = _ReadGeometriesFile(listGeometries,filename,atts);
                if( fullfilename.size() > 0 ) {
                    string extension;
                    if( filename.find_last_of('.') != string::npos ) {
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
                    string norender = string("__norenderif__:")+extension;
                    FOREACH(itinfo,listGeometries) {
                        itinfo->_bVisible = true;
                        itinfo->_filenamerender = norender;
                    }
                    listGeometries.front()._filenamerender = fullfilename;
                    if( robot->InitFromGeometries(listGeometries) ) {
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
                        boost::filesystem::path pfilename(filename);
                        robot->SetName(utils::ConvertToOpenRAVEName(pfilename.stem().string()));
#else
                        boost::filesystem::path pfilename(filename, boost::filesystem::native);
                        robot->SetName(utils::ConvertToOpenRAVEName(pfilename.stem()));
#endif
#else
                        robot->SetName("object");
#endif
                        return robot;
                    }
                }
                robot.reset();
            }
        }
        else {
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_Robot, pinterface, "robot", atts);
            if( !preader ) {
                return RobotBasePtr();
            }
            bool bSuccess = _ParseXMLFile(preader, filename);
            preader->endElement("robot");     // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !bSuccess || !robot ) {
                return RobotBasePtr();
            }
            robot->__struri = filename;
        }

        return robot;
    }

    virtual RobotBasePtr ReadRobotData(RobotBasePtr robot, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!robot ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            FOREACH(itviewer, _listViewers) {
                (*itviewer)->RemoveKinBody(robot);
            }
            if( std::find(_vecrobots.begin(),_vecrobots.end(),robot) != _vecrobots.end() ) {
                throw openrave_exception(str(boost::format("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n")%robot->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsXData(data) ) {
            // have to copy since it takes vector<char>
            std::vector<char> newdata(data.size()+1, 0);  // need a null-terminator
            std::copy(data.begin(),data.end(),newdata.begin());
            if( !RaveParseXData(shared_from_this(), robot, newdata, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsIVData(data) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("iv data not supported",ORE_InvalidArguments);
        }
        else {
            InterfaceBasePtr pinterface = robot;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_Robot, pinterface, "robot", atts);
            if( !preader ) {
                return RobotBasePtr();
            }
            bool bSuccess = _ParseXMLData(preader, data);
            preader->endElement("robot");     // have to end the tag!
            robot = RaveInterfaceCast<RobotBase>(pinterface);
            if( !bSuccess || !robot ) {
                return RobotBasePtr();
            }
            robot->__struri = preader->_filename;
        }
        return robot;
    }

    virtual KinBodyPtr ReadKinBodyURI(KinBodyPtr body, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            FOREACH(itviewer, _listViewers) {
                (*itviewer)->RemoveKinBody(body);
            }
            if( std::find(_vecbodies.begin(),_vecbodies.end(),body) != _vecbodies.end() ) {
                throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%body->GetName()));
            }
        }

        if( _IsColladaFile(filename) ) {
            if( !RaveParseColladaFile(shared_from_this(), body, filename, atts) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsXFile(filename) ) {
            if( !RaveParseXFile(shared_from_this(), body, filename, atts) ) {
                return KinBodyPtr();
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) {
            if( !body ) {
                body = RaveCreateKinBody(shared_from_this(),"");
            }
            if( !!body ) {
                std::list<KinBody::GeometryInfo> listGeometries;
                std::string fullfilename = _ReadGeometriesFile(listGeometries,filename,atts);
                if( fullfilename.size() > 0 ) {
                    string extension;
                    if( filename.find_last_of('.') != string::npos ) {
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
                    string norender = string("__norenderif__:")+extension;
                    FOREACH(itinfo,listGeometries) {
                        itinfo->_bVisible = true;
                        itinfo->_filenamerender = norender;
                    }
                    listGeometries.front()._filenamerender = fullfilename;
                    if( body->InitFromGeometries(listGeometries) ) {
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
                        boost::filesystem::path pfilename(filename);
                        body->SetName(utils::ConvertToOpenRAVEName(pfilename.stem().string()));
#else
                        boost::filesystem::path pfilename(filename, boost::filesystem::native);
                        body->SetName(utils::ConvertToOpenRAVEName(pfilename.stem()));
#endif
#else
                        body->SetName("object");
#endif
                        return body;
                    }
                }
                body.reset();
            }
        }
        else {
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) {
                return KinBodyPtr();
            }
            bool bSuccess = _ParseXMLFile(preader, filename);
            preader->endElement("kinbody");     // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !bSuccess || !body ) {
                return KinBodyPtr();
            }
            body->__struri = filename;
        }

        return body;
    }

    virtual KinBodyPtr ReadKinBodyData(KinBodyPtr body, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!body ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            FOREACH(itviewer, _listViewers) {
                (*itviewer)->RemoveKinBody(body);
            }
            if( std::find(_vecbodies.begin(),_vecbodies.end(),body) != _vecbodies.end() ) {
                throw openrave_exception(str(boost::format("KinBody::Init for %s, cannot Init a body while it is added to the environment\n")%body->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsXData(data) ) {
            // have to copy since it takes vector<char>
            std::vector<char> newdata(data.size()+1, 0);  // need a null-terminator
            std::copy(data.begin(),data.end(),newdata.begin());
            if( !RaveParseXData(shared_from_this(), body, newdata, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsIVData(data) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("iv data not supported",ORE_InvalidArguments);
        }
        else {
            InterfaceBasePtr pinterface = body;
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), PT_KinBody, pinterface, "kinbody", atts);
            if( !preader ) {
                return KinBodyPtr();
            }
            bool bSuccess = _ParseXMLData(preader, data);
            preader->endElement("kinbody");     // have to end the tag!
            body = RaveInterfaceCast<KinBody>(pinterface);
            if( !bSuccess || !body ) {
                return KinBodyPtr();
            }
            body->__struri = preader->_filename;
        }
        return body;
    }

    virtual InterfaceBasePtr ReadInterfaceURI(const std::string& filename, const AttributesList& atts)
    {
        try {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),atts,false);
            if( !preader ) {
                return InterfaceBasePtr();
            }
            bool bSuccess = _ParseXMLFile(preader, filename);
            boost::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = boost::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !bSuccess || !preadable || !preadable->_pinterface) {
                return InterfaceBasePtr();
            }
            preader->endElement(RaveGetInterfaceName(preadable->_pinterface->GetInterfaceType()));     // have to end the tag!
            preadable->_pinterface->__struri = filename;
            return preadable->_pinterface;
        }
        catch(const std::exception &ex) {
            RAVELOG_ERROR(str(boost::format("ReadInterfaceXMLFile exception: %s\n")%ex.what()));
        }
        return InterfaceBasePtr();
    }

    virtual InterfaceBasePtr ReadInterfaceURI(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( (type == PT_KinBody ||type == PT_Robot ) && _IsColladaFile(filename) ) {
            if( type == PT_KinBody ) {
                BOOST_ASSERT(!pinterface|| (pinterface->GetInterfaceType()==PT_KinBody||pinterface->GetInterfaceType()==PT_Robot));
                KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
                if( !RaveParseColladaFile(shared_from_this(), pbody, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = pbody;
            }
            else if( type == PT_Robot ) {
                BOOST_ASSERT(!pinterface||pinterface->GetInterfaceType()==PT_Robot);
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
                if( !RaveParseColladaFile(shared_from_this(), probot, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = probot;
            }
            else {
                return InterfaceBasePtr();
            }
            pinterface->__struri = filename;
        }
        if( (type == PT_KinBody ||type == PT_Robot ) && _IsXFile(filename) ) {
            if( type == PT_KinBody ) {
                BOOST_ASSERT(!pinterface|| (pinterface->GetInterfaceType()==PT_KinBody||pinterface->GetInterfaceType()==PT_Robot));
                KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
                if( !RaveParseXFile(shared_from_this(), pbody, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = pbody;
            }
            else if( type == PT_Robot ) {
                BOOST_ASSERT(!pinterface||pinterface->GetInterfaceType()==PT_Robot);
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
                if( !RaveParseXFile(shared_from_this(), probot, filename, atts) ) {
                    return InterfaceBasePtr();
                }
                pinterface = probot;
            }
            else {
                return InterfaceBasePtr();
            }
            pinterface->__struri = filename;
        }
        else {
            BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
            boost::shared_ptr<OpenRAVEXMLParser::InterfaceXMLReadable> preadable = boost::dynamic_pointer_cast<OpenRAVEXMLParser::InterfaceXMLReadable>(preader->GetReadable());
            if( !!preadable ) {
                if( !_ParseXMLFile(preader, filename) ) {
                    return InterfaceBasePtr();
                }
                preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType()));     // have to end the tag!
                pinterface = preadable->_pinterface;
            }
            else {
                pinterface = ReadInterfaceURI(filename,AttributesList());
                if( !!pinterface &&( pinterface->GetInterfaceType() != type) ) {
                    return InterfaceBasePtr();
                }
            }
            pinterface->__struri = filename;
        }
        return pinterface;
    }

    virtual InterfaceBasePtr ReadInterfaceData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        // check for collada?
        BaseXMLReaderPtr preader = OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(), type, pinterface, RaveGetInterfaceName(type), atts);
        if( !preader ) {
            return InterfaceBasePtr();
        }
        bool bSuccess = _ParseXMLData(preader, data);
        preader->endElement(RaveGetInterfaceName(pinterface->GetInterfaceType()));     // have to end the tag!
        if( !bSuccess ) {
            return InterfaceBasePtr();
        }
        pinterface->__struri = preader->_filename;
        return pinterface;
    }

    virtual boost::shared_ptr<TriMesh> ReadTrimeshURI(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, const AttributesList& atts)
    {
        RaveVector<float> diffuseColor, ambientColor;
        return _ReadTrimeshURI(ptrimesh,filename,diffuseColor, ambientColor, atts);
    }

    virtual boost::shared_ptr<TriMesh> _ReadTrimeshURI(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        string filedata = RaveFindLocalFile(filename);
        if( filedata.size() == 0 ) {
            return boost::shared_ptr<TriMesh>();
        }
        Vector vScaleGeometry(1,1,1);
        float ftransparency;
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                ss >> vScaleGeometry.x >> vScaleGeometry.y >> vScaleGeometry.z;
                if( !ss ) {
                    vScaleGeometry.z = vScaleGeometry.y = vScaleGeometry.x;
                }
            }
        }
        if( !ptrimesh ) {
            ptrimesh.reset(new TriMesh());
        }
        if( !OpenRAVEXMLParser::CreateTriMeshData(shared_from_this(),filedata, vScaleGeometry, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
            ptrimesh.reset();
        }
        return ptrimesh;
    }

    /// \brief parses the file into GeometryInfo and returns the full path of the file opened
    ///
    /// \param[in] listGeometries geometry list to be filled
    virtual std::string _ReadGeometriesFile(std::list<KinBody::GeometryInfo>& listGeometries, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        string filedata = RaveFindLocalFile(filename);
        if( filedata.size() == 0 ) {
            return std::string();
        }
        Vector vScaleGeometry(1,1,1);
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                ss >> vScaleGeometry.x >> vScaleGeometry.y >> vScaleGeometry.z;
                if( !ss ) {
                    vScaleGeometry.z = vScaleGeometry.y = vScaleGeometry.x;
                }
            }
        }
        if( OpenRAVEXMLParser::CreateGeometries(shared_from_this(),filedata, vScaleGeometry, listGeometries) && listGeometries.size() > 0 ) {
            return filedata;
        }
        listGeometries.clear();
        return std::string();
    }

    virtual void _AddViewer(ViewerBasePtr pnewviewer)
    {
        CHECK_INTERFACE(pnewviewer);
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        BOOST_ASSERT(find(_listViewers.begin(),_listViewers.end(),pnewviewer) == _listViewers.end() );
        _CheckUniqueName(ViewerBaseConstPtr(pnewviewer),true);
        _listViewers.push_back(pnewviewer);
    }

    virtual ViewerBasePtr GetViewer(const std::string& name) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( name.size() == 0 ) {
            return _listViewers.size() > 0 ? _listViewers.front() : ViewerBasePtr();
        }
        FOREACHC(itviewer, _listViewers) {
            if( (*itviewer)->GetName() == name ) {
                return *itviewer;
            }
        }
        return ViewerBasePtr();
    }

    void GetViewers(std::list<ViewerBasePtr>& listViewers) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        listViewers = _listViewers;
    }

    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->plot3(ppoints, numPoints, stride, fPointSize, color, drawstyle));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle, bool bhasalpha)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->plot3(ppoints, numPoints, stride, fPointSize, colors, drawstyle, bhasalpha));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlinestrip(ppoints, numPoints, stride, fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlinestrip(ppoints, numPoints, stride, fwidth,colors));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlinelist(ppoints, numPoints, stride, fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlinelist(ppoints, numPoints, stride, fwidth,colors));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawarrow(p1,p2,fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawbox(vpos, vextents));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawplane(tplane, vextents, vtexture));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawtrimesh(ppoints, stride, pIndices, numTriangles, color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawtrimesh(ppoints, stride, pIndices, numTriangles, colors));
        }
        return handles;
    }

    virtual KinBodyPtr GetBodyFromEnvironmentId(int id)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
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
        //_nCurSimTime = 0; // don't reset since it is important to keep time monotonic
        _nSimStartTime = utils::GetMicroTime()-_nCurSimTime;
    }

    virtual bool IsSimulationRunning() const {
        return _bEnableSimulation;
    }

    virtual void StopSimulation()
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _bEnableSimulation = false;
        _fDeltaSimTime = 1.0f;
    }
    virtual uint64_t GetSimulationTime() {
        return _nCurSimTime;
    }

    virtual void SetDebugLevel(int level) {
        RaveSetDebugLevel(level);
    }
    virtual int GetDebugLevel() const {
        return RaveGetDebugLevel();
    }

    virtual void GetPublishedBodies(std::vector<KinBody::BodyState>& vbodies, uint64_t timeout)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            vbodies = _vPublishedBodies;
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            vbodies = _vPublishedBodies;
        }
    }

    virtual void UpdatePublishedBodies(uint64_t timeout=0)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _UpdatePublishedBodies();
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT("timeout of %f s failed",(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            _UpdatePublishedBodies();
        }
    }

    virtual void _UpdatePublishedBodies()
    {
        // updated the published bodies, resize dynamically in case an exception occurs
        // when creating an item and bad data is left inside _vPublishedBodies
        _vPublishedBodies.resize(0);
        if( _vPublishedBodies.capacity() < _vecbodies.size() ) {
            _vPublishedBodies.reserve(_vecbodies.size());
        }

        std::vector<int> vdofbranches;
        FOREACH(itbody, _vecbodies) {
            _vPublishedBodies.push_back(KinBody::BodyState());
            KinBody::BodyState& state = _vPublishedBodies.back();
            state.pbody = *itbody;
            (*itbody)->GetLinkTransformations(state.vectrans, vdofbranches);
            (*itbody)->GetDOFValues(state.jointvalues);
            state.strname =(*itbody)->GetName();
            state.environmentid = (*itbody)->GetEnvironmentId();
            _vPublishedBodies.push_back(state);
        }
    }

protected:

    void _SetDefaultGravity()
    {
        if( !!_pPhysicsEngine ) {
            // At a latitude of L with altitude H (above sea level), the acceleration due to gravity at sea level is approximately
            // g= 9.780327 * ( 1 + .0053024*sin(L)**2 - .0000058*sin(2L)**2 ) - 0.000003086*H meters per second**2.
            // tokyo,japan 35.6894875 deg
            // rate of change with respect to altitude is da/dH= -2*g*R**2/(R+H)3 = -2g*a*/(R+H)
            _pPhysicsEngine->SetGravity(Vector(0,0,-9.797930195020351));
        }
    }

    virtual bool _ParseXMLFile(BaseXMLReaderPtr preader, const std::string& filename)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLFile(preader, filename);
    }

    virtual bool _ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLData(preader, pdata);
    }

    virtual void _Clone(boost::shared_ptr<Environment const> r, int options, bool bCheckSharedResources=false)
    {
        if( !bCheckSharedResources ) {
            Destroy();
        }

        boost::mutex::scoped_lock lockinit(_mutexInit);
        if( !bCheckSharedResources ) {
            SetCollisionChecker(CollisionCheckerBasePtr());
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        _nBodiesModifiedStamp = r->_nBodiesModifiedStamp;
        _homedirectory = r->_homedirectory;
        _fDeltaSimTime = r->_fDeltaSimTime;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _nEnvironmentIndex = r->_nEnvironmentIndex;
        _bRealTime = r->_bRealTime;

        _bInit = true;
        _bEnableSimulation = r->_bEnableSimulation;

        SetDebugLevel(r->GetDebugLevel());

        if( !bCheckSharedResources || !(options & Clone_Bodies) ) {
            {
                // clear internal interface lists
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
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
            }
            // a little tricky due to a deadlocking situation
            std::map<int, KinBodyWeakPtr> mapBodies;
            {
                boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
                mapBodies = _mapBodies;
                _mapBodies.clear();
            }
            mapBodies.clear();
        }

        list<ViewerBasePtr> listViewers = _listViewers;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listViewers.clear();
        }

        if( !(options & Clone_Viewer) ) {
            RAVELOG_DEBUG("resetting raveviewer\n");
            FOREACH(itviewer, listViewers) {
                // don't reset the viewer since it can already be dead
                // todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
                (*itviewer)->quitmainloop();
            }
            listViewers.clear();
        }

        EnvironmentMutex::scoped_lock lock(GetMutex());
        //boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds); // why is this here? if locked, then KinBody::_ComputeInternalInformation freezes on GetBodyFromEnvironmentId call

        bool bCollisionCheckerChanged = false;
        if( !!r->GetCollisionChecker() ) {
            if( !bCheckSharedResources || (!!_pCurrentChecker && _pCurrentChecker->GetXMLId() != r->GetCollisionChecker()->GetXMLId()) ) {
                try {
                    CollisionCheckerBasePtr p = RaveCreateCollisionChecker(shared_from_this(),r->GetCollisionChecker()->GetXMLId());
                    p->Clone(r->GetCollisionChecker(),options);
                    SetCollisionChecker(p);
                    bCollisionCheckerChanged = true;
                }
                catch(const std::exception& ex) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to clone collision checker %s: %s", r->GetCollisionChecker()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
                }
            }
        }
        else {
            SetCollisionChecker(CollisionCheckerBasePtr());
        }

        bool bPhysicsEngineChanged = false;
        if( !!r->GetPhysicsEngine() ) {
            if( !bCheckSharedResources || (!!_pPhysicsEngine && _pPhysicsEngine->GetXMLId() != r->GetPhysicsEngine()->GetXMLId()) ) {
                try {
                    PhysicsEngineBasePtr p = RaveCreatePhysicsEngine(shared_from_this(),r->GetPhysicsEngine()->GetXMLId());
                    p->Clone(r->GetPhysicsEngine(),options);
                    SetPhysicsEngine(p);
                    bPhysicsEngineChanged = true;
                }
                catch(const std::exception& ex) {
                    throw OPENRAVE_EXCEPTION_FORMAT("failed to clone physics engine %s: %s", r->GetPhysicsEngine()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
                }
            }
        }
        else {
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        if( options & Clone_Bodies ) {
            boost::timed_mutex::scoped_lock lock(r->_mutexInterfaces);
            std::vector<RobotBasePtr> vecrobots;
            std::vector<KinBodyPtr> vecbodies;
            std::vector<std::pair<Vector,Vector> > linkvelocities;
            _mapBodies.clear();
            if( bCheckSharedResources ) {
                // delete any bodies/robots from mapBodies that are not in r->_vecrobots and r->_vecbodies
                vecrobots.swap(_vecrobots);
                vecbodies.swap(_vecbodies);
            }
            // first initialize the pointers
            list<KinBodyPtr> listToClone, listToCopyState;
            FOREACHC(itrobot, r->_vecrobots) {
                try {
                    RobotBasePtr pnewrobot;
                    if( bCheckSharedResources ) {
                        FOREACH(itrobot2,vecrobots) {
                            if( (*itrobot2)->GetName() == (*itrobot)->GetName() && (*itrobot2)->GetKinematicsGeometryHash() == (*itrobot)->GetKinematicsGeometryHash() ) {
                                pnewrobot = *itrobot2;
                                break;
                            }
                        }
                    }
                    if( !pnewrobot ) {
                        pnewrobot = RaveCreateRobot(shared_from_this(), (*itrobot)->GetXMLId());
                        pnewrobot->_name = (*itrobot)->_name; // at least copy the names
                        listToClone.push_back(*itrobot);
                    }
                    else {
                        //TODO
                        //pnewrobot->ReleaseAllGrabbed(); // will re-grab later?
                        listToCopyState.push_back(*itrobot);
                    }
                    pnewrobot->_environmentid = (*itrobot)->GetEnvironmentId();
                    BOOST_ASSERT( _mapBodies.find(pnewrobot->GetEnvironmentId()) == _mapBodies.end() );
                    _vecbodies.push_back(pnewrobot);
                    _vecrobots.push_back(pnewrobot);
                    _mapBodies[pnewrobot->GetEnvironmentId()] = pnewrobot;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone robot %s: %s")%(*itrobot)->GetName()%ex.what()));
                }
            }
            FOREACHC(itbody, r->_vecbodies) {
                if( _mapBodies.find((*itbody)->GetEnvironmentId()) != _mapBodies.end() ) {
                    continue;
                }
                try {
                    KinBodyPtr pnewbody;
                    if( bCheckSharedResources ) {
                        FOREACH(itbody2,vecbodies) {
                            if( (*itbody2)->GetName() == (*itbody)->GetName() && (*itbody2)->GetKinematicsGeometryHash() == (*itbody)->GetKinematicsGeometryHash() ) {
                                pnewbody = *itbody2;
                                break;
                            }
                        }
                    }
                    if( !pnewbody ) {
                        pnewbody.reset(new KinBody(PT_KinBody,shared_from_this()));
                        pnewbody->_name = (*itbody)->_name; // at least copy the names
                        listToClone.push_back(*itbody);
                    }
                    else {
                        listToCopyState.push_back(*itbody);
                    }
                    pnewbody->_environmentid = (*itbody)->GetEnvironmentId();
                    _vecbodies.push_back(pnewbody);
                    _mapBodies[pnewbody->GetEnvironmentId()] = pnewbody;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone body %s: %s")%(*itbody)->GetName()%ex.what()));
                }
            }

            // copy state before cloning
            if( listToCopyState.size() > 0 ) {
                FOREACH(itbody,listToCopyState) {
                    KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                    if( bCollisionCheckerChanged ) {
                        GetCollisionChecker()->InitKinBody(pnewbody);
                    }
                    if( bPhysicsEngineChanged ) {
                        GetPhysicsEngine()->InitKinBody(pnewbody);
                    }
                    pnewbody->__hashkinematics = (*itbody)->__hashkinematics;
                    if( pnewbody->IsRobot() ) {
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                        // don't clone grabbed bodies!
                        RobotBase::RobotStateSaver saver(poldrobot, 0xffffffff&~KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewrobot);
                        pnewrobot->__hashrobotstructure = poldrobot->__hashrobotstructure;
                    }
                    else {
                        KinBody::KinBodyStateSaver saver(*itbody, 0xffffffff);
                        saver.Restore(pnewbody);
                    }
                }
            }

            // now clone
            FOREACHC(itbody, listToClone) {
                try {
                    KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                    if( !!pnewbody ) {
                        pnewbody->Clone(*itbody,options);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone body %s: %s")%(*itbody)->GetName()%ex.what()));
                }
            }
            FOREACH(itbody,listToClone) {
                KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                pnewbody->_ComputeInternalInformation();
                GetCollisionChecker()->InitKinBody(pnewbody);
                GetPhysicsEngine()->InitKinBody(pnewbody);
                pnewbody->__hashkinematics = (*itbody)->__hashkinematics; /// _ComputeInternalInformation resets the hashes
                if( pnewbody->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                    pnewrobot->__hashrobotstructure = poldrobot->__hashrobotstructure;
                }
            }
            // update the state after every body is initialized!
            FOREACH(itbody,listToClone) {
                KinBodyPtr pnewbody = _mapBodies[(*itbody)->GetEnvironmentId()].lock();
                if( (*itbody)->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_mapBodies[(*itbody)->GetEnvironmentId()].lock());
                    // need to also update active dof/active manip since it is erased by _ComputeInternalInformation
                    RobotBase::RobotStateSaver saver(poldrobot, KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator);
                    saver.Restore(pnewrobot);
                }
                else {
                    KinBody::KinBodyStateSaver saver(*itbody, KinBody::Save_LinkVelocities); // all the others should have been saved?
                    saver.Restore(pnewbody);
                }
            }
            if( listToCopyState.size() > 0 ) {
                // check for re-grabs after cloning is done
                FOREACH(itbody,listToCopyState) {
                    if( (*itbody)->IsRobot() ) {
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(*itbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_mapBodies[(*itbody)->GetEnvironmentId()].lock());
                        RobotBase::RobotStateSaver saver(poldrobot, KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewrobot);
                    }
                }
            }
        }
        if( options & Clone_Sensors ) {
            boost::timed_mutex::scoped_lock lock(r->_mutexInterfaces);
            FOREACHC(itsensor,r->_listSensors) {
                try {
                    SensorBasePtr pnewsensor = RaveCreateSensor(shared_from_this(), (*itsensor)->GetXMLId());
                    pnewsensor->Clone(*itsensor, options);
                    _listSensors.push_back(pnewsensor);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone sensor %s: %s")%(*itsensor)->GetName()%ex.what()));
                }
            }
        }
        // sensors might be attached on a robot?, so have to re-update
        FOREACH(itrobot, _vecrobots) {
            (*itrobot)->_UpdateAttachedSensors();
        }

        if( options & Clone_Simulation ) {
            _bEnableSimulation = r->_bEnableSimulation;
            _nCurSimTime = r->_nCurSimTime;
            _nSimStartTime = r->_nSimStartTime;
        }

        if( options & Clone_Viewer ) {
            list<ViewerBasePtr> listViewers2;
            r->GetViewers(listViewers2);
            FOREACH(itviewer2, listViewers2) {
                try {
                    ViewerBasePtr pviewer;
                    if( bCheckSharedResources ) {
                        FOREACH(itviewer,listViewers) {
                            if( (*itviewer)->GetXMLId() == (*itviewer2)->GetXMLId() ) {
                                pviewer = *itviewer;
                                listViewers.erase(itviewer);
                                break;
                            }
                        }
                    }
                    if( !pviewer ) {
                        pviewer = RaveCreateViewer(shared_from_this(),(*itviewer2)->GetXMLId());
                    }
                    pviewer->Clone(*itviewer2,options);
                    AddViewer(pviewer);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR(str(boost::format("failed to clone viewer %s: %s")%(*itviewer2)->GetName()%ex.what()));
                }
            }
        }

        // reset left-over viewers
        FOREACH(itviewer, listViewers) {
            (*itviewer)->quitmainloop();
        }
        listViewers.clear();

        if( !bCheckSharedResources ) {
            if( !!_threadSimulation ) {
                _threadSimulation->join();
            }
            _threadSimulation.reset(new boost::thread(boost::bind(&Environment::_SimulationThread,this)));
        }
    }

    virtual bool _CheckUniqueName(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        FOREACHC(itbody,_vecbodies) {
            if(( *itbody != pbody) &&( (*itbody)->GetName() == pbody->GetName()) ) {
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
            if(( *itsensor != psensor) &&( (*itsensor)->GetName() == psensor->GetName()) ) {
                if( bDoThrow ) {
                    throw openrave_exception(str(boost::format("sensor %s does not have unique name")%psensor->GetName()));
                }
                return false;
            }
        }
        return true;
    }
    virtual bool _CheckUniqueName(ViewerBaseConstPtr pviewer, bool bDoThrow=false) const
    {
        FOREACHC(itviewer,_listViewers) {
            if(( *itviewer != pviewer) &&( (*itviewer)->GetName() == pviewer->GetName()) ) {
                if( bDoThrow ) {
                    throw openrave_exception(str(boost::format("viewer '%s' does not have unique name")%pviewer->GetName()));
                }
                return false;
            }
        }
        return true;
    }

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
        uint64_t nLastUpdateTime = utils::GetMicroTime();
        uint64_t nLastSleptTime = utils::GetMicroTime();
        while( _bInit ) {
            bool bNeedSleep = true;
            boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv;
            if( _bEnableSimulation ) {
                bNeedSleep = false;
                lockenv = _LockEnvironmentWithTimeout(100000);
                if( !!lockenv ) {
                    //Get deltasimtime in microseconds
                    int64_t deltasimtime = (int64_t)(_fDeltaSimTime*1000000.0f);
                    try {
                        StepSimulation(_fDeltaSimTime);
                    }
                    catch(const std::exception &ex) {
                        RAVELOG_ERROR("simulation thread exception: %s\n",ex.what());
                    }
                    uint64_t passedtime = utils::GetMicroTime()-_nSimStartTime;
                    int64_t sleeptime = _nCurSimTime-passedtime;
                    //Hardcoded tolerance for now
                    const int tol=2;
                    if( _bRealTime ) {
                        if(( sleeptime > deltasimtime/tol) &&( sleeptime > 1000) ) {
                            lockenv.reset();
                            // sleep for less time since sleep isn't accurate at all and we have a 7ms buffer
                            int actual_sleep=max((int)sleeptime*6/8,1000);
                            boost::this_thread::sleep (boost::posix_time::microseconds(actual_sleep));
                            //RAVELOG_INFO("sleeptime ideal %d, actually slept: %d\n",(int)sleeptime,(int)actual_sleep);
                            nLastSleptTime = utils::GetMicroTime();
                            //Since already slept this cycle, wait till next time to sleep.
                            bNeedSleep = false;
                        }
                        else if( sleeptime < -deltasimtime/tol && ( sleeptime < -1000) ) {
                            // simulation is getting late, so catch up (doesn't happen often in light loads)
                            //RAVELOG_INFO("sim catching up: %d\n",-(int)sleeptime);
                            _nSimStartTime += -sleeptime;     //deltasimtime;
                        }
                    }
                    else {
                        nLastSleptTime = utils::GetMicroTime();
                    }

                    //RAVELOG_INFOA("sim: %f, real: %f\n",_nCurSimTime*1e-6f,(utils::GetMicroTime()-_nSimStartTime)*1e-6f);
                }
            }

            if( utils::GetMicroTime()-nLastSleptTime > 20000 ) {     // 100000 freezes the environment
                lockenv.reset();
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                bNeedSleep = false;
                nLastSleptTime = utils::GetMicroTime();
            }

            if( utils::GetMicroTime()-nLastUpdateTime > 10000 ) {
                if( !lockenv ) {
                    lockenv = _LockEnvironmentWithTimeout(100000);
                }
                if( !!lockenv ) {
                    nLastUpdateTime = utils::GetMicroTime();
                    // environment might be getting destroyed during this call, so to avoid a potential deadlock, add a timeout
                    try {
                        UpdatePublishedBodies(1000000); // 1.0s
                    }
                    catch(const std::exception& ex) {
                        RAVELOG_WARN("timeout of UpdatePublishedBodies\n");
                    }
                }
            }

            //TODO: Verify if this always has to happen even if thread slept in RT if statement above
            lockenv.reset(); // always release at the end of loop to give other threads time
            if( bNeedSleep ) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> _LockEnvironmentWithTimeout(uint64_t timeout)
    {
        // try to acquire the lock
#if BOOST_VERSION >= 103500
        boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetMutex(),boost::defer_lock_t()));
#else
        boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetMutex(),false));
#endif
        uint64_t basetime = utils::GetMicroTime();
        while(utils::GetMicroTime()-basetime<timeout ) {
            lockenv->try_lock();
            if( !!*lockenv ) {
                break;
            }
        }

        if( !*lockenv ) {
            lockenv.reset();
        }
        return lockenv;
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

    static bool _IsXFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 2 ) {
            return false;
        }
        return filename[len-2] == '.' && ::tolower(filename[len-1]) == 'x';
    }

    static bool _IsXData(const std::string& data)
    {
        return data.size() >= 4 && data[0] == 'x' && data[1] == 'o' && data[2] == 'f' && data[3] == ' ';
    }

    static bool _IsIVData(const std::string& data)
    {
        if( data.size() >= 10 ) {
            if( data.substr(0,10) == string("#Inventor ") ) {
                return true;
            }
        }
        return false;
    }

    static bool _IsOpenRAVEFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 4 ) {
            return false;
        }
        if(( filename[len-4] == '.') &&( ::tolower(filename[len-3]) == 'x') &&( ::tolower(filename[len-2]) == 'm') &&( ::tolower(filename[len-1]) == 'l') ) {
            return true;
        }
        return false;
    }
    static bool _IsRigidModelFile(const std::string& filename)
    {
        static boost::array<std::string,21> s_geometryextentsions = { { "iv","vrml","wrl","stl","blend","3ds","ase","obj","ply","dxf","lwo","lxo","ac","ms3d","x","mesh.xml","irrmesh","irr","nff","off","raw"}};
        FOREACH(it, s_geometryextentsions) {
            if( filename.size() > it->size()+1 ) {
                size_t len = filename.size();
                if( filename.at(len-it->size()-1) == '.' ) {
                    bool bsuccess = true;
                    for(size_t i = 0; i < it->size(); ++i) {
                        if( ::tolower(filename[len-i-1]) != (*it)[it->size()-i-1] ) {
                            bsuccess = false;
                            break;
                        }
                    }
                    if( bsuccess ) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    std::vector<RobotBasePtr> _vecrobots;      ///< robots (possibly controlled)
    std::vector<KinBodyPtr> _vecbodies;     ///< all objects that are collidable (includes robots)

    list<ModuleBasePtr> _listModules;     ///< modules loaded in the environment
    list<SensorBasePtr> _listSensors;     ///< sensors loaded in the environment
    list<ViewerBasePtr> _listViewers;     ///< viewers loaded in the environment

    dReal _fDeltaSimTime;                    ///< delta time for simulate step
    uint64_t _nCurSimTime;                        ///< simulation time since the start of the environment
    uint64_t _nSimStartTime;
    int _nBodiesModifiedStamp;     ///< incremented every tiem bodies vector is modified
    bool _bRealTime;

    CollisionCheckerBasePtr _pCurrentChecker;
    PhysicsEngineBasePtr _pPhysicsEngine;

    int _nEnvironmentIndex;                   ///< next network index
    std::map<int, KinBodyWeakPtr> _mapBodies;     ///< a map of all the bodies in the environment. Controlled through the KinBody constructor and destructors

    boost::shared_ptr<boost::thread> _threadSimulation;                      ///< main loop for environment simulation

    mutable EnvironmentMutex _mutexEnvironment;          ///< protects internal data from multithreading issues
    mutable boost::mutex _mutexEnvironmentIds;      ///< protects _vecbodies/_vecrobots from multithreading issues
    mutable boost::timed_mutex _mutexInterfaces;     ///< lock when managing interfaces like _listOwnedInterfaces, _listModules, _mapBodies
    mutable boost::mutex _mutexInit;     ///< lock for destroying the environment

    vector<KinBody::BodyState> _vPublishedBodies;
    string _homedirectory;
    UserDataPtr _handlegenericrobot, _handlegenerictrajectory, _handlegenericphysicsengine, _handlegenericcollisionchecker;

    list<InterfaceBasePtr> _listOwnedInterfaces;

    std::list<UserDataWeakPtr> _listRegisteredCollisionCallbacks;     ///< see EnvironmentBase::RegisterCollisionCallback

    bool _bInit;                   ///< environment is initialized
    bool _bEnableSimulation;            ///< enable simulation loop

    friend class EnvironmentXMLReader;
};

#endif
