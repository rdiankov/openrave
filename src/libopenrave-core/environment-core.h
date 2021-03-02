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

#include "ravep.h"
#include "colladaparser/colladacommon.h"
#include "jsonparser/jsoncommon.h"

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#include <pcrecpp.h>

#define CHECK_INTERFACE(pinterface) { \
        if( (pinterface)->GetEnv() != shared_from_this() ) \
            throw openrave_exception(str(boost::format(_("env=%d, Interface %s:%s is from a different environment (env=%d) than the current one."))%GetId()%RaveGetInterfaceName((pinterface)->GetInterfaceType())%(pinterface)->GetXMLId()%(pinterface)->GetEnv()->GetId()),ORE_InvalidArguments); \
} \

#define CHECK_COLLISION_BODY(body) { \
        CHECK_INTERFACE(body); \
}

inline dReal TransformDistanceFast(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*e);
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
                boost::timed_mutex::scoped_lock lock(penv->_mutexInterfaces);
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

    class BodyCallbackData : public UserData
    {
public:
        BodyCallbackData(const BodyCallbackFn& callback, boost::shared_ptr<Environment> penv) : _callback(callback), _pweakenv(penv) {
        }
        virtual ~BodyCallbackData() {
            boost::shared_ptr<Environment> penv = _pweakenv.lock();
            if( !!penv ) {
                boost::timed_mutex::scoped_lock lock(penv->_mutexInterfaces);
                penv->_listRegisteredBodyCallbacks.erase(_iterator);
            }
        }

        list<UserDataWeakPtr>::iterator _iterator;
        BodyCallbackFn _callback;
protected:
        boost::weak_ptr<Environment> _pweakenv;
    };
    friend class BodyCallbackData;
    typedef boost::shared_ptr<BodyCallbackData> BodyCallbackDataPtr;

public:
    Environment() : EnvironmentBase()
    {
        _homedirectory = RaveGetHomeDirectory();
        RAVELOG_DEBUG_FORMAT("env=%d, setting openrave home directory to %s", GetId()%_homedirectory);

        _nBodiesModifiedStamp = 0;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _bRealTime = true;
        _bInit = false;
        _bEnableSimulation = true;     // need to start by default
        _unit = std::make_pair("meter",1.0); //default unit settings

        _vRapidJsonLoadBuffer.resize(4000000);
        _prLoadEnvAlloc.reset(new rapidjson::MemoryPoolAllocator<>(&_vRapidJsonLoadBuffer[0], _vRapidJsonLoadBuffer.size()));

        _handlegenericrobot = RaveRegisterInterface(PT_Robot,"GenericRobot", RaveGetInterfaceHash(PT_Robot), GetHash(), CreateGenericRobot);
        _handlegenerictrajectory = RaveRegisterInterface(PT_Trajectory,"GenericTrajectory", RaveGetInterfaceHash(PT_Trajectory), GetHash(), CreateGenericTrajectory);
        _handlemulticontroller = RaveRegisterInterface(PT_Controller,"GenericMultiController", RaveGetInterfaceHash(PT_Controller), GetHash(), CreateMultiController);
        _handlegenericphysicsengine = RaveRegisterInterface(PT_PhysicsEngine,"GenericPhysicsEngine", RaveGetInterfaceHash(PT_PhysicsEngine), GetHash(), CreateGenericPhysicsEngine);
        _handlegenericcollisionchecker = RaveRegisterInterface(PT_CollisionChecker,"GenericCollisionChecker", RaveGetInterfaceHash(PT_CollisionChecker), GetHash(), CreateGenericCollisionChecker);
    }

    virtual ~Environment()
    {
        Destroy();
    }

    virtual void Init(bool bStartSimulationThread=true)
    {
        boost::mutex::scoped_lock lockinit(_mutexInit);
        if( _bInit ) {
            RAVELOG_WARN("environment is already initialized, ignoring\n");
            return;
        }

        _nBodiesModifiedStamp = 0;

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

        const char* pOPENRAVE_DEFAULT_COLLISIONCHECKER = std::getenv("OPENRAVE_DEFAULT_COLLISIONCHECKER");
        if( !!pOPENRAVE_DEFAULT_COLLISIONCHECKER && strlen(pOPENRAVE_DEFAULT_COLLISIONCHECKER) > 0 ) {
            localchecker = RaveCreateCollisionChecker(shared_from_this(), std::string(pOPENRAVE_DEFAULT_COLLISIONCHECKER));
        }

        if( !localchecker ) {
            boost::array<string,4> checker_prefs = { { "fcl_", "ode", "bullet", "pqp"}};     // ode takes priority since bullet has some bugs with deleting bodies
            FOREACH(itchecker,checker_prefs) {
                localchecker = RaveCreateCollisionChecker(shared_from_this(), *itchecker);
                if( !!localchecker ) {
                    break;
                }
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

        if( bStartSimulationThread ) {
            _bEnableSimulation = true;
            _StartSimulationThread();
        }
        else {
            // since not running, disable it
            _bEnableSimulation = false;
        }
    }

    virtual void Destroy()
    {
        boost::mutex::scoped_lock lockdestroy(_mutexInit);
        if( !_bInit ) {
            RAVELOG_VERBOSE_FORMAT("env=%d is already destroyed", GetId());
            return;
        }

        // destruction order is *very* important, don't touch it without consultation
        _bInit = false;

        RAVELOG_VERBOSE_FORMAT("env=%d destructor", GetId());
        _StopSimulationThread();

        // destroy the modules (their destructors could attempt to lock environment, so have to do it before global lock)
        // however, do not clear the _listModules yet
        RAVELOG_DEBUG_FORMAT("env=%d destroy module", GetId());
        list< pair<ModuleBasePtr, std::string> > listModules;
        list<ViewerBasePtr> listViewers = _listViewers;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules = _listModules;
            listViewers = _listViewers;
        }
        FOREACH(itmodule,listModules) {
            itmodule->first->Destroy();
        }
        listModules.clear();

        FOREACH(itviewer, listViewers) {
            // don't reset the viewer since it can already be dead
            // todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
            RAVELOG_DEBUG_FORMAT("quitting viewer %s", (*itviewer)->GetXMLId());
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

            // clear internal interface lists, have to Destroy all kinbodys without locking _mutexInterfaces since some can hold BodyCallbackData, which requires to lock _mutexInterfaces
            std::vector<RobotBasePtr> vecrobots;
            std::vector<KinBodyPtr> vecbodies;
            list<SensorBasePtr> listSensors;
            {
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                vecrobots.swap(_vecrobots);
                vecbodies.swap(_vecbodies);
                listSensors.swap(_listSensors);
                _vPublishedBodies.clear();
                _nBodiesModifiedStamp++;
                _listModules.clear();
                _listViewers.clear();
                _listOwnedInterfaces.clear();
            }

            // destroy the dangling pointers outside of _mutexInterfaces

            // release all grabbed
            FOREACH(itrobot,vecrobots) {
                (*itrobot)->ReleaseAllGrabbed();
            }
            FOREACH(itbody,vecbodies) {
                (*itbody)->Destroy();
            }
            vecbodies.clear();
            FOREACH(itrobot,vecrobots) {
                (*itrobot)->Destroy();
            }
            vecrobots.clear();

            FOREACH(itsensor,listSensors) {
                (*itsensor)->Configure(SensorBase::CC_PowerOff);
                (*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
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
        list<ViewerBasePtr> listViewers;
        GetViewers(listViewers);
        if( listViewers.size() > 0 ) {
            RAVELOG_DEBUG("resetting raveviewer\n");
            FOREACH(itviewer, listViewers) {
                (*itviewer)->Reset();
            }
        }

        EnvironmentMutex::scoped_lock lockenv(GetMutex());

        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment();
        }
        std::vector<KinBodyPtr> vcallbackbodies;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);

            FOREACH(itbody,_vecbodies) {
                (*itbody)->_environmentid=0;
                (*itbody)->Destroy();
            }
            if( _listRegisteredBodyCallbacks.size() > 0 ) {
                vcallbackbodies.insert(vcallbackbodies.end(), _vecbodies.begin(), _vecbodies.end());
            }
            _vecbodies.clear();
            FOREACH(itrobot,_vecrobots) {
                (*itrobot)->_environmentid=0;
                (*itrobot)->Destroy();
            }
            if( _listRegisteredBodyCallbacks.size() > 0 ) {
                vcallbackbodies.insert(vcallbackbodies.end(), _vecrobots.begin(), _vecrobots.end());
            }
            _vecrobots.clear();
            _vPublishedBodies.clear();
            _nBodiesModifiedStamp++;

            _vecWeakBodies.clear();
            _environmentIndexRecyclePool.clear();

            FOREACH(itsensor,_listSensors) {
                (*itsensor)->Configure(SensorBase::CC_PowerOff);
                (*itsensor)->Configure(SensorBase::CC_RenderGeometryOff);
            }
            _listSensors.clear();
        }
        if( vcallbackbodies.size() > 0 ) {
            FOREACH(itbody, vcallbackbodies) {
                _CallBodyCallbacks(*itbody, 0);
            }
            vcallbackbodies.clear();
        }

        list< pair<ModuleBasePtr, std::string> > listModules;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules = _listModules;
        }

        FOREACH(itmodule,listModules) {
            itmodule->first->Reset();
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
            RAVELOG_WARN_FORMAT("Error %d with executing module %s", ret%module->GetXMLId());
        }
        else {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listModules.emplace_back(module,  cmdargs);
        }

        return ret;
    }

    void GetModules(std::list<ModuleBasePtr>& listModules, uint64_t timeout) const
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listModules.clear();
            FOREACHC(it, _listModules) {
                listModules.push_back(it->first);
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            listModules.clear();
            FOREACHC(it, _listModules) {
                listModules.push_back(it->first);
            }
        }
    }

    bool LoadURI(const std::string& uri, const AttributesList& atts) override
    {
        if ( _IsColladaURI(uri) ) {
            return RaveParseColladaURI(shared_from_this(), uri, atts);
        }
        else if ( _IsJSONURI(uri) ) {
            _ClearRapidJsonBuffer();
            return RaveParseJSONURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else if ( _IsMsgPackURI(uri) ) {
            _ClearRapidJsonBuffer();
            return RaveParseMsgPackURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else {
            RAVELOG_WARN_FORMAT("load failed on uri '%s' since could not determine the file type", uri);
        }
        return false;
    }

    virtual bool Load(const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        OpenRAVEXMLParser::GetXMLErrorCount() = 0;
        if( _IsColladaURI(filename) ) {
            if( RaveParseColladaURI(shared_from_this(), filename, atts) ) {
                UpdatePublishedBodies();
                return true;
            }
        }
        else if( _IsColladaFile(filename) ) {
            if( RaveParseColladaFile(shared_from_this(), filename, atts) ) {
                UpdatePublishedBodies();
                return true;
            }
        }
        else if( _IsJSONFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( RaveParseJSONFile(shared_from_this(), filename, UFIM_Exact, atts, *_prLoadEnvAlloc) ) {
                return true;
            }
        }
        else if( _IsMsgPackFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( RaveParseMsgPackFile(shared_from_this(), filename, UFIM_Exact, atts, *_prLoadEnvAlloc) ) {
                return true;
            }
        }
        else if( _IsXFile(filename) ) {
            RobotBasePtr robot;
            if( RaveParseXFile(shared_from_this(), robot, filename, atts) ) {
                _AddRobot(robot, IAM_AllowRenaming);
                UpdatePublishedBodies();
                return true;
            }
        }
        else if( !_IsOpenRAVEFile(filename) && _IsRigidModelFile(filename) ) {
            KinBodyPtr pbody = ReadKinBodyURI(KinBodyPtr(),filename,atts);
            if( !!pbody ) {
                _AddKinBody(pbody,IAM_AllowRenaming);
                UpdatePublishedBodies();
                return true;
            }
        }
        else {
            if( _ParseXMLFile(OpenRAVEXMLParser::CreateInterfaceReader(shared_from_this(),atts,true), filename) ) {
                if( OpenRAVEXMLParser::GetXMLErrorCount() == 0 ) {
                    UpdatePublishedBodies();
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
        if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            return RaveParseJSONData(shared_from_this(), data, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            return RaveParseMsgPackData(shared_from_this(), data, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        return _ParseXMLData(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(),atts),data);
    }

    bool LoadJSON(const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts) override
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return RaveParseJSON(shared_from_this(), rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, atts, *_prLoadEnvAlloc);
    }

    virtual void Save(const std::string& filename, SelectionOptions options, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            if( _IsJSONFile(filename) ) {
                _ClearRapidJsonBuffer();
                RaveWriteJSONFile(shared_from_this(),filename,atts,*_prLoadEnvAlloc);
            }
            else if( _IsMsgPackFile(filename) ) {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackFile(shared_from_this(),filename,atts,*_prLoadEnvAlloc);
            }
            else {
                RaveWriteColladaFile(shared_from_this(),filename,atts);
            }
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
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

        if( _IsJSONFile(filename) ) {
            if( listbodies.size() == 1 ) {
                _ClearRapidJsonBuffer();
                RaveWriteJSONFile(listbodies.front(),filename,atts,*_prLoadEnvAlloc);
            }
            else {
                _ClearRapidJsonBuffer();
                RaveWriteJSONFile(listbodies,filename,atts,*_prLoadEnvAlloc);
            }
        }
        else if( _IsMsgPackFile(filename) ) {
            if( listbodies.size() == 1 ) {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackFile(listbodies.front(),filename,atts,*_prLoadEnvAlloc);
            }
            else {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackFile(listbodies,filename,atts,*_prLoadEnvAlloc);
            }
        }
        else {
            if( listbodies.size() == 1 ) {
                RaveWriteColladaFile(listbodies.front(),filename,atts);
            }
            else {
                RaveWriteColladaFile(listbodies,filename,atts);
            }
        }
    }

    virtual void SerializeJSON(rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, SelectionOptions options, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            RaveWriteJSON(shared_from_this(), rEnvironment, allocator, atts);
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
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
            RaveWriteJSON(listbodies.front(), rEnvironment, allocator, atts);
        }
        else {
            RaveWriteJSON(listbodies, rEnvironment, allocator, atts);
        }
    }

    virtual void WriteToMemory(const std::string& filetype, std::vector<char>& output, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList())
    {
        if (filetype != "collada" && filetype != "json" && filetype != "msgpack") {
            throw OPENRAVE_EXCEPTION_FORMAT("got invalid filetype %s, only support collada and json", filetype, ORE_InvalidArguments);
        }

        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::list<KinBodyPtr> listbodies;
        switch(options) {
        case SO_Everything:
            if (filetype == "collada") {
                RaveWriteColladaMemory(shared_from_this(), output, atts);
            }
            else if (filetype == "json") {
                _ClearRapidJsonBuffer();
                RaveWriteJSONMemory(shared_from_this(), output, atts,*_prLoadEnvAlloc);
            }
            else if (filetype == "msgpack") {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackMemory(shared_from_this(), output, atts,*_prLoadEnvAlloc);
            }
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body %s", itatt->second);
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
            if (filetype == "collada") {
                RaveWriteColladaMemory(listbodies.front(), output, atts);
            }
            else if (filetype == "json") {
                _ClearRapidJsonBuffer();
                RaveWriteJSONMemory(listbodies.front(), output, atts,*_prLoadEnvAlloc);
            }
            else if (filetype == "msgpack") {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackMemory(listbodies.front(), output, atts,*_prLoadEnvAlloc);
            }
        }
        else {
            if (filetype == "collada") {
                RaveWriteColladaMemory(listbodies, output, atts);
            }
            else if (filetype == "json") {
                _ClearRapidJsonBuffer();
                RaveWriteJSONMemory(listbodies, output, atts,*_prLoadEnvAlloc);
            }
            else if (filetype == "msgpack") {
                _ClearRapidJsonBuffer();
                RaveWriteMsgPackMemory(listbodies, output, atts,*_prLoadEnvAlloc);
            }
        }
    }

    virtual void Add(InterfaceBasePtr pinterface, InterfaceAddMode addMode, const std::string& cmdargs)
    {
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_Robot: _AddRobot(RaveInterfaceCast<RobotBase>(pinterface),addMode); break;
        case PT_KinBody: _AddKinBody(RaveInterfaceCast<KinBody>(pinterface),addMode); break;
        case PT_Module: {
            int ret = AddModule(RaveInterfaceCast<ModuleBase>(pinterface),cmdargs);
            OPENRAVE_ASSERT_OP_FORMAT(ret,==,0,"module %s failed with args: %s",pinterface->GetXMLId()%cmdargs,ORE_InvalidArguments);
            break;
        }
        case PT_Viewer: _AddViewer(RaveInterfaceCast<ViewerBase>(pinterface)); break;
        case PT_Sensor: _AddSensor(RaveInterfaceCast<SensorBase>(pinterface),addMode); break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("Interface %d cannot be added to the environment"),pinterface->GetInterfaceType(),ORE_InvalidArguments);
        }
    }

    virtual void _AddKinBody(KinBodyPtr pbody, InterfaceAddMode addMode)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pbody);
        if( !utils::IsValidName(pbody->GetName()) ) {
            throw openrave_exception(str(boost::format(_("kinbody name: \"%s\" is not valid"))%pbody->GetName()));
        }

        if( !_CheckUniqueName(KinBodyConstPtr(pbody), !!(addMode & IAM_StrictNameChecking)) ) {
            // continue to add random numbers until a unique name is found
            string oldname=pbody->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                pbody->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(pbody), false) ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, setting body name from %s -> %s due to conflict", GetId()%oldname%newname);
                    break;
                }
            }
        }
        if( !_CheckUniqueId(KinBodyConstPtr(pbody), !!(addMode & IAM_StrictIdChecking)) ) {
            // continue to add random numbers until a unique name is found
            string oldname=pbody->GetId(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                pbody->SetId(newname);
                if( utils::IsValidName(newname) && _CheckUniqueId(KinBodyConstPtr(pbody), false) ) {
                    if( !oldname.empty() ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, setting body id from %s -> %s due to conflict", GetId()%oldname%newname);
                    }
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
        if( !!pbody->GetSelfCollisionChecker() && pbody->GetSelfCollisionChecker() != _pCurrentChecker ) {
            // also initialize external collision checker if specified for this body
            pbody->GetSelfCollisionChecker()->InitKinBody(pbody);
        }
        _pPhysicsEngine->InitKinBody(pbody);
        // send all the changed callbacks of the body since anything could have changed
        pbody->_PostprocessChangedParameters(0xffffffff&~KinBody::Prop_JointMimic&~KinBody::Prop_LinkStatic&~KinBody::Prop_BodyRemoved);
        _CallBodyCallbacks(pbody, 1);
    }

    virtual void _AddRobot(RobotBasePtr robot, InterfaceAddMode addMode)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(robot);
        if( !robot->IsRobot() ) {
            throw openrave_exception(str(boost::format(_("kinbody \"%s\" is not a robot"))%robot->GetName()));
        }
        if( !utils::IsValidName(robot->GetName()) ) {
            throw openrave_exception(str(boost::format(_("kinbody name: \"%s\" is not valid"))%robot->GetName()));
        }

        if( !_CheckUniqueName(KinBodyConstPtr(robot), !!(addMode & IAM_StrictNameChecking)) ) {
            // continue to add random numbers until a unique name is found
            string oldname=robot->GetName(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                robot->SetName(newname);
                if( utils::IsValidName(newname) && _CheckUniqueName(KinBodyConstPtr(robot),false) ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, setting robot name from %s -> %s due to conflict", GetId()%oldname%newname);
                    break;
                }
            }
        }
        if( !_CheckUniqueId(KinBodyConstPtr(robot), !!(addMode & IAM_StrictIdChecking)) ) {
            // continue to add random numbers until a unique name is found
            string oldname=robot->GetId(),newname;
            for(int i = 0;; ++i) {
                newname = str(boost::format("%s%d")%oldname%i);
                robot->SetId(newname);
                if( utils::IsValidName(newname) && _CheckUniqueId(KinBodyConstPtr(robot),false) ) {
                    if( !oldname.empty() ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, setting robot id from %s -> %s due to conflict", GetId()%oldname%newname);
                    }
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
        robot->_ComputeInternalInformation(); // have to do this after _vecrobots is added since SensorBase::SetName can call EnvironmentBase::GetSensor to initialize itself
        _pCurrentChecker->InitKinBody(robot);
        if( !!robot->GetSelfCollisionChecker() && robot->GetSelfCollisionChecker() != _pCurrentChecker ) {
            // also initialize external collision checker if specified for this body
            robot->GetSelfCollisionChecker()->InitKinBody(robot);
        }
        _pPhysicsEngine->InitKinBody(robot);
        // send all the changed callbacks of the body since anything could have changed
        robot->_PostprocessChangedParameters(0xffffffff&~KinBody::Prop_JointMimic&~KinBody::Prop_LinkStatic&~KinBody::Prop_BodyRemoved);
        _CallBodyCallbacks(robot, 1);
    }

    virtual void _AddSensor(SensorBasePtr psensor, InterfaceAddMode addMode)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(psensor);
        if( !utils::IsValidName(psensor->GetName()) ) {
            throw openrave_exception(str(boost::format(_("sensor name: \"%s\" is not valid"))%psensor->GetName()));
        }

        if( !_CheckUniqueName(SensorBaseConstPtr(psensor), !!(addMode & IAM_StrictNameChecking)) ) {
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
        // no id for sensor right now
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listSensors.push_back(psensor);
        }
        psensor->Configure(SensorBase::CC_PowerOn);
    }

    virtual bool Remove(InterfaceBasePtr pinterface)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_KinBody:
        case PT_Robot: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
            {
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                vector<KinBodyPtr>::iterator it = std::find(_vecbodies.begin(), _vecbodies.end(), pbody);
                if( it == _vecbodies.end() ) {
                    return false;
                }
                _RemoveKinBodyFromIterator(it);
            }
            // pbody is valid so run any callbacks and exit
            _CallBodyCallbacks(pbody, 0);
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
            ModuleBasePtr pmodule = RaveInterfaceCast<ModuleBase>(pinterface);
            FOREACH(itmodule, _listModules) {
                if( itmodule->first == pmodule ) {
                    itmodule->first->Destroy();
                    _listModules.erase(itmodule);
                    return true;
                }
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
            RAVELOG_WARN_FORMAT("unmanaged interfaces of type %s cannot be removed", RaveGetInterfaceName(pinterface->GetInterfaceType()));
            break;
        }
        return false;
    }

    virtual bool RemoveKinBodyByName(const std::string& name)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        KinBodyPtr pbody;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            vector<KinBodyPtr>::iterator it = _vecbodies.end();
            FOREACHC(itbody, _vecbodies) {
                if( (*itbody)->GetName() == name ) {
                    it = itbody;
                    break;
                }
            }
            if( it == _vecbodies.end() ) {
                return false;
            }
            pbody = *it;
            _RemoveKinBodyFromIterator(it);
        }
        // pbody is valid so run any callbacks and exit
        _CallBodyCallbacks(pbody, 0);
        return true;
    }

    virtual UserDataPtr RegisterBodyCallback(const BodyCallbackFn& callback)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        BodyCallbackDataPtr pdata(new BodyCallbackData(callback,boost::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredBodyCallbacks.insert(_listRegisteredBodyCallbacks.end(),pdata);
        return pdata;
    }

    KinBodyPtr GetKinBody(const std::string& pname) const override
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecbodies) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
        return KinBodyPtr();
    }

    KinBodyPtr GetKinBodyById(const std::string& id) const override
    {
        if( id.empty() ) {
            return KinBodyPtr();
        }

        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        for(const KinBodyPtr& pbody : _vecbodies) {
            if(pbody->GetId()==id) {
                return pbody;
            }
        }
        return KinBodyPtr();
    }

    int GetNumBodies() const override
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        return (int)_vecbodies.size();
    }

    int GetMaxEnvironmentBodyIndex() const override
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);

        // bodies are sorted by index, so last body should have the largest
        const int lastBodyEnvironmentBodyIndex = _vecbodies.back()->GetEnvironmentBodyIndex();
        BOOST_ASSERT(lastBodyEnvironmentBodyIndex > 0 && lastBodyEnvironmentBodyIndex == (_vecbodies.size() - 1));
        
        return (int)_vecbodies.back()->GetEnvironmentBodyIndex();
    }

    virtual RobotBasePtr GetRobot(const std::string& pname) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        FOREACHC(it, _vecrobots) {
            if((*it)->GetName()==pname) {
                return *it;
            }
        }
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
            RAVELOG_DEBUG_FORMAT("env %d, disabling physics for", GetId());
            _pPhysicsEngine = RaveCreatePhysicsEngine(shared_from_this(),"GenericPhysicsEngine");
            _SetDefaultGravity();
        }
        else {
            RAVELOG_DEBUG_FORMAT("setting %s physics engine", _pPhysicsEngine->GetXMLId());
        }
        _pPhysicsEngine->InitEnvironment();
        return true;
    }

    virtual PhysicsEngineBasePtr GetPhysicsEngine() const {
        return _pPhysicsEngine;
    }

    virtual UserDataPtr RegisterCollisionCallback(const CollisionCallbackFn& callback)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        CollisionCallbackDataPtr pdata(new CollisionCallbackData(callback,boost::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredCollisionCallbacks.insert(_listRegisteredCollisionCallbacks.end(),pdata);
        return pdata;
    }
    virtual bool HasRegisteredCollisionCallbacks() const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        return _listRegisteredCollisionCallbacks.size() > 0;
    }

    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>& listcallbacks) const
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
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
            RAVELOG_DEBUG_FORMAT("setting %s collision checker", _pCurrentChecker->GetXMLId());
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

    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(trimesh,pbody,report);
    }

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckStandaloneSelfCollision(pbody,report);
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
        list< pair<ModuleBasePtr, std::string> > listModules;
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
            itmodule->first->SimulationStep(fTimeStep);
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
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

    virtual void Triangulate(TriMesh& trimesh, const KinBody &body)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());     // reading collision data, so don't want anyone modifying it
        FOREACHC(it, body.GetLinks()) {
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
                    Triangulate(trimesh, **itbody);
                }
                break;

            case SO_Robots:
                if( !!robot ) {
                    Triangulate(trimesh, **itbody);
                }
                break;
            case SO_Everything:
                Triangulate(trimesh, **itbody);
                break;
            case SO_Body:
                if( (*itbody)->GetName() == selectname ) {
                    Triangulate(trimesh, **itbody);
                }
                break;
            case SO_AllExceptBody:
                if( (*itbody)->GetName() != selectname ) {
                    Triangulate(trimesh, **itbody);
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
                throw openrave_exception(str(boost::format(_("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        if( _IsColladaURI(filename) ) {
            if( !RaveParseColladaURI(shared_from_this(), robot, filename, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONURI(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackURI(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsColladaFile(filename) ) {
            if( !RaveParseColladaFile(shared_from_this(), robot, filename, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONFile(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackFile(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
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
                    }
                    else {
                        robot.reset();
                    }
                }
                else {
                    robot.reset();
                }
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
            //robot->__struri = filename;
        }

        // have to set the URI to the passed in one rather than the resolved one, otherwise external components won't be able to compare if a URI is equivalent or not
        if( !!robot ) {
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
                throw openrave_exception(str(boost::format(_("KinRobot::Init for %s, cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONData(shared_from_this(), robot, data, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackData(shared_from_this(), robot, data, atts, *_prLoadEnvAlloc) ) {
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
            throw OPENRAVE_EXCEPTION_FORMAT0(_("iv data not supported"),ORE_InvalidArguments);
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

        if( !!robot ) {
            // check if have to reset the URI
            FOREACHC(itatt, atts) {
                if( itatt->first == "uri" ) {
                    robot->__struri = itatt->second;
                }
            }
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
                throw openrave_exception(str(boost::format(_("KinBody::Init for %s, cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        if( _IsColladaURI(filename) ) {
            if( !RaveParseColladaURI(shared_from_this(), body, filename, atts) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsJSONURI(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsMsgPackURI(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsColladaFile(filename) ) {
            if( !RaveParseColladaFile(shared_from_this(), body, filename, atts) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsJSONFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONFile(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                return KinBodyPtr();
            }
        }
        else if( _IsMsgPackFile(filename) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackFile(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
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
                        body->__struri = fullfilename;
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
                    }
                    else {
                        // failed
                        body.reset();
                    }
                }
                else {
                    // nothing to load
                    body.reset();
                }
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
            //body->__struri = filename;
        }

        // have to set the URI to the passed in one rather than the resolved one, otherwise external components won't be able to compare if a URI is equivalent or not
        if( !!body ) {
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
                throw openrave_exception(str(boost::format(_("KinBody::Init for %s, cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONData(shared_from_this(), body, data, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackData(shared_from_this(), body, data, atts, *_prLoadEnvAlloc) ) {
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
            throw OPENRAVE_EXCEPTION_FORMAT0(_("iv data not supported"),ORE_InvalidArguments);
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

        if( !!body ) {
            // check if have to reset the URI
            FOREACHC(itatt, atts) {
                if( itatt->first == "uri" ) {
                    body->__struri = itatt->second;
                }
            }
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
            RAVELOG_ERROR_FORMAT("ReadInterfaceXMLFile exception: %s", ex.what());
        }
        return InterfaceBasePtr();
    }

    virtual InterfaceBasePtr ReadInterfaceURI(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        bool bIsColladaURI = false;
        bool bIsColladaFile = false;
        bool bIsJSONURI = false;
        bool bIsJSONFile = false;
        bool bIsMsgPackURI = false;
        bool bIsMsgPackFile = false;
        bool bIsXFile = false;
        if( _IsColladaURI(filename) ) {
            bIsColladaURI = true;
        }
        else if( _IsJSONURI(filename) ) {
            bIsJSONURI = true;
        }
        else if( _IsMsgPackURI(filename) ) {
            bIsMsgPackURI = true;
        }
        else if( _IsColladaFile(filename) ) {
            bIsColladaFile = true;
        }
        else if( _IsJSONFile(filename) ) {
            bIsJSONFile = true;
        }
        else if( _IsMsgPackFile(filename) ) {
            bIsMsgPackFile = true;
        }
        else if( _IsXFile(filename) ) {
            bIsXFile = true;
        }

        if( (type == PT_KinBody ||type == PT_Robot ) && (bIsColladaURI||bIsJSONURI||bIsMsgPackURI||bIsColladaFile||bIsJSONFile||bIsMsgPackFile||bIsXFile) ) {
            if( type == PT_KinBody ) {
                BOOST_ASSERT(!pinterface|| (pinterface->GetInterfaceType()==PT_KinBody||pinterface->GetInterfaceType()==PT_Robot));
                KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
                if( bIsColladaURI ) {
                    if( !RaveParseColladaURI(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONURI ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseJSONURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsMsgPackURI ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseMsgPackURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsColladaFile ) {
                    if( !RaveParseColladaFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONFile ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseJSONFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsMsgPackFile ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseMsgPackFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsXFile ) {
                    if( !RaveParseXFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                pinterface = pbody;
            }
            else if( type == PT_Robot ) {
                BOOST_ASSERT(!pinterface||pinterface->GetInterfaceType()==PT_Robot);
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pinterface);
                if( bIsColladaURI ) {
                    if( !RaveParseColladaURI(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONURI ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseJSONURI(shared_from_this(), probot, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsMsgPackURI ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseMsgPackURI(shared_from_this(), probot, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsColladaFile ) {
                    if( !RaveParseColladaFile(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsJSONFile ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseJSONFile(shared_from_this(), probot, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsMsgPackFile ) {
                    _ClearRapidJsonBuffer();
                    if( !RaveParseMsgPackFile(shared_from_this(), probot, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
                else if( bIsXFile ) {
                    if( !RaveParseXFile(shared_from_this(), probot, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
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
        //EnvironmentMutex::scoped_lock lockenv(GetMutex()); // don't lock!
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
        if( !OpenRAVEXMLParser::CreateTriMeshFromFile(shared_from_this(),filedata, vScaleGeometry, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
            ptrimesh.reset();
        }
        return ptrimesh;
    }

    virtual boost::shared_ptr<TriMesh> ReadTrimeshData(boost::shared_ptr<TriMesh> ptrimesh, const std::string& data, const std::string& formathint, const AttributesList& atts)
    {
        RaveVector<float> diffuseColor, ambientColor;
        return _ReadTrimeshData(ptrimesh, data, formathint, diffuseColor, ambientColor, atts);
    }

    virtual boost::shared_ptr<TriMesh> _ReadTrimeshData(boost::shared_ptr<TriMesh> ptrimesh, const std::string& data, const std::string& formathint, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, const AttributesList& atts)
    {
        if( data.size() == 0 ) {
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
        if( !OpenRAVEXMLParser::CreateTriMeshFromData(data, formathint, vScaleGeometry, *ptrimesh, diffuseColor, ambientColor, ftransparency) ) {
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
    virtual OpenRAVE::GraphHandlePtr drawlabel(const std::string& label, const RaveVector<float>& worldPosition)
    {
        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlabel(label, worldPosition));
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
        if (id < _vecWeakBodies.size()) {
            return KinBodyPtr(_vecWeakBodies.at(id));
        }
        return KinBodyPtr();
    }

    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime)
    {
        {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            _bEnableSimulation = true;
            _fDeltaSimTime = fDeltaTime;
            _bRealTime = bRealTime;
            //_nCurSimTime = 0; // don't reset since it is important to keep time monotonic
            _nSimStartTime = utils::GetMicroTime()-_nCurSimTime;
        }
        _StartSimulationThread();
    }

    virtual bool IsSimulationRunning() const {
        return _bEnableSimulation;
    }

    virtual void StopSimulation(int shutdownthread=1)
    {
        {
            EnvironmentMutex::scoped_lock lockenv(GetMutex());
            _bEnableSimulation = false;
            _fDeltaSimTime = 1.0f;
        }
        if( shutdownthread ) {
            _StopSimulationThread();
        }
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            vbodies = _vPublishedBodies;
        }
    }

    virtual bool GetPublishedBody(const std::string &name, KinBody::BodyState& bodystate, uint64_t timeout=0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( _vPublishedBodies[ibody].strname == name) {
                    bodystate = _vPublishedBodies[ibody];
                    return true;
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( _vPublishedBodies[ibody].strname == name) {
                    bodystate = _vPublishedBodies[ibody];
                    return true;
                }
            }
        }

        return false;
    }

    virtual bool GetPublishedBodyJointValues(const std::string& name, std::vector<dReal> &jointValues, uint64_t timeout=0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( _vPublishedBodies[ibody].strname == name) {
                    jointValues = _vPublishedBodies[ibody].jointvalues;
                    return true;
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( _vPublishedBodies[ibody].strname == name) {
                    jointValues = _vPublishedBodies[ibody].jointvalues;
                    return true;
                }
            }
        }

        return false;
    }

    void GetPublishedBodyTransformsMatchingPrefix(const std::string& prefix, std::vector<std::pair<std::string, Transform> >& nameTransfPairs, uint64_t timeout = 0)
    {
        if( timeout == 0 ) {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            nameTransfPairs.resize(0);
            if( nameTransfPairs.capacity() < _vPublishedBodies.size() ) {
                nameTransfPairs.reserve(_vPublishedBodies.size());
            }
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( strncmp(_vPublishedBodies[ibody].strname.c_str(), prefix.c_str(), prefix.size()) == 0 ) {
                    nameTransfPairs.emplace_back(_vPublishedBodies[ibody].strname,  _vPublishedBodies[ibody].vectrans.at(0));
                }
            }
        }
        else {
            boost::timed_mutex::scoped_timed_lock lock(_mutexInterfaces, boost::get_system_time() + boost::posix_time::microseconds(timeout));
            if (!lock.owns_lock()) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }

            nameTransfPairs.resize(0);
            if( nameTransfPairs.capacity() < _vPublishedBodies.size() ) {
                nameTransfPairs.reserve(_vPublishedBodies.size());
            }
            for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
                if ( strncmp(_vPublishedBodies[ibody].strname.c_str(), prefix.c_str(), prefix.size()) == 0 ) {
                    nameTransfPairs.emplace_back(_vPublishedBodies[ibody].strname,  _vPublishedBodies[ibody].vectrans.at(0));
                }
            }
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
                throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
            }
            _UpdatePublishedBodies();
        }
    }

    virtual void _UpdatePublishedBodies()
    {
        // updated the published bodies, resize dynamically in case an exception occurs
        // when creating an item and bad data is left inside _vPublishedBodies
        _vPublishedBodies.resize(_vecbodies.size());
        int iwritten = 0;

        std::vector<dReal> vdoflastsetvalues;
        for(int ibody = 0; ibody < (int)_vecbodies.size(); ++ibody) {
            const KinBodyPtr& pbody = _vecbodies[ibody];
            if( pbody->_nHierarchyComputed != 2 ) {
                // skip
                continue;
            }

            KinBody::BodyState& state = _vPublishedBodies[iwritten];
            state.Reset();
            state.pbody = pbody;
            pbody->GetLinkTransformations(state.vectrans, vdoflastsetvalues);
            pbody->GetLinkEnableStates(state.vLinkEnableStates);
            pbody->GetDOFValues(state.jointvalues);
            pbody->GetGrabbedInfo(state.vGrabbedInfos);
            state.strname =pbody->GetName();
            state.uri = pbody->GetURI();
            state.updatestamp = pbody->GetUpdateStamp();
            state.environmentid = pbody->GetEnvironmentId();
            if( pbody->IsRobot() ) {
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                if( !!probot ) {
                    RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
                    if( !!pmanip ) {
                        state.activeManipulatorName = pmanip->GetName();
                        state.activeManipulatorTransform = pmanip->GetTransform();
                    }
                    probot->GetConnectedBodyActiveStates(state.vConnectedBodyActiveStates);
                }
            }
            ++iwritten;
        }

        if( iwritten < (int)_vPublishedBodies.size() ) {
            _vPublishedBodies.resize(iwritten);
        }
    }

    virtual std::pair<std::string, dReal> GetUnit() const
    {
        return _unit;
    }

    virtual void SetUnit(std::pair<std::string, dReal> unit)
    {
        _unit = unit;
    }

    /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
    virtual void ExtractInfo(EnvironmentBaseInfo& info) override
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::vector<KinBodyPtr> vBodies;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            vBodies = _vecbodies;
        }
        info._vBodyInfos.resize(vBodies.size());
        for(size_t i = 0; i < info._vBodyInfos.size(); ++i) {
            if (vBodies[i]->IsRobot()) {
                info._vBodyInfos[i].reset(new RobotBase::RobotBaseInfo());
                RaveInterfaceCast<RobotBase>(vBodies[i])->ExtractInfo(*(OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(info._vBodyInfos[i])));
            } else {
                info._vBodyInfos[i].reset(new KinBody::KinBodyInfo());
                vBodies[i]->ExtractInfo(*info._vBodyInfos[i]);
            }
        }
        info._name = _name;
        info._keywords = _keywords;
        info._description = _description;
        if (!!_pPhysicsEngine) {
            info._gravity = _pPhysicsEngine->GetGravity();
        }
        info._uInt64Parameters = _mapUInt64Parameters;
    }

    /// \brief update EnvironmentBase according to new EnvironmentBaseInfo, returns false if update cannot be performed and requires InitFromInfo
    void UpdateFromInfo(const EnvironmentBaseInfo& info, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, UpdateFromInfoMode updateMode) override
    {
        RAVELOG_VERBOSE_FORMAT("=== UpdateFromInfo start, env=%d ===", GetId());

        vCreatedBodies.clear();
        vModifiedBodies.clear();
        vRemovedBodies.clear();

        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::vector<dReal> vDOFValues;

        if( updateMode != UFIM_OnlySpecifiedBodiesExact ) {
            // copy basic info into EnvironmentBase
            _revision = info._revision;
            _name = info._name;
            _keywords = info._keywords;
            _description = info._description;
            _mapUInt64Parameters = info._uInt64Parameters;

            // set gravity
            if (!!_pPhysicsEngine) {
                Vector gravityDiff = _pPhysicsEngine->GetGravity() - info._gravity;
                if (OpenRAVE::RaveFabs(gravityDiff.x) > 1e-7 || OpenRAVE::RaveFabs(gravityDiff.y) > 1e-7 || OpenRAVE::RaveFabs(gravityDiff.z) > 1e-7) {
                    _pPhysicsEngine->SetGravity(info._gravity);
                }
            }
        }

        // make a copy of _vecbodies because we will be doing some reordering
        std::vector<KinBodyPtr> vBodies;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            vBodies = _vecbodies;
        }
        std::vector<int> vUsedBodyIndices; // used indices of vBodies

        // internally manipulates _vecbodies using _AddKinBody/_AddRobot/_RemoveKinBodyFromIterator
        for(int inputBodyIndex = 0; inputBodyIndex < (int)info._vBodyInfos.size(); ++inputBodyIndex) {
            const KinBody::KinBodyInfoConstPtr& pKinBodyInfo = info._vBodyInfos[inputBodyIndex];
            const KinBody::KinBodyInfo& kinBodyInfo = *pKinBodyInfo;
            RAVELOG_VERBOSE_FORMAT("==== body: env = %d, id = %s, name = %s ===", GetId()%pKinBodyInfo->_id%pKinBodyInfo->_name);
            RobotBase::RobotBaseInfoConstPtr pRobotBaseInfo = OPENRAVE_DYNAMIC_POINTER_CAST<const RobotBase::RobotBaseInfo>(pKinBodyInfo);
            KinBodyPtr pMatchExistingBody; // matches to pKinBodyInfo
            int bodyIndex = -1; // index to vBodies to use. -1 if not used
            {
                // find existing body in the env
                std::vector<KinBodyPtr>::iterator itExistingSameId = vBodies.end();
                std::vector<KinBodyPtr>::iterator itExistingSameName = vBodies.end();
                std::vector<KinBodyPtr>::iterator itExistingSameIdName = vBodies.end();

                if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                    // can be any of the bodies, but have to make sure not to overlap
                    //bodyIndex = inputBodyIndex;
                    // search only in the unprocessed part of vBodies
                    for(int ibody = 0; ibody < (int)vBodies.size(); ++ibody) {
                        if( find(vUsedBodyIndices.begin(), vUsedBodyIndices.end(), ibody) != vUsedBodyIndices.end() ) {
                            continue;
                        }
                        const KinBodyPtr& pbody = vBodies[ibody];
                        bool bIdMatch = !pbody->_id.empty() && pbody->_id == kinBodyInfo._id;
                        bool bNameMatch = !pbody->_name.empty() && pbody->_name == kinBodyInfo._name;
                        if( bIdMatch && bNameMatch ) {
                            itExistingSameIdName = itExistingSameId = itExistingSameName = vBodies.begin() + ibody;
                            break;
                        }
                        if( bIdMatch && itExistingSameId == vBodies.end() ) {
                            itExistingSameId = vBodies.begin() + ibody;
                        }
                        if( bNameMatch && itExistingSameName == vBodies.end() ) {
                            itExistingSameName = vBodies.begin() + ibody;
                        }
                    }
                }
                else {
                    bodyIndex = inputBodyIndex;
                    // search only in the unprocessed part of vBodies
                    if( (int)vBodies.size() > inputBodyIndex ) {
                        for (std::vector<KinBodyPtr>::iterator itBody = vBodies.begin() + inputBodyIndex; itBody != vBodies.end(); ++itBody) {
                            bool bIdMatch = !(*itBody)->_id.empty() && (*itBody)->_id == kinBodyInfo._id;
                            bool bNameMatch = !(*itBody)->_name.empty() && (*itBody)->_name == kinBodyInfo._name;
                            if( bIdMatch && bNameMatch ) {
                                itExistingSameIdName = itBody;
                                itExistingSameId = itBody;
                                itExistingSameName = itBody;
                                break;
                            }
                            if( bIdMatch && itExistingSameId == vBodies.end() ) {
                                itExistingSameId = itBody;
                            }
                            if( bNameMatch && itExistingSameName == vBodies.end() ) {
                                itExistingSameName = itBody;
                            }
                        }
                    }
                }

                std::vector<KinBodyPtr>::iterator itExisting = itExistingSameIdName;
                if( itExisting == vBodies.end() ) {
                    itExisting = itExistingSameId;
                }
                if( itExisting == vBodies.end() ) {
                    itExisting = itExistingSameName;
                }

                // check if interface type changed, if so, remove the body and treat it as a new body
                if (itExisting != vBodies.end()) {
                    KinBodyPtr pBody = *itExisting;
                    bool bInterfaceMatches = pBody->GetXMLId() == pKinBodyInfo->_interfaceType;
                    if( !bInterfaceMatches || pBody->IsRobot() != pKinBodyInfo->_isRobot ) {
                        RAVELOG_VERBOSE_FORMAT("env=%d, body %s interface is changed, remove old body from environment. xmlid=%s, _interfaceType=%s, isRobot %d != %d", GetId()%pBody->_id%pBody->GetXMLId()%pKinBodyInfo->_interfaceType%pBody->IsRobot()%pKinBodyInfo->_isRobot);
                        itExisting = vBodies.end();
                        vRemovedBodies.push_back(pBody);

                        boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                        vector<KinBodyPtr>::iterator itBodyToRemove = std::find(_vecbodies.begin(), _vecbodies.end(), pBody);
                        if( itBodyToRemove != _vecbodies.end() ) {
                            _RemoveKinBodyFromIterator(itBodyToRemove); // requires _mutexInterfaces lock
                        }
                    }
                }

                if( itExisting != vBodies.end() ) {
                    if( itExisting != itExistingSameName && itExistingSameName != vBodies.end() ) {
                        // new name will conflict with *itExistingSameName, so should change the names to something temporarily
                        // for now, clear since the body should be processed later again
                        (*itExistingSameName)->_name.clear();
                    }
                    pMatchExistingBody = *itExisting;
                    int nMatchingIndex = itExisting-vBodies.begin();
                    if ( bodyIndex >= 0 && bodyIndex != nMatchingIndex) {
                        // re-arrange vBodies according to the order of infos
                        KinBodyPtr pTempBody = vBodies.at(bodyIndex);
                        vBodies.at(bodyIndex) = pMatchExistingBody;
                        *itExisting = pTempBody;
                    }

                    if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                        vUsedBodyIndices.push_back(nMatchingIndex);
                    }
                }
            }

            KinBodyPtr pInitBody; // body that has to be Init() again
            if( !!pMatchExistingBody ) {
                RAVELOG_VERBOSE_FORMAT("env=%d, update existing body %s", GetId()%pMatchExistingBody->_id);
                // interface should match at this point
                // update existing body or robot
                UpdateFromInfoResult updateFromInfoResult = UFIR_NoChange;
                if (pKinBodyInfo->_isRobot && pMatchExistingBody->IsRobot()) {
                    RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(pMatchExistingBody);
                    if( !!pRobotBaseInfo ) {
                        updateFromInfoResult = pRobot->UpdateFromRobotInfo(*pRobotBaseInfo);
                    }
                    else {
                        updateFromInfoResult = pRobot->UpdateFromKinBodyInfo(*pKinBodyInfo);
                    }
                } else {
                    updateFromInfoResult = pMatchExistingBody->UpdateFromKinBodyInfo(*pKinBodyInfo);
                }
                RAVELOG_VERBOSE_FORMAT("env=%d, update body %s from info result %d", GetId()%pMatchExistingBody->_id%updateFromInfoResult);
                if (updateFromInfoResult == UFIR_NoChange) {
                    continue;
                }
                vModifiedBodies.push_back(pMatchExistingBody);
                if (updateFromInfoResult == UFIR_Success) {
                    continue;
                }

                // updating this body requires removing it and re-adding it to env
                {
                    boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                    vector<KinBodyPtr>::iterator itExisting = std::find(_vecbodies.begin(), _vecbodies.end(), pMatchExistingBody);
                    if( itExisting != _vecbodies.end() ) {
                        _RemoveKinBodyFromIterator(itExisting);
                    }
                }

                if (pMatchExistingBody->IsRobot()) {
                    RobotBasePtr pRobot = RaveInterfaceCast<RobotBase>(pMatchExistingBody);
                    if (updateFromInfoResult == UFIR_RequireRemoveFromEnvironment) {
                        // first try udpating again after removing from env
                        if( !!pRobotBaseInfo ) {
                            updateFromInfoResult = pRobot->UpdateFromRobotInfo(*pRobotBaseInfo);
                        }
                        else {
                            updateFromInfoResult = pRobot->UpdateFromKinBodyInfo(*pKinBodyInfo);
                        }
                    }
                    if (updateFromInfoResult != UFIR_NoChange && updateFromInfoResult != UFIR_Success) {
                        // have to reinit
                        if( !!pRobotBaseInfo ) {
                            pRobot->InitFromRobotInfo(*pRobotBaseInfo);
                        }
                        else {
                            pRobot->InitFromKinBodyInfo(*pKinBodyInfo);
                        }
                    }

                    pInitBody = pRobot;
                    _AddRobot(pRobot, IAM_StrictNameChecking); // internally locks _mutexInterfaces, name guarnateed to be unique
                }
                else {
                    if (updateFromInfoResult == UFIR_RequireRemoveFromEnvironment) {
                        // first try udpating again after removing from env
                        updateFromInfoResult = pMatchExistingBody->UpdateFromKinBodyInfo(*pKinBodyInfo);
                    }
                    if (updateFromInfoResult != UFIR_NoChange && updateFromInfoResult != UFIR_Success) {
                        // have to reinit
                        pMatchExistingBody->InitFromKinBodyInfo(*pKinBodyInfo);
                    }

                    pInitBody = pMatchExistingBody;
                    _AddKinBody(pMatchExistingBody, IAM_StrictNameChecking); // internally locks _mutexInterfaces, name guarnateed to be unique
                }
            }
            else {
                // for new body or robot
                KinBodyPtr pNewBody;
                if (pKinBodyInfo->_isRobot) {
                    RAVELOG_VERBOSE_FORMAT("add new robot %s", pKinBodyInfo->_id);
                    RobotBasePtr pRobot = RaveCreateRobot(shared_from_this(), pKinBodyInfo->_interfaceType);
                    if( !pRobot ) {
                        pRobot = RaveCreateRobot(shared_from_this(), "");
                    }

                    if( !!pRobotBaseInfo ) {
                        pRobot->InitFromRobotInfo(*pRobotBaseInfo);
                    }
                    else {
                        pRobot->InitFromKinBodyInfo(*pKinBodyInfo);
                    }
                    pInitBody = pRobot;
                    _AddRobot(pRobot, IAM_AllowRenaming);
                    pNewBody = RaveInterfaceCast<KinBody>(pRobot);
                }
                else {
                    RAVELOG_VERBOSE_FORMAT("add new kinbody %s", pKinBodyInfo->_id);
                    pNewBody = RaveCreateKinBody(shared_from_this(), pKinBodyInfo->_interfaceType);
                    if( !pNewBody ) {
                        pNewBody = RaveCreateKinBody(shared_from_this(), "");
                    }
                    pNewBody->InitFromKinBodyInfo(*pKinBodyInfo);
                    pInitBody = pNewBody;
                    _AddKinBody(pNewBody, IAM_AllowRenaming);
                }

                if( bodyIndex >= 0 ) {
                    if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                        vUsedBodyIndices.push_back(bodyIndex);
                    }
                    vBodies.insert(vBodies.begin()+bodyIndex, pNewBody);
                }
                else {
                    if( updateMode == UFIM_OnlySpecifiedBodiesExact ) {
                        vUsedBodyIndices.push_back(vBodies.size());
                    }
                    vBodies.push_back(pNewBody);
                }
                vCreatedBodies.push_back(pNewBody);
            }

            if (!!pInitBody) {
                // only for init body we need to set name and dofvalues again
                OPENRAVE_ASSERT_OP_FORMAT0(pInitBody->GetName(), ==, pKinBodyInfo->_name, "names should be matching", ORE_InvalidArguments);

                // dof value
                bool bChanged = false;
                pInitBody->GetDOFValues(vDOFValues);
                FOREACH(it, pKinBodyInfo->_dofValues) {
                    FOREACH(itJoint, pInitBody->_vecjoints) {
                        if ((*itJoint)->GetName() == it->first.first) {
                            if( RaveFabs(vDOFValues[(*itJoint)->GetDOFIndex()+it->first.second] - (*it).second) > 1e-10 ) {
                                vDOFValues[(*itJoint)->GetDOFIndex()+it->first.second] = (*it).second;
                                bChanged = true;
                            }
                            break;
                        }
                    }
                }

                if( !bChanged ) {
                    dReal dist = TransformDistanceFast(pInitBody->GetTransform(), pKinBodyInfo->_transform);
                    if( dist > 1e-7 ) {
                        bChanged = true;
                    }
                }

                if( bChanged ) {
                    pInitBody->SetDOFValues(vDOFValues, pKinBodyInfo->_transform, KinBody::CLA_Nothing);
                }
            }
        }

        if( updateMode != UFIM_OnlySpecifiedBodiesExact ) {
            // remove extra bodies at the end of vBodies
            if( vBodies.size() > info._vBodyInfos.size() ) {
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                for (std::vector<KinBodyPtr>::iterator itBody = vBodies.begin() + info._vBodyInfos.size(); itBody != vBodies.end(); ) {
                    KinBodyPtr pBody = *itBody;
                    RAVELOG_VERBOSE_FORMAT("remove extra body env=%d, id=%s, name=%s", GetId()%pBody->_id%pBody->_name);

                    vector<KinBodyPtr>::iterator itBodyToRemove = std::find(_vecbodies.begin(), _vecbodies.end(), pBody);
                    if( itBodyToRemove != _vecbodies.end() ) {
                        _RemoveKinBodyFromIterator(itBodyToRemove); // assumes _mutexInterfaces locked
                    }

                    vRemovedBodies.push_back(pBody);
                    itBody = vBodies.erase(itBody);
                }
            }
        }

        // after all bodies are added, update the grab states
        std::vector<KinBody::GrabbedInfoConstPtr> vGrabbedInfos;
        for(const KinBody::KinBodyInfoPtr& pKinBodyInfo : info._vBodyInfos) {
            const std::string& bodyName = pKinBodyInfo->_name;

            // find existing body in the env, use name since that is more guaranteed to be unique
            std::vector<KinBodyPtr>::iterator itExistingBody = vBodies.end();
            FOREACH(itBody, vBodies) {
                if ((*itBody)->_name == bodyName) {
                    itExistingBody = itBody;
                    break;
                }
            }

            if (itExistingBody != vBodies.end()) {
                // grabbed infos
                vGrabbedInfos.clear();
                vGrabbedInfos.reserve(pKinBodyInfo->_vGrabbedInfos.size());
                FOREACHC(itGrabbedInfo, pKinBodyInfo->_vGrabbedInfos) {
                    if (!!GetKinBody((*itGrabbedInfo)->_grabbedname)) {
                        vGrabbedInfos.push_back(*itGrabbedInfo);
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%d, body %s grabbed by %s is gone, ignoring grabbed info %s", GetId()%(*itGrabbedInfo)->_grabbedname%pKinBodyInfo->_name%(*itGrabbedInfo)->_id);
                    }
                }
                (*itExistingBody)->ResetGrabbed(vGrabbedInfos);
            }
            else {
                RAVELOG_WARN_FORMAT("env=%d, could not find body with name='%s'", GetId()%bodyName);
            }
        }

        UpdatePublishedBodies();
    }

    int GetRevision() const override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _revision;
    }

    void SetName(const std::string& sceneName) override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _name = sceneName;
    }

    std::string GetName() const {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _name;
    }

    void SetDescription(const std::string& sceneDescription) override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _description = sceneDescription;
    }

    std::string GetDescription() const override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _description;
    }

    void SetKeywords(const std::vector<std::string>& sceneKeywords) override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _keywords = sceneKeywords;
    }

    std::vector<std::string> GetKeywords() const override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _keywords;
    }

    void SetUInt64Parameter(const std::string& parameterName, uint64_t value) override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        _mapUInt64Parameters[parameterName] = value;
    }

    bool RemoveUInt64Parameter(const std::string& parameterName) override
    {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        return _mapUInt64Parameters.erase(parameterName) > 0;
    }

    uint64_t GetUInt64Parameter(const std::string& parameterName, uint64_t defaultValue) const override {
        EnvironmentMutex::scoped_lock lockenv(GetMutex());
        std::map<std::string, uint64_t>::const_iterator it = _mapUInt64Parameters.find(parameterName);
        if( it != _mapUInt64Parameters.end() ) {
            return it->second;
        }

        return defaultValue;
    }

protected:

    /// \brief removes a kinbody from _vecbodies
    ///
    /// assumes environment and _mutexInterfaces are locked
    /// \param[in] it the iterator into _vecbodies to erase
    vector<KinBodyPtr>::iterator _RemoveKinBodyFromIterator(vector<KinBodyPtr>::iterator it)
    {
        // before deleting, make sure no robots are grabbing it!!
        FOREACH(itrobot, _vecbodies) {
            KinBody &body = **it;
            if( (*itrobot)->IsGrabbing(body) ) {
                RAVELOG_WARN_FORMAT("env=%d, remove %s already grabbed by robot %s!", GetId()%body.GetName()%(*itrobot)->GetName());
                (*itrobot)->Release(body);
            }
        }

        (*it)->ReleaseAllGrabbed();
        if( (*it)->IsRobot() ) {
            vector<RobotBasePtr>::iterator itrobot = std::find(_vecrobots.begin(), _vecrobots.end(), RaveInterfaceCast<RobotBase>(*it));
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
        (*it)->_PostprocessChangedParameters(KinBody::Prop_BodyRemoved);
        RemoveEnvironmentId(*it);
        vector<KinBodyPtr>::iterator itnew = _vecbodies.erase(it);
        _nBodiesModifiedStamp++;
        return itnew;
    }

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
        _environmentIndexRecyclePool = r->_environmentIndexRecyclePool;
        _bRealTime = r->_bRealTime;

        _name = r->_name;
        _description = r->_description;
        _keywords = r->_keywords;
        _mapUInt64Parameters = r->_mapUInt64Parameters;

        _bInit = true;
        _bEnableSimulation = r->_bEnableSimulation;

        SetDebugLevel(r->GetDebugLevel());

        if( !bCheckSharedResources || !(options & Clone_Bodies) ) {
            {
                // clear internal interface lists
                boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
                // release all grabbed
                FOREACH(itrobot,_vecbodies) {
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
            std::vector<KinBodyWeakPtr> weakBodies;
            {
                boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
                weakBodies = _vecWeakBodies;
                _vecWeakBodies.clear();
            }
            weakBodies.clear();
        }

        list<ViewerBasePtr> listViewers = _listViewers;
        list< pair<ModuleBasePtr, std::string> > listModules = _listModules;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            _listViewers.clear();
            _listModules.clear();
        }

        if( !(options & Clone_Viewer) ) {
            RAVELOG_VERBOSE("resetting raveviewer\n");
            FOREACH(itviewer, listViewers) {
                // don't reset the viewer since it can already be dead
                // todo: this call could lead into a deadlock if a SIGINT got called from the viewer thread
                (*itviewer)->quitmainloop();
            }
            listViewers.clear();
        }

        if( !(options & Clone_Modules) ) {
            listModules.clear();
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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to clone collision checker %s: %s"), r->GetCollisionChecker()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to clone physics engine %s: %s"), r->GetPhysicsEngine()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
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
            _vecWeakBodies.clear();
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
                    const int bodyId = (*itrobot)->GetEnvironmentId();
                    pnewrobot->_environmentid = bodyId;
                    BOOST_ASSERT( _vecWeakBodies.size() < bodyId + 1 || !_vecWeakBodies.at(bodyId).lock());
                    _vecbodies.push_back(pnewrobot);
                    _vecrobots.push_back(pnewrobot);
                    if (_vecWeakBodies.size() < bodyId + 1) {
                        _vecWeakBodies.resize(bodyId + 1, KinBodyWeakPtr());
                    }
                    _vecWeakBodies[bodyId] = pnewrobot;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone robot %s: %s", (*itrobot)->GetName()%ex.what());
                }
            }
            for (const KinBodyPtr& pbody : r->_vecbodies) {
                const KinBody& body = *pbody;
                const int bodyId = body.GetEnvironmentId();
                if( bodyId < _vecWeakBodies.size() && !!_vecWeakBodies.at(bodyId).lock() ) {
                    continue;
                }
                try {
                    KinBodyPtr pnewbody;
                    if( bCheckSharedResources ) {
                        FOREACH(itbody2,vecbodies) {
                            if( !(*itbody2) ) {
                                RAVELOG_WARN_FORMAT("env=%d, a body in vecbodies is not initialized", GetId());
                            }
                            else {
                                if( (*itbody2)->GetName() == body.GetName() && (*itbody2)->GetKinematicsGeometryHash() == body.GetKinematicsGeometryHash() ) {
                                    pnewbody = *itbody2;
                                    break;
                                }
                            }
                        }
                    }
                    if( !pnewbody ) {
                        pnewbody.reset(new KinBody(PT_KinBody,shared_from_this()));
                        pnewbody->_name = body._name; // at least copy the names
                        listToClone.push_back(pbody);
                    }
                    else {
                        listToCopyState.push_back(pbody);
                    }
                    pnewbody->_environmentid = bodyId;
                    _vecbodies.push_back(pnewbody);

                    if (_vecWeakBodies.size() < bodyId + 1) {
                        _vecWeakBodies.resize(bodyId + 1, KinBodyWeakPtr());
                    }
                    _vecWeakBodies[bodyId] = pnewbody;
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("env=%d, failed to clone body %s: %s", GetId()%body.GetName()%ex.what());
                }
            }

            // copy state before cloning
            if( listToCopyState.size() > 0 ) {
                for (const KinBodyPtr& pbody : listToCopyState) {
                    const KinBody& body = *pbody;
                    const int bodyId = body.GetEnvironmentId();
                    KinBodyPtr pnewbody = _vecWeakBodies[bodyId].lock();
                    if( bCollisionCheckerChanged ) {
                        GetCollisionChecker()->InitKinBody(pnewbody);
                    }
                    if( bPhysicsEngineChanged ) {
                        GetPhysicsEngine()->InitKinBody(pnewbody);
                    }
                    pnewbody->__hashkinematics = body.__hashkinematics;
                    if( pnewbody->IsRobot() ) {
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                        // don't clone grabbed bodies!
                        RobotBase::RobotStateSaver saver(poldrobot, 0xffffffff&~KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewrobot);
                        pnewrobot->__hashrobotstructure = poldrobot->__hashrobotstructure;
                    }
                    else {
                        KinBody::KinBodyStateSaver saver(pbody, 0xffffffff&~KinBody::Save_GrabbedBodies);
                        saver.Restore(pnewbody);
                    }
                }
            }

            // now clone
            for (const KinBodyPtr& pbody : listToClone) {
                const KinBody& body = *pbody;
                const int bodyId = body.GetEnvironmentId();
                try {
                    KinBodyPtr pnewbody = _vecWeakBodies[bodyId].lock();
                    if( !!pnewbody ) {
                        pnewbody->Clone(pbody,options);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone body %s: %s", body.GetName()%ex.what());
                }
            }

            for (const KinBodyPtr& pbody : listToClone) {
                const KinBody& body = *pbody;
                const int bodyId = body.GetEnvironmentId();
                KinBodyPtr pnewbody = _vecWeakBodies[bodyId].lock();
                pnewbody->_ComputeInternalInformation();
                GetCollisionChecker()->InitKinBody(pnewbody);
                GetPhysicsEngine()->InitKinBody(pnewbody);
                pnewbody->__hashkinematics = body.__hashkinematics; /// _ComputeInternalInformation resets the hashes
                if( pnewbody->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                    pnewrobot->__hashrobotstructure = poldrobot->__hashrobotstructure;
                }
            }
            // update the state after every body is initialized!
            for (const KinBodyPtr& pbody : listToClone) {
                const KinBody& body = *pbody;
                const int bodyId = body.GetEnvironmentId();
                KinBodyPtr pnewbody = _vecWeakBodies[bodyId].lock();
                if( body.IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_vecWeakBodies[bodyId].lock());
                    // need to also update active dof/active manip since it is erased by _ComputeInternalInformation
                    RobotBase::RobotStateSaver saver(poldrobot, KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities|KinBody::Save_ActiveDOF|KinBody::Save_ActiveManipulator);
                    saver.Restore(pnewrobot);
                }
                else {
                    KinBody::KinBodyStateSaver saver(pbody, KinBody::Save_GrabbedBodies|KinBody::Save_LinkVelocities); // all the others should have been saved?
                    saver.Restore(pnewbody);
                }
            }
            if( listToCopyState.size() > 0 ) {
                // check for re-grabs after cloning is done
                for (const KinBodyPtr& pbody : listToCopyState) {
                    const KinBody& body = *pbody;
                    if( body.IsRobot() ) {
                        const int bodyId = body.GetEnvironmentId();
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_vecWeakBodies[bodyId].lock());
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
                    RAVELOG_ERROR_FORMAT("failed to clone sensor %: %s", (*itsensor)->GetName()%ex.what());
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

        if( options & Clone_Modules ) {
            list< pair<ModuleBasePtr, std::string> > listModules2 = r->_listModules;
            FOREACH(itmodule2, listModules2) {
                try {
                    ModuleBasePtr pmodule;
                    std::string cmdargs;
                    if( bCheckSharedResources ) {
                        FOREACH(itmodule,listModules) {
                            if( itmodule->first->GetXMLId() == itmodule2->first->GetXMLId() ) {
                                pmodule = itmodule->first;
                                cmdargs = itmodule->second;
                                listModules.erase(itmodule);
                                break;
                            }
                        }
                    }
                    if( !pmodule ) {
                        pmodule = RaveCreateModule(shared_from_this(),itmodule2->first->GetXMLId());
                        cmdargs = itmodule2->second;
                    }
                    // add first before cloning!
                    AddModule(pmodule, cmdargs);
                    pmodule->Clone(itmodule2->first, options);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone module %s: %s", itmodule2->first->GetXMLId()%ex.what());
                }
            }
        }

        listModules.clear(); // have to clear the unused modules

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
                    Add(pviewer, IAM_AllowRenaming, std::string());
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone viewer %s: %s", (*itviewer2)->GetName()%ex.what());
                }
            }
        }

        // reset left-over viewers
        FOREACH(itviewer, listViewers) {
            (*itviewer)->quitmainloop();
        }
        listViewers.clear();

        if( !bCheckSharedResources ) {
            if( !!_threadSimulation && _bEnableSimulation ) {
                _StartSimulationThread();
            }
        }
    }

    /// \brief checks if name is unique in _vecbodies
    ///
    /// assuming _mutexInterfaces is locked
    virtual bool _CheckUniqueName(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        FOREACHC(itbody,_vecbodies) {
            if(( *itbody != pbody) &&( (*itbody)->GetName() == pbody->GetName()) ) {
                if( bDoThrow ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, body %s does not have unique name"), GetId()%pbody->GetName(), ORE_BodyNameConflict);
                }
                return false;
            }
        }
        return true;
    }

    /// \brief do not allow empty ids
    virtual bool _CheckUniqueId(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        const std::string& inputBodyId = pbody->GetId();
        if( inputBodyId.empty() ) {
            if( bDoThrow ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, body '%s' does not have a valid id '%s'"), GetId()%pbody->GetName()%inputBodyId, ORE_BodyIdConflict);
            }
            return false;
        }
        FOREACHC(itbody,_vecbodies) {
            if(( *itbody != pbody) &&( (*itbody)->GetId() == inputBodyId) ) {
                if( bDoThrow ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, body '%s' does not have unique id '%s'"), GetId()%pbody->GetName()%pbody->GetId(), ORE_BodyIdConflict);
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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, sensor %s does not have unique name"), GetId()%psensor->GetName(), ORE_SensorNameConflict);
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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, viewer '%s' does not have unique name"), GetId()%pviewer->GetName(), ORE_BodyNameConflict);
                }
                return false;
            }
        }
        return true;
    }

    virtual void SetEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);
        const bool bRecycleId = !_environmentIndexRecyclePool.empty();
        int bodyId = 0;
        if (bRecycleId) {
            bodyId = _environmentIndexRecyclePool.back();
            _environmentIndexRecyclePool.pop_back();
            //RAVELOG_INFO_FORMAT("env=%d, recycled body bodyId=%d for %s", GetId()%bodyId%pbody->GetName());
        }
        else {
            bodyId = _vecWeakBodies.empty() ? 1 : _vecWeakBodies.size(); // no kin body should have an environment bodyId higher than _vecWeakBodies.size() - 1 when _environmentIndexRecyclePool is empty.
            //RAVELOG_INFO_FORMAT("env=%d, assigned new body bodyId=%d for %s", GetId()%bodyId%pbody->GetName());
        }
        BOOST_ASSERT( _vecWeakBodies.size() < bodyId + 1 || !_vecWeakBodies.at(bodyId).lock());
        pbody->_environmentid=bodyId;

        if (_vecWeakBodies.size() < bodyId + 1) {
            _vecWeakBodies.resize(bodyId + 1, KinBodyWeakPtr());
        }
        _vecWeakBodies[bodyId] = pbody;
    }

    virtual void RemoveEnvironmentId(KinBodyPtr pbody)
    {
        boost::mutex::scoped_lock locknetworkid(_mutexEnvironmentIds);

        const int bodyId = pbody->_environmentid;
        if (bodyId == _vecWeakBodies.size() - 1) {
            int numErase = 1; // last element is already decided to be erased
            for (; numErase < _vecWeakBodies.size() - 1; ++numErase) {
                if (!!_vecWeakBodies[_vecWeakBodies.size() - 1 - numErase].lock()) {
                    break;
                }
            }
            _vecWeakBodies.erase(_vecWeakBodies.end() - numErase, _vecWeakBodies.end());
        }
        else {
            _vecWeakBodies.at(pbody->_environmentid) = KinBodyWeakPtr();
        }

        _environmentIndexRecyclePool.push_back(pbody->_environmentid); // for recycle later
        //RAVELOG_INFO_FORMAT("env=%d, removed body id=%d from %s, recycle later", GetId()%pbody->GetName()%pbody->_environmentid);

        pbody->_environmentid = 0;
        pbody->_DeinitializeInternalInformation();
    }

    void _StartSimulationThread()
    {
        if( !_threadSimulation ) {
            _bShutdownSimulation = false;
            _threadSimulation.reset(new boost::thread(boost::bind(&Environment::_SimulationThread, this)));
        }
    }

    void _StopSimulationThread()
    {
        _bShutdownSimulation = true;
        if( !!_threadSimulation ) {
            _threadSimulation->join();
            _threadSimulation.reset();
        }
    }

    void _SimulationThread()
    {
        int environmentid = RaveGetEnvironmentId(shared_from_this());

        uint64_t nLastUpdateTime = utils::GetMicroTime();
        uint64_t nLastSleptTime = utils::GetMicroTime();
        RAVELOG_VERBOSE_FORMAT("starting simulation thread envid=%d", environmentid);
        while( _bInit && !_bShutdownSimulation ) {
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

    /// _mutexInterfaces should not be locked
    void _CallBodyCallbacks(KinBodyPtr pbody, int action)
    {
        std::list<UserDataWeakPtr> listRegisteredBodyCallbacks;
        {
            boost::timed_mutex::scoped_lock lock(_mutexInterfaces);
            listRegisteredBodyCallbacks = _listRegisteredBodyCallbacks;
        }
        FOREACH(it, listRegisteredBodyCallbacks) {
            BodyCallbackDataPtr pdata = boost::dynamic_pointer_cast<BodyCallbackData>(it->lock());
            if( !!pdata ) {
                pdata->_callback(pbody, action);
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

    static bool _IsColladaURI(const std::string& uri)
    {
        string scheme, authority, path, query, fragment;
        string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);
        return bmatch && scheme.size() > 0 && _IsColladaFile(path); //scheme.size() > 0;
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

    static bool _IsJSONURI(const std::string& uri)
    {
        string scheme, authority, path, query, fragment;
        string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);
        return bmatch && scheme.size() > 0 && _IsJSONFile(path); //scheme.size() > 0;
    }

    static bool _IsJSONFile(const std::string& filename)
    {
        size_t len = filename.size();
        if( len < 5 ) {
            return false;
        }
        if( filename[len-5] == '.' && ::tolower(filename[len-4]) == 'j' && ::tolower(filename[len-3]) == 's' && ::tolower(filename[len-2]) == 'o' && ::tolower(filename[len-1]) == 'n' ) {
            return true;
        }
        return false;
    }

    static bool _IsJSONData(const std::string& data)
    {
        return data.size() >= 2 && data[0] == '{';
    }

    static bool _IsMsgPackURI(const std::string& uri)
    {
        string scheme, authority, path, query, fragment;
        string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);
        return bmatch && scheme.size() > 0 && _IsMsgPackFile(path); //scheme.size() > 0;
    }

    static bool _IsMsgPackFile(const std::string& filename)
    {
        // .msgpack
        size_t len = filename.size();
        if( len < 8 ) {
            return false;
        }
        if( filename[len-8] == '.' && ::tolower(filename[len-7]) == 'm' && ::tolower(filename[len-6]) == 's' && ::tolower(filename[len-5]) == 'g' && ::tolower(filename[len-4]) == 'p' && ::tolower(filename[len-3]) == 'a' && ::tolower(filename[len-2]) == 'c' && ::tolower(filename[len-1]) == 'k' ) {
            return true;
        }
        return false;
    }

    static bool _IsMsgPackData(const std::string& data)
    {
        return data.size() > 0 && !std::isprint(data[0]);
    }

    void _ClearRapidJsonBuffer()
    {
        // TODO resize smartly
        _prLoadEnvAlloc->Clear();
    }

    std::vector<RobotBasePtr> _vecrobots;      ///< robots (possibly controlled). protected by _mutexInterfaces
    std::vector<KinBodyPtr> _vecbodies;     ///< all objects that are collidable (includes robots). protected by _mutexInterfaces

    list< std::pair<ModuleBasePtr, std::string> > _listModules;     ///< modules loaded in the environment and the strings they were intialized with. Initialization strings are used for cloning.
    list<SensorBasePtr> _listSensors;     ///< sensors loaded in the environment
    list<ViewerBasePtr> _listViewers;     ///< viewers loaded in the environment

    dReal _fDeltaSimTime;                    ///< delta time for simulate step
    uint64_t _nCurSimTime;                        ///< simulation time since the start of the environment
    uint64_t _nSimStartTime;
    int _nBodiesModifiedStamp;     ///< incremented every tiem bodies vector is modified

    CollisionCheckerBasePtr _pCurrentChecker;
    PhysicsEngineBasePtr _pPhysicsEngine;

    std::vector<KinBodyWeakPtr> _vecWeakBodies;     ///< a vector of all the bodies in the environment. Controlled through the KinBody constructor and destructors
    std::vector<int> _environmentIndexRecyclePool; ///< body indices which can be reused later, because kin bodies who had these id's previously are already removed from the environment. This is to prevent env id's from growing without bound when kin bodies are removed and added repeatedly. 

    boost::shared_ptr<boost::thread> _threadSimulation;                      ///< main loop for environment simulation

    mutable EnvironmentMutex _mutexEnvironment;          ///< protects internal data from multithreading issues
    mutable boost::mutex _mutexEnvironmentIds;      ///< protects _vecbodies/_vecrobots from multithreading issues
    mutable boost::timed_mutex _mutexInterfaces;     ///< lock when managing interfaces like _listOwnedInterfaces, _listModules, _vecWeakBodies
    mutable boost::mutex _mutexInit;     ///< lock for destroying the environment

    vector<KinBody::BodyState> _vPublishedBodies;
    string _homedirectory;
    std::pair<std::string, dReal> _unit; ///< unit name mm, cm, inches, m and the conversion for meters

    UserDataPtr _handlegenericrobot, _handlegenerictrajectory, _handlemulticontroller, _handlegenericphysicsengine, _handlegenericcollisionchecker;

    list<InterfaceBasePtr> _listOwnedInterfaces;

    std::list<UserDataWeakPtr> _listRegisteredCollisionCallbacks;     ///< see EnvironmentBase::RegisterCollisionCallback
    std::list<UserDataWeakPtr> _listRegisteredBodyCallbacks;     ///< see EnvironmentBase::RegisterBodyCallback

    std::map<std::string, uint64_t> _mapUInt64Parameters; ///< a custom user-driven parameters
    std::vector<uint8_t> _vRapidJsonLoadBuffer;
    boost::shared_ptr<rapidjson::MemoryPoolAllocator<> > _prLoadEnvAlloc; ///< allocator used for loading environments

    bool _bInit;                   ///< environment is initialized
    bool _bEnableSimulation;            ///< enable simulation loop
    bool _bShutdownSimulation; ///< if true, the simulation thread should shutdown
    bool _bRealTime;

    friend class EnvironmentXMLReader;
};

#endif
