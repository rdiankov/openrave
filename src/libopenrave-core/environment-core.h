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
#include "stringutils.h"

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

#include <chrono>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <unordered_map>

#include <pcrecpp.h>

#define CHECK_INTERFACE(pinterface) { \
        if( (pinterface)->GetEnv() != shared_from_this() ) { \
            throw openrave_exception(str(boost::format(_("env=%s, Interface %s:%s is from a different environment (env=%s) than the current one."))%GetNameId()%RaveGetInterfaceName((pinterface)->GetInterfaceType())%(pinterface)->GetXMLId()%(pinterface)->GetEnv()->GetNameId()),ORE_InvalidArguments); \
        } \
} \

#define CHECK_COLLISION_BODY(body) { \
        CHECK_INTERFACE(body); \
}

// can be used with string_view. unfortunately unordered_map::find cannot be used with heterogeneous types yet (potentially in c++20 standard)
template <typename Value>
using string_map = std::map<std::string, Value, std::less<> >;

inline dReal TransformDistanceFast(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*e);
}

/// \brief ensures vector size is at least size
template <typename T>
inline void EnsureVectorSize(std::vector<T>& vec, size_t size)
{
    if (vec.size() < size) {
        //RAVELOG_WARN_FORMAT("resizing 0x%x from %d to %d", &vec%(vec.size())%(index + 1));
        vec.resize(size);
    }
}

class TimedUniqueLock : public std::unique_lock<std::timed_mutex> {
public:
    /**
     * Try to lock a mutex for a given duration if the duration is a positive value. Otherwise it waits for the lock without a timeout.
     */
    TimedUniqueLock(std::timed_mutex& mutex, int64_t timeoutus)
        : std::unique_lock<std::timed_mutex>(mutex, std::defer_lock) {
        if( timeoutus > 0 ) {
            this->try_lock_for(std::chrono::microseconds(timeoutus));
        }
        else {
            this->lock();
        }
    }
};

class TimedSharedLock : public std::shared_lock<std::shared_timed_mutex> {
public:
    /**
     * Try to lock a mutex for a given duration if the duration is a positive value. Otherwise it waits for the lock without a timeout.
     */
    TimedSharedLock(std::shared_timed_mutex& mutex, int64_t timeoutus)
        : std::shared_lock<std::shared_timed_mutex>(mutex, std::defer_lock) {
        if( timeoutus > 0 ) {
            this->try_lock_for(std::chrono::microseconds(timeoutus));
        }
        else {
            this->lock();
        }
    }
};

class TimedExclusiveLock {
public:
    /**
     * Try to exclusively lock a mutex for a given duration if the duration is a positive value. Otherwise it waits for the lock without a timeout.
     */
    TimedExclusiveLock(std::shared_timed_mutex& mutex, int64_t timeoutus)
        : _mutex(mutex), _lockAquired(false)
    {
        if( timeoutus > 0 ) {
            _lockAquired = _mutex.try_lock_for(std::chrono::microseconds(timeoutus));
        }
        else {
            _mutex.lock();
            _lockAquired = true;
        }
    }

    /// \brief check if lock is succesfully aquired.
    /// for consistency with TimedSharedLock
    inline bool operator!() const {
        return !_lockAquired;
    }

    /// \brief unlocks mutex if preiously locked
    ~TimedExclusiveLock()
    {
        if (_lockAquired) {
            _mutex.unlock();
            _lockAquired = false;
        }
    }

    // prohibit copying
    TimedExclusiveLock& operator=(const TimedExclusiveLock&) = delete;
    TimedExclusiveLock(const TimedExclusiveLock&) = delete;

protected:
    std::shared_timed_mutex& _mutex;
    bool _lockAquired;
};

/// \biref Helper class to save and restore the kinbody id
class KinBodyIdSaver
{
public:
    KinBodyIdSaver(KinBodyPtr pbody) : _pbody(pbody) {
        if( !!_pbody ) {
            _id = pbody->GetId();
        }
    }
    virtual ~KinBodyIdSaver() {
        if ( !!_pbody ) {
            _pbody->SetId(_id);
        }
    }

protected:
    KinBodyPtr _pbody; ///< pointer to kinbody
    std::string _id; ///< original body id
};
typedef boost::shared_ptr<KinBodyIdSaver> KinBodyIdSaverPtr;

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
                ExclusiveLock lock219(penv->_mutexInterfaces);
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
                ExclusiveLock lock442(penv->_mutexInterfaces);
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
        _Init();
    }

    Environment(const std::string& name) : EnvironmentBase(name)
    {
        _Init();
    }

    virtual ~Environment()
    {
        Destroy();
    }

    void Init(bool bStartSimulationThread) override
    {
        std::lock_guard<std::mutex> lockinit(_mutexInit);
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
            RAVELOG_DEBUG("using '%s' collision checker\n", localchecker->GetXMLId().c_str());
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

    virtual void Destroy() override
    {
        std::lock_guard<std::mutex> lockdestroy(_mutexInit);
        if( !_bInit ) {
            RAVELOG_VERBOSE_FORMAT("env=%s is already destroyed", GetNameId());
            return;
        }

        // destruction order is *very* important, don't touch it without consultation
        _bInit = false;

        RAVELOG_VERBOSE_FORMAT("env=%s destructor, _vecbodies.size():%d", GetNameId()%_vecbodies.size());
        if (_vecbodies.size() > 10000 || _mapBodyNameIndex.size() > 10000 || _mapBodyIdIndex.size() > 10000) { // don't know good threshold
            RAVELOG_WARN_FORMAT("env=%s, _vecbodies.size():%d, _mapBodyNameIndex.size():%d, _mapBodyIdIndex.size():%d seems large, maybe there is memory leak", GetNameId()%_vecbodies.size()%_mapBodyNameIndex.size()%_mapBodyIdIndex.size());
        }
        _StopSimulationThread();

        // destroy the modules (their destructors could attempt to lock environment, so have to do it before global lock)
        // however, do not clear the _listModules yet
        RAVELOG_DEBUG_FORMAT("env=%s destroy module", GetNameId());
        list< pair<ModuleBasePtr, std::string> > listModules;
        list<ViewerBasePtr> listViewers = _listViewers;
        {
            SharedLock lock814(_mutexInterfaces);
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
            RAVELOG_DEBUG_FORMAT("env=%s, quitting viewer '%s'", GetNameId()%(*itviewer)->GetXMLId());
            (*itviewer)->quitmainloop();
        }
        listViewers.clear();

        // lock the environment
        {
            EnvironmentLock lockenv(GetMutex());
            _bEnableSimulation = false;
            if( !!_pPhysicsEngine ) {
                _pPhysicsEngine->DestroyEnvironment();
            }
            if( !!_pCurrentChecker ) {
                _pCurrentChecker->DestroyEnvironment();
            }

            // clear internal interface lists, have to Destroy all kinbodys without locking _mutexInterfaces since some can hold BodyCallbackData, which requires to lock _mutexInterfaces
            std::vector<KinBodyPtr> vecbodies;
            list<SensorBasePtr> listSensors;
            {
                ExclusiveLock lock874(_mutexInterfaces);
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
            for (KinBodyPtr& pbody : vecbodies) {
                if (!!pbody) {
                    pbody->ReleaseAllGrabbed();
                }
            }
            for (KinBodyPtr& pbody : vecbodies) {
                if (!!pbody) {
                    pbody->Destroy();
                }
            }
            vecbodies.clear();

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

    void Reset() override
    {
        RAVELOG_DEBUG_FORMAT("env=%s, resetting", GetNameId());
        // destruction order is *very* important, don't touch it without consultation
        list<ViewerBasePtr> listViewers;
        GetViewers(listViewers);
        if( listViewers.size() > 0 ) {
            RAVELOG_DEBUG("resetting raveviewer\n");
            FOREACH(itviewer, listViewers) {
                (*itviewer)->Reset();
            }
        }

        EnvironmentLock lockenv(GetMutex());

        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->DestroyEnvironment();
        }

        std::vector<KinBodyPtr> vcallbackbodies;
        {
            ExclusiveLock lock348(_mutexInterfaces);

            for (KinBodyPtr& pbody : _vecbodies) {
                if (!pbody) {
                    continue;
                }
                pbody->_environmentBodyIndex=0;
                pbody->Destroy();
            }
            if( _listRegisteredBodyCallbacks.size() > 0 ) {
                for (KinBodyPtr& pbody : _vecbodies) {
                    if (!!pbody) {
                        vcallbackbodies.push_back(pbody);
                    }
                }
            }
            _vecbodies.clear();

            _mapBodyNameIndex.clear();
            _mapBodyIdIndex.clear();

            _vPublishedBodies.clear();
            _nBodiesModifiedStamp++;

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
        list<InterfaceBasePtr> listOwnedInterfaces;
        {
            SharedLock lock385(_mutexInterfaces);
            listModules = _listModules;
            listOwnedInterfaces.swap(_listOwnedInterfaces);
        }

        FOREACH(itmodule,listModules) {
            itmodule->first->Reset();
        }
        listModules.clear();
        listOwnedInterfaces.clear();

        if( !!_pCurrentChecker ) {
            _pCurrentChecker->InitEnvironment();
        }
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->InitEnvironment();
        }
    }

    virtual UserDataPtr GlobalState() override {
        return RaveGlobalState();
    }

    virtual void OwnInterface(InterfaceBasePtr pinterface) override
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentLock lockenv(GetMutex());
        ExclusiveLock lock473(_mutexInterfaces);
        _listOwnedInterfaces.push_back(pinterface);
    }
    virtual void DisownInterface(InterfaceBasePtr pinterface) override
    {
        CHECK_INTERFACE(pinterface);
        EnvironmentLock lockenv(GetMutex());
        ExclusiveLock lock277(_mutexInterfaces);
        _listOwnedInterfaces.remove(pinterface);
    }

    EnvironmentBasePtr CloneSelf(int options) override
    {
        EnvironmentLock lockenv(GetMutex());
        boost::shared_ptr<Environment> penv(new Environment());
        penv->_Clone(boost::static_pointer_cast<Environment const>(shared_from_this()),options,false);
        return penv;
    }

    EnvironmentBasePtr CloneSelf(const std::string& clonedEnvName, int options) override
    {
        EnvironmentLock lockenv(GetMutex());
        boost::shared_ptr<Environment> penv(new Environment(clonedEnvName));
        penv->_Clone(boost::static_pointer_cast<Environment const>(shared_from_this()),options,false);
        return penv;
    }

    void Clone(EnvironmentBaseConstPtr preference, int cloningoptions) override
    {
        EnvironmentLock lockenv(GetMutex());
        _Clone(boost::static_pointer_cast<Environment const>(preference),cloningoptions,true);
    }

    void Clone(EnvironmentBaseConstPtr preference, const std::string& clonedEnvName, int cloningoptions) override
    {
        EnvironmentLock lockenv(GetMutex());
        _Clone(boost::static_pointer_cast<Environment const>(preference), cloningoptions,true);
        _name = clonedEnvName;
        if (_name.empty()) {
            _formatedNameId = str(boost::format("%d")%GetId());
        }
        else {
            _formatedNameId = str(boost::format("%d(%s)")%GetId()%_name);
        }
    }

    virtual int AddModule(ModuleBasePtr module, const std::string& cmdargs)
    {
        CHECK_INTERFACE(module);
        int ret = module->main(cmdargs);
        if( ret != 0 ) {
            RAVELOG_WARN_FORMAT("Error %d with executing module '%s'", ret%module->GetXMLId());
        }
        else {
            EnvironmentLock lockenv(GetMutex());
            ExclusiveLock lock668(_mutexInterfaces);
            _listModules.emplace_back(module,  cmdargs);
        }

        return ret;
    }

    void GetModules(std::list<ModuleBasePtr>& listModules, uint64_t timeout) const override
    {
        TimedSharedLock lock145(_mutexInterfaces, timeout);
        if (!lock145) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        listModules.clear();
        FOREACHC(it, _listModules) {
            listModules.push_back(it->first);
        }
    }

    bool LoadURI(const std::string& uri, const AttributesList& atts) override
    {
        std::string path;
        if (!_IsURI(uri, path)) {
            RAVELOG_WARN("Failed to load URI: '%s' is not a URI.", uri.c_str());
            return false;
        }
        if (_IsColladaFile(path)) {
            return RaveParseColladaURI(shared_from_this(), uri, atts);
        }
        else if (_IsJSONFile(path)) {
            _ClearRapidJsonBuffer();
            return RaveParseJSONURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else if (_IsMsgPackFile(path)) {
            _ClearRapidJsonBuffer();
            return RaveParseMsgPackURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else if (StringEndsWith(path, ".json.gpg")) {
            _ClearRapidJsonBuffer();
            return RaveParseEncryptedJSONURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else if (StringEndsWith(path, ".msgpack.gpg")) {
            _ClearRapidJsonBuffer();
            return RaveParseEncryptedMsgPackURI(shared_from_this(), uri, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        else {
            RAVELOG_WARN_FORMAT("load failed on uri '%s' since could not determine the file type", uri);
        }
        return false;
    }

    virtual bool Load(const std::string& filename, const AttributesList& atts) override
    {
        EnvironmentLock lockenv(GetMutex());
        OpenRAVEXMLParser::GetXMLErrorCount() = 0;
        std::string path;
        if (_IsURI(filename, path)) {
            if (_IsColladaFile(path)) {
                if( RaveParseColladaURI(shared_from_this(), filename, atts) ) {
                    UpdatePublishedBodies();
                    return true;
                }
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
        else if (StringEndsWith(filename, ".json.gpg")) {
            _ClearRapidJsonBuffer();
            if( RaveParseEncryptedJSONFile(shared_from_this(), filename, UFIM_Exact, atts, *_prLoadEnvAlloc) ) {
                return true;
            }
        }
        else if (StringEndsWith(filename, ".msgpack.gpg")) {
            _ClearRapidJsonBuffer();
            if( RaveParseEncryptedMsgPackFile(shared_from_this(), filename, UFIM_Exact, atts, *_prLoadEnvAlloc) ) {
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

        RAVELOG_WARN("load failed on file '%s'\n", filename.c_str());
        return false;
    }

    virtual bool LoadData(const std::string& data, const AttributesList& atts, const std::string& uri)
    {
        EnvironmentLock lockenv(GetMutex());
        if( _IsColladaData(data) ) {
            return RaveParseColladaData(shared_from_this(), data, atts);
        }
        if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            return RaveParseJSONData(shared_from_this(), uri, data, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            return RaveParseMsgPackData(shared_from_this(), uri, data, UFIM_Exact, atts, *_prLoadEnvAlloc);
        }
        return _ParseXMLData(OpenRAVEXMLParser::CreateEnvironmentReader(shared_from_this(),atts),data);
    }

    bool LoadJSON(const rapidjson::Value& rEnvInfo, UpdateFromInfoMode updateMode, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, const AttributesList& atts, const std::string &uri) override
    {
        EnvironmentLock lockenv(GetMutex());
        _ClearRapidJsonBuffer();
        return RaveParseJSON(shared_from_this(), uri, rEnvInfo, updateMode, vCreatedBodies, vModifiedBodies, vRemovedBodies, atts, *_prLoadEnvAlloc);
    }

    virtual void Save(const std::string& filename, SelectionOptions options, const AttributesList& atts) override
    {
        EnvironmentLock lockenv(GetMutex());
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
            else if (StringEndsWith(filename, ".json.gpg")) {
                _ClearRapidJsonBuffer();
                RaveWriteEncryptedJSONFile(shared_from_this(), filename, atts, *_prLoadEnvAlloc);
            }
            else if (StringEndsWith(filename, ".msgpack.gpg")) {
                _ClearRapidJsonBuffer();
                RaveWriteEncryptedMsgPackFile(shared_from_this(), filename, atts, *_prLoadEnvAlloc);
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
                        RAVELOG_WARN_FORMAT("failed to get body '%s'", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && !pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_Robots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && find(listignore.begin(), listignore.end(), pbody->GetName()) == listignore.end() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        }
        }

        if( _IsJSONFile(filename) ) {
            _ClearRapidJsonBuffer();
            RaveWriteJSONFile(listbodies,filename,atts,*_prLoadEnvAlloc);
        }
        else if( _IsMsgPackFile(filename) ) {
            _ClearRapidJsonBuffer();
            RaveWriteMsgPackFile(listbodies,filename,atts,*_prLoadEnvAlloc);
        }
        else if( StringEndsWith(filename, ".json.gpg") ) {
            _ClearRapidJsonBuffer();
            RaveWriteEncryptedJSONFile(listbodies,filename,atts,*_prLoadEnvAlloc);
        }
        else if( StringEndsWith(filename, ".msgpack.gpg") ) {
            _ClearRapidJsonBuffer();
            RaveWriteEncryptedMsgPackFile(listbodies,filename,atts,*_prLoadEnvAlloc);
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

    virtual void SerializeJSON(rapidjson::Value& rEnvironment, rapidjson::Document::AllocatorType& allocator, SelectionOptions options, const AttributesList& atts) override
    {
        EnvironmentLock lockenv(GetMutex());
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
                        RAVELOG_WARN_FORMAT("failed to get body '%s'", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && !pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_Robots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && find(listignore.begin(), listignore.end(), pbody->GetName()) == listignore.end() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        }
        }

        RaveWriteJSON(listbodies, rEnvironment, allocator, atts);
    }

    virtual void WriteToMemory(const std::string& filetype, std::vector<char>& output, SelectionOptions options=SO_Everything, const AttributesList& atts = AttributesList()) override
    {
        if (filetype != "collada" && filetype != "json" && filetype != "msgpack") {
            throw OPENRAVE_EXCEPTION_FORMAT("got invalid filetype '%s', only support collada and json", filetype, ORE_InvalidArguments);
        }

        EnvironmentLock lockenv(GetMutex());
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
            else if (filetype == "json.gpg") {
                _ClearRapidJsonBuffer();
                RaveWriteEncryptedJSONMemory(shared_from_this(), output, atts,*_prLoadEnvAlloc);
            }
            else if (filetype == "msgpack.gpg") {
                _ClearRapidJsonBuffer();
                RaveWriteEncryptedMsgPackMemory(shared_from_this(), output, atts,*_prLoadEnvAlloc);
            }
            return;

        case SO_Body: {
            std::string targetname;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    KinBodyPtr pbody = GetKinBody(itatt->second);
                    if( !pbody ) {
                        RAVELOG_WARN_FORMAT("failed to get body '%s'", itatt->second);
                    }
                    else {
                        listbodies.push_back(pbody);
                    }
                }
            }
            break;
        }
        case SO_NoRobots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && !pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_Robots:
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && pbody->IsRobot() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        case SO_AllExceptBody: {
            std::list<std::string> listignore;
            FOREACHC(itatt,atts) {
                if( itatt->first == "target" ) {
                    listignore.push_back(itatt->second);
                }
            }
            for (KinBodyPtr& pbody : _vecbodies) {
                if( !!pbody && find(listignore.begin(), listignore.end(), pbody->GetName()) == listignore.end() ) {
                    listbodies.push_back(pbody);
                }
            }
            break;
        }
        }

        if (filetype == "collada") {
            if( listbodies.size() == 1 ) {
                RaveWriteColladaMemory(listbodies.front(), output, atts);
            }
            else {
                RaveWriteColladaMemory(listbodies, output, atts);
            }
        }
        else if (filetype == "json") {
            _ClearRapidJsonBuffer();
            RaveWriteJSONMemory(listbodies, output, atts,*_prLoadEnvAlloc);
        }
        else if (filetype == "msgpack") {
            _ClearRapidJsonBuffer();
            RaveWriteMsgPackMemory(listbodies, output, atts,*_prLoadEnvAlloc);
        }
        else if (filetype == "json.gpg") {
            _ClearRapidJsonBuffer();
            RaveWriteEncryptedJSONMemory(listbodies, output, atts,*_prLoadEnvAlloc);
        }
        else if (filetype == "msgpack.gpg") {
            _ClearRapidJsonBuffer();
            RaveWriteEncryptedMsgPackMemory(listbodies, output, atts,*_prLoadEnvAlloc);
        }
    }

    virtual void Add(InterfaceBasePtr pinterface, InterfaceAddMode addMode, const std::string& cmdargs) override
    {
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_Robot: _AddRobot(RaveInterfaceCast<RobotBase>(pinterface), addMode); break;
        case PT_KinBody: _AddKinBody(RaveInterfaceCast<KinBody>(pinterface), addMode); break;
        case PT_Module: {
            int ret = AddModule(RaveInterfaceCast<ModuleBase>(pinterface), cmdargs);
            OPENRAVE_ASSERT_OP_FORMAT(ret,==,0,"module '%s' failed with args: '%s'",pinterface->GetXMLId()%cmdargs,ORE_InvalidArguments);
            break;
        }
        case PT_Viewer: _AddViewer(RaveInterfaceCast<ViewerBase>(pinterface)); break;
        case PT_Sensor: _AddSensor(RaveInterfaceCast<SensorBase>(pinterface),addMode); break;
        default:
            throw OPENRAVE_EXCEPTION_FORMAT(_("Interface %d cannot be added to the environment"),pinterface->GetInterfaceType(),ORE_InvalidArguments);
        }
    }

    virtual void AddKinBody(KinBodyPtr pbody, InterfaceAddMode addMode, int requestedEnvironmentBodyIndex) override
    {
        _AddKinBody(pbody, addMode, requestedEnvironmentBodyIndex);
    }

    virtual void AddRobot(RobotBasePtr probot, InterfaceAddMode addMode, int requestedEnvironmentBodyIndex) override
    {
        _AddRobot(probot, addMode, requestedEnvironmentBodyIndex);
    }

    virtual void _AddKinBody(KinBodyPtr pbody, InterfaceAddMode addMode, int requestedEnvironmentBodyIndex = 0)
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_INTERFACE(pbody);
        if( !utils::IsValidName(pbody->GetName()) ) {
            if( addMode & IAM_StrictNameChecking ) {
                throw openrave_exception(str(boost::format(_("Body name: '%s' is not valid"))%pbody->GetName()));
            }
            else {
                pbody->SetName(utils::ConvertToOpenRAVEName(pbody->GetName()));
            }
        }

        if( !_CheckUniqueName(KinBodyConstPtr(pbody), !!(addMode & IAM_StrictNameChecking)) ) {
            _EnsureUniqueName(pbody);
        }
        if( !_CheckUniqueId(KinBodyConstPtr(pbody), !!(addMode & IAM_StrictIdChecking)) ) {
            _EnsureUniqueId(pbody);
        }
        {
            ExclusiveLock lock969(_mutexInterfaces);
            const int newBodyIndex = _AssignEnvironmentBodyIndex(pbody, requestedEnvironmentBodyIndex);
            _AddKinBodyInternal(pbody, newBodyIndex);
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
        const uint32_t maskPotentialyChanged(0xffffffff&~KinBody::Prop_JointMimic& ~KinBody::Prop_LinkStatic& ~KinBody::Prop_BodyRemoved& ~KinBody::Prop_LinkGeometry& ~KinBody::Prop_LinkGeometryGroup& ~KinBody::Prop_LinkDynamics);
        pbody->_PostprocessChangedParameters(maskPotentialyChanged);
        _CallBodyCallbacks(pbody, 1);
    }

    virtual void _AddRobot(RobotBasePtr robot, InterfaceAddMode addMode, int requestedEnvironmentBodyIndex = 0)
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_INTERFACE(robot);
        if( !robot->IsRobot() ) {
            throw openrave_exception(str(boost::format(_("kinbody '%s' is not a robot"))%robot->GetName()));
        }
        if( !utils::IsValidName(robot->GetName()) ) {
            if( addMode & IAM_StrictNameChecking ) {
                throw openrave_exception(str(boost::format(_("Robot name: '%s' is not valid"))%robot->GetName()));
            }
            else {
                robot->SetName(utils::ConvertToOpenRAVEName(robot->GetName()));
            }
        }

        if( !_CheckUniqueName(KinBodyConstPtr(robot), !!(addMode & IAM_StrictNameChecking)) ) {
            _EnsureUniqueName(robot);
        }
        if( !_CheckUniqueId(KinBodyConstPtr(robot), !!(addMode & IAM_StrictIdChecking)) ) {
            _EnsureUniqueId(robot);
        }
        {
            ExclusiveLock lock823(_mutexInterfaces);
            const int newBodyIndex = _AssignEnvironmentBodyIndex(robot, requestedEnvironmentBodyIndex);
            _AddKinBodyInternal(robot, newBodyIndex);
            _nBodiesModifiedStamp++;
        }
        robot->_ComputeInternalInformation(); // have to do this after _vecbodies is added since SensorBase::SetName can call EnvironmentBase::GetSensor to initialize itself
        _pCurrentChecker->InitKinBody(robot);
        if( !!robot->GetSelfCollisionChecker() && robot->GetSelfCollisionChecker() != _pCurrentChecker ) {
            // also initialize external collision checker if specified for this body
            robot->GetSelfCollisionChecker()->InitKinBody(robot);
        }
        _pPhysicsEngine->InitKinBody(robot);
        // send all the changed callbacks of the body since anything could have changed
        const uint32_t maskPotentialyChanged(0xffffffff&~KinBody::Prop_JointMimic& ~KinBody::Prop_LinkStatic& ~KinBody::Prop_BodyRemoved& ~KinBody::Prop_LinkGeometry& ~KinBody::Prop_LinkGeometryGroup& ~KinBody::Prop_LinkDynamics);
        robot->_PostprocessChangedParameters(maskPotentialyChanged);
        _CallBodyCallbacks(robot, 1);
    }

    virtual void _AddSensor(SensorBasePtr psensor, InterfaceAddMode addMode)
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_INTERFACE(psensor);
        if( !utils::IsValidName(psensor->GetName()) ) {
            if( addMode & IAM_StrictNameChecking ) {
                throw openrave_exception(str(boost::format(_("Sensor name: '%s' is not valid"))%psensor->GetName()));
            }
            else {
                psensor->SetName(utils::ConvertToOpenRAVEName(psensor->GetName()));
            }
        }

        // no id for sensor right now
        if( !_CheckUniqueName(SensorBaseConstPtr(psensor), !!(addMode & IAM_StrictNameChecking)) ) {
            _EnsureUniqueName(psensor);
        }
        {
            ExclusiveLock lock960(_mutexInterfaces);
            _listSensors.push_back(psensor);
        }
        psensor->Configure(SensorBase::CC_PowerOn);
    }

    virtual bool Remove(InterfaceBasePtr pinterface) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_INTERFACE(pinterface);
        switch(pinterface->GetInterfaceType()) {
        case PT_KinBody:
        case PT_Robot: {
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
            const int envBodyIndex = pbody->GetEnvironmentBodyIndex();
            {
                ExclusiveLock lock534(_mutexInterfaces);
                if ( envBodyIndex <= 0 || envBodyIndex > ((int) _vecbodies.size()) - 1 || !_vecbodies.at(envBodyIndex)) {
                    return false;
                }
                _InvalidateKinBodyFromEnvBodyIndex(envBodyIndex);
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
            RAVELOG_WARN_FORMAT("unmanaged interfaces of type '%s' cannot be removed", RaveGetInterfaceName(pinterface->GetInterfaceType()));
            break;
        }
        return false;
    }

    virtual bool RemoveKinBodyByName(const std::string& name) override
    {
        EnvironmentLock lockenv(GetMutex());
        KinBodyPtr pbody;
        {
            ExclusiveLock lock101(_mutexInterfaces);
            const string_map<int>::const_iterator it = _mapBodyNameIndex.find(name);
            if (it == _mapBodyNameIndex.end()) {
                return false;
            }
            int environmentBodyIndex = it->second;
            pbody = _InvalidateKinBodyFromEnvBodyIndex(environmentBodyIndex); // should be equal to pbody->GetEnvironmentBodyIndex());
        }
        if( !!pbody ) {
            // pbody is valid so run any callbacks and exit
            _CallBodyCallbacks(pbody, 0);
            return true;
        }

        return false;
    }

    virtual UserDataPtr RegisterBodyCallback(const BodyCallbackFn& callback) override
    {
        ExclusiveLock lock705(_mutexInterfaces);
        BodyCallbackDataPtr pdata(new BodyCallbackData(callback,boost::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredBodyCallbacks.insert(_listRegisteredBodyCallbacks.end(),pdata);
        return pdata;
    }

    KinBodyPtr GetKinBody(const std::string& name) const override
    {
        if (name.empty()) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, empty name is used to find body. Maybe caller has to be fixed.", GetId());
            return KinBodyPtr();
        }
        SharedLock lock585(_mutexInterfaces);
        if (!_vecbodies.empty()) {
            const int envBodyIndex = _FindBodyIndexByName(name);
            //RAVELOG_VERBOSE_FORMAT("env=%d, name '%s' (envBodyIndex=%d) is nullptr, maybe already removed from env?", GetId()%name%envBodyIndex);
            return _vecbodies.at(envBodyIndex);
        }
        return KinBodyPtr();
    }

    KinBodyPtr GetKinBody(const string_view name) const override
    {
        if (name.empty() ) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, empty name is used to find body. Maybe caller has to be fixed.", GetId());
            return KinBodyPtr();
        }
        SharedLock lock585(_mutexInterfaces);
        if (!_vecbodies.empty()) {
            const int envBodyIndex = _FindBodyIndexByName(name);
            //RAVELOG_VERBOSE_FORMAT("env=%d, name '%s' (envBodyIndex=%d) is nullptr, maybe already removed from env?", GetId()%pname%envBodyIndex);
            return _vecbodies.at(envBodyIndex);
        }
        return KinBodyPtr();
    }

    KinBodyPtr GetKinBody(const char* pname) const override
    {
        if (!pname || pname[0] == 0 ) {
            //RAVELOG_VERBOSE_FORMAT("env=%d, empty name is used to find body. Maybe caller has to be fixed.", GetId());
            return KinBodyPtr();
        }
        SharedLock lock585(_mutexInterfaces);
        if (!_vecbodies.empty()) {
            const int envBodyIndex = _FindBodyIndexByName(pname);
            //RAVELOG_VERBOSE_FORMAT("env=%d, name '%s' (envBodyIndex=%d) is nullptr, maybe already removed from env?", GetId()%pname%envBodyIndex);
            return _vecbodies.at(envBodyIndex);
        }
        return KinBodyPtr();
    }

    KinBodyPtr GetKinBodyById(const std::string& id) const override
    {
        if( id.empty() ) {
            return KinBodyPtr();
        }

        SharedLock lock942(_mutexInterfaces);
        const string_map<int>::const_iterator it = _mapBodyIdIndex.find(id);
        if (it == _mapBodyIdIndex.end()) {
            RAVELOG_WARN_FORMAT("env=%s, id '%s' is not found", GetNameId()%id);
            return 0;
        }
        const int envBodyIndex = it->second;
        const KinBodyPtr& pbody = _vecbodies.at(envBodyIndex);
        if (!!pbody ) {
            if( pbody->GetId()==id) {
                return pbody;
            }
            else {
                //RAVELOG_WARN_FORMAT("env=%d, body '%s' has id '%s', but environment stored its id as '%s'", GetId()%pbody->GetName()%pbody->GetId()%id);
                throw OPENRAVE_EXCEPTION_FORMAT("env=%s, body '%s' has id '%s', but environment stored its id as '%s'", GetNameId()%pbody->GetId()%GetId()%pbody->GetName()%pbody->GetName()%id, ORE_BodyIdConflict);
            }
        }
        return KinBodyPtr();
    }

    KinBodyPtr GetKinBodyById(const string_view id) const
    {
        if( id.empty() ) {
            return KinBodyPtr();
        }

        SharedLock lock942(_mutexInterfaces);
        string_view testid = id;
        const string_map<int>::const_iterator it = _mapBodyIdIndex.find(testid);
        if (it == _mapBodyIdIndex.end()) {
            RAVELOG_WARN_FORMAT("env=%s, id '%s' is not found", GetNameId()%id);
            return 0;
        }
        const int envBodyIndex = it->second;
        const KinBodyPtr& pbody = _vecbodies.at(envBodyIndex);
        if (!!pbody ) {
            if( pbody->GetId()==id) {
                return pbody;
            }
            else {
                //RAVELOG_WARN_FORMAT("env=%d, body '%s' has id '%s', but environment stored its id as '%s'", GetId()%pbody->GetName()%pbody->GetId()%id);
                throw OPENRAVE_EXCEPTION_FORMAT("env=%s, body '%s' has id '%s', but environment stored its id as '%s'", GetNameId()%pbody->GetId()%GetId()%pbody->GetName()%pbody->GetName()%id, ORE_BodyIdConflict);
            }
        }
        return KinBodyPtr();
    }

    int GetNumBodies() const override
    {
        SharedLock lock627(_mutexInterfaces);
        return (int)_mapBodyIdIndex.size();
    }

    // assumes _mutexInterfaces is locked
    inline int _GetNumBodies() const
    {
        return (int)_mapBodyNameIndex.size();
    }

    int GetMaxEnvironmentBodyIndex() const override
    {
        SharedLock lock257(_mutexInterfaces);
        if (_vecbodies.size() <= 1) { // if there is only one element, it's a nullpointer
            return 0;
        }

        if (_environmentIndexRecyclePool.empty()) { // common case, _vecbodies is filled with non-null bodies
            const int lastBodyEnvironmentBodyIndex = _vecbodies.back()->GetEnvironmentBodyIndex();
            // bodies are sorted by environment body index, so last body should have the largest
            BOOST_ASSERT(lastBodyEnvironmentBodyIndex > 0);
            BOOST_ASSERT(lastBodyEnvironmentBodyIndex == ((int)_vecbodies.size()) - 1);
            return lastBodyEnvironmentBodyIndex;
        }

        // uncommon case, find largest by iterating through. we can potentially cache this, but I think it's ok for now
        for (std::vector<KinBodyPtr>::const_reverse_iterator rit = _vecbodies.crbegin(); rit != _vecbodies.crend(); ++rit) {
            const KinBodyPtr& pbody = *rit;
            if (!!pbody) {
                return pbody->GetEnvironmentBodyIndex();
            }
        }
        return 0;
    }

    virtual RobotBasePtr GetRobot(const std::string& pname) const override
    {
        if( pname.empty() ) {
            return RobotBasePtr();
        }

        SharedLock lock412(_mutexInterfaces);
        if (_vecbodies.empty()) {
            return RobotBasePtr();
        }
        const int envBodyIndex = _FindBodyIndexByName(pname);
        const KinBodyPtr& pbody = _vecbodies.at(envBodyIndex);
        if (!!pbody && pbody->IsRobot()) {
            return RaveInterfaceCast<RobotBase>(pbody);
        }

        if (!pbody) {
            RAVELOG_WARN_FORMAT("env=%s, name '%s' (envBodyIndex=%d) is nullptr, maybe already removed from env?", GetNameId()%pname%envBodyIndex);
        }
        else {
            RAVELOG_WARN_FORMAT("env=%s, name '%s' (envBodyIndex=%d) is not robot.", GetNameId()%pname%envBodyIndex);
        }

        return RobotBasePtr();
    }

    /// assumes _mutexInterfaces is locked
    inline int _FindBodyIndexByName(const std::string& name) const
    {
        if (name.empty()) {
            return 0;
        }
        const string_map<int>::const_iterator it = _mapBodyNameIndex.find(name);
        if (it == _mapBodyNameIndex.end()) {
            //RAVELOG_WARN_FORMAT("env=%d, name '%s' is not found", GetId()%name);
            return 0;
        }
        const int envBodyIndex = it->second;
        //BOOST_ASSERT(0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()); // too many asserts
        return envBodyIndex;
    }

    /// assumes _mutexInterfaces is locked
    inline int _FindBodyIndexByName(const string_view name) const
    {
        if (name.empty() ) {
            return 0;
        }
        const string_map<int>::const_iterator it = _mapBodyNameIndex.find(name); // TODO
        if (it == _mapBodyNameIndex.end()) {
            //RAVELOG_WARN_FORMAT("env=%d, name '%s' is not found", GetId()%name);
            return 0;
        }
        const int envBodyIndex = it->second;
        //BOOST_ASSERT(0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()); // too many asserts
        return envBodyIndex;
    }

    inline int _FindBodyIndexByName(const char* pname) const
    {
        if (!pname || pname[0] == 0 ) {
            return 0;
        }
        const string_map<int>::const_iterator it = _mapBodyNameIndex.find(pname); // TODO
        if (it == _mapBodyNameIndex.end()) {
            //RAVELOG_WARN_FORMAT("env=%d, name '%s' is not found", GetId()%name);
            return 0;
        }
        const int envBodyIndex = it->second;
        //BOOST_ASSERT(0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()); // too many asserts
        return envBodyIndex;
    }

    virtual SensorBasePtr GetSensor(const std::string& name) const override
    {
        SharedLock lock022(_mutexInterfaces);
        for (const KinBodyPtr& pbody : _vecbodies) {
            if( !!pbody && pbody->IsRobot() ) {
                const RobotBasePtr& probot = RaveInterfaceCast<RobotBase>(pbody);
                FOREACHC(itsensor, probot->GetAttachedSensors()) {
                    SensorBasePtr psensor = (*itsensor)->GetSensor();
                    if( !!psensor &&( psensor->GetName() == name) ) {
                        return psensor;
                    }
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

    virtual bool SetPhysicsEngine(PhysicsEngineBasePtr pengine) override
    {
        EnvironmentLock lockenv(GetMutex());
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->DestroyEnvironment();
        }
        _pPhysicsEngine = pengine;
        if( !_pPhysicsEngine ) {
            RAVELOG_DEBUG_FORMAT("env=%s, disabling physics for", GetNameId());
            _pPhysicsEngine = RaveCreatePhysicsEngine(shared_from_this(),"GenericPhysicsEngine");
            _SetDefaultGravity();
        }
        else {
            RAVELOG_DEBUG_FORMAT("setting '%s' physics engine", _pPhysicsEngine->GetXMLId());
        }
        _pPhysicsEngine->InitEnvironment();
        return true;
    }

    virtual PhysicsEngineBasePtr GetPhysicsEngine() const override {
        return _pPhysicsEngine;
    }

    virtual UserDataPtr RegisterCollisionCallback(const CollisionCallbackFn& callback) override
    {
        ExclusiveLock lock990(_mutexInterfaces);
        CollisionCallbackDataPtr pdata(new CollisionCallbackData(callback,boost::static_pointer_cast<Environment>(shared_from_this())));
        pdata->_iterator = _listRegisteredCollisionCallbacks.insert(_listRegisteredCollisionCallbacks.end(),pdata);
        return pdata;
    }
    virtual bool HasRegisteredCollisionCallbacks() const override
    {
        ExclusiveLock lock931(_mutexInterfaces);
        return _listRegisteredCollisionCallbacks.size() > 0;
    }

    virtual void GetRegisteredCollisionCallbacks(std::list<CollisionCallbackFn>& listcallbacks) const override
    {
        ExclusiveLock lock303(_mutexInterfaces);
        listcallbacks.clear();
        FOREACHC(it, _listRegisteredCollisionCallbacks) {
            CollisionCallbackDataPtr pdata = boost::dynamic_pointer_cast<CollisionCallbackData>(it->lock());
            listcallbacks.push_back(pdata->_callback);
        }
    }

    virtual bool SetCollisionChecker(CollisionCheckerBasePtr pchecker) override
    {
        EnvironmentLock lockenv(GetMutex());
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
            RAVELOG_DEBUG_FORMAT("setting '%s' collision checker", _pCurrentChecker->GetXMLId());
            SharedLock lock132(_mutexInterfaces);
            for (KinBodyPtr& pbody : _vecbodies) {
                if (!pbody) {
                    continue;
                }
                pbody->_ResetInternalCollisionCache();
            }
        }
        return _pCurrentChecker->InitEnvironment();
    }

    virtual CollisionCheckerBasePtr GetCollisionChecker() const {
        return _pCurrentChecker;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        return _pCurrentChecker->CheckCollision(pbody1,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody1);
        CHECK_COLLISION_BODY(pbody2);
        return _pCurrentChecker->CheckCollision(pbody1,pbody2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report ) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(plink,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink1->GetParent());
        CHECK_COLLISION_BODY(plink2->GetParent());
        return _pCurrentChecker->CheckCollision(plink1,plink2,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(plink,pbody,report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(plink,vbodyexcluded,vlinkexcluded,report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(pbody,vbodyexcluded,vlinkexcluded,report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(plink->GetParent());
        return _pCurrentChecker->CheckCollision(ray,plink,report);
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(ray,pbody,report);
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report) override
    {
        return _pCurrentChecker->CheckCollision(ray,report);
    }

    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckCollision(trimesh,pbody,report);
    }

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report) override
    {
        EnvironmentLock lockenv(GetMutex());
        CHECK_COLLISION_BODY(pbody);
        return _pCurrentChecker->CheckStandaloneSelfCollision(pbody,report);
    }

    virtual void StepSimulation(dReal fTimeStep) override
    {
        EnvironmentLock lockenv(GetMutex());

        uint64_t step = (uint64_t)ceil(1000000.0 * (double)fTimeStep);
        fTimeStep = (dReal)((double)step * 0.000001);

        // call the physics first to get forces
        _pPhysicsEngine->SimulateStep(fTimeStep);

        // make a copy instead of locking the mutex pointer since will be calling into user functions
        vector<KinBodyPtr> vecbodies;
        list<SensorBasePtr> listSensors;
        list< pair<ModuleBasePtr, std::string> > listModules;
        {
            SharedLock lock358(_mutexInterfaces);
            vecbodies = _vecbodies;
            listSensors = _listSensors;
            listModules = _listModules;
        }

        for (const KinBodyPtr& pBody : vecbodies) {
            if (!pBody) {
                continue;
            }
            if( pBody->GetEnvironmentBodyIndex() ) {     // have to check if valid
                pBody->SimulationStep(fTimeStep);
            }
        }
        FOREACH(itmodule, listModules) {
            itmodule->first->SimulationStep(fTimeStep);
        }

        // simulate the sensors last (ie, they always reflect the most recent bodies
        FOREACH(itsensor, listSensors) {
            (*itsensor)->SimulationStep(fTimeStep);
        }
        for (const KinBodyPtr& pBody : vecbodies) {
            if (!pBody) {
                continue;
            }
            if( !pBody->IsRobot() ) {
                continue;
            }
            const RobotBasePtr& probot = RaveInterfaceCast<RobotBase>(pBody);
            FOREACHC(itsensor, probot->GetAttachedSensors()) {
                if( !!(*itsensor)->GetSensor() ) {
                    (*itsensor)->GetSensor()->SimulationStep(fTimeStep);
                }
            }
        }
        _nCurSimTime += step;
    }

    virtual EnvironmentMutex& GetMutex() const override {
        return _mutexEnvironment;
    }

    virtual void GetBodies(std::vector<KinBodyPtr>& bodies, uint64_t timeout) const override
    {
        TimedSharedLock lock853(_mutexInterfaces, timeout);
        if (!lock853) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        bodies.clear();
        bodies.reserve(_vecbodies.size());
        for (const KinBodyPtr& pbody : _vecbodies) {
            if (!pbody) {
                continue;
            }
            bodies.push_back(pbody);
        }
    }

    virtual void GetRobots(std::vector<RobotBasePtr>& robots, uint64_t timeout) const override
    {
        TimedSharedLock lock186(_mutexInterfaces, timeout);
        if (!lock186) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        robots.clear();
        for (const KinBodyPtr& pbody : _vecbodies) {
            if (!pbody || !pbody->IsRobot()) {
                continue;
            }
            robots.push_back(RaveInterfaceCast<RobotBase>(pbody));
        }
    }

    virtual void GetSensors(std::vector<SensorBasePtr>& vsensors, uint64_t timeout) const
    {
        TimedSharedLock lock041(_mutexInterfaces, timeout);
        if (!lock041) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        _GetSensors(vsensors);
    }

    virtual void _GetSensors(std::vector<SensorBasePtr>& vsensors) const
    {
        vsensors.resize(0);
        for (const KinBodyPtr& pBody : _vecbodies) {
            if (!pBody || !pBody->IsRobot()) {
                continue;
            }
            RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pBody);
            FOREACHC(itsensor, probot->GetAttachedSensors()) {
                SensorBasePtr psensor = (*itsensor)->GetSensor();
                if( !!psensor ) {
                    vsensors.push_back(psensor);
                }
            }
        }
    }

    virtual void Triangulate(TriMesh& trimesh, const KinBody &body) override
    {
        EnvironmentLock lockenv(GetMutex());     // reading collision data, so don't want anyone modifying it
        FOREACHC(it, body.GetLinks()) {
            trimesh.Append((*it)->GetCollisionData(), (*it)->GetTransform());
        }
    }

    virtual void TriangulateScene(TriMesh& trimesh, SelectionOptions options,const std::string& selectname) override
    {
        EnvironmentLock lockenv(GetMutex());
        ExclusiveLock lock830(_mutexInterfaces);
        for (KinBodyPtr& pbody : _vecbodies) {
            if (!pbody) {
                continue;
            }
            const KinBody& body = *pbody;
            RobotBasePtr robot;
            if( body.IsRobot() ) {
                robot = RaveInterfaceCast<RobotBase>(pbody);
            }
            switch(options) {
            case SO_NoRobots:
                if( !robot ) {
                    Triangulate(trimesh, body);
                }
                break;

            case SO_Robots:
                if( !!robot ) {
                    Triangulate(trimesh, body);
                }
                break;
            case SO_Everything:
                Triangulate(trimesh, body);
                break;
            case SO_Body:
                if( body.GetName() == selectname ) {
                    Triangulate(trimesh, body);
                }
                break;
            case SO_AllExceptBody:
                if( body.GetName() != selectname ) {
                    Triangulate(trimesh, body);
                }
                break;
//            case SO_BodyList:
//                if( find(listnames.begin(),listnames.end(),(pbody)->GetName()) != listnames.end() ) {
//                    Triangulate(trimesh,pbody);
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
        EnvironmentLock lockenv(GetMutex());

        if( !!robot ) {
            SharedLock lock617(_mutexInterfaces);

            int bodyIndex = robot->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinRobot::Init for '%s', cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        std::string path;
        if (_IsURI(filename, path)) {
            if (_IsColladaFile(path)) {
                if( !RaveParseColladaURI(shared_from_this(), robot, filename, atts) ) {
                    return RobotBasePtr();
                }
            }
            else if (_IsJSONFile(path)) {
                _ClearRapidJsonBuffer();
                if( !RaveParseJSONURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                    return RobotBasePtr();
                }
            }
            else if (_IsMsgPackFile(path)) {
                _ClearRapidJsonBuffer();
                if( !RaveParseMsgPackURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                    return RobotBasePtr();
                }
            }
            else if (StringEndsWith(path, ".json.gpg")) {
                _ClearRapidJsonBuffer();
                if( !RaveParseEncryptedJSONURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                    return RobotBasePtr();
                }
            }
            else if (StringEndsWith(path, ".msgpack.gpg")) {
                _ClearRapidJsonBuffer();
                if( !RaveParseEncryptedMsgPackURI(shared_from_this(), robot, filename, atts, *_prLoadEnvAlloc) ) {
                    return RobotBasePtr();
                }
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
                std::vector<KinBody::GeometryInfo> vGeometries;
                std::string fullfilename = _ReadGeometriesFile(vGeometries,filename,atts);
                if( fullfilename.size() > 0 ) {
                    string extension;
                    if( filename.find_last_of('.') != string::npos ) {
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
                    string norender = string("__norenderif__:")+extension;
                    FOREACH(itinfo,vGeometries) {
                        itinfo->_bVisible = true;
                        itinfo->_filenamerender = norender;
                    }
                    vGeometries.front()._filenamerender = fullfilename;
                    if( robot->InitFromGeometries(vGeometries) ) {
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
        }

        // have to set the URI to the passed in one rather than the resolved one, otherwise external components won't be able to compare if a URI is equivalent or not
        if( !!robot ) {
            if( robot->__struri.empty() ) {
                robot->__struri = filename;
            }
        }

        return robot;
    }

    virtual RobotBasePtr ReadRobotData(RobotBasePtr robot, const std::string& data, const AttributesList& atts, const std::string& uri) override
    {
        EnvironmentLock lockenv(GetMutex());

        if( !!robot ) {
            SharedLock lock681(_mutexInterfaces);

            int bodyIndex = robot->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinRobot::Init for '%s', cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), robot, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONData(shared_from_this(), robot, uri, data, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackData(shared_from_this(), robot, uri, data, atts, *_prLoadEnvAlloc) ) {
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
            if( robot->__struri.empty() ) {
                robot->__struri = preader->_filename;
            }
            if( robot->__struri.empty() ) {
                robot->__struri = uri;
            }
        }

        return robot;
    }

    virtual RobotBasePtr ReadRobotJSON(RobotBasePtr robot, const rapidjson::Value& rEnvInfo, const AttributesList& atts, const std::string &uri)
    {
        EnvironmentLock lockenv(GetMutex());

        if( !!robot ) {  // TODO: move this to a shared place
            SharedLock lock681(_mutexInterfaces);

            int bodyIndex = robot->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinRobot::Init for '%s', cannot Init a robot while it is added to the environment\n"))%robot->GetName()));
            }
        }

        _ClearRapidJsonBuffer();
        if( !RaveParseJSON(shared_from_this(), uri, robot, rEnvInfo, atts, *_prLoadEnvAlloc) ) {
            robot.reset();
        }
        return robot;
    }

    virtual KinBodyPtr ReadKinBodyURI(KinBodyPtr body, const std::string& filename, const AttributesList& atts) override
    {
        EnvironmentLock lockenv(GetMutex());

        if( !!body ) {
            SharedLock lock285(_mutexInterfaces);

            int bodyIndex = body->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinBody::Init for '%s', cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        std::string path;
        if (_IsURI(filename, path)) {
            if (_IsColladaFile(path)) {
                if( !RaveParseColladaURI(shared_from_this(), body, filename, atts) ) {
                    return KinBodyPtr();
                }
            }
            else if (_IsJSONFile(path)) {
                _ClearRapidJsonBuffer();
                if( !RaveParseJSONURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                    return KinBodyPtr();
                }
            }
            else if (_IsMsgPackFile(path)) {
                _ClearRapidJsonBuffer();
                if( !RaveParseMsgPackURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                    return KinBodyPtr();
                }
            }
            else if (StringEndsWith(path, ".json.gpg")) {
                _ClearRapidJsonBuffer();
                if( !RaveParseEncryptedJSONURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                    return KinBodyPtr();
                }
            }
            else if (StringEndsWith(path, ".msgpack.gpg")) {
                _ClearRapidJsonBuffer();
                if( !RaveParseEncryptedMsgPackURI(shared_from_this(), body, filename, atts, *_prLoadEnvAlloc) ) {
                    return KinBodyPtr();
                }
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
                std::vector<KinBody::GeometryInfo> vGeometries;
                std::string fullfilename = _ReadGeometriesFile(vGeometries,filename,atts);
                if( fullfilename.size() > 0 ) {
                    string extension;
                    if( filename.find_last_of('.') != string::npos ) {
                        extension = filename.substr(filename.find_last_of('.')+1);
                    }
                    string norender = string("__norenderif__:")+extension;
                    FOREACH(itinfo,vGeometries) {
                        itinfo->_bVisible = true;
                        itinfo->_filenamerender = norender;
                    }
                    vGeometries.front()._filenamerender = fullfilename;
                    if( body->InitFromGeometries(vGeometries) ) {
                        if( body->__struri.empty() ) {
                            body->__struri = fullfilename;
                        }
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
        }

        // have to set the URI to the passed in one rather than the resolved one, otherwise external components won't be able to compare if a URI is equivalent or not
        if( !!body ) {
            if( body->__struri.empty() ) {
                body->__struri = filename;
            }
        }

        return body;
    }

    virtual KinBodyPtr ReadKinBodyData(KinBodyPtr body, const std::string& data, const AttributesList& atts, const std::string& uri)
    {
        EnvironmentLock lockenv(GetMutex());

        if( !!body ) {
            SharedLock lock937(_mutexInterfaces);

            int bodyIndex = body->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinBody::Init for '%s', cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        if( _IsColladaData(data) ) {
            if( !RaveParseColladaData(shared_from_this(), body, data, atts) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsJSONData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseJSONData(shared_from_this(), body, uri, data, atts, *_prLoadEnvAlloc) ) {
                return RobotBasePtr();
            }
        }
        else if( _IsMsgPackData(data) ) {
            _ClearRapidJsonBuffer();
            if( !RaveParseMsgPackData(shared_from_this(), body, uri, data, atts, *_prLoadEnvAlloc) ) {
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
            if( body->__struri.empty() ) {
                body->__struri = preader->_filename;
            }
        }

        return body;
    }

    virtual KinBodyPtr ReadKinBodyJSON(KinBodyPtr body, const rapidjson::Value& rEnvInfo, const AttributesList& atts, const std::string &uri)
    {
        EnvironmentLock lockenv(GetMutex());

        if( !!body ) {  // TODO: move this to a shared place
            SharedLock lock937(_mutexInterfaces);

            int bodyIndex = body->GetEnvironmentBodyIndex();
            if( bodyIndex > 0 && bodyIndex < (int)_vecbodies.size() && !!_vecbodies.at(bodyIndex) ) {
                throw openrave_exception(str(boost::format(_("KinBody::Init for '%s', cannot Init a body while it is added to the environment\n"))%body->GetName()));
            }
        }

        _ClearRapidJsonBuffer();
        if( !RaveParseJSON(shared_from_this(), uri, body, rEnvInfo, atts, *_prLoadEnvAlloc) ) {
            body.reset();
        }
        return body;
    }

    virtual InterfaceBasePtr ReadInterfaceURI(const std::string& filename, const AttributesList& atts)
    {
        try {
            EnvironmentLock lockenv(GetMutex());
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
            if(preadable->_pinterface->__struri.empty() ) {
                preadable->_pinterface->__struri = filename;
            }
            return preadable->_pinterface;
        }
        catch(const std::exception &ex) {
            RAVELOG_ERROR_FORMAT("ReadInterfaceXMLFile exception: %s", ex.what());
        }
        return InterfaceBasePtr();
    }

    virtual InterfaceBasePtr ReadInterfaceURI(InterfaceBasePtr pinterface, InterfaceType type, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentLock lockenv(GetMutex());
        bool bIsCollada = false;
        bool bIsJSON = false;
        bool bIsMsgPack = false;
        bool bIsEncryptedJSON = false;
        bool bIsEncryptedMsgPack = false;
        bool bIsX = false;

        std::string path;
        bool bIsURI = _IsURI(filename, path);
        if (bIsURI) {
            path = filename;
        }
        if (_IsColladaFile(path)) {
            bIsCollada = true;
        }
        else if (_IsJSONFile(path)) {
            bIsJSON = true;
        }
        else if (_IsMsgPackFile(path)) {
            bIsMsgPack = true;
        }
        else if (StringEndsWith(path, ".json.gpg")) {
            bIsEncryptedJSON = true;
        }
        else if (StringEndsWith(path, ".msgpack.gpg")) {
            bIsEncryptedMsgPack = true;
        }
        else if (_IsXFile(path)) {
            bIsX = true;
        }

        if (type == PT_KinBody || type == PT_Robot) {
            BOOST_ASSERT(!pinterface || (pinterface->GetInterfaceType() == PT_KinBody || pinterface->GetInterfaceType() == PT_Robot));
            KinBodyPtr pbody = RaveInterfaceCast<KinBody>(pinterface);
            if (bIsCollada) {
                if (bIsURI) {
                    if( !RaveParseColladaURI(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                } else {
                    if( !RaveParseColladaFile(shared_from_this(), pbody, filename, atts) ) {
                        return InterfaceBasePtr();
                    }
                }
            } else if (bIsJSON) {
                _ClearRapidJsonBuffer();
                if (bIsURI) {
                    if( !RaveParseJSONURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                } else {
                    if( !RaveParseJSONFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
            } else if (bIsMsgPack) {
                _ClearRapidJsonBuffer();
                if (bIsURI) {
                    if( !RaveParseMsgPackURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                } else {
                    if( !RaveParseMsgPackFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                }
            } else if (bIsEncryptedJSON) {
                _ClearRapidJsonBuffer();
                if (bIsURI) {
                    if ( !RaveParseEncryptedJSONURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                } else {
                    if (!RaveParseEncryptedJSONFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc)) {
                        return InterfaceBasePtr();
                    }
                }
            } else if (bIsEncryptedMsgPack) {
                _ClearRapidJsonBuffer();
                if (bIsURI) {
                    if ( !RaveParseEncryptedMsgPackURI(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc) ) {
                        return InterfaceBasePtr();
                    }
                } else {
                    if (!RaveParseEncryptedMsgPackFile(shared_from_this(), pbody, filename, atts, *_prLoadEnvAlloc)) {
                        return InterfaceBasePtr();
                    }
                }
            } else if (bIsX) {
                if( !RaveParseXFile(shared_from_this(), pbody, filename, atts) ) {
                    return InterfaceBasePtr();
                }
            } else {
                return InterfaceBasePtr();
            }
            pinterface = (type == PT_Robot) ? OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase>(pbody) : pbody;
            if( pinterface->__struri.empty() ) {
                pinterface->__struri = filename;
            }
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
            if( pinterface->__struri.empty() ) {
                pinterface->__struri = filename;
            }
        }
        return pinterface;
    }

    virtual InterfaceBasePtr ReadInterfaceData(InterfaceBasePtr pinterface, InterfaceType type, const std::string& data, const AttributesList& atts, const std::string& uri)
    {
        EnvironmentLock lockenv(GetMutex());

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
        if( pinterface->__struri.empty() ) {
            pinterface->__struri = preader->_filename;
        }
        if( pinterface->__struri.empty() ) {
            pinterface->__struri = uri;
        }
        return pinterface;
    }

    virtual boost::shared_ptr<TriMesh> ReadTrimeshURI(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, const AttributesList& atts)
    {
        RaveVector<float> diffuseColor, ambientColor;
        return _ReadTrimeshURI(ptrimesh,filename,diffuseColor, ambientColor, atts);
    }

    virtual boost::shared_ptr<TriMesh> _ReadTrimeshURI(boost::shared_ptr<TriMesh> ptrimesh, const std::string& filename, RaveVector<float>& diffuseColor, RaveVector<float>& ambientColor, const AttributesList& atts)
    {
        //EnvironmentLock lockenv(GetMutex()); // don't lock!
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
    /// \param[in] vGeometries geometry list to be filled
    virtual std::string _ReadGeometriesFile(std::vector<KinBody::GeometryInfo>& vGeometries, const std::string& filename, const AttributesList& atts)
    {
        EnvironmentLock lockenv(GetMutex());
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
        if( OpenRAVEXMLParser::CreateGeometries(shared_from_this(),filedata, vScaleGeometry, vGeometries) && vGeometries.size() > 0 ) {
            return filedata;
        }
        vGeometries.clear();
        return std::string();
    }

    virtual void _AddViewer(ViewerBasePtr pnewviewer)
    {
        CHECK_INTERFACE(pnewviewer);
        EnvironmentLock lockenv(GetMutex());
        ExclusiveLock lock212(_mutexInterfaces);
        BOOST_ASSERT(find(_listViewers.begin(),_listViewers.end(),pnewviewer) == _listViewers.end() );
        _CheckUniqueName(ViewerBaseConstPtr(pnewviewer),true);
        _listViewers.push_back(pnewviewer);
    }

    virtual ViewerBasePtr GetViewer(const std::string& name) const
    {
        SharedLock lock510(_mutexInterfaces);
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
        SharedLock lock032(_mutexInterfaces);
        listViewers = _listViewers;
    }

    virtual OpenRAVE::GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle)
    {
        SharedLock lock749(_mutexInterfaces);
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
        SharedLock lock696(_mutexInterfaces);
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
        SharedLock lock215(_mutexInterfaces);
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
        SharedLock lock188(_mutexInterfaces);
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
        SharedLock lock436(_mutexInterfaces);
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
        SharedLock lock785(_mutexInterfaces);
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
        SharedLock lock750(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawarrow(p1,p2,fwidth,color));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawlabel(const std::string& label, const RaveVector<float>& worldPosition, const RaveVector<float>& color = RaveVector<float>(0,0,0,1), float height=0.05)
    {
        SharedLock lock777(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawlabel(label, worldPosition, color, height));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents)
    {
        SharedLock lock782(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawbox(vpos, vextents));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawboxarray(const std::vector<RaveVector<float> >& vpos, const RaveVector<float>& vextents, const std::vector<RaveVector<float> >& vcolors)
    {
        SharedLock lock103(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawboxarray(vpos, vextents, vcolors));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawaabb(const AABB& aabb, const RaveTransform<float>& transform, const RaveVector<float>& vcolor, float transparency)
    {
        SharedLock lock782(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawaabb(aabb, transform, vcolor, transparency));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawobb(const OrientedBox& obb, const RaveVector<float>& vcolor, float transparency)
    {
        SharedLock lock782(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawobb(obb, vcolor, transparency));
        }
        return handles;
    }
    virtual OpenRAVE::GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture)
    {
        SharedLock lock756(_mutexInterfaces);
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
        SharedLock lock993(_mutexInterfaces);
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
        SharedLock lock394(_mutexInterfaces);
        if( _listViewers.size() == 0 ) {
            return OpenRAVE::GraphHandlePtr();
        }
        GraphHandleMultiPtr handles(new GraphHandleMulti());
        FOREACHC(itviewer, _listViewers) {
            handles->Add((*itviewer)->drawtrimesh(ppoints, stride, pIndices, numTriangles, colors));
        }
        return handles;
    }

    KinBodyPtr GetBodyFromEnvironmentBodyIndex(int bodyIndex) const override
    {
        SharedLock lock430(_mutexInterfaces);
        if (0 < bodyIndex && bodyIndex < (int) _vecbodies.size()) {
            return _vecbodies.at(bodyIndex);
        }
        return KinBodyPtr();
    }

    bool GetBodyNameFromEnvironmentBodyIndex(int bodyIndex, std::string& out) const override
    {
        SharedLock lock(_mutexInterfaces);
        if (0 < bodyIndex && bodyIndex < (int)_vecbodies.size()) {
            out = _vecbodies.at(bodyIndex)->GetName();
            return true;
        }
        return false;
    }

    int GetEnvironmentBodyIndexByName(string_view bodyName) const override
    {
        SharedLock lock(_mutexInterfaces);
        return _FindBodyIndexByName(bodyName);
    }

    void GetBodiesFromEnvironmentBodyIndices(const std::vector<int>& bodyIndices,
                                             std::vector<KinBodyPtr>& bodies) const override
    {
        bodies.clear();
        bodies.reserve(bodyIndices.size());
        SharedLock lock138(_mutexInterfaces);
        for (int bodyIndex : bodyIndices) {
            if (0 < bodyIndex && bodyIndex < (int) _vecbodies.size()) {
                bodies.push_back(_vecbodies.at(bodyIndex));
            }
            else {
                RAVELOG_WARN_FORMAT("env=%s, could not find body for environment body index=%d from %d bodies", GetNameId()%bodyIndex%_vecbodies.size());
                bodies.push_back(KinBodyPtr());
            }
        }
    }

    virtual void StartSimulation(dReal fDeltaTime, bool bRealTime)
    {
        {
            EnvironmentLock lockenv(GetMutex());
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
            EnvironmentLock lockenv(GetMutex());
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
        TimedSharedLock lock452(_mutexInterfaces, timeout);
        if (!lock452) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        vbodies = _vPublishedBodies;
    }

    virtual bool GetPublishedBody(const std::string &name, KinBody::BodyState& bodystate, uint64_t timeout=0)
    {
        TimedSharedLock lock306(_mutexInterfaces, timeout);
        if (!lock306) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
            if ( _vPublishedBodies[ibody].strname == name) {
                bodystate = _vPublishedBodies[ibody];
                return true;
            }
        }

        return false;
    }

    virtual bool GetPublishedBodyJointValues(const std::string& name, std::vector<dReal> &jointValues, uint64_t timeout=0)
    {
        TimedSharedLock lock118(_mutexInterfaces, timeout);
        if (!lock118) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        for ( size_t ibody = 0; ibody < _vPublishedBodies.size(); ++ibody) {
            if ( _vPublishedBodies[ibody].strname == name) {
                jointValues = _vPublishedBodies[ibody].jointvalues;
                return true;
            }
        }

        return false;
    }

    void GetPublishedBodyTransformsMatchingPrefix(const std::string& prefix, std::vector<std::pair<std::string, Transform> >& nameTransfPairs, uint64_t timeout = 0)
    {
        TimedSharedLock lock079(_mutexInterfaces, timeout);
        if (!lock079) {
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

    virtual void UpdatePublishedBodies(uint64_t timeout=0)
    {
        EnvironmentLock lockenv(GetMutex());
        TimedExclusiveLock lock152(_mutexInterfaces, timeout);
        if (!lock152) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("timeout of %f s failed"),(1e-6*static_cast<double>(timeout)),ORE_Timeout);
        }
        _UpdatePublishedBodies();
    }

    /// assumes GetMutex() and _mutexInterfaces are both exclusively locked
    virtual void _UpdatePublishedBodies()
    {
        // updated the published bodies, resize dynamically in case an exception occurs
        // when creating an item and bad data is left inside _vPublishedBodies
        _vPublishedBodies.resize(_GetNumBodies());
        int iwritten = 0;

        std::vector<dReal> vdoflastsetvalues;
        for(const KinBodyPtr& pbody : _vecbodies) {
            if (!pbody || pbody->GetEnvironmentBodyIndex() == 0) {
                continue;
            }
            if( pbody->_nHierarchyComputed != 2 ) {
                // skip
                continue;
            }

            KinBody::BodyState& state = _vPublishedBodies.at(iwritten);

            // If this state was already inititalized from this body, we might be able to skip updating it if the body itself hasn't changed.
            const bool canSkipUpdate = state.pbody == pbody && state.updatestamp == pbody->GetUpdateStamp();

            // Only if the body is mismatched with the state do we need to do a full update
            if (!canSkipUpdate) {
                state.Reset();
                state.pbody = pbody;
                pbody->GetLinkTransformations(state.vectrans, vdoflastsetvalues);
                pbody->GetLinkEnableStates(state.vLinkEnableStates);
                pbody->GetDOFValues(state.jointvalues);
                pbody->GetGrabbedInfo(state.vGrabbedInfos);
                state.strname = pbody->GetName();
                state.uri = pbody->GetURI();
                state.updatestamp = pbody->GetUpdateStamp();
                state.environmentid = pbody->GetEnvironmentBodyIndex();
                if (pbody->IsRobot()) {
                    RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                    if (!!probot) {
                        RobotBase::ManipulatorPtr pmanip = probot->GetActiveManipulator();
                        if (!!pmanip) {
                            state.activeManipulatorName = pmanip->GetName();
                            state.activeManipulatorTransform = pmanip->GetTransform();
                        }
                        probot->GetConnectedBodyActiveStates(state.vConnectedBodyActiveStates);
                    }
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
        return std::make_pair(std::string(GetLengthUnitString(_unitInfo.lengthUnit)), 1.0 / GetLengthUnitStandardValue<dReal>(_unitInfo.lengthUnit));
    }

    virtual void SetUnit(std::pair<std::string, dReal> unit)
    {
        _unitInfo.lengthUnit = GetLengthUnitFromString(unit.first, LU_Meter);
    }

    virtual UnitInfo GetUnitInfo() const
    {
        return _unitInfo;
    }

    virtual void SetUnitInfo(UnitInfo unitInfo)
    {
        _unitInfo = unitInfo;
    }

    /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
    virtual void ExtractInfo(EnvironmentBaseInfo& info) override
    {
        EnvironmentLock lockenv(GetMutex());
        std::vector<KinBodyPtr> vBodies;
        int numBodies = 0;
        {
            SharedLock lock464(_mutexInterfaces);
            numBodies = _GetNumBodies();
            vBodies = _vecbodies;
            if( (int)vBodies.size() != numBodies + (int)_environmentIndexRecyclePool.size() + 1 ) {
                // There is a +1 because the first element of _vecbodies is a null pointer.
                RAVELOG_WARN_FORMAT("env=%s, _vecbodies has %d bodies. _mapBodyNameIndex has %d bodies. pool has %d indices.",
                                    GetNameId() % _vecbodies.size() % _mapBodyNameIndex.size() % _environmentIndexRecyclePool.size());
            }
        }
        info._vBodyInfos.resize(numBodies);
        int validBodyItr = 0;
        for(KinBodyPtr& pbody : vBodies) {
            if (!pbody) {
                continue;
            }
            KinBody::KinBodyInfoPtr& pbodyFromInfo = info._vBodyInfos.at(validBodyItr);
            if (pbody->IsRobot()) {
                pbodyFromInfo.reset(new RobotBase::RobotBaseInfo());
                RaveInterfaceCast<RobotBase>(pbody)->ExtractInfo(*(OPENRAVE_DYNAMIC_POINTER_CAST<RobotBase::RobotBaseInfo>(pbodyFromInfo)), EIO_Everything);
            }
            else {
                pbodyFromInfo.reset(new KinBody::KinBodyInfo());
                pbody->ExtractInfo(*pbodyFromInfo, EIO_Everything);
            }
            ++validBodyItr;
        }
        if( validBodyItr != numBodies) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, started out with %d bodies, but could only iterate over %d valid ones."), GetNameId()%numBodies%validBodyItr, ORE_InvalidState);
        }
        info._keywords = _keywords;
        info._description = _description;
        if (!!_pPhysicsEngine) {
            info._gravity = _pPhysicsEngine->GetGravity();
        }
        info._uInt64Parameters = _mapUInt64Parameters;
        info._unitInfo = _unitInfo;
    }

    /// \brief update EnvironmentBase according to new EnvironmentBaseInfo, returns false if update cannot be performed and requires InitFromInfo
    void UpdateFromInfo(const EnvironmentBaseInfo& info, std::vector<KinBodyPtr>& vCreatedBodies, std::vector<KinBodyPtr>& vModifiedBodies, std::vector<KinBodyPtr>& vRemovedBodies, UpdateFromInfoMode updateMode) override
    {
        RAVELOG_VERBOSE_FORMAT("=== UpdateFromInfo start, env=%d ===", GetId());

        vCreatedBodies.clear();
        vModifiedBodies.clear();
        vRemovedBodies.clear();

        EnvironmentLock lockenv(GetMutex());
        std::vector<dReal> vDOFValues;

        if( updateMode != UFIM_OnlySpecifiedBodiesExact ) {
            // copy basic info into EnvironmentBase
            _revision = info._revision;
            //_name = info._name; not copying name, just like __nUniqueId, it is not updated from info
            _keywords = info._keywords;
            _description = info._description;
            _mapUInt64Parameters = info._uInt64Parameters;
            if( _unitInfo != info._unitInfo ) {
                rapidjson::Document rThisUnitInfo;
                orjson::SaveJsonValue(rThisUnitInfo, _unitInfo);
                rapidjson::Document rNewUnitInfo;
                orjson::SaveJsonValue(rNewUnitInfo, info._unitInfo);
                RAVELOG_WARN_FORMAT("env=%s, env unit '%s' does not match one coming from UpdateFromInfo %s", GetNameId()%orjson::DumpJson(rThisUnitInfo)%orjson::DumpJson(rNewUnitInfo));
                // throw OPENRAVE_EXCEPTION_FORMAT("env=%s, env unit '%s' does not match one coming from UpdateFromInfo %s", GetNameId()%_unitInfo.toString()%info._unitInfo.toString(), ORE_InvalidArguments);
            }

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
        std::list<KinBodyPtr> listBodiesTemporarilyRenamed; // in order to avoid clashing names, sometimes bodies are renamed temporariliy and placed in this list. If they targetted again with a different name, then they are put back into the environment
        {
            SharedLock lock533(_mutexInterfaces);
            vBodies = _vecbodies;
        }
        {
            for (std::vector<KinBodyPtr>::iterator it = vBodies.begin(); it != vBodies.end(); ) {
                if (!*it) {
                    it = vBodies.erase(it);
                }
                else {
                    it++;
                }
            }
        }

        // Set of indices that have already been used for vBodies
        std::unordered_set<int> usedBodyIndexSet;

        // internally manipulates _vecbodies using _AddKinBody/_AddRobot/_RemoveKinBodyFromIterator
        for(int inputBodyIndex = 0; inputBodyIndex < (int)info._vBodyInfos.size(); ++inputBodyIndex) {
            const KinBody::KinBodyInfoConstPtr& pKinBodyInfo = info._vBodyInfos[inputBodyIndex];
            const KinBody::KinBodyInfo& kinBodyInfo = *pKinBodyInfo;
            RAVELOG_VERBOSE_FORMAT("env=%s, id '%s', name '%s'", GetNameId()%pKinBodyInfo->_id%pKinBodyInfo->_name);
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
                    for (int ibody = 0; ibody < (int)vBodies.size(); ++ibody) {
                        if (usedBodyIndexSet.find(ibody) != usedBodyIndexSet.end()) {
                            continue;
                        }
                        const KinBodyPtr& pbody = vBodies[ibody];
                        if (!pbody) {
                            continue;
                        }
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
                            if (!(*itBody)) {
                                continue;
                            }
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
                        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' interface is changed, remove old body from environment. xmlid=%s, _interfaceType=%s, isRobot %d != %d", GetNameId()%pBody->_id%pBody->GetXMLId()%pKinBodyInfo->_interfaceType%pBody->IsRobot()%pKinBodyInfo->_isRobot);
                        itExisting = vBodies.end();
                        vRemovedBodies.push_back(pBody);

                        ExclusiveLock lock690(_mutexInterfaces);
                        vector<KinBodyPtr>::iterator itBodyToRemove = std::find(_vecbodies.begin(), _vecbodies.end(), pBody);
                        if( itBodyToRemove != _vecbodies.end() ) {
                            _InvalidateKinBodyFromEnvBodyIndex(pBody->GetEnvironmentBodyIndex());
                        }
                    }
                }

                if( itExisting != vBodies.end() ) {
                    if( itExisting != itExistingSameName && itExistingSameName != vBodies.end() ) {
                        // new name will conflict with *itExistingSameName, so should change the names to something temporarily
                        // for now, clear since the body should be processed later again
                        RAVELOG_DEBUG_FORMAT("env=%s, have to clear body name '%s' id=%s for loading body with id=%s", GetNameId()%(*itExistingSameName)->GetName()%(*itExistingSameName)->GetId()%(*itExisting)->GetId());
                        (*itExistingSameName)->SetName(_GetUniqueName((*itExistingSameName)->GetName()+"_tempRenamedDueToConflict_"));
                        listBodiesTemporarilyRenamed.push_back(*itExistingSameName);
                    }
                    pMatchExistingBody = *itExisting;
                    int nMatchingIndex = itExisting-vBodies.begin();
                    if ( bodyIndex >= 0 && bodyIndex != nMatchingIndex) {
                        // re-arrange vBodies according to the order of infos
                        KinBodyPtr pTempBody = vBodies.at(bodyIndex);
                        vBodies.at(bodyIndex) = pMatchExistingBody;
                        *itExisting = pTempBody;
                    }

                    if (updateMode == UFIM_OnlySpecifiedBodiesExact) {
                        usedBodyIndexSet.emplace(nMatchingIndex);
                    }
                }
            }

            KinBodyPtr pInitBody; // body that has to be Init() again
            std::vector<KinBodyPtr> pGrabbingBodies;
            std::vector<KinBody::LinkPtr> pGrabbingLinks;
            std::vector<rapidjson::Document> rGrabbedUserDataDocuments;
            std::vector<std::set<int>> linkIndicesToIgnore;

            if( !!pMatchExistingBody ) {
                listBodiesTemporarilyRenamed.remove(pMatchExistingBody); // if targreted, then do not need to remove anymore

                RAVELOG_VERBOSE_FORMAT("env=%s, update existing body id '%s'", GetNameId()%pMatchExistingBody->_id);
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
                RAVELOG_VERBOSE_FORMAT("env=%s, update body '%s' from info result %d", GetNameId() % pMatchExistingBody->_id % static_cast<int>(updateFromInfoResult));
                if (updateFromInfoResult == UFIR_NoChange) {
                    continue;
                }
                if (info._lastModifiedAtUS > pMatchExistingBody->_lastModifiedAtUS) {
                    pMatchExistingBody->_lastModifiedAtUS = info._lastModifiedAtUS;
                }
                pMatchExistingBody->_revisionId = info._revisionId;
                vModifiedBodies.push_back(pMatchExistingBody);
                if (updateFromInfoResult == UFIR_Success) {
                    continue;
                }

                // if this body is grabbed by another bodies, save the grabbing link and user data
                for (const KinBodyWeakPtr& pBody : pMatchExistingBody->_listAttachedBodies) {
                    KinBodyPtr pAttached = pBody.lock();
                    if (!pAttached) {
                        continue;
                    }
                    for (const KinBody::MapGrabbedByEnvironmentIndex::value_type& grabPair : pAttached->_grabbedBodiesByEnvironmentIndex) {
                        const GrabbedPtr& pGrabbed = grabPair.second;
                        KinBodyConstPtr pGrabbedBody = pGrabbed->_pGrabbedBody.lock();
                        if( !!pGrabbedBody && pGrabbedBody.get() == &*pMatchExistingBody ) {
                            pGrabbingBodies.push_back(pAttached);
                            pGrabbingLinks.push_back(pGrabbed->_pGrabbingLink);
                            rapidjson::Document rGrabbedUserData;
                            rGrabbedUserData.CopyFrom(pGrabbed->_rGrabbedUserData, rGrabbedUserData.GetAllocator());
                            rGrabbedUserDataDocuments.push_back(std::move(rGrabbedUserData));
                            linkIndicesToIgnore.push_back(pGrabbed->_setGrabberLinkIndicesToIgnore);
                        }
                    }
                }

                // updating this body requires removing it and re-adding it to env
                {
                    ExclusiveLock lock253(_mutexInterfaces);
                    vector<KinBodyPtr>::iterator itExisting = std::find(_vecbodies.begin(), _vecbodies.end(), pMatchExistingBody);
                    if( itExisting != _vecbodies.end() ) {
                        _InvalidateKinBodyFromEnvBodyIndex(pMatchExistingBody->GetEnvironmentBodyIndex()); // essentially removes the entry from the environment
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
                        // have to reinit, but preserves the current body id since it could potentially clash with what pKinBodyInfo/pRobotBaseInfo has
                        KinBodyIdSaver bodyIdSaver(pRobot);
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
                        // have to reinit, but preserves the current body id since it could potentially clash with what pKinBodyInfo has
                        KinBodyIdSaver bodyIdSaver(pMatchExistingBody);
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
                    RAVELOG_VERBOSE_FORMAT("add new robot id '%s'", pKinBodyInfo->_id);
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
                    RAVELOG_VERBOSE_FORMAT("add new kinbody id '%s'", pKinBodyInfo->_id);
                    pNewBody = RaveCreateKinBody(shared_from_this(), pKinBodyInfo->_interfaceType);
                    if( !pNewBody ) {
                        pNewBody = RaveCreateKinBody(shared_from_this(), "");
                    }
                    pNewBody->InitFromKinBodyInfo(*pKinBodyInfo);
                    pInitBody = pNewBody;
                    _AddKinBody(pNewBody, IAM_AllowRenaming);
                }

                if (bodyIndex >= 0) {
                    if (updateMode == UFIM_OnlySpecifiedBodiesExact) {
                        usedBodyIndexSet.emplace(bodyIndex);
                    }
                    vBodies.insert(vBodies.begin() + bodyIndex, pNewBody);
                }
                else {
                    if (updateMode == UFIM_OnlySpecifiedBodiesExact) {
                        usedBodyIndexSet.emplace(vBodies.size());
                    }
                    vBodies.push_back(pNewBody);
                }
                pNewBody->_lastModifiedAtUS = info._lastModifiedAtUS;
                pNewBody->_revisionId = info._revisionId;
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

                // re-grab after add this body back to the environment
                for (int grabbingBodyIndex = 0; grabbingBodyIndex < (int)pGrabbingBodies.size(); grabbingBodyIndex++) {
                    pGrabbingBodies[grabbingBodyIndex]->Grab(pInitBody, pGrabbingLinks[grabbingBodyIndex], linkIndicesToIgnore[grabbingBodyIndex], rGrabbedUserDataDocuments[grabbingBodyIndex]);
                }
            }
        }

        if( updateMode != UFIM_OnlySpecifiedBodiesExact ) {
            // remove extra bodies at the end of vBodies
            if( vBodies.size() > info._vBodyInfos.size() ) {
                ExclusiveLock lock009(_mutexInterfaces);
                for (std::vector<KinBodyPtr>::iterator itBody = vBodies.begin() + info._vBodyInfos.size(); itBody != vBodies.end(); ) {
                    KinBodyPtr pBody = *itBody;
                    if (!pBody) {
                        ++itBody;
                        continue;
                    }
                    RAVELOG_VERBOSE_FORMAT("remove extra body env=%s, id=%s, name=%s", GetNameId()%pBody->_id%pBody->_name);

                    vector<KinBodyPtr>::iterator itBodyToRemove = std::find(_vecbodies.begin(), _vecbodies.end(), pBody);
                    if( itBodyToRemove != _vecbodies.end() ) {
                        _InvalidateKinBodyFromEnvBodyIndex(pBody->GetEnvironmentBodyIndex());
                    }

                    vRemovedBodies.push_back(pBody);
                    itBody = vBodies.erase(itBody);
                }
            }
        }

        for(KinBodyPtr pRenamedBody : listBodiesTemporarilyRenamed) {
            RAVELOG_INFO_FORMAT("env=%s, body '%s' (id=%s) was renamed to avoid a name conflict, but not modified to another name, so assuming that it should be deleted", GetNameId()%pRenamedBody->GetName()%pRenamedBody->GetId());
            Remove(pRenamedBody);
            vRemovedBodies.push_back(pRenamedBody);
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
                if ((int)pKinBodyInfo->_vGrabbedInfos.size() != (*itExistingBody)->GetNumGrabbed()) {
                    RAVELOG_DEBUG_FORMAT("env=%s, body name='%s' updating grab from %d -> %d", GetNameId()%bodyName%(*itExistingBody)->GetNumGrabbed()%pKinBodyInfo->_vGrabbedInfos.size());
                    // when grab info changes, have to report to caller
                    if (std::find(vModifiedBodies.begin(), vModifiedBodies.end(), *itExistingBody) == vModifiedBodies.end() && std::find(vCreatedBodies.begin(), vCreatedBodies.end(), *itExistingBody) == vCreatedBodies.end()) {
                        vModifiedBodies.push_back(*itExistingBody);
                    }
                }
                vGrabbedInfos.clear();
                vGrabbedInfos.reserve(pKinBodyInfo->_vGrabbedInfos.size());
                FOREACHC(itGrabbedInfo, pKinBodyInfo->_vGrabbedInfos) {
                    if (!!GetKinBody((*itGrabbedInfo)->_grabbedname)) {
                        vGrabbedInfos.push_back(*itGrabbedInfo);
                    }
                    else {
                        RAVELOG_WARN_FORMAT("env=%s, body '%s' grabbed by '%s' is gone, ignoring grabbed info id '%s'", GetNameId()%(*itGrabbedInfo)->_grabbedname%pKinBodyInfo->_name%(*itGrabbedInfo)->_id);
                    }
                }
                (*itExistingBody)->ResetGrabbed(vGrabbedInfos);
            }
            else {
                RAVELOG_WARN_FORMAT("env=%s, could not find body with name='%s'", GetNameId()%bodyName);
            }
        }

        UpdatePublishedBodies();
    }

    int GetRevision() const override {
        EnvironmentLock lockenv(GetMutex());
        return _revision;
    }

    void SetDescription(const std::string& sceneDescription) override {
        EnvironmentLock lockenv(GetMutex());
        _description = sceneDescription;
    }

    std::string GetDescription() const override {
        EnvironmentLock lockenv(GetMutex());
        return _description;
    }

    void SetKeywords(const std::vector<std::string>& sceneKeywords) override {
        EnvironmentLock lockenv(GetMutex());
        _keywords = sceneKeywords;
    }

    std::vector<std::string> GetKeywords() const override {
        EnvironmentLock lockenv(GetMutex());
        return _keywords;
    }

    void SetUInt64Parameter(const std::string& parameterName, uint64_t value) override {
        EnvironmentLock lockenv(GetMutex());
        _mapUInt64Parameters[parameterName] = value;
    }

    bool RemoveUInt64Parameter(const std::string& parameterName) override
    {
        EnvironmentLock lockenv(GetMutex());
        return _mapUInt64Parameters.erase(parameterName) > 0;
    }

    uint64_t GetUInt64Parameter(const std::string& parameterName, uint64_t defaultValue) const override {
        EnvironmentLock lockenv(GetMutex());
        std::map<std::string, uint64_t>::const_iterator it = _mapUInt64Parameters.find(parameterName);
        if( it != _mapUInt64Parameters.end() ) {
            return it->second;
        }

        return defaultValue;
    }

    bool NotifyKinBodyNameChanged(const std::string& oldName, const std::string& newName) override
    {
        ExclusiveLock lock114(_mutexInterfaces);
        const string_map<int>::const_iterator itOld = _mapBodyNameIndex.find(oldName);
        const string_map<int>::const_iterator itNew = _mapBodyNameIndex.find(newName);
        if (itOld == _mapBodyNameIndex.end()) {
            return itNew == _mapBodyNameIndex.end(); // new should be empty
        }

        if (itNew != _mapBodyNameIndex.end()) {
            if( itOld == itNew ) {
                // same..
                return true;
            }
            else {
                // cannot complete the operation, something is wrong since id is not unique anymore
                return false;
            }
        }
        const int envBodyIndex = itOld->second;
        _mapBodyNameIndex.erase(itOld);
        _mapBodyNameIndex[newName] = envBodyIndex;
        RAVELOG_VERBOSE_FORMAT("env=%s, body '%s' is renamed to '%s'", GetNameId()%oldName%newName);

        return true;
    }

    bool NotifyKinBodyIdChanged(const std::string& oldId, const std::string& newId) override
    {
        ExclusiveLock lock891(_mutexInterfaces);
        const string_map<int>::const_iterator itOld = _mapBodyIdIndex.find(oldId);
        const string_map<int>::const_iterator itNew = _mapBodyIdIndex.find(newId);
        if (itOld == _mapBodyIdIndex.end()) {
            return itNew == _mapBodyIdIndex.end(); // new should be empty
        }

        if (itNew != _mapBodyIdIndex.end()) {
            if( itOld == itNew ) {
                // same..
                return true;
            }
            else {
                // cannot complete the operation, something is wrong since id is not unique anymore
                return false;
            }
        }
        const int envBodyIndex = itOld->second;
        _mapBodyIdIndex.erase(itOld);
        _mapBodyIdIndex[newId] = envBodyIndex;
        RAVELOG_VERBOSE_FORMAT("env=%s, body id changed from '%s' to '%s'", GetNameId()%oldId%newId);
        return true;
    }

protected:

    void _Init()
    {
        _homedirectory = RaveGetHomeDirectory();
        RAVELOG_DEBUG_FORMAT("env=%s, setting openrave home directory to '%s'", GetNameId()%_homedirectory);

        _nBodiesModifiedStamp = 0;

        _assignedBodySensorNameIdSuffix = 0;

        _fDeltaSimTime = 0.01f;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _bRealTime = true;
        _bInit = false;
        _bEnableSimulation = true;     // need to start by default
        _unitInfo = UnitInfo();
        _unitInfo.lengthUnit = LU_Meter; // default unit settings
        _unitInfo.angleUnit = AU_Radian; // default unit settings

        _vRapidJsonLoadBuffer.resize(4000000);
        _prLoadEnvAlloc.reset(new rapidjson::MemoryPoolAllocator<>(&_vRapidJsonLoadBuffer[0], _vRapidJsonLoadBuffer.size()));

        _handlegenericrobot = RaveRegisterInterface(PT_Robot,"GenericRobot", RaveGetInterfaceHash(PT_Robot), GetHash(), CreateGenericRobot);
        _handlegenerictrajectory = RaveRegisterInterface(PT_Trajectory,"GenericTrajectory", RaveGetInterfaceHash(PT_Trajectory), GetHash(), CreateGenericTrajectory);
        _handlemulticontroller = RaveRegisterInterface(PT_Controller,"GenericMultiController", RaveGetInterfaceHash(PT_Controller), GetHash(), CreateMultiController);
        _handlegenericphysicsengine = RaveRegisterInterface(PT_PhysicsEngine,"GenericPhysicsEngine", RaveGetInterfaceHash(PT_PhysicsEngine), GetHash(), CreateGenericPhysicsEngine);
        _handlegenericcollisionchecker = RaveRegisterInterface(PT_CollisionChecker,"GenericCollisionChecker", RaveGetInterfaceHash(PT_CollisionChecker), GetHash(), CreateGenericCollisionChecker);
    }

    /// \brief invalidates a kinbody from _vecbodies
    /// \param[in] bodyIndex environment body index of kin body to be invalidated
    /// assumes environment and _mutexInterfaces are exclusively locked
    KinBodyPtr _InvalidateKinBodyFromEnvBodyIndex(int bodyIndex)
    {
        KinBodyPtr& pbodyref = _vecbodies.at(bodyIndex);
        if (!pbodyref) {
            return KinBodyPtr();
        }
        KinBody& body = *pbodyref;
        const std::string& name = body.GetName();

        // Before deleting a body, ensure that no other bodies are still grabbing it.
        // Since all grabbing bodies should be in the attached set for this body, we can localize the search to these bodies.
        // Otherwise we have to scan the entire environment, which is slow for scenes with many bodies in them
        {
            // Get the list of bodies that are grabbing this body for cleanup in a second pass.
            // We can't ungrab here because it would modify our listAttachedBodies while we are iterating it.
            std::vector<KinBodyPtr> grabbingBodies;
            for (KinBodyWeakPtr& pWeakOtherBody : body._listAttachedBodies) {
                KinBodyPtr pOtherBody = pWeakOtherBody.lock();
                if (!!pOtherBody && pOtherBody->IsGrabbing(body)) {
                    grabbingBodies.emplace_back(std::move(pOtherBody));
                }
            }

            // Once we've found all of the grabbing bodies, ungrab ourselves with all of them
            for (KinBodyPtr& pOtherBody : grabbingBodies) {
                RAVELOG_WARN_FORMAT("env=%s, remove %s already grabbed by body %s!", GetNameId()%body.GetName()%pOtherBody->GetName());
                pOtherBody->Release(body);
            }
        }

        body.ReleaseAllGrabbed();
        if( !!_pCurrentChecker ) {
            _pCurrentChecker->RemoveKinBody(pbodyref);
        }
        {
            CollisionCheckerBasePtr pSelfColChecker = body.GetSelfCollisionChecker();
            if (!!pSelfColChecker && pSelfColChecker != _pCurrentChecker) {
                pSelfColChecker->RemoveKinBody(pbodyref);
            }
        }
        // remove from self collision checker of other bodies since it may have been grabbed.
        for (KinBodyPtr& potherbody : _vecbodies) {
            if( !!potherbody && pbodyref != potherbody ) {
                CollisionCheckerBasePtr pOtherSelfColChecker = potherbody->GetSelfCollisionChecker();
                if( !!pOtherSelfColChecker && pOtherSelfColChecker != _pCurrentChecker ) {
                    pOtherSelfColChecker->RemoveKinBody(pbodyref); // should be okay to call RemoveKinBody even when the pbodyref has not been aeed to the pOtherSelfColChecker
                }
            }
        }
        if( !!_pPhysicsEngine ) {
            _pPhysicsEngine->RemoveKinBody(pbodyref);
        }
        body._PostprocessChangedParameters(KinBody::Prop_BodyRemoved);

        // invalidate cache
        if (_mapBodyNameIndex.erase(name) == 0) {
            RAVELOG_WARN_FORMAT("env=%s, pbody of name '%s' not found in _mapBodyNameIndex of size %d, this should not happen!", GetNameId()%name%_mapBodyNameIndex.size());
        }
        const std::string& id = body.GetId();
        if (_mapBodyIdIndex.erase(id) == 0) {
            RAVELOG_WARN_FORMAT("env=%s, pbody of id '%s' not found in _mapBodyIdIndex of size %d, this should not happen!", GetNameId()%id%_mapBodyIdIndex.size());
        }
        _UnassignEnvironmentBodyIndex(body);

        KinBodyPtr pbody;
        pbody.swap(pbodyref); // essentially resets _vecbodies[bodyIndex]

        _nBodiesModifiedStamp++;
        return pbody;
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
        EnvironmentLock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLFile(preader, filename);
    }

    virtual bool _ParseXMLData(BaseXMLReaderPtr preader, const std::string& pdata)
    {
        EnvironmentLock lockenv(GetMutex());
        return OpenRAVEXMLParser::ParseXMLData(preader, pdata);
    }

    virtual void _Clone(boost::shared_ptr<Environment const> r, int options, bool bCheckSharedResources=false)
    {
        if( !bCheckSharedResources ) {
            Destroy();
        }

        std::lock_guard<std::mutex> lockinit(_mutexInit);
        if( !bCheckSharedResources ) {
            SetCollisionChecker(CollisionCheckerBasePtr());
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        _nBodiesModifiedStamp = r->_nBodiesModifiedStamp;
        _homedirectory = r->_homedirectory;
        _fDeltaSimTime = r->_fDeltaSimTime;
        _nCurSimTime = 0;
        _nSimStartTime = utils::GetMicroTime();
        _bRealTime = r->_bRealTime;

        _description = r->_description;
        _keywords = r->_keywords;
        _mapUInt64Parameters = r->_mapUInt64Parameters;

        _assignedBodySensorNameIdSuffix = r->_assignedBodySensorNameIdSuffix;

        _bInit = true;
        _bEnableSimulation = r->_bEnableSimulation;

        SetDebugLevel(r->GetDebugLevel());

        if( !bCheckSharedResources || !(options & Clone_Bodies) ) {
            {
                // clear internal interface lists
                ExclusiveLock lock007(_mutexInterfaces);
                // release all grabbed
                for (KinBodyPtr& probot : _vecbodies) {
                    if (!!probot) {
                        probot->ReleaseAllGrabbed();
                    }
                }
                for (KinBodyPtr& pbody : _vecbodies) {
                    if (!!pbody) {
                        pbody->Destroy();
                    }
                }
                _vecbodies.clear();
                _mapBodyNameIndex.clear();
                _mapBodyIdIndex.clear();
                _environmentIndexRecyclePool.clear();

                _vPublishedBodies.clear();
            }
        }

        list<ViewerBasePtr> listViewers = _listViewers;
        list< pair<ModuleBasePtr, std::string> > listModules = _listModules;
        {
            ExclusiveLock lock753(_mutexInterfaces);
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

        EnvironmentLock lock(GetMutex());

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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to clone collision checker '%s': %s"), r->GetCollisionChecker()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
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
                    throw OPENRAVE_EXCEPTION_FORMAT(_("failed to clone physics engine '%s': %s"), r->GetPhysicsEngine()->GetXMLId()%ex.what(),ORE_InvalidPlugin);
                }
            }
        }
        else {
            SetPhysicsEngine(PhysicsEngineBasePtr());
        }

        if( options & Clone_Bodies ) {
            ExclusiveLock lock717(r->_mutexInterfaces);
            _environmentIndexRecyclePool = r->_environmentIndexRecyclePool;

            std::vector<std::pair<Vector,Vector> > linkvelocities;
            std::vector<KinBodyPtr> vecbodies;
            string_map<int> mapBodyNameIndex, mapBodyIdIndex;

            if( bCheckSharedResources ) {
                // delete any bodies/robots from mapBodies that are not in r->_vecbodies
                vecbodies.swap(_vecbodies);
                mapBodyNameIndex.swap(_mapBodyNameIndex);
                mapBodyIdIndex.swap(_mapBodyIdIndex);
            }
            // first initialize the pointers
            list<KinBodyPtr> listToClone, listToCopyState;
            for (const KinBodyPtr& pbodyInOtherEnv : r->_vecbodies) {
                if (!pbodyInOtherEnv || !pbodyInOtherEnv->IsRobot()) {
                    continue;
                }
                RobotBasePtr probotInOtherEnv = RaveInterfaceCast<RobotBase>(pbodyInOtherEnv);
                BOOST_ASSERT(!!probotInOtherEnv);
                const RobotBase& robotInOtherEnv = *probotInOtherEnv;
                try {
                    RobotBasePtr pnewrobot;
                    if( bCheckSharedResources ) {
                        for (const KinBodyPtr& probotInThisEnv : vecbodies) {
                            if( !!probotInThisEnv &&
                                probotInThisEnv->IsRobot() &&
                                probotInThisEnv->GetName() == robotInOtherEnv.GetName() &&
                                probotInThisEnv->GetKinematicsGeometryHash() == robotInOtherEnv.GetKinematicsGeometryHash() ) {
                                pnewrobot = RaveInterfaceCast<RobotBase>(probotInThisEnv);
                                break;
                            }
                        }
                    }
                    if( !pnewrobot ) {
                        pnewrobot = RaveCreateRobot(shared_from_this(), robotInOtherEnv.GetXMLId());
                        // at least copy the name and ids who are assumed to be unique within env
                        pnewrobot->_name = robotInOtherEnv._name;
                        pnewrobot->_id = robotInOtherEnv._id;
                        listToClone.push_back(probotInOtherEnv);
                    }
                    else {
                        //TODO
                        //pnewrobot->ReleaseAllGrabbed(); // will re-grab later?
                        listToCopyState.push_back(probotInOtherEnv);
                    }
                    const int envBodyIndex = robotInOtherEnv.GetEnvironmentBodyIndex();
                    BOOST_ASSERT( 0 < envBodyIndex);
                    BOOST_ASSERT( (int)_vecbodies.size() < envBodyIndex + 1 || !_vecbodies.at(envBodyIndex));
                    pnewrobot->_environmentBodyIndex = envBodyIndex; // I guess it's ok to directly set env body index, because it was unique in the other env.
                    {
                        ExclusiveLock lock584(_mutexInterfaces);
                        _AddKinBodyInternal(pnewrobot, envBodyIndex);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone robot '%s': %s", robotInOtherEnv.GetName()%ex.what());
                }
            }
            for (const KinBodyPtr& pbody : r->_vecbodies) {
                if (!pbody || pbody->IsRobot()) { // here, handle non-robot kinbody. robot is already handled in previous loop, so skip
                    continue;
                }
                const KinBody& body = *pbody;
                const int envBodyIndex = body.GetEnvironmentBodyIndex();
                try {
                    KinBodyPtr pnewbody;
                    if( bCheckSharedResources ) {
                        const std::string& name = body.GetName();
                        const string_map<int>::const_iterator it = mapBodyNameIndex.find(name);
                        if (it != mapBodyNameIndex.end()) {
                            const int envBodyIdx = it->second;
                            BOOST_ASSERT(0 < envBodyIdx && envBodyIdx < (int) vecbodies.size());
                            const KinBodyPtr& pNewBodyCandidate = vecbodies.at(envBodyIdx);
                            if( !pNewBodyCandidate ) {
                                RAVELOG_WARN_FORMAT("env=%s, a body (name=%s, envBodyIndex=%d) in vecbodies is not initialized", GetNameId()%name%envBodyIdx);
                            }
                            else if (pNewBodyCandidate->GetKinematicsGeometryHash() == body.GetKinematicsGeometryHash() ) {
                                pnewbody = pNewBodyCandidate;
                            }
                        }
                    }
                    if( !pnewbody ) {
                        pnewbody.reset(new KinBody(PT_KinBody,shared_from_this()));
                        // at least copy the name and ids who are assumed to be unique within env
                        pnewbody->_name = body._name;
                        pnewbody->_id = body._id;
                        listToClone.push_back(pbody);
                    }
                    else {
                        listToCopyState.push_back(pbody);
                    }
                    pnewbody->_environmentBodyIndex = envBodyIndex;

                    {
                        ExclusiveLock lock647(_mutexInterfaces);
                        _AddKinBodyInternal(pnewbody, envBodyIndex);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("env=%s, failed to clone body '%s': %s", GetNameId()%body.GetName()%ex.what());
                }
            }

            // copy state before cloning
            if( listToCopyState.size() > 0 ) {
                for (const KinBodyPtr& pbody : listToCopyState) {
                    const KinBody& body = *pbody;
                    const int envBodyIndex = body.GetEnvironmentBodyIndex();
                    KinBodyPtr pnewbody = _vecbodies.at(envBodyIndex);
                    if( bCollisionCheckerChanged ) {
                        GetCollisionChecker()->InitKinBody(pnewbody);
                    }
                    if( bPhysicsEngineChanged ) {
                        GetPhysicsEngine()->InitKinBody(pnewbody);
                    }
                    pnewbody->__hashKinematicsGeometryDynamics = body.__hashKinematicsGeometryDynamics;
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
                const int envBodyIndex = body.GetEnvironmentBodyIndex();
                try {
                    KinBodyPtr pnewbody = _vecbodies.at(envBodyIndex);
                    if( !!pnewbody ) {
                        // Should ignore grabbed bodies since the grabbed states will be updated later (via state saver's Restore) below.
                        pnewbody->Clone(pbody, options|Clone_IgnoreGrabbedBodies);
                    }
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("env=%s, failed to clone body '%s': %s", GetNameId()%body.GetName()%ex.what());
                }
            }

            for (const KinBodyPtr& pbody : listToClone) {
                const KinBody& body = *pbody;
                const int envBodyIndex = body.GetEnvironmentBodyIndex();
                KinBodyPtr pnewbody = _vecbodies.at(envBodyIndex);
                pnewbody->_ComputeInternalInformation();
                GetCollisionChecker()->InitKinBody(pnewbody);
                GetPhysicsEngine()->InitKinBody(pnewbody);
                pnewbody->__hashKinematicsGeometryDynamics = body.__hashKinematicsGeometryDynamics;
                if( pnewbody->IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                    pnewrobot->__hashrobotstructure = poldrobot->__hashrobotstructure;
                }
            }
            // update the state after every body is initialized!
            for (const KinBodyPtr& pbody : listToClone) {
                const KinBody& body = *pbody;
                const int envBodyIndex = body.GetEnvironmentBodyIndex();
                KinBodyPtr pnewbody = _vecbodies.at(envBodyIndex);
                if( body.IsRobot() ) {
                    RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                    RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(pnewbody);
                    if( !!poldrobot && !!pnewrobot ) {
                        pnewrobot->_RestoreStateForClone(poldrobot, false);
                    }
                }
                else {
                    if( !!pbody && !!pnewbody ) {
                        pnewbody->_RestoreStateForClone(pbody);
                    }
                }
            }
            if( listToCopyState.size() > 0 ) {
                // check for re-grabs after cloning is done
                for (const KinBodyPtr& pbody : listToCopyState) {
                    const KinBody& body = *pbody;
                    if( body.IsRobot() ) {
                        const int envBodyIndex = body.GetEnvironmentBodyIndex();
                        RobotBasePtr poldrobot = RaveInterfaceCast<RobotBase>(pbody);
                        RobotBasePtr pnewrobot = RaveInterfaceCast<RobotBase>(_vecbodies.at(envBodyIndex));
                        if( !!poldrobot && !!pnewrobot ) {
                            pnewrobot->_RestoreStateForClone(poldrobot, true);
                        }
                    }
                }
            }
        }
        if( options & Clone_Sensors ) {
            ExclusiveLock lock748(r->_mutexInterfaces);
            FOREACHC(itsensor,r->_listSensors) {
                try {
                    SensorBasePtr pnewsensor = RaveCreateSensor(shared_from_this(), (*itsensor)->GetXMLId());
                    pnewsensor->Clone(*itsensor, options);
                    _listSensors.push_back(pnewsensor);
                }
                catch(const std::exception &ex) {
                    RAVELOG_ERROR_FORMAT("failed to clone sensor '%s': %s", (*itsensor)->GetName()%ex.what());
                }
            }
        }
        // sensors might be attached on a robot?, so have to re-update
        for (KinBodyPtr& pbody : _vecbodies) {
            if (!pbody || !pbody->IsRobot()) {
                continue;
            }
            RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
            probot->_UpdateAttachedSensors();
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
                    RAVELOG_ERROR_FORMAT("failed to clone module '%s': %s", itmodule2->first->GetXMLId()%ex.what());
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
                    RAVELOG_ERROR_FORMAT("failed to clone viewer '%s': %s", (*itviewer2)->GetName()%ex.what());
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

    /// \brief adds pbody to _vecbodies and other internal data structures
    /// \param pbody kin body to be added to the environment
    /// \param envBodyIndex environment body index of the pbody
    /// assuming _mutexInterfaces is exclusively locked
    inline void _AddKinBodyInternal(KinBodyPtr pbody, int envBodyIndex)
    {
        EnsureVectorSize(_vecbodies, envBodyIndex+1);
        _vecbodies.at(envBodyIndex) = pbody;

        {
            const std::string& name = pbody->GetName();
            _mapBodyNameIndex[name] = envBodyIndex;
            //RAVELOG_DEBUG_FORMAT("env=%d: name=%s -> bodyIndex=%d, _mapBodyNameIndex has %d elements", GetId()%name%envBodyIndex%_mapBodyNameIndex.size());
        }

        {
            const std::string& id = pbody->GetId();
            _mapBodyIdIndex[id] = envBodyIndex;
            //RAVELOG_DEBUG_FORMAT("env=%d: id=%s -> bodyIndex=%d, _mapBodyIdIndex has %d elements", GetId()%id%newBodyIndex%_mapBodyIdIndex.size());
        }
    }

    /// \brief assign body / sensor to unique id by adding suffix
    ///
    /// locks _mutexInterfaces internally
    inline void _EnsureUniqueId(const KinBodyPtr& pbody)
    {
        const std::string baseId = pbody->GetId(); // has to store value instead of reference, as we call SetId internally
        string newId;
        while (true) {
            newId = str(boost::format("%s%d")%baseId%_assignedBodySensorNameIdSuffix++);
            pbody->SetId(newId);

            // most likely unique, but have to double check
            if( utils::IsValidName(newId) && _CheckUniqueId(pbody, false) ) {
                if( !baseId.empty() ) {
                    RAVELOG_DEBUG_FORMAT("env=%s, setting body '%s' id from '%s' -> '%s' due to conflict", GetNameId()%pbody->GetName()%baseId%newId);
                }
                break;
            }
            RAVELOG_INFO_FORMAT("env=%s, tried renaming body '%s' id from '%s' -> '%s' due to conflict, but conflict again. This is highly unlikely to happen.", GetNameId()%pbody->GetName()%baseId%newId);
        }
    }


    /// \brief name body / sensor to unique name by adding suffix
    ///
    /// locks _mutexInterfaces internally
    template<typename T>
    inline void _EnsureUniqueName(const T& pObject)
    {
        const std::string baseName = pObject->GetName(); // has to store value instead of reference, as we call SetName internally
        BOOST_ASSERT(utils::IsValidName(baseName));
        string newName;
        while (true) {
            newName = str(boost::format("%s%d")%baseName%_assignedBodySensorNameIdSuffix++);
            pObject->SetName(newName);

            //SharedLock lock833(_mutexInterfaces); // _CheckUniqueId already locks _mutexInterfaces
            // most likely unique, but have to double check
            if( utils::IsValidName(newName) && _CheckUniqueName(pObject, false) ) {
                if( !baseName.empty() ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, setting body name from '%s' -> '%s' due to conflict", GetId()%baseName%newName);
                }
                break;
            }
            RAVELOG_INFO_FORMAT("env=%d, tried renaming object (body or sensor) from '%s' -> '%s' due to conflict, but conflict again. This is highly unlikely to happen.", GetId()%baseName%newName);
        }
    }

    inline std::string _GetUniqueName(const std::string& baseName)
    {
        BOOST_ASSERT(utils::IsValidName(baseName));
        std::string newName;
        while (true) {
            newName = str(boost::format("%s%d")%baseName%_assignedBodySensorNameIdSuffix++);

            //SharedLock lock504(_mutexInterfaces);
            // most likely unique, but have to double check
            if( utils::IsValidName(newName) && _CheckUniqueName(newName, false) ) {
                break;
            }
        }

        return newName;
    }

    /// locks _mutexInterfaces internally
    virtual bool _CheckUniqueName(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        const std::string& name = pbody->GetName();

        SharedLock lock086(_mutexInterfaces);
        const string_map<int>::const_iterator it = _mapBodyNameIndex.find(name);
        if (it == _mapBodyNameIndex.end()) {
            return true;
        }

        // if _mapBodyNameIndex contained invalid env body indexBody, it's a bug that _mapBodyNameIndex and _vecbodies are not in sync
        const int envBodyIndex = it->second;
        if( envBodyIndex <= 0 ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, body '%s' is not added to the environment."), GetNameId()%pbody->GetName(), ORE_InvalidState);
        }
        if( envBodyIndex >= (int)_vecbodies.size() ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, body '%s' has environmentBodyIndex=%d, which is greater than the number of bodies %d, _mapBodyNameIndex=%d."), GetNameId()%pbody->GetName()%envBodyIndex%_vecbodies.size()%_mapBodyNameIndex.size(), ORE_InvalidState);
        }

        const KinBodyPtr& pExistingBody = _vecbodies.at(envBodyIndex);
        BOOST_ASSERT(!!pExistingBody); // if _mapBodyNameIndex contained env body index of null KinBody, it's a bug that _mapBodyNameIndex and _vecbodies are not in sync

        if (pExistingBody == pbody) {
            return true; // found itself, this case is considered as ok (name is unique)
        }

        if( bDoThrow ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, body (id='%s', envBodyIndex=%d) has same name '%s' as existing body (id='%s', envBodyIndex=%d)"), GetNameId()%pbody->GetId()%pbody->GetEnvironmentBodyIndex()%name%pExistingBody->GetId()%pExistingBody->GetEnvironmentBodyIndex(), ORE_BodyNameConflict);
        }
        return false;
    }

    /// \brief return true if no body has this name
    ///
    /// \param name name of the body
    virtual bool _CheckUniqueName(const std::string& name, bool bDoThrow=false) const
    {
        SharedLock lock797(_mutexInterfaces);
        const string_map<int>::const_iterator it = _mapBodyNameIndex.find(name);
        if (it == _mapBodyNameIndex.end()) {
            return true;
        }

        const int envBodyIndex = it->second;
        BOOST_ASSERT(0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()); // if _mapBodyNameIndex contained invalid env body indexBody, it's a bug that _mapBodyNameIndex and _vecbodies are not in sync

        // should not be here...
        return !_vecbodies.at(envBodyIndex);
    }

    /// \brief do not allow empty ids
    /// locks _mutexInterfaces internally
    virtual bool _CheckUniqueId(KinBodyConstPtr pbody, bool bDoThrow=false) const
    {
        const std::string& inputBodyId = pbody->GetId();
        if( inputBodyId.empty() ) {
            if( bDoThrow ) {
                throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, body '%s' does not have a valid id '%s'"), GetId()%pbody->GetName()%inputBodyId, ORE_BodyIdConflict);
            }
            return false;
        }

        SharedLock lock646(_mutexInterfaces);
        const string_map<int>::const_iterator it = _mapBodyIdIndex.find(inputBodyId);
        if (it == _mapBodyIdIndex.end()) {
            return true;
        }

        const int envBodyIndex = it->second;
        BOOST_ASSERT(0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()); // if _mapBodyIdIndex contained invalid env body indexBody, it's a bug that _mapBodyIdIndex and _vecbodies are not in sync

        const KinBodyPtr& pExistingBody = _vecbodies.at(envBodyIndex);
        BOOST_ASSERT(!!pExistingBody); // if _mapBodyIdIndex contained env body index of null KinBody, it's a bug that _mapBodyIdIndex and _vecbodies are not in sync

        if (pExistingBody == pbody) {
            return true; // found itself, this case is considered as ok (id is unique)
        }

        if( bDoThrow ) {
            throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, body (name='%s', envBodyIndex=%d) has same id '%s' as existing body (name='%s', envBodyIndex=%d)"), GetId()%pbody->GetId()%pbody->GetEnvironmentBodyIndex()%inputBodyId%pExistingBody->GetName()%pExistingBody->GetEnvironmentBodyIndex(), ORE_BodyIdConflict);
        }
        return false;
    }

    virtual bool _CheckUniqueName(SensorBaseConstPtr psensor, bool bDoThrow=false) const
    {
        FOREACHC(itsensor,_listSensors) {
            if(( *itsensor != psensor) &&( (*itsensor)->GetName() == psensor->GetName()) ) {
                if( bDoThrow ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%d, sensor '%s' does not have unique name"), GetId()%psensor->GetName(), ORE_SensorNameConflict);
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

    /// assumes _mutexInterfaces is locked
    virtual int _AssignEnvironmentBodyIndex(KinBodyPtr pbody, int requestedEnvironmentBodyIndex = 0)
    {
        const bool bRecycleId = !_environmentIndexRecyclePool.empty();
        int envBodyIndex = 0;
        if (requestedEnvironmentBodyIndex > 0) {
            // user requested env body index
            std::set<int>::iterator smallestIt = _environmentIndexRecyclePool.find(requestedEnvironmentBodyIndex);
            if (smallestIt != _environmentIndexRecyclePool.end()) {
                envBodyIndex = *smallestIt;
                _environmentIndexRecyclePool.erase(smallestIt);
                RAVELOG_VERBOSE_FORMAT("env=%s, recycled body envBodyIndex=%d for '%s'. %d remaining in pool", GetNameId()%envBodyIndex%pbody->GetName()%_environmentIndexRecyclePool.size());
            }
            else {
                envBodyIndex = requestedEnvironmentBodyIndex;
                if ((size_t) envBodyIndex < _vecbodies.size() && !!_vecbodies.at(envBodyIndex)) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("env=%s, environmentBodyIndex=%d is used by existing body=%s while trying to add new body=%s"), GetNameId() % requestedEnvironmentBodyIndex % _vecbodies.at(envBodyIndex)->GetName() % pbody->GetName(), ORE_EnvironmentBodyIndexConflict);
                }
                RAVELOG_VERBOSE_FORMAT("env=%s, use requested envBodyIndex=%d for '%s'. %d remaining in pool", GetNameId()%envBodyIndex%pbody->GetName()%_environmentIndexRecyclePool.size());
            }
        }
        else {
            if (bRecycleId) {
                std::set<int>::iterator smallestIt = _environmentIndexRecyclePool.begin();
                envBodyIndex = *smallestIt;
                _environmentIndexRecyclePool.erase(smallestIt);
                RAVELOG_VERBOSE_FORMAT("env=%s, recycled body envBodyIndex=%d for '%s'. %d remaining in pool", GetNameId()%envBodyIndex%pbody->GetName()%_environmentIndexRecyclePool.size());
            }
            else {
                envBodyIndex = _vecbodies.empty() ? 1 : _vecbodies.size(); // skip 0
                if( envBodyIndex > 200 ) { // give some number sufficiently big so that leaking of objects can be detected rather than spamming the log
                    RAVELOG_DEBUG_FORMAT("env=%s, assigned new body envBodyIndex=%d for '%s', this should not happen unless total number of bodies in env keeps increasing", GetNameId()%envBodyIndex%pbody->GetName());
                }
            }
        }
        pbody->_environmentBodyIndex = envBodyIndex;
        return envBodyIndex;
    }

    /// assumes _mutexInterfaces is exclusively locked
    virtual void _UnassignEnvironmentBodyIndex(KinBody& body)
    {
        const int envBodyIndex = body._environmentBodyIndex;
        if (0 < envBodyIndex && envBodyIndex < (int) _vecbodies.size()) {
            _environmentIndexRecyclePool.insert(envBodyIndex); // for recycle later
            RAVELOG_VERBOSE_FORMAT("env=%s, removed body name '%s' (environmentBodyIndex=%d), recycle body index later", GetNameId()%body.GetName()%body._environmentBodyIndex);
        }
        else {
            RAVELOG_WARN_FORMAT("env=%s, removed body name '%s' (environmentBodyIndex=%d, _vecbodies size=%d) is not valid. ", GetNameId()%body.GetName()%body._environmentBodyIndex%_vecbodies.size());
        }

        body._environmentBodyIndex = 0;
        body._DeinitializeInternalInformation();
    }

    void _StartSimulationThread()
    {
        if( !_threadSimulation ) {
            _bShutdownSimulation = false;
            _threadSimulation = boost::make_shared<std::thread>(std::bind(&Environment::_SimulationThread, this));
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
            boost::shared_ptr<EnvironmentLock> lockenv;
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
                            std::this_thread::sleep_for(std::chrono::microseconds(actual_sleep));
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
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    /// _mutexInterfaces should not be locked
    void _CallBodyCallbacks(KinBodyPtr pbody, int action)
    {
        std::list<UserDataWeakPtr> listRegisteredBodyCallbacks;
        {
            ExclusiveLock lock638(_mutexInterfaces);
            listRegisteredBodyCallbacks = _listRegisteredBodyCallbacks;
        }
        FOREACH(it, listRegisteredBodyCallbacks) {
            BodyCallbackDataPtr pdata = boost::dynamic_pointer_cast<BodyCallbackData>(it->lock());
            if( !!pdata ) {
                pdata->_callback(pbody, action);
            }
        }
    }

    boost::shared_ptr<EnvironmentLock> _LockEnvironmentWithTimeout(uint64_t timeout)
    {
        // try to acquire the lock
        boost::shared_ptr<EnvironmentLock> lockenv = boost::make_shared<EnvironmentLock>(GetMutex(), OpenRAVE::defer_lock_t());
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

    static bool _IsURI(const std::string& uri, std::string& path)
    {
        string scheme, authority, query, fragment;
        string s1, s3, s6, s8;
        static pcrecpp::RE re("^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\\?([^#]*))?(#(.*))?");
        bool bmatch = re.FullMatch(uri, &s1, &scheme, &s3, &authority, &path, &s6, &query, &s8, &fragment);
        return bmatch && !scheme.empty();
    }

    static bool _IsColladaFile(const std::string& filename)
    {
        return StringEndsWith(filename, ".dae") || StringEndsWith(filename, ".zae");
    }
    static bool _IsColladaData(const std::string& data)
    {
        return data.find("<COLLADA") != std::string::npos;
    }

    static bool _IsXFile(const std::string& filename)
    {
        return StringEndsWith(filename, ".x");
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
        return StringEndsWith(filename, ".xml");
    }
    static bool _IsRigidModelFile(const std::string& filename)
    {
        static boost::array<std::string,21> s_geometryextentsions = { { "iv","vrml","wrl","stl","blend","3ds","ase","obj","ply","dxf","lwo","lxo","ac","ms3d","x","mesh.xml","irrmesh","irr","nff","off","raw"}};
        for (const std::string& ext : s_geometryextentsions) {
            if (StringEndsWith(filename, ext)) {
                return true;
            }
        }
        return false;
    }

    static bool _IsJSONFile(const std::string& filename)
    {
        return StringEndsWith(filename, ".json");
    }

    static bool _IsJSONData(const std::string& data)
    {
        return data.size() >= 2 && data[0] == '{';
    }

    static bool _IsMsgPackFile(const std::string& filename)
    {
        // .msgpack
        return StringEndsWith(filename, ".msgpack");
    }

    static bool _IsMsgPackData(const std::string& data)
    {
        return data.size() > 0 && !std::isprint(data[0]);
    }

    // Encrypted data is also binary, so functionally it is indistinguishable from MsgPack data.
    static bool _IsEncryptedData(const std::string& data)
    {
        return data.size() > 0 && !std::isprint(data[0]);
    }

    void _ClearRapidJsonBuffer()
    {
        // TODO resize smartly
        _prLoadEnvAlloc->Clear();
    }

    std::vector<KinBodyPtr> _vecbodies;     ///< all objects that are collidable (includes robots) sorted by env body index ascending order. Note that some element can be nullptr, and size of _vecbodies should be kept unchanged when body is removed from env. protected by _mutexInterfaces. [0] should always be kept null since 0 means no assignment.

    string_map<int> _mapBodyNameIndex; /// maps body name to env body index of bodies stored in _vecbodies sorted by name. used to lookup kin body by name. protected by _mutexInterfaces.
    string_map<int> _mapBodyIdIndex; /// maps body id to env body index of bodies stored in _vecbodies sorted by name. used to lookup kin body by name. protected by _mutexInterfaces

    std::set<int> _environmentIndexRecyclePool; ///< body indices which can be reused later, because kin bodies who had these id's previously are already removed from the environment. This is to prevent env id's from growing without bound when kin bodies are removed and added repeatedly. protected by _mutexInterfaces

    int _assignedBodySensorNameIdSuffix; // cache of suffix used to make body (including robot) and sensor name and id unique in env

    list< std::pair<ModuleBasePtr, std::string> > _listModules;     ///< modules loaded in the environment and the strings they were intialized with. Initialization strings are used for cloning. protectred by _mutexInterfaces
    list<SensorBasePtr> _listSensors;     ///< sensors loaded in the environment. protectred by _mutexInterfaces
    list<ViewerBasePtr> _listViewers;     ///< viewers loaded in the environment. protectred by _mutexInterfaces

    dReal _fDeltaSimTime;                    ///< delta time for simulate step
    uint64_t _nCurSimTime;                        ///< simulation time since the start of the environment
    uint64_t _nSimStartTime;
    int _nBodiesModifiedStamp;     ///< incremented every tiem bodies vector is modified

    CollisionCheckerBasePtr _pCurrentChecker;
    PhysicsEngineBasePtr _pPhysicsEngine;

    boost::shared_ptr<std::thread> _threadSimulation;                      ///< main loop for environment simulation

    mutable EnvironmentMutex _mutexEnvironment;          ///< protects internal data from multithreading issues
    mutable std::shared_timed_mutex _mutexInterfaces;     ///< lock when managing interfaces like _listOwnedInterfaces, _listModules as well as _vecbodies and supporting data such as _mapBodyNameIndex, _mapBodyIdIndex and _environmentIndexRecyclePool

    using ExclusiveLock = std::lock_guard< std::shared_timed_mutex >;
    using SharedLock = std::shared_lock< std::shared_timed_mutex >;

    mutable std::mutex _mutexInit;     ///< lock for destroying the environment

    vector<KinBody::BodyState> _vPublishedBodies; ///< protected by _mutexInterfaces
    string _homedirectory;
    std::pair<std::string, dReal> _unit; ///< unit name mm, cm, inches, m and the conversion for meters
    UnitInfo _unitInfo; ///< unitInfo that describes length unit, mass unit, time unit and angle unit

    UserDataPtr _handlegenericrobot, _handlegenerictrajectory, _handlemulticontroller, _handlegenericphysicsengine, _handlegenericcollisionchecker;

    list<InterfaceBasePtr> _listOwnedInterfaces; ///< protected by _mutexInterfaces

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
