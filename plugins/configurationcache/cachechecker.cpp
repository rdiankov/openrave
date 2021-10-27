// -*- coding: utf-8 -*-
// Copyright (C) 2014 Alejandro Perez & Rosen Diankov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "openraveplugindefs.h"
#include "configurationcachetree.h"

namespace configurationcache
{

class CacheCollisionChecker : public CollisionCheckerBase
{
public:
    CacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput) : CollisionCheckerBase(penv)
    {
        RegisterCommand("TrackRobotState",boost::bind(&CacheCollisionChecker::_TrackRobotStateCommand,this,_1,_2),
                        "set up the cache to track a body state. [bodyname affinedofs]");
        RegisterCommand("GetTrackedRobot",boost::bind(&CacheCollisionChecker::_GetTrackedRobotCommand,this,_1,_2),
                        "get the robot being tracked by the collisionchecker");
        RegisterCommand("GetCacheStatistics",boost::bind(&CacheCollisionChecker::_GetCacheStatisticsCommand,this,_1,_2),
                        "get the cache statistics: cachecollisions, cachehits");
        RegisterCommand("GetSelfCacheStatistics",boost::bind(&CacheCollisionChecker::_GetSelfCacheStatisticsCommand,this,_1,_2),
                        "get the self collision cache statistics: selfcachecollisions, selfcachehits");
        RegisterCommand("SetSelfCacheParameters",boost::bind(&CacheCollisionChecker::_SetSelfCacheParametersCommand,this,_1,_2),
                        "set the self collision cache parameters: collisionthreshold, freespacethreshold, insertiondistancemultiplier, base");
        RegisterCommand("SetCacheParameters",boost::bind(&CacheCollisionChecker::_SetCacheParametersCommand,this,_1,_2),
                        "set the collision cache parameters: collisionthreshold, freespacethreshold, insertiondistancemultiplier, base");
        RegisterCommand("ValidateCache",boost::bind(&CacheCollisionChecker::_ValidateCacheCommand,this,_1,_2),
                        "test the validity of the cache");
        RegisterCommand("ValidateSelfCache",boost::bind(&CacheCollisionChecker::_ValidateSelfCacheCommand,this,_1,_2),
                        "test the validity of the self collision cache");
        RegisterCommand("ResetCache",boost::bind(&CacheCollisionChecker::_ResetCacheCommand,this,_1,_2),
                        "reset collision cache");
        RegisterCommand("ResetSelfCache",boost::bind(&CacheCollisionChecker::_ResetSelfCacheCommand,this,_1,_2),
                        "reset self collision cache");
        RegisterCommand("UpdateCollisionConfigurations",boost::bind(&CacheCollisionChecker::_UpdateCollisionConfigurationsCommand,this,_1,_2),
                        "remove all nodes in collision with this body. [bodyname]");
        RegisterCommand("UpdateFreeConfigurations",boost::bind(&CacheCollisionChecker::_UpdateFreeConfigurationsCommand,this,_1,_2),
                        "remove all free nodes that overlap with this body. [bodyname]");
        RegisterCommand("SaveCache",boost::bind(&CacheCollisionChecker::_SaveCacheCommand,this,_1,_2),
                        "save self collision cache");
        RegisterCommand("LoadCache",boost::bind(&CacheCollisionChecker::_LoadCacheCommand,this,_1,_2),
                        "load self collision cache");
        RegisterCommand("GetCacheTimes",boost::bind(&CacheCollisionChecker::_GetCacheTimesCommand,this,_1,_2),
                        "get the cache times: insert, query, collision checking, load");
        std::string collisionname="ode";
        sinput >> collisionname;
        _pintchecker = RaveCreateCollisionChecker(GetEnv(), collisionname);
        OPENRAVE_ASSERT_FORMAT(!!_pintchecker, "internal checker %s is not valid", collisionname, ORE_Assert);
        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits = 0;

        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits = 0;

        __cachehash.resize(0);

        _stime = 0;
        _ftime = 0;
        _intime = 0;
        _querytime = 0;
        _loadtime = 0;
        _savetime = 0;
        _rawtime = 0;
        _resettime = 0;
        _selfintime = 0;
        _selfquerytime = 0;
        _selfrawtime = 0;

    }

    virtual ~CacheCollisionChecker() {
    }

    ConfigurationCachePtr GetCache()
    {
        return _cache;
    }

    ConfigurationCachePtr GetSelfCache()
    {
        return _selfcache;
    }

    virtual bool SetCollisionOptions(int collisionoptions)
    {
        return _pintchecker->SetCollisionOptions(collisionoptions);
    }

    virtual int GetCollisionOptions() const {
        return _pintchecker->GetCollisionOptions();
    }

    virtual void SetTolerance(dReal tolerance) {
        _pintchecker->SetTolerance(tolerance);
    }

    virtual void SetGeometryGroup(const std::string& groupname)
    {
        // reset cache if geometry group is different
        if(  groupname != _pintchecker->GetGeometryGroup() ) {
            if( !!_cache ) {
                _cache->Reset();
                //_cache.reset(new ConfigurationCache(GetRobot()));
            }
            if( !!_selfcache) {
                _selfcache->Reset();
                //_selfcache.reset(new ConfigurationCache(GetRobot(),false));
            }
        }
        _pintchecker->SetGeometryGroup(groupname);
    }

    virtual const std::string& GetGeometryGroup() const
    {
        return _pintchecker->GetGeometryGroup();
    }

    virtual bool InitEnvironment() {
        // reset cache?
        return _pintchecker->InitEnvironment();
    }

    virtual void DestroyEnvironment()
    {
        if( !!_cache ) {
            _cache->Reset();
        }
        if( !!_selfcache ) {
            _selfcache->Reset();
        }
        if( !!_pintchecker ) {
            _pintchecker->DestroyEnvironment();
        }
        _handleRobotDOFChange.reset();
        _probot.reset();
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        OPENRAVE_SHARED_PTR<CacheCollisionChecker const> clone = OPENRAVE_DYNAMIC_POINTER_CAST<CacheCollisionChecker const> (preference);

        DestroyEnvironment();

        if(!!clone->_pintchecker) {
            CollisionCheckerBasePtr p = RaveCreateCollisionChecker(GetEnv(),clone->_pintchecker->GetXMLId());
            p->Clone(clone->_pintchecker,cloningoptions);

            _pintchecker = p;
            _pintchecker->InitEnvironment();
        }
        else{
            _pintchecker.reset();
        }

        _strRobotName = clone->_strRobotName;
        _probot.reset(); // have to rest to force creating a new cache
        _probot = GetRobot();

        _cachedcollisionchecks=clone->_cachedcollisionchecks;
        _cachedcollisionhits=clone->_cachedcollisionhits;
        _cachedfreehits=clone->_cachedfreehits;

        _selfcachedcollisionchecks=clone->_selfcachedcollisionchecks;
        _selfcachedcollisionhits=clone->_selfcachedcollisionhits;
        _selfcachedfreehits = clone->_selfcachedfreehits;

    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from free configurations since body has been added)
        return _pintchecker->InitKinBody(pbody);
    }

    virtual void RemoveKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from collision configurations)?
        _pintchecker->RemoveKinBody(pbody);
    }

    /// \brief collisionchecker checks if there is a configuration in _cache within the threshold, and if so, uses that information, if not, runs standard collisioncheck and stores the result.
    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())
    {

        RobotBasePtr probot = GetRobot();

        // run standard collisioncheck if there is no cache
        if( !_cache || pbody1 != probot ) {
            _stime = utils::GetMilliTime();
            bool ccol = _pintchecker->CheckCollision(pbody1, report);
            _rawtime += utils::GetMilliTime()-_stime;
            return ccol;
        }
        if( !!report ) {
            report->Reset();
        }

        KinBody::LinkConstPtr robotlink, collidinglink;
        dReal closestdist=0;

        // see if cache contains the result, closestdist is used to determine if the configuration should be inserted into the cache
        _stime = utils::GetMilliTime();
        int ret = _cache->CheckCollision(robotlink, collidinglink, closestdist);
        _querytime += utils::GetMilliTime()-_stime;

        ++_cachedcollisionchecks;

        // print stats every now and then
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            if (_cachedcollisionchecks % 5000 == 0) {
                _ss.str(std::string());
                _ss << "insert " << _intime << "ms " << "query " << _querytime << "ms " << "raw " << _rawtime << "ms" << " size " << _cache->GetNumKnownNodes() << " hits " << _cachedcollisionhits+_cachedfreehits << "/" << _cachedcollisionchecks;

                RAVELOG_VERBOSE(_ss.str());
            }
        }

        // cache hit (collision)
        if( ret == 1 ) {
            ++_cachedcollisionhits;
            // in collision, create collision report
            if( !!report ) {
                report->plink1 = robotlink;
                report->plink2 = collidinglink;
            }
            return true;
        } // (free configuration)
        else if( ret == 0 ) {
            ++_cachedfreehits;
            // free space
            return false;
        }

        // cache miss
        if( !report ) {
            // create an empty collision report
            report.reset(new CollisionReport());
        }

        // raw collisioncheck
        _stime = utils::GetMilliTime();
        bool col = _pintchecker->CheckCollision(pbody1, report);
        _rawtime += utils::GetMilliTime()-_stime;


        _stime = utils::GetMilliTime();
        _cache->GetDOFValues(_dofvals);
        // insert collisioncheck result into cache
        _cache->InsertConfiguration(_dofvals, !col ? CollisionReportPtr() : report, closestdist);
        _intime += utils::GetMilliTime()-_stime;

        return col;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()) {
        bool col = _pintchecker->CheckCollision(pbody1, pbody2, report);
        return col;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(plink, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(plink1, plink2, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) {

        return _pintchecker->CheckCollision(plink, pbody, report);
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) {

        return _pintchecker->CheckCollision(plink, vbodyexcluded, vlinkexcluded, report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr report = CollisionReportPtr()) {

        return _pintchecker->CheckCollision(pbody, vbodyexcluded, vlinkexcluded, report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(ray, plink, report);
    }

    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(ray, pbody, report);
    }

    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(ray, report);
    }

    virtual bool CheckCollision(const TriMesh& trimesh, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(trimesh, pbody, report);
    }

    /// \brief collisionchecker checks if there is a configuration in _selfcache within the threshold, and if so, uses that information, if not, runs standard collisioncheck and stores the result.
    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) {

        RobotBasePtr probot = GetRobot();
        if( !_selfcache || pbody != probot ) {
            _stime = utils::GetMilliTime();
            bool scol = _pintchecker->CheckStandaloneSelfCollision(pbody, report);
            _selfrawtime += utils::GetMilliTime()-_stime;
            return scol;
        }
        if( !!report ) {
            report->Reset();
        }

        // see if cache contains the result
        KinBody::LinkConstPtr robotlink, collidinglink;
        dReal closestdist=0;

        _stime = utils::GetMilliTime();
        int ret = _selfcache->CheckCollision(robotlink, collidinglink, closestdist);
        _selfquerytime += utils::GetMilliTime()-_stime;

        ++_selfcachedcollisionchecks;

        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            if (_selfcachedcollisionchecks % 700 == 0) {
                _ss.str(std::string());
                _ss << "self-insert " << _selfintime << "ms " << "self-query " << _selfquerytime << "ms " << "self-raw " << _selfrawtime << "ms " << "load " << _loadtime << "ms " << "size " << _selfcache->GetNumKnownNodes() << " hits " << _selfcachedcollisionhits+_selfcachedfreehits << "/" << _selfcachedcollisionchecks;

                if (_selfrawtime > 0 && (_selfintime+_selfquerytime) > 0) {
                    _ss << " avg rawtime " << (_selfcachedcollisionchecks-(_selfcachedcollisionhits+_selfcachedfreehits))/_selfrawtime << "ms " << " avg cachetime " << (_selfcachedcollisionhits+_selfcachedfreehits)/(_selfquerytime+_selfintime) << "ms";
                }

                RAVELOG_VERBOSE(_ss.str());
            }
        }

        // save cache every other iteration if its size has increased by 1.5
        if (_selfcachedcollisionchecks % 4000 == 0) {
            if (_size*1.5 < _selfcache->GetNumKnownNodes()) {
                _selfcache->SaveCache(GetCacheHash());
                _size = _selfcache->GetNumKnownNodes();
            }
        }
        if( ret == 1 ) {
            ++_selfcachedcollisionhits;
            // in collision
            if( !!report ) {
                report->plink1 = robotlink;
                report->plink2 = collidinglink;
            }
            return true;
        }
        else if( ret == 0 ) {
            ++_selfcachedfreehits;
            // free space
            return false;
        }

        // cache miss
        if( !report ) {
            report.reset(new CollisionReport());
        }

        _stime = utils::GetMilliTime();
        bool col = _pintchecker->CheckStandaloneSelfCollision(pbody, report);
        _selfrawtime += utils::GetMilliTime()-_stime;

        _stime = utils::GetMilliTime();
        _selfcache->GetDOFValues(_dofvals);
        _selfcache->InsertConfiguration(_dofvals, !col ? CollisionReportPtr() : report, closestdist);
        _selfintime += utils::GetMilliTime()-_stime;

        return col;
    }

    virtual bool CheckStandaloneSelfCollision(KinBody::LinkConstPtr plink, CollisionReportPtr report = CollisionReportPtr()) {

        return _pintchecker->CheckStandaloneSelfCollision(plink, report);
    }

protected:
    virtual bool _TrackRobotStateCommand(std::ostream& sout, std::istream& sinput)
    {
        string bodyname;
        int affinedofs = 0;
        sinput >> bodyname >> affinedofs;
        _strRobotName = bodyname;
        _probot = GetRobot();
        if( !_probot ) {
            return false;
        }

        // if there is no cache, create one
        if (_selfcache->GetNumKnownNodes() == 0) {
            // _cache is the environment collision cache, envupdates is true, i.e., the cache will be updated on changes and it will not save/load
            _cache.reset(new ConfigurationCache(_probot));
            // _selfcache is the selfcollision cache, envupdates is false, i.e., the cache will not be updated when the environment changes it will save and load the cache
            _selfcache.reset(new ConfigurationCache(_probot, false)); //envupdates should be disabled for self collision cache

            _SetParams();
        }

        // check if a selfcache for this robot exists on this disk
        std::string fulldirname = RaveFindDatabaseFile(("selfcache."+GetCacheHash()));
        if (fulldirname != "" && _selfcache->GetNumKnownNodes() == 0) {
            _stime = utils::GetMilliTime();
            _selfcache->LoadCache(GetCacheHash(), GetEnv());
            _loadtime = utils::GetMilliTime()-_stime;
            _size = _selfcache->GetNumKnownNodes();
            RAVELOG_VERBOSE_FORMAT("Loaded %d configurations in %d ms from %s", _size%_loadtime%fulldirname);

            __cachehash = "";
        }

        RAVELOG_DEBUG_FORMAT("Now tracking robot %s", bodyname);

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;

        _numdofs = _probot->GetActiveDOF();
        _dofindices = _probot->GetActiveDOFIndices();

        // callback that resets cache when the robot's DOF are changed
        _handleRobotDOFChange = _probot->RegisterChangeCallback(KinBody::Prop_RobotActiveDOFs, boost::bind(&CacheCollisionChecker::_UpdateRobotDOF, this));

        return true;
    }

    virtual bool _GetCacheStatisticsCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _cachedcollisionchecks << " " << _cachedcollisionhits << " " << _cachedfreehits << " " << _cache->GetNumKnownNodes();

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        return true;
    }

    virtual bool _GetSelfCacheStatisticsCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _selfcachedcollisionchecks << " " << _selfcachedcollisionhits << " " << _selfcachedfreehits << " " << _selfcache->GetNumKnownNodes();

        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;
        return true;
    }

    virtual bool _GetCacheTimesCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << "insert " << _intime << "ms " << "query " << _querytime << "ms " << "raw " << _rawtime << "ms " << "self-insert " << _selfintime << "ms " << "self-query " << _selfquerytime << "ms " << "self-raw " << _selfrawtime << "ms " << "load " << _loadtime << "ms" << " hits " << _cachedcollisionhits+_cachedfreehits;

        _stime = 0;
        _ftime = 0;
        _intime = 0;
        _querytime = 0;
        _loadtime = 0;
        _savetime = 0;
        _rawtime = 0;
        _resettime = 0;
        _selfintime = 0;
        _selfquerytime = 0;
        _selfrawtime = 0;
        return true;
    }

    virtual bool _SetCacheParametersCommand(std::ostream& sout, std::istream& sinput)
    {

        dReal colthresh, freethresh, indist, base;

        sinput >> colthresh >> freethresh >> indist >> base;

        _cache->SetCollisionThresh(colthresh);
        _cache->SetFreeSpaceThresh(freethresh);
        _cache->SetInsertionDistanceMult(indist);
        _cache->SetBase(base);

        sout << " " << _cache->GetCollisionThresh() << " " << _cache->GetFreeSpaceThresh() << " " << _cache->GetInsertionDistanceMult() << " " << _cache->GetBase();
        return true;
    }

    virtual bool _SetSelfCacheParametersCommand(std::ostream& sout, std::istream& sinput)
    {

        dReal selfcolthresh, selffreethresh, selfindist, selfbase;

        sinput >> selfcolthresh >> selffreethresh >> selfindist >> selfbase;

        _selfcache->SetCollisionThresh(selfcolthresh);
        _selfcache->SetFreeSpaceThresh(selffreethresh);
        _selfcache->SetInsertionDistanceMult(selfindist);
        _selfcache->SetBase(selfbase);

        sout << " " << _selfcache->GetCollisionThresh() << " " << _selfcache->GetFreeSpaceThresh() << " " << _selfcache->GetInsertionDistanceMult() << " " << _selfcache->GetBase();
        return true;
    }

    virtual bool _ValidateCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _cache->Validate();
        return true;
    }

    virtual bool _ValidateSelfCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _selfcache->Validate();
        return true;
    }

    virtual bool _ResetCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        _cache->Reset();

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        return true;
    }

    virtual bool _ResetSelfCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        _selfcache->Reset();

        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;
        return true;
    }

    virtual bool _UpdateCollisionConfigurationsCommand(std::ostream& sout, std::istream& sinput)
    {
        string bodyname;
        sinput >> bodyname;

        _cache->UpdateCollisionConfigurations(GetEnv()->GetKinBody(bodyname));
        return true;

    }

    virtual bool _UpdateFreeConfigurationsCommand(std::ostream& sout, std::istream& sinput)
    {
        string bodyname;
        sinput >> bodyname;

        _cache->UpdateFreeConfigurations(GetEnv()->GetKinBody(bodyname));
        return true;

    }

    virtual bool _GetTrackedRobotCommand(std::ostream& sout, std::istream& sinput)
    {
        GetRobot();

        if (!!_probot) {
            sout << _probot->GetName();
        }
        else{
            sout << " ";
        }

        return true;
    }

    virtual bool _SaveCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        _selfcache->SaveCache(GetCacheHash());
        return true;
    }


    virtual bool _LoadCacheCommand(std::ostream& sout, std::istream& sinput)
    {
        _selfcache->LoadCache(GetCacheHash(), GetEnv());
        return true;
    }

    RobotBasePtr GetRobot()
    {
        if( !_probot && _strRobotName.size() > 0 ) {
            _probot = GetEnv()->GetRobot(_strRobotName);
            if( !!_probot ) {
                // initialized! so also initialize the cache
                _InitializeCache();
            }
        }
        return _probot;
    }

    /// \brief generate a string to be used to save/load selfcollision cache. hash considers: robot, grabbed bodies, parameters for the cache, and DOF
    std::string GetCacheHash()
    {
        _robothash = GetRobot()->GetRobotStructureHash();

        _vGrabbedBodies.resize(0);
        GetRobot()->GetGrabbed(_vGrabbedBodies);
        FOREACH(newbody, _vGrabbedBodies){
            _robothash += (*newbody)->GetKinematicsGeometryHash();
        }


        _oss << _selfcache->GetCollisionThresh() << _selfcache->GetFreeSpaceThresh() << _selfcache->GetInsertionDistanceMult() << _selfcache->GetBase();

        _robothash += _oss.str();

        __cachehash = utils::GetMD5HashString(_robothash);

        _oss.str(std::string());
        _oss << GetRobot()->GetDOF();
        __cachehash += _oss.str();

        _oss.str(std::string());

        return __cachehash;
    }


    // for testing, will remove soon (cloning collision checkers resets all parameters)
    void _SetParams()
    {
        _cache->SetCollisionThresh(0.1);
        _cache->SetFreeSpaceThresh(0.1);
        _cache->SetInsertionDistanceMult(0.1);
        _cache->SetBase(2.0);

        _selfcache->SetCollisionThresh(0.2);
        _selfcache->SetFreeSpaceThresh(0.3);
        _selfcache->SetInsertionDistanceMult(0.5);
        _selfcache->SetBase(1.8);
    }

    void _InitializeCache()
    {
        _cache.reset(new ConfigurationCache(_probot));
        _selfcache.reset(new ConfigurationCache(_probot, false)); //envupdates should be disabled for self collision cache

        _SetParams();

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;

        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;
    }

    void _UpdateRobotDOF()
    {
        // if DOF changed, reset environment cache
        if (_probot->GetActiveDOF() != _numdofs || _probot->GetActiveDOFIndices() != _dofindices)
        {
            RAVELOG_VERBOSE_FORMAT("Updating robot dofs, %d/%d",_numdofs%_probot->GetActiveDOF());
            _cache.reset(new ConfigurationCache(_probot));

            _numdofs = _probot->GetActiveDOF();
            _dofindices = _probot->GetActiveDOFIndices();

        }
    }

    std::vector<dReal> _dofvals;
    std::vector<KinBodyPtr> _vGrabbedBodies;
    std::vector<int> _dofindices;
    ConfigurationCachePtr _cache;
    ConfigurationCachePtr _selfcache;
    CollisionCheckerBasePtr _pintchecker;
    std::string _strRobotName; ///< the robot name to track
    std::string __cachehash;
    std::string _robothash;
    RobotBasePtr _probot; ///< robot pointer, shouldn't be used directly, use with GetRobot()
    int _numdofs;
    int _cachedcollisionchecks, _cachedcollisionhits, _cachedfreehits, _size;
    int _selfcachedcollisionchecks, _selfcachedcollisionhits, _selfcachedfreehits;
    uint64_t _stime, _ftime, _intime, _querytime, _loadtime, _savetime, _rawtime, _resettime, _selfintime, _selfquerytime, _selfrawtime;
    stringstream _ss;
    ostringstream _oss;

    UserDataPtr _handleRobotDOFChange;
};

CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new CacheCollisionChecker(penv, sinput));
}

};
