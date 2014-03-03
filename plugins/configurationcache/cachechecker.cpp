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
        RegisterCommand("UpdateCollisionConfigurations",boost::bind(&CacheCollisionChecker::_UpdateCollisionConfigurations,this,_1,_2),
                        "remove all nodes in collision with this body. [bodyname]");
        RegisterCommand("UpdateFreeConfigurations",boost::bind(&CacheCollisionChecker::_UpdateFreeConfigurations,this,_1,_2),
                        "remove all free nodes that overlap with this body. [bodyname]");

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
        if( groupname != _pintchecker->GetGeometryGroup() ) {
            if( !!_cache ) {
                _cache->Reset();
            }
            if( !!_selfcache) {
                _selfcache->Reset();
            }
        }
        _pintchecker->SetGeometryGroup(groupname);
    }

    virtual const std::string& GetGeometryGroup() {
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
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<CacheCollisionChecker const> clone = boost::dynamic_pointer_cast<CacheCollisionChecker const> (preference);
        
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
        _probot = GetRobot();

        _cachedcollisionchecks=clone->_cachedcollisionchecks;
        _cachedcollisionhits=clone->_cachedcollisionhits;
        _cachedfreehits=clone->_cachedfreehits;

        _selfcachedcollisionchecks=clone->_selfcachedcollisionchecks;
        _selfcachedcollisionhits=clone->_selfcachedcollisionhits;
        _selfcachedfreehits = clone->_selfcachedfreehits;

        //RAVELOG_DEBUG("Cloning cache collision checker\n");
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from free configurations since body has been added)
        return _pintchecker->InitKinBody(pbody);
    }

    virtual void RemoveKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from collision configurations)?
        _pintchecker->RemoveKinBody(pbody);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr())
    {
        RobotBasePtr probot = GetRobot();
        if( !_cache || pbody1 != _cache->GetRobot() ) {
            return _pintchecker->CheckCollision(pbody1, report);
        }
        if( !!report ) {
            report->Reset();
        }

        // see if cache contains the result
        KinBody::LinkConstPtr robotlink, collidinglink;
        dReal closestdist=0;
        int ret = _cache->CheckCollision(robotlink, collidinglink, closestdist);

        ++_cachedcollisionchecks;
        if( ret == 1 ) {
            ++_cachedcollisionhits;
            // in collision
            if( !!report ) {
                report->plink1 = robotlink;
                report->plink2 = collidinglink;
                report->numCols = 1;
            }
            return true;
        }
        else if( ret == 0 ) {
            ++_cachedfreehits;
            // free space
            return false;
        }

        // cache miss
        if( !report ) {
            report.reset(new CollisionReport());
        }
        bool col = _pintchecker->CheckCollision(pbody1, report);
        _cache->GetDOFValues(_dofvals);
        _cache->InsertConfiguration(_dofvals, !col ? CollisionReportPtr() : report, closestdist);
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

    virtual bool CheckStandaloneSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr()) {

        RobotBasePtr probot = GetRobot();
        if( !_selfcache || pbody != _selfcache->GetRobot() ) {
            return _pintchecker->CheckStandaloneSelfCollision(pbody, report);
        }
        if( !!report ) {
            report->Reset();
        }

        // see if cache contains the result
        KinBody::LinkConstPtr robotlink, collidinglink;
        dReal closestdist=0;
        int ret = _selfcache->CheckCollision(robotlink, collidinglink, closestdist);

        ++_selfcachedcollisionchecks;
        if( ret == 1 ) {
            ++_selfcachedcollisionhits;
            // in collision
            if( !!report ) {
                report->plink1 = robotlink;
                report->plink2 = collidinglink;
                report->numCols = 1;
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
        bool col = _pintchecker->CheckStandaloneSelfCollision(pbody, report);
        _selfcache->GetDOFValues(_dofvals);
        _selfcache->InsertConfiguration(_dofvals, !col ? CollisionReportPtr() : report, closestdist);
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
        // always recreate?
        _cache.reset(new ConfigurationCache(_probot));
        _selfcache.reset(new ConfigurationCache(_probot, false)); //envupdates should be disabled for self collision cache

        _SetParams();

        RAVELOG_DEBUG_FORMAT("Now tracking robot %s", bodyname);

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;

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

    virtual bool _UpdateCollisionConfigurations(std::ostream& sout, std::istream& sinput)
    {
        string bodyname;
        sinput >> bodyname;

        _cache->UpdateCollisionConfigurations(GetEnv()->GetKinBody(bodyname));
        return true;

    }

    virtual bool _UpdateFreeConfigurations(std::ostream& sout, std::istream& sinput)
    {
        string bodyname;
        sinput >> bodyname;

        _cache->UpdateFreeConfigurations(GetEnv()->GetKinBody(bodyname));
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

    // for testing, will remove soon (cloning collision checkers resets all parameters)
    void _SetParams()
    {
        _cache->SetCollisionThresh(0.3);
        _cache->SetFreeSpaceThresh(0.3);
        _cache->SetInsertionDistanceMult(0.5);
        _cache->SetBase(1.6);

        _selfcache->SetCollisionThresh(0.1);
        _selfcache->SetFreeSpaceThresh(0.3);
        _selfcache->SetInsertionDistanceMult(0.5);
        _selfcache->SetBase(1.5);
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

    std::vector<dReal> _dofvals;
    ConfigurationCachePtr _cache;
    ConfigurationCachePtr _selfcache;
    CollisionCheckerBasePtr _pintchecker;
    std::string _strRobotName; ///< the robot name to track
    RobotBasePtr _probot; ///< robot pointer, shouldn't be used directly, use with GetRobot()
    int _cachedcollisionchecks, _cachedcollisionhits, _cachedfreehits;
    int _selfcachedcollisionchecks, _selfcachedcollisionhits, _selfcachedfreehits;
};

CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new CacheCollisionChecker(penv, sinput));
}

};
