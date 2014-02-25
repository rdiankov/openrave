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
        // TODO Reset command
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

    virtual void DestroyEnvironment() {
        if( !!_cache ) {

            _cache->Reset();
        }

        if( !!_selfcache ) {
            _selfcache->Reset();
        }

        _pintchecker->DestroyEnvironment();
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from free configurations since body has been added)
        return _pintchecker->InitKinBody(pbody);
    }

    virtual void RemoveKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from collision configurations)?
        _pintchecker->RemoveKinBody(pbody);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()) {

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
        //return _pintchecker->CheckStandaloneSelfCollision(pbody, report);

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
        _probot = GetEnv()->GetRobot(bodyname);
        if( !_probot ) {
            return false;
        }
        // always recreate?
        _cache.reset(new ConfigurationCache(_probot));
        _selfcache.reset(new ConfigurationCache(_probot));
        RAVELOG_DEBUG_FORMAT("Now tracking robot %s", bodyname);
        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        return true;
    }

    virtual bool _GetCacheStatisticsCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _cachedcollisionchecks << " " << _cachedcollisionhits << " " << _cachedfreehits << " " << _cache->GetNumNodes();

        _cachedcollisionchecks=0;
        _cachedcollisionhits=0;
        _cachedfreehits=0;
        return true;
    }

    virtual bool _GetSelfCacheStatisticsCommand(std::ostream& sout, std::istream& sinput)
    {
        sout << _selfcachedcollisionchecks << " " << _selfcachedcollisionhits << " " << _selfcachedfreehits << " " << _selfcache->GetNumNodes();

        _selfcachedcollisionchecks=0;
        _selfcachedcollisionhits=0;
        _selfcachedfreehits=0;
        return true;
    }

    std::vector<dReal> _dofvals;
    ConfigurationCachePtr _cache;
    ConfigurationCachePtr _selfcache;
    CollisionCheckerBasePtr _pintchecker;
    RobotBasePtr _probot;
    int _cachedcollisionchecks, _cachedcollisionhits, _cachedfreehits;
    int _selfcachedcollisionchecks, _selfcachedcollisionhits, _selfcachedfreehits;
};

CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new CacheCollisionChecker(penv, sinput));
}

};
