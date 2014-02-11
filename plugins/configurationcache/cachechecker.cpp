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

        std::string collisionname="ode";
        sinput >> collisionname;
        _pintchecker = RaveCreateCollisionChecker(GetEnv(), collisionname);
        OPENRAVE_ASSERT_FORMAT(!!_pintchecker, "internal checker %s is not valid", collisionname, ORE_Assert);
        _cachedcollisionchecks = 0;
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
        // comments:
        // everytime there is a collision, the update stamp gets updated
        // cache should be able to tell when a collision checking procedure is for an 'edge', i.e., from qi to qf, and store the information

        // see if cache contains the result
        KinBody::LinkConstPtr robotlink, collidinglink;
        dReal closestdist=0;
        int ret = _cache->CheckCollision(robotlink, collidinglink, closestdist);
        ++_cachedcollisionchecks;
        if( ret > 1 ) {
            // in collision
            if( !!report ) {
                report->plink1 = robotlink;
                report->plink2 = collidinglink;
                report->numCols = 1;
            }
            return true;
        }
        else if( ret == 0 ) {
            // free space
            return false;
        }

        if( !report ) {
            report.reset(new CollisionReport());
        }
        bool col = _pintchecker->CheckCollision(pbody1, report);
        
        if( col ) {
            _cache->GetDOFValues(_dofvals);
            _cache->InsertConfiguration(_dofvals, CollisionReportPtr(), closestdist);
        }
        else {
            // if not, compute before inserting
            _cache->InsertConfiguration(_dofvals, report, closestdist);
        }
//        else{
//            int csize = _cache->GetSize();
//            RAVELOG_DEBUG_FORMAT("cache size %d cached ccs %d\n",csize%_cachedcollisionchecks);
//        }
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
        return _pintchecker->CheckStandaloneSelfCollision(pbody, report);
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
        RAVELOG_WARN_FORMAT("Now tracking robot %s", bodyname);
        return true;
    }

    std::vector<dReal> _dofvals;
    ConfigurationCachePtr _cache;
    CollisionCheckerBasePtr _pintchecker;
    RobotBasePtr _probot;
    int _cachedcollisionchecks;
};

CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new CacheCollisionChecker(penv, sinput));
}

};
