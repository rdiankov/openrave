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
#include "plugindefs.h"
#include "configurationcachetree.h"

namespace configurationcache
{
using namespace OpenRAVE;

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
            // reset cache
        }
        _pintchecker->SetGeometryGroup(groupname);
    }

    virtual const std::string& GetGeometryGroup() {
        return _pintchecker->GetGeometryGroup();
    }

    virtual bool InitEnvironment() {
        // reset cache
        return _pintchecker->InitEnvironment();
    }

    virtual void DestroyEnvironment() {
        // reset cache
        _pintchecker->DestroyEnvironment();
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from free configurations)
        return _pintchecker->InitKinBody(pbody);
    }

    virtual void RemoveKinBody(KinBodyPtr pbody) {
        // reset cache for pbody (remove from collision configurations)
        _pintchecker->RemoveKinBody(pbody);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(pbody1, report);
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr report = CollisionReportPtr()) {
        return _pintchecker->CheckCollision(pbody1, pbody2, report);
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
        int affinedofs=0;
        sinput >> bodyname >> affinedofs;
        RobotBasePtr probot = GetEnv()->GetRobot(bodyname);
        if( !probot ) {
            return false;
        }
        _cache.reset(new ConfigurationCache(probot));
        RAVELOG_DEBUG_FORMAT("now tracking robot %s", bodyname);
        return true;
    }

    ConfigurationCachePtr _cache;
    CollisionCheckerBasePtr _pintchecker;
};

CollisionCheckerBasePtr CreateCacheCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new CacheCollisionChecker(penv, sinput));
}

};
