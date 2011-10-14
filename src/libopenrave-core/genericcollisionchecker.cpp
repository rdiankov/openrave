// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

namespace OpenRAVE {

class GenericCollisionChecker : public CollisionCheckerBase
{
public:
    GenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput) : CollisionCheckerBase(penv) {
    }
    virtual ~GenericCollisionChecker() {
    }

    virtual bool InitEnvironment() {
        return true;
    }
    virtual void DestroyEnvironment() {
    }

    virtual bool InitKinBody(KinBodyPtr pbody) {
        SetCollisionData(pbody, UserDataPtr()); return true;
    }
    virtual bool DestroyKinBody(KinBodyPtr pbody) {
        SetCollisionData(pbody, UserDataPtr()); return true;
    }
    virtual bool Enable(KinBodyConstPtr pbody, bool bEnable) {
        return true;
    }
    virtual bool EnableLink(KinBody::LinkConstPtr pbody, bool bEnable) {
        return true;
    }

    virtual bool SetCollisionOptions(int collisionoptions) {
        return true;
    }
    virtual int GetCollisionOptions() const {
        return 0;
    }

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput) {
        return true;
    }

    virtual bool CheckCollision(KinBodyConstPtr pbody1, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBodyConstPtr pbody1, KinBodyConstPtr pbody2, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink1, KinBody::LinkConstPtr plink2, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBody::LinkConstPtr plink, KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }

    virtual bool CheckCollision(KinBody::LinkConstPtr plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(KinBodyConstPtr pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, CollisionReportPtr) {
        return false;
    }

    virtual bool CheckCollision(const RAY& ray, KinBody::LinkConstPtr plink, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, CollisionReportPtr) {
        return false;
    }
    virtual bool CheckSelfCollision(KinBodyConstPtr pbody, CollisionReportPtr) {
        return false;
    }

    virtual void SetTolerance(dReal tolerance) {
    }
};

CollisionCheckerBasePtr CreateGenericCollisionChecker(EnvironmentBasePtr penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new GenericCollisionChecker(penv,sinput));
}

}
