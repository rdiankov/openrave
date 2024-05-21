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
    GenericCollisionChecker(const EnvironmentBasePtr& penv, std::istream& sinput) : CollisionCheckerBase(penv) {
    }
    virtual ~GenericCollisionChecker() {
    }

    void Clone(const InterfaceBaseConstPtr& preference, int cloningoptions)
    {
        CollisionCheckerBase::Clone(preference, cloningoptions);
        boost::shared_ptr<GenericCollisionChecker const > r = boost::dynamic_pointer_cast<GenericCollisionChecker const>(preference);
        _geometrygroup = r->_geometrygroup;
    }

    virtual bool InitEnvironment() {
        return true;
    }
    virtual void DestroyEnvironment() {
    }

    virtual bool InitKinBody(const KinBodyPtr& pbody) {
        return true;
    }
    virtual void RemoveKinBody(const KinBodyPtr& pbody) {
    }
    virtual bool Enable(const KinBodyConstPtr& pbody, bool bEnable) {
        return true;
    }
    virtual bool EnableLink(const KinBody::LinkConstPtr& pbody, bool bEnable) {
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

    virtual bool CheckCollision(const KinBodyConstPtr& pbody1, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const KinBodyConstPtr& pbody1, const KinBodyConstPtr& pbody2, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const KinBody::LinkConstPtr& plink, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const KinBody::LinkConstPtr& plink1, const KinBody::LinkConstPtr& plink2, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const KinBody::LinkConstPtr& plink, const KinBodyConstPtr& pbody, const CollisionReportPtr&) {
        return false;
    }

    virtual bool CheckCollision(const KinBody::LinkConstPtr& plink, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const KinBodyConstPtr& pbody, const std::vector<KinBodyConstPtr>& vbodyexcluded, const std::vector<KinBody::LinkConstPtr>& vlinkexcluded, const CollisionReportPtr&) {
        return false;
    }

    virtual bool CheckCollision(const RAY& ray, const KinBody::LinkConstPtr& plink, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, const KinBodyConstPtr& pbody, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckCollision(const RAY& ray, const CollisionReportPtr&) {
        return false;
    }

    virtual bool CheckCollision(const TriMesh& trimesh, const KinBodyConstPtr& pbody, const CollisionReportPtr&) {
        return false;
    }
    
    virtual bool CheckStandaloneSelfCollision(const KinBodyConstPtr& pbody, const CollisionReportPtr&) {
        return false;
    }
    virtual bool CheckStandaloneSelfCollision(const KinBody::LinkConstPtr& pbody, const CollisionReportPtr&) {
        return false;
    }

    virtual void SetTolerance(dReal tolerance) {
    }

    virtual void SetGeometryGroup(const std::string& groupname)
    {
        _geometrygroup = groupname;
    }

    virtual const std::string& GetGeometryGroup() const
    {
        return _geometrygroup;
    }

    virtual bool SetBodyGeometryGroup(const KinBodyConstPtr& pbody, const std::string& groupname) {
        return false;
    }

    virtual const std::string& GetBodyGeometryGroup(const KinBodyConstPtr& pbody) const {
        return _geometrygroup;
    }

    std::string _geometrygroup;
};

CollisionCheckerBasePtr CreateGenericCollisionChecker(const EnvironmentBasePtr& penv, std::istream& sinput)
{
    return CollisionCheckerBasePtr(new GenericCollisionChecker(penv,sinput));
}

}
