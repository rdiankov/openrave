// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
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
#ifndef RAVE_COLLISION_ODE
#define RAVE_COLLISION_ODE

#include "odespace.h"

class ODECollisionChecker : public OpenRAVE::CollisionCheckerBase
{
public:
    ODECollisionChecker(OpenRAVE::EnvironmentBase* penv);
    ~ODECollisionChecker();
    
    virtual bool InitEnvironment();
    virtual void DestroyEnvironment();
    virtual bool InitKinBody(KinBody* pbody);
    virtual bool DestroyKinBody(KinBody* pbody);

    virtual bool SetCollisionOptions(int collisionoptions);
    virtual int GetCollisionOptions() const;

    virtual bool SetCollisionOptions(std::ostream& sout, std::istream& sinput);

    virtual bool Enable(const KinBody* pbody, bool bEnable);
    virtual bool EnableLink(const KinBody::Link* plink, bool bEnable);

    virtual bool CheckCollision(const KinBody* pbody1, COLLISIONREPORT*);
    virtual bool CheckCollision(const KinBody* pbody1, const KinBody* pbody2, COLLISIONREPORT*);
    virtual bool CheckCollision(const KinBody::Link* plink, COLLISIONREPORT*);
    virtual bool CheckCollision(const KinBody::Link* plink1, const KinBody::Link* plink2, COLLISIONREPORT*);
    virtual bool CheckCollision(const KinBody::Link* plink, const KinBody* pbody, COLLISIONREPORT*);
    
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, COLLISIONREPORT*);
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, COLLISIONREPORT*);
    
    virtual bool CheckCollision(const RAY& ray, const KinBody::Link* plink, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const RAY& ray, const KinBody* pbody, COLLISIONREPORT* pReport);
    virtual bool CheckCollision(const RAY& ray, COLLISIONREPORT* pReport);
	
    //tolerance check
    virtual bool CheckCollision(const KinBody::Link* plink, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, OpenRAVE::dReal tolerance);
    virtual bool CheckCollision(const KinBody* pbody, const std::set<KinBody*>& vbodyexcluded, const std::set<KinBody::Link*>& vlinkexcluded, OpenRAVE::dReal tolerance);

    virtual bool CheckSelfCollision(const KinBody* pbody, COLLISIONREPORT* pReport)
    {
        if( pbody == NULL || pbody->GetLinks().size() <= 1 )
            return false;

        // check collision, ignore adjacent bodies
        FOREACHC(itset, pbody->GetNonAdjacentLinks()) {
            assert( (*itset&0xffff) < (int)pbody->GetLinks().size() && (*itset>>16) < (int)pbody->GetLinks().size() );
            if( GetEnv()->CheckCollision(pbody->GetLinks()[*itset&0xffff], pbody->GetLinks()[*itset>>16], pReport) ) {
                RAVELOG_VERBOSEA("selfcol %S, Links %S %S are colliding\n", pbody->GetName(), pbody->GetLinks()[*itset&0xffff]->GetName(), pbody->GetLinks()[*itset>>16]->GetName());
                return true;
            }
        }

        return false;
    }

private:
    static void* GetCollisionInfo(const KinBody* pbody) { return pbody->GetCollisionData(); }

    static void KinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2);
    void _KinBodyCollisionCallback (dGeomID o1, dGeomID o2);
    static void KinBodyKinBodyCollisionCallback (void *data, dGeomID o1, dGeomID o2);
    void _KinBodyKinBodyCollisionCallback (dGeomID o1, dGeomID o2);
    static void LinkCollisionCallback (void *data, dGeomID o1, dGeomID o2);
    void _LinkCollisionCallback (dGeomID o1, dGeomID o2);
    static void RayCollisionCallback (void *data, dGeomID o1, dGeomID o2);
    void _RayCollisionCallback (dGeomID o1, dGeomID o2, dReal fmaxdist);

    int _options;
    dGeomID geomray; // used for all ray tests
    ODESpace odespace;

    // collision specific state
    bool _bCollision;
    COLLISIONREPORT* _pCurrentReport;
    const KinBody* _pbody;
    const KinBody::Link* _plink;
};

#endif
