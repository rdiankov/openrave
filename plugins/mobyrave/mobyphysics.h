// -*- coding: utf-8 -*-
// Copyright (c) 2015 James Taylor, Rosen Diankov
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


#include <openrave/plugin.h>
#include <boost/bind.hpp>


//using namespace Moby;
using namespace OpenRAVE;
using namespace std;

class MobyPhysicsEngine : public PhysicsEngineBase
{

   /* inline boost::shared_ptr<MobyPhysicsEngine> shared_physics() {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine>(shared_from_this());
    }
    inline boost::shared_ptr<MobyPhysicsEngine const> shared_physics_const() const {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine const>(shared_from_this());
    }*/


public:
    MobyPhysicsEngine(EnvironmentBasePtr penv, std::istream& ss) : PhysicsEngineBase(penv) {
       
    }
    virtual ~MobyPhysicsEngine() {}
    
    bool SetStaticBodyTransform(ostream& sout, istream& sinput)
    
    {
        return true;
	
    }
    virtual bool InitEnvironment()
    {
        
        return true;
    }

    virtual void DestroyEnvironment()
    {
       
    }

     virtual bool InitKinBody(KinBodyPtr pbody)
    {
       
       // return !!pinfo;
    }


    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        
    }

    virtual bool SetPhysicsOptions(int physicsoptions)
    {
       // _options = physicsoptions;
        return true;
    }

    virtual int GetPhysicsOptions() const
    {
        //return _options;
    }

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) {
        return false;
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
       
        return false;
    }
    virtual bool SetLinkVelocities(KinBodyPtr pbody, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        
        return false;
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel)
    {
       
        return true;
    }

    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, std::vector<std::pair<Vector,Vector> >& velocities)
    {
       

        return true;
    }

    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<dReal>& pJointVelocity)
    {
        
        return true;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<dReal>& pJointVelocity)
    {
       
        return true;
    }

      virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques)
    {
       
       
        return true;
    
    }

    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
       
        return true;
    }

    virtual void SetGravity(const Vector& gravity)
    {
       _gravity = gravity;
    }

    virtual Vector GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
   
    }

private :

    Vector _gravity;
   

};



