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
//#include <boost/bind.hpp>

#include "mobyspace.h"

#include <Moby/TimeSteppingSimulator.h>
#include <Moby/GravityForce.h>

//using namespace Moby;
//using namespace OpenRAVE;
//using namespace std;

class MobyPhysicsEngine : public PhysicsEngineBase
{

    inline boost::shared_ptr<MobyPhysicsEngine> shared_physics() {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine>(shared_from_this());
    }

    inline boost::shared_ptr<MobyPhysicsEngine const> shared_physics_const() const {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine const>(shared_from_this());
    }

public:
    MobyPhysicsEngine(EnvironmentBasePtr penv, std::istream& ss) : PhysicsEngineBase(penv), _StepSize(0.001), _space(new MobySpace(penv, GetPhysicsInfo, true)) 
    {
        // TODO: map any environment settings into the simulator settings

    }
    virtual ~MobyPhysicsEngine() {}

/*  
    // artifact of BulletPhysics template.  Necessary for Moby?  
    bool SetStaticBodyTransform(ostream& sout, istream& sinput)
    
    {
        return true;
	
    }
*/

    virtual bool InitEnvironment()
    {
        RAVELOG_INFO( "init Moby physics environment\n" );
        //std::cout << "initializing Moby simulation" << std::endl;
        _space->SetSynchronizationCallback(boost::bind(&MobyPhysicsEngine::_SyncCallback, shared_physics(),_1));

        _sim.reset(new Moby::TimeSteppingSimulator());
        //_sim.reset(new Moby::Simulator());      // getting a null pointer except on the integrator using Sim class

        RAVELOG_INFO( "Moby simulator object created\n" );
        if(!_space->InitEnvironment(_sim)) {
            return false;
        }        
        
        RAVELOG_INFO( "Initializing bodies\n" );
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies); 
        FOREACHC(itbody, vbodies) { 
            InitKinBody(*itbody);
        }

        RAVELOG_INFO( "setting gravity\n" );
        SetGravity(_gravity);

        return true;
    }

    virtual void DestroyEnvironment()
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            (*itbody)->RemoveUserData("mobyphysics");
        }
        RAVELOG_INFO( "destroy Moby physics environment\n" );
        _space->DestroyEnvironment();

       // clean up any other resources here
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        MobySpace::KinBodyInfoPtr pinfo = _space->InitKinBody(pbody);
        pbody->SetUserData("mobyphysics", pinfo);

        // set any body specific parameters here

        return !!pinfo;
    }


    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            pbody->RemoveUserData("mobyphysics");
        }
    }

    virtual bool SetPhysicsOptions(int physicsoptions)
    {
        _options = physicsoptions;
        return true;
    }

    virtual int GetPhysicsOptions() const
    {
        return _options;
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
        // if this is the first time setting gravity, initialize the Moby
        //  gravity force object and assign it to all bodies in the sim
        if( !_mobyGravity ) {
            _mobyGravity = boost::shared_ptr<Moby::GravityForce>(new Moby::GravityForce());

            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            FOREACHC(itbody, vbodies) {
                MobySpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
                FOREACH(itlink, pinfo->vlinks) {
                    (*itlink)->get_recurrent_forces().push_back(_mobyGravity);
                }
            }
        }

        // update the Moby gravity force object
        _mobyGravity->gravity = Ravelin::Vector3d(gravity.x, gravity.y, gravity.z);
       
        // update the local OpenRave gravity variable  
        _gravity = gravity;
    }

    virtual Vector GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
        // The requested fTimeElapsed may be large in comparison to a 
        // an integration step size that is accurate.  Current 
        // configuration dictates an fTimeElapsed of 1ms which is at 
        // the upper bound of accuracy for integration steps.  Some
        // logic should be emplaced to select for an accurate 
        // integration step if fTimeElapsed is set larger than 1ms
        // For now, assume fTimeElapsed is a resonable value for 
        // accurate integration

/*
        dReal endOfStepTime = ?;
        dReal t = ?;
        do {
            // compute the least sized step requested
            //dReal actualStep = fTimeElapsed < _StepSize ? fTimeElapsed : _StepSize;

            // if actualStep is equal to _StepSize, there may be some residual time 
            // after a number of steps so need to compute the last fragment of time 
            // as accurately as possible and therefore the above actualStep 
            // computation is too simplistic

            _sim->step( actualStep );
            t += actualStep;        // naive fp adding will have error here
        } while(t<endOfStepTime);
*/
        

        //RAVELOG_INFO( "attempting to step\n" );
        _sim->step(fTimeElapsed);

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            MobySpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            FOREACH(itlink, pinfo->vlinks) {
                Transform t = MobySpace::GetTransform((*itlink)->get_pose());
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());
            }
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
        //RAVELOG_INFO( "completed step\n" );
    }

    dReal _StepSize;
    Vector _gravity;

private:
    static MobySpace::KinBodyInfoPtr GetPhysicsInfo(KinBodyConstPtr pbody)
    {
        return boost::dynamic_pointer_cast<MobySpace::KinBodyInfo>(pbody->GetUserData("mobyphysics"));
    }

    void _SyncCallback(MobySpace::KinBodyInfoConstPtr pinfo)
    {
        Ravelin::SVelocityd zerov = Ravelin::SVelocityd::zero(Moby::GLOBAL);

        // reset dynamics
        FOREACH(itlink, pinfo->vlinks) {
            (*itlink)->set_velocity(zerov);
        }
    }

    int _options;
    boost::shared_ptr<MobySpace> _space;
    boost::shared_ptr<Moby::Simulator> _sim; 
    boost::shared_ptr<Moby::GravityForce> _mobyGravity;
};

