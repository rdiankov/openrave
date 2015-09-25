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
#include <Moby/EulerIntegrator.h>
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

    virtual bool InitEnvironment()
    {
        RAVELOG_INFO( "init Moby physics environment\n" );
        _space->SetSynchronizationCallback(boost::bind(&MobyPhysicsEngine::_SyncCallback, shared_physics(),_1));

        // +basic simulator
        _sim.reset(new Moby::Simulator());
        _sim->integrator = boost::shared_ptr<Moby::Integrator>(new Moby::EulerIntegrator());
        // -basic simulator

        // +simulator with constraints (limits and contact)
        //_sim.reset(new Moby::TimeSteppingSimulator());
        // -simulator with constraints (limits and contact)

        if(!_space->InitEnvironment(_sim)) {
            return false;
        }

        // if the gravity force is uninitialized create the reference        
        if( !_space->gravity ) {
            _space->gravity.reset( new Moby::GravityForce());
        }     
   
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies); 
        FOREACHC(itbody, vbodies) { 
            InitKinBody(*itbody);
        }

        SetGravity(_gravity);

        RAVELOG_INFO( "Moby physics environment created\n" );
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
       //_sim->reset();
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
    // For each of the following Get and Set methods velocity and torque return false until validated
    // In what reference frames should the velocities be get and set?  link local frame (not com frame) w.r.t. global frame
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
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        Moby::RigidBodyPtr rb = _space->GetLinkBody(plink);
        if( !!rb )
        {
            Ravelin::SVelocityd svel = rb->get_velocity();
        }
/*
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        if (!!rigidbody) {
            btVector3 pf = rigidbody->getLinearVelocity();
            linearvel = Vector(pf[0],pf[1],pf[2]);
            pf = rigidbody->getAngularVelocity();
            angularvel = Vector(pf[0],pf[1],pf[2]);
        }
        else {
            linearvel = angularvel = Vector(0,0,0);
        }
*/
        return false;
    }

    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, std::vector<std::pair<Vector,Vector> >& velocities)
    {
       

        return false;
    }

    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<dReal>& pJointVelocity)
    {
        
        return false;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<dReal>& pJointVelocity)
    {
       
        return false;
    }

    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques)
    {
       
       
        return false;
    
    }

    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
       
        return false;
    }

    virtual void SetGravity(const Vector& gravity)
    {     
        // if gravity has not been initialized create the reference
        if(!_space->gravity) 
        {
            _space->gravity.reset( new Moby::GravityForce());
        }

        // update the Moby gravity force object
        _space->gravity->gravity = Ravelin::Vector3d(gravity.x, gravity.y, gravity.z);
       
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

        // +dbg
        std::vector<Moby::DynamicBodyPtr> dbs = _sim->get_dynamic_bodies();
        RAVELOG_INFO(str(boost::format("dbs.size[%u]\n") % dbs.size()));
        for(std::vector<Moby::DynamicBodyPtr>::iterator it=dbs.begin(); it!=dbs.end();it++) 
        {
            Moby::RigidBodyPtr rb = boost::dynamic_pointer_cast<Moby::RigidBody>(*it);
            Ravelin::Pose3d pose = rb->get_pose();
            RAVELOG_INFO(str(boost::format("x[%f,%f,%f]\n") % pose.x.x() % pose.x.y() % pose.x.z())); 
        } 
        // -dbg

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            MobySpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            RAVELOG_INFO(str(boost::format("bodies.size[%u], links.size[%u]\n") % vbodies.size() % pinfo->vlinks.size())); 
            FOREACH(itlink, pinfo->vlinks) {
                Transform t = MobySpace::GetTransform((*itlink)->get_pose());
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());

                 // +dbg
                 double vt = _sim->current_time;
                 Ravelin::Pose3d pose = (*itlink)->get_pose();
                 //std::list<Moby::RecurrentForcePtr> lrf = (*itlink)->get_recurrent_forces();
                 //RAVELOG_INFO(str(boost::format("lrf.size[%u]\n") % lrf.size())); 
                 //for( std::list<Moby::RecurrentForcePtr>::iterator it=lrf.begin(); it!=lrf.end(); it++) 
                 //{
                 //    boost::shared_ptr<Moby::GravityForce> g = boost::dynamic_pointer_cast<Moby::GravityForce>(*it);
                 //    if( !g ) continue;
                 //    RAVELOG_INFO(str(boost::format("g[%f,%f,%f]\n") % g->gravity.x() % g->gravity.y() % g->gravity.z())); 
                 //}
                 RAVELOG_INFO(str(boost::format("vt[%f], x[%f,%f,%f]\n") % vt % pose.x.x() % pose.x.y() % pose.x.z())); 
                 // -dbg
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
};

