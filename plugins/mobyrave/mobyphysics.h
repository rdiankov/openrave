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

#include "mobyspace.h"

#include <Moby/TimeSteppingSimulator.h>
#include <Moby/EulerIntegrator.h>
#include <Moby/GravityForce.h>

class MobyPhysics : public PhysicsEngineBase
{

    inline boost::shared_ptr<MobyPhysics> shared_physics() {
        return boost::static_pointer_cast<MobyPhysics>(shared_from_this());
    }

    inline boost::shared_ptr<MobyPhysics const> shared_physics_const() const {
        return boost::static_pointer_cast<MobyPhysics const>(shared_from_this());
    }

    class PhysicsPropertiesXMLReader : public BaseXMLReader
    {
    public:
        PhysicsPropertiesXMLReader(boost::shared_ptr<MobyPhysics> physics, const AttributesList& atts) : _physics(physics) {
        }

        string robot_id;

        virtual ProcessElement startElement(const string& name, const AttributesList& atts) {
            if( !!_pcurreader ) 
            {
                if( _pcurreader->startElement(name,atts) == PE_Support ) 
                {
                    return PE_Support;
                }
                return PE_Ignore;
            }

            if( find(GetTags().begin(),GetTags().end(),name) == GetTags().end() ) 
            {
                return PE_Pass;
            }

            _ss.str("");
            return PE_Support;
        }

        virtual bool endElement(const string& name)
        {
            if( name == "mobyphysics" ) 
            {
                return true;
            }
	    else if( name == "gravity" ) 
            {
                Vector g;

                _ss >> g[0] >> g[1] >> g[2];
        
                _physics->SetGravity(g);
            }
            else 
            {
                RAVELOG_ERROR("unknown field %s\n", name.c_str());
            }

            if( !_ss ) 
            {
                RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
            }

            return false;
        }

        virtual void characters(const string& ch)
        {
            if( !!_pcurreader ) 
            {
                _pcurreader->characters(ch);
            }
            else 
            {
                _ss.clear();
                _ss << ch;
            }
        }

        static const boost::array<string, 1>& GetTags() 
        {
            static const boost::array<string, 1> tags = {{"gravity"}};
                return tags;
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<MobyPhysics> _physics;
        stringstream _ss;
    };

public:

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
    	return BaseXMLReaderPtr(new PhysicsPropertiesXMLReader(boost::dynamic_pointer_cast<MobyPhysics>(ptr),atts));
    }

    MobyPhysics(EnvironmentBasePtr penv, istream& sinput) : PhysicsEngineBase(penv), _StepSize(0.001), _space(new MobySpace(penv, GetPhysicsInfo, true)) 
    {
	stringstream ss;
	__description = ":Interface Authors: James Taylor and Rosen Diankov\n\nInterface to `Moby Physics Engine <https://github.com/PositronicsLab/Moby/>`_\n";

        FOREACHC(it, PhysicsPropertiesXMLReader::GetTags()) {
            ss << "**" << *it << "**, ";
        }
        ss << "\n\n";
        RAVELOG_INFO( "processed xml\n" );
    }

    virtual ~MobyPhysics() 
    {

    }

    virtual bool InitEnvironment()
    {
        RAVELOG_INFO( "init Moby physics environment\n" );
        _space->SetSynchronizationCallback(boost::bind(&MobyPhysics::_SyncCallback, shared_physics(),_1));

        // +basic simulator
        //_sim.reset(new Moby::Simulator());
        //_sim->integrator = boost::shared_ptr<Moby::Integrator>(new Moby::EulerIntegrator());
        // -basic simulator

        // +simulator with constraints (limits and contact)
        _sim.reset(new Moby::TimeSteppingSimulator());
        // -simulator with constraints (limits and contact)

        if(!_space->InitEnvironment(_sim)) {
            return false;
        }

        // if the gravity force is uninitialized create the reference        
        if( !_space->_gravity ) {
            _space->_gravity.reset( new Moby::GravityForce());
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

    virtual bool SetPhysicsOptions(ostream& sout, istream& sinput) {
        return false;
    }

    // Note: this implementation is only additive
    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        Moby::RigidBodyPtr body = _space->GetLinkBody(plink);
        boost::shared_ptr<Ravelin::Pose3d> pose(new Ravelin::Pose3d(Ravelin::Quatd(0,0,0,1), _space->GetRavelinOrigin(position), Moby::GLOBAL));
        _space->AddImpulse(body, _space->GetRavelinSForce(force, Vector(0,0,0), pose));

        return true;
    }

    // Note: velocities w.r.t to the body's inertial reference frame
    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        Moby::RigidBodyPtr body = _space->GetLinkBody(plink);
        if( !body )
        {
            return false;
        }

        Ravelin::SVelocityd v(angularvel[0],angularvel[1],angularvel[2],linearvel[0],linearvel[1],linearvel[2], body->get_inertial_pose());
 
        _space->SetVelocity(body, v);

        return true;
    }

    // Note: velocities w.r.t to the respective body's inertial reference frame
    virtual bool SetLinkVelocities(KinBodyPtr pbody, const vector<pair<Vector,Vector> >& velocities)
    {
        _space->Synchronize(pbody);
        FOREACHC(itlink, pbody->GetLinks()) 
        {
            int idx = (*itlink)->GetIndex();
            Moby::RigidBodyPtr body = _space->GetLinkBody(*itlink);
            if(!!body) 
            {
                Vector omega = velocities.at(idx).first;
                Vector dx = velocities.at(idx).second;

                Ravelin::SVelocityd v(omega[0],omega[1],omega[2],dx[0],dx[1],dx[2], body->get_inertial_pose());

                _space->SetVelocity(body, v);
            }
        }

        return true;
    }

    // Note: w.r.t to what reference frame?
    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        Moby::RigidBodyPtr body = _space->GetLinkBody(plink);
        if( !body )
        {
            return false;
        }

        Ravelin::SVelocityd svel = body->get_velocity();
        Ravelin::Vector3d dx = svel.get_linear();
        Ravelin::Vector3d omega = svel.get_angular();

        linearvel = Vector(dx[0],dx[1],dx[2]);
        angularvel = Vector(omega[0],omega[1],omega[2]);

        return true;
    }

    // Note: w.r.t to what reference frame?
    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, vector<pair<Vector,Vector> >& velocities)
    {
        _space->Synchronize(pbody);
        velocities.resize(0);
        velocities.resize(pbody->GetLinks().size());

        FOREACHC(itlink, pbody->GetLinks()) 
        {
            Moby::RigidBodyPtr body = _space->GetLinkBody(*itlink);
            if(!!body) 
            {
                Ravelin::SVelocityd svel = body->get_velocity();
                Ravelin::Vector3d dx = svel.get_linear();
                Ravelin::Vector3d omega = svel.get_angular();

                velocities.at((*itlink)->GetIndex()).first = Vector(dx[0],dx[1],dx[2]);
                velocities.at((*itlink)->GetIndex()).second = Vector(omega[0],omega[1],omega[2]);
            }
        }

        return true;
    }

    // Note: neither in current physicsengine interface nor a python binding, came from bulletphysics
    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const vector<dReal>& pJointVelocity)
    {
        return false;
    }

    // Note: neither in current physicsengine interface nor a python binding, came from bulletphysics
    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, vector<dReal>& pJointVelocity)
    {
        _space->Synchronize(pjoint->GetParent());
        Moby::JointPtr joint = _space->GetJoint(pjoint);
        if( !joint )
        {
            return false;
        }

        Ravelin::VectorNd dq = (joint->qd);

        // what frame is the velocity w.r.t.

        pJointVelocity = vector<dReal>( dq.size() );
        for( unsigned i = 0; i < dq.size(); i++ )
        {
            pJointVelocity[i] = dq[i];
        }

        return true;
    }

    // Note: this implementation is only additive
    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const vector<dReal>& pTorques)
    {
        _space->Synchronize(pjoint->GetParent());

        Moby::JointPtr joint = _space->GetJoint(pjoint);
        _space->AddControl(joint, _space->GetRavelinVectorN(pTorques));

        return true;
    
    }

    // Note: this implementation is only additive
    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        Moby::RigidBodyPtr body = _space->GetLinkBody(plink);
        _space->AddImpulse(body, _space->GetRavelinSForce(Vector(0,0,0), torque, body->get_inertial_pose()));

        return true;
    }

    virtual bool GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque)
    {
        return false;
    }

    virtual bool GetJointForceTorque(KinBody::JointConstPtr pjoint, Vector& force, Vector& torque)
    {
        return false;
    }

    virtual void SetGravity(const Vector& gravity)
    {     
        // if gravity has not been initialized create the reference
        if(!_space->_gravity) 
        {
            _space->_gravity.reset( new Moby::GravityForce());
        }

        // update the Moby gravity force object
        _space->_gravity->gravity = Ravelin::Vector3d(gravity.x, gravity.y, gravity.z);
       
        // update the local OpenRave gravity variable  
        _gravity = gravity;
    }

    virtual const Vector& GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
        _sim->step(fTimeElapsed);

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            MobySpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);

            FOREACH(itlink, pinfo->vlinks) {
                Transform t = MobySpace::GetTransform(*(*itlink)->get_pose().get());
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());

            }
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
    }

    dReal GetTime()
    {
        return _sim->current_time;
    }

    bool SendCommand(ostream& os, istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "setposition" ) {
            string jointname;
            unsigned axis;
            dReal value;

            for(unsigned i = 0; i < 3; i++)
            {
                if(i==0) {
                    is >> jointname;
                } else if(i==1) {
                    is >> axis;
                } else { // i==2
                    is >> value;
                }

                if( !is ) {
                    RAVELOG_WARN("setposition bad command\n");
                    return false;
                }
            }

            Moby::JointPtr joint = _space->GetJoint(jointname);
            if( !!joint )
            {
                _space->SetPosition(joint, axis, value);
                return true;
            }
            RAVELOG_WARN("setposition invalid joint\n");
            return false;
        }

        throw openrave_exception(str(boost::format(("command %s supported"))%cmd),OpenRAVE::ORE_CommandNotSupported);
        return false;
    }

    dReal _StepSize;
    Vector _gravity;
    boost::shared_ptr<MobySpace> _space;

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
    boost::shared_ptr<Moby::Simulator> _sim; 
};

