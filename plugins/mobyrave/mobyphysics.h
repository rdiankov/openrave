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

class MobyPhysicsEngine : public PhysicsEngineBase
{

    inline boost::shared_ptr<MobyPhysicsEngine> shared_physics() {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine>(shared_from_this());
    }

    inline boost::shared_ptr<MobyPhysicsEngine const> shared_physics_const() const {
        return boost::dynamic_pointer_cast<MobyPhysicsEngine const>(shared_from_this());
    }

    class PhysicsPropertiesXMLReader : public BaseXMLReader
    {
    public:
        PhysicsPropertiesXMLReader(boost::shared_ptr<MobyPhysicsEngine> physics, const AttributesList& atts) : _physics(physics) {
        }

        virtual ProcessElement startElement(const string& name, const AttributesList& atts) {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }

            if( find(GetTags().begin(),GetTags().end(),name) == GetTags().end() ) {
                return PE_Pass;
            }
            _ss.str("");
            return PE_Support;
        }

        virtual bool endElement(const string& name)
        {
            if( name == "mobyproperties" ) 
            {
                return true;
            }
	    else if( name == "gravity" ) 
            {
                Vector g;

                _ss >> g[0] >> g[1] >> g[2];
        
                _physics->SetGravity(g);
            }
            else if( name == "kp" || name == "tpi" || name == "kvp" || name == "tvi" ) 
            {
                string robotid;
                dReal k[6];

                _ss >> robotid;
                _ss >> k[0] >> k[1] >> k[2] >> k[3] >> k[4] >> k[5];

                // query the gain buffer to see if a map exists for the robotid
                map<string, map<string,vector<dReal> > >::iterator rit;
                rit = _physics->_mapGainBuffer.find(robotid);
                if( rit == _physics->_mapGainBuffer.end() ) 
                {
                    // the gain map for robot does not exist yet, so allocate one 
                    _physics->_mapGainBuffer.insert(pair<string, map<string, vector<dReal> > >( robotid, map<string, vector<dReal> >() ));
                    rit = _physics->_mapGainBuffer.find(robotid);
                }
                
                // add the gains into the robot map
                rit->second.insert(pair<string, vector<dReal> >(name, vector<dReal>(k, k+6)));
            }
            else 
            {
                RAVELOG_ERROR("unknown field %s\n", name.c_str());
            }

            if( !_ss ) {
                RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
            }

            return false;
        }

        virtual void characters(const string& ch)
        {
            if( !!_pcurreader ) {
                _pcurreader->characters(ch);
            }
            else {
                _ss.clear();
                _ss << ch;
            }
        }

        static const boost::array<string, 5>& GetTags() {
        static const boost::array<string, 5> tags = {{"gravity","kp","tpi","kvp","tvi"}};
            return tags;
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<MobyPhysicsEngine> _physics;
        stringstream _ss;
    };

public:

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
    	return BaseXMLReaderPtr(new PhysicsPropertiesXMLReader(boost::dynamic_pointer_cast<MobyPhysicsEngine>(ptr),atts));
    }

    MobyPhysicsEngine(EnvironmentBasePtr penv, istream& sinput) : PhysicsEngineBase(penv), _StepSize(0.001), _space(new MobySpace(penv, GetPhysicsInfo, true)) 
    {
	stringstream ss;
	__description = ":Interface Authors: James Taylor and Rosen Diankov\n\nInterface to `Moby Physics Engine <https://github.com/PositronicsLab/Moby/>`_\n";

/*
        dReal gains[3] = {1.0,1.0,1.0};  // default gain values
        _mapGainBuffer.insert(pair<string, vector<dReal> >("default", vector<dReal>(gains, gains+3) ));
*/

/*
        vector<dReal> gainKP;
        vector<dReal> gainTPI;
        vector<dReal> gainKVP;
        vector<dReal> gainTVI;
       _gainKP.resize(_probot->GetDOF()); memset(&_gainKP[0], 0, _gainKP.size()*sizeof(dReal));
       _gainTPI.resize(_probot->GetDOF()); memset(&_gainTPI[0], 0, _gainTPI.size()*sizeof(dReal));
       _gainKVP.resize(_probot->GetDOF()); memset(&_gainKVP[0], 0, _gainKVP.size()*sizeof(dReal));
       _gainTVI.resize(_probot->GetDOF()); memset(&_gainTVI[0], 0, _gainTVI.size()*sizeof(dReal));
*/
        FOREACHC(it, PhysicsPropertiesXMLReader::GetTags()) {
            ss << "**" << *it << "**, ";
        }
        ss << "\n\n";
        RAVELOG_INFO( "processed xml\n" );
    }

    virtual ~MobyPhysicsEngine() 
    {

    }

    virtual bool InitEnvironment()
    {
        RAVELOG_INFO( "init Moby physics environment\n" );
        _space->SetSynchronizationCallback(boost::bind(&MobyPhysicsEngine::_SyncCallback, shared_physics(),_1));

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
        map<string, map<string, vector<dReal> > >::iterator bit;   // body iterator
        bit = _mapGainBuffer.find(pbody->GetName());
        if(bit != _mapGainBuffer.end() )
        {
            _space->MapGains(pbody, bit->second );
        }

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

        ElectricMotorActuatorInfoPtr motor_info = pjoint->GetInfo()._infoElectricMotor;
        if(!motor_info) 
        {
            //RAVELOG_INFO(str(boost::format("motor info was nothing\n")));
        }
        else
        {
            //RAVELOG_INFO(str(boost::format("motor parameters: gear_ratio[%f], assigned_power_rating[%f], max_speed[%f], no_load_speed[%f], stall_torque[%f], nominal_torque[%f], rotor_inertia[%f], torque_constant[%f], nominal_voltage[%f], speed_constant[%f], starting_current[%f], terminal_resistance[%f]\n") % motor_info->gear_ratio % motor_info->assigned_power_rating % motor_info->max_speed % motor_info->no_load_speed % motor_info->stall_torque % motor_info->nominal_torque % motor_info->rotor_inertia % motor_info->torque_constant % motor_info->nominal_voltage % motor_info->speed_constant % motor_info->starting_current % motor_info->terminal_resistance )); 
        }

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

    virtual Vector GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
        //RAVELOG_INFO( "attempting to step\n" );
        _sim->step(fTimeElapsed);

        // +dbg
        vector<Moby::DynamicBodyPtr> dbs = _sim->get_dynamic_bodies();
        //RAVELOG_INFO(str(boost::format("dbs.size[%u]\n") % dbs.size()));
        for(vector<Moby::DynamicBodyPtr>::iterator it=dbs.begin(); it!=dbs.end();it++) 
        {
            // attempt to cast
            Moby::RigidBodyPtr rb = boost::dynamic_pointer_cast<Moby::RigidBody>(*it);
            if(rb) {
                boost::shared_ptr<const Ravelin::Pose3d> pose = rb->get_mixed_pose();
                //RAVELOG_INFO(str(boost::format("x[%f,%f,%f]\n") % pose->x.x() % pose->x.y() % pose->x.z())); 
            }
        } 
        // -dbg

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            MobySpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            //RAVELOG_INFO(str(boost::format("bodies.size[%u], links.size[%u]\n") % vbodies.size() % pinfo->vlinks.size())); 
            FOREACH(itlink, pinfo->vlinks) {
                Transform t = MobySpace::GetTransform(*(*itlink)->get_pose().get());
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());

                 // +dbg
                 //double vt = _sim->current_time;
                 boost::shared_ptr<const Ravelin::Pose3d> pose = (*itlink)->get_pose();
                 //RAVELOG_INFO(str(boost::format("vt[%f], x[%f,%f,%f]\n") % vt % pose->x.x() % pose->x.y() % pose->x.z())); 
                 // -dbg
            }
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
        //RAVELOG_INFO( "completed step\n" );
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

/*
    bool GetGains(RobotBasePtr probot, int dofIndex, vector<dReal>& gains) {
        map<KinBodyPtr, map<int, vector<dReal> > >::iterator bit;
        bit = _space->_mapGains.find(probot);
        if(bit == _space->_mapGains.end() )
        {
            //RAVELOG_INFO(str(boost::format("Could not locate gains for robot %s.\n") % probot->GetName() ));
            return false;
        }
        
        map<int, vector<dReal> >::iterator dit;    // dof iterator
        dit = bit->second.find(dofIndex);
        if(dit == bit->second.end() )
        {
            //RAVELOG_INFO(str(boost::format("Could not locate gains for dofIndex %d.\n") % dofIndex ));
            return false;
        }
     
        gains = dit->second;
        //RAVELOG_INFO(str(boost::format("Found gains [%f,%f,%f] for dofIndex %d.\n") % gains[0] % gains[1] % gains[2] % dofIndex ));
 
        return true;
    }
*/
    bool GetGain(RobotBasePtr probot, int dofIndex, string gainid, dReal& gainval) {
        map<KinBodyPtr, map<string, vector<dReal> > >::iterator bit;
        bit = _space->_mapGains.find(probot);
        if(bit == _space->_mapGains.end() )
        {
            //RAVELOG_INFO(str(boost::format("Could not locate gains for robot %s.\n") % probot->GetName() ));
            return false;
        }
        
        map<string, vector<dReal> >::iterator git;
        git = bit->second.find( gainid );
        if( git == bit->second.end() )
        {
            return false;
        }

        if( dofIndex < 0 || ((unsigned)dofIndex) >= git->second.size() )
        {
            return false;
        }

        gainval = git->second[dofIndex];
 
        return true;
    }

    dReal _StepSize;
    Vector _gravity;
    boost::shared_ptr<MobySpace> _space;

    //map<string, vector<dReal> > _mapGainBuffer;
    map<string, map<string,vector<dReal> > > _mapGainBuffer;

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

