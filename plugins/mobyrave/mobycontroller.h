// -*- coding: utf-8 -*-
// Copyright (C) 2015 James Taylor and Rosen Diankov <rosen.diankov@gmail.com>
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
#include "plugindefs.h"

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

class MobyController : public ControllerBase
{
    class ControllerPropertiesXMLReader : public BaseXMLReader
    {
    public:
        ControllerPropertiesXMLReader(boost::shared_ptr<MobyController> controller, const AttributesList& atts) : _controller(controller) {
        }

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
            if( name == "mobycontroller" ) 
            {
                return true;
            }
	    else if( name == "gains" ) 
            {
                return true;
            }
            else if( name == "kp" || name == "kip" || name == "kd" || name == "kid" || name == "kp_factor" || name == "kip_factor" || name == "kd_factor" || name == "kid_factor" ) 
            {
                // reads an arbitrary number of values out of the stream
                std::vector<dReal> vals;

                while( !_ss.eof() ) {
                    dReal val;
                    _ss >> val;
                    vals.push_back(val);
                }

                _controller->_mapGains.insert(pair<string, vector<dReal> >(name, vals));
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

        static const boost::array<string, 9>& GetTags() 
        {
            static const boost::array<string, 9> tags = {{"gains","kp","kip","kd","kid","kp_factor","kip_factor","kd_factor","kid_factor"}};
                return tags;
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<MobyController> _controller;
        stringstream _ss;
    };


public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
    	return BaseXMLReaderPtr(new ControllerPropertiesXMLReader(boost::dynamic_pointer_cast<MobyController>(ptr),atts));
    }

    MobyController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false), _bEnableLogging(false), _bEnableTuningLogging(false), _bSteadyState(false)    {
        _penv = penv;

        __description = ":Interface Author: James Taylor and Rosen Diankov\n\nThe Moby controller is capable of supporting any combination of PID control depending on the gains specified in the controller xml configuration\n\n";
        RegisterCommand("Pause",boost::bind(&MobyController::_Pause,this,_1,_2),
                        "pauses the controller from reacting to commands ");
        RegisterCommand("SetCheckCollisions",boost::bind(&MobyController::_SetCheckCollisions,this,_1,_2),
                        "If set, will check if the robot gets into a collision during movement");
        RegisterCommand("SetThrowExceptions",boost::bind(&MobyController::_SetThrowExceptions,this,_1,_2),
                        "If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");
        RegisterCommand("SetEnableLogging",boost::bind(&MobyController::_SetEnableLogging,this,_1,_2),
                        "If set, will write trajectories to disk");
        RegisterCommand("SetEnableTuningLogging",boost::bind(&MobyController::_SetEnableTuningLogging,this,_1,_2),
                        "If set, will write state and control error to disk");
        _fCommandTime = 0;
        _fSpeed = 1;
        _nControlTransformation = 0;

    }

    virtual ~MobyController() {

    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        if( flog.is_open() ) {
            flog.close();
        }
        if( _stateLog.is_open() ) {
            _stateLog.close();
        }
        if( _tuningLog.is_open() ) {
            _tuningLog.close();
        }
        if( !!_probot ) {
            if( _bEnableLogging ) {
                string filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".traj.xml");
                flog.open(filename.c_str());
                if( !flog ) {
                    RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
                }
                //flog << "<" << GetXMLId() << " robot=\"" << _probot->GetName() << "\"/>" << endl;
            }
            if( _bEnableTuningLogging ) {
                string filename;
                filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".state");
                _stateLog.open( filename.c_str(), std::ofstream::out | std::ofstream::trunc );
                if( !_stateLog ) {
                    RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
                }

                filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".controlerror");
                _tuningLog.open( filename.c_str(), std::ofstream::out | std::ofstream::trunc );
                if( !_tuningLog ) {
                    RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
                }
            }
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
            _dofcircular.resize(0);
            FOREACH(it,_dofindices) {
                KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                _dofcircular.push_back(pjoint->IsCircular(*it-pjoint->GetDOFIndex()));
            }
            _cblimits = _probot->RegisterChangeCallback(KinBody::Prop_JointLimits|KinBody::Prop_JointAccelerationVelocityTorqueLimits,boost::bind(&MobyController::_SetJointLimits,boost::bind(&utils::sptr_from<MobyController>, weak_controller())));
            _SetJointLimits();

            if( _dofindices.size() > 0 ) {
                _gjointvalues.reset(new ConfigurationSpecification::Group());
                _gjointvalues->offset = 0;
                _gjointvalues->dof = _dofindices.size();
                stringstream ss_val;
                ss_val << "joint_values " << _probot->GetName();
                FOREACHC(it, _dofindices) {
                    ss_val << " " << *it;
                }
                _gjointvalues->name = ss_val.str();

                _gjointvelocities.reset(new ConfigurationSpecification::Group());
                _gjointvelocities->offset = 6;
                _gjointvelocities->dof = _dofindices.size();
                stringstream ss_vel;
                ss_vel << "joint_velocities " << _probot->GetName();
                FOREACHC(it, _dofindices) {
                    ss_vel << " " << *it;
                }
                _gjointvelocities->name = ss_vel.str();
            }
            if( nControlTransformation ) {
                _gtransform.reset(new ConfigurationSpecification::Group());
                _gtransform->offset = _probot->GetDOF();
                _gtransform->dof = RaveGetAffineDOF(DOF_Transform);
                _gtransform->name = str(boost::format("affine_transform %s %d")%_probot->GetName()%DOF_Transform);
            }

            // allocate the history buffers
            _aggregateErrorP.resize(_probot->GetDOF()); memset(&_aggregateErrorP[0], 0, _aggregateErrorP.size()*sizeof(dReal));
            _aggregateErrorD.resize(_probot->GetDOF()); memset(&_aggregateErrorD[0], 0, _aggregateErrorD.size()*sizeof(dReal));
            
        }

        // get a reference to the physics engine
        _mobyPhysics = boost::dynamic_pointer_cast<MobyPhysics>(_penv->GetPhysicsEngine());

        RAVELOG_INFO(str(boost::format("Controller Initialized\n")));
        _bPause = false;

        return true;
    }

    virtual void Reset(int options)
    {
        _ptraj.reset();
        _vecdesired.resize(0);
        if( flog.is_open() ) {
            flog.close();
        }
        if( _stateLog.is_open() ) {
            _stateLog.close();
        }
        if( _tuningLog.is_open() ) {
            _tuningLog.close();
        }
        _bIsDone = true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return _nControlTransformation;
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        if( values.size() != _dofindices.size() ) {
            throw openrave_exception(str(boost::format("wrong desired dimensions %d!=%d")%values.size()%_dofindices.size()),ORE_InvalidArguments);
        }
        _fCommandTime = 0;
        _ptraj.reset();
        // do not set done to true here! let it be picked up by the simulation thread.
        // this will also let it have consistent mechanics as SetPath
        // (there's a race condition we're avoiding where a user calls SetDesired and then state savers revert the robot)
        if( !_bPause ) {
            EnvironmentMutex::scoped_lock lockenv(_probot->GetEnv()->GetMutex());
            _vecdesired = values;
            if( _nControlTransformation ) {
                if( !!trans ) {
                    _tdesired = *trans;
                }
                else {
                    _tdesired = _probot->GetTransform();
                }
                _SetDOFValues(_vecdesired,_tdesired,0);
            }
            else {
                _SetDOFValues(_vecdesired,0);
            }
            _bIsDone = false;     // set after _vecdesired has changed
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        OPENRAVE_ASSERT_FORMAT0(!ptraj || GetEnv()==ptraj->GetEnv(), "trajectory needs to come from the same environment as the controller", ORE_InvalidArguments);
        boost::mutex::scoped_lock lock(_mutex);
        if( _bPause ) {
            RAVELOG_DEBUG("MobyController cannot start trajectories when paused\n");
            _ptraj.reset();
            _bIsDone = true;
            return false;
        }
        _fCommandTime = 0;
        _bIsDone = true;
        _vecdesired.resize(0);
        _ptraj.reset();

        if( !!ptraj ) {
            _samplespec._vgroups.resize(0);
            _bTrajHasJoints = false;
            if( !!_gjointvalues ) {
                // have to reset the name since _gjointvalues can be using an old one
                stringstream ss;
                ss << "joint_values " << _probot->GetName();
                FOREACHC(it, _dofindices) {
                    ss << " " << *it;
                }
                _gjointvalues->name = ss.str();
                _bTrajHasJoints = ptraj->GetConfigurationSpecification().FindCompatibleGroup(_gjointvalues->name,false) != ptraj->GetConfigurationSpecification()._vgroups.end();
                if( _bTrajHasJoints ) {
                    _samplespec._vgroups.push_back(*_gjointvalues);
                }
            }
            if( !!_gjointvelocities ) {
                // have to reset the name since _gjointvalues can be using an old one
                stringstream ss;
                ss << "joint_velocities " << _probot->GetName();
                FOREACHC(it, _dofindices) {
                    ss << " " << *it;
                }
                _gjointvalues->name = ss.str();
                _bTrajHasJoints = ptraj->GetConfigurationSpecification().FindCompatibleGroup(_gjointvelocities->name,false) != ptraj->GetConfigurationSpecification()._vgroups.end();
                if( _bTrajHasJoints ) {
                    _samplespec._vgroups.push_back(*_gjointvelocities);
                }
            }
            _bTrajHasTransform = false;
            if( !!_gtransform ) {
                // have to reset the name since _gtransform can be using an old one
                _gtransform->name = str(boost::format("affine_transform %s %d")%_probot->GetName()%DOF_Transform);
                _bTrajHasTransform = ptraj->GetConfigurationSpecification().FindCompatibleGroup(_gtransform->name,false) != ptraj->GetConfigurationSpecification()._vgroups.end();
                if( _bTrajHasTransform ) {
                    _samplespec._vgroups.push_back(*_gtransform);
                }
            }
            _samplespec.ResetGroupOffsets();
            _vgrablinks.resize(0);
            _vgrabbodylinks.resize(0);
            int dof = _samplespec.GetDOF();
            FOREACHC(itgroup,ptraj->GetConfigurationSpecification()._vgroups) {
                if( itgroup->name.size()>=8 && itgroup->name.substr(0,8) == "grabbody") {
                    stringstream ss(itgroup->name);
                    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
                    if( itgroup->dof == 1 && tokens.size() >= 4 ) {
                        if( tokens.at(2) == _probot->GetName() ) {
                            KinBodyPtr pbody = GetEnv()->GetKinBody(tokens.at(1));
                            if( !!pbody ) {
                                _samplespec._vgroups.push_back(*itgroup);
                                _samplespec._vgroups.back().offset = dof;
                                _vgrabbodylinks.push_back(GrabBody(dof,boost::lexical_cast<int>(tokens.at(3)), pbody));
                                if( tokens.size() >= 11 ) {
                                    Transform trelativepose;
                                    trelativepose.rot[0] = boost::lexical_cast<dReal>(tokens[4]);
                                    trelativepose.rot[1] = boost::lexical_cast<dReal>(tokens[5]);
                                    trelativepose.rot[2] = boost::lexical_cast<dReal>(tokens[6]);
                                    trelativepose.rot[3] = boost::lexical_cast<dReal>(tokens[7]);
                                    trelativepose.trans[0] = boost::lexical_cast<dReal>(tokens[8]);
                                    trelativepose.trans[1] = boost::lexical_cast<dReal>(tokens[9]);
                                    trelativepose.trans[2] = boost::lexical_cast<dReal>(tokens[10]);
                                    _vgrabbodylinks.back().trelativepose.reset(new Transform(trelativepose));
                                }
                            }
                            dof += _samplespec._vgroups.back().dof;
                        }
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("robot %s invalid grabbody tokens: %s")%_probot->GetName()%ss.str()));
                    }
                }
                else if( itgroup->name.size()>=4 && itgroup->name.substr(0,4) == "grab") {
                    stringstream ss(itgroup->name);
                    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
                    if( tokens.size() >= 2 && tokens[1] == _probot->GetName() ) {
                        _samplespec._vgroups.push_back(*itgroup);
                        _samplespec._vgroups.back().offset = dof;
                        for(int idof = 0; idof < _samplespec._vgroups.back().dof; ++idof) {
                            _vgrablinks.emplace_back(dof+idof, boost::lexical_cast<int>(tokens.at(2+idof)));
                        }
                        dof += _samplespec._vgroups.back().dof;
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("robot %s invalid grab tokens: %s")%_probot->GetName()%ss.str()));
                    }
                }
            }
            BOOST_ASSERT(_samplespec.IsValid());

            // see if at least one point can be sampled, this make it easier to debug bad trajectories
            vector<dReal> v;
            ptraj->Sample(v,0,_samplespec);
            if( _bTrajHasTransform ) {
                Transform t;
                _samplespec.ExtractTransform(t,v.begin(),_probot);
            }

            if( !!flog && _bEnableLogging ) {
                ptraj->serialize(flog);
            }

            _ptraj = RaveCreateTrajectory(GetEnv(),ptraj->GetXMLId());
            _ptraj->Clone(ptraj,0);
            _bIsDone = false;
        }

        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        if( _bPause ) {
            return;
        }
  
        boost::mutex::scoped_lock lock(_mutex);
        TrajectoryBaseConstPtr ptraj = _ptraj; // because of multi-threading setting issues
        if( !!ptraj ) {
            _bSteadyState = false;

            vector<dReal> sampledata;
            ptraj->Sample(sampledata,_fCommandTime,_samplespec);

            // already sampled, so change the command times before before setting values
            // incase the below functions fail
            bool bIsDone = _bIsDone;
            if( _fCommandTime > ptraj->GetDuration() ) {
                _fCommandTime = ptraj->GetDuration();
                bIsDone = true;
            }
            else {
                _fCommandTime += _fSpeed * fTimeElapsed;
            }

            // first process all grab info
            list<KinBodyPtr> listrelease;
            list<pair<KinBodyPtr, KinBody::LinkPtr> > listgrab;
            list<int> listgrabindices;
            FOREACH(itgrabinfo,_vgrablinks) {
                int bodyid = int(std::floor(sampledata.at(itgrabinfo->first)+0.5));
                if( bodyid != 0 ) {
                    KinBodyPtr pbody = GetEnv()->GetBodyFromEnvironmentId(abs(bodyid));
                    if( !pbody ) {
                        RAVELOG_WARN(str(boost::format("failed to find body id %d")%bodyid));
                        continue;
                    }
                    if( bodyid < 0 ) {
                        if( !!_probot->IsGrabbing(pbody) ) {
                            listrelease.push_back(pbody);
                        }
                    }
                    else {
                        KinBody::LinkPtr pgrabbinglink = _probot->IsGrabbing(pbody);
                        if( !!pgrabbinglink ) {
                            if( pgrabbinglink->GetIndex() != itgrabinfo->second ) {
                                listrelease.push_back(pbody);
                                listgrab.emplace_back(pbody, _probot->GetLinks().at(itgrabinfo->second));
                            }
                        }
                        else {
                            listgrab.emplace_back(pbody, _probot->GetLinks().at(itgrabinfo->second));
                        }
                    }
                }
            }
            FOREACH(itgrabinfo,_vgrabbodylinks) {
                int dograb = int(std::floor(sampledata.at(itgrabinfo->offset)+0.5));
                if( dograb <= 0 ) {
                    if( !!_probot->IsGrabbing(itgrabinfo->pbody) ) {
                        listrelease.push_back(itgrabinfo->pbody);
                    }
                }
                else {
                    KinBody::LinkPtr pgrabbinglink = _probot->IsGrabbing(itgrabinfo->pbody);
                    if( !!pgrabbinglink ) {
                        listrelease.push_back(itgrabinfo->pbody);
                    }
                    listgrabindices.push_back(static_cast<int>(itgrabinfo-_vgrabbodylinks.begin()));
                }
            }

            // sample the trajectory for the current time
            vector<dReal> sample;
            ptraj->Sample(sample, _fCommandTime, _samplespec);

            // get the joint positions from the trajectory
            vector<ConfigurationSpecification::Group>::const_iterator pit;
            pit = _samplespec.FindCompatibleGroup("joint_values", false);
            vector<dReal> vdofvalues(pit->dof);
            for(int i = 0; i < pit->dof; i++) 
            {
                vdofvalues[i] = sample[i+pit->offset];
            } 

            // get the joint velocities from the trajectory
            vector<ConfigurationSpecification::Group>::const_iterator vit;
            vit = _samplespec.FindCompatibleGroup("joint_velocities", false);
            vector<dReal> desiredvelocity(vit->dof);
            for(int i = 0; i < vit->dof; i++) 
            {
                desiredvelocity[i] = sample[i+vit->offset];
            } 

            // compute the controls
            if( vdofvalues.size() > 0 )
            {
                _SetControls(vdofvalues, desiredvelocity, _fCommandTime > 0 ? fTimeElapsed : 0);
            }

            // always release after setting dof values
            FOREACH(itbody,listrelease)
            {
                _probot->Release(*itbody);
            }
            FOREACH(itindex,listgrabindices)
            {
                const GrabBody& grabinfo = _vgrabbodylinks.at(*itindex);
                KinBody::LinkPtr plink = _probot->GetLinks().at(grabinfo.robotlinkindex);
                if( !!grabinfo.trelativepose )
                {
                    grabinfo.pbody->SetTransform(plink->GetTransform() * *grabinfo.trelativepose);
                }
                _probot->Grab(grabinfo.pbody, plink);
            }
            FOREACH(it,listgrab)
            {
                _probot->Grab(it->first,it->second);
            }

            // set _bIsDone after all computation is done!
            _bIsDone = bIsDone;
            if( bIsDone ) {
                // trajectory is done, so reset it so the controller doesn't
                // continously set the dof values (which can get annoying)
                _ptraj.reset();
            }

            if( _vecdesired.size() > 0 )
            {
                if( _nControlTransformation )
                {
                    _SetDOFValues(_vecdesired,_tdesired, 0);
                }
                else
                {
                    _SetDOFValues(_vecdesired, 0);
                }
                _bIsDone = true;
                // don't need to set it anymore
                _vecdesired.resize(0);
            }
        }
        else 
        {
            // compute controls to maintain the robot in a steady state otherwise
            // the robot will collapse under its own weight absent a trajectory
            // A real robot generally has brakes to prevent this as well as friction 
            // in the motors and gearbox that resists or at least slows collapse
            int dof = _probot->GetDOF();

            // if the robot is transitioning to steady state, record the current 
            // joint values so that it can by used as the desired position in
            // steady state control.
            if(!_bSteadyState) {
                _bSteadyState = true;

                vector<KinBody::JointPtr> vbodyjoints;
                vbodyjoints.reserve(_probot->GetJoints().size()+_probot->GetPassiveJoints().size());
                vbodyjoints.insert(vbodyjoints.end(),_probot->GetJoints().begin(),_probot->GetJoints().end());
                vbodyjoints.insert(vbodyjoints.end(),_probot->GetPassiveJoints().begin(),_probot->GetPassiveJoints().end());

                _vecsteady.resize(dof);
                FOREACH(itjoint, vbodyjoints) 
                {
                    int axis = 0;
                    _vecsteady[(*itjoint)->GetDOFIndex()] = (*itjoint)->GetValue(axis);
                }
            }

            // make a zero joint velocity for each joint
            vector<dReal> desiredvelocity(dof);
            for(int i = 0; i < dof; i++) 
            {
                desiredvelocity[i] = 0;
            } 

            // compute the controls
            _SetControls(_vecsteady, desiredvelocity, _fCommandTime > 0 ? fTimeElapsed : 0);
        }
    }

    virtual bool IsDone() 
    {
        return _bIsDone;
    }

    virtual dReal GetTime() const 
    {
        return _fCommandTime;
    }

    virtual RobotBasePtr GetRobot() const 
    {
        return _probot;
    }

private:
    virtual void _SetControls(const std::vector<dReal>& desiredposition, const std::vector<dReal>& desiredvelocity, dReal timeelapsed)
    {
        // get the current state of the robot
        vector<dReal> position, velocity;
        _probot->GetDOFValues(position);
        _probot->GetDOFVelocities(velocity);

        dReal time = _mobyPhysics->GetTime();

        // Note: may not want to log when _bSteadyState is true (or forward any commands
        // from the simulated environment to any pipeline stage) as these commands are 
        // purely for the simulator's needs and do not reflect real robot commands

        // open new tuning log records with the current timestamp
        if( !!_stateLog && !!_tuningLog && _bEnableTuningLogging )
        {
            _stateLog << time;
            _tuningLog << time;
        }

        FOREACH(it,_dofindices)
        {
            double gearRatio = 1;      // assume a 1-to-1 gear ratio
            double torqueNominal = 0;  // zero nominal torque indicates no value specified
            double torqueStall = 0;    // zero stall torque indicates no value specified
            double torqueAtMotorOut;   // torque generated at the motor output shaft
            double torqueAtGearboxOut; // torque generated at the gearbox output shaft
            dReal kP, kIp, kD, kId;    // gain coefficients
            dReal kP_factor, kIp_factor, kD_factor, kId_factor;    // gain coefficients scaling factors

            // get the joint
            KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);

            // get the motor parameters
            ElectricMotorActuatorInfoPtr motorInfo = pjoint->GetInfo()._infoElectricMotor;

            // if the joint has motorInfo map in the local parameters
            if( !!motorInfo ) 
            {
                // if the motorInfo gear_ratio is valid, override the local gearRatio
                if(motorInfo->gear_ratio != 0) 
                {
                    gearRatio = motorInfo->gear_ratio;
                }
                torqueNominal = motorInfo->nominal_torque;
                torqueStall = motorInfo->stall_torque;
            }

            // a gain is set to zero if the gain is not found in the gain data
            // which subsequently zeros that component's error in the control signal
            _GetGain((*it), "kp", kP);
            _GetGain((*it), "kip", kIp);
            _GetGain((*it), "kd", kD);
            _GetGain((*it), "kid", kId);
            
            // a gain factor is a scaling value for the gain itself.
            // if no factor exists for the gain then assume a value of one
            if( !_GetGain((*it), "kp_factor", kP_factor) ) {
                kP_factor = 1;
            }
            if( !_GetGain((*it), "kip_factor", kIp_factor) ) {
                kIp_factor = 1;
            }
            if( !_GetGain((*it), "kd_factor", kD_factor) ) {
                kD_factor = 1;
            }
            if( !_GetGain((*it), "kid_factor", kId_factor) ) {
                kId_factor = 1;
            }

            // compute component error
            // Note: delta P may wrap the unit circle interval, 
            //       so correct P_err if it exceeds interval.
            dReal P_err = desiredposition.at(*it) - position.at(*it);
            P_err = P_err > PI ? -(2 * PI - P_err) : P_err;
            P_err = P_err < -PI ? -(-2 * PI + P_err) : P_err;
            dReal Ip_err = _aggregateErrorP.at(*it);
            dReal D_err = desiredvelocity.at(*it) - velocity.at(*it);
            dReal Id_err = _aggregateErrorD.at(*it);

            // The motor command torque is computed piecewise as there 
            // may be filtering at different stages of the computation
            dReal positional_component, derivative_component;

            // compute the motor command

            // compute the positional component
            // Note: filtering may be applied to any of these operations
            // Note: coefficients determine whether a component contributes
            dReal P = kP * kP_factor * P_err;
            dReal Ip = kIp * kIp_factor * Ip_err;
            positional_component = P + Ip;
            torqueAtMotorOut = positional_component;

            // compute the derivative component
            // Note: filtering may be applied to any of these operations
            // Note: coefficients determine whether a component contributes
            dReal D = kD * kD_factor * D_err;
            dReal Id = kId * kId_factor * Id_err;
            derivative_component = D + Id;
            torqueAtMotorOut += derivative_component;

            // warn if the motor output torque is greater than stall or nominal torque
            if( !_bSteadyState && torqueStall > 0 && fabs(torqueAtMotorOut) > torqueStall ) 
            {
                RAVELOG_WARN(str(boost::format("Controller generated a motor torque with a magnitude greater than the motor's specified stall torque: sim_time[%f], robot[%s], joint[%s], torque[%f], stall_torque[%f]\nMotors are incapable of physically generating torques greater than stall torque.\n") % time % _probot->GetName().c_str() % pjoint->GetName().c_str() % torqueAtMotorOut % torqueStall ));
            } else if( !_bSteadyState && torqueNominal > 0 && fabs(torqueAtMotorOut) > torqueNominal ) 
            {
                RAVELOG_WARN(str(boost::format("Controller generated a motor torque with a magnitude greater than the motor's specified nominal torque: sim_time[%f], robot[%s], joint[%s], torque[%f], nominal_torque[%f]\nContinuous application of torques greater than nominal torque may damage the real motor.\n") % time % _probot->GetName().c_str() % pjoint->GetName().c_str() % torqueAtMotorOut % torqueNominal ));
            }
  
            // compute the torque to be applied at the joint
            torqueAtGearboxOut = torqueAtMotorOut * gearRatio;

            // log tuning data
            if( !!_stateLog && !!_tuningLog && _bEnableTuningLogging ) {
                _stateLog << " " << desiredposition.at(*it) << " " << position.at(*it) << " " << desiredvelocity.at(*it) << " " << velocity.at(*it) << " " << torqueAtMotorOut << " " << torqueAtGearboxOut;

                // if the component is expected to contribute (as indicated by 
                // the presence of a gain coefficient), then record the error 
                // in the tuning log
                if( kP > 0 )
                {
                    _tuningLog << " " << P_err;
                }
                if( kIp > 0 )
                {
                    _tuningLog << " " << Ip_err;
                }
                if( kD > 0 )
                {
                    _tuningLog << " " << D_err;
                }
                if( kId > 0 )
                {
                    _tuningLog << " " << Id_err;
                }
            }

            // apply computed torque
            vector<dReal> u(1);
            u[0] = torqueAtGearboxOut;
            _mobyPhysics->AddJointTorque( pjoint, u );

            // update histories
            _aggregateErrorP.at(*it) += P_err;
            _aggregateErrorD.at(*it) += D_err;
        }
 
        // close the current tuning data record
        if( !!_stateLog && !!_tuningLog && _bEnableTuningLogging ) {
            _stateLog << std::endl;
            _tuningLog << std::endl;
        }
    }

    virtual bool _Pause(std::ostream& os, std::istream& is)
    {
        is >> _bPause;
        return !!is;
    }

    virtual bool _SetCheckCollisions(std::ostream& os, std::istream& is)
    {
        is >> _bCheckCollision;
        if( _bCheckCollision ) {
            _report.reset(new CollisionReport());
        }
        return !!is;
    }

    virtual bool _SetThrowExceptions(std::ostream& os, std::istream& is)
    {
        is >> _bThrowExceptions;
        return !!is;
    }

    virtual bool _SetEnableLogging(std::ostream& os, std::istream& is)
    {
        is >> _bEnableLogging;
        return !!is;
    }

    virtual bool _SetEnableTuningLogging(std::ostream& os, std::istream& is)
    {
        is >> _bEnableTuningLogging;
        return !!is;
    }

    inline boost::shared_ptr<MobyController> shared_controller()
    {
        return boost::static_pointer_cast<MobyController>(shared_from_this());
    }

    inline boost::shared_ptr<MobyController const> shared_controller_const() const
    {
        return boost::static_pointer_cast<MobyController const>(shared_from_this());
    }

    inline boost::weak_ptr<MobyController> weak_controller()
    {
        return shared_controller();
    }

    virtual void _SetJointLimits()
    {
        if( !!_probot ) {
            _probot->GetDOFLimits(_vlower[0],_vupper[0]);
            _probot->GetDOFVelocityLimits(_vupper[1]);
            _probot->GetDOFAccelerationLimits(_vupper[2]);
        }
    }

    virtual void _SetDOFValues(const std::vector<dReal>&values, dReal timeelapsed)
    {
        vector<dReal> prevvalues, curvalues, curvel;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        _probot->GetDOFVelocities(curvel);
        Vector linearvel, angularvel;
        _probot->GetLinks().at(0)->GetVelocity(linearvel,angularvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(prevvalues, curvalues, timeelapsed);
        _probot->SetDOFValues(curvalues,true);
        _probot->SetDOFVelocities(curvel,linearvel,angularvel);
        _CheckConfiguration();
    }
    virtual void _SetDOFValues(const std::vector<dReal>&values, const Transform &t, dReal timeelapsed)
    {
        BOOST_ASSERT(_nControlTransformation);
        vector<dReal> prevvalues, curvalues, curvel;
        _probot->GetDOFValues(prevvalues);
        curvalues = prevvalues;
        _probot->GetDOFVelocities(curvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(prevvalues, curvalues, timeelapsed);
        _probot->SetDOFValues(curvalues,t, true);
        _probot->SetDOFVelocities(curvel,Vector(),Vector());
        _CheckConfiguration();
    }

    void _CheckLimits(std::vector<dReal>& prevvalues, std::vector<dReal>&curvalues, dReal timeelapsed)
    {
        for(size_t i = 0; i < _vlower[0].size(); ++i) {
            if( !_dofcircular[i] ) {
                if( curvalues.at(i) < _vlower[0][i]-g_fEpsilonJointLimit ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating lower limit %e < %e, time=%f")%_probot->GetName()%i%_vlower[0][i]%curvalues[i]%_fCommandTime));
                }
                if( curvalues.at(i) > _vupper[0][i]+g_fEpsilonJointLimit ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating upper limit %e > %e, time=%f")%_probot->GetName()%i%_vupper[0][i]%curvalues[i]%_fCommandTime));
                }
            }
        }
        if( timeelapsed > 0 ) {
            vector<dReal> vdiff = curvalues;
            _probot->SubtractDOFValues(vdiff,prevvalues);
            for(size_t i = 0; i < _vupper[1].size(); ++i) {
                dReal maxallowed = timeelapsed * _vupper[1][i]+1e-6;
                if( RaveFabs(vdiff.at(i)) > maxallowed ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating max velocity displacement %.15e > %.15e, time=%f")%_probot->GetName()%i%RaveFabs(vdiff.at(i))%maxallowed%_fCommandTime));
                }
            }
        }
    }

    void _CheckConfiguration()
    {
        if( _bCheckCollision ) {
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_probot),_report) ) {
                _ReportError(str(boost::format("collsion in trajectory: %s, time=%f\n")%_report->__str__()%_fCommandTime));
            }
            if( _probot->CheckSelfCollision(_report) ) {
                _ReportError(str(boost::format("self collsion in trajectory: %s, time=%f\n")%_report->__str__()%_fCommandTime));
            }
        }
    }

    void _ReportError(const std::string& s)
    {
        if( !!_ptraj ) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                string filename = str(boost::format("%s/failedtrajectory%d.xml")%RaveGetHomeDirectory()%(RaveRandomInt()%1000));
                ofstream f(filename.c_str());
                f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
                _ptraj->serialize(f);
                RAVELOG_VERBOSE(str(boost::format("trajectory dumped to %s")%filename));
            }
        }
        if( _bThrowExceptions ) {
            throw openrave_exception(s,ORE_Assert);
        }
        else {
            RAVELOG_WARN(s);
        }
    }

    /// locates a gain value within the set of gains
    /// @param[in] dof the joint index of the desired gain
    /// @param[in] id the identifier of the desired gain set
    /// @param[out] value the gain value for the specified dof and id 
    /// @return true if a gain value was located with the specified dof
    ///         and id OR false if the gain value was not located
    bool _GetGain(int dof, string id, dReal& value) {
        // initialize the gain value
        value = 0;

        // locate the gain set by id
        map<string, vector<dReal> >::iterator git;
        git = _mapGains.find( id );
        if( git == _mapGains.end() )
        {
            // the requested gain set does not exist in the gain map
            return false;
        }

        if( dof < 0 || ((unsigned)dof) >= git->second.size() )
        {
            // the requested dof does not have a value for this gain set
            return false;
        }

        // otherwise a gain was located so update the gain value
        value = git->second[dof];
 
        // and indicate success
        return true;
    }

    RobotBasePtr _probot;               ///< controlled body
    dReal _fSpeed;                    ///< how fast the robot should go
    TrajectoryBasePtr _ptraj;         ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()
    bool _bTrajHasJoints, _bTrajHasTransform;
    std::vector< pair<int, int> > _vgrablinks; /// (data offset, link index) pairs
    struct GrabBody
    {
        GrabBody() : offset(0), robotlinkindex(0) {
        }
        GrabBody(int offset, int robotlinkindex, KinBodyPtr pbody) : offset(offset), robotlinkindex(robotlinkindex), pbody(pbody) {
        }
        int offset;
        int robotlinkindex;
        KinBodyPtr pbody;
        boost::shared_ptr<Transform> trelativepose; ///< relative pose of body with link when grabbed. if it doesn't exist, then do not pre-transform the pose
    };
    std::vector<GrabBody> _vgrabbodylinks;
    dReal _fCommandTime;

    std::vector<dReal> _vecdesired;         ///< desired values of the joints
    Transform _tdesired;

    std::vector<int> _dofindices;
    std::vector<uint8_t> _dofcircular;
    boost::array< std::vector<dReal>, 3> _vlower, _vupper; ///< position, velocity, acceleration limits
    int _nControlTransformation;
    ofstream flog;
    int cmdid;
    bool _bPause, _bIsDone, _bCheckCollision, _bThrowExceptions, _bEnableLogging;
    CollisionReportPtr _report;
    UserDataPtr _cblimits;
    ConfigurationSpecification _samplespec;
    boost::shared_ptr<ConfigurationSpecification::Group> _gjointvalues, _gjointvelocities, _gtransform;
    boost::mutex _mutex;

    vector<dReal> _aggregateErrorP;     //< PI history
    vector<dReal> _aggregateErrorD;     //< DI history

    EnvironmentBasePtr _penv;

    boost::shared_ptr<MobyPhysics> _mobyPhysics;

    bool _bEnableTuningLogging;
    std::ofstream _tuningLog;
    std::ofstream _stateLog;

    bool _bSteadyState;
    std::vector<dReal> _vecsteady;

    // templated comparator for comparing the value of two shared pointers
    // used predominantly to ensure maps keyed on shared pointers are hashed properly
    template<class T>
    class _CompareSharedPtrs {
    public:
       bool operator()(boost::shared_ptr<T> a, boost::shared_ptr<T> b) const {
          return a.get() < b.get();
       }
    };

    map<string, vector<dReal> > _mapGains;

};

ControllerBasePtr CreateMobyController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new MobyController(penv,sinput));
}
