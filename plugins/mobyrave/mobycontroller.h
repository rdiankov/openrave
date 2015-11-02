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

class MobyPIDController : public ControllerBase
{
public:
    MobyPIDController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false), _bEnableLogging(false)
    {
        _penv = penv;

        __description = ":Interface Author: James Taylor and Rosen Diankov\n";
//\nMoby PID controller used for planning and non-physics simulations. Forces exact robot positions.\n\n\
If \ref ControllerBase::SetPath is called and the trajectory finishes, then the controller will continue to set the trajectory's final joint values and transformation until one of three things happens:\n\n\
1. ControllerBase::SetPath is called.\n\n\
2. ControllerBase::SetDesired is called.\n\n\
3. ControllerBase::Reset is called resetting everything\n\n\
If SetDesired is called, only joint values will be set at every timestep leaving the transformation alone.\n";
        RegisterCommand("Pause",boost::bind(&MobyPIDController::_Pause,this,_1,_2),
                        "pauses the controller from reacting to commands ");
        RegisterCommand("SetCheckCollisions",boost::bind(&MobyPIDController::_SetCheckCollisions,this,_1,_2),
                        "If set, will check if the robot gets into a collision during movement");
        RegisterCommand("SetThrowExceptions",boost::bind(&MobyPIDController::_SetThrowExceptions,this,_1,_2),
                        "If set, will throw exceptions instead of print warnings. Format is:\n\n  [0/1]");
        RegisterCommand("SetEnableLogging",boost::bind(&MobyPIDController::_SetEnableLogging,this,_1,_2),
                        "If set, will write trajectories to disk");
        _fCommandTime = 0;
        _fSpeed = 1;
        _nControlTransformation = 0;

        _stateLog.open( "state", std::ofstream::out | std::ofstream::trunc );
        _tuningLog.open( "gainerror", std::ofstream::out | std::ofstream::trunc );
    }
    virtual ~MobyPIDController() {

        _stateLog.close();
        _tuningLog.close();
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        if( flog.is_open() ) {
            flog.close();
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
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
            _dofcircular.resize(0);
            FOREACH(it,_dofindices) {
                KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                _dofcircular.push_back(pjoint->IsCircular(*it-pjoint->GetDOFIndex()));
            }
            _cblimits = _probot->RegisterChangeCallback(KinBody::Prop_JointLimits|KinBody::Prop_JointAccelerationVelocityTorqueLimits,boost::bind(&MobyPIDController::_SetJointLimits,boost::bind(&utils::sptr_from<MobyPIDController>, weak_controller())));
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
            _aggregateError.resize(_probot->GetDOF()); memset(&_aggregateError[0], 0, _aggregateError.size()*sizeof(dReal));
            _prevVelocities.resize(_probot->GetDOF()); memset(&_prevVelocities[0], 0, _prevVelocities.size()*sizeof(dReal));
            
        }

        // get a reference to the physics engine
        _mobyPhysics = boost::dynamic_pointer_cast<MobyPhysicsEngine>(_penv->GetPhysicsEngine());
        _firstStep = true;

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
            RAVELOG_DEBUG("MobyPIDController cannot start trajectories when paused\n");
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
                            _vgrablinks.push_back(make_pair(dof+idof,boost::lexical_cast<int>(tokens.at(2+idof))));
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
                                listgrab.push_back(make_pair(pbody,_probot->GetLinks().at(itgrabinfo->second)));
                            }
                        }
                        else {
                            listgrab.push_back(make_pair(pbody,_probot->GetLinks().at(itgrabinfo->second)));
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
/*
            std::cout << "sample: ";
            for(unsigned i = 0; i < sample.size(); i++) 
            {
                std::cout << sample[i] << ", ";
            } 
            std::cout << endl;
*/
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
            Transform t;
            if( _bTrajHasTransform && _nControlTransformation ) {
                _samplespec.ExtractTransform(t,sampledata.begin(),_probot);
                if( vdofvalues.size() > 0 ) {
                    _SetControls(vdofvalues, desiredvelocity,t, _fCommandTime > 0 ? fTimeElapsed : 0);
                }
                else {
                    _probot->SetTransform(t);
                }
            }
            else if( vdofvalues.size() > 0 ) {
                _SetControls(vdofvalues, desiredvelocity, _fCommandTime > 0 ? fTimeElapsed : 0);
            }

            // always release after setting dof values
            FOREACH(itbody,listrelease) {
                _probot->Release(*itbody);
            }
            FOREACH(itindex,listgrabindices) {
                const GrabBody& grabinfo = _vgrabbodylinks.at(*itindex);
                KinBody::LinkPtr plink = _probot->GetLinks().at(grabinfo.robotlinkindex);
                if( !!grabinfo.trelativepose ) {
                    grabinfo.pbody->SetTransform(plink->GetTransform() * *grabinfo.trelativepose);
                }
                _probot->Grab(grabinfo.pbody, plink);
            }
            FOREACH(it,listgrab) {
                _probot->Grab(it->first,it->second);
            }
            // set _bIsDone after all computation is done!
            _bIsDone = bIsDone;
            if( bIsDone ) {
                // trajectory is done, so reset it so that the controller doesn't continously set the dof values (which can get annoying)
                _ptraj.reset();
            }
        }
/*
        else 
        {

            // compute controls to maintain the robot in a steady state
            // Note: this won't quite work as intended because the desired positions 
            // are based on current position and not on the position of the arm when this
            // branch was first entered.  Should use _vecdesired instead, but need to clarify
            // when SetDesired might be called for a steady state robot

            vector<KinBody::JointPtr> vbodyjoints;
            vbodyjoints.reserve(_probot->GetJoints().size()+_probot->GetPassiveJoints().size());
            vbodyjoints.insert(vbodyjoints.end(),_probot->GetJoints().begin(),_probot->GetJoints().end());
            vbodyjoints.insert(vbodyjoints.end(),_probot->GetPassiveJoints().begin(),_probot->GetPassiveJoints().end());

            // make a set of current positions for each joint
            int dof = _probot->GetDOF();
            vector<dReal> vdofvalues(dof);
            FOREACH(itjoint, vbodyjoints) 
            {
                int axis = 0;
                vdofvalues[(*itjoint)->GetDOFIndex()] = (*itjoint)->GetValue(axis);
            }

            // make a zero joint velocity for each joint
            vector<dReal> desiredvelocity(dof);
            for(int i = 0; i < dof; i++) 
            {
                desiredvelocity[i] = 0;
            } 

            // compute the controls
            _SetControls(vdofvalues, desiredvelocity, _fCommandTime > 0 ? fTimeElapsed : 0);
        }
*/

        if( _vecdesired.size() > 0 ) {
            if( _nControlTransformation ) {
                _SetDOFValues(_vecdesired,_tdesired, 0);
            }
            else {
                _SetDOFValues(_vecdesired, 0);
            }
            _bIsDone = true;
            // don't need to set it anymore
            _vecdesired.resize(0);
        }

        _LogJointAccelerations(fTimeElapsed);
        _firstStep = false;
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
        // for debugging and testing assume a single manipulator
        RobotBase::ManipulatorPtr manip = _probot->GetManipulators()[0];

        //vector<dReal> torques(_probot->GetDOF());
        vector<dReal> position, velocity;
        _probot->GetDOFValues(position);
        _probot->GetDOFVelocities(velocity);

        _stateLog << _mobyPhysics->GetTime();
        _tuningLog << _mobyPhysics->GetTime();

        FOREACH(it,_dofindices) {
            //vector<dReal> gains;
            double gearRatio = 1;     // assume a 1-to-1 gear ratio
            double torqueNominal = 0;
            double torqueStall = 0;

            double torqueAtMotorOut;
            double torqueAtGearboxOut;

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

                //torqueNominal = torqueStall;
            }

/*
            // get the gains
            _mobyPhysics->GetGains(_probot, (*it), gains);

            if(gains.size() < 3) 
            {
                // the number of gains is insufficient
                RAVELOG_INFO(str(boost::format("expecting 3 gain values but found %d instead.  Cannot compute PID control.\n") % gains.size() ));
                continue;
            }
*/
            dReal kP = 0, kD = 0, kI = 0, tPI = 0, kVP = 0, tVI = 0;

            if( !_mobyPhysics->GetGain(_probot, (*it), "kp", kP ) ) 
            {
                kP = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "tpi", tPI ) ) 
            {
                tPI = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "kvp", kVP ) ) 
            {
                kVP = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "tvi", tVI ) ) 
            {
                tVI = 0;
            }

            dReal ddx = 0;

            if(!_firstStep) {
                ddx = _accelerations.at(*it);
            }

            // P delta may wrap the unit circle interval, so correct.
            dReal errP = desiredposition.at(*it) - position.at(*it);
            errP = errP > PI ? -(2 * PI - errP) : errP;
            errP = errP < -PI ? -(-2 * PI + errP) : errP;

            dReal errD = desiredvelocity.at(*it) - velocity.at(*it);
            dReal errI = _aggregateError.at(*it);

/*
            dReal kP = gains[0];
            dReal kD = gains[1];
            dReal kI = gains[2];
*/
            dReal P = kP * errP;
            dReal D = kD * errD;
            dReal I = kI * errI;


            //RAVELOG_INFO(str(boost::format("vt[%f], kP[%f], tPI[%f], kVP[%f], tVI{%f}.\n") % _mobyPhysics->GetTime() % kP % tPI % kVP % tVI ));
            //RAVELOG_INFO(str(boost::format("vt[%f], errP[%f], errD[%f], errI[%f].\n") % _mobyPhysics->GetTime() % errP % errD % errI ));

            torqueAtMotorOut = P + I + D;
///*
            // if the motor output torque is greater than the nominal torque is a ceiling (also nominal torque has to be set to trigger)
            if( torqueNominal > 0 && fabs(torqueAtMotorOut) > torqueNominal ) 
            {
                double sign = torqueAtMotorOut / fabs(torqueAtMotorOut);
                torqueAtMotorOut = sign * torqueNominal;
            }
//*/  
            torqueAtGearboxOut = torqueAtMotorOut * gearRatio;

            //torques.at(*it) = gearRatio * (P + I + D);
            //torques.at(*it) = P + I + D;

            //RAVELOG_INFO(str(boost::format("computed torque[%f].\n") % torques.at(*it) ));
            //RAVELOG_INFO(str(boost::format("torque motor[%f], gearbox[%f].\n") % torqueAtMotorOut % torqueAtGearboxOut ));

            _stateLog << " " << desiredposition.at(*it) << " " << position.at(*it) << " " << desiredvelocity.at(*it) << " " << velocity.at(*it) << " " << torqueAtMotorOut << " " << torqueAtGearboxOut;
            //_stateLog << " " << desiredposition.at(*it) << " " << position.at(*it) << " " << desiredvelocity.at(*it) << " " << velocity.at(*it) << " " << torqueAtMotorOut << " " << ddx;
            _tuningLog << " " << errP << " " << errD << " " << errI;

            vector<dReal> u(1);
            //u[0] = torques.at(*it);
            u[0] = torqueAtGearboxOut;

            _aggregateError.at(*it) += errP;
            _mobyPhysics->AddJointTorque( pjoint, u );
        }
  
        _stateLog << std::endl;
        _tuningLog << std::endl;
/*
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
*/
    }
    virtual void _SetControls(const std::vector<dReal>& desiredposition, const std::vector<dReal>& desiredvelocity, const Transform &t, dReal timeelapsed)
    {
        vector<dReal> torques(_probot->GetDOF());
        vector<dReal> position, velocity;
        _probot->GetDOFValues(position);
        _probot->GetDOFVelocities(velocity);

        FOREACH(it,_dofindices) {
            KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
            ElectricMotorActuatorInfoPtr motor_info = pjoint->GetInfo()._infoElectricMotor;

/*
            vector<dReal> gains;
            _mobyPhysics->GetGains(_probot, (*it), gains);

            if(gains.size() < 3) 
            {
                // the number of gains is insufficient
                RAVELOG_INFO(str(boost::format("expecting 3 gain values but found %d instead.  Cannot compute PID control.\n") % gains.size() ));
                continue;
            }
*/
            dReal kP = 0, kD = 0, kI = 0, tPI = 0, kVP = 0, tVI = 0;

            if( !_mobyPhysics->GetGain(_probot, (*it), "kp", kP ) ) 
            {
                kP = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "tpi", tPI ) ) 
            {
                tPI = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "kvp", kVP ) ) 
            {
                kVP = 0;
            }
            if( !_mobyPhysics->GetGain(_probot, (*it), "tvi", tVI ) ) 
            {
                tVI = 0;
            }


            dReal errP = desiredposition.at(*it) - position.at(*it);
            dReal errD = desiredvelocity.at(*it) - velocity.at(*it);
            dReal errI = _aggregateError.at(*it);

/*
            // gains need to come from xml and be configured per joint 
            dReal kP = gains[0];
            dReal kD = gains[1];
            dReal kI = gains[2];
*/
            dReal P = kP * errP;
            dReal D = kD * errD;
            dReal I = kI * errI;

            RAVELOG_INFO(str(boost::format("vt[%f], errP[%f], errD[%f], errI[%f].\n") % _mobyPhysics->GetTime() % errP % errD % errI ));

            torques.at(*it) = P + I + D; 

            RAVELOG_INFO(str(boost::format("computed torque[%f].\n") % torques.at(*it) ));

            vector<dReal> u(1);
            u[0] = torques.at(*it);

            _aggregateError.at(*it) += errP;
            _mobyPhysics->AddJointTorque( pjoint, u );
        }
/*
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
*/
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

    inline boost::shared_ptr<MobyPIDController> shared_controller() {
        return boost::dynamic_pointer_cast<MobyPIDController>(shared_from_this());
    }
    inline boost::shared_ptr<MobyPIDController const> shared_controller_const() const {
        return boost::dynamic_pointer_cast<MobyPIDController const>(shared_from_this());
    }
    inline boost::weak_ptr<MobyPIDController> weak_controller() {
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

    void _LogJointAccelerations(dReal dt)
    {
        bool echo = false;

        vector<dReal> velocities;
        // allocate a vector for accelerations
        _accelerations.resize(_probot->GetDOF()); memset(&_accelerations[0], 0, _accelerations.size()*sizeof(dReal));

        // read velocities(i) for all joints
        _probot->GetDOFVelocities(velocities);

        // if there has been a preceding step, read velocity(i-1) and compute accelerations
        if(!_firstStep)
        {        
            if(echo) 
            {
                std::cout << "accelerations{ ";
            }

            // compute accelerations and output if desired
            FOREACH(it,_dofindices)
            {
                _accelerations[*it] = (velocities[*it] - _prevVelocities[*it]) / dt;

                if(echo) 
                {
                    if(*it) 
                        std::cout << ", ";
                    std::cout << *it << ":" << _accelerations[*it];
                }
            }

            if(echo) 
            {
                std::cout << "}" << std::endl;
            }
        }

        // store velocities(i) for next step usage as velocities(i-1)
        FOREACH(it,_dofindices)
        {
            _prevVelocities[*it] = velocities[*it];
        }
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

    vector<dReal> _aggregateError;

    EnvironmentBasePtr _penv;

    bool _firstStep;
    boost::shared_ptr<MobyPhysicsEngine> _mobyPhysics;
    std::vector<dReal> _prevVelocities;
    vector<dReal> _accelerations;

    std::ofstream _tuningLog;
    std::ofstream _stateLog;
};

ControllerBasePtr CreateMobyPIDController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new MobyPIDController(penv,sinput));
}
