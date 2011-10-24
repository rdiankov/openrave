// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef RAVE_CONTROLLERS_H
#define RAVE_CONTROLLERS_H

#include <boost/bind.hpp>

class IdealController : public ControllerBase
{
public:
    IdealController(EnvironmentBasePtr penv) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true), _bCheckCollision(false), _bThrowExceptions(false)
    {
        __description = ":Interface Author: Rosen Diankov\n\nIdeal controller used for planning and non-physics simulations. Forces exact robot positions.\n\n\
If \ref ControllerBase::SetPath is called and the trajectory finishes, then the controller will continue to set the trajectory's final joint values and transformation until one of three things happens:\n\n\
1. ControllerBase::SetPath is called.\n\n\
2. ControllerBase::SetDesired is called.\n\n\
3. ControllerBase::Reset is called resetting everything\n\n\
If SetDesired is called, only joint values will be set at every timestep leaving the transformation alone.\n";
        RegisterCommand("Pause",boost::bind(&IdealController::_Pause,this,_1,_2),
                        "pauses the controller from reacting to commands ");
        RegisterCommand("SetCheckCollisions",boost::bind(&IdealController::_SetCheckCollisions,this,_1,_2),
                        "If set, will check if the robot gets into a collision during movement");
        RegisterCommand("SetThrowExceptions",boost::bind(&IdealController::_SetThrowExceptions,this,_1,_2),
                        "If set, will throw exceptions instead of print warnings");
        fTime = 0;
        _fSpeed = 1;
        _nControlTransformation = 0;
    }
    virtual ~IdealController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        if( flog.is_open() ) {
            flog.close();
        }
        if( !!_probot ) {
            string filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".traj.xml");
            flog.open(filename.c_str());
            if( !flog ) {
                RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
            }
            flog << "<" << GetXMLId() << " robot=\"" << _probot->GetName() << "\"/>" << endl;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
            _dofchecklimits.resize(0);
            FOREACH(it,_dofindices) {
                KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                _dofchecklimits.push_back(!pjoint->IsCircular(*it-pjoint->GetDOFIndex()));
            }
            _cblimits = _probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IdealController::_SetJointLimits,boost::bind(&sptr_from<IdealController>, weak_controller())));
            _SetJointLimits();

            _samplespec._vgroups.resize(2);
            _samplespec._vgroups[0].offset = 0;
            _samplespec._vgroups[0].dof = _probot->GetDOF();
            stringstream ss;
            ss << "joint_values " << _probot->GetName();
            for(int i = 0; i < _probot->GetDOF(); ++i) {
                ss << " " << i;
            }
            _samplespec._vgroups[0].name = ss.str();
            _samplespec._vgroups[1].offset = _probot->GetDOF();
            _samplespec._vgroups[1].dof = RaveGetAffineDOF(DOF_Transform);
            _samplespec._vgroups[1].name = str(boost::format("affine_transform %s %d")%_probot->GetName()%DOF_Transform);
        }
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
        fTime = 0;
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
                _SetDOFValues(_vecdesired,_tdesired);
            }
            else {
                _SetDOFValues(_vecdesired);
            }
            _bIsDone = false;     // set after _vecdesired has changed
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !!ptraj ) {
            // see if at least one point can be sampled, this make it easier to debug bad trajectories
            vector<dReal> v;
            ptraj->Sample(v,0,_samplespec);
            Transform t;
            RaveGetTransformFromAffineDOFValues(t,v.begin()+_probot->GetDOF(),DOF_Transform);
        }

        if( _bPause ) {
            RAVELOG_DEBUG("IdealController cannot player trajectories when paused\n");
            _ptraj.reset();
            _bIsDone = true;
            return false;
        }
        _ptraj = ptraj;
        fTime = 0;
        _bIsDone = !_ptraj;
        _vecdesired.resize(0);

        if( !!_ptraj ) {
            _bTrajHasJoints = ptraj->GetConfigurationSpecification().FindCompatibleGroup(_samplespec._vgroups.at(0)) != ptraj->GetConfigurationSpecification()._vgroups.end();
            _bTrajHasTransform = ptraj->GetConfigurationSpecification().FindCompatibleGroup(_samplespec._vgroups.at(1)) != ptraj->GetConfigurationSpecification()._vgroups.end();
            if( !!flog ) {
                _ptraj->serialize(flog);
            }
        }

        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        if( _bPause ) {
            return;
        }
        TrajectoryBaseConstPtr ptraj = _ptraj; // because of multi-threading setting issues
        if( !!ptraj ) {
            vector<dReal> sampledata;
            ptraj->Sample(sampledata,fTime,_samplespec);

            vector<dReal> vdofvalues;
            if( _bTrajHasJoints && _dofindices.size() > 0 ) {
                vdofvalues.resize(_probot->GetDOF());
                std::copy(sampledata.begin(),sampledata.begin()+_probot->GetDOF(),vdofvalues.begin());
            }

            Transform t;
            if( _bTrajHasTransform && _nControlTransformation ) {
                RaveGetTransformFromAffineDOFValues(t,sampledata.begin()+_probot->GetDOF(),DOF_Transform);
                if( vdofvalues.size() > 0 ) {
                    _SetDOFValues(vdofvalues,t);
                }
                else {
                    _probot->SetTransform(t);
                }
            }
            else if( vdofvalues.size() > 0 ) {
                _SetDOFValues(vdofvalues);
            }

            if( fTime > ptraj->GetDuration() ) {
                fTime = ptraj->GetDuration();
                _bIsDone = true;
            }
            else {
                fTime += _fSpeed * fTimeElapsed;
            }
        }

        if( _vecdesired.size() > 0 ) {
            if( _nControlTransformation ) {
                _SetDOFValues(_vecdesired,_tdesired);
            }
            else {
                _SetDOFValues(_vecdesired);
            }
            _bIsDone = true;
        }
    }

    virtual bool IsDone() {
        return _bIsDone;
    }
    virtual dReal GetTime() const {
        return fTime;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

private:
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

    inline boost::shared_ptr<IdealController> shared_controller() {
        return boost::static_pointer_cast<IdealController>(shared_from_this());
    }
    inline boost::shared_ptr<IdealController const> shared_controller_const() const {
        return boost::static_pointer_cast<IdealController const>(shared_from_this());
    }
    inline boost::weak_ptr<IdealController> weak_controller() {
        return shared_controller();
    }

    virtual void _SetJointLimits()
    {
        if( !!_probot ) {
            _probot->GetDOFLimits(_vlower,_vupper);
        }
    }

    virtual void _SetDOFValues(const std::vector<dReal>&values)
    {
        vector<dReal> curvalues, curvel;
        _probot->GetDOFValues(curvalues);
        _probot->GetDOFVelocities(curvel);
        Vector linearvel, angularvel;
        _probot->GetLinks().at(0)->GetVelocity(linearvel,angularvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(curvalues);
        _probot->SetDOFValues(curvalues,true);
        _probot->SetDOFVelocities(curvel,linearvel,angularvel);
        _CheckConfiguration();
    }
    virtual void _SetDOFValues(const std::vector<dReal>&values, const Transform &t)
    {
        BOOST_ASSERT(_nControlTransformation);
        vector<dReal> curvalues, curvel;
        _probot->GetDOFValues(curvalues);
        _probot->GetDOFVelocities(curvel);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
            curvel.at(*it) = 0;
        }
        _CheckLimits(curvalues);
        _probot->SetDOFValues(curvalues,t, true);
        _probot->SetDOFVelocities(curvel,Vector(),Vector());
        _CheckConfiguration();
    }

    void _CheckLimits(std::vector<dReal>&curvalues)
    {
        for(size_t i = 0; i < _vlower.size(); ++i) {
            if( _dofchecklimits[i] ) {
                if( curvalues.at(i) < _vlower[i]-5e-5f ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating lower limit %s < %s")%_probot->GetName()%i%_vlower[i]%curvalues[i]));
                }
                if( curvalues.at(i) > _vupper[i]+5e-5f ) {
                    _ReportError(str(boost::format("robot %s dof %d is violating upper limit %s > %s")%_probot->GetName()%i%_vupper[i]%curvalues[i]));
                }
            }
        }
    }

    void _CheckConfiguration()
    {
        if( _bCheckCollision ) {
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_probot),_report) ) {
                _ReportError(str(boost::format("collsion in trajectory: %s\n")%_report->__str__()));
            }
            if( _probot->CheckSelfCollision(_report) ) {
                _ReportError(str(boost::format("self collsion in trajectory: %s\n")%_report->__str__()));
            }
        }
    }

    void _ReportError(const std::string& s)
    {
        if( _bThrowExceptions ) {
            throw openrave_exception(s,ORE_Assert);
        }
        else {
            RAVELOG_WARN(s);
        }
    }

    RobotBasePtr _probot;               ///< controlled body
    dReal _fSpeed;                    ///< how fast the robot should go
    TrajectoryBaseConstPtr _ptraj;         ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()
    bool _bTrajHasJoints, _bTrajHasTransform;

    dReal fTime;

    std::vector<dReal> _vecdesired;         ///< desired values of the joints
    Transform _tdesired;

    std::vector<int> _dofindices;
    std::vector<uint8_t> _dofchecklimits;
    std::vector<dReal> _vlower, _vupper;
    int _nControlTransformation;
    ofstream flog;
    int cmdid;
    bool _bPause, _bIsDone, _bCheckCollision, _bThrowExceptions;
    CollisionReportPtr _report;
    UserDataPtr _cblimits;
    ConfigurationSpecification _samplespec;
};

class RedirectController : public ControllerBase
{
public:
    RedirectController(EnvironmentBasePtr penv) : ControllerBase(penv), _bAutoSync(true) {
        __description = ":Interface Author: Rosen Diankov\n\nRedirects all input and output to another controller (this avoides cloning the other controller while still allowing it to be used from cloned environments)";
    }
    virtual ~RedirectController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>&dofindices, int nControlTransformation)
    {
        _dofindices.clear();
        _pcontroller.reset();
        _probot = GetEnv()->GetRobot(robot->GetName());
        if( _probot != robot ) {
            _pcontroller = robot->GetController();
            if( !!_pcontroller ) {
                _dofindices = _pcontroller->GetControlDOFIndices();
            }
        }
        if( _bAutoSync ) {
            _sync();
        }
        return true;
    }

    // don't touch the referenced controller, since could be just destroying clones
    virtual void Reset(int options) {
    }

    virtual bool SetDesired(const std::vector<dReal>&values, TransformConstPtr trans)
    {
        if( !_pcontroller->SetDesired(values, trans) ) {
            return false;
        }
        if(_bAutoSync) {
            _sync();
        }
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !_pcontroller->SetPath(ptraj) ) {
            return false;
        }
        if(_bAutoSync) {
            _sync();
        }
        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed) {
        if( !!_pcontroller ) {
            _pcontroller->SimulationStep(fTimeElapsed);
            if(_bAutoSync) {
                _sync();
            }
        }
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return !_pcontroller ? 0 : _pcontroller->IsControlTransformation();
    }
    virtual bool IsDone() {
        return _bAutoSync ? _bSyncDone&&_pcontroller->IsDone() : _pcontroller->IsDone();
    }

    virtual dReal GetTime() const {
        return _pcontroller->GetTime();
    }
    virtual void GetVelocity(std::vector<dReal>&vel) const {
        return _pcontroller->GetVelocity(vel);
    }
    virtual void GetTorque(std::vector<dReal>&torque) const {
        return _pcontroller->GetTorque(torque);
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        ControllerBase::Clone(preference,cloningoptions);
        boost::shared_ptr<RedirectController const> r = boost::dynamic_pointer_cast<RedirectController const>(preference);
        _probot = GetEnv()->GetRobot(r->_probot->GetName());
        _pcontroller = r->_pcontroller;     // hmm......... this requires some thought
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        streampos pos = is.tellg();
        is >> cmd;
        if( !is ) {
            throw openrave_exception("invalid argument",ORE_InvalidArguments);
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "sync" ) {
            _sync();
            return true;
        }
        else if( cmd == "autosync" ) {
            is >> _bAutoSync;
            if( !is ) {
                return false;
            }
            if( _bAutoSync ) {
                _sync();
            }
            return true;
        }

        is.seekg(pos);
        if( !_pcontroller ) {
            return false;
        }
        return _pcontroller->SendCommand(os,is);
    }

private:
    virtual void _sync()
    {
        if( !!_pcontroller ) {
            vector<Transform> vtrans;
            _pcontroller->GetRobot()->GetLinkTransformations(vtrans);
            _probot->SetLinkTransformations(vtrans);
            _bSyncDone = _pcontroller->IsDone();
        }
    }

    std::vector<int> _dofindices;
    bool _bAutoSync, _bSyncDone;
    RobotBasePtr _probot;               ///< controlled body
    ControllerBasePtr _pcontroller;
};

#endif
