// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

class IdealController : public ControllerBase
{
 public:
    IdealController(EnvironmentBasePtr penv) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true) {
        __description = ":Interface Author: Rosen Diankov\nIdeal controller used for planning and non-physics simulations. Forces exact robot positions.";
        fTime = 0;
        _fSpeed = 1;
    }
    virtual ~IdealController() {}

    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _probot = robot;
        if( flog.is_open() )
            flog.close();

        if( !!_probot ) {
            string filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".traj");
            flog.open(filename.c_str());
            if( !flog )
                RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
            flog << GetXMLId() << " " << _probot->GetName() << endl << endl;
        }
        _bPause = false;
        return true;
    }

    virtual void Reset(int options)
    {
        _ptraj.reset();
        _vecdesired.resize(0);
        if( flog.is_open() )
            flog.close();
    }

    virtual bool SetDesired(const std::vector<dReal>& values)
    {
        if( (int)values.size() != _probot->GetDOF() )
            throw openrave_exception(str(boost::format("wrong dimensions %d!=%d")%values.size()%_probot->GetDOF()),ORE_InvalidArguments);
        fTime = 0;
        _ptraj.reset();
        // do not set done to true here! let it be picked up by the simulation thread.
        // this will also let it have consistent mechanics as SetPath
        // (there's a race condition we're avoiding where a user calls SetDesired and then state savers revert the robot)
        if( !_bPause ) {
            _probot->SetJointValues(values);
            _vecdesired = values;
            _bIsDone = false; // set after _vecdesired has changed
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( _bPause ) {
            RAVELOG_DEBUGA("IdealController cannot player trajectories when paused\n");
            _ptraj.reset();
            _bIsDone = true;
            return false;
        }

        _ptraj = ptraj;
        fTime = 0;
        _bIsDone = !_ptraj;
        _vecdesired.resize(0);

        if( !!_ptraj && !!flog ) {
            flog << endl << "trajectory: " << ++cmdid << endl;
            _ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        }

        return true;
    }

    virtual bool SimulationStep(dReal fTimeElapsed)
    {
        if( _bPause )
            return true;
    
        if( !!_ptraj ) {
            Trajectory::TPOINT tp;
            if( !_ptraj->SampleTrajectory(fTime, tp) )
                return true;

            if( tp.q.size() > 0 )
                _probot->SetJointValues(tp.q,tp.trans,true);
            else
                _probot->SetTransform(tp.trans);

            if( fTime > _ptraj->GetTotalDuration() ) {
                fTime = _ptraj->GetTotalDuration();
                _bIsDone = true;
            }

            fTime += _fSpeed * fTimeElapsed;
        }

        if( _vecdesired.size() > 0 ) {
            _probot->SetJointValues(_vecdesired,true);
            _bIsDone = true;
        }
    
        return _bIsDone;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        if( !is )
            throw openrave_exception("invalid argument",ORE_InvalidArguments);

        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "pause" )
            is >> _bPause;
        else
            throw openrave_exception("not commands supported",ORE_CommandNotSupported);
        return true;
    }
    virtual bool IsDone()
    {
        return _bIsDone;
    }
    virtual dReal GetTime() const
    {
        return fTime;
    }
    virtual RobotBasePtr GetRobot() const { return _probot; }

private:
    RobotBasePtr _probot;           ///< controlled body
    dReal _fSpeed;                ///< how fast the robot should go
    TrajectoryBaseConstPtr _ptraj;     ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()

    dReal fTime;

    std::vector<dReal> _vecdesired;     ///< desired values of the joints    

    ofstream flog;
    int cmdid;
    bool _bPause;
    bool _bIsDone;
};

class RedirectController : public ControllerBase
{
 public:
 RedirectController(EnvironmentBasePtr penv) : ControllerBase(penv), _bAutoSync(true) {
        __description = ":Interface Author: Rosen Diankov\nRedirects all input and output to another controller (this avoides cloning the other controller while still allowing it to be used from cloned environments)";
    }
    virtual ~RedirectController() {}
    
    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _pcontroller.reset();
        _probot = GetEnv()->GetRobot(robot->GetName());
        if( _probot != robot )
            _pcontroller = robot->GetController();
        if( _bAutoSync )
            _sync();
        return true;
    }

    // don't touch the referenced controller, since could be just destroying clones
    virtual void Reset(int options) {}

    virtual bool SetDesired(const std::vector<dReal>& values)
    {
        if( !_pcontroller->SetDesired(values) )
            return false;
        if(_bAutoSync)
            _sync();
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !_pcontroller->SetPath(ptraj) )
            return false;
        if(_bAutoSync)
            _sync();
        return true;
    }

    virtual bool SimulationStep(dReal fTimeElapsed) {
        if( !_pcontroller )
            return false;
        bool bret = _pcontroller->SimulationStep(fTimeElapsed);
        if(_bAutoSync)
            _sync();
        return bret;
    }
    virtual bool IsDone() { return _bAutoSync ? _bSyncDone&&_pcontroller->IsDone() : _pcontroller->IsDone(); }

    virtual dReal GetTime() const { return _pcontroller->GetTime(); }
    virtual void GetVelocity(std::vector<dReal>& vel) const { return _pcontroller->GetVelocity(vel); }
    virtual void GetTorque(std::vector<dReal>& torque) const { return _pcontroller->GetTorque(torque); }
    
    virtual RobotBasePtr GetRobot() const { return _probot; }
    virtual ActuatorState GetActuatorState(int index) const {return _pcontroller->GetActuatorState(index); }

    virtual bool Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        boost::shared_ptr<RedirectController const> r = boost::dynamic_pointer_cast<RedirectController const>(preference);
        if( !r )
            return false;
        if( !ControllerBase::Clone(preference,cloningoptions) )
            return false;
        _probot = GetEnv()->GetRobot(r->_probot->GetName());
        _pcontroller = r->_pcontroller; // hmm......... this requires some thought
        return true;
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        streampos pos = is.tellg();
        is >> cmd;
        if( !is )
            throw openrave_exception("invalid argument",ORE_InvalidArguments);

        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "sync" ) {
            _sync();
            return true;
        }
        else if( cmd == "autosync" ) {
            is >> _bAutoSync;
            if( !is )
                return false;
            if( _bAutoSync )
                _sync();
            return true;
        }

        is.seekg(pos);
        return _pcontroller->SendCommand(os,is);
    }
    
private:
    virtual void _sync()
    {
        if( !!_pcontroller ) {
            vector<Transform> vtrans;
            _pcontroller->GetRobot()->GetBodyTransformations(vtrans);
            _probot->SetBodyTransformations(vtrans);
            _bSyncDone = _pcontroller->IsDone();
        }
    }

    bool _bAutoSync, _bSyncDone;
    RobotBasePtr _probot;           ///< controlled body
    ControllerBasePtr _pcontroller;
};

#endif
