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

// controller for the SSC-32 board
class IdealController : public ControllerBase
{
 public:
    IdealController(EnvironmentBasePtr penv) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true) {
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
            string filename = GetEnv()->GetHomeDirectory() + string("/traj_") + _probot->GetName();
            flog.open(filename.c_str());
            if( !flog )
                RAVELOG_WARNA("failed to open %s\n", filename.c_str());
            flog << "IdealController " << filename << endl << endl;
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

#endif
