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
#ifndef RAVE_GENERIC_ROBOT_H
#define RAVE_GENERIC_ROBOT_H

class GenericRobot : public RobotBase
{
 public:
    enum RobotState { ST_NONE=0, ST_PD_CONTROL, ST_PATH_FOLLOW };
 GenericRobot(EnvironmentBasePtr penv) : RobotBase(penv), _state(ST_NONE) { __description = "Simplest robot possible that just passes the trajectories to the controller"; }
    virtual ~GenericRobot() {}

    virtual bool SetController(ControllerBasePtr p, const string& args)
    {
        _pController = p;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),args) ) {
                RAVELOG_WARNA("GenericRobot %s: Failed to init controller\n", GetName().c_str());
                _pController.reset();
                return false;
            }

            if( _state == ST_PATH_FOLLOW )
                _pController->SetPath(_trajcur);
        }
        return true;
    }

    virtual void SetMotion(TrajectoryBaseConstPtr ptraj)
    {
        _trajcur = ptraj;
        if( _trajcur->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("trajectory has no points\n");
            return;
        }

        if( _trajcur->GetDOF() != GetDOF() )
            RAVELOG_WARNA("trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n", _trajcur->GetDOF(), GetDOF());
        BOOST_ASSERT( _trajcur->GetDOF() == GetDOF() );
        _trajcur = ptraj;
        _state = ST_PATH_FOLLOW;

        if( !!_pController )
            _pController->SetPath(_trajcur);
        else
            RAVELOG_WARNA("controller is not set\n");
    }
 
    virtual void SetActiveMotion(TrajectoryBaseConstPtr ptraj)
    {
        if( ptraj->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("trajectory has no points\n");
            return;
        }

        if( ptraj->GetDOF() != GetActiveDOF() ) {
            RAVELOG_WARNA("trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n", ptraj->GetDOF(), GetActiveDOF());
            return;
        }
        BOOST_ASSERT( ptraj->GetDOF() == GetActiveDOF() );

        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(ptraj->GetDOF());
        GetFullTrajectoryFromActive(pfulltraj, ptraj);
        _trajcur = pfulltraj;

        _state = ST_PATH_FOLLOW;

        if( !!_pController )
            _pController->SetPath(_trajcur);
        else
            RAVELOG_WARNA("controller is not set\n");
    }


    RobotState GetState() { return _state; }

    virtual ControllerBasePtr GetController() const { return _pController; }

    virtual void SimulationStep(dReal fElapsedTime)
    {
        RobotBase::SimulationStep(fElapsedTime);
        if( !!_pController ) {
            if( _pController->SimulationStep(fElapsedTime) ) {
                _state = ST_NONE;
            }
        }
    }

 protected:
    TrajectoryBaseConstPtr _trajcur;
    ControllerBasePtr _pController;
    RobotState _state;

};

#endif
