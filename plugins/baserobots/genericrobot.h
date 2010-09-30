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
 GenericRobot(EnvironmentBasePtr penv) : RobotBase(penv), _state(ST_NONE) {
        __description = ":Interface Author: Rosen Diankov\nSimplest robot possible that just passes the trajectories to the controller";
    }
    virtual ~GenericRobot() {}

    virtual bool SetController(ControllerBasePtr p, const string& args)
    {
        _pController = p;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),args) ) {
                RAVELOG_WARNA(str(boost::format("GenericRobot %s: Failed to init controller %s\n")%GetName()%p->GetXMLId()));
                _pController.reset();
                return false;
            }

            if( _state == ST_PATH_FOLLOW )
                _pController->SetPath(_trajcur);
        }
        return true;
    }

    virtual bool SetMotion(TrajectoryBaseConstPtr ptraj)
    {
        BOOST_ASSERT(ptraj->GetPoints().size() > 0 || !"trajectory has no points\n");
        BOOST_ASSERT(ptraj->GetDOF() == GetDOF() || !"trajectory of wrong dimension");
        _trajcur = ptraj;
        _state = ST_PATH_FOLLOW;
        return _pController->SetPath(_trajcur);
    }
 
    virtual bool SetActiveMotion(TrajectoryBaseConstPtr ptraj)
    {
        BOOST_ASSERT(ptraj->GetPoints().size() > 0 || !"trajectory has no points\n");
        BOOST_ASSERT(ptraj->GetDOF() == GetActiveDOF() || !"trajectory of wrong dimension");
        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),ptraj->GetDOF());
        GetFullTrajectoryFromActive(pfulltraj, ptraj);
        _trajcur = pfulltraj;

        _state = ST_PATH_FOLLOW;
        return _pController->SetPath(_trajcur);
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
