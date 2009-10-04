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
#include "plugindefs.h"

GenericRobot::GenericRobot(EnvironmentBase* penv) : RobotBase(penv), _pController(NULL)
{
    _state = ST_NONE;
    // crashes if things get out of date, do *not* use environment pointer in constructor
//    SetController(L"IdealController", NULL, false);
//    if( _pController == NULL ) {
//        RAVEPRINT(L"Robot: could not find IdealController\n");
//        _pController = GetEnv()->CreateController(L"ControllerPD");
//    }
}

GenericRobot::~GenericRobot()
{
    Destroy();
}

void GenericRobot::Destroy()
{
    delete _pController; _pController = NULL;

    RobotBase::Destroy();
}

void GenericRobot::SetMotion(const Trajectory* ptraj)
{
    assert( ptraj != NULL );
    
    if( ptraj->GetPoints().size() == 0 ) {
        RAVELOG_WARNA("trajectory has no points\n");
        return;
    }

    if( ptraj->GetDOF() != GetDOF() )
        RAVELOG_WARNA("trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n", ptraj->GetDOF(), GetDOF());
    assert( ptraj->GetDOF() == GetDOF() );
    if( !_curTrajectory || _curTrajectory->GetDOF() != ptraj->GetDOF() )
        _curTrajectory.reset(GetEnv()->CreateTrajectory(ptraj->GetDOF()));
    *_curTrajectory = *ptraj;
    _state = ST_PATH_FOLLOW;

    if( _pController != NULL )
        _pController->SetPath(_curTrajectory.get());
    else
        RAVELOG_WARNA("controller is not set\n");
}

void GenericRobot::SetActiveMotion(const Trajectory* ptraj)
{
    assert( ptraj != NULL );
    
    if( ptraj->GetPoints().size() == 0 )
        return;

    if( ptraj->GetDOF() != GetActiveDOF() )
            RAVEPRINT(L"trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n", ptraj->GetDOF(), GetActiveDOF());
    assert( ptraj->GetDOF() == GetActiveDOF() );
    if( !_curTrajectory || _curTrajectory->GetDOF() != ptraj->GetDOF() )
        _curTrajectory.reset(GetEnv()->CreateTrajectory(ptraj->GetDOF()));
    GetFullTrajectoryFromActive(_curTrajectory.get(), ptraj);

    _state = ST_PATH_FOLLOW;

    if( _pController != NULL )
        _pController->SetPath(_curTrajectory.get());
}

void GenericRobot::SimulationStep(dReal fElapsedTime)
{
    RobotBase::SimulationStep(fElapsedTime);
    
    if( _pController != NULL ) {
        if( _pController->SimulationStep(fElapsedTime) ) {
            _state = ST_NONE;
        }
    }

    switch(_state) {
        case ST_PATH_FOLLOW:
            break;

        default:
            break;
    }
}

bool GenericRobot::SetController(ControllerBase* p, const char* args, bool bDestroyOldController)
{
    if( bDestroyOldController ) {
        delete _pController; _pController = NULL;
    }

    if( p != NULL ) {    
        if( !p->Init(this, args) ) {
            RAVELOG(L"GenericRobot %S: Failed to init controller\n", GetName());
            return false;
        }
        
        
        switch(_state) {
        case ST_PATH_FOLLOW:
            p->SetPath(_curTrajectory.get());
            break;
            
        default:
            //vector<dReal> desired;
            //if( GetDOF() > 0 ) {
            //    GetJointValues(desired);
            //    _pController->SetDesired(&desired[0]);
            //}
            break;
        }
    }
    
    _pController = p;
    return true;
}

bool GenericRobot::SetController(const wchar_t* pname, const char* args, bool bDestroyOldController)
{
    ControllerBase* p = GetEnv()->CreateController(pname);
    
    if( p == NULL ) {
        RAVELOG(L"RobotBase::SetController - Unable to load controller\n");
        return false;
    }
    
    return SetController(p, args, bDestroyOldController);
}
