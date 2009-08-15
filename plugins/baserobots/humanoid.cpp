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

Humanoid::Humanoid(EnvironmentBase* penv) : RobotBase(penv)
{
    _state = ST_NONE;
    _pController = GetEnv()->CreateController(L"IdealController");
    if( _pController == NULL )
        _pController = GetEnv()->CreateController(L"ControllerPD");
    _bUseController = true;
    _vfront = Vector(1,0,0);
    _vup = Vector(0,0,1);

    memset(_plinkFeet, 0, sizeof(_plinkFeet));
    memset(_plinkHands, 0, sizeof(_plinkHands));
    _plinkHead = NULL;
}

Humanoid::~Humanoid()
{
    Destroy();
}

void Humanoid::Destroy()
{
    delete _pController; _pController = NULL;

    RobotBase::Destroy();
}

bool Humanoid::Init(char* strData, const char**atts)
{
    if( RobotBase::Init(strData, atts) ) {
        return LocalInit();
    }

    return false;
}

bool Humanoid::LocalInit()
{
    if( _vecManipulators.size() > 0 ) _plinkHead = _vecManipulators[0].pEndEffector;    
    if( _vecManipulators.size() > 1 ) _plinkFeet[0] = _vecManipulators[1].pEndEffector;
    if( _vecManipulators.size() > 2 ) _plinkFeet[1] = _vecManipulators[2].pEndEffector;
    if( _vecManipulators.size() > 3 ) _plinkHands[0] = _vecManipulators[3].pEndEffector;
    if( _vecManipulators.size() > 4 ) _plinkHands[1] = _vecManipulators[4].pEndEffector;
    return true;
}

void Humanoid::SetMotion(const Trajectory* ptraj)
{
    assert( ptraj != NULL );
    
    if( ptraj->GetPoints().size() == 0 )
        return;

    assert( ptraj->GetDOF() == GetDOF() );
    if( !_curTrajectory || _curTrajectory->GetDOF() != ptraj->GetDOF() )
        _curTrajectory.reset(GetEnv()->CreateTrajectory(ptraj->GetDOF()));
    *_curTrajectory = *ptraj;
    _state = ST_PATH_FOLLOW;

    if( _pController != NULL )
        _pController->SetPath(_curTrajectory.get());
}

void Humanoid::SetActiveMotion(const Trajectory* ptraj)
{
    assert( ptraj != NULL );
    
    if( ptraj->GetPoints().size() == 0 )
        return;

    assert( ptraj->GetDOF() == GetActiveDOF() );
    if( !_curTrajectory || _curTrajectory->GetDOF() != ptraj->GetDOF() )
        _curTrajectory.reset(GetEnv()->CreateTrajectory(ptraj->GetDOF()));
    GetFullTrajectoryFromActive(_curTrajectory.get(), ptraj);
    _state = ST_PATH_FOLLOW;

    if( _pController != NULL )
        _pController->SetPath(_curTrajectory.get());
}

void Humanoid::SimulationStep(dReal fElapsedTime)
{
    if( _bUseController ) {
        assert( _pController != NULL );
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

bool Humanoid::SetController(ControllerBase* p, const char* args, bool bDestroyOldController)
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

bool Humanoid::SetController(const wchar_t* pname, const char* args, bool bDestroyOldController)
{
    ControllerBase* p = GetEnv()->CreateController(pname);
    
    if( p == NULL ) {
        RAVELOG(L"RobotBase::SetController - Unable to load controller\n");
        return false;
    }
    
    return SetController(p, args, bDestroyOldController);
}

