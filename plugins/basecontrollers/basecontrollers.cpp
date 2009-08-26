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

// IdealController
IdealController::IdealController(EnvironmentBase* penv) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true)
{
    _ptraj = NULL;
    fTime = 0;
    _fSpeed = 1;
}

IdealController::~IdealController()
{
    flog.close();
}


bool IdealController::Init(RobotBase* robot, const char* args)
{
    _probot = robot;
    if( flog.is_open() )
        flog.close();

    if( _probot != NULL ) {
        string filename = GetEnv()->GetHomeDirectory() + string("/traj_") + _stdwcstombs(_probot->GetName());
        flog.open(filename.c_str());
        if( !flog )
            RAVEPRINT(L"failed to open %s\n", filename);
        flog << "IdealController " << filename << endl << endl;
    }
    _bPause = false;
    return _probot != NULL;
}

void IdealController::Reset(int options)
{
    _ptraj = NULL;
    _vecdesired.resize(0);
    if( flog.is_open() )
        flog.close();
}

bool IdealController::SetDesired(const dReal* pValues)
{
    assert( _probot != NULL );

    fTime = 0;
    _ptraj = NULL;
    _bIsDone = true;

     _probot->SetJointValues(NULL, NULL, pValues, true);

    if( !_bPause ) {
        // only update vecdesired if not paused
        _vecdesired.resize(_probot->GetDOF());
        memcpy(&_vecdesired[0], pValues, sizeof(_vecdesired[0]) * _vecdesired.size());
    }
    return true;
}

bool IdealController::SetPath(const Trajectory* ptraj)
{
    if( _bPause ) {
        RAVEPRINT(L"IdealController cannot player trajectories when paused\n");
        _ptraj = NULL;
        _bIsDone = true;
        return false;
    }

    _ptraj = ptraj;
    fTime = 0;
    _bIsDone = false;
    _vecdesired.resize(0);

    if( ptraj != NULL && !!flog ) {
        flog << endl << "trajectory: " << ++cmdid << endl;
        ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    }

    return true;
}

bool IdealController::SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime)
{
    return SetPath(ptraj);
}

bool IdealController::SimulationStep(dReal fTimeElapsed)
{
    if( _bPause )
        return true;
    
    if( _ptraj != NULL ) {
        Trajectory::TPOINT tp;
        if( !_ptraj->SampleTrajectory(fTime, tp) )
            return true;

        if( tp.q.size() > 0 ) {
            _probot->SetJointValues(NULL, &tp.trans, &tp.q[0], true);
        }
        else {
            _probot->SetTransform(tp.trans);
        }

        if( fTime > _ptraj->GetTotalDuration() ) {
            fTime = _ptraj->GetTotalDuration();
            _bIsDone = true;
        }

        fTime += _fSpeed * fTimeElapsed;
    }

    if( _vecdesired.size() > 0 ) {
        assert((int)_vecdesired.size()==_probot->GetDOF());
        _probot->SetJointValues(NULL, NULL, &_vecdesired[0], true);
    }
    
    return _bIsDone;
}

bool IdealController::SendCmd(const char* pcmd)
{
    assert(_probot != NULL );

    string cmd = pcmd;
    char* ptoken = strtok(&cmd[0], " ");
    if( stricmp(ptoken, "pause") == 0 ) {
        char* p = strtok(NULL," ");
        _bPause = (p == NULL || atoi(p) > 0);
    }
    return true;
}

bool IdealController::SupportsCmd(const char* pcmd)
{
    return stricmp(pcmd, "pause");
}
