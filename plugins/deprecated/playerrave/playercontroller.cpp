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

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include <sstream>

// boosting
#include <algorithm>
#include <functional>
#include <boost/bind.hpp>

#include "playercontroller.h"

static map<int, string> s_mapPlayerStates, s_mapStallActions;

PlayerController::TRAJECTORY::TRAJECTORY(int id, PlayerController::StallAction act, const string& stallcmd, const vector<PLAYERPROXY>& vproxies)
    : _id(id), _stallaction(act), _strStallCommand(stallcmd)
{
    _vignore.resize(vproxies.size());
    for(int i = 0; i < (int)_vignore.size(); ++i)
        _vignore[i] = vproxies[i].bIgnore;
}

PlayerController::PlayerController(EnvironmentBase* penv) : ControllerBase(penv)
{
    nLastRecvTime = 0;
    nFeedbackTime = 40; // update robot at 25fps

    _bStopThread=true;
    _bDestroyThread = false;
    _pThread = NULL;
    _bPause = false;
    _bIsDone = true;
    _bSendTimestamps = false;
    _bHoldOnStall = false;
    _saNextStallAction = SA_Nothing;
    s_mapPlayerStates[AS_Idle] = "Idle";
    s_mapPlayerStates[AS_Moving] = "Moving";
    s_mapPlayerStates[AS_Stalled] = "Stalled";
    s_mapPlayerStates[AS_Braked] = "Braked";
    s_mapStallActions[SA_Nothing] = "nothing";
    s_mapStallActions[SA_StopAll] = "stopall";
    s_mapStallActions[SA_CancelCurrent] = "cancel current";
}

PlayerController::~PlayerController()
{
    Destroy();
}

void PlayerController::Destroy()
{
    if (!_bStopThread)
        StopThread();

    for(int i = 0; i < _vecproxies.size(); ++i) delete _vecproxies[i].pproxy;
    _vecproxies.clear();
    FOREACH(itclient, _vclients) delete *itclient;
    _vclients.clear();

    flog.close();
}

bool PlayerController::Init(RobotBase* robot, const char* args)
{
    if( _vclients.size() > 0 ) {
        Destroy();
    }

    _probot = robot;
    if( _probot == NULL || args == NULL )
        return false;

    string host;
    int _port;
    vector<dReal> vmaxvel;
    PlayerClient* pclient = NULL;
    _listQueuedTrajectories.clear();

    // default values
    host = "";
    _port = 0;
    _bIsDone = true;
    _bPause = false;
    _bHoldOnStall = false;
    _asCurrentState = AS_Idle;
    _vActuatorStates.resize(robot->GetDOF(),AS_Undefined);
    _vReadValues.resize(robot->GetDOF());
    _vReadVelocities.resize(robot->GetDOF());
    _vReadTorques.resize(robot->GetDOF());

    int offset = 0;
    time_t timesecs;
    time(&timesecs);
    struct tm *timenow = localtime(&timesecs);

    string tempargs = args;
    const char* pdelim = " \n\r\t";
    char* token = strtok(&tempargs[0], pdelim);
    if( token == NULL ) goto PlayerNotPresent;
    host = token;

    token = strtok(NULL, pdelim);
    if( token == NULL ) goto PlayerNotPresent;
    _port = atoi(token);

    try {
        // Connect to the server
        RAVEPRINT(L"PlayerClient connecting to [%s:%d]\n", host.c_str(), _port);
        pclient = new PlayerClient(host, _port);
        
        // set the timeout to 5s, remove this if not present
        // necessary for some driver initialization!
        pclient->SetRequestTimeout(5);
        _vclients.push_back(pclient);
    }
    catch(...) {
        goto PlayerNotPresent;
    }

    robot->GetJointMaxVel(vmaxvel);

    try {
        try {

            // process the proxies
            while( (token = strtok(NULL, pdelim)) != NULL) {
                
                if( stricmp(token, "actarray") == 0 ) {
                    token = strtok(NULL, pdelim);
                    if( token == NULL ) break;
                    int index = atoi(token);
                    
                    PLAYERPROXY px;
                    
                    px.pproxy = new ActArrayProxy(pclient, index);
                    
                    u32 timestamp = timeGetTime();
                    px._pclient = pclient;
                    px.offset = offset;
                    px.pproxy->RequestGeometry();
                    px.vgeom.resize(px.pproxy->GetCount());
                    
                    RAVEPRINT(L"PlayerController: ActArrayProxy:%d, time to geom=%dms\n", index, timeGetTime()-timestamp);
                    
                    if( px.vgeom.size() == 0 ) {
                        RAVEPRINT(L"proxy has 0 actuators, ignoring...\n");
                        continue;
                    }
                    
                    for(int i = 0; i < px.vgeom.size(); ++i) {
                        px.vgeom[i] = px.pproxy->GetActuatorGeom(i);
                        
                        if( px.vgeom[i].min > PI ) px.vgeom[i].min = PI;
                        else if( px.vgeom[i].min < -PI ) px.vgeom[i].min = -PI;
                        if( px.vgeom[i].max > PI ) px.vgeom[i].max = PI;
                        else if( px.vgeom[i].max < -PI ) px.vgeom[i].max = -PI;
                        

                        //s_jointset[dJointGetType(robot->GetJoint(offset+i)->joint)](robot->GetJoint(offset+i)->joint, dParamLoStop + dParamGroup * i, px.vgeom[i].min);
                        //s_jointset[dJointGetType(robot->GetJoint(offset+i)->joint)](robot->GetJoint(offset+i)->joint, dParamHiStop + dParamGroup * i, px.vgeom[i].max);
                        RAVEPRINT(L"act%d:%d min: %f max: %f, speed=%f\n", index, i, px.vgeom[i].min, px.vgeom[i].max, vmaxvel[offset+i]);
                    }
                    _vecproxies.push_back(px);
                    offset += px.pproxy->GetCount();
                }
                else if( stricmp(token, "offset") == 0 ) { // add an offset
                    offset += atoi(strtok(NULL, pdelim));
                    if( token == NULL ) break;
                }
                else if( stricmp(token, "client") == 0 ) { // create a new client

                    char* phost = strtok(NULL, pdelim);
                    if( phost == NULL ) goto PlayerNotPresent;
                    host = phost;
                    
                    char* pport = strtok(NULL, pdelim);
                    if( pport == NULL ) goto PlayerNotPresent;
                    _port = atoi(pport);
                    
                    RAVEPRINT(L"PlayerClient connecting to [%s:%d]\n", host.c_str(), _port);
                    pclient = new PlayerClient(host, _port);
                    pclient->SetRequestTimeout(5);
                    _vclients.push_back(pclient);
                }
            }
        }
        catch(PlayerError& err) {
            RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
        }
    }
    catch(...) {
        RAVEPRINT(L"unexpected exception\n");
    }

    if( _vecproxies.size() == 0 )
        goto PlayerNotPresent;

    system("mkdir trajectories");
    logid = 1;
    char filename[128];
    sprintf(filename, "trajectories/%S", _probot->GetName());

    // add a timestamp
    strftime(filename+strlen(filename),sizeof(filename)-strlen(filename),"_%y%m%d_%H%M%S.log",timenow);
    flog.open(filename);

    _mode = PLAYER_DATAMODE_PULL;

    try {
        try {
            // set the speed
            FOREACH(itproxy, _vecproxies) {
                for(int i = 0; i < (int)itproxy->vgeom.size(); ++i)
                    itproxy->pproxy->SetSpeedConfig(i, vmaxvel[itproxy->offset+i]); // set the speed from the xml file
            }

            FOREACH(itclient, _vclients)
                (*itclient)->SetDataMode(_mode);
            
            StartThread();
        }
        catch(PlayerError& err) {
            RAVEPRINT(L"failed to set speed: %s\n", err.GetErrorStr().c_str());
            Destroy();
            return false;
        }
    }
    catch(...) {
        RAVEPRINT(L"unexpected exception\n");
        Destroy();
        return false;
    }

    return true;

 PlayerNotPresent:
    RAVEPRINT(L"Error in initializing Player\n");
    Destroy();
    return false;
}

void PlayerController::Reset(int options)
{
    MutexLock m(&_mutexReadVals);
    _listQueuedTrajectories.clear();
    _trajCurrent._id = 0;
    _bIsDone = true;
    _bPause = false;
    memset(&_vActuatorStates[0], 0, sizeof(_vActuatorStates[0])*_vActuatorStates.size());
    _asCurrentState = AS_Idle;

    FOREACH(itclient, _vclients)
        (*itclient)->SetDataMode(PLAYER_DATAMODE_PUSH);

    FOREACH(itproxy, _vecproxies) {
        itproxy->bIgnore = false;
        itproxy->_nCurExecutingTrajId = 0;

        try {
            // reset
            itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_OFF);
            // enable again
            itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_ON);
        }
        catch(PlayerError& err) {
            RAVEPRINT(L"failed to set power: %s\n", err.GetErrorStr().c_str());
        }
    }

    FOREACH(itclient, _vclients)
        (*itclient)->SetDataMode(_mode);
}

bool PlayerController::SetDesired(const dReal* pValues)
{
    assert( _probot != NULL );
    
    // if paused, set the desired right away!!
    if( _bPause ) {
        _probot->SetJointValues(NULL, NULL, pValues, true);
        //_bIsDone = true;
        return false;
    }

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(_probot->GetDOF()));
    Trajectory::TPOINT tp;
    _probot->GetJointValues(tp.q);
    ptraj->AddPoint(tp);
    
    for(int i = 0; i < _probot->GetDOF(); ++i) tp.q[i] = pValues[i];
    ptraj->AddPoint(tp);
    ptraj->CalcTrajTiming(_probot, Trajectory::LINEAR, true, false);

    return SetPath(ptraj.get());
}

bool PlayerController::SetPath(const Trajectory* ptraj)
{
    if( _bPause || ptraj == NULL)
        return false;

    return SetPath(ptraj, logid++, 0);
}

bool PlayerController::SetPath(const Trajectory* ptraj, int nTrajectoryId, float fDivergenceTime)
{
#ifndef dSINGLE
    assert(0);
#endif
    
    if( _bPause || ptraj == NULL)
        return false;

    assert( ptraj->GetPoints().back().q.size() == GetDOF() );

    flog << "trajectory " << nTrajectoryId << endl;
    ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    flog << endl;
    
    _bIsDone = false;

    if( _vclients.size() > 0 ) {

        int trajformat = PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION;
        if( _bSendTimestamps )
            trajformat |= PLAYER_ACTARRAY_TRAJFMT_FLAG_TIMESTAMPS;    
        if( _bHoldOnStall )
            trajformat |= PLAYER_ACTARRAY_TRAJFMT_FLAG_HOLD_ON_STALL;

        // send the trajectory
        try {
            static vector<float> vdata;
            
            int dof = 0;
            FOREACH(itproxy, _vecproxies) {
                
                // have to divide the trajectory
                size_t proxydof = itproxy->vgeom.size();
                dof = itproxy->offset;
                
                if( itproxy->bIgnore ) {
                    continue;
                }
                
                vdata.resize(ptraj->GetPoints().size()*(_bSendTimestamps?(proxydof+1):proxydof));
                float* pfdata = &vdata[0];
                for(size_t i = 0; i < ptraj->GetPoints().size(); ++i, pfdata += proxydof) {
#ifdef dSINGLE
                    if( _bSendTimestamps )
                        *pfdata++ = ptraj->GetPoints()[i].time;
                    memcpy(pfdata, &ptraj->GetPoints()[i].q[dof], sizeof(dReal)*proxydof);
#endif
                }
                
                int err = itproxy->pproxy->MoveWithTrajectory((int)ptraj->GetPoints().size(), trajformat, fDivergenceTime, nTrajectoryId, &vdata[0]);
                dof += proxydof;
                
                if( err < 0 )
                    RAVEPRINT(L"error in MoveWithTrajectory: %d\n", err);
            }
            
        }
        catch(...) {
            RAVEPRINT(L"Failed to send trajectory\n");
        }

        MutexLock m(&_mutexReadVals);
        _listQueuedTrajectories.push_back(TRAJECTORY(nTrajectoryId, _saNextStallAction, _strStallCommand, _vecproxies));
    }
    
    return true;
}

void PlayerController::SetVelocities(const std::vector<dReal>& vels)
{
    assert( _probot != NULL );

    if( _bPause ) {
        return;
    }

    if(_probot->GetDOF() != vels.size()) {
        RAVEPRINT(L"ERROR in PlayerController::SetVelocities - Input vector must be same size as number of robot DOF, controller stopped.\n");
        return;
    }
    
    if( _bPause ) {
        RAVELOG(L"PlayerController cannot set velocities when paused\n");
        return;
    }

    MoveAtSpeed(&vels[0]);
}

bool PlayerController::SendCmd(const char* pcmd)
{
    assert(_probot != NULL );

    RAVEPRINT(L"sendcmd: %s\n", pcmd);
    string cmd = pcmd;
    char* ptoken = strtok(&cmd[0], " ");
    if( stricmp(ptoken, "setvel") == 0 ) {
        vector<dReal> vels(_probot->GetDOF(),0);
        for(int i = 0; i < vels.size(); ++i)
            vels[i] = atof(strtok(NULL, " "));
        SetVelocities(vels);
    }
    else if( stricmp(ptoken, "setjointvel") == 0) {
        vector<dReal> vels(_probot->GetDOF(),0);
        int index = atoi(strtok(NULL, " "));
        if( index < 0 || index >= _probot->GetDOF() ) {
            RAVEPRINT(L"PlayerController setjointvel invalid index = %d\n", index);
            return false;
        }
        vels[index] = atof(strtok(NULL, " "));
        SetVelocities(vels);
    }
    else if( stricmp(ptoken, "setspeed") == 0 ) {
        vector<dReal> vels(_probot->GetDOF(),0);
        int index = atoi(strtok(NULL, " "));
        if( index < 0 || index >= _probot->GetDOF() ) {
            RAVEPRINT(L"PlayerController setjointvel invalid index = %d\n", index);
            return false;
        }
        float fspeed = atof(strtok(NULL, " "));

        FOREACH(itproxy, _vecproxies) {
            if( itproxy->offset <= index && index < itproxy->offset+itproxy->vgeom.size() ) {
                itproxy->_pclient->SetDataMode(PLAYER_DATAMODE_PUSH);
                itproxy->pproxy->SetSpeedConfig(index-itproxy->offset, fspeed);
                itproxy->_pclient->SetDataMode(_mode);
            }
        }
    }
    else if( stricmp(ptoken, "pause") == 0 ) {
        char* p = strtok(NULL," ");
        _bPause = (p == NULL || atoi(p) > 0);
    }
    else if( stricmp(ptoken, "timestamps") == 0 ) {
        char* p = strtok(NULL," ");
        _bSendTimestamps = (p == NULL || atoi(p) > 0);
    }
    else if( stricmp(ptoken, "hold_on_stall") == 0 ) {
        char* p = strtok(NULL, " ");
        if( p != NULL ) {
            _bHoldOnStall = !!atoi(p);
            RAVELOG(L"new trajectories hold on stall = %d\n", (int)_bHoldOnStall);
        }
    }
    else if( stricmp(ptoken, "ignoreproxy") == 0 ) {
        // set bits to ignore a proxy, will read the proxy ids until end
        // an ignored proxy will not accept any new commands from the player and it
        // will be treated as a passive proxy
        char* p = strtok(NULL," ");
        FOREACH(itproxy, _vecproxies)
            itproxy->bIgnore = false;
        while(p != NULL ) {
            int ind = atoi(p);
            if( ind >= 0 && ind < (int)_vecproxies.size() ) {
                RAVELOG(L"PlayerController ignoring proxy %d\n", ind);
                _vecproxies[ind].bIgnore = true;
            }
            p = strtok(NULL, " ");
        }
    }
    else if( stricmp(ptoken, "stallaction") == 0 ) {
        // set bits to ignore a proxy, will read the proxy ids until end
        char* p = strtok(NULL," ");
        if( stricmp(p, "nothing") == 0 )
            _saNextStallAction = SA_Nothing;
        else if( stricmp(p, "stopall") == 0 )
            _saNextStallAction = SA_StopAll;
        else if( stricmp(p, "cancel") == 0 )
            _saNextStallAction = SA_CancelCurrent;
        else if( stricmp(p, "continue") == 0 )
            _saNextStallAction = SA_ContinueCurrent;
        else if( stricmp(p, "cancel_with_cmd") == 0 ) {
            _saNextStallAction = SA_CancelAndExecCommand;
            _strStallCommand = p+strlen(p)+1;
            RAVELOG(L"cancelling with command execution: %s\n", _strStallCommand.c_str());
        }
        else {
            RAVEPRINT(L"bad stall action, setting to nothing\n");
            _saNextStallAction = SA_Nothing;
        }
    }
    else if( stricmp(ptoken, "brake") == 0 ) {

        if( _bPause ) {
            RAVELOG(L"paused, so cannot accept braked command\n");
        }
        else {
            char* p = strtok(NULL," ");
            if( p != NULL ) {
                int dobrake = atoi(p);
                
                RAVELOG(L"braking\n");
                FOREACH(itproxy, _vecproxies) {
                    if( itproxy->bIgnore ) continue;
                    itproxy->pproxy->SetBrakesConfig(dobrake);
                }
            }
        }
    }
    else if( stricmp(ptoken, "reset") == 0 ) {
        Reset(0);
    }
    else {
        RAVEPRINT(L"Unrecognized command: %s\n", pcmd);
        return false;
    }

    return true;
}

bool PlayerController::SupportsCmd(const char* pcmd)
{
    return stricmp(pcmd, "setvel") == 0 || stricmp(pcmd, "pause") == 0 || 
        stricmp(pcmd, "ignoreproxy") == 0 || stricmp(pcmd, "timestamps") == 0 ||
        stricmp(pcmd, "setjointvel") == 0 || stricmp(pcmd, "stallaction") == 0 ||
        stricmp(pcmd, "brake") == 0 || stricmp(pcmd, "reset") == 0 ||
        stricmp(pcmd, "hold_on_stall") == 0 || stricmp(pcmd, "setspeed") == 0;
}

bool PlayerController::SimulationStep(dReal fTimeElapsed)
{
    if( _vclients.size() == 0 )
        return false;
    
    if( _bDestroyThread ) {
        StopThread();
        Destroy();
        return false;
    }

    vector<dReal> values;
        
    // get the current pose
    if( !_bPause && timeGetTime() - nLastRecvTime > nFeedbackTime ) {
        
        values.resize(_probot->GetDOF());
        {
            MutexLock m(&_mutexReadVals);
            values = _vReadValues;
        }

        _probot->SetJointValues(NULL, NULL, &values[0], true);
        nLastRecvTime = timeGetTime();
    }
    else _probot->GetJointValues(values);

    return IsDone();
}

bool PlayerController::MoveTo(const dReal* pdesired)
{
    if( _bPause ) {
        _probot->SetJointValues(NULL, NULL, pdesired, true);
        return true;
    }
    
    try {
        int dof = 0;
        FOREACH(itproxy, _vecproxies) {
            
            if( itproxy->bIgnore ) {
                continue;
            }
            
            dof = itproxy->offset;
            for(int i = 0; i < itproxy->vgeom.size()  && dof < _probot->GetDOF(); ++i, ++dof) {
                itproxy->pproxy->MoveTo(i, CLAMP_ON_RANGE((float)pdesired[dof], itproxy->vgeom[i].min, itproxy->vgeom[i].max));
            }
        }
    }
    catch(...) {
        RAVEPRINT(L"Failed to communicate with Player\n");
        return false;
    }
    
    return true;
}

bool PlayerController::MoveAtSpeed(const dReal* pdesiredVel)
{
    if( _bPause ) {
        return true;
    }

    try {
        int dof = 0;
        FOREACH(itproxy, _vecproxies) {

            if( itproxy->bIgnore ) continue;

            dof = itproxy->offset;
            for(int i = 0; i < itproxy->vgeom.size()  && dof < _probot->GetDOF(); ++i, ++dof) {
                itproxy->pproxy->MoveAtSpeed(i, pdesiredVel[dof]);
            }
        }
    }
    catch(...) {
        RAVEPRINT(L"Failed to communicate with Player\n");
        return false;
    }

    return true;
}

float PlayerController::GetTime() const
{
    // hopefully the times are all the same
    int i = 0;
    FOREACH(itproxy, _vecproxies) {

        if( i < _trajCurrent._vignore.size() && _trajCurrent._vignore[i] )
            continue;
        
        if( itproxy->_nCurExecutingTrajId > 0 )
            return itproxy->_fCurTrajTime;
        
        ++i;
    }
    
    return 0;
}

void PlayerController::GetVelocity(std::vector<dReal>& vel) const
{
    MutexLock m(&_mutexReadVals);
    vel = _vReadVelocities;
}

void PlayerController::GetTorque(std::vector<dReal>& torque) const
{
    MutexLock m(&_mutexReadVals);
    torque = _vReadTorques;
}

PlayerController::ActuatorState PlayerController::GetActuatorState(int index) const
{
    assert( index >= 0 && index < _probot->GetDOF() );
    return _vActuatorStates[index];
}

void PlayerController::StartThread()
{
    assert(NULL == _pThread);
    pthread_mutex_init(&_mutexReadVals, NULL);
    _pThread = new boost::thread(boost::bind(&PlayerController::RunThread, this));
}

void PlayerController::StopThread()
{
    assert(_pThread);
    _bStopThread = true;
    _pThread->join();
    delete _pThread;
    _pThread = NULL;
    pthread_mutex_destroy(&_mutexReadVals);
}

// non-blocking
void PlayerController::RunThread()
{
    if( _probot == NULL )
        return;
    
    _bStopThread = false;
    _bDestroyThread = false;
    vector<PLAYERPROXY> _veclocalproxies = _vecproxies;

    RAVEPRINT(L"PlayerController: starting thread\n");
    while (!_bStopThread) {

        if( !_bDestroyThread ) {
            try {
                if( _mode == PLAYER_DATAMODE_PUSH) {
                    FOREACH(itclient, _vclients) {
                        if ((*itclient)->Peek()) {
                            (*itclient)->Read();
                        }
                    }
                }
                else {
                    FOREACH(itclient, _vclients)
                        (*itclient)->Read();
                }
                
                MutexLock m(&_mutexReadVals);

                ActuatorState robotstate = AS_Idle, nextrobotstate = AS_Idle, trajrobotstate = AS_Idle;
                int nReadyTrajId = 0; // -1 - bad traj, 0 - no traj, > 0 - the traj id

                int nExecTraj = 0;

                // priority stalled > ready > braked > moving > idle
                
                TRAJECTORY nexttraj; // possibly next trajectory on the queue
                if( _listQueuedTrajectories.size() > 0 )
                    nexttraj = _listQueuedTrajectories.front();

                vector<bool>::iterator itnextignore = nexttraj._vignore.begin();
                vector<bool>::iterator itcurignore = _trajCurrent._vignore.begin();
                
                FOREACH(itproxy, _veclocalproxies) {

                    itproxy->_nCurExecutingTrajId = itproxy->pproxy->GetTrajectoryId();
                    itproxy->_fCurTrajTime = itproxy->pproxy->GetTrajectoryTime();
                    RAVELOG(L"proxy%d: traj=%d, state=%d\n", (int)(itproxy-_veclocalproxies.begin()), itproxy->_nCurExecutingTrajId, itproxy->pproxy->GetActuatorData(0).state);
                    
                    int nDOFReady=0;

                    int dof = itproxy->offset;
                    for(int i = 0; i < itproxy->vgeom.size()  && dof < _probot->GetDOF(); ++i, ++dof) {
                        player_actarray_actuator_t act = itproxy->pproxy->GetActuatorData(i);
                        _vReadValues[dof] = act.position;
                        _vReadVelocities[dof] = act.speed;
                        _vReadTorques[dof] = act.current;

                        switch(act.state) {
                        case PLAYER_ACTARRAY_ACTSTATE_IDLE:
                            _vActuatorStates[i] = AS_Idle;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_MOVING:
                            _vActuatorStates[i] = AS_Moving;
                            if( robotstate == AS_Idle )
                                robotstate = AS_Moving;
                            if( _listQueuedTrajectories.size() > 0 && !*itnextignore && nextrobotstate == AS_Idle )
                                nextrobotstate = AS_Moving;
                            if( _trajCurrent._vignore.size() > 0 && !*itcurignore && nextrobotstate == AS_Idle )
                                trajrobotstate = AS_Moving;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_STALLED:
                            _vActuatorStates[i] = AS_Stalled;
                            robotstate = AS_Stalled;
                            if( _listQueuedTrajectories.size() > 0 && !*itnextignore )
                                nextrobotstate = AS_Stalled;
                            if( _trajCurrent._vignore.size() > 0 && !*itcurignore )
                                trajrobotstate = AS_Stalled;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_BRAKED:
                            _vActuatorStates[i] = AS_Braked;
                            if( robotstate != AS_Stalled )
                                robotstate = AS_Braked;
                            if( _listQueuedTrajectories.size() > 0 && !*itnextignore && nextrobotstate != AS_Stalled )
                                nextrobotstate = AS_Braked;
                            if( _trajCurrent._vignore.size() > 0 && !*itcurignore && nextrobotstate != AS_Stalled )
                                trajrobotstate = AS_Braked;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_READY:
                            nDOFReady++;
                            break;
                        default:
                            RAVELOG(L"undefined actuator state %d!\n", act.state);
                            _vActuatorStates[i] = AS_Undefined;
                            break;
                        }
                    }

                    if( nDOFReady == itproxy->vgeom.size() && nextrobotstate != AS_Stalled ) {
                        // if all actuators are ready check for executing trajectory
                        if( itproxy->_nCurExecutingTrajId > 0 ) {
                            if( nReadyTrajId == 0 )
                                nReadyTrajId = itproxy->_nCurExecutingTrajId;
                            else if( nReadyTrajId != itproxy->_nCurExecutingTrajId )
                            nReadyTrajId = -1;
                        }
                        else nReadyTrajId = -1;
                    }

                    if( itproxy->_nCurExecutingTrajId > 0 ) {
                        if( nExecTraj == 0 )
                            nExecTraj = itproxy->_nCurExecutingTrajId;
                        else if( nExecTraj != itproxy->_nCurExecutingTrajId )
                            nExecTraj = -1;
                    }

                    ++itnextignore;
                    ++itcurignore;
                }
                
                // only execute a ready trajectory if the entire robot is NOT moving
                // Because of the ability to give different proxies different commands, have to make
                // sure that new commands are only executed when the previous command (which can affect
                // only a certain set of proxies) is done/stalled/braked.
                if( nReadyTrajId > 0 && robotstate != AS_Moving && robotstate != AS_Stalled ) {
                    // ready to execute the next trajectory
                    
                    list<TRAJECTORY>::iterator it = _listQueuedTrajectories.begin();
                    while(it != _listQueuedTrajectories.end()) {
                        if( it->_id == nReadyTrajId )
                            break;
                        ++it;
                    }

                    if( it == _listQueuedTrajectories.end() ) {
                        RAVEPRINT(L"error, trajectory %d not found!\n", nReadyTrajId);
                    }
                    else {
                        if( it != _listQueuedTrajectories.begin() ) {
                            RAVELOG(L"starting wrong trajectory %d, removing others\n", nReadyTrajId);
                            _listQueuedTrajectories.erase(_listQueuedTrajectories.begin(), it);
                            it = _listQueuedTrajectories.begin();
                        }
                        else
                            RAVELOG(L"starting trajectory %d\n", nReadyTrajId);
                        
                        _trajCurrent = *it;
                        _listQueuedTrajectories.erase(it);

                        vector<bool>::iterator itignore = _trajCurrent._vignore.begin();
                        FOREACH(itproxy, _veclocalproxies) {
                            if( *itignore++ ) continue;
                            
                            itproxy->pproxy->Start(0); // start right away
                        }

                        // change the robot state
                        if( robotstate == AS_Idle || robotstate == AS_Braked )
                            robotstate = AS_Moving;
                    }
                }

                if( _trajCurrent._id != nExecTraj)
                    RAVELOG(L"executing traj different (cur) %d != (exec) %d! (%d)\n", _trajCurrent._id, nExecTraj, _listQueuedTrajectories.size());
                
                // if a trajectory is running, use that state
                if( _trajCurrent._id > 0 )
                    robotstate = trajrobotstate;
                
                //wait for the first movement to start monitoring trajectory
                if( _asCurrentState != robotstate ) {
                    
                    RAVELOG(L"new robotstate: %s, traj=%d\n",s_mapPlayerStates[robotstate].c_str(), _trajCurrent._id); //test printout
                
                    switch(robotstate){
                    case AS_Stalled: //the controller has detected a probable collision
                        //check if you want to stop on stall for this trajectory
                        if(_trajCurrent._id > 0 ) {
                        
                            switch(_trajCurrent._stallaction) {
                            case SA_StopAll: {
                                RAVEPRINT(L"Robot is stalled, stopping all\n");
                                
                                vector<bool>::iterator itignore = _trajCurrent._vignore.begin();
                                FOREACH(itproxy, _veclocalproxies) {
                                    if( *itignore++ || itproxy->_nCurExecutingTrajId != _trajCurrent._id ) continue;
                                    
                                    itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_OFF); //this dumps all trajectories queued in the controller
                                }
                                
                                itignore = _trajCurrent._vignore.begin();
                                FOREACH(itproxy, _veclocalproxies) {
                                    if( *itignore++ || itproxy->_nCurExecutingTrajId != _trajCurrent._id ) continue;
                                    
                                    itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_ON); //this re-enables trajectory control for the robot
                                }
                                _bIsDone = true;
                                _trajCurrent._id = 0; // reset
                                break;
                            }
                            case SA_CancelCurrent: {
                                RAVEPRINT(L"Robot is stalled, going to next\n");
                                
                                try {
                                    vector<bool>::iterator itignore = _trajCurrent._vignore.begin();
                                    FOREACH(itproxy, _veclocalproxies) {
                                        if( *itignore++ || itproxy->_nCurExecutingTrajId != _trajCurrent._id ) continue;
                                        itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_CANCELCURRENT);
                                    }
                                }
                                catch(PlayerError& err) {
                                    if( err.GetErrorCode() == -2 ) {
                                        RAVEPRINT(L"failed to cancel current trajectory\n");
                                    }
                                    else RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
                                }
                                
                                _trajCurrent._id = 0; // reset
                                break;
                            }
                            case SA_CancelAndExecCommand: {
                                RAVEPRINT(L"Cancelling current command executing: %s\n", _trajCurrent._strStallCommand.c_str());
                                
                                // execute the problem command

                                try {
                                    vector<bool>::iterator itignore = _trajCurrent._vignore.begin();
                                    FOREACH(itproxy, _veclocalproxies) {
                                        if( *itignore++ || itproxy->_nCurExecutingTrajId != _trajCurrent._id ) continue;
                                        itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_CANCELCURRENT);
                                    }
                                }
                                catch(PlayerError& err) {
                                    if( err.GetErrorCode() == -2 ) {
                                        RAVEPRINT(L"failed to cancel current trajectory\n");
                                    }
                                    else RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
                                }
                                
                                _trajCurrent._id = 0; // reset
                                break;
                            }
                            case SA_ContinueCurrent: {
                                RAVELOG(L"Robot is stalled, continuing\n");
                                
                                vector<bool>::iterator itignore = _trajCurrent._vignore.begin();
                                FOREACH(itproxy, _veclocalproxies) {
                                    if( *itignore++ || itproxy->_nCurExecutingTrajId != _trajCurrent._id ) continue;
                                    itproxy->pproxy->Start(0);
                                }
                                
                                break;
                            }
                            case SA_Nothing:
                                RAVELOG(L"Robot is stalled, doing nothing\n");
                            }
                            
                            
                            // 
                        }

                        
                        break;
                    case AS_Moving:
                        break;
                    }
                    
                    _asCurrentState = robotstate;
                }
                
                switch(_asCurrentState) {
                case AS_Stalled:
                    // just cancel the current command for those proxies that do not share the current trajectory
                    try {
                        FOREACH(itproxy, _veclocalproxies) {
                            if( _trajCurrent._id > 0 && (itproxy->_nCurExecutingTrajId == _trajCurrent._id) )
                                continue;
                            //RAVEPRINT(L"cancelling current (offset=%d)\n", itproxy->offset);
                            itproxy->pproxy->SetPowerConfig(PLAYER_ACTARRAY_POWER_CANCELCURRENT);
                        }
                    }
                    catch(PlayerError& err) {
                        if( err.GetErrorCode() == -2 ) {
                            RAVEPRINT(L"failed to cancel current trajectory\n");
                        }
                        else RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
                    }
                    break;
                    
                case AS_Braked: //trajectory is finished and is holding position at the end
                    // only done when no more queued trajectories and
                    // each of the drivers are returning 0 for their trajectory id
                    if( _listQueuedTrajectories.size() == 0 && nExecTraj == 0) {
                        RAVELOG(L"robot %S done\n", _probot->GetName());
                        _trajCurrent._id = 0;
                        _bIsDone = true;
                    }
                    break;      
                case AS_Idle: //arm has gone into idle mode for some reason
                    // only reset traj if not ready
                    if( nReadyTrajId <= 0 )
                        _trajCurrent._id = 0;
                    if( _listQueuedTrajectories.size() == 0 && _trajCurrent._id == 0 )
                        _bIsDone = true;
                    break;  
                }
                
                pthread_mutex_unlock(&_mutexReadVals);
            }
            catch(...) {
                RAVEPRINT(L"PlayerController: failed to read from player...\n");
                //_bDestroyThread = true;
            }
        }
    
        usleep(500);
    }
}
