// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
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


#include "simpleplayercontroller.h"
int SimplePlayerController::_pos3dIndex = 0;
float SimplePlayerController::smalloffset = 0.001;

SimplePlayerController::SimplePlayerController(EnvironmentBase* penv) : ControllerBase(penv)
{
    _port = 0; // default
    nLastRecvTime = 0;
    nFeedbackTime = 40; // update robot at 25fps
    _pclient = NULL;
    _ptraj = NULL;
    fTime = 0;
    

    _bStopThread=true;
    _bDestroyThread = false;
    _pThread = NULL;
    _bPause = false;
    _bIsDone = true;
    nCheckTime = 0;
    _bSendTimestamps = false;
    _bMonitorTraj = false;
    _bStopNextTrajOnStall = false;
    _bMonitorActuatorStates = false;
    _psegwayclient = NULL;
}

SimplePlayerController::~SimplePlayerController()
{
    Destroy();
}

void SimplePlayerController::Destroy()
{
    if (!_bStopThread)
        StopThread();
    delete ppos3dproxy;
    for(int i = 0; i < _vecproxies.size(); ++i) delete _vecproxies[i].pproxy;
    _vecproxies.clear();
    delete _pclient; _pclient = NULL;

    flog.close();
}

bool SimplePlayerController::Init(RobotBase* robot, const char* args)
{
    _probot = robot;
    if( _probot == NULL || args == NULL )
        return false;

    // default values
    host = "";
    _port = 0;
    _ptraj = NULL;
    fTime = 0;
    _state = None;
    _segwaystate = None;
    _segwayDoneState = AS_Undefined;
    bplusminusoffset = false;
    _bPause = false;
    _vPrevValues.resize(robot->GetDOF());
    _vActuatorStates.resize(robot->GetDOF(),AS_Undefined);
    _vPrevActuatorStates.resize(robot->GetDOF(),AS_Undefined);

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

    // Connect to the server
    RAVEPRINT(L"PlayerClient connecting to [%s:%d]\n", host.c_str(), _port);

    try {
        _pclient = new PlayerClient(host, _port);
    }
    catch(...) {
        goto PlayerNotPresent;
    }
    
    // set the timeout to 5s, remove this if not present
    // necessary for some driver initialization!
    _pclient->SetRequestTimeout(5);
    try {
    try {

        // process the proxies
        while( (token = strtok(NULL, pdelim)) != NULL) {
            
            if( stricmp(token, "actarray") == 0 ) {
                token = strtok(NULL, pdelim);
                if( token == NULL ) break;
                int index = atoi(token);

                PLAYERPROXY px;
                
                px.pproxy = new ActArrayProxy(_pclient, index);
                
                u32 timestamp = timeGetTime();
                px.offset = offset;
                px.pproxy->RequestGeometry();
                px.vgeom.resize(px.pproxy->GetCount());
                px.bStopNextTrajOnStall = false;
                
                RAVEPRINT(L"SimplePlayerController: ActArrayProxy:%d, time to geom=%dms\n", index, timeGetTime()-timestamp);

                //RAVEPRINT(L"type: %d\n", px.pproxy->GetActuatorGeom(0).type);
                if( px.vgeom.size() == 0 ) {
                    RAVEPRINT(L"proxy has 0 actuators, ignoring...\n");
                    continue;
                }

                for(int i = 0; i < px.vgeom.size(); ++i) {
                    px.vgeom[i] = px.pproxy->GetActuatorGeom(i);
                    RAVEPRINT(L"act%d:%d min: %f max: %f\n", index, i, px.vgeom[i].min, px.vgeom[i].max);
                }
                _vecproxies.push_back(px);
                offset += px.pproxy->GetCount();
            }
            else if( stricmp(token, "offset") == 0 ) {
                offset += atoi(strtok(NULL, pdelim));
                if( token == NULL ) break;
            }
            else if( stricmp(token, "segway") == 0 ) {
                string segwayhost = strtok(NULL, pdelim);
                int segwayport = atoi(strtok(NULL, pdelim));
                RAVEPRINT(L"PlayerClient connecting to segway at [%s:%d]\n", segwayhost.c_str(), segwayport);
                try {
                    _psegwayclient = new PlayerClient(segwayhost, segwayport);
                }
                catch(...) {
                    goto PlayerNotPresent;
                }         
                //we don't want to request data from this server
                _psegwayclient->SetReplaceRule(true); //so message que doesn't fill up
                _psegwayclient->SetDataMode(PLAYER_DATAMODE_PULL);

                //initialize the planner proxy
                pplannerproxy = new PlannerProxy(_psegwayclient, 0);
            }
        }
    
        //initialize the position 3d proxy
        ppos3dproxy = new Position3dProxy(_pclient, _pos3dIndex);


    }
    catch(PlayerError& err) {
        RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
    }
    }
    catch(...) {
        RAVEPRINT(L"unexpected exception\n");
    }

//    catch(...) {
//        RAVEPRINT(L"SimplePlayerController: unknown exception\n");
//    }

    if( _vecproxies.size() == 0 )
        goto PlayerNotPresent;

    system("mkdir trajectories");
    logid = 0;
    char filename[128];
    sprintf(filename, "trajectories/%S", _probot->GetName());

    // add a timestamp
    strftime(filename+strlen(filename),sizeof(filename)-strlen(filename),"_%y%m%d_%H%M%S.log",timenow);
    flog.open(filename);

    _mode = PLAYER_DATAMODE_PULL;
    _pclient->SetDataMode(_mode);

    _pclient->SetRequestTimeout(3); // reset to something more reasonable
    StartThread();

    return true;

 PlayerNotPresent:
    RAVEPRINT(L"Error in initializing Player\n");
    Destroy();
    return false;
}

bool SimplePlayerController::SetDesired(const dReal* pValues)
{
    assert( _probot != NULL );
    
    // if paused, set the desired right away!!
    if( _bPause ) {
        _probot->SetJointValues(NULL, NULL, pValues, true);
        //_bIsDone = true;
        return false;
    }

    _vecdesired.resize(_probot->GetDOF());
    memcpy(&_vecdesired[0], pValues, sizeof(_vecdesired[0]) * _vecdesired.size());
    fTime = 0;
    _ptraj = NULL;
    _state = Servo;
    nCheckTime = timeGetTime();
    
    flog << "pos " << logid++ << endl;
    FOREACH(it, _vecdesired) flog << *it << " ";
    flog << endl;

    // if paused, set the desired right away!!
    _bIsDone = false;
    if( !MoveTo(&_vecdesired[0]) ) {
        RAVEPRINT(L"Failed to set desired\n");
    }

    return true;
}

bool SimplePlayerController::SetPath(const Trajectory* ptraj)
{
#ifndef dSINGLE
    assert(0);
#endif
    
    if( _bPause ) {
        RAVEPRINT(L"SimplePlayerController cannot play trajectories when paused\n");
        //_bIsDone = true;
        return false;
    }

    _ptraj = ptraj;
    fTime = 0;
    _state = None;
    nCheckTime = timeGetTime();

    if( ptraj == NULL )
        return false;

    assert( ptraj->GetPoints().back().q.size() == GetDOF() );

    flog << "trajectory " << logid++ << endl;
    ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    flog << endl;
    
    
    _bIsDone = false;
    _vecdesired = ptraj->GetPoints().back().q;
    nCheckTime = timeGetTime();
    
    if( _pclient != NULL ) {
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

                int err = itproxy->pproxy->MoveWithTrajectory(ptraj->GetPoints().size(),_bSendTimestamps?PLAYER_ACTARRAY_TRAJFMT_FLAG_TIMESTAMPS:0, 0, 0, &vdata[0]);
                dof += proxydof;

                if( err < 0 )
                    RAVEPRINT(L"error in MoveWithTrajectory: %d\n", err);
            }

            _state = Traj;
        }
        catch(...) {
            RAVEPRINT(L"Failed to send trajectory\n");
        }
    }

    return true;
}

void SimplePlayerController::SetVelocities(const std::vector<dReal>& vels)
{
    assert( _probot != NULL );
  
    if(_probot->GetDOF() != vels.size()) {
        RAVEPRINT(L"ERROR in SimplePlayerController::SetVelocities - Input vector must be same size as number of robot DOF, controller stopped.\n");
        _state = None;
        return;
    }
    
    if( _bPause ) {
        RAVELOG(L"SimplePlayerController cannot set velocities when paused\n");
        return;
    }

    fTime = 0;
    _ptraj = NULL;    
    _state = None;
    nCheckTime = timeGetTime()+2000; // always wait at least 4s in order for robot to get started

    if( MoveAtSpeed(&vels[0]) ) {
        _vPrevValues.resize(0); // have to reset here
        _state = Velocity;
    }
}

bool SimplePlayerController::SendCmd(const char* pcmd)
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
            RAVEPRINT(L"SimplePlayerController setjointvel invalid index = %d\n", index);
            return false;
        }
        vels[index] = atof(strtok(NULL, " "));
        SetVelocities(vels);
    }
    else if( stricmp(ptoken, "pause") == 0 ) {
        char* p = strtok(NULL," ");
        _bPause = (p == NULL || atoi(p) > 0);
    }
    else if( stricmp(ptoken, "timestamps") == 0 ) {
        char* p = strtok(NULL," ");
        _bSendTimestamps = (p == NULL || atoi(p) > 0);
    }
    else if( stricmp(ptoken, "ignoreproxy") == 0 ) {
        // set bits to ignore a proxy, will read the proxy ids until end
        char* p = strtok(NULL," ");
        FOREACH(itproxy, _vecproxies)
            itproxy->bIgnore = false;
        while(p != NULL ) {
            int ind = atoi(p);
            if( ind >= 0 && ind < (int)_vecproxies.size() ) {
                RAVELOG(L"SimplePlayerController ignoring proxy %d\n", ind);
                _vecproxies[ind].bIgnore = true;
            }
            p = strtok(NULL, " ");
        }
    }
    else if( stricmp(ptoken, "monitorstates") == 0 ) {
        char* p = strtok(NULL," ");
        _bMonitorActuatorStates = (p == NULL || atoi(p) > 0);
        if( _bMonitorActuatorStates )
            RAVELOG(L"SimplePlayerController monitoring actuator states\n");
        else
            RAVELOG(L"SimplePlayerController monitoring goal configurations\n");
    }
    else if( stricmp(ptoken, "stop_next_traj_on_stall") == 0 ) {
        // set bits to ignore a proxy, will read the proxy ids until end
        char* p = strtok(NULL," ");
        _bStopNextTrajOnStall = (bool)atoi(p);
        _bMonitorActuatorStates = true;
        p = strtok(NULL," ");
        while(p != NULL ) {
            int ind = atoi(p);
            if( ind >= 0 && ind < (int)_vecproxies.size() ) {
                RAVELOG(L"SimplePlayerController ignoring proxy %d\n", ind);
                _vecproxies[ind].bStopNextTrajOnStall = _bStopNextTrajOnStall;
            }
            p = strtok(NULL, " ");
        }
    }
    else if( stricmp(ptoken, "power") == 0 ) {
        int dopower = 0;
        char* p = strtok(NULL," ");
        if( p != NULL )
            dopower = atoi(p);
        RAVELOG(L"setting power to %d\n", dopower);
        FOREACH(itproxy, _vecproxies) {
            if( itproxy->bIgnore ) continue;
            itproxy->pproxy->SetPowerConfig(dopower);
        }
    }
    else if( stricmp(ptoken, "grabobject") == 0 ) {
        //format is [grabobject mass(1 float) cog in world coordinates(3 float) inertia(6 float: upper half of inertia matrix)] 
        std::vector<dReal> vals(10);

        char* p = strtok(NULL," ");
        for(int i = 0; i < 10; i++)
        {
            if( p == NULL)
            {
                RAVEPRINT(L"SimplePlayerController Error: not enough arguments for grabobject, command should be [grabobject mass(1 float) cog in world coordinates(3 float) inertia(6 float: upper half of inertia matrix)]\n", pcmd);                
                return false;
            }
            vals[i] = atof(p);
            p = strtok(NULL, " ");
        }

        player_pose3d_t pos;
        player_pose3d_t vel;
            
        pos.proll = vals[0];
        pos.px = vals[1];
        pos.py = vals[2];
        pos.pz = vals[3];

        vel.px = vals[4];
        vel.py = vals[5];
        vel.pz = vals[6];
        vel.proll = vals[7];
        vel.ppitch = vals[8];
        vel.pyaw = vals[9];
        try {
        try {
            RAVEPRINT(L"ABout to send!\n");
            ppos3dproxy->GoTo(pos, vel);
            RAVEPRINT(L"sent!\n");
        }
        catch(PlayerError& err) {
            RAVEPRINT(L"err: %s\n", err.GetErrorStr().c_str());
        }
        }
        catch(...) {
            RAVEPRINT(L"Can't send grabbed body parameters\n");
        }
         
    }
    else if( stricmp(ptoken, "drivesegway") == 0 ) {
        if(_psegwayclient == NULL)
            RAVEPRINT(L"Simpleplayercontrolker Error: Not connected to segway client, ignoring command.\n");
        else
        {

            //need to make sure commands aren't the same to detect when segway stops
            prevsegwaycommand[0] = segwaycommand[0];
            prevsegwaycommand[1] = segwaycommand[1];
            prevsegwaycommand[2] = segwaycommand[2];

            segwaycommand[0] = atof(strtok(NULL," "));
            segwaycommand[1] = atof(strtok(NULL," "));
            segwaycommand[2] = atof(strtok(NULL," "));

            if(prevsegwaycommand[0] == segwaycommand[0] && prevsegwaycommand[1] == segwaycommand[1] && prevsegwaycommand[2] == segwaycommand[2])
            {
                bplusminusoffset = !bplusminusoffset;
                if(bplusminusoffset)
                    segwaycommand[2] += smalloffset;
                else
                    segwaycommand[2] -= smalloffset;

            }
            
            pplannerproxy->SetGoalPose(segwaycommand[0], segwaycommand[1],segwaycommand[2]);
            RAVEPRINT(L"Sending command to segway\n");
            _bIsDone = false;
            _segwaystate = Traj;
        }


        /*
        //wait for the robot to finish executing the path
        RAVEPRINT(L"Waiting for segway to stop moving....\n");
        _psegwayclient->Read();
        int doneval = pplannerproxy->GetPathDone();
        RAVEPRINT(L"%d\n",doneval);
        //wait till planner starts running
        while(doneval)
        {
            usleep(50000);
            _psegwayclient->Read();
            doneval = pplannerproxy->GetPathDone();
            RAVEPRINT(L"%d\n",doneval);
        }
        //wait till trajectory stops
        while(!doneval)
        {
            usleep(50000);
            _psegwayclient->Read();
            doneval = pplannerproxy->GetPathDone();
            RAVEPRINT(L"%d\n",doneval);
        }
        RAVEPRINT(L"Segway stopped\n");
        */
    }
    else {
        RAVEPRINT(L"Unrecognized command: %s\n", pcmd);
        return false;
    }

    return true;
}

bool SimplePlayerController::SupportsCmd(const char* pcmd)
{
    return stricmp(pcmd, "setvel") == 0 || stricmp(pcmd, "pause") == 0 || 
        stricmp(pcmd, "ignoreproxy") == 0 || stricmp(pcmd, "timestamps") == 0 ||  stricmp(pcmd, "setjointvel") == 0 || stricmp(pcmd, "stop_next_traj_on_stall") == 0 || stricmp(pcmd,"power") == 0 ||
        stricmp(pcmd, "grabobject") == 0 || stricmp(pcmd, "monitorstates") == 0 || stricmp(pcmd, "drivesegway") == 0;
}

bool SimplePlayerController::SimulationStep(float fTimeElapsed)
{
    if( _pclient == NULL )
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
        pthread_mutex_lock(&_mutexReadVals);
        values = _vReadValues;
        pthread_mutex_unlock(&_mutexReadVals);

        _probot->SetJointValues(NULL, NULL, &values[0], true);
        nLastRecvTime = timeGetTime();
    }
    else _probot->GetJointValues(values);

    // if any proxies are getting ignored, set values to _vecdesired
    int dof = 0;
    FOREACH(itproxy, _vecproxies) {
        if( itproxy->bIgnore ) {
            for(size_t i = 0; i < itproxy->vgeom.size(); ++i) {
                if( i+dof < (int)_vecdesired.size() ) {
                    values[i+dof] = _vecdesired[i+dof];
                    if( i+dof < (int)_vPrevValues.size() ) {
                        _vPrevValues[i+dof] = _vecdesired[i+dof];
                    }
                }
            }
        }
        dof += itproxy->vgeom.size();
    }

    switch(_state) {
        case Traj:

            if( _bMonitorActuatorStates ) {
                
                //RAVEPRINT(L"actstate: %d\n",GetActuatorState(0)); //test printout
                //wait for the first movement to start monitoring trajectory
                if(!_bMonitorTraj) {
                    if((_vPrevActuatorStates[0] != AS_Moving) && (_vActuatorStates[0] == AS_Moving)) {
                        RAVEPRINT(L"traj just started, monitoring\n");
                        _bMonitorTraj = true;    
                    }
                    else {
                        //RAVELOG(L"waiting for traj to start, not monitoring yet %d %d %d\n",_bMonitorTraj,_vPrevActuatorStates[0],_vActuatorStates[0]);
                        break;
                    }
                }            
                switch(_vActuatorStates[0]){
                case AS_Stalled: //the controller has detected a probable collision
                    //check if you want to stop on stall for this trajectory                    
                    if(!_bStopNextTrajOnStall)
                        break;
                    
                    
                    try {
                        FOREACH(itproxy, _vecproxies) {
                            if(itproxy->bStopNextTrajOnStall)
                                itproxy->pproxy->SetPowerConfig(0); //this dumps all trajectories queed in the controller
                        }
                        
                    }
                    catch(...) {
                        RAVEPRINT(L"Failed to stop trajectory: %s\n", playerc_error_str());
                    }
                    
                    
                    try {
                        FOREACH(itproxy, _vecproxies) {
                            if(itproxy->bStopNextTrajOnStall) {
                                itproxy->pproxy->SetPowerConfig(1); //this re-enables trajectory control for the robot
                            }
                        }
                        
                    }
                    catch(...) {
                        RAVEPRINT(L"Failed to re-enable trajectories %s\n", playerc_error_str());
                    }
                    
                    _state = None;
                    _bIsDone = true;
                    _bMonitorTraj = false;
                    _bStopNextTrajOnStall = false;
                    FOREACH(itproxy, _vecproxies) {
                        itproxy->bStopNextTrajOnStall = false; //set this back to normal
                    }
                    RAVEPRINT(L"Robot is stalled, stopping\n");
                    break;
                case AS_Braked: //trajectory is finished and is holding position at the end
                    _state = None;
                    _bIsDone = true;
                    _bMonitorTraj = false;
                    _bStopNextTrajOnStall = false;
                    FOREACH(itproxy, _vecproxies) {
                        itproxy->bStopNextTrajOnStall = false; //set this back to normal
                    }
                    RAVEPRINT(L"Robot is braked, stopping\n");
                    break;      
                case AS_Idle: //arm has gone into idle mode for some reason
                    _state = None;
                    _bIsDone = true;
                    _bMonitorTraj = false;
                    _bStopNextTrajOnStall = false;
                    FOREACH(itproxy, _vecproxies) {
                        itproxy->bStopNextTrajOnStall = false; //set this back to normal
                    }
                    RAVEPRINT(L"Robot is idle, stopping\n");
                    break;  
                case AS_Moving:
                    //RAVELOG(L"Robot is moving, continuing\n");
                    break;
                }
                break; //it's important that this break is inside the ifdef
                
                
                //NEED TO HANDLE CASE WHERE ARM IS BRAKED AND FINGERS ARE MOVING IN A TRAJECTORY
                
            }
            // fall through
        case Servo:
            
            assert( _vecdesired.size() == _probot->GetDOF() );
            
            if( _probot->ConfigDist(&_vecdesired[0], &values[0]) < 0.08f ) { // have to make this big
                
                if( _vPrevValues.size() == values.size() ) {
                    // have to check signed
                    if( (int)(timeGetTime()-nCheckTime) > 800  ) {
                        
                        // check for changes
                        if( _probot->ConfigDist(&_vPrevValues[0], &values[0]) < 0.005f) {
                            _state = None;
                            _bIsDone = true;
                        }
                        else {
                            RAVELOG(L"player waiting for robot to stop: %d\n", _state);
                            _vPrevValues = values;
                        }
                        nCheckTime = timeGetTime();
                    }
                }
                else _vPrevValues = values;
            }
            else {
//                wstringstream ss;
//                ss << L"servo error: ";
//                for(int i = 0; i < _probot->GetDOF(); ++i) {
//                    if( fabsf(_vecdesired[i]-values[i]) > 0.01f )
//                        ss << _vecdesired[i]-values[i] << " ";
//                    else ss << "0 ";
//                }
//                ss << endl;
//                RAVEPRINT(ss.str().c_str());
                _vPrevValues = values;
            }
            
            
            break;
            
        case Velocity:
            if( _vPrevValues.size() == values.size() ) {
                // check for changes, have to check signed
                if( (int)(timeGetTime()-nCheckTime) > 400  ) {
                    if( _probot->ConfigDist(&_vPrevValues[0], &values[0]) < 0.005f) {
                        _state = None;
                        _bIsDone = true;
                        wstringstream s;
                        s << "Velocity done at: ";
                        FOREACH(it, values) s << *it << " ";
                        s << endl;
                        RAVELOG(s.str().c_str());
                    }
                    else {
                        RAVELOG(L"player waiting for robot to stop: %d\n", _state);
                        _vPrevValues = values;
                    }
                    nCheckTime = timeGetTime();
                }
                else _vPrevValues = values;
            }
            else
                _vPrevValues = values;

            break;

        case None:
            _bIsDone = true;
            break;
    }

    //some more complicated state handling for the segway may be needed
    if(_psegwayclient != NULL)
    {
        switch(_segwaystate){
            case Traj:

                switch(_segwayDoneState){
                    case AS_Stalled: 
                        break;
                    case AS_Braked: 
                        break;      
                    case AS_Idle:

                        //if the segway goal is set and we are in idle, then we must be at the goal
                        if(segwayplannergoal[0] == segwaycommand[0] && segwayplannergoal[1] == segwaycommand[1] && segwayplannergoal[2] == segwaycommand[2])
                        {
                            _segwaystate = None;
                            //maintain state of isdone, so the arm can finish _bIsDone = _bIsDone;
                            RAVEPRINT(L"Segway is idle, stopping\n");

                        }
                        else
                        {
                            _bIsDone = false;
                            //RAVEPRINT(L"Waiting for segway to receive command\n");
                        }




                        /*
                        //if segway was moving but went to idle then trajectory done
                        if(_prevsegwayDoneState == AS_Moving)
                        {
                         
                            _segwaystate = None;
                            //maintain state of isdone, so the arm can finish _bIsDone = _bIsDone;
                            RAVEPRINT(L"Segway is idle, stopping\n");
                        }
                        //if segway was idle and stayed idle, we are waiting for the segway to start moving
                        //but there is a time-out
                        else if(_prevsegwayDoneState == AS_Idle)
                        {
                        
                      
                        }
                        */

                        break;  
                    case AS_Moving:
                        //RAVEPRINT(L"Segway is moving\n");
                        _bIsDone = false;
                        break;
                }
                break;
            case None:
                break;
            case Servo:
                break;
        }

        _prevsegwayDoneState = _segwayDoneState;

    }


    _vPrevActuatorStates[0] = _vActuatorStates[0];
    
    return IsDone();
}

bool SimplePlayerController::MoveTo(const dReal* pdesired)
{
    if( _bPause ) {
        _probot->SetJointValues(NULL, NULL, pdesired, true);
        return true;
    }

    if( _pclient == NULL )
        return false;
    
    try {
        int dof = 0;
        FOREACH(itproxy, _vecproxies) {
            
            if( itproxy->bIgnore ) {
                continue;
            }
            
            dof = itproxy->offset;
            for(int i = 0; i < itproxy->vgeom.size()  && dof < _probot->GetDOF(); ++i, ++dof) {
                itproxy->pproxy->MoveTo(i, pdesired[dof]);//CLAMP_ON_RANGE(pdesired[dof], itproxy->vgeom[i].min, itproxy->vgeom[i].max));
            }
        }
    }
    catch(...) {
        RAVEPRINT(L"Failed to communicate with Player\n");
        return false;
    }
    
    return true;
}

bool SimplePlayerController::MoveAtSpeed(const dReal* pdesiredVel)
{
    if( _bPause ) {
        return true;
    }

    if( _pclient == NULL )
        return false;

    try {
        int dof = 0;
        FOREACH(itproxy, _vecproxies) {

            if( itproxy->bIgnore ) {
                continue;
            }

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

ControllerBase::ActuatorState SimplePlayerController::GetActuatorState(int index)
{
    if( index < 0 || index >= _probot->GetDOF() ) {
        RAVEPRINT(L"SimplePlayerController GetActuatorState invalid index = %d\n", index);
        return AS_Undefined;
    }

    return _vActuatorStates[index];
}


void SimplePlayerController::StartThread()
{
    assert(NULL == _pThread);
    pthread_mutex_init(&_mutexReadVals, NULL);
    _pThread = new boost::thread(boost::bind(&SimplePlayerController::RunThread, this));
}

void SimplePlayerController::StopThread()
{
    assert(_pThread);
    _bStopThread = true;
    _pThread->join();
    delete _pThread;
    _pThread = NULL;
    pthread_mutex_destroy(&_mutexReadVals);
}

// non-blocking
void SimplePlayerController::RunThread()
{
    if( _probot == NULL )
        return;
    
    _bStopThread = false;
    _bDestroyThread = false;
    vector<dReal> values(_probot->GetDOF());
    vector<uint8_t> actstates(_probot->GetDOF());
    vector<PLAYERPROXY> _veclocalproxies = _vecproxies;

    RAVEPRINT(L"SimplePlayerController: starting thread\n");
    while (!_bStopThread) {

        if( !_bDestroyThread ) {
            try {
                if( _mode == PLAYER_DATAMODE_PUSH) {
                    if (_pclient->Peek()) {
                        _pclient->Read();
                    }
                }
                else {
                    _pclient->Read();
                }

                if(_psegwayclient != NULL)
                {
                    _psegwayclient->Read();
                    if(pplannerproxy->GetPathDone())
                        _segwayDoneState = AS_Idle;
                    else
                        _segwayDoneState = AS_Moving;

                    segwayplannergoal[0] = pplannerproxy->GetGoal().px;
                    segwayplannergoal[1] = pplannerproxy->GetGoal().py;
                    segwayplannergoal[2] = pplannerproxy->GetGoal().pa;

                }

                int dof;
                FOREACH(itproxy, _veclocalproxies) {
                    
                    dof = itproxy->offset;
                    for(int i = 0; i < itproxy->vgeom.size()  && dof < _probot->GetDOF(); ++i, ++dof) {
                        values[dof] = itproxy->pproxy->GetActuatorData(i).position;
                        actstates[dof] = itproxy->pproxy->GetActuatorData(i).state;
                    }
                }

                // pass to thread
                pthread_mutex_lock(&_mutexReadVals);
                _vReadValues = values;
                for(int i =0; i < actstates.size();i++)
                {
                    
                    switch(actstates[i])
                    {
                        case PLAYER_ACTARRAY_ACTSTATE_IDLE:
                            _vActuatorStates[i] = AS_Idle;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_MOVING:
                            _vActuatorStates[i] = AS_Moving;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_STALLED:
                            _vActuatorStates[i] = AS_Stalled;
                            break;
                        case PLAYER_ACTARRAY_ACTSTATE_BRAKED:
                            _vActuatorStates[i] = AS_Braked;
                            break;
                        default:
                            _vActuatorStates[i] = AS_Undefined;
                            break;
                    }
                }
                pthread_mutex_unlock(&_mutexReadVals);
            }
            catch(...) {
                RAVEPRINT(L"SimplePlayerController: failed to read from player...\n");
                //_bDestroyThread = true;
            }
        }

        usleep(500);
        //boost::xtime xt;
//        boost::xtime_get(&xt, boost::TIME_UTC);
//        // we sleep for 500 us
//        xt.nsec += 500*1000;
//        boost::thread::sleep(xt);
    }
}
