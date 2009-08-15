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

// boosting
#include <algorithm>
#include <functional>
#include <boost/bind.hpp>

#include "playerproblem.h"

#ifdef _WIN32
LARGE_INTEGER g_lfreq;
#endif

u64 GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count;
    QueryPerformanceCounter(&count);
    return count.QuadPart * 1000000 / g_lfreq.QuadPart;
#else
    timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec*1000000+t.tv_usec;
#endif
}

/////////////////////
// Player Problem //
/////////////////////
PlayerProblem::PlayerProblem(EnvironmentBase* penv) : ProblemInstance(penv), robot(NULL)
{
#ifdef _WIN32
    QueryPerformanceFrequency(&g_lfreq);
#endif

    _pclient = NULL;
    _plaserclient = NULL;
    _plasermotorclient = NULL;
    _pposclient = NULL;
    _bStopPlayerThread=true;
    _bDestroyPlayerThread = false;
    _pThread = NULL;
    pthread_mutex_init(&_mutex_player, 0);
}

void PlayerProblem::Destroy()
{
    robot = NULL;

    if (!_bStopPlayerThread)
        StopPlayerThread();

    // close player vision server
    _bDestroyPlayerThread = false;

    delete _plaserclient; _plaserclient = NULL;
    delete _pposclient; _pposclient = NULL;
    delete _plasermotorclient; _plasermotorclient = NULL;
    delete _pclient; _pclient = NULL;
    pthread_mutex_destroy(&_mutex_player);
}

PlayerProblem::~PlayerProblem()
{
    Destroy();
}

/// cmd format:
/// robot_name [sensor name_of_sensor_system host port]
/// 
/// robot_name is the name of the robot that will be controlled by this problem
/// Can add more than one sensor systems by specifying 'sensor' then the above parameters.
int PlayerProblem::main(const char* cmd, EnvironmentBase* penv)
{
    if( cmd == NULL )
        return 0;

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = _ravembstowcs(p);


    // parse the commands
    p = strtok(p+strlen(p)+1, delim);
    while(p != NULL) {

        if( stricmp(p, "laser") == 0 ) {
            char* phost = strtok(NULL, delim);
            char* pport = strtok(NULL, delim);

            if( phost == NULL || pport == NULL ) {
                RAVEPRINT(L"sensor error\n");
                break;
            }

            int port = atoi(pport);

            RAVEPRINT(L"connecting to [%s:%d]\n", phost, port);
            
            try {
                _pclient = new PlayerClient(phost, port);
                
                // set the timeout to 5s, remove this if not present
                //_pclient->SetRequestTimeout(3);
                _pclient->SetDataMode(PLAYER_DATAMODE_PULL);
                
                _plaserclient = new LaserProxy(_pclient, 0);

                try {
                RAVEPRINT(L"requestgeom\n");
                _plaserclient->RequestGeom();
                } catch(...) {}
                
                //RAVEPRINT(L"RequestConfigure\n");
                //_plaserclient->RequestConfigure();
                
                RAVEPRINT(L"starting laser client: maxrange=%f, scanres=%f, rangeres=%f, minangle=%f, maxangle=%f\n",
                        (float)_plaserclient->GetMaxRange(), (float)_plaserclient->GetScanRes(), (float)_plaserclient->GetRangeRes(),
                        (float)_plaserclient->GetMinAngle(), (float)_plaserclient->GetMaxAngle());
            }
            catch(...) {
                RAVEPRINT(L"Error in initializing Player: %s\n", playerc_error_str());
                Destroy();
                return false;
            }

            // get the actarray motor
            try {
                _plasermotorclient = new ActArrayProxy(_pclient, 1);
                _plasermotorclient->RequestGeometry();
            }
            catch(...) {
                RAVEPRINT(L"failed to create actarray proxy\n");
                delete _plasermotorclient; _plasermotorclient = NULL;
            }

            StartPlayerThread();
        }
        else if( stricmp(p, "playerclient") == 0 ) {
            char* phost = strtok(NULL, delim);
            char* pport = strtok(NULL, delim);

            if( phost == NULL || pport == NULL ) {
                RAVEPRINT(L"sensor error\n");
                break;
            }

            int port = atoi(pport);

            RAVEPRINT(L"connecting to [%s:%d]\n", phost, port);
            
            try {
                _pclient = new PlayerClient(phost, port);
                
                // set the timeout to 5s, remove this if not present
                //_pclient->SetRequestTimeout(5);
            }
            catch(...) {
                RAVEPRINT(L"Error in initializing Player\n");
                Destroy();
                return false;
            }

            //_pclient->SetRequestTimeout(3);
            _pclient->SetDataMode(PLAYER_DATAMODE_PULL);
            StartPlayerThread();
        }
        else
            break;

        p = strtok(NULL, delim);
    }

    SetActiveRobots(GetEnv()->GetRobots());

    return 0;
}

void PlayerProblem::SetActiveRobots(const std::vector<RobotBase*>& robots)
{
    robot = NULL;

    vector<RobotBase*>::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( wcsicmp((*itrobot)->GetName(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVEPRINT(L"Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}

void PlayerProblem::SendCommand(const char* cmd, string& response)
{
    SetActiveRobots(GetEnv()->GetRobots());
    
    const char* delim = " \r\n\t";
    char* mycmd = strdup(cmd);
    char* p = strtok(mycmd, delim);
    while(p != NULL ) {
        if( stricmp(p, "laserdata") == 0 ) { // real
            GetRealLaserMap(response);
        }
        else if( stricmp(p, "laserdata2d") == 0 ) { // real
            GetRealLaserMap2D(response);
        }
        else if( stricmp(p, "createpos3d") == 0 ) { // create a position 3d interface

            if( _pclient == NULL ) {
                response = "";
                RAVEPRINT(L"player client not created\n");
            }
            else {
                pthread_mutex_lock(&_mutex_player);
                if( _pposclient != NULL ) {
                    delete _pposclient; _pposclient = NULL;
                    sleep(1); // wait
                }

                int id = atoi(strtok(NULL, delim));
                try {
                    _pposclient = new Position3dProxy(_pclient, 0);
                    _pposclient->GoTo(id, 0, 0, 0, 0, 0);
                }
                catch(...) {
                    _pposclient = NULL;
                    RAVEPRINT(L"failed to create position3d:%d\n", id);
                }
                
                pthread_mutex_unlock(&_mutex_player);
            }
        }
        else if( stricmp(p, "getpos3d") == 0 ) { // query a positionn 3d structure
            
            if( _pclient == NULL ) {
                response = "";
                RAVEPRINT(L"player client not created\n");
            }
            else {
                if( _pposclient == NULL ) {
                    RAVEPRINT(L"pos3d client not init\n");
                    return;
                }
                
                stringstream ss;
                ss << _pposclient->GetStall() << " "
                   << _pposclient->GetXPos() << " " << _pposclient->GetYPos() << " " << _pposclient->GetZPos() << " "
                   << _pposclient->GetRoll() << " " << _pposclient->GetPitch() << " " << _pposclient->GetYaw() << " "
                   << _pposclient->GetXSpeed() << " " << _pposclient->GetYSpeed() << " " << _pposclient->GetZSpeed() << " "
                   << _pposclient->GetRollSpeed() << " " << _pposclient->GetPitchSpeed() << " " << _pposclient->GetYawSpeed();
                response = ss.str();
            }
        }
        else if( stricmp(p, "sendlinkpos") == 0 ) { // send a link of a robot using position3d interface (12 values)
            
            if( _pclient == NULL ) {
                response = "";
                RAVEPRINT(L"player client not created\n");
            }
            else {
                char* p = strtok(NULL, delim);
                if( p == NULL ) {
                    RAVEPRINT(L"sendlinkpos not enough params\n");
                    return;
                }
                int positionid = atoi(p);
                    
                if( _pposclient == NULL ) {
                    pthread_mutex_lock(&_mutex_player);
                    
                    try {
                        _pposclient = new Position3dProxy(_pclient, positionid);
                    }
                    catch(...) {
                        _pposclient = NULL;
                        RAVEPRINT(L"failed to create position3d:0\n");
                    }
                
                    pthread_mutex_unlock(&_mutex_player);

                    if( _pposclient == NULL ) {
                        return;
                    }
                }
                
                p = strtok(NULL, delim);
                if( p != NULL ) {
                    int ilink = atoi(p);
                    if( robot != NULL && ilink >= 0 && ilink < robot->GetLinks().size()) {
                        TransformMatrix t(robot->GetLinks()[ilink]->GetTransform());
                        float data[12] = {t.m[0], t.m[1], t.m[2], t.trans.x, t.m[4], t.m[5], t.m[6], t.trans.y, t.m[8], t.m[9], t.m[10], t.trans.z};
                        _pposclient->GoTo(*(player_pose3d_t*)&data[0], *(player_pose3d_t*)&data[6]);
                    }
                }
                else RAVEPRINT(L"sendlinkpos not enough params\n");
            }
        }
        else if( stricmp(p, "setlaservelocity") == 0 ) {
            
            if( _pclient == NULL )
                RAVEPRINT(L"laser client not loaded\n");
            else {
                // create an actarray driver and send the command
                float fvel = atof(strtok(NULL, delim));
                if( _plasermotorclient != NULL ) {
                    try {
                        _plasermotorclient->MoveAtSpeed(0,fvel);
                    }
                    catch(...) {
                        RAVEPRINT(L"failed to set velocity to %f rad/s on laser: %s\n", fvel, playerc_error_str());
                    }
                }
                else RAVEPRINT(L"laser actarray motor not initialized\n");
            }
        }
        
        p = strtok(NULL, delim);
    }
    
    free(mycmd);
}

void PlayerProblem::Query(const char* query, string& response)
{
}

bool PlayerProblem::SimulationStep(dReal fElapsedTime)
{
    return false;
}

void PlayerProblem::GetRealLaserMap(string& response)
{
    if( _plaserclient == NULL ) {
        RAVEPRINT(L"laser not initialized\n");
        return;
    }

    TransformMatrix trobot = robot->GetLink(L"wam1")->GetTransform();
    stringstream ss;
    
    for(int i = 0; i < 3; ++i ) ss << trobot.m[4*0+i] << " " << trobot.m[4*1+i] << " " << trobot.m[4*2+i] << " ";
    ss << trobot.trans.x << " " << trobot.trans.y << " " << trobot.trans.z << " ";
    ss << _plaserclient->GetRobotPose().pa << " " << _plaserclient->GetScanRes() << " ";
    
    for(int i = _plaserclient->GetCount(); i > 0; --i) {
        float r = (float)_plaserclient->GetRange(i);
        float b = (float)_plaserclient->GetBearing(i);

        ss << r << " " << b << " ";
    }

    response = ss.str();
}

void PlayerProblem::GetRealLaserMap2D(string& response)
{
    if( _plaserclient == NULL ) {
        RAVEPRINT(L"laser not initialized\n");
        return;
    }

    const char* delim = " \r\n\t";
    char* p = strtok(NULL, delim);
    bool bControlVelocity = false;
    float fvel;
    int numscans = 1;
    int show = 0;
    TransformMatrix tlaserleft, tlaserright;
    const char* probotlink = NULL;
    static void* pfigure = NULL;

    while(p != NULL ) {
        if( stricmp(p, "velocity") == 0 ) {
            fvel = (float)atof(strtok(NULL, delim));
            bControlVelocity = true;
        }
        else if( stricmp(p, "numscans") == 0 ) {
            numscans = atoi(strtok(NULL, delim));
        }
        else if( stricmp(p, "show") == 0 ) {
            show = 1;
        }
        else if( stricmp(p, "robotlink") == 0 ) {
            probotlink = strtok(NULL, delim);
        }
        else if( stricmp(p, "laserleft") == 0 ) {
            tlaserleft.m[0] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[4] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[8] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[1] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[5] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[9] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[2] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[6] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.m[10] = (dReal)atof(strtok(NULL, delim));
            tlaserleft.trans.x = (dReal)atof(strtok(NULL, delim));
            tlaserleft.trans.y = (dReal)atof(strtok(NULL, delim));
            tlaserleft.trans.z = (dReal)atof(strtok(NULL, delim));
        }
        else if( stricmp(p, "laserright") == 0 ) {
            tlaserright.m[0] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[4] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[8] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[1] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[5] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[9] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[2] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[6] = (dReal)atof(strtok(NULL, delim));
            tlaserright.m[10] = (dReal)atof(strtok(NULL, delim));
            tlaserright.trans.x = (dReal)atof(strtok(NULL, delim));
            tlaserright.trans.y = (dReal)atof(strtok(NULL, delim));
            tlaserright.trans.z = (dReal)atof(strtok(NULL, delim));
        }
        else
            break;
        p = strtok(NULL, delim);
    }

    if( bControlVelocity ) {
        // create an actarray driver and send the command
        if( _plasermotorclient != NULL ) {
            try {
                _plasermotorclient->MoveAtSpeed(0,fvel);
            }
            catch(...) {
                RAVELOG(L"failed to set velocity\n");
            }
        }
    }

    TransformMatrix trobot;
    if( probotlink != NULL ) {
        const KinBody::Link* plink = robot->GetLink(_ravembstowcs(probotlink).c_str());
        if( plink != NULL )
            trobot = plink->GetTransform();
        else RAVEPRINT(L"failed to find link %s\n", probotlink);
    }

    stringstream ss;
    
    vector<Vector> vpoints;
    if( show ) vpoints.reserve(50000);

    ss <<  _plaserclient->GetScanRes() << " ";
    TransformMatrix tfinal, tlaserpose;

    float prevang = 1000;
    int totalpts = 0;
    for(int n = 0; n < numscans; ++n) {

        u32 laserstart = timeGetTime();
        
        while(fabsf(prevang - _plaserclient->GetRobotPose().pa) < 0.001f ) {
            printf("%f\n", _plaserclient->GetScanRes());
            usleep(10000);

            if( timeGetTime()-laserstart > 3000 ) {
                RAVEPRINT(L"laser isn't moving\n");
                n = numscans;
                break;
            }
        }
        
        prevang = _plaserclient->GetRobotPose().pa;
        tlaserpose.m[5] = tlaserpose.m[10] = cosf(prevang);
        tlaserpose.m[6] = -sinf(prevang);
        tlaserpose.m[9] = -tlaserpose.m[6];
        
        tfinal = trobot * tlaserleft * tlaserpose * tlaserright;
        for(int i = 0; i < 3; ++i ) ss << tfinal.m[4*0+i] << " " << tfinal.m[4*1+i] << " " << tfinal.m[4*2+i] << " ";
        ss << tfinal.trans.x << " " << tfinal.trans.y << " " << tfinal.trans.z << " ";
        ss << prevang << " " << _plaserclient->GetCount() << " ";

        for(int i = _plaserclient->GetCount(); i > 0; --i) {
            float r = (float)_plaserclient->GetRange(i);
            float b = (float)_plaserclient->GetBearing(i);
            
            ss << r << " " << b << " ";

            if( show ) {
                Vector v = tfinal * Vector(r*cosf(b), r*sinf(b), 0, 1);
                if( lengthsqr3(v-trobot.trans) < 1.5f )
                    vpoints.push_back(v);
            }
        }

        totalpts += _plaserclient->GetCount();
    }
    response = ss.str();
    
    if( pfigure != NULL )
        GetEnv()->closegraph(pfigure);
        
    if( show ) {
        pfigure = GetEnv()->plot3((float*)&vpoints[0], vpoints.size(), sizeof(Vector), 0.005f, Vector(0,1,0));
    }
    RAVEPRINT(L"GetRealLaserMap2D: numpoints=%d\n", totalpts);
}

void PlayerProblem::StartPlayerThread()
{
    assert(NULL == _pThread);
    _pThread = new boost::thread(boost::bind(&PlayerProblem::RunPlayerThread, this));
}

void PlayerProblem::RunPlayerThread()
{
    _bStopPlayerThread = false;
    _bDestroyPlayerThread = false;

    RAVEPRINT(L"PlayerProblem: starting thread\n");
    while (!_bStopPlayerThread) {

        if( !_bDestroyPlayerThread ) {
            try {
                //if( _mode == PLAYER_DATAMODE_PUSH) {
//                    if (_pclient->Peek()) {
//                        _pclient->Read();
//                    }
//                }
//                else {
                pthread_mutex_lock(&_mutex_player);
                _pclient->Read();
                pthread_mutex_unlock(&_mutex_player);
                //}
            }
            catch(...) {
                RAVEPRINT(L"PlayerProblem: failed to read from player...\n");
                _bDestroyPlayerThread = true;
            }
        }

//        boost::xtime xt;
//        boost::xtime_get(&xt, boost::TIME_UTC);
//        // we sleep for 1000 us
//        xt.nsec += 1000*1000;
//        boost::thread::sleep(xt);
        usleep(1000);
    }
}

void PlayerProblem::StopPlayerThread()
{
    assert(_pThread);
    _bStopPlayerThread = true;
    _pThread->join();
    delete _pThread; _pThread = NULL;
}
