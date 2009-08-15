// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "ravep.h"

#include <pthread.h>
#include <iostream>
#include <sstream>

#include <boost/shared_ptr.hpp>

#ifdef _WIN32
#define CLOSESOCKET closesocket
typedef int socklen_t;
#else
#include <fcntl.h>
#define CLOSESOCKET close
#endif

static int s_nIdIndex = 1;

static map<int, PlannerBase*> s_mapPlanners;
static map<int, pair<ProblemInstance*, wstring> > s_mapProblems;
static map<int, void*> s_mapFigureIds;
static int s_NextFigureId = 1;

class LockEnvironment
{
public:
    LockEnvironment(EnvironmentBase* penv) : _penv(penv) { _penv->LockPhysics(true); }
    ~LockEnvironment() { _penv->LockPhysics(false); }

private:
    EnvironmentBase* _penv;
};

/// Macros
KinBody* orMacroGetBody(EnvironmentBase* penv, char* in)
{
    int index;
    if( sscanf(in, "%d", &index) != 1 )
        return NULL;

    return penv->GetBodyFromNetworkId(index);
}

RobotBase* orMacroGetRobot(EnvironmentBase* penv, char* in)
{
    int index;
    if( sscanf(in, "%d", &index) != 1 )
        return NULL;

    KinBody* pbody = penv->GetBodyFromNetworkId(index);

    if( pbody == NULL || !pbody->IsRobot() )
        return NULL;

    // manually type cast
    return (RobotBase*)(pbody);
}

RobotBase* orMacroGetRobot(EnvironmentBase* penv, istream& os)
{
    int index;
    os >> index;
    if( !os )
        return NULL;

    KinBody* pbody = penv->GetBodyFromNetworkId(index);

    if( pbody == NULL || !pbody->IsRobot() )
        return NULL;

    // manually type cast
    return (RobotBase*)(pbody);
}

PlannerBase* orMacroGetPlanner(char* in)
{
    int index;
    if( sscanf(in, "%d", &index) != 1 )
        return NULL;

    map<int, PlannerBase*>::iterator itplanner = s_mapPlanners.find(index);
    if( itplanner == s_mapPlanners.end() )
        return NULL;
    
    return itplanner->second;
}

/// orRender - Render the new OpenRAVE scene
bool worRender(char* in, void* pData, RaveServer* pserv)
{
    stringstream ss(in);
    string cmd;
    while(1) {
        ss >> cmd;
        if( !ss )
            break;

        if( stricmp(cmd.c_str(), "start") == 0 ) {
            pserv->GetEnv()->GetViewer()->StartPlaybackTimer();
        }
        else if( stricmp(cmd.c_str(), "stop") == 0 ) {
            pserv->GetEnv()->GetViewer()->StopPlaybackTimer();
        }
        else {
            RAVELOG_WARNA("unknown render command: %s\n", cmd.c_str());
        }

        if( ss.eof() || !ss )
            break;
    }

    return true;
}

/// orMove - move the scene by one timestep
bool worStep(char* in, void* pData, RaveServer* pserv)
{
    return true;
}

/// orEnvSetOptions - Set physics simulation parameters,
bool orEnvSetOptions(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    if( stricmp(in, "quit") == 0 ) {
        pserv->GetEnv()->Reset();
        delete pserv->GetEnv();
        exit(0);
    }
    *ppPassData = pserv;
    return true;
}

bool worSetOptions(char* in, void* pData, RaveServer* pserv)
{
    string strtemp = in;
    const char* pdelim = " \r\n,";
    char* p = strtok(&strtemp[0], pdelim);
    while( p != NULL ) {
        if( stricmp(p, "physics") == 0 ) {
            p = strtok(NULL, pdelim);
            PhysicsEngineBase* pnewengine = NULL;
            if( p != NULL )
                pnewengine = pserv->GetEnv()->CreatePhysicsEngine(_ravembstowcs(p).c_str());
            
            if( pnewengine != NULL ) {
                pserv->GetEnv()->SetPhysicsEngine(pnewengine);
                
                if( pData != NULL ) {
                    delete ((RaveServer*)pData)->_pphysics; // delete the old
                    ((RaveServer*)pData)->_pphysics = pnewengine;
                }

                if( pnewengine != NULL )
                    RAVELOG_DEBUGA("setting physics engine to %s\n", p);
            }
            else {
                pserv->GetEnv()->SetPhysicsEngine(NULL);

                if( pData != NULL ) {
                    delete ((RaveServer*)pData)->_pphysics; // delete the old
                    ((RaveServer*)pData)->_pphysics = NULL;
                }
                
                RAVELOG_DEBUGA("resetting physics engine\n");
            }
        }
        if( stricmp(p, "collision") == 0 ) {
            p = strtok(NULL, pdelim);
            CollisionCheckerBase* pchecker = NULL;
            if( p != NULL )
                pchecker = pserv->GetEnv()->CreateCollisionChecker(_ravembstowcs(p).c_str());
            
            if( pchecker != NULL ) {
                pserv->GetEnv()->SetCollisionChecker(pchecker);
                
                if( pData != NULL ) {
                    delete ((RaveServer*)pData)->_pcolchecker; // delete the old
                    ((RaveServer*)pData)->_pcolchecker = pchecker;
                }

                if( pchecker != NULL )
                    RAVELOG_DEBUGA("setting collision checker to %s\n", p);
            }
            else {
                pserv->GetEnv()->SetCollisionChecker(NULL);
                
                if( pData != NULL ) {
                    delete ((RaveServer*)pData)->_pcolchecker; // delete the old
                    ((RaveServer*)pData)->_pcolchecker = NULL;
                }
                
                RAVELOG_DEBUGA("resetting collision checker\n");
            }
        }
        if( stricmp(p, "simulation") == 0 ) {
            p = strtok(NULL, pdelim);
            if( p != NULL ) {
                if( stricmp(p, "start") == 0 || stricmp(p, "on") == 0 ) {
                    dReal fdeltatime = 0.01f;
                    p = strtok(NULL, pdelim);
                    if( p != NULL )
                        fdeltatime = atof(p);

                    RAVELOG_DEBUGA("starting simulation loop, timestep=%f\n", (float)fdeltatime);
                    LockEnvironment envlock(pserv->GetEnv());
                    pserv->GetEnv()->StartSimulation(fdeltatime);
                }
                else {
                    RAVELOG_DEBUGA("stopping simulation loop\n");
                    LockEnvironment envlock(pserv->GetEnv());
                    pserv->GetEnv()->StopSimulation();
                }
            }
        }
        else if( stricmp(p, "publishanytime") == 0 ) {
            p = strtok(NULL, pdelim);
            if( p != NULL ) {
                if( atoi(p) ) {
                    RAVELOG_DEBUGA("publishing bodies anytime\n");
                    pserv->GetEnv()->SetPublishBodiesAnytime(true);
                }
                else {
                    RAVELOG_DEBUGA("publishing bodies only at end of simulations and send commands\n");
                    pserv->GetEnv()->SetPublishBodiesAnytime(false);
                }
                  
            }
        }
        else if( stricmp(p, "debug") == 0 ) {
            p = strtok(NULL, pdelim);
            if( p != NULL ) {
                pserv->GetEnv()->SetDebugLevel((DebugLevel)atoi(p));
            }
        }
        else if( stricmp(p, "gravity") == 0 ) {
            dReal x = atof(strtok(NULL, pdelim));
            dReal y = atof(strtok(NULL, pdelim));
            dReal z = atof(strtok(NULL, pdelim));
            pserv->GetEnv()->GetPhysicsEngine()->SetGravity(Vector(x,y,z));
            RAVELOG_DEBUGA("set gravity (%f,%f,%f)\n", x, y, z);
        }
        else if( stricmp(p, "quit") == 0 ) {
            //pserv->GetEnv()->Reset();
            //delete pserv->GetEnv();
            exit(0);
        }
        p = strtok(NULL, pdelim);
    }

    return true;
}

/// orEnvSetOptions - Set physics simulation parameters,
bool orEnvLoadScene(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    if( in == NULL || in[0] == 0 ) {
        // if empty, clear the scene
        RAVELOG_DEBUGA("Destroying scene\n");
        pserv->GetEnv()->SetPhysicsEngine(NULL);
        if( pserv != NULL ) {
            delete pserv->_pphysics; // delete the old
            pserv->_pphysics = NULL;
        }
        RAVELOG_DEBUGA("resetting physics engine\n");
        ((Environment*)pserv->GetEnv())->Destroy();
        return true;
    }
    else {
        char* p = strchr(in, ' ');
        if( p != NULL ) {
            if( p[1] == '1' ) {
                RAVELOG_VERBOSEA("Destroying scene\n");
                ((Environment*)pserv->GetEnv())->Destroy();
                s_mapProblems.clear();
                s_mapPlanners.clear();
                RAVELOG_VERBOSEA("Done destroying\n");
                
//                pserv->GetEnv()->SetPhysicsEngine(NULL);
//                if( pserv != NULL ) {
//                    delete pserv->_pphysics; // delete the old
//                    pserv->_pphysics = NULL;
//                }
//                RAVELOG(L"resetting physics engine\n");
            }
            *p = 0;
        }

        RAVELOGA("Loading scene %s\n", in);
        LockEnvironment envlock(pserv->GetEnv());
        if( strlen(in) > 0 )
            return pserv->GetEnv()->Load(in);
    }

    return true;
}

/// robot = orEnvCreateRobot(name, xmlfile) - create a specific robot, return a robot handle (a robot is also a kinbody)
bool orEnvCreateRobot(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char robotname[32], xmlfile[256], robottype[32];
    if( sscanf(in, "%s %s %s", robotname, xmlfile, robottype) != 3 )
        return false;

    wstring wtype = _ravembstowcs(robottype);

    LockEnvironment envlock(pserv->GetEnv());
    RobotBase* robot = pserv->GetEnv()->CreateRobot(wtype.c_str());
    if( robot == NULL ) {
        RAVELOG_ERRORA("Failed to create robot %s", wtype.c_str());
        return false;
    }
    
    if( !robot->Init(xmlfile, NULL) )
        return false;
    
    robot->SetName(robotname);

    if( !pserv->GetEnv()->AddRobot(robot) ) {
        return false;
    }

    if( robot == NULL ) {
        out = "0";
        return true;
    }

    char str[16];
    sprintf(str, "%d", robot->GetNetworkId());
    out = str;

    return true;
}

/// planner = orEnvCreatePlanner(name) - create a planner
bool orEnvCreatePlanner(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char plannertype[32];
    if( sscanf(in, "%s", plannertype) != 1 )
        return false;

    PlannerBase* planner = pserv->GetEnv()->CreatePlanner(_ravembstowcs(in).c_str());
    if( planner == NULL )
        return false;

    s_mapPlanners[s_nIdIndex] = planner;

    char str[16];
    sprintf(str, "%d", s_nIdIndex);
    out = str;
    s_nIdIndex++;
    return true;
}


bool _worEnvDeleteProblem(char* in, void* pData, RaveServer* pserv)
{
    delete (ProblemInstance*)pData;
    return true;
}

/// planner = orEnvCreatePlanner(name) - create a planner
bool orEnvCreateProblem(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char* pproblemname = strchr(in, ' ');
    if( pproblemname == NULL )
        return false;

    *pproblemname++ = 0;
    
    char* pstartargs = strchr(pproblemname+2, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    wstring probname = _ravembstowcs(pproblemname);
    bool bDestroyDuplicates = !!atoi(in);

    pserv->SyncWithWorkerThread();
    
    if( bDestroyDuplicates ) {
        // if there's a duplicate problem instance, delete it
        map<int, pair<ProblemInstance*, wstring> >::iterator itprob = s_mapProblems.begin();
        while(itprob != s_mapProblems.end()) {
            if( wcsicmp(itprob->second.second.c_str(), probname.c_str()) == 0 ) {
                RAVELOG_DEBUGA("deleting duplicate problem %S\n", probname.c_str());
                if( !pserv->GetEnv()->RemoveProblem(itprob->second.first) )
                    RAVELOG_WARNA("environment failed to remove duplicate problem %S\n", probname.c_str());
                
                // delete the problem in the worker thread
                RaveServer::WORKERSTRUCT worker;
                worker.fnWorker = _worEnvDeleteProblem;
                worker.pdata = itprob->second.first;
                pserv->ScheduleWorker(worker);
                
                s_mapProblems.erase(itprob++);
            }
            else ++itprob;
        }
    }
    
    ProblemInstance* prob = pserv->GetEnv()->CreateProblem(probname.c_str());
    if( prob == NULL ) {
        RAVELOG_ERRORA("Cannot find probleminstance: %S\n", probname.c_str());
        return false;
    }
    
    *ppPassData = (void*)(new pair<ProblemInstance*,string>(prob, pstartargs));
    
    s_mapProblems[s_nIdIndex] = pair<ProblemInstance*, wstring>(prob, probname);

    char str[16];
    sprintf(str, "%d", s_nIdIndex);
    out = str;
    s_nIdIndex++;
    return true;
}

bool worEnvCreateProblem(char* in, void *pData, RaveServer* pserv)
{
    pair<ProblemInstance*,string>* p = (pair<ProblemInstance*,string>*)pData;
    if( pserv->GetEnv()->LoadProblem(p->first, p->second.c_str()) != 0 ) {
        delete p->first;
    }
    delete p;
    return true;
}

bool worEnvDestroyProblem(char* in, void *pData, RaveServer* pserv)
{
    int index = atoi(in);
    map<int, pair<ProblemInstance*, wstring> >::iterator it = s_mapProblems.find(index);

    if( it != s_mapProblems.end() ) {
        if( !pserv->GetEnv()->RemoveProblem(it->second.first) )
            RAVELOG_WARNA("orEnvDestroyProblem: failed to remove problem from environment\n");
        delete it->second.first;
        s_mapProblems.erase(it);
    }
    else RAVELOG_WARNA("orEnvDestroyProblem: cannot find problem with id %d\n", index);

    return true;
}

/// body = orEnvCreateKinBody(name, xmlfile) - create a specific kinbody
bool orEnvCreateKinBody(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char bodyname[32], xmlfile[256];
    if( sscanf(in, "%s %s", bodyname, xmlfile) != 2 )
        return false;

    LockEnvironment envlock(pserv->GetEnv());
    KinBody* body = pserv->GetEnv()->CreateKinBody();

    if( !body->Init(xmlfile, NULL) )
        return false;
    body->SetName(bodyname);

    if( !pserv->GetEnv()->AddKinBody(body) )
        return false;

    char str[16];
    sprintf(str, "%d", body->GetNetworkId());
    out = str;
    return true;
}

// bodyid = orEnvGetBody(bodyname)
// Returns the id of the body given its name
bool orEnvGetBody(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    // thread safe?
    KinBody* pbody = pserv->GetEnv()->GetKinBody(_ravembstowcs(in).c_str());
    if( pbody == NULL ) {
        out = "0";
    }
    else {
        char str[64];
        sprintf(str, "%d", pbody->GetNetworkId());
        out = str;
    }

    return true;
}

bool orEnvGetRobots(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    // thread safe?
    vector<RobotBase*> currobots = pserv->GetEnv()->GetRobots();

    char str[32];
    sprintf(str, "%"PRIdS"\n", currobots.size());
    out = str;

    FOREACHC(it, currobots) { //TODO:Get robot type!                                     
        sprintf(str, "%d %S %s ", (*it)->GetNetworkId(), (*it)->GetName(), (*it)->GetXMLId());
        out += str;
        out += (*it)->GetXMLFilename();
        out += "\n"; out += " ";
    }

    return true;
}

bool orEnvGetBodies(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    // thread safe?
    const vector<KinBody*>& vbodies = pserv->GetEnv()->GetBodies();

    char str[128];
    sprintf(str, "%"PRIdS"\n", vbodies.size());
    out = str;

    FOREACHC(it, vbodies) { //TODO:Get robot type!
        sprintf(str, "%d %S %s ", (*it)->GetNetworkId(), (*it)->GetName(), (*it)->GetXMLId());
        out += str;
        out += (*it)->GetXMLFilename();
        out += "\n"; out += " ";
    }

    return true;
}

/// values = orBodySetTransform(body, position, rotation) - returns the dof values of a kinbody
bool worKinBodySetTransform(char* in, void *pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    KinBody* pbody = orMacroGetBody(pserv->GetEnv(), in);
    if( pbody == NULL )
        return false;

    Transform t;
    std::stringstream s(pstartargs);
    vector<float> values;
    while(!s.eof()) {
        float f;
        s >> f;
        if( s.fail() )
            break;
        values.push_back(f);
    }

    if( values.size() == 7 ) {
        // quaternion and translation
        t.rot.x = values[0];
        t.rot.y = values[1];
        t.rot.z = values[2];
        t.rot.w = values[3];
        t.trans.x = values[4];
        t.trans.y = values[5];
        t.trans.z = values[6];
    }
    else if( values.size() == 12 ) {
        // rotation mat and translation
        TransformMatrix tm;
        tm.m[0] = values[0];    tm.m[1] = values[3];        tm.m[2] = values[6];
        tm.m[4] = values[1];    tm.m[5] = values[4];        tm.m[6] = values[7];
        tm.m[8] = values[2];    tm.m[9] = values[5];        tm.m[10] = values[8];
        tm.trans.x = values[9];
        tm.trans.y = values[10];
        tm.trans.z = values[11];
        t = Transform(tm);
    }
    else if( values.size() == 3 ) {
        // just translation, rotation is identity
        t.trans.x = values[0];
        t.trans.y = values[1];
        t.trans.z = values[2];
    }
    else return false;

    // normalize the rotation first
    normalize4(t.rot, t.rot);

    {
        LockEnvironment envlock(pserv->GetEnv());
        pbody->SetTransform(t);
    }

    // RTTI doesn't work on linux
    if( pbody->IsRobot() ) {
        RobotBase* probot = (RobotBase*)(pbody);

        if( probot->GetController() != NULL )
            // if robot, have to turn off any trajectory following
            probot->GetController()->SetPath(NULL);

        //vector<dReal> joints;
//        probot->GetJointValues(joints);
//        if( probot->GetController() != NULL && joints.size() > 0 )
//            probot->GetController()->SetDesired(&joints[0]);
    }

    return true;
}

/// orRobotSetActiveDOFs(robot, indices, affinedofs, axis) - returns the dof values of a kinbody
bool worBodyDestroy(char* in, void* pData, RaveServer* pserv)
{
    KinBody* pbody = orMacroGetBody(pserv->GetEnv(),in);
    if( pbody == NULL )
        return false;

    LockEnvironment envlock(pserv->GetEnv());
    return pserv->GetEnv()->RemoveKinBody(pbody, true);
}

bool worBodyEnable(char* in, void* pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    KinBody* pbody = orMacroGetBody(pserv->GetEnv(),in);
    if( pbody == NULL )
        return false;

    int enable=1;
    if( sscanf(pstartargs, "%d", &enable) == 1 ) {
        if( enable ) RAVELOGA("Enabling body %S\n", pbody->GetName());
        else RAVELOGA("Disabling body %S\n", pbody->GetName());
        pbody->Enable(enable);
    }

    return true;
}

/// values = orBodyGetLinks(body) - returns the dof values of a kinbody
bool orBodyGetLinks(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    vector<Transform> trans;
    {
        LockEnvironment envlock(pserv->GetEnv());
        body->GetBodyTransformations(trans); // thread safe?
    }

    stringstream ss;
    FOREACHC(it, trans)
        ss << TransformMatrix(*it) << " ";

    out = ss.str();
    return true;
}

bool orRobotControllerSend(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL ) {
        out = "0";
        return false;
    }

    *pstartargs++ = 0;

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL || robot->GetController() == NULL ) {
        out = "0";
        return false;
    }

    // the next word should be the command
    if( robot->GetController()->SupportsCmd(pstartargs) ) {
        out = "1";
        return true;
    }
    else {
        RAVELOG_WARNA("Robot %S controller doesn't support command: \"%s\"\n", robot->GetName(), pstartargs);
    }

    out = "0";
    return false;
}

bool worRobotControllerSend(char* in, void* pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;
    
    *pstartargs++ = 0;
    
    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL || robot->GetController() == NULL )
        return false;
    
    return robot->GetController()->SendCmd(pstartargs);
}

bool _worRobotSensorSend(char* in, void* pData, RaveServer* pserv)
{
    istream* is = (istream*)((void**)pData)[0];
    ostream* os = (ostream*)((void**)pData)[1];
    SensorBase* psensor = (SensorBase*)((void**)pData)[2];
    
    if( !psensor->SendCmd(*is, *os) ) {
        RAVELOG_ERRORA("sensor failed to send cmd\n");
        return false;
    }

    return true;
}

bool orRobotSensorSend(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    stringstream ss(in);

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),ss);
    if( robot == NULL ) {
        out = "0";
        return false;
    }

    int sensorindex = 0;
    string cmd;
    ss >> sensorindex;
    stringstream::streampos pos = ss.tellg();
    ss >> cmd;

    if( !ss )
        return false;
    
    if( sensorindex < 0 || sensorindex >= (int)robot->GetSensors().size() )
        return false;

    if( !robot->GetSensors()[sensorindex].GetSensor()->SupportsCmd(cmd.c_str()) ) {
        RAVELOG_WARNA("Robot %S sensor %d doesn't support command: \"%s\"\n", robot->GetName(), sensorindex, cmd.c_str());
        return false;
    }

    ss.clear();
    ss.seekg(pos);

    stringstream sout;
    void* parray[] = {&ss, &sout, robot->GetSensors()[sensorindex].GetSensor()};

    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = _worRobotSensorSend;
    worker.args = in;
    worker.pdata = parray;
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();

    out = sout.str();
    return true;
};

bool orRobotSensorData(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    stringstream ss(in);

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),ss);
    if( robot == NULL ) {
        out = "0";
        return false;
    }

    int sensorindex = 0, options = 0;
    ss >> sensorindex >> options;

    if( !ss )
        return false;
    
    if( sensorindex < 0 || sensorindex >= (int)robot->GetSensors().size() )
        return false;

    SensorBase* psensor = robot->GetSensors()[sensorindex].GetSensor();
    boost::shared_ptr<SensorBase::SensorData> pdata(psensor->CreateSensorData());

    if( !pdata ) {
        RAVELOG_ERRORA("Robot %S, failed to create sensor %s data\n", robot->GetName(), robot->GetSensors()[sensorindex].GetName());
        return false;
    }

    if( !psensor->GetSensorData(pdata.get()) ) {
        RAVELOG_ERRORA("Robot %S, failed to get sensor %s data\n", robot->GetName(), robot->GetSensors()[sensorindex].GetName());
        return false;
    }

    // serialize the data
    stringstream sout;
    sout << pdata->GetType() << " ";

    switch(pdata->GetType()) {
    case SensorBase::ST_Laser: {
        SensorBase::LaserSensorData* plaserdata = (SensorBase::LaserSensorData*)pdata.get();
        sout << plaserdata->ranges.size() << " ";
        if( plaserdata->positions.size() != plaserdata->ranges.size() )
            sout << "1 ";
        else
            sout << plaserdata->positions.size() << " ";
        
        if( options & 1 )
            sout << plaserdata->intensity.size() << " ";
        else
            sout << "0 "; // don't send any intensity data

        FOREACH(it, plaserdata->ranges)
            sout << it->x << " " << it->y << " " << it->z << " ";
        if( plaserdata->positions.size() != plaserdata->ranges.size() )
            sout << plaserdata->t.trans.x << " " << plaserdata->t.trans.y << " " << plaserdata->t.trans.z << " ";
        
        if( options & 1 ) {
            FOREACH(it, plaserdata->intensity)
                sout << *it << " ";
        }

        break;
    }
    case SensorBase::ST_Camera: {
        SensorBase::CameraSensorData* pcameradata = (SensorBase::CameraSensorData*)pdata.get();

        if( psensor->GetSensorGeometry()->GetType() != SensorBase::ST_Camera ) {
            RAVELOG_ERRORA("sensor geometry not a camera type\n");
            return false;
        }

        SensorBase::CameraGeomData* pgeom = (SensorBase::CameraGeomData*)psensor->GetSensorGeometry();

        if( (int)pcameradata->vimagedata.size() != pgeom->width*pgeom->height*3 ) {
            RAVELOG_ERRORA("image data wrong size %"PRIdS" != %"PRIdS"\n", pcameradata->vimagedata.size(), pgeom->width*pgeom->height*3);
            return false;
        }

        sout << pgeom->width << " " << pgeom->height << " ";
        for(int i = 0; i < 4; ++i)
            sout << pgeom->KK[i] << " ";
        sout << TransformMatrix(pcameradata->t) << " ";
        //FOREACH(it, pcameradata->vimagedata) sout << (int)*it << " ";

        // RLE encoding (about 3x faster than sending raw images)
        int curvalue = 0, lastdiff = 0, lastvalue = 0xffffff&*(int*)&pcameradata->vimagedata[0];
        list<int> difs, values;
        for(int i = 1; i < (int)pcameradata->vimagedata.size()/3; ++i) {
            curvalue = 0xffffff&*(int*)&pcameradata->vimagedata[3*i];
            if( curvalue != lastvalue ) {
                values.push_back(lastvalue);
                difs.push_back(i-lastdiff);
                lastdiff = i;
                lastvalue = curvalue;
            }
        }
        difs.push_back(pcameradata->vimagedata.size()/3-lastdiff);
        values.push_back(curvalue);

        sout << values.size() << " ";
        FOREACH(it, values)
            sout << *it << " ";
        sout << difs.size() << " ";
        FOREACH(it, difs)
            sout << *it << " ";
        break;
    }
    case SensorBase::ST_JointEncoder:
    case SensorBase::ST_Force6D:
    default:
        RAVELOG_WARNA("sensor type %d not supported\n", pdata->GetType());
        break;
    }
    
    out = sout.str();
    return true;
}

bool _worRobotControllerSet(char*, void* pData, RaveServer* pserv)
{
    char* in = *(char**)pData;
    string* pout = (string*)((void**)pData)[1];
    
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;
    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL ) {
        RAVELOG_WARNA("failed to find robot %S\n", in);
        return false;
    }

    char* pcontrollername = pstartargs;

    pstartargs = strchr(pstartargs+2, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    if( robot->SetController(_ravembstowcs(pcontrollername).c_str(), pstartargs, true) ) {
        *pout = "1";
        return true;
    }
    else {
        *pout = "0";
        return false;
    }
    
    return true;
}

bool orRobotControllerSet(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    void* mydata[2] = { in, &out };

    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = _worRobotControllerSet;
    worker.pdata = mydata;
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();
    return true;
}

bool worEnvClose(char* in, void* pData, RaveServer* pserv)
{
    if( in == NULL || in[0] == 0 ) {
        // delete all!
        FOREACH(it, s_mapFigureIds) 
            pserv->GetEnv()->closegraph(it->second);
        s_mapFigureIds.clear();
    }
    else {
        stringstream ss(in);

        while(1) {
            int id; ss >> id;
            if( !ss )
                break;
          
            map<int,void*>::iterator it = s_mapFigureIds.find(id);
            if( it != s_mapFigureIds.end() ) {
                pserv->GetEnv()->closegraph(it->second);
                s_mapFigureIds.erase(it);
            }
        }
    }
    return true;
}

bool worEnvPlot(char* in, void* pData, RaveServer* pserv)
{
    int id;// = *(int*)&pData; // do this only with -fno-strict-aliasing
    memcpy(&id, &pData, sizeof(int));

    std::stringstream s(in);
    vector<RaveVector<float> > vpoints;
    vector<float> vcolors;

    int numpoints=0, numcolors=0;
    float fsize=0;
    int drawstyle = 0;
    float ftransparency=0;

    s >> numpoints;
    vpoints.reserve(numpoints);

    for(int i = 0; i < numpoints; ++i) {
        Vector v;
        s >> v.x >> v.y >> v.z;
        vpoints.push_back(v);
    }

    s >> numcolors;
    vcolors.resize(3*numcolors);

    for(int i = 0; i < numcolors*3; ++i)
        s >> vcolors[i];
    
    if( vcolors.size() == 0 ) {
        vcolors.push_back(1);
        vcolors.push_back(0.5f);
        vcolors.push_back(0.5f);
    }

    s >> fsize >> drawstyle >> ftransparency;
    float falpha = 1-ftransparency;

    if( !s ) {
        RAVELOG_ERRORA("error occured in orEnvPlot stream\n");
        return false;
    }

    if( numcolors > 1 && numcolors != numpoints) {
        RAVELOG_WARNA("number of colors (%"PRIdS") != number of points (%d)\n", vcolors.size(), numpoints);
        numcolors = 1;
    }

    void* figure = NULL;

    switch(drawstyle) {
    case 0: // regular points
        if( numcolors != numpoints ) 
            figure = pserv->GetEnv()->plot3(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,
                                     RaveVector<float>(vcolors[0], vcolors[1], vcolors[2], falpha),0);
        else
            figure = pserv->GetEnv()->plot3(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,&vcolors[0],0);
        break;
    case 1: // line strip
        if( numcolors != numpoints ) 
            figure = pserv->GetEnv()->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,
                                             RaveVector<float>(vcolors[0], vcolors[1], vcolors[2], falpha));
        else
            figure = pserv->GetEnv()->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,&vcolors[0]);
        break;
    case 2: // list lists
        if( numcolors != numpoints ) 
            figure = pserv->GetEnv()->drawlinelist(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,
                                            RaveVector<float>(vcolors[0], vcolors[1], vcolors[2], falpha));
        else
            figure = pserv->GetEnv()->drawlinelist(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,&vcolors[0]);
        break;
    case 3: // spheres
        if( numcolors != numpoints ) 
            figure = pserv->GetEnv()->plot3(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,
                                     RaveVector<float>(vcolors[0], vcolors[1], vcolors[2], falpha),1);
        else
            figure = pserv->GetEnv()->plot3(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),fsize,&vcolors[0],1);
        break;
    case 4: // triangle list
        //if( numcolors != numpoints ) 
        figure = pserv->GetEnv()->drawtrimesh(&vpoints[0].x,sizeof(vpoints[0]),NULL, vpoints.size()/3,
                                       RaveVector<float>(vcolors[0], vcolors[1], vcolors[2], falpha));
        //else
        //figure = pserv->GetEnv()->drawtrimesh(&vpoints[0].x,sizeof(vpoints[0]),vpoints.size(),&vcolors[0]);
        break;  
    }

    if( figure != NULL )
        s_mapFigureIds[id] = figure;
    else
        id = 0;
    
    return true;
}

bool orEnvPlot(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    int id = s_NextFigureId++;
    *(int*)ppPassData = id;

    char str[32];
    sprintf(str, "%d", id);
    out = str;
    return true;
}

/// orRobotSetActiveDOFs(robot, indices, affinedofs, axis) - returns the dof values of a kinbody
bool worRobotSetActiveDOFs(char* in, void* pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;

    int numindices;
    std::stringstream s(pstartargs);
    s >> numindices;
    assert( numindices >= 0 );
    vector<int> vindices;
    for(int i = 0; i < numindices; ++i) {
        int tempindex;
        s >> tempindex;
        if( tempindex < 0 || tempindex >= robot->GetDOF() ) {
            RAVELOG_WARNA("bad degree of freedom\n");
            return false;
        }
        vindices.push_back(tempindex);
    }

    int affinedofs;
    s >> affinedofs;
    Vector axis;

    if( affinedofs & RobotBase::DOF_RotationAxis ) {
        s >> axis.x;
        s >> axis.y;
        s >> axis.z;
    }

    robot->SetActiveDOFs(vindices, affinedofs, &axis);
    return true;
}

/// dofs = orRobotGetActiveDOF(body) - returns the active degrees of freedom of the robot
bool orRobotGetActiveDOF(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();
    
    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;

    int dof = robot->GetActiveDOF();
    char str[32];
    sprintf(str, "%d", dof);
    out = str;
    return true;
}

/// dofs = orBodyGetAABB(body) - returns the number of active joints of the body 
bool orBodyGetAABB(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();
    
    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    AABB ab = body->ComputeAABB();
    stringstream ss;
    ss << ab.pos.x << " " << ab.pos.y << " " << ab.pos.z << " "
       << ab.extents.x << " " << ab.extents.y << " " << ab.extents.z;
    out = ss.str();
    return true;
}

/// values = orBodyGetLinks(body) - returns the dof values of a kinbody
bool orBodyGetAABBs(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    stringstream ss;
    FOREACHC(itlink, body->GetLinks()) {
        AABB ab = (*itlink)->ComputeAABB();
        ss << ab.pos.x << " " << ab.pos.y << " " << ab.pos.z << " "
           << ab.extents.x << " " << ab.extents.y << " " << ab.extents.z << " ";
    }

    out = ss.str();
    return true;
}

/// dofs = orBodyGetDOF(body) - returns the number of active joints of the body 
bool orBodyGetDOF(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();
    
    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    int dof = body->GetDOF();
    char str[32];
    sprintf(str, "%d", dof);
    out = str;
    return true;
}

///// values = orBodyGetTransform(body) - returns the dof values of a kinbody
//bool orBodyGetTransform(char* in, string& out, void **ppPassData, RaveServer* pserv)
//{
//    pserv->SyncWithWorkerThread();
//
//    KinBody* pbody = orMacroGetBody(pserv->GetEnv(),in);
//    if( robot == NULL )
//        return false;
//
//    TransformMatrix t = TransformMatrix(GetBodyTransform(pbody->GetLinks().front()->GetBody()));
//
//    char str[255];
//    sprintf(str, "%f %f %f ", (float)t.x, (float)t.y, (float)t.z);
//    out = str;
//
//    for(int i = 0; i < 3; ++i) {
//        sprintf(str, "%f %f %f ", (float)t.m[4*i+0], (float)t.m[4*i+1], (float)t.m[4*i+2]);
//        out += str;
//    }
//    
//    return true;
//}

/// values = orBodyGetDOFValues(body, indices) - returns the dof values of a kinbody
bool orBodyGetJointValues(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs != NULL )
        *pstartargs++ = 0;

    pserv->SyncWithWorkerThread();

    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    vector<dReal> values;

    if( pstartargs != NULL ) {
        TemplateStreamParser<int, char> parser;
        parser.Format(pstartargs, strlen(pstartargs));
        const int* pindices = (const int*)parser.GetData();
        
        if( pindices != NULL && parser.GetCount() > 0 ) {
            // make sure all indices are good
            vector<dReal> vAllValues;
            body->GetJointValues(vAllValues);
            values.resize(parser.GetCount());
            
            for(int i = 0; i < parser.GetCount(); ++i) {
                if( pindices[i] < 0 || pindices[i] >= body->GetDOF() ) {
                    RAVELOG_ERRORA("orBodyGetJointValues bad index\n");
                    return false;
                }
                
                values[i] = vAllValues[pindices[i]];
            }
        }
    }

    if( values.size() == 0 ) {
        body->GetJointValues(values); // thread safe?
    }

    stringstream ss;
    FOREACH(it,values)
        ss << *it << " ";
    out = ss.str();

    return true;
}

/// values = orRobotGetDOFValues(body, indices) - returns the dof values of a kinbody
bool orRobotGetDOFValues(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs != NULL )
        *pstartargs++ = 0;

    pserv->SyncWithWorkerThread();

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;

    vector<dReal> values;

    if( pstartargs != NULL ) {
        TemplateStreamParser<int, char> parser;
        parser.Format(pstartargs, strlen(pstartargs));
        const int* pindices = (const int*)parser.GetData();
        
        if( pindices != NULL && parser.GetCount() > 0 ) {
            // make sure all indices are good
            vector<dReal> vAllValues;
            robot->GetJointValues(vAllValues);
            values.resize(parser.GetCount());
            
            for(int i = 0; i < parser.GetCount(); ++i) {
                if( pindices[i] < 0 || pindices[i] >= robot->GetDOF() ) {
                    RAVELOG_ERRORA("orRobotGetDOFValues bad index\n");
                    return false;
                }
                
                values[i] = vAllValues[pindices[i]];
            }
        }
    }

    if( values.size() == 0 )
        robot->GetActiveDOFValues(values); // thread safe?

    stringstream ss;
    FOREACH(it, values)
        ss << *it << " ";
    out = ss.str();
    return true;
}

/// [lower, upper] = orKinBodyGetDOFLimits(body) - returns the dof limits of a kinbody
bool orRobotGetDOFLimits(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;

    vector<dReal> lower(robot->GetActiveDOF()), upper(robot->GetActiveDOF());
    robot->GetActiveDOFLimits(&lower[0], &upper[0]); // thread safe?

    stringstream ss;
    ss << lower.size() << " ";
    FOREACH(it, lower)
        ss << *it << " ";
    FOREACH(it, upper)
        ss << *it << " ";

    out = ss.str();
    return true;
}

bool orRobotGetManipulators(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;
    
    stringstream ss;
    ss << robot->GetManipulators().size() << " ";
    FOREACHC(itmanip, robot->GetManipulators()) {
        if( itmanip->pBase == NULL )
            ss << "-1 ";
        else
            ss << itmanip->pBase->GetIndex() << " ";
        if( itmanip->pEndEffector == NULL )
            ss << "-1 ";
        else
            ss << itmanip->pEndEffector->GetIndex() << " ";
        ss << TransformMatrix(itmanip->tGrasp) << " ";
        ss << itmanip->_vecjoints.size() << " ";
        FOREACHC(it, itmanip->_vecjoints)
            ss << *it << " ";
        ss << itmanip->_vecarmjoints.size() << " ";
        FOREACHC(it, itmanip->_vecarmjoints)
            ss << *it << " ";
//        ss << itmanip->_vClosedGrasp.size() << " ";
//        FOREACH(it, itmanip->_vClosedGrasp)
//            ss << *it << " ";
//        ss << itmanip->_vOpenGrasp.size() << " ";
//        FOREACH(it, itmanip->_vOpenGrasp)
//            ss << *it << " ";
        if( itmanip->GetIKSolverName().size() == 0 )
            ss << "0 ";
        else {
            ss << itmanip->GetIKSolverName().size() << " " << itmanip->GetIKSolverName() << " ";
        }
    }

    out = ss.str();
    return true;
}

bool orRobotGetAttachedSensors(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;

    stringstream ss;
    ss << robot->GetSensors().size() << " ";
    FOREACHC(itsensor, robot->GetSensors()) {
        if( itsensor->GetName() == NULL || itsensor->GetName()[0] == 0 )
            ss << "0 ";
        else
            ss << strlen(itsensor->GetName()) << " " << itsensor->GetName() << " ";
        
        if( itsensor->GetAttachingLink() == NULL )
            ss << "-1 ";
        else
            ss << itsensor->GetAttachingLink()->GetIndex() << " ";

        ss << TransformMatrix(itsensor->GetRelativeTransform()) << " ";

        if( itsensor->GetSensor() == NULL )
            ss << "0 " << TransformMatrix() << " ";
        else
            ss << strlen(itsensor->GetSensor()->GetXMLId()) << " "
               << itsensor->GetSensor()->GetXMLId() << " " 
               << TransformMatrix(itsensor->GetSensor()->GetTransform()) << " ";
    }

    out = ss.str();
    return true;
}

/// orBodySetJointValues(body, values, indices)
bool worBodySetJointValues(char* in, void *pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    KinBody* body = orMacroGetBody(pserv->GetEnv(),in);
    if( body == NULL )
        return false;

    const char* delim = " \n\r\t";
    char* token = strtok(pstartargs, " \n\r\t");
    if( token == NULL )
        return false;
    int dof = atoi(token);
    if( dof <= 0 )
        return false;

    vector<dReal> vvalues(dof);
    vector<int> vindices;

    for(int i = 0; i < dof; ++i) {
        token = strtok(NULL, delim);
        if( token == NULL )
            return false;
        vvalues[i] = atof(token);
    }

    // check if there are indices
    token = strtok(NULL, delim);
    if( token != NULL ) {

        vector<dReal> v;
        vindices.resize(dof);

        for(int i = 0; i < dof; ++i) {
            if( token == NULL ) {
                RAVELOG_ERRORA("not enough indices (%d) \n", dof);
                return false;
            }
            vindices[i] = atoi(token);
            if( vindices[i] < 0 || vindices[i] >= body->GetDOF() ) {
                RAVELOG_ERRORA("bad index: %d\n", vindices[i]);
                return false;
            }
            token = strtok(NULL, delim);
        }

        body->GetJointValues(v);
        for(int i = 0; i < dof; ++i)
            v[vindices[i]] = vvalues[i];
        vvalues = v;
    }
    else {
        // use all the dofs
        if( (int)vvalues.size() != body->GetDOF() )
            return false;
    }

    body->SetJointValues(NULL, NULL, &vvalues[0], true);

    // RTTI doesn't work on linux
    if( body->IsRobot() ) {
        // if robot, have to turn off any trajectory following
        RobotBase* probot = (RobotBase*)(body);
        if( probot->GetController() != NULL ) {
            // reget the values since they'll go through the joint limits
            probot->GetJointValues(vvalues);
            probot->GetController()->SetDesired(vvalues.size() > 0 ? &vvalues[0] : NULL);
        }
    }

    return true;
}

/// orRobotSetDOFValues(body, values, indices)
bool worRobotSetDOFValues(char* in, void *pData, RaveServer* pserv)
{
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
    if( robot == NULL )
        return false;
    
    const char* delim = " \n\r\t";
    char* token = strtok(pstartargs, " \n\r\t");
    if( token == NULL )
        return false;
    int dof = atoi(token);
    if( dof <= 0 )
        return false;

    vector<dReal> vvalues(dof);
    vector<int> vindices;

    for(int i = 0; i < dof; ++i) {
        token = strtok(NULL, delim);
        if( token == NULL )
            return false;
        vvalues[i] = atof(token);
    }

    vector<dReal> v;

    // check if there are indices
    token = strtok(NULL, delim);
    if( token != NULL ) {

        vindices.resize(dof);

        for(int i = 0; i < dof; ++i) {
            if( token == NULL ) {
                RAVELOG_ERRORA("not enough indices (%d) \n", dof);
                return false;
            }
            vindices[i] = atoi(token);
            if( vindices[i] < 0 || vindices[i] >= robot->GetDOF() ) {
                RAVELOG_ERRORA("bad index: %d\n", vindices[i]);
                return false;
            }
            token = strtok(NULL, delim);
        }

        robot->GetJointValues(v);
        if( v.size() == 0 )
            return true;
        for(int i = 0; i < dof; ++i)
            v[vindices[i]] = vvalues[i];

        robot->SetJointValues(NULL, NULL, &v[0], true);
    }
    else {
        // use active dofs
        if( (int)vvalues.size() != robot->GetActiveDOF() )
            return false;
        if( robot->GetActiveDOF() == 0 )
            return true;

        robot->SetActiveDOFValues(NULL, &vvalues[0], true);
    }

    if( robot->GetController() != NULL ) {
        robot->GetJointValues(v);
        robot->GetController()->SetDesired(&v[0]);
    }
    
    return true;
}

/// orRobotStartActiveTrajectory(robot, jointvalues, timestamps, transformations)
/// - starts a trajectory on the robot with the active degrees of freedom
bool worRobotStartActiveTrajectory(char* in, void *pData, RaveServer* pserv)
{
    stringstream ss(in);
    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),ss);
    if( robot == NULL )
        return false;

    int numpoints, havetime, havetrans;
    ss >> numpoints >> havetime >> havetrans;
    if( !ss )
        return false;
    
    Transform trans = robot->GetTransform();

    vector<Trajectory::TPOINT> vpoints(numpoints);
    FOREACH(it, vpoints) {
        it->q.resize(robot->GetActiveDOF());
        it->trans = trans;
        FOREACH(itval, it->q)
            ss >> *itval;
        if( !ss )
            return false;
    }
    if( havetime ) {
        FOREACH(it, vpoints)
            ss >> it->time;
    }

    if( havetrans ) {

        if( havetrans == 1 ) { // 3x4 matrix
            TransformMatrix m;
            FOREACH(it, vpoints) {
                ss >> m;
                it->trans = m;
            }
        }
        else { // quaternion and translation
            FOREACH(it, vpoints)
                ss >> it->trans;
        }
    }

    // add all the points
    Trajectory* pfulltraj = pserv->GetEnv()->CreateTrajectory(robot->GetDOF());

    if( robot->GetActiveDOF() > 0 ) {
        Trajectory* ptraj = pserv->GetEnv()->CreateTrajectory(robot->GetActiveDOF());
        FOREACH(it, vpoints)
            ptraj->AddPoint(*it);
        robot->GetFullTrajectoryFromActive(pfulltraj, ptraj, false);
        delete ptraj;
    }
    else {
        Trajectory::TPOINT tp;
        robot->GetJointValues(tp.q);
        FOREACH(it, vpoints) {
            tp.time = it->time;
            tp.trans = it->trans;
            pfulltraj->AddPoint(tp);
        }
    }

    if( !ss ) {
        delete pfulltraj;
        return false;
    }

    pfulltraj->CalcTrajTiming(robot, pfulltraj->GetInterpMethod(), !havetime, false);

    robot->SetMotion(pfulltraj);
    delete pfulltraj;

    return true;
}

/// [collision, bodycolliding] = orEnvCheckCollision(body) - returns whether a certain body is colliding with the scene
bool orEnvCheckCollision(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();
        
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    *pstartargs++ = 0;

    int index;
    if( sscanf(in, "%d", &index) != 1 )
        return false;

    KinBody* pbody = pserv->GetEnv()->GetBodyFromNetworkId(index);
    if( pbody == NULL )
        return false;

    set<KinBody*> vignore;
    if( sscanf(pstartargs, "%d", &index) == 1 ) {
        KinBody* pignore = pserv->GetEnv()->GetBodyFromNetworkId(index);
        if( pignore != NULL )
            vignore.insert(pignore);
    }

    COLLISIONREPORT report;
    LockEnvironment envlock(pserv->GetEnv());

    set<KinBody::Link*> empty;
    if( pserv->GetEnv()->CheckCollision(pbody, vignore, empty, &report) ) {
        out = "1";
        RAVELOG_VERBOSEA("collision %S:%S with %S:%S\n",
                         report.plink1?report.plink1->GetParent()->GetName():L"(NULL",
                         report.plink1?report.plink1->GetName():L"(NULL)",
                         report.plink2?report.plink2->GetParent()->GetName():L"(NULL)",
                         report.plink2?report.plink2->GetName():L"(NULL)");
    }
    else out = "0";

    int bodyindex = 0;
    if( report.plink1 != NULL && report.plink1->GetParent() != pbody )
        bodyindex = report.plink1->GetParent()->GetNetworkId();
    if( report.plink2 != NULL && report.plink2->GetParent() != pbody )
        bodyindex = report.plink2->GetParent()->GetNetworkId();

    char str[25];
    sprintf(str, " %d", bodyindex);
    out += str;

    return true;
}

/// [collision, info] = orEnvRayCollision(rays) - returns the position and normals where all the rays collide
/// every ray is 6 dims
/// collision is a N dim vector that is 0 for non colliding rays and 1 for colliding rays
/// info is a Nx6 vector where the first 3 columns are position and last 3 are normals
bool orEnvRayCollision(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    vector<float> info;

    pserv->SyncWithWorkerThread();

    LockEnvironment envlock(pserv->GetEnv());
    
    int oldoptions = pserv->GetEnv()->GetCollisionOptions();
    pserv->GetEnv()->SetCollisionOptions(CO_Contacts);

    COLLISIONREPORT report;
    RAY r;
    KinBody* pbody = NULL;
    std::stringstream s(in);
    int bodyid;
    bool bcollision;
    s >> bodyid;
    if(bodyid != -1)
        pbody = pserv->GetEnv()->GetBodyFromNetworkId(bodyid);

    stringstream sout;

    while(!s.eof()) {
        s >> r.pos.x >> r.pos.y >> r.pos.z >> r.dir.x >> r.dir.y >> r.dir.z;
        if( s.fail() )
            break;

        if(pbody != NULL)
            bcollision = pserv->GetEnv()->CheckCollision(r, pbody, &report);
        else
            bcollision = pserv->GetEnv()->CheckCollision(r, &report);
        
        if(bcollision) {
            assert( report.contacts.size() > 0 );
            COLLISIONREPORT::CONTACT& c = report.contacts.front();
            sout << "1 ";
            info.push_back(c.pos.x); info.push_back(c.pos.y); info.push_back(c.pos.z);
            info.push_back(c.norm.x); info.push_back(c.norm.y); info.push_back(c.norm.z);
        }
        else {
            sout << "0 ";
            for(int i = 0; i < 6; ++i)
                info.push_back(0);
        }
    }

    pserv->GetEnv()->SetCollisionOptions(oldoptions);

    FOREACH(it, info)
        sout << *it << " ";
    out = sout.str();
    
    return true;
}

bool orEnvTriangulate(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    vector<int> collision;
    vector<float> info;

    pserv->SyncWithWorkerThread();

    int inclusive;
    set<int> setobjids;
    stringstream sin(in), sout;
    sin >> inclusive;
    while(!sin.eof()) {
        int id;
        sin >> id;
        if( !sin )
            break;
        setobjids.insert(id);
    }

    LockEnvironment envlock(pserv->GetEnv());

    vector<KinBody*> vbodies;
    pserv->GetEnv()->GetBodies(vbodies);

    KinBody::Link::TRIMESH trimesh;
    FOREACH(itbody, vbodies) {
        if( (setobjids.find((*itbody)->GetNetworkId()) == setobjids.end()) ^ !inclusive )
            continue;
        pserv->GetEnv()->Triangulate(trimesh, *itbody);
    }

    assert( (trimesh.indices.size()%3) == 0 );
    sout << trimesh.vertices.size() << " " << trimesh.indices.size()/3 << " ";
    FOREACH(itvert, trimesh.vertices)
        sout << itvert->x << " " << itvert->y << " " << itvert->z << " ";
    FOREACH(itind, trimesh.indices)
        sout << *itind << " ";

    out = sout.str();
    return true;
}

/// success = orPlannerInit(planner, robot, params) - initializes the planner
bool _worPlannerInit(char* in, void* pData, RaveServer* pserv)
{
    void** p = (void**)pData;
    bool bsuccess = ((PlannerBase*)p[0])->InitPlan((RobotBase*)p[1], (PlannerBase::PlannerParameters*)p[2]);
    *(bool*)(p+3) = bsuccess;
    return true;
}

bool orPlannerInit(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char strnull[] = "";
    char* pstartargs = strchr(in, ' ');
    if( pstartargs == NULL )
        return false;

    PlannerBase* planner = orMacroGetPlanner(in);
    if( planner == NULL )
        return false;

    pstartargs++;
    RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),pstartargs);
    if( robot == NULL )
        return false;

    pstartargs = strchr(in, ' ');
    if( pstartargs != NULL )
        *pstartargs++ = 0;
    else
        pstartargs = strnull;

    *ppPassData = NULL;
    PlannerBase::PlannerParameters params;
    
    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = _worPlannerInit;
    worker.pdata = malloc( sizeof(void*) * 4 );
    ((void**)worker.pdata)[0] = planner;
    ((void**)worker.pdata)[1] = robot;
    ((void**)worker.pdata)[2] = &params;
    ((void**)worker.pdata)[3] = NULL;
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();

    // get the result
    out = ((void**)worker.pdata)[3] ? "1" : "0";
    free(worker.pdata);

    return true;
}

/// trajectory = orPlannerPlan(planner, robot) - execute the planner
bool _worPlannerPlan(char* in, void* pData, RaveServer* pserv)
{
    void** p = (void**)pData;
    ((PlannerBase*)p[0])->PlanPath((Trajectory*)p[1]);
    return true;
}

bool orPlannerPlan(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    PlannerBase* planner = orMacroGetPlanner(in);
    if( planner == NULL )
        return false;

    boost::shared_ptr<Trajectory> traj(pserv->GetEnv()->CreateTrajectory(planner->GetRobot()->GetDOF()));

    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = _worPlannerPlan;
    worker.pdata = malloc( sizeof(void*) * 4 );
    ((void**)worker.pdata)[0] = planner;
    ((void**)worker.pdata)[1] = traj.get();
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();

    stringstream ss;
    traj->Write(ss, Trajectory::TO_OneLine|Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
    out = ss.str();
    return true;
}

/// contacts = orKinBodyGetContacts(body) - returns all contact forces and positions of a body
bool orKinBodyGetContacts(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    return true;
}

// waits for rave to finish commands
// if a robot id is specified, also waits for that robot's trajectory to finish
bool orEnvWait(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();
    fflush(NULL); // flush all streams
    out = "1";

    if( in != NULL ) {

        int timeout = -1;
        char* ptimeout = strchr(in, ' ');

        // might be a robot id
        RobotBase* robot = orMacroGetRobot(pserv->GetEnv(),in);
        if( robot == NULL )
            return true;

        out = "0";
        if( ptimeout != NULL ) {
            timeout = (int)(1000*atof(ptimeout)); // specified in seconds
            RAVELOG_DEBUGA("orEnvWait: %dms\n", timeout);
        }

        if( robot->GetController() != NULL ) {
            while( !robot->GetController()->IsDone() ) {

                Sleep(1);

                if( timeout > 0 ) {
                    if( --timeout == 0 )
                        break;
                }
                if( pserv->IsClosing() )
                    return false;
            }

            if( timeout != 0 ) // only ret success
                out = "1";
        }
    }

    return true;
}

bool _worProblemSendCommand(char* in, void* pData, RaveServer* pserv)
{
    void** mydata = (void**)pData;
    int problemid = *(int*)mydata[0];
    boost::shared_ptr<LockEnvironment> penvlock;

    if( *(bool*)mydata[3] )
        penvlock.reset(new LockEnvironment(pserv->GetEnv()));

    if( problemid > 0 ) {
        map<int, pair<ProblemInstance*, wstring> >::iterator it = s_mapProblems.find(problemid);
        if( it == s_mapProblems.end() ) {
            RAVELOG_WARNA("failed to find problem %d\n", problemid);
            return false;
        }
        assert( it->second.first != NULL );
        it->second.first->SendCommand((const char*)mydata[1], *(string*)mydata[2]);
    }
    else {
        string curout;
        string* pout = (string*)mydata[2];
        FOREACHC(itprob, pserv->GetEnv()->GetProblems()) {
            curout = ""; // reset
            (*itprob)->SendCommand((const char*)mydata[1], curout);
            *pout += curout;
        }
    }
    return true;
}

/// sends a comment to the problem
bool orProblemSendCommand(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    char* pstartargs = strtok(in, " ");
    if( pstartargs == NULL )
        return false;
    int problemid = atoi(pstartargs);
    pstartargs = strtok(NULL, " ");
    if( pstartargs == NULL )
        return false;
    int dosync = atoi(pstartargs);
    pstartargs = strtok(NULL, " ");
    if( pstartargs == NULL )
        return false;
    bool bDoLock = atoi(pstartargs)!=0;
    pstartargs = strtok(NULL, "\0");
    if( pstartargs == NULL )
        return false;

    pserv->SyncWithWorkerThread();

    void* mydata[4] = { &problemid, pstartargs, &out, &bDoLock };
    *(void**)ppPassData = mydata;

    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = dosync ? _worProblemSendCommand : _worProblemSendCommand;
    worker.pdata = mydata;
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();

    return true;
}

/// success = orEnvLoadPlugin(pluginame)
bool _worEnvLoadPlugin(char* in, void *pData, RaveServer* pserv)
{
    void** mydata = (void**)pData;
    string* pout = (string*)mydata[1];
    if( pserv->GetEnv()->LoadPlugin((const char*)mydata[0]) )
        *pout = "1";
    else *pout = "0";
    return true;
}

/// sends a comment to the problem
bool orEnvLoadPlugin(char* in, string& out, void **ppPassData, RaveServer* pserv)
{
    pserv->SyncWithWorkerThread();

    void* mydata[2] = { in, &out };
    *(void**)ppPassData = mydata;

    RaveServer::WORKERSTRUCT worker;
    worker.fnWorker = _worEnvLoadPlugin;
    worker.pdata = mydata;
    pserv->ScheduleWorker(worker);

    // wait for completion
    pserv->SyncWithWorkerThread();

    return true;
}

//////////////////////
// Simple socket class
//////////////////////
Socket::Socket(RaveServer* pserver)
{
    bInit = false;
    client_sockfd = 0;
    _pserver = pserver;
};

Socket::~Socket()
{
    if( bInit )
        Close();
}

bool Socket::Accept(int server_sockfd)
{
    if( bInit )
        Close();
    
    bool success = true;

	//signal(SIGCHLD, SIG_IGN); 
    //RAVELOG(L"server waiting for connection, %d\n", server_sockfd);

//	char str[sizeof(server_address)+1];
//	memcpy(str, &server_address, sizeof(server_address));
//	str[sizeof(server_address)] = 0;

	client_len = sizeof(client_address);
	client_sockfd = accept(server_sockfd, (struct sockaddr *)&client_address, (socklen_t*)&client_len);

	if( client_sockfd == -1 ) {
		success = false;
        client_sockfd = 0;
	}

    bInit = success;

    return bInit;
}

void Socket::Close()
{
    if( bInit ) {
        // close
        CLOSESOCKET(client_sockfd); client_sockfd = 0;
        bInit = false;
    }
}

bool Socket::ReadLine(string& s)
{
    struct timeval tv;
    fd_set readfds, exfds;
    s.resize(0);

    tv.tv_sec = 0;
    tv.tv_usec = 0;

    	
    FD_ZERO(&exfds);
    FD_SET(client_sockfd, &exfds);
    
    // don't care about writefds and exceptfds:
    int num = select(client_sockfd+1, NULL, NULL, &exfds, &tv);
    
    if ( num > 0 && FD_ISSET(client_sockfd, &exfds) ) {
        RAVELOG_ERRORA("socket exception detected\n");
        Close();
        return false;
    }

    FD_ZERO(&readfds);
    FD_SET(client_sockfd, &readfds);
    
    // don't care about writefds and exceptfds:
    num = select(client_sockfd+1, &readfds, NULL, NULL, &tv);
    
    if ( num == 0 || !FD_ISSET(client_sockfd, &readfds) ) {
      return false;
    }
	
    // protocol: size1 size2 "size1 bytes" "size2 bytes"
    long nBytesReceived; 
    char c;
    int failed = 0;

    while(1) {
        if( (nBytesReceived = recv(client_sockfd, &c, sizeof(char), 0)) > 0 ) {
            if( c == '\n' || c == '\r')
                break;
            s.push_back(c);
        }
        else if( nBytesReceived == 0 ) {
	  //RAVELOG_DEBUGA("closing connection\n");
          //  Close();
            return false;
        }
        else {
            if( failed < 10 ) {    
                failed++;
                Sleep(1);
                continue;
            }
            perror("failed to read line");
            Close();
            return false;
        }
    }

    return true;
}

void Socket::SendData(const void* pdata, int size_to_write)
{
    if( client_sockfd == 0 )
        return;

    int nBytesReceived;

#ifndef _WIN32
    // check if closed, only for linux systems
    struct timeval tv;
    fd_set exfds, writefds;

    tv.tv_sec = 0;
    tv.tv_usec = 0;
	
    FD_ZERO(&exfds);
    FD_ZERO(&writefds);
    FD_SET(client_sockfd, &exfds);
    
    // don't care about writefds and exceptfds:
    int num = select(client_sockfd+1, NULL, NULL, &exfds, &tv);
    
    if ( num > 0 && FD_ISSET(client_sockfd, &exfds) ) {
        RAVELOG_ERRORA("socket exception detected\n");
        Close();
        return;
    }

    FD_SET(client_sockfd, &writefds);
    
    // don't care about writefds and exceptfds:
    num = select(client_sockfd+1, NULL, &writefds, NULL, &tv);
    
    if ( num == 0 || !FD_ISSET(client_sockfd, &writefds) ) {
        RAVELOG_WARNA("no writable socket\n");
        return;
    }

    /*int ret = recv(client_sockfd, NULL, 0, MSG_PEEK|MSG_DONTWAIT);
    if( ret == 0 ) {
      RAVEPRINT(L"socket close detected\n");
        Close();
        return;
	}*/
#endif
        
    char* pbuf = (char*)pdata;

    if( (nBytesReceived = send(client_sockfd, (char*)&size_to_write, 4, 0)) != 4 ) {
        RAVELOG_ERRORA("failed to send command: %d\n", nBytesReceived);
        return;
    }

    while(size_to_write > 0 ) {
        nBytesReceived = send(client_sockfd, pbuf, size_to_write, 0);
        if( nBytesReceived <= 0 ) {
            if( nBytesReceived == -1 )
                return;

            //perror("failed to read line");
            continue;
        }

        size_to_write -= nBytesReceived;
        pbuf += nBytesReceived;
    }
}

RaveServer::RaveServer(EnvironmentBase* penv) : RaveServerBase(penv), _nPort(-1), bInitThread(false), bCloseThread(false)
{
    _pphysics = NULL;
    _pcolchecker = NULL;
    _bWorking = false;
    bDestroying = false;
    
    mapNetworkFns["body_checkcollision"] = RAVENETWORKFN(orEnvCheckCollision, NULL, true);
    mapNetworkFns["body_getjoints"] = RAVENETWORKFN(orBodyGetJointValues, NULL, true);
    mapNetworkFns["body_destroy"] = RAVENETWORKFN(NULL, worBodyDestroy, false);
    mapNetworkFns["body_enable"] = RAVENETWORKFN(NULL, worBodyEnable, false);
    mapNetworkFns["body_getaabb"] = RAVENETWORKFN(orBodyGetAABB, NULL, true);
    mapNetworkFns["body_getaabbs"] = RAVENETWORKFN(orBodyGetAABBs, NULL, true);
    mapNetworkFns["body_getcontacts"] = RAVENETWORKFN(orKinBodyGetContacts, NULL, true);
    mapNetworkFns["body_getlinks"] = RAVENETWORKFN(orBodyGetLinks,NULL, true);
    mapNetworkFns["body_getdof"] = RAVENETWORKFN(orBodyGetDOF, NULL, true);
    mapNetworkFns["body_settransform"] = RAVENETWORKFN(NULL, worKinBodySetTransform, false);
    mapNetworkFns["body_setjoints"] = RAVENETWORKFN(NULL, worBodySetJointValues, false);
    mapNetworkFns["close"] = RAVENETWORKFN(NULL, worEnvClose, false);
    mapNetworkFns["createrobot"] = RAVENETWORKFN(orEnvCreateRobot, NULL, true);
    mapNetworkFns["createplanner"] = RAVENETWORKFN(orEnvCreatePlanner, NULL, true);
    mapNetworkFns["createbody"] = RAVENETWORKFN(orEnvCreateKinBody, NULL, true);
    mapNetworkFns["createproblem"] = RAVENETWORKFN(orEnvCreateProblem, worEnvCreateProblem, true);
    mapNetworkFns["env_dstrprob"] = RAVENETWORKFN(NULL, worEnvDestroyProblem, false);
    mapNetworkFns["env_getbodies"] = RAVENETWORKFN(orEnvGetBodies, NULL, true);
    mapNetworkFns["env_getrobots"] = RAVENETWORKFN(orEnvGetRobots, NULL, true);
    mapNetworkFns["env_getbody"] = RAVENETWORKFN(orEnvGetBody, NULL, true);
    mapNetworkFns["env_loadplugin"] = RAVENETWORKFN(orEnvLoadPlugin, NULL, true);
    mapNetworkFns["env_raycollision"] = RAVENETWORKFN(orEnvRayCollision, NULL, true);
    mapNetworkFns["env_triangulate"] = RAVENETWORKFN(orEnvTriangulate, NULL, true);
    mapNetworkFns["loadscene"] = RAVENETWORKFN(orEnvLoadScene, NULL, true);
    mapNetworkFns["planner_init"] = RAVENETWORKFN(orPlannerInit, NULL, true);
    mapNetworkFns["planner_plan"] = RAVENETWORKFN(orPlannerPlan, NULL, true);
    mapNetworkFns["plot"] = RAVENETWORKFN(orEnvPlot, worEnvPlot, true); 
    mapNetworkFns["problem_sendcmd"] = RAVENETWORKFN(orProblemSendCommand, NULL, true);
    mapNetworkFns["robot_controllersend"] = RAVENETWORKFN(orRobotControllerSend, worRobotControllerSend, true);
    mapNetworkFns["robot_controllerset"] = RAVENETWORKFN(orRobotControllerSet, NULL, true);
    mapNetworkFns["robot_getactivedof"] = RAVENETWORKFN(orRobotGetActiveDOF, NULL, true);
    mapNetworkFns["robot_getdofvalues"] = RAVENETWORKFN(orRobotGetDOFValues, NULL, true);
    mapNetworkFns["robot_getlimits"] = RAVENETWORKFN(orRobotGetDOFLimits, NULL, true);
    mapNetworkFns["robot_getmanipulators"] = RAVENETWORKFN(orRobotGetManipulators, NULL, true);
    mapNetworkFns["robot_getsensors"] = RAVENETWORKFN(orRobotGetAttachedSensors, NULL, true);
    mapNetworkFns["robot_sensorsend"] = RAVENETWORKFN(orRobotSensorSend, NULL, true);
    mapNetworkFns["robot_sensordata"] = RAVENETWORKFN(orRobotSensorData, NULL, true);
    mapNetworkFns["robot_setactivedofs"] = RAVENETWORKFN(NULL, worRobotSetActiveDOFs, false);
    mapNetworkFns["robot_setdof"] = RAVENETWORKFN(NULL, worRobotSetDOFValues, false);
    mapNetworkFns["robot_traj"] = RAVENETWORKFN(NULL, worRobotStartActiveTrajectory, false);
    mapNetworkFns["render"] = RAVENETWORKFN(NULL, worRender, false);
    mapNetworkFns["setoptions"] = RAVENETWORKFN(orEnvSetOptions, worSetOptions, false);
    mapNetworkFns["step"] = RAVENETWORKFN(NULL, worStep, false);
    mapNetworkFns["test"] = RAVENETWORKFN(NULL, NULL, false);
    mapNetworkFns["wait"] = RAVENETWORKFN(orEnvWait, NULL, true);
    
    pthread_mutex_init(&_mutexWorker, NULL);
    pthread_cond_init(&_condWorker, NULL);

    pfLog = fopen("serverlog.txt", "w");
    if( pfLog != NULL )
        RAVELOG_DEBUGA("logging network to serverlog.txt\n");
}

RaveServer::~RaveServer()
{
    Destroy();

    pthread_mutex_destroy(&_mutexWorker);
    pthread_cond_destroy(&_condWorker);

    if( pfLog != NULL ) {
        fclose(pfLog);
        pfLog = NULL;
    }

    delete _pphysics;
    delete _pcolchecker;
}

void RaveServer::Destroy()
{
    Reset();
    s_mapFigureIds.clear();
    s_mapProblems.clear();
    s_mapPlanners.clear();

    pthread_mutex_lock(&_mutexWorker); // need lock to keep multiple threads out of Destroy
    if( bDestroying ) {
        pthread_mutex_unlock(&_mutexWorker);
        return;
    }
    bDestroying = true;
    pthread_mutex_unlock(&_mutexWorker);

    if( bInitThread ) {
        bCloseThread = true;
        void* retvalue;
        pthread_cond_signal(&_condWorker); // in case
        pthread_join(servthread, &retvalue);

        FOREACH(it, listSocketThreads) pthread_join(*it, &retvalue);
        listSocketThreads.clear();

        bCloseThread = false;
        bInitThread = false;
    
        CLOSESOCKET(server_sockfd); server_sockfd = 0;
    }
    
    bDestroying = false;
}

void RaveServer::Reset()
{
    pthread_mutex_lock(&_mutexWorker);
    listWorkers.clear();
    pthread_mutex_unlock(&_mutexWorker);

    // wait for worker thread to stop
    while(_bWorking) {
        pthread_cond_signal(&_condWorker);
        Sleep(1);
    }

    FOREACH(it, s_mapFigureIds)  
        GetEnv()->closegraph(it->second);
    s_mapFigureIds.clear();
}

bool RaveServer::Init(int port)
{
    Destroy();

    _nPort = port;
    memset(&server_address, 0, sizeof(server_address));
	server_sockfd = socket(AF_INET, SOCK_STREAM, 0);
	server_address.sin_family = AF_INET;
	server_address.sin_addr.s_addr = htonl(INADDR_ANY);
	server_address.sin_port = htons(_nPort);
	server_len = sizeof(server_address);

	// this allows to immediately reconnect to OpenRave
	// when the program crashed and is rerun immediately
	int yes = 1;
	int err = setsockopt(server_sockfd, SOL_SOCKET,SO_REUSEADDR, (const char*)&yes, sizeof(int));
    if( err ) {
        RAVELOG_ERRORA("failed to set socket option, err=%d\n", err);
		perror("failed to set socket options\n");
		return false;
	}

	err = bind(server_sockfd, (struct sockaddr *)&server_address, server_len);
    if( err ) {
        RAVELOG_ERRORA("failed to bind server to port %d, error=%d\n", _nPort, err);
        return false;
    }

    err = listen(server_sockfd, 16);
    if( err ) {
        RAVELOG_ERRORA("failed to listen to server port %d, error=%d\n", _nPort, err);
        return false;
    }

    // set nonblocking
#ifdef _WIN32
    u_long flags = 1;
    ioctlsocket(server_sockfd, FIONBIO, &flags);
#else
    int flags;

    /* If they have O_NONBLOCK, use the Posix way to do it */
#if defined(O_NONBLOCK)
    /* Fixme: O_NONBLOCK is defined but broken on SunOS 4.1.x and AIX 3.2.5. */
    if (-1 == (flags = fcntl(server_sockfd, F_GETFL, 0)))
        flags = 0;
    if( fcntl(server_sockfd, F_SETFL, flags | O_NONBLOCK) < 0 )
        return false;
#else
    /* Otherwise, use the old way of doing it */
    flags = 1;
    if( ioctl(server_sockfd, FIOBIO, &flags) < 0 )
        return false;
#endif
#endif

    if( pthread_create(&servthread, NULL, listen_thread, this) ) {
        RAVELOG_ERRORA("Failed to create thread\n");
        return false;
    }

    bInitThread = true;
    return true;
}

void RaveServer::Worker()
{
    _bWorking = true;
    pthread_mutex_lock(&_mutexWorker);
    list<WORKERSTRUCT> listlocalworkers;
    if( listWorkers.size() == 0 ) {
        _bWorking = false;
        pthread_cond_signal(&_condWorker);
        pthread_mutex_unlock(&_mutexWorker);
        return;
    }

    listlocalworkers.swap(listWorkers);
    pthread_mutex_unlock(&_mutexWorker);
    
    // transfer the current workers to a temporary list so 
    FOREACH(it, listlocalworkers)
        it->fnWorker(&it->args[0], it->pdata, this);
    
    pthread_cond_signal(&_condWorker);
    *(volatile bool*)&_bWorking = false;
}

void RaveServer::SyncWithWorkerThread()
{
    pthread_mutex_lock(&_mutexWorker);

    while((listWorkers.size() > 0 || _bWorking) && !bCloseThread) 
        pthread_cond_wait(&_condWorker, &_mutexWorker);

    pthread_mutex_unlock(&_mutexWorker);
}

void RaveServer::ScheduleWorker(const RaveServer::WORKERSTRUCT& w)
{
    pthread_mutex_lock(&_mutexWorker);
    listWorkers.push_back(w);
    pthread_mutex_unlock(&_mutexWorker);
}

void* RaveServer::listen_thread(void* param)
{
    RaveServer* pserv = (RaveServer*)param;
    return pserv->_listen_thread();
}

void* RaveServer::_listen_thread()
{
    Socket* psocket = new Socket(this);

    while(!bCloseThread) {

        // finally initialize the socket
        if( !psocket->Accept(server_sockfd) ) {
            Sleep(100);
            continue;
        }

        // start a new thread
        RAVELOG_VERBOSEA("creating socket connection\n");
        pthread_t readthread;
        if( pthread_create(&readthread, NULL, read_thread, psocket) ) {
            RAVELOG_ERRORA("Failed to create thread\n");
            continue;
        }
        listSocketThreads.push_back(readthread);

        psocket = new Socket(this);
    }

    delete psocket;
    RAVELOG_DEBUGA("**Server thread exiting\n");

    return NULL;
}

void* RaveServer::read_thread(void* param)
{
    Socket* psocket = (Socket*)param;
    return psocket->GetServer()->_read_thread(psocket);
}

void* RaveServer::_read_thread(Socket* psocket)
{
    assert( psocket != NULL );
    string line;
    RaveServer::WORKERSTRUCT w;
    
    while(!bCloseThread) {
        //Sleep(100);
        if( psocket->ReadLine(line) && line.length() ) {
            
            if( pfLog != NULL && GetEnv()->GetDebugLevel()>0) {
                static int index=0;
                fprintf(pfLog, "%4d: %s\n", index++, line.c_str());
            }
            
            char* pcmd = &line[0];
            char* pcmdend = strchr(pcmd, ' ');
            if( pcmdend != NULL )
                *pcmdend++ = 0;
            else pcmdend = pcmd+line.size();

            string cmd = &line[0];
            map<string, RAVENETWORKFN>::iterator itfn = mapNetworkFns.find(cmd);
            if( itfn != mapNetworkFns.end() ) {

                bool bCallWorker = true;
                w.pdata = NULL;

                // need to set w.args before pcmdend is modified
                if( pcmdend != NULL && itfn->second.fnWorker != NULL )
                    w.args = pcmdend;

                string out;
                if( itfn->second.fnSocketThread != NULL ) {
                    if( itfn->second.fnSocketThread(pcmdend, out, &w.pdata, this) ) {

                        if( itfn->second.bReturnResult )
                            psocket->SendData(out.c_str(), out.size());
                
                        if( !itfn->second.fnWorker )
                            bCallWorker = false;
                    }
                    else {
                        bCallWorker = false;
                        if( pfLog != NULL  ) {
                            fprintf(pfLog, "  error\n");
                        }
                        if( itfn->second.bReturnResult )
                            psocket->SendData("error\n", 6);
                    }
                }
                else {
                    if( itfn->second.bReturnResult )
                        psocket->SendData(out.c_str(), out.size()); // return dummy
                    bCallWorker = itfn->second.fnWorker!=NULL;
                }
                
                if( bCallWorker ) {
                    assert(itfn->second.fnWorker!=NULL);
                    w.fnWorker = itfn->second.fnWorker;

                    ScheduleWorker(w);
                }
            }
            else {
                RAVELOG_ERRORA("Failed to recognize command: %s\n", cmd.c_str());
                psocket->SendData("error\n",1);
            }
        }
        else if( !psocket->IsInit() )
            break;

        Sleep(1);
    }

    RAVELOG_VERBOSEA("Closing socket connection\n");
    delete psocket;
    return NULL;
}
