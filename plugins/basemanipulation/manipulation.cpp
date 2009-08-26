// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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

#include "basemanipulation.h"
#include "manipulation.h"

/////////////////////////////
// BaseManipulationProblem //
/////////////////////////////

dReal TransformDistance(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    dReal e1 = (t1.rot-t2.rot).lengthsqr4();
    dReal e2 = (t1.rot+t2.rot).lengthsqr4();
    return RaveSqrt((t1.trans-t2.trans).lengthsqr3() + frotweight*min(e1,e2));
}

BaseManipulationProblem::BaseManipulationProblem(EnvironmentBase* penv) : CmdProblemInstance(penv), robot(NULL), pGrasperProblem(NULL)
{
    RegisterCommand("SetActiveManip",(CommandFn)&BaseManipulationProblem::SetActiveManip,
                    "Set the active manipulator");
    RegisterCommand("Traj",(CommandFn)&BaseManipulationProblem::Traj,
                    "Execute a trajectory from a file on the local filesystem");
    RegisterCommand("GrabBody",(CommandFn)&BaseManipulationProblem::GrabBody,
                    "Robot calls ::Grab on a body with its current manipulator");
    RegisterCommand("ReleaseAll",(CommandFn)&BaseManipulationProblem::ReleaseAll,
                    "Releases all grabbed bodies (RobotBase::ReleaseAllGrabbed).");
    RegisterCommand("MoveHandStraight",(CommandFn)&BaseManipulationProblem::MoveHandStraight,
                    "Move the active end-effector in a straight line until collision or IK fails.");
    RegisterCommand("MoveManipulator",(CommandFn)&BaseManipulationProblem::MoveManipulator,
                    "Moves arm joints of active manipulator to a given set of joint values");
    RegisterCommand("MoveActiveJoints",(CommandFn)&BaseManipulationProblem::MoveActiveJoints,
                    "Moves the current active joints to a specified goal destination\n");
    RegisterCommand("MoveToHandPosition",(CommandFn)&BaseManipulationProblem::MoveToHandPosition,
                    "Move the manipulator's end effector to some 6D pose.");
    RegisterCommand("MoveUnsyncJoints",(CommandFn)&BaseManipulationProblem::MoveUnsyncJoints,
                    "Moves the active joints to a position where the inactive (hand) joints can\n"
                    "fully move to their goal. This is necessary because synchronization with arm\n"
                    "and hand isn't guaranteed.\n"
                    "Options: handjoints savetraj planner");
    RegisterCommand("CloseFingers",(CommandFn)&BaseManipulationProblem::CloseFingers,
                    "Closes the active manipulator fingers using the grasp planner.");
    RegisterCommand("ReleaseFingers",(CommandFn)&BaseManipulationProblem::ReleaseFingers,
                    "Releases the active manipulator fingers using the grasp planner.\n"
                    "Also releases the given object.");
    RegisterCommand("IKTest",(CommandFn)&BaseManipulationProblem::IKtest,
                    "Tests for an IK solution if active manipulation has an IK solver attached");
    RegisterCommand("DebugIK",(CommandFn)&BaseManipulationProblem::DebugIK,
                    "Function used for debugging and testing an IK solver");
    RegisterCommand("SmoothTrajectory",(CommandFn)&BaseManipulationProblem::SmoothTrajectory,
                    "Smooths a trajectory of points and returns the new trajectory such that\n"
                    "it is guaranteed to contain no co-linear points in configuration space\n");
    RegisterCommand("Help", (CommandFn)&BaseManipulationProblem::Help,"Help message");
}

void BaseManipulationProblem::Destroy()
{
    CmdProblemInstance::Destroy();
    robot = NULL;
}

BaseManipulationProblem::~BaseManipulationProblem()
{
    Destroy();
}

int BaseManipulationProblem::main(const char* cmd)
{
    if( cmd == NULL )
        return 0;

    RAVELOG_DEBUGA("env: %s\n", cmd);

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = _ravembstowcs(p);

    _strRRTPlannerName.resize(0);
    
    const char* planneropt = strstr(cmd, "planner ");
    if( planneropt != NULL ) {

        planneropt += 8;
        char plannername[255];
        if( sscanf(planneropt, "%s", plannername) == 1 ) {
            // try to create it
            PlannerBase* planner = GetEnv()->CreatePlanner(_ravembstowcs(plannername).c_str());
            if( planner != NULL ) {
                _strRRTPlannerName = _ravembstowcs(plannername);
            }
            else {
                RAVELOG_ERRORA("BaseManipulation: failed to find planner %s\n", plannername);
            }
        }
    }

    RAVELOG_DEBUGA("Starting BaseManipulationProblem\n");
    pGrasperProblem = GetEnv()->CreateProblem(L"GrasperProblem");
    if( pGrasperProblem == NULL ) {
        RAVELOG_WARNA("Failed to create GrasperProblem\n");
    }
    else if( pGrasperProblem->main(NULL) < 0 )
        return -1;

    if( _strRRTPlannerName.size() == 0 ) {
        PlannerBase* planner = GetEnv()->CreatePlanner(L"rBiRRT");
        if( planner != NULL ) {
            _strRRTPlannerName = L"rBiRRT";
        }
        else {
            planner = GetEnv()->CreatePlanner(L"BiRRT");
            if( planner != NULL )
                _strRRTPlannerName = L"BiRRT";
        }

        if( planner == NULL ) {
            RAVELOG_ERRORA("failed to find birrt planner\n");
            return -1;
        }
        delete planner;
    }
    
    RAVELOG_DEBUGA("BaseManipulation: using %S planner\n", _strRRTPlannerName.c_str());

    SetActiveRobots(GetEnv()->GetRobots());

    return 0;
}

void BaseManipulationProblem::SetActiveRobots(const std::vector<RobotBase*>& robots)
{
    robot = NULL;

    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBase*>::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( wcsicmp((*itrobot)->GetName(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}

bool BaseManipulationProblem::SendCommand(const char* cmd, string& response)
{
    SetActiveRobots(GetEnv()->GetRobots());
    if( robot == NULL ) {
        RAVELOG_ERRORA("robot is NULL, send command failed\n");
        return false;
    }
    return CmdProblemInstance::SendCommand(cmd, response);
}

void BaseManipulationProblem::Query(const char* query, string& response)
{
}

bool BaseManipulationProblem::SimulationStep(dReal fElapsedTime)
{
    return 0;
}

bool BaseManipulationProblem::SetActiveManip(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Starting SetActiveManip...\n");
    string cmd;
    int index = -1;

    if(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            return false;
        
        if( stricmp(cmd.c_str(), "rightarm") == 0 ) index = 4;
        else if( stricmp(cmd.c_str(), "leftarm") == 0 ) index = 3;
        else if( stricmp(cmd.c_str(), "leftleg") == 0 ) index = 1;
        else if( stricmp(cmd.c_str(), "rightleg") == 0 ) index = 2;
        else if( stricmp(cmd.c_str(), "head") == 0 ) index = 0;
        else {
            index = atoi(cmd.c_str());
        }
    }

    if( index >= 0 && index < (int)robot->GetManipulators().size() ) {
        robot->SetActiveManipulator(index);
        sout << "1";
    }
    else {
        RAVELOG_ERRORA("invaild manip %d\n", index);
        sout << "0";
    }

    return true;
}

string getfilename_withseparator(istream& sinput, char separator)
{
    string filename;
    if( !getline(sinput, filename, separator) ) {
        // just input directly
        RAVELOG_ERRORA("graspset filename not terminated with ';'\n");
        sinput >> filename;
    }

    // trim leading spaces
    size_t startpos = filename.find_first_not_of(" \t");
    size_t endpos = filename.find_last_not_of(" \t");

    // if all spaces or empty return an empty string  
    if(( string::npos == startpos ) || ( string::npos == endpos))
        return "";

    filename = filename.substr( startpos, endpos-startpos+1 );
    return filename;
}

bool BaseManipulationProblem::Traj(ostream& sout, istream& sinput)
{
    string filename; sinput >> filename;
    if( !sinput )
        return false;

    Trajectory* ptraj = GetEnv()->CreateTrajectory(robot->GetDOF());
    
    char sep = ' ';
    if( filename == "sep" ) {
        sinput >> sep;
        filename = getfilename_withseparator(sinput,sep);
    }

    if( filename == "stream" ) {
        // the trajectory is embedded in the stream
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory from stream\n");

        if( !ptraj->Read(sinput, robot) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to get trajectory\n");
            return false;
        }
    }
    else {
        RAVELOG_DEBUGA("BaseManipulation: reading trajectory: %s\n", filename.c_str());

        if( !ptraj->Read(filename.c_str(), robot) ) {
            RAVELOG_ERRORA("BaseManipulation: failed to read trajectory %s\n", filename.c_str());
            return false;
        }
    }
        
    bool bResetTrans = false; sinput >> bResetTrans;
    
    if( bResetTrans ) {
        RAVELOG_VERBOSEA("resetting transformations of trajectory\n");
        Transform tcur = robot->GetTransform();
        // set the transformation of every point to the current robot transformation
        FOREACH(itpoint, ptraj->GetPoints())
            itpoint->trans = tcur;
    }

    RAVELOG_VERBOSEA("executing traj with %"PRIdS" points\n", ptraj->GetPoints().size());
    robot->SetMotion(ptraj);
    sout << "1";
    return true;
}

bool BaseManipulationProblem::GrabBody(ostream& sout, istream& sinput)
{
    KinBody* ptarget = NULL;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "name") == 0 ) {
            string name;
            sinput >> name;
            if( name.size() > 0 )
                ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
        }
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    if(ptarget == NULL) {
        RAVELOG_ERRORA("ERROR Manipulation::GrabBody - Invalid body name.\n");
        return false;
    }

    RAVELOG_DEBUGA("robot %S:%S grabbing body %S...\n", robot->GetName(), (robot->GetActiveManipulator()!=NULL&&robot->GetActiveManipulator()->pEndEffector!=NULL)?robot->GetActiveManipulator()->pEndEffector->GetName():L"(NULL)",ptarget->GetName());
    robot->Grab(ptarget);
    return true;
}

bool BaseManipulationProblem::ReleaseAll(ostream& sout, istream& sinput)
{
    if( robot != NULL ) {
        RAVELOG_DEBUGA("Releasing all bodies\n");
        robot->ReleaseAllGrabbed();
    }
    return true;
}

bool BaseManipulationProblem::MoveHandStraight(ostream& sout, istream& sinput)
{
    float stepsize = 0.003f;
    Vector direction = Vector(0,1,0);
    string strtrajfilename;
    bool bExecute = true, bOutputTraj = false;

    int minsteps = 0;
    int maxsteps = 10000;

    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return -1;
    
    bool bIgnoreFirstCollision = true;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "minsteps") == 0 ) {
            sinput >> minsteps;
        }
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "maxsteps") == 0 ) {
            sinput >> maxsteps;
        }
        else if( stricmp(cmd.c_str(), "stepsize") == 0 ) {
            sinput >> stepsize;
        }
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else if( stricmp(cmd.c_str(), "direction") == 0 ) {
            sinput >> direction.x >> direction.y >> direction.z;
        }
        else if( stricmp(cmd.c_str(), "ignorefirstcollision") == 0 )
            sinput >> bIgnoreFirstCollision;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }
    
    RAVELOG_DEBUGA("Starting MoveHandStraight dir=(%f,%f,%f)...\n",(float)direction.x, (float)direction.y, (float)direction.z);
    robot->RegrabAll();

    RobotBase::RobotStateSaver saver(robot);

    robot->SetActiveDOFs(pmanip->_vecarmjoints,0,NULL);
    JitterActiveDOF(robot); // try to jitter out, don't worry if it fails

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    Trajectory::TPOINT point;
    vector<dReal> vPrevValues;
    bool bPrevInCollision = GetEnv()->CheckCollision(robot)||robot->CheckSelfCollision();
    robot->GetActiveDOFValues(vPrevValues);

    if( bPrevInCollision && !bIgnoreFirstCollision ) {
        RAVELOG_WARNA("MoveHandStraight: robot in collision\n");
        return false;
    }

    Transform handTr = robot->GetActiveManipulator()->GetEndEffectorTransform();

    point.q = vPrevValues;
    ptraj->AddPoint(point);

    int i;
    for (i = 0; i < maxsteps;  i++) {
        handTr.trans += stepsize*direction;
        if( !pmanip->FindIKSolution(handTr,point.q,false)) {
            RAVELOG_DEBUGA("Arm Lifting: broke due to ik\n");
            break;
        }
        
        size_t j = 0;
        for(; j < point.q.size(); j++) {
            if(fabsf(point.q[j] - vPrevValues[j]) > 0.2)
                break;
        }

        if( j < point.q.size()) {
            RAVELOG_DEBUGA("Arm Lifting: broke due to discontinuity\n");
            break;
        }
        
        robot->SetActiveDOFValues(NULL,&point.q[0]);
        
        bool bInCollision = GetEnv()->CheckCollision(robot)||robot->CheckSelfCollision();
        if(bInCollision && !bPrevInCollision && i >= minsteps) {
            RAVELOG_DEBUGA("Arm Lifting: broke due to collision\n");
            break;
        }
        
        ptraj->AddPoint(point);
        vPrevValues = point.q;
        bPrevInCollision = bInCollision;
    }
    
    if( i > 0 ) {
        RAVELOG_DEBUGA("hand moved %f\n", (float)i*stepsize);
        
        SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);
        sout << "1";
        return i >= minsteps;
    }

    RAVELOG_DEBUGA("hand didn't move\n");
    return i >= minsteps;
}

bool BaseManipulationProblem::MoveManipulator(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Starting MoveManipulator...\n");

    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    string strtrajfilename;
    bool bExecute = true, bOutputTraj = false;
    std::vector<dReal> goals;
    PlannerBase::PlannerParameters params;
    params.nMaxIterations = 4000; // max iterations before failure

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "armvals") == 0 ) {
            goals.resize(pmanip->_vecarmjoints.size());
            FOREACH(it, goals)
                sinput >> *it;
        }
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "maxiter") == 0 )
            sinput >> params.nMaxIterations;
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }
    
    if( goals.size() != pmanip->_vecarmjoints.size() )
        return false;

    RobotBase::RobotStateSaver saver(robot);

    robot->SetActiveDOFs(pmanip->_vecarmjoints);
    JitterActiveDOF(robot);
    
    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

    std::vector<dReal> values;
    robot->GetActiveDOFValues(values);

    // make sure the initial and goal configs are not in collision
    robot->SetActiveDOFValues(NULL, &goals[0], true);
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("jitter failed\n");
        return false;
    }
    robot->GetActiveDOFValues(params.vgoalconfig);

    robot->SetActiveDOFValues(NULL, &values[0]);
    
    // jitter again for initial collision
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("jitter failed\n");
        return false;
    }
    robot->GetActiveDOFValues(params.vinitialconfig);

    boost::shared_ptr<PlannerBase> rrtplanner(GetEnv()->CreatePlanner(_strRRTPlannerName.c_str()));

    if( rrtplanner.get() == NULL ) {
        RAVELOG_WARNA("failed to create planner\n");
        return false;
    }
    
    bool bSuccess = false;
    RAVELOG_INFOA("starting planning\n");
    
    for(int iter = 0; iter < 3; ++iter) {
        if( !rrtplanner->InitPlan(robot, &params) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            break;
        }
        
        if( rrtplanner->PlanPath(ptraj.get()) ) {
            bSuccess = true;
            RAVELOG_INFOA("finished planning\n");
            break;
        }
        else RAVELOG_WARNA("PlanPath failed\n");
    }

    if( !bSuccess )
        return false;
    
    SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);
    sout << "1";
    return true;
}

bool BaseManipulationProblem::MoveActiveJoints(ostream& sout, istream& sinput)
{
    string strtrajfilename;
    bool bExecute = true, bOutputTraj = false;
    
    PlannerBase::PlannerParameters params;
    params.nMaxIterations = 4000; // max iterations before failure

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "goal") == 0 ) {
            params.vgoalconfig.resize(robot->GetActiveDOF());
            FOREACH(it, params.vgoalconfig)
                sinput >> *it;
        }
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "maxiter") == 0 )
            sinput >> params.nMaxIterations;
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    if( (int)params.vgoalconfig.size() != robot->GetActiveDOF() )
        return false;
    
    RobotBase::RobotStateSaver saver(robot);

    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("failed\n");
        return false;
    }

    // restore
    robot->GetActiveDOFValues(params.vinitialconfig);
    robot->SetActiveDOFValues(NULL, &params.vgoalconfig[0]);
    
    // jitter again for goal
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("failed\n");
        return false;
    }

    boost::shared_ptr<PlannerBase> rrtplanner(GetEnv()->CreatePlanner(_strRRTPlannerName.c_str()));

    if( rrtplanner == NULL ) {
        RAVELOG_ERRORA("failed to create BiRRTs\n");
        return false;
    }
    
    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    
    RAVELOG_DEBUGA("starting planning\n");
    
    if( !rrtplanner->InitPlan(robot, &params) ) {
        RAVELOG_ERRORA("InitPlan failed\n");
        return false;
    }
    
    if( !rrtplanner->PlanPath(ptraj.get()) ) {
        RAVELOG_WARNA("PlanPath failed\n");
        return false;
    }

    RAVELOG_DEBUGA("finished planning\n");
    SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);

    return true;
}

bool BaseManipulationProblem::MoveToHandPosition(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Starting MoveToHandPosition...\n");

    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    list<TransformMatrix> listgoals;
    bool bIncludeHandTm = false;
    TransformMatrix handTm = pmanip->GetEndEffectorTransform();
    
    string strtrajfilename;
    bool bExecute = true, bOutputTraj = false;;
    bool bCloneEnv = false;
    
    Vector vconstraintaxis, vconstraintpos;
    int affinedofs = 0;
    int nSeedIkSolutions = 0; // no extra solutions
    int nMaxTries = 3; // max tries for the planner

    PlannerBase::PlannerParameters params;
    params.nMaxIterations = 4000;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "trans") == 0 ) {
            bIncludeHandTm = true;
            sinput >> handTm.trans.x >> handTm.trans.y >> handTm.trans.z;
        }
        else if( stricmp(cmd.c_str(), "rot") == 0 ) {
            bIncludeHandTm = true;
            sinput >> handTm.m[0] >> handTm.m[4] >> handTm.m[8]
                   >> handTm.m[1] >> handTm.m[5] >> handTm.m[9]
                   >> handTm.m[2] >> handTm.m[6] >> handTm.m[10];
        }
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "matrix") == 0 ) {
            listgoals.push_back(TransformMatrix());
            sinput >> listgoals.back();
        }
        else if( stricmp(cmd.c_str(), "matrices") == 0 ) {
            int num = 0;
            sinput >> num;
            while(num-->0) {
                listgoals.push_back(TransformMatrix());
                sinput >> listgoals.back();
            }
        }
        else if( stricmp(cmd.c_str(), "affinedofs") == 0 )
            sinput >> affinedofs;
        else if( stricmp(cmd.c_str(), "maxiter") == 0 )
            sinput >> params.nMaxIterations;
        else if( stricmp(cmd.c_str(), "maxtries") == 0 )
            sinput >> nMaxTries;
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else if( stricmp(cmd.c_str(), "cloneenv") == 0 )
            bCloneEnv = true;
        else if( stricmp(cmd.c_str(), "seedik") == 0 )
            sinput >> nSeedIkSolutions;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }
    
    if( bIncludeHandTm )
        listgoals.push_back(handTm);

    robot->RegrabAll();

    std::vector<dReal> viksolution, armgoals;

    if( nSeedIkSolutions < 0 ) {
        vector<vector<dReal> > solutions;

        FOREACH(ittrans, listgoals) {
            pmanip->FindIKSolutions(*ittrans, solutions, true);
            
            armgoals.reserve(armgoals.size()+solutions.size()*pmanip->_vecarmjoints.size());
            FOREACH(itsol, solutions)
                armgoals.insert(armgoals.end(), itsol->begin(), itsol->end());
        }
    }
    else if( nSeedIkSolutions > 0 ) {
        FOREACH(ittrans, listgoals) {
            int nsampled = SampleIkSolutions(robot, *ittrans, nSeedIkSolutions, armgoals);
            if( nsampled != nSeedIkSolutions )
                RAVELOG_WARNA("only found %d/%d ik solutions\n", nsampled, nSeedIkSolutions);
        }
    }
    else {
        FOREACH(ittrans, listgoals) {
            if( pmanip->FindIKSolution(*ittrans, viksolution, true) ) {
                stringstream s;
                s << "ik sol: ";
                FOREACH(it, viksolution)
                    s << *it << " ";
                s << endl;
                RAVELOG_DEBUGA(s.str().c_str());
                armgoals.insert(armgoals.end(), viksolution.begin(), viksolution.end());
            }
        }
    }

    if( armgoals.size() == 0 ) {
        RAVELOG_WARNA("No IK Solution found\n");
        return false;
    }

    RAVELOG_INFOA("MoveToHandPosition found %"PRIdS" solutions\n", armgoals.size()/pmanip->_vecarmjoints.size());
    
    robot->SetActiveDOFs(pmanip->_vecarmjoints, affinedofs);
    robot->GetActiveDOFValues(params.vinitialconfig);

    robot->SetActiveDOFs(pmanip->_vecarmjoints, 0);

    params.vgoalconfig.reserve(armgoals.size());

    vector<dReal> vgoals;

    for(int i = 0; i < (int)armgoals.size(); i += pmanip->_vecarmjoints.size()) {
        robot->SetActiveDOFValues(NULL, &armgoals[i]);

        robot->SetActiveDOFs(pmanip->_vecarmjoints, affinedofs);

        if( JitterActiveDOF(robot) ) {
            robot->GetActiveDOFValues(vgoals);
            params.vgoalconfig.insert(params.vgoalconfig.end(), vgoals.begin(), vgoals.end());
        }
    }

    if( params.vgoalconfig.size() == 0 ) {
        RAVELOG_WARNA("jitter failed for goal\n");
        return false;
    }

    // restore
    robot->SetActiveDOFValues(NULL, &params.vinitialconfig[0]);

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

    Trajectory::TPOINT pt;
    pt.q = params.vinitialconfig;
    ptraj->AddPoint(pt);
    
    // jitter again for initial collision
    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("jitter failed for initial\n");
        return false;
    }
    robot->GetActiveDOFValues(params.vinitialconfig);

    // check if grasped 
    EnvironmentBase* penv = GetEnv();
    RobotBase* pnewrobot = robot;
    boost::shared_ptr<EnvironmentBase> pcloneenv;

    // clone environment just for testing purposes
    if( bCloneEnv ) {
        pcloneenv.reset(GetEnv()->CloneSelf(Clone_Bodies));
        if( !pcloneenv )
            RAVELOG_ERRORA("failed to clone environment\n");
        else
            penv = pcloneenv.get();

        pnewrobot = (RobotBase*)penv->GetKinBody(robot->GetName());
    }

    boost::shared_ptr<PlannerBase> rrtplanner(penv->CreatePlanner(_strRRTPlannerName.c_str()));

    if( rrtplanner.get() == NULL ) {
        RAVELOG_ERRORA("failed to create BiRRTs\n");
        return false;
    }
    
    bool bSuccess = false;
    RAVELOG_INFOA("starting planning\n");

    for(int iter = 0; iter < nMaxTries; ++iter) {
        if( !rrtplanner->InitPlan(pnewrobot, &params) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            return false;
        }
        
        if( rrtplanner->PlanPath(ptraj.get()) ) {
            bSuccess = true;
            RAVELOG_INFOA("finished planning\n");
            break;
        }
        else RAVELOG_WARNA("PlanPath failed\n");
    }

    rrtplanner.reset(); // have to destroy before environment
    
    if( !bSuccess )
        return false;

    SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);
    sout << "1";
    return true;
}

bool BaseManipulationProblem::MoveUnsyncJoints(ostream& sout, istream& sinput)
{
    string strplanner = "BasicRRT";
    string strsavetraj;
    string cmd;
    vector<int> vhandjoints;
    vector<dReal> vhandgoal;
    bool bExecute = true, bOutputTraj = false;
    
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strsavetraj;
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "handjoints") == 0 ) {
            int dof = 0;
            sinput >> dof;
            if( !sinput || dof == 0 )
                return false;
            vhandjoints.resize(dof);
            vhandgoal.resize(dof);
            FOREACH(it, vhandgoal)
                sinput >> *it;
            FOREACH(it, vhandjoints)
                sinput >> *it;
        }
        else if( stricmp(cmd.c_str(), "planner") == 0 ) {
            sinput >> strplanner;
        }
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    uint32_t starttime = timeGetTime();

    if( !JitterActiveDOF(robot) ) {
        RAVELOG_WARNA("failed to jitter robot out of collision\n");
    }

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    
    if( !MoveUnsyncGoalFunction::_MoveUnsyncJoints(GetEnv(), robot, ptraj.get(), vhandjoints, vhandgoal, _ravembstowcs(strplanner.c_str()).c_str()) )
        return false;

    assert(ptraj->GetPoints().size() > 0);

    bool bExecuted = SetTrajectory(ptraj.get(), bExecute, strsavetraj, bOutputTraj?&sout:NULL);
    sout << (int)bExecuted << " ";

    sout << (timeGetTime()-starttime)/1000.0f << " ";
    FOREACH(it, ptraj->GetPoints().back().q)
        sout << *it << " ";

    return true;
}

bool BaseManipulationProblem::CloseFingers(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Starting CloseFingers...\n");

    bool bExecute = true, bOutputTraj = false;
    string strtrajfilename;

    Vector direction;
    
    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    vector<dReal> voffset;
    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "offset") == 0 ) {
            voffset.resize(pmanip->_vecjoints.size());
            FOREACH(it, voffset)
                sinput >> *it;
        }
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    RobotBase::RobotStateSaver saver(robot);

    //close fingers
    vector<int> activejoints = pmanip->_vecjoints;

    robot->SetActiveDOFs(activejoints,0,NULL);

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

    // have to add the first point
    Trajectory::TPOINT ptfirst;
    robot->GetActiveDOFValues(ptfirst.q);
 
    boost::shared_ptr<PlannerBase> graspplanner(GetEnv()->CreatePlanner(L"Grasper"));
    if( graspplanner == NULL ) {
        RAVELOG_ERRORA("grasping planner failure!\n");
        return -1;
    }
    
    PlannerBase::PlannerParameters params2;
 
    params2.vinitialconfig.resize(robot->GetActiveDOF());
    robot->GetActiveDOFValues(&params2.vinitialconfig[0]);    
  
    float stand_off = 0.0f; ///start closing fingers when at this distance
    params2.vParameters.push_back(stand_off);
    
    float bface_target = 0.0f; ///point the hand at the target or not (1 = yes, else no)
    params2.vParameters.push_back(bface_target);

    float roll_hand = 0.0f; /// rotate the hand about the palm normal by this many radians
    params2.vParameters.push_back(roll_hand);

    //direction of approach
    params2.vParameters.push_back(direction.x);
    params2.vParameters.push_back(direction.y);
    params2.vParameters.push_back(direction.z);

    //palm normal (not used here)
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);

    //return a trajectory
    params2.vParameters.push_back(0);

    if( !graspplanner->InitPlan(robot, &params2) ) {
        RAVELOG_ERRORA("InitPlan failed\n");
        return false;
    }
    
    if( !graspplanner->PlanPath(ptraj.get()) ) {
        RAVELOG_WARNA("PlanPath failed\n");
        return false;
    }   

    if( ptraj->GetPoints().size() > 0 ) {
        vector<dReal> vclosingsign, vclosingsign_full; // sometimes the sign of closing the fingers can be positive
        vclosingsign_full.insert(vclosingsign_full.end(), robot->GetDOF(), 1);
        
        // extract the sign from each manipulator
        vector<RobotBase::Manipulator>::const_iterator itmanip;
        FORIT(itmanip, robot->GetManipulators()) {
            for(int i = 0; i < (int)itmanip->_vClosedGrasp.size(); ++i) {
                if( itmanip->_vClosedGrasp[i] < itmanip->_vOpenGrasp[i] )
                    vclosingsign_full[itmanip->_vecjoints[i]] = -1;
            }
        }

        vclosingsign.resize(robot->GetActiveDOF());
        for(int i = 0; i < robot->GetActiveDOF(); ++i) {
            int index = robot->GetActiveJointIndex(i);
            if( index >= 0 )
                vclosingsign[i] = vclosingsign_full[index];
        }

        Trajectory::TPOINT p = ptraj->GetPoints().back();
        if(p.q.size() == voffset.size() ) {
            for(size_t i = 0; i < voffset.size(); ++i)
                p.q[i] += voffset[i]*vclosingsign[i];
        }

        ptraj->Clear();
        ptraj->AddPoint(ptfirst);
        ptraj->AddPoint(p);
        SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);
    }

    sout << "1";
    return true;
}

bool BaseManipulationProblem::ReleaseFingers(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Releasing fingers...\n");
    
    KinBody* ptarget = NULL;
    vector<dReal> movingdir(robot->GetActiveDOF());

    bool bExecute = true, bOutputTraj = false;
    string strtrajfilename;

    Vector direction;
    
    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
            bOutputTraj = true;
        else if( stricmp(cmd.c_str(), "writetraj") == 0 )
            sinput >> strtrajfilename;
        else if( stricmp(cmd.c_str(), "target") == 0 ) {
            string name; sinput >> name;
            ptarget = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
        }
        else if( stricmp(cmd.c_str(), "movingdir") == 0 ) {
            // moving direction, has to be of size activedof
            FOREACH(it, movingdir)
                sinput >> *it;
        }
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

    // have to add the first point
    Trajectory::TPOINT ptfirst;
    if( GetEnv()->CheckCollision(robot) ) {
        robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);

        if( !JitterActiveDOF(robot) ) {
            RAVELOG_WARNA("robot initially in collision\n");
            return false;
        }
    }

    robot->GetActiveDOFValues(ptfirst.q);
 
    boost::shared_ptr<PlannerBase> graspplanner(GetEnv()->CreatePlanner(L"Grasper"));
    if( graspplanner == NULL ) {
        RAVELOG_ERRORA("grasping planner failure!\n");
        return false;
    }
    
    PlannerBase::PlannerParameters params2;
    robot->GetActiveDOFValues(params2.vinitialconfig);
  
    params2.vgoalconfig = movingdir; // which ways the fingers should move

    float stand_off = 0.0f; ///start closing fingers when at this distance
    params2.vParameters.push_back(stand_off);
    
    float bface_target = 0.0f; ///point the hand at the target or not (1 = yes, else no)
    params2.vParameters.push_back(bface_target);

    float roll_hand = 0.0f; /// rotate the hand about the palm normal by this many radians
    params2.vParameters.push_back(roll_hand);

    //direction of approach
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);

    //palm normal (not used here)
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);
    params2.vParameters.push_back(0);

    //return a trajectory
    params2.vParameters.push_back(0);

    if( !graspplanner->InitPlan(robot, &params2) ) {
        RAVELOG_ERRORA("InitPlan failed\n");
        return false;
    }
    
    if( !graspplanner->PlanPath(ptraj.get()) ) {
        RAVELOG_WARNA("PlanPath failed\n");
        return false;
    }   

    if( ptarget != NULL )
        robot->Release(ptarget);

    SetTrajectory(ptraj.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);
    sout << "1";
    return true;
}

bool BaseManipulationProblem::IKtest(ostream& sout, istream& sinput)
{
    RAVELOG_DEBUGA("Starting IKtest...\n");

    vector<dReal> varmjointvals, values;
    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    TransformMatrix handTm = pmanip->GetEndEffectorTransform();
    bool bCheckCollision = true;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "trans") == 0 ) {
            sinput >> handTm.trans.x >> handTm.trans.y >> handTm.trans.z;
        }
        else if( stricmp(cmd.c_str(), "rot") == 0 ) {
            sinput >> handTm.m[0] >> handTm.m[4] >> handTm.m[8]
                   >> handTm.m[1] >> handTm.m[5] >> handTm.m[9]
                   >> handTm.m[2] >> handTm.m[6] >> handTm.m[10];
        }
        else if( stricmp(cmd.c_str(), "matrix") == 0 ) {
            sinput >> handTm;
        }
        else if( stricmp(cmd.c_str(), "armjoints") == 0 ) {
            varmjointvals.resize(pmanip->_vecarmjoints.size());
            FOREACH(it, varmjointvals)
                sinput >> *it;
        }
        else if( stricmp(cmd.c_str(), "nocol") == 0 ) {
            bCheckCollision = false;
        }
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    RobotBase::RobotStateSaver saver(robot);

    Transform handTr(handTm);
    
    robot->GetJointValues(values);

    for(size_t i = 0; i < varmjointvals.size(); i++)
        values[pmanip->_vecarmjoints[i]] = varmjointvals[i];

    robot->SetJointValues(NULL,NULL,&values[0]);

    vector<dReal> q1;
    
    if( !pmanip->FindIKSolution(handTr, q1, bCheckCollision) ) {
        RAVELOG_WARNA("No IK solution found\n");
        return false;
    }
    
    stringstream s2;
    s2 << "ik sol: ";
    FOREACH(it, q1) {
        s2 << *it << " ";
        sout << *it << " ";
    }
    s2 << endl;
    RAVELOG_DEBUGA(s2.str().c_str());
    return true;
}

bool DebugIKFindSolution(RobotBase::Manipulator* pmanip, const Transform& twrist,
                         vector<dReal>& viksolution, bool bEnvCollision, dReal* parameters, int paramindex)
{
    for(dReal f = 0; f <= 1; f += 0.001f) {
        parameters[paramindex] = f;
        if( paramindex > 0 ) {
            if( DebugIKFindSolution(pmanip, twrist, viksolution, bEnvCollision, parameters, paramindex-1) )
                return true;
        }
        else {
            if( pmanip->FindIKSolution(twrist, parameters, viksolution, bEnvCollision) )
                return true;
        }
    }

    return false;
}

void DebugIKFindSolutions(RobotBase::Manipulator* pmanip, const Transform& twrist,
                          vector< vector<dReal> >& viksolutions, bool bEnvCollision,
                          dReal* parameters, int paramindex)
{
    for(dReal f = 0; f <= 1; f += 0.001f) {
        parameters[paramindex] = f;
        if( paramindex > 0 ) {
            DebugIKFindSolutions(pmanip, twrist, viksolutions, bEnvCollision, parameters, paramindex-1);
        }
        else {
            vector< vector<dReal> > vtempsol;
            if( pmanip->FindIKSolutions(twrist, parameters, vtempsol, bEnvCollision) ) {
                viksolutions.insert(viksolutions.end(), vtempsol.begin(), vtempsol.end());
            }
        }
    }
}

static bool CheckCollision(RobotBase* probot, const vector<dReal>& q0, const vector<dReal>& q1, const vector<dReal>& qresolutioninv)
{
    assert( probot != NULL && probot->GetDOF() == (int)q0.size() && probot->GetDOF()== (int)q1.size() && probot->GetDOF() == (int)qresolutioninv.size() );
    
    // set the bounds based on the interval type
    int start = 1;

    // first make sure the end is free
    vector<dReal> vtempconfig(probot->GetDOF()), jointIncrement(probot->GetDOF());

    // compute  the discretization
    int i, numSteps = 1;
    for (i = 0; i < (int)vtempconfig.size(); i++) {
        int steps = (int)(fabs(q1[i] - q0[i]) * qresolutioninv[i]);
        if (steps > numSteps)
            numSteps = steps;
    }

    // compute joint increments
    for (i = 0; i < (int)vtempconfig.size(); i++)
        jointIncrement[i] = (q1[i] - q0[i])/((float)numSteps);

    // check for collision along the straight-line path
    // NOTE: this does not check the end config, and may or may
    // not check the start based on the value of 'start'
    for (int f = start; f < numSteps; f++) {

        for (i = 0; i < (int)vtempconfig.size(); i++)
            vtempconfig[i] = q0[i] + (jointIncrement[i] * f);
        
        probot->SetJointValues(NULL, NULL, &vtempconfig[0]);
        if( probot->GetEnv()->CheckCollision(probot) || probot->CheckSelfCollision() )
            return true;
    }

    return false;
}

static void OptimizePathRandomized(RobotBase* probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv, int nMaxIterations)
{
    if( path.size() <= 2 )
        return;

    list< Trajectory::TPOINT >::iterator itstart, itend;
    
    int nrejected = 0;
    int i = nMaxIterations;
    while(i > 0 && nrejected < (int)path.size() ) {

        --i;

        // pick a random node on the path, and a random jump ahead
        int endIndex = 1+RANDOM_INT((int)path.size()-1);
        int startIndex = RANDOM_INT(endIndex);
        
        itstart = path.begin();
        advance(itstart, startIndex);
        itend = itstart;
        advance(itend, endIndex-startIndex);
        nrejected++;

        // check if the nodes can be connected by a straight line
        if (CheckCollision(probot, itstart->q, itend->q, qresolutioninv)) {
            if( nrejected++ > (int)path.size()*2 )
                break;
            continue;
        }

        // splice out in-between nodes in path
        path.erase(++itstart, itend);
        nrejected = 0;

        if( path.size() <= 2 )
            return;
    }
}

// check all pairs of nodes
static void OptimizePathAll(RobotBase* probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv)
{
    list< Trajectory::TPOINT >::iterator itstart = path.begin();
    while(itstart != path.end() ) {
        
        list< Trajectory::TPOINT >::iterator itend = path.end();
        while(--itend != itstart) {
            if (!CheckCollision(probot, itstart->q, itend->q, qresolutioninv)) {
                // splice out in-between nodes in path
                list< Trajectory::TPOINT >::iterator itnext = itstart;
                path.erase(++itnext, itend);
                break;
            }
        }

        ++itstart;
    }
}

bool BaseManipulationProblem::SmoothTrajectory(ostream& sout, istream& sinput)
{
    bool bExecute = true;
    int nMaxSmoothIterations = 100;
    boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
    vector<dReal> qresolutioninv(robot->GetDOF(),50.0f);

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;
        
        if( stricmp(cmd.c_str(), "traj") == 0 ) {
            if( !ptraj->Read(sinput,robot) ) {
                RAVELOG_ERRORA("failed to read trajectory\n");
                return false;
            }
        }
        else if( stricmp(cmd.c_str(), "trajfile") == 0 ) {
            string filename;
            sinput >> filename;
            if( !ptraj->Read(filename.c_str(),robot) ) {
                RAVELOG_ERRORA("failed to read trajectory\n");
                return false;
            }
        }
        else if( stricmp(cmd.c_str(), "maxsmoothiter") == 0 ) {
            sinput >> nMaxSmoothIterations;
        }
        else if( stricmp(cmd.c_str(), "resolution") == 0 ) {
            FOREACH(it,qresolutioninv)
                sinput >> *it;
        }
        else if( stricmp(cmd.c_str(), "execute") == 0 )
            sinput >> bExecute;
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    if( ptraj->GetPoints().size() == 0 ) {
        RAVELOG_ERRORA("trajectory not initialized\n");
        return false;
    }

    list<Trajectory::TPOINT> path;
    FOREACH(itpoint, ptraj->GetPoints())
        path.push_back(*itpoint);

    OptimizePathRandomized(robot,path,qresolutioninv, nMaxSmoothIterations);
    OptimizePathAll(robot, path, qresolutioninv);

    boost::shared_ptr<Trajectory> pnewtraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
    FOREACH(it, path) {
        pnewtraj->AddPoint(Trajectory::TPOINT(it->q,it->trans,0));
    }

    if( bExecute ) {
        pnewtraj->CalcTrajTiming(robot,pnewtraj->GetInterpMethod(),true,false);
        robot->SetMotion(pnewtraj.get());
    }

    pnewtraj->Write(sout, Trajectory::TO_OneLine);
    return true;
}

bool BaseManipulationProblem::DebugIK(ostream& sout, istream& sinput)
{
    int num_itrs = 10000;
    stringstream s;
    fstream fsfile;

    string readfilename, genfilename;
    bool bReadFile = false;
    bool bGenFile = false;
    dReal frotweight = 0.4f, ftransweight = 1.0f;

    RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;

    string cmd;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "readfile") == 0 ) {
            sinput >> readfilename;
            bReadFile = true;
        }
        else if( stricmp(cmd.c_str(), "genfile") == 0 ) {
            sinput >> genfilename;
            bGenFile = true;
        }
        else if( stricmp(cmd.c_str(), "numtests") == 0 ) {
            sinput >> num_itrs;
        }
        else if( stricmp(cmd.c_str(), "rotonly") == 0 ) {
            ftransweight = 0;
        }
        else break;

        if( !sinput ) {
            RAVELOG_ERRORA("failed\n");
            return false;
        }
    }

    vector<dReal> vjoints(pmanip->_vecarmjoints.size(),0), vrand(pmanip->_vecarmjoints.size(),0);
    vector<dReal> vlowerlimit, vupperlimit, viksolution;
    vector< vector<dReal> > viksolutions;

    RAVELOG_DEBUGA("Starting DebugIK... iter=%d\n", num_itrs);
    
    robot->SetActiveDOFs(pmanip->_vecarmjoints);
    robot->GetActiveDOFLimits(vlowerlimit, vupperlimit);
    
    if(bGenFile) {
        fsfile.open(genfilename.c_str(),ios_base::out);
        fsfile << num_itrs <<endl;
    }
    if(bReadFile) {
        fsfile.open(readfilename.c_str(),ios_base::in);
        if(!fsfile.is_open())
        {
            RAVELOG_ERRORA("BaseManipulationProblem::DebugIK - Error: Cannot open specified file.\n");
            return -1;
        }

        fsfile >> num_itrs;
    }

    vector<dReal> vfreeparams(pmanip->GetNumFreeParameters()) ,vfreeparams2(pmanip->GetNumFreeParameters());

    Transform twrist, twrist_out;
    int i = 0;
    int success = 0;
    while(i < num_itrs) {
        if(bReadFile) {
            FOREACH(it, vjoints)
                fsfile >> *it;
            
            fsfile >> twrist;
        }
        else {
            for(int j = 0; j < (int)vjoints.size(); j++) {
                if( GetEnv()->RandomFloat() > 0.05f ) {
                    vjoints[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*GetEnv()->RandomFloat();
                }
                else
                    vjoints[j] = 0;
            }
        }

        robot->SetActiveDOFValues(NULL,&vjoints[0],true);

        if(GetEnv()->CheckCollision(robot) ) {
            RAVELOG_VERBOSEA("robot in collision\n");
            continue;
        }
        if( robot->CheckSelfCollision()) {
            RAVELOG_VERBOSEA("robot in self-collision\n");
            continue;
        }

        RAVELOG_DEBUGA("iteration %d\n",i);
        twrist = pmanip->GetEndEffectorTransform();

        if(bGenFile) {
            FOREACH(it, vjoints)
                fsfile << *it << " ";  
            fsfile << twrist << endl;
        }

        // find a random config
        while(1) {
            for(int j = 0; j < (int)vrand.size(); j++)
                vrand[j] = vlowerlimit[j] + (vupperlimit[j]-vlowerlimit[j])*GetEnv()->RandomFloat();
        
            robot->SetActiveDOFValues(NULL,&vrand[0], true);
            if(!GetEnv()->CheckCollision(robot) && !robot->CheckSelfCollision())
                break;
        }
            
        if( !pmanip->FindIKSolution(twrist, viksolution, true) ) {    
            s.str("");
            s << "FindIKSolution: No ik solution found, i = " << i << endl << "Joint Val: ";
            FOREACH(it, vjoints)
                s << *it << " ";
            s << endl << "Transform: " << twrist << endl << endl;
            RAVELOG_WARNA(s.str().c_str());
            ++i;
            continue;
        }

        robot->SetActiveDOFValues(NULL,&viksolution[0], true);
        twrist_out = pmanip->GetEndEffectorTransform();
        
        if(TransformDistance(twrist, twrist_out, frotweight, ftransweight) > 0.05f) {
            s.str("");
            s << "FindIKSolution: Incorrect IK, i = " << i <<" error: " << TransformDistance(twrist, twrist_out, frotweight, ftransweight) << endl
              << "Original Joint Val: ";
            FOREACH(it, vjoints)
                s << *it << " ";
            s << endl << "Returned Joint Val: ";
            FOREACH(it, viksolution)
                s << *it << " ";
            s << endl << "Transform in: " << twrist << endl;
            s << "Transform out: " << twrist_out << endl << endl;
            RAVELOG_WARNA(s.str().c_str());
            ++i;
            continue;
        }
        
        // test the multiple solution function
        robot->SetActiveDOFValues(NULL,&vrand[0], true);
        if( !pmanip->FindIKSolutions(twrist, viksolutions, true) ) {
            s.str("");
            s << "FindIKSolutions: No ik solution found for, i = " << i << endl << "Joint Val: ";
            FOREACH(it, vjoints)
                s << *it << " ";
            s << endl << "Transform: " << twrist << endl << endl;
            RAVELOG_WARNA(s.str().c_str());
            ++i;
            continue;
        }

        bool bfail = false;
        FOREACH(itsol, viksolutions) {
            robot->SetActiveDOFValues(NULL,&(*itsol)[0], true);
            twrist_out = pmanip->GetEndEffectorTransform();
            if(TransformDistance(twrist, twrist_out, frotweight, ftransweight) > 0.05f) {
                s.str("");
                s << "FindIKSolutions: Incorrect IK, i = " << i << " error: " << TransformDistance(twrist, twrist_out, frotweight, ftransweight) << endl
                  << "Original Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Returned Joint Val: ";
                FOREACH(it, *itsol)
                    s << *it << " ";
                s << endl << "Transform in: " << twrist << endl;
                s << "Transform out: " << twrist_out << endl << endl;
                RAVELOG_WARNA(s.str().c_str());
                bfail = true;
                break;
            }
        }

        if( bfail ) {
            ++i;
            continue;
        }

        if( pmanip->GetNumFreeParameters() == 0 ) {
            success++;
            i++;
            continue;
        }

        // test with the free parameters
        robot->SetActiveDOFValues(NULL,&vrand[0], true);
        if( !DebugIKFindSolution(pmanip, twrist, viksolution, true, &vfreeparams[0], vfreeparams.size()-1) ) {
                s.str("");
                s << "FindIKSolution (freeparams): No ik solution found, i = " << i << endl << "Joint Val: ";
                for(size_t j = 0; j < vjoints.size(); j++)
                    s << vjoints[j] << " ";
                s << endl << "Transform: " << twrist << endl << endl;
                RAVELOG_WARNA(s.str().c_str());
                ++i;
                continue;
            }

        robot->SetActiveDOFValues(NULL,&viksolution[0], true);
        twrist_out = pmanip->GetEndEffectorTransform();
        
        if(TransformDistance(twrist, twrist_out, frotweight, ftransweight) > 0.05f) {
            s.str("");
            s << "FindIKSolution (freeparams): Incorrect IK, i = " << i << " error: " << TransformDistance(twrist, twrist_out, frotweight, ftransweight) << endl
              << "freeparams: ";
            FOREACH(it, vfreeparams)
                s << *it << " ";
            s << endl << "Original Joint Val: ";
            FOREACH(it, vjoints)
                s << *it << " ";
            s << endl << "Returned Joint Val: ";
            FOREACH(it, viksolution)
                s << *it << " ";
            s << endl << "Transform in: " << twrist << endl;
            s << "Transform out: " << twrist_out << endl << endl;
            RAVELOG_WARNA(s.str().c_str());
            ++i;
            continue;
        }

        if( !pmanip->GetFreeParameters(&vfreeparams2[0]) ) {
            RAVELOG_ERRORA("failed to get free parameters\n");
            ++i;
            continue;
        }

        // make sure they are the same
        for(int j = 0; j < pmanip->GetNumFreeParameters(); ++j) {
            if( fabsf(vfreeparams[j]-vfreeparams2[j]) > 0.01f ) {
                RAVELOG_WARNA("free params %d not equal: %f!=%f\n", j, vfreeparams[j], vfreeparams2[j]);
                pmanip->GetFreeParameters(&vfreeparams2[0]);
                ++i;
                continue;
            }
        }
        
        // test the multiple solution function
        robot->SetActiveDOFValues(NULL,&vrand[0], true);
        viksolution.resize(0);
        DebugIKFindSolutions(pmanip, twrist, viksolutions, true, &vfreeparams[0], vfreeparams.size()-1);

        if( viksolutions.size() == 0 ) {
            s.str("");
            s << "FindIKSolutions (freeparams): No ik solution found for, i = " << i << endl << "Joint Val: ";
            for(size_t j = 0; j < vjoints.size(); j++)
                s << vjoints[j] << " ";
            s << endl << "Transform: " << twrist << endl << endl;
            RAVELOG_WARNA(s.str().c_str());
            ++i;
            continue;
        }

        bfail = false;
        FOREACH(itsol, viksolutions) {
            robot->SetActiveDOFValues(NULL,&(*itsol)[0], true);
            twrist_out = pmanip->GetEndEffectorTransform();
            if(TransformDistance(twrist, twrist_out, frotweight, ftransweight) > 0.05f) {
                s.str("");
                s << "FindIKSolutions (freeparams): Incorrect IK, i = " << i <<" error: " << TransformDistance(twrist, twrist_out, frotweight, ftransweight) << endl
                  << "Original Joint Val: ";
                FOREACH(it, vjoints)
                    s << *it << " ";
                s << endl << "Returned Joint Val: ";
                FOREACH(it, *itsol)
                    s << *it << " ";
                s << endl << "Transform in: " << twrist << endl;
                s << "Transform out: " << twrist_out << endl << endl;
                RAVELOG_WARNA(s.str().c_str());
                bfail = true;
                break;
            }
        }

        if( bfail ) {
            ++i;
            continue;
        }

        success++;
        i++;
    }

    RAVELOG_INFOA("DebugIK done, success rate %f.\n", (float)success/(float)num_itrs);
    sout << (float)success/(float)num_itrs;
    return true;
}

bool BaseManipulationProblem::SetTrajectory(Trajectory* pActiveTraj, bool bExecute, const string& strsavetraj, ostream* pout)
{
    assert( pActiveTraj != NULL );
    if( pActiveTraj->GetPoints().size() == 0 )
        return false;

    pActiveTraj->CalcTrajTiming(robot, pActiveTraj->GetInterpMethod(), true, true);

    bool bExecuted = false;
    if( bExecute ) {
        if( pActiveTraj->GetPoints().size() > 1 ) {
            robot->SetActiveMotion(pActiveTraj);
            bExecute = true;
        }
        // have to set anyway since calling script will orEnvWait!
        else if( robot->GetController() != NULL ) {
            boost::shared_ptr<Trajectory> pfulltraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
            robot->GetFullTrajectoryFromActive(pfulltraj.get(), pActiveTraj);

            if( robot->GetController()->SetDesired(&pfulltraj->GetPoints()[0].q[0]))
                bExecuted = true;
        }
    }

    if( strsavetraj.size() || pout != NULL ) {
        boost::shared_ptr<Trajectory> pfulltraj(GetEnv()->CreateTrajectory(robot->GetDOF()));
        robot->GetFullTrajectoryFromActive(pfulltraj.get(), pActiveTraj);

        if( strsavetraj.size() > 0 )
            pfulltraj->Write(strsavetraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);

        if( pout != NULL )
            pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
    }
    
    return bExecuted;
}

bool BaseManipulationProblem::Help(ostream& sout, istream& sinput)
{
    sout << "----------------------------------" << endl
         << "BaseManipulation Problem Commands:" << endl;
    GetCommandHelp(sout);
    sout << "----------------------------------" << endl;
    return true;
}
