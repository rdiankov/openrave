// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "commonmanipulation.h"

#include <boost/algorithm/string.hpp>

class BaseManipulation : public ModuleBase
{
public:
    BaseManipulation(EnvironmentBasePtr penv) : ModuleBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nVery useful routines for manipulation planning and planning in general. The planners use analytical inverse kinematics and search based techniques. Most of the MoveX commands by default execute the plan on the current robot by calling :meth:`.RobotBase.GetController().SetPath`. This can be disabled by adding 'execute 0' to the command line";
        RegisterCommand("Traj",boost::bind(&BaseManipulation::Traj,this,_1,_2),
                        "Execute a trajectory from a file on the local filesystem");
        RegisterCommand("GrabBody",boost::bind(&BaseManipulation::GrabBody,this,_1,_2),
                        "Robot calls ::Grab on a body with its current manipulator");
        RegisterCommand("ReleaseAll",boost::bind(&BaseManipulation::ReleaseAll,this,_1,_2),
                        "Releases all grabbed bodies (RobotBase::ReleaseAllGrabbed).");
        RegisterCommand("MoveHandStraight",boost::bind(&BaseManipulation::MoveHandStraight,this,_1,_2),
                        "Move the active end-effector in a straight line until collision or IK fails. Parameters:\n\n\
- steplength - the increments in workspace in which the robot tests for the next configuration.\n\n\
- minsteps - The minimum number of steps that need to be taken in order for success to declared. If robot doesn't reach this number of steps, it fails.\n\n\
- maxsteps - The maximum number of steps the robot should take.\n\n\
- direction - The workspace direction to move end effector in.\n\n\
Method wraps the WorkspaceTrajectoryTracker planner. For more details on parameters, check out its documentation.");
        RegisterCommand("MoveManipulator",boost::bind(&BaseManipulation::MoveManipulator,this,_1,_2),
                        "Moves arm joints of active manipulator to a given set of joint values");
        RegisterCommand("MoveActiveJoints",boost::bind(&BaseManipulation::MoveActiveJoints,this,_1,_2),
                        "Moves the current active joints to a specified goal destination:\n\n\
- maxiter - The maximum number of iterations on the internal planner.\n\
- maxtries - The maximum number of times to restart the planner.\n\
- steplength - See PlannerParameters::_fStepLength\n\n");
        RegisterCommand("MoveToHandPosition",boost::bind(&BaseManipulation::_MoveToHandPosition,this,_1,_2),
                        "Move the manipulator's end effector to reach a set of 6D poses. Parameters:\n\n\
- ");
        RegisterCommand("MoveUnsyncJoints",boost::bind(&BaseManipulation::MoveUnsyncJoints,this,_1,_2),
                        "Moves the active joints to a position where the inactive (hand) joints can\n"
                        "fully move to their goal. This is necessary because synchronization with arm\n"
                        "and hand isn't guaranteed.\n"
                        "Options: handjoints savetraj planner");
        RegisterCommand("JitterActive",boost::bind(&BaseManipulation::JitterActive,this,_1,_2),
                        "Jitters the active DOF for a collision-free position.");
        RegisterCommand("SetMinimumGoalPaths",boost::bind(&BaseManipulation::SetMinimumGoalPathsCommand,this,_1,_2),
                        "Sets _minimumgoalpaths for all planner parameters.");
        RegisterCommand("SetPostProcessing",boost::bind(&BaseManipulation::SetPostProcessingCommand,this,_1,_2),
                        "Sets post processing parameters.");
        RegisterCommand("SetRobot",boost::bind(&BaseManipulation::SetRobotCommand,this,_1,_2),
                        "Sets the robot.");
        _minimumgoalpaths=1;
    }

    virtual ~BaseManipulation() {
    }

    virtual void Destroy()
    {
        robot.reset();
        ModuleBase::Destroy();
    }

    virtual void Reset()
    {
        ModuleBase::Reset();
    }

    virtual int main(const std::string& args)
    {
        _fMaxVelMult=1;
        _minimumgoalpaths=1;
        string strRobotName;
        stringstream ss(args);
        ss >> strRobotName;
        robot = GetEnv()->GetRobot(strRobotName);

        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "planner" ) {
                ss >> _strRRTPlannerName;
            }
            else if( cmd == "maxvelmult" ) {
                ss >> _fMaxVelMult;
            }
            if( ss.fail() || !ss ) {
                break;
            }
        }

        PlannerBasePtr planner;
        if( _strRRTPlannerName.size() > 0 ) {
            planner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        }
        if( !planner ) {
            _strRRTPlannerName = "BiRRT";
            planner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
            if( !planner ) {
                _strRRTPlannerName = "";
            }
        }

        RAVELOG_DEBUG(str(boost::format("BaseManipulation: using %s planner\n")%_strRRTPlannerName));
        return 0;
    }

    virtual bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return ModuleBase::SendCommand(sout,sinput);
    }
protected:

    inline boost::shared_ptr<BaseManipulation> shared_problem() {
        return boost::dynamic_pointer_cast<BaseManipulation>(shared_from_this());
    }
    inline boost::shared_ptr<BaseManipulation const> shared_problem_const() const {
        return boost::dynamic_pointer_cast<BaseManipulation const>(shared_from_this());
    }

    bool Traj(ostream& sout, istream& sinput)
    {
        string filename; sinput >> filename;
        if( !sinput ) {
            return false;
        }
        RAVELOG_WARN("BaseManipulation Traj command is deprecated, use robot.GetController().SetPath(traj) instead\n");
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        char sep = ' ';
        if( filename == "sep" ) {
            sinput >> sep;
            filename = utils::GetFilenameUntilSeparator(sinput,sep);
        }

        if( filename == "stream" ) {
            // the trajectory is embedded in the stream
            RAVELOG_VERBOSE("BaseManipulation: reading trajectory from stream\n");
            ptraj->deserialize(sinput);
        }
        else {
            RAVELOG_VERBOSE(str(boost::format("BaseManipulation: reading trajectory: %s\n")%filename));
            ifstream f(filename.c_str());
            ptraj->deserialize(sinput);
        }

        bool bResetTrans = false; sinput >> bResetTrans;
        bool bResetTiming = false; sinput >> bResetTiming;

        if( bResetTrans ) {
            RAVELOG_WARN("resetting transformations of trajectory not supported\n");
        }

        if(( ptraj->GetDuration() == 0) || bResetTiming ) {
            RAVELOG_VERBOSE(str(boost::format("retiming trajectory: %f\n")%_fMaxVelMult));
            planningutils::SmoothActiveDOFTrajectory(ptraj, robot, _fMaxVelMult);
        }
        RAVELOG_VERBOSE(str(boost::format("executing traj with %d points\n")%ptraj->GetNumWaypoints()));
        if( !robot->GetController()->SetPath(ptraj) ) {
            return false;
        }
        sout << "1";
        return true;
    }

    bool SetRobotCommand(ostream& sout, istream& sinput)
    {
        string strRobotName;
        sinput >> strRobotName;
        robot = GetEnv()->GetRobot(strRobotName);
        return !!robot;
    }

    bool MoveHandStraight(ostream& sout, istream& sinput)
    {
        Vector direction = Vector(0,0,1);
        string strtrajfilename;
        bool bExecute = true;
        int minsteps = 0;
        int maxsteps = 10000;
        bool starteematrix = false;

        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        Transform Tee;

        WorkspaceTrajectoryParametersPtr params(new WorkspaceTrajectoryParameters(GetEnv()));
        boost::shared_ptr<ostream> pOutputTrajStream;
        params->ignorefirstcollision = 0.1;     // 0.1**2 * 5 * 0.5 = 0.025 m
        string plannername = "workspacetrajectorytracker";
        params->_fStepLength = 0.01;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "minsteps" ) {
                sinput >> minsteps;
            }
            else if( cmd == "outputtraj") {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "maxsteps") {
                sinput >> maxsteps;
            }
            else if((cmd == "steplength")||(cmd == "stepsize")) {
                sinput >> params->_fStepLength;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj") {
                sinput >> strtrajfilename;
            }
            else if( cmd == "direction") {
                sinput >> direction.x >> direction.y >> direction.z;
                direction.normalize3();
            }
            else if( cmd == "ignorefirstcollision") {
                sinput >> params->ignorefirstcollision;
            }
            else if( cmd == "greedysearch" ) {
                sinput >> params->greedysearch;
            }
            else if( cmd == "maxdeviationangle" ) {
                sinput >> params->maxdeviationangle;
            }
            else if( cmd == "jacobian" ) {
                RAVELOG_WARN("MoveHandStraight jacobian parameter not supported anymore\n");
            }
            else if( cmd == "planner" ) {
                sinput >> plannername;
            }
            else if( cmd == "starteematrix" ) {
                TransformMatrix tm;
                starteematrix = true;
                sinput >> tm;
                Tee = tm;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( maxsteps <= 0 ) {
            RAVELOG_WARN("maxsteps is not positive\n");
            return false;
        }

        params->minimumcompletetime = params->_fStepLength * minsteps;
        RAVELOG_DEBUG(str(boost::format("Starting MoveHandStraight dir=(%f,%f,%f)...")%direction.x%direction.y%direction.z));
        robot->RegrabAll();

        RobotBase::RobotStateSaver saver(robot);

        robot->SetActiveDOFs(pmanip->GetArmIndices());
        params->SetRobotActiveJoints(robot);

        if( !starteematrix ) {
            planningutils::JitterActiveDOF(robot,100);     // try to jitter out, don't worry if it fails
            robot->GetActiveDOFValues(params->vinitialconfig);
            Tee = pmanip->GetTransform();
        }
        else {
            params->vinitialconfig.resize(0);     // set by SetRobotActiveJoints
        }

        // compute a workspace trajectory (important to do this after jittering!)
        {
            params->workspacetraj = RaveCreateTrajectory(GetEnv(),"");
            ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D,"quadratic");
            params->workspacetraj->Init(spec);
            vector<dReal> data(spec._vgroups[0].dof);
            IkParameterization ikparam(Tee,IKP_Transform6D);
            ikparam.GetValues(data.begin());
            params->workspacetraj->Insert(0,data);
            Tee.trans += direction*maxsteps*params->_fStepLength;
            ikparam.SetTransform6D(Tee);
            ikparam.GetValues(data.begin());
            params->workspacetraj->Insert(1,data);
            vector<dReal> maxvelocities(spec._vgroups[0].dof,1);
            vector<dReal> maxaccelerations(spec._vgroups[0].dof,5);
            planningutils::RetimeAffineTrajectory(params->workspacetraj,maxvelocities,maxaccelerations);
        }

        if( params->workspacetraj->GetDuration() <= 10*g_fEpsilon ) {
            RAVELOG_WARN("workspace traj is empty\n");
            return false;
        }

        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),plannername);
        if( !planner ) {
            RAVELOG_WARN("failed to create planner\n");
            return false;
        }

        if( !planner->InitPlan(robot, params) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }

        TrajectoryBasePtr poutputtraj = RaveCreateTrajectory(GetEnv(),"");
        if( !planner->PlanPath(poutputtraj) ) {
            return false;
        }
        if( params->ignorefirstcollision == 0 && (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            planningutils::VerifyTrajectory(params,poutputtraj);
        }
        CM::SetActiveTrajectory(robot, poutputtraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool MoveManipulator(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting MoveManipulator...\n");
        RobotBase::RobotStateSaver saver(robot,KinBody::Save_ActiveDOF);
        robot->SetActiveDOFs(robot->GetActiveManipulator()->GetArmIndices());
        BOOST_ASSERT(robot->GetActiveDOF()>0);
        return MoveActiveJoints(sout,sinput);
    }

    bool MoveActiveJoints(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        int nMaxTries = 2;     // max tries for the planner
        boost::shared_ptr<ostream> pOutputTrajStream;

        int nMaxJitterIterations = 1000;
        RRTParametersPtr params(new RRTParameters());
        params->_minimumgoalpaths = _minimumgoalpaths;
        params->_nMaxIterations = 4000;     // max iterations before failure
        dReal jitter = 0.04;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "goal" ) {
                params->vgoalconfig.reserve(params->vgoalconfig.size()+robot->GetActiveDOF());
                for(int i = 0; i < robot->GetActiveDOF(); ++i) {
                    dReal f=0;
                    sinput >> f;
                    params->vgoalconfig.push_back(f);
                }
            }
            else if( cmd == "jitter" ) {
                sinput >> jitter;
            }
            else if( cmd == "goals" ) {
                size_t numgoals = 0;
                sinput >> numgoals;
                params->vgoalconfig.reserve(params->vgoalconfig.size()+numgoals*robot->GetActiveDOF());
                for(size_t i = 0; i < numgoals*robot->GetActiveDOF(); ++i) {
                    dReal f=0;
                    sinput >> f;
                    params->vgoalconfig.push_back(f);
                }
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "maxiter" ) {
                sinput >> params->_nMaxIterations;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "steplength" ) {
                sinput >> params->_fStepLength;
            }
            else if( cmd == "maxtries" ) {
                sinput >> nMaxTries;
            }
            else if( cmd == "postprocessingparameters" ) {
                if( !getline(sinput, params->_sPostProcessingParameters) ) {
                    return false;
                }
                boost::trim(params->_sPostProcessingParameters);
            }
            else if( cmd == "postprocessingplanner" ) {
                if( !getline(sinput, params->_sPostProcessingPlanner) ) {
                    return false;
                }
                boost::trim(params->_sPostProcessingPlanner);
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( params->vgoalconfig.size() == 0 ) {
            return false;
        }
        RobotBase::RobotStateSaver saver(robot);
        params->SetRobotActiveJoints(robot);
        if( _sPostProcessingParameters.size() > 0 ) {
            params->_sPostProcessingParameters = _sPostProcessingParameters;
        }

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        // make sure the initial and goal configs are not in collision
        vector<dReal> vonegoal(robot->GetActiveDOF());
        vector<dReal> voriggoalconfig; voriggoalconfig.reserve(params->vgoalconfig.size());
        std::vector<uint8_t> vgoalchanged; vgoalchanged.reserve(params->vgoalconfig.size()/robot->GetActiveDOF());

        size_t writeindex=0;
        for(size_t i = 0; i < params->vgoalconfig.size(); i += robot->GetActiveDOF()) {
            std::copy(params->vgoalconfig.begin()+i,params->vgoalconfig.begin()+i+robot->GetActiveDOF(),vonegoal.begin());
            robot->SetActiveDOFValues(vonegoal, true);
            robot->GetActiveDOFValues(vonegoal);
            uint8_t goalchanged = 0;
            switch( planningutils::JitterActiveDOF(robot,nMaxJitterIterations,jitter) ) {
            case 0:
                RAVELOG_WARN(str(boost::format("jitter failed %d\n")%i));
                continue;
            case 1:
                goalchanged = 1;
                break;
            }
            voriggoalconfig.insert(voriggoalconfig.end(),vonegoal.begin(),vonegoal.end());
            if( goalchanged ) {
                robot->GetActiveDOFValues(vonegoal);
            }
            vgoalchanged.push_back(goalchanged);
            std::copy(vonegoal.begin(),vonegoal.end(),params->vgoalconfig.begin()+writeindex);
            writeindex += robot->GetActiveDOF();
        }
        if( writeindex == 0 ) {
            return false;
        }
        params->vgoalconfig.resize(writeindex);
        robot->SetActiveDOFValues(params->vinitialconfig);

        vector<dReal> vinsertconfiguration; // configuration to add at the beginning of the trajectory, usually it is in collision
        // jitter again for initial collision
        switch( planningutils::JitterActiveDOF(robot,nMaxJitterIterations,jitter) ) {
        case 0:
            RAVELOG_WARN("jitter failed for initial\n");
            return false;
        case 1:
            RAVELOG_DEBUG("original robot position in collision, so jittered out of it\n");
            vinsertconfiguration = params->vinitialconfig;
            robot->GetActiveDOFValues(params->vinitialconfig);
            break;
        }

        PlannerBasePtr rrtplanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( !rrtplanner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");

        RAVELOG_DEBUG("starting planning\n");
        bool bSuccess = false;
        for(int itry = 0; itry < nMaxTries; ++itry) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERROR("InitPlan failed\n");
                return false;
            }

            if( !rrtplanner->PlanPath(ptraj) ) {
                RAVELOG_WARN("PlanPath failed\n");
            }
            else {
                bSuccess = true;
                RAVELOG_DEBUG("finished planning\n");
                break;
            }
        }

        if( !bSuccess ) {
            return false;
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params, ptraj);
        }
        if( vinsertconfiguration.size() > 0 ) {
            planningutils::InsertActiveDOFWaypointWithRetiming(0,vinsertconfiguration,vector<dReal>(), ptraj, robot, _fMaxVelMult);
        }

        if( jitter > 0 ) {
            // could have returned a jittered goal different from the original goals
            try {
                stringstream soutput, sinput;
                sinput << "GetGoalIndex";
                rrtplanner->SendCommand(soutput,sinput);

                int goalindex = -1;
                soutput >> goalindex;
                if( goalindex >= 0 && goalindex < (int)vgoalchanged.size() && vgoalchanged.at(goalindex) ) {
                    std::copy(voriggoalconfig.begin()+goalindex*robot->GetActiveDOF(),voriggoalconfig.begin()+(goalindex+1)*robot->GetActiveDOF(),vonegoal.begin());
                    planningutils::InsertActiveDOFWaypointWithRetiming(ptraj->GetNumWaypoints(),vonegoal,vector<dReal>(), ptraj, robot, _fMaxVelMult);
                }
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN(str(boost::format("planner %s does not support GetGoalIndex command necessary for determining what goal it chose! %s")%rrtplanner->GetXMLId()%ex.what()));
            }
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool _MoveToHandPosition(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting MoveToHandPosition...\n");
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        std::list<IkParameterization> listgoals;

        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        Vector vconstraintaxis, vconstraintpos;
        int affinedofs = 0;
        int nSeedIkSolutions = 8;     // no extra solutions
        int nMaxTries = 3;     // max tries for the planner

        RRTParametersPtr params(new RRTParameters());
        params->_minimumgoalpaths = _minimumgoalpaths;
        params->_nMaxIterations = 4000;
        int nMaxJitterIterations = 1000;
        // constraint stuff
        boost::array<double,6> vconstraintfreedoms = { { 0,0,0,0,0,0}};
        Transform tConstraintTargetWorldFrame, tConstraintTaskFrame;
        double constrainterrorthresh=0;
        int goalsamples = 40;
        string cmd;
        dReal jitter = 0.03;
        dReal jitterikparam = 0;
        dReal goalsampleprob = 0.1;
        int nGoalMaxTries=10;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "translation" ) {
                Vector trans;
                sinput >> trans.x >> trans.y >> trans.z;
                listgoals.push_back(IkParameterization());
                listgoals.back().SetTranslation3D(trans);
            }
            else if( cmd == "rotation" ) {
                Vector q;
                sinput >> q.x >> q.y >> q.z >> q.w;
                listgoals.push_back(IkParameterization());
                listgoals.back().SetRotation3D(q);
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "matrix" ) {
                TransformMatrix m;
                sinput >> m;
                listgoals.push_back(IkParameterization(Transform(m)));
            }
            else if( cmd == "goalsamples" ) {
                sinput >> goalsamples;
            }
            else if( cmd == "matrices" ) {
                TransformMatrix m;
                int num = 0;
                sinput >> num;
                while(num-->0) {
                    sinput >> m;
                    listgoals.push_back(IkParameterization(Transform(m)));
                }
            }
            else if( cmd == "pose" ) {
                Transform t;
                sinput >> t;
                listgoals.push_back(IkParameterization(t));
            }
            else if( cmd == "poses" ) {
                int num = 0;
                sinput >> num;
                while(num-->0) {
                    Transform t;
                    sinput >> t;
                    listgoals.push_back(IkParameterization(t));
                }
            }
            else if( cmd == "ikparams" ) {
                int num = 0;
                sinput >> num;
                while(num-->0) {
                    IkParameterization ikparam;
                    sinput >> ikparam;
                    listgoals.push_back(ikparam);
                }
            }
            else if( cmd == "ikparam" ) {
                IkParameterization ikparam;
                sinput >> ikparam;
                listgoals.push_back(ikparam);
            }
            else if( cmd == "affinedofs" ) {
                sinput >> affinedofs;
            }
            else if( cmd == "maxiter" ) {
                sinput >> params->_nMaxIterations;
            }
            else if( cmd == "maxtries" ) {
                sinput >> nMaxTries;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "seedik" ) {
                sinput >> nSeedIkSolutions;
            }
            else if( cmd == "steplength" ) {
                sinput >> params->_fStepLength;
            }
            else if( cmd == "constraintfreedoms" ) {
                FOREACH(it,vconstraintfreedoms) {
                    sinput >> *it;
                }
            }
            else if( cmd == "constraintmatrix" ) {
                TransformMatrix m; sinput >> m; tConstraintTargetWorldFrame = m;
            }
            else if( cmd == "constraintpose" ) {
                sinput >> tConstraintTargetWorldFrame;
            }
            else if( cmd == "constrainttaskmatrix" ) {
                TransformMatrix m; sinput >> m; tConstraintTaskFrame = m;
            }
            else if( cmd == "constrainttaskpose" ) {
                sinput >> tConstraintTaskFrame;
            }
            else if( cmd == "constrainterrorthresh" ) {
                sinput >> constrainterrorthresh;
            }
            else if( cmd == "minimumgoalpaths" ) {
                sinput >> params->_minimumgoalpaths;
            }
            else if( cmd == "postprocessingparameters" ) {
                if( !getline(sinput, params->_sPostProcessingParameters) ) {
                    return false;
                }
                boost::trim(params->_sPostProcessingParameters);
            }
            else if( cmd == "postprocessingplanner" ) {
                if( !getline(sinput, params->_sPostProcessingPlanner) ) {
                    return false;
                }
                boost::trim(params->_sPostProcessingPlanner);
            }
            else if( cmd == "jitter" ) {
                sinput >> jitter;
            }
            else if( cmd == "jitterikparam" || cmd == "jittergoal" ) {
                sinput >> jitterikparam;
            }
            else if( cmd == "goalsampleprob" ) {
                sinput >> goalsampleprob;
            }
            else if( cmd == "goalmaxtries" ) {
                sinput >> nGoalMaxTries;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        robot->RegrabAll();
        RobotBase::RobotStateSaver saver(robot);
        robot->SetActiveDOFs(pmanip->GetArmIndices(), affinedofs);
        params->SetRobotActiveJoints(robot);
        if( _sPostProcessingParameters.size() > 0 ) {
            params->_sPostProcessingParameters = _sPostProcessingParameters;
        }

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        if( constrainterrorthresh > 0 ) {
            RAVELOG_DEBUG("setting jacobian constraint function in planner parameters\n");
            robot->SetActiveDOFValues(params->vinitialconfig); // have to set the initial configuraiton!
            boost::shared_ptr<CM::GripperJacobianConstrains<double> > pconstraints(new CM::GripperJacobianConstrains<double>(robot->GetActiveManipulator(),tConstraintTargetWorldFrame,tConstraintTaskFrame, vconstraintfreedoms,constrainterrorthresh));
            pconstraints->_distmetricfn = params->_distmetricfn;
            params->_neighstatefn = boost::bind(&CM::GripperJacobianConstrains<double>::RetractionConstraint,pconstraints,_1,_2);
            // use linear interpolation!
            params->_sPostProcessingPlanner = "shortcut_linear";
            params->_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
            vector<dReal> vdelta(params->vinitialconfig.size(),0);
            if( !params->_neighstatefn(params->vinitialconfig,vdelta,0)) {
                throw OPENRAVE_EXCEPTION_FORMAT0("initial configuration does not follow constraints",ORE_InconsistentConstraints);
            }
        }

        robot->SetActiveDOFs(pmanip->GetArmIndices(), 0);

        vector<dReal> vgoal;
        planningutils::ManipulatorIKGoalSampler goalsampler(pmanip, listgoals,goalsamples,nGoalMaxTries);
        goalsampler.SetJitter(jitterikparam);
        params->vgoalconfig.reserve(nSeedIkSolutions*robot->GetActiveDOF());
        while(nSeedIkSolutions > 0) {
            if( goalsampler.Sample(vgoal) ) {
                if(constrainterrorthresh > 0 ) {
                    robot->SetActiveDOFValues(vgoal);
                    switch( planningutils::JitterActiveDOF(robot,nMaxJitterIterations,jitter,params->_neighstatefn) ) {
                    case 0:
                        RAVELOG_DEBUG("constraint function failed\n");
                        continue;
                    case 1:
                        robot->GetActiveDOFValues(vgoal);
                        break;
                    }
                }
                params->vgoalconfig.insert(params->vgoalconfig.end(), vgoal.begin(), vgoal.end());
                --nSeedIkSolutions;
            }
            else {
                --nSeedIkSolutions;
            }
        }
        goalsampler.SetSamplingProb(goalsampleprob);
        params->_samplegoalfn = boost::bind(&planningutils::ManipulatorIKGoalSampler::Sample,&goalsampler,_1);

        if( params->vgoalconfig.size() == 0 ) {
            RAVELOG_WARN("failed to find goal\n");
            return false;
        }

        // restore
        robot->SetActiveDOFValues(params->vinitialconfig);

        vector<dReal> vinsertconfiguration; // configuration to add at the beginning of the trajectory, usually it is in collision
        // jitter again for initial collision
        switch( planningutils::JitterActiveDOF(robot,nMaxJitterIterations,jitter,constrainterrorthresh > 0 ? params->_neighstatefn : PlannerBase::PlannerParameters::NeighStateFn()) ) {
        case 0:
            RAVELOG_WARN("jitter failed for initial\n");
            return false;
        case 1:
            RAVELOG_DEBUG("original robot position in collision, so jittered out of it\n");
            vinsertconfiguration = params->vinitialconfig;
            robot->GetActiveDOFValues(params->vinitialconfig);
            break;
        }

        PlannerBasePtr rrtplanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( !rrtplanner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }

        bool bSuccess = false;
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        RAVELOG_DEBUG("starting planning\n");

        for(int iter = 0; iter < nMaxTries; ++iter) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERROR("InitPlan failed\n");
                return false;
            }

            if( rrtplanner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_DEBUG("finished planning\n");
                break;
            }
            else {
                RAVELOG_WARN("PlanPath failed\n");
            }
        }

        if( !bSuccess ) {
            return false;
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params, ptraj);
        }

        if( vinsertconfiguration.size() > 0 ) {
            planningutils::InsertActiveDOFWaypointWithRetiming(0,vinsertconfiguration,vector<dReal>(), ptraj, robot, _fMaxVelMult);
        }

        if( jitterikparam > 0 ) {
            // could have returned a jittered goal different from the original goals
            try {
                stringstream soutput, sinput;
                sinput << "GetGoalIndex";
                rrtplanner->SendCommand(soutput,sinput);
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN(str(boost::format("planner %s does not support GetGoalIndex command necessary for determining what goal it chose! %s")%rrtplanner->GetXMLId()%ex.what()));
            }
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        sout << "1";
        return true;
    }

    bool MoveUnsyncJoints(ostream& sout, istream& sinput)
    {
        string strplanner = "BasicRRT";
        string strsavetraj;
        string cmd;
        vector<int> vhandjoints;
        vector<dReal> vhandgoal;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;
        int nMaxTries=1;
        int maxdivision=10;
        int nMaxJitterIterations = 1000;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "writetraj" ) {
                sinput >> strsavetraj;
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "handjoints" ) {
                int dof = 0;
                sinput >> dof;
                if( !sinput ||( dof == 0) ) {
                    return false;
                }
                vhandjoints.resize(dof);
                vhandgoal.resize(dof);
                FOREACH(it, vhandgoal) {
                    sinput >> *it;
                }
                FOREACH(it, vhandjoints) {
                    sinput >> *it;
                }
            }
            else if( cmd == "planner" ) {
                sinput >> strplanner;
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "maxtries" ) {
                sinput >> nMaxTries;
            }
            else if( cmd == "maxdivision" ) {
                sinput >> maxdivision;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        uint32_t starttime = utils::GetMilliTime();
        if( planningutils::JitterActiveDOF(robot,nMaxJitterIterations) == 0 ) {
            RAVELOG_WARN("failed to jitter robot out of collision\n");
        }

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");

        bool bSuccess = false;
        for(int itry = 0; itry < nMaxTries; ++itry) {
            if( CM::MoveUnsync::_MoveUnsyncJoints(GetEnv(), robot, ptraj, vhandjoints, vhandgoal, strplanner,maxdivision) ) {
                bSuccess = true;
                break;
            }
        }
        if( !bSuccess ) {
            return false;
        }

        BOOST_ASSERT(ptraj->GetNumWaypoints() > 0);

        bool bExecuted = CM::SetActiveTrajectory(robot, ptraj, bExecute, strsavetraj, pOutputTrajStream,_fMaxVelMult);
        sout << (int)bExecuted << " ";
        sout << (utils::GetMilliTime()-starttime)/1000.0f << " ";
        vector<dReal> q;
        ptraj->GetWaypoint(-1,q,robot->GetActiveConfigurationSpecification());
        FOREACH(it, q) {
            sout << *it << " ";
        }
        return true;
    }

    bool JitterActive(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting JitterActive...\n");
        bool bExecute = true, bOutputFinal=false;
        boost::shared_ptr<ostream> pOutputTrajStream;
        string cmd;
        int nMaxJitterIterations=5000;
        dReal fJitter=0.03f;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "maxiter" ) {
                sinput >> nMaxJitterIterations;
            }
            else if( cmd == "jitter" ) {
                sinput >> fJitter;
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "outputfinal" ) {
                bOutputFinal = true;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(robot->GetActiveConfigurationSpecification());

        vector<dReal> vvalues;
        robot->GetActiveDOFValues(vvalues);
        ptraj->Insert(0,vvalues);
        switch( planningutils::JitterActiveDOF(robot,nMaxJitterIterations,fJitter) ) {
        case 0:
            RAVELOG_WARN("could not jitter out of collision\n");
            return false;
        case 1:
            robot->GetActiveDOFValues(vvalues);
            ptraj->Insert(1,vvalues);
        default:
            break;
        }

        if( bOutputFinal ) {
            FOREACH(itq,vvalues) {
                sout << *itq << " ";
            }
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, "", pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool GrabBody(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("BaseManipulation GrabBody command is deprecated. Use Robot::Grab (11/03/07)\n");
        KinBodyPtr ptarget;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "name" ) {
                string name;
                sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else {
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if(!ptarget) {
            RAVELOG_ERROR("ERROR Manipulation::GrabBody - Invalid body name.\n");
            return false;
        }

        RAVELOG_DEBUG(str(boost::format("robot %s:%s grabbing body %s...\n")%robot->GetName()%robot->GetActiveManipulator()->GetEndEffector()->GetName()%ptarget->GetName()));
        robot->Grab(ptarget);
        return true;
    }

    bool ReleaseAll(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("BaseManipulation ReleaseAll command is deprecated. Use Robot::ReleaseAllGrabbed (11/03/07)\n");
        if( !!robot ) {
            RAVELOG_DEBUG("Releasing all bodies\n");
            robot->ReleaseAllGrabbed();
        }
        return true;
    }

protected:
    bool SetMinimumGoalPathsCommand(ostream& sout, istream& sinput)
    {
        sinput >> _minimumgoalpaths;
        BOOST_ASSERT(_minimumgoalpaths>=0);
        return !!sinput;
    }

    bool SetPostProcessingCommand(ostream& sout, istream& sinput)
    {
        if( !getline(sinput, _sPostProcessingParameters) ) {
            return false;
        }
        return !!sinput;
    }

    RobotBasePtr robot;
    string _strRRTPlannerName;
    dReal _fMaxVelMult;
    int _minimumgoalpaths;
    string _sPostProcessingParameters;
};

ModuleBasePtr CreateBaseManipulation(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new BaseManipulation(penv));
}
