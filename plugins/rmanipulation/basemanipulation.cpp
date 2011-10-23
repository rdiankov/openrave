// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

class BaseManipulation : public ModuleBase
{
public:
    BaseManipulation(EnvironmentBasePtr penv) : ModuleBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nVery useful routines for manipulation planning and planning in general. The planners use analytical inverse kinematics and search based techniques. Most of the MoveX commands by default execute the plan on the current robot by calling :meth:`.RobotBase.SetActiveMotion`. This can be disabled by adding 'execute 0' to the command line";
        RegisterCommand("SetActiveManip",boost::bind(&BaseManipulation::SetActiveManip,this,_1,_2),
                        "Set the active manipulator");
        RegisterCommand("Traj",boost::bind(&BaseManipulation::Traj,this,_1,_2),
                        "Execute a trajectory from a file on the local filesystem");
        RegisterCommand("VerifyTrajectory",boost::bind(&BaseManipulation::_VerifyTrajectoryCommand,this,_1,_2),
                        "Verifies the robot trajectory by checking collisions with the environment and other user-specified constraints.");
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
        RegisterCommand("FindIKWithFilters",boost::bind(&BaseManipulation::FindIKWithFilters,this,_1,_2),
                        "Samples IK solutions using custom filters that constrain the end effector in the world. Parameters:\n\n\
- cone - Constraint the direction of a local axis with respect to a cone in the world. Takes in: worldaxis(3), localaxis(3), anglelimit. \n\
- solveall - When specified, will return all possible solutions.\n\
- ikparam - The serialized ik parameterization to use for FindIKSolution(s).\n\
- filteroptions\n\
");
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
        return boost::static_pointer_cast<BaseManipulation>(shared_from_this());
    }
    inline boost::shared_ptr<BaseManipulation const> shared_problem_const() const {
        return boost::static_pointer_cast<BaseManipulation const>(shared_from_this());
    }

    bool SetActiveManip(ostream& sout, istream& sinput)
    {
        string manipname;
        int index = -1;

        if(!sinput.eof()) {
            sinput >> manipname;
            if( !sinput ) {
                return false;
            }
            // find the manipulator with the right name
            index = 0;
            FOREACHC(itmanip, robot->GetManipulators()) {
                if( manipname == (*itmanip)->GetName() ) {
                    break;
                }
                ++index;
            }

            if( index >= (int)robot->GetManipulators().size() ) {
                index = atoi(manipname.c_str());
            }
        }

        if(( index >= 0) &&( index < (int)robot->GetManipulators().size()) ) {
            robot->SetActiveManipulator(index);
            return true;
        }

        return false;
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
            filename = getfilename_withseparator(sinput,sep);
        }

        if( filename == "stream" ) {
            // the trajectory is embedded in the stream
            RAVELOG_VERBOSE("BaseManipulation: reading trajectory from stream\n");
            if( !ptraj->Read(sinput, robot) ) {
                RAVELOG_ERROR("BaseManipulation: failed to get trajectory\n");
                return false;
            }
        }
        else {
            RAVELOG_VERBOSE(str(boost::format("BaseManipulation: reading trajectory: %s\n")%filename));
            ifstream f(filename.c_str());
            if( !ptraj->Read(f, robot) ) {
                RAVELOG_ERROR(str(boost::format("BaseManipulation: failed to read trajectory %s\n")%filename));
                return false;
            }
        }

        bool bResetTrans = false; sinput >> bResetTrans;
        bool bResetTiming = false; sinput >> bResetTiming;

        if( bResetTrans ) {
            RAVELOG_WARN("resetting transformations of trajectory not supported\n");
        }

        if(( ptraj->GetDuration() == 0) || bResetTiming ) {
            RAVELOG_VERBOSE(str(boost::format("retiming trajectory: %f\n")%_fMaxVelMult));
            ptraj->CalcTrajTiming(robot,0,true,false,_fMaxVelMult);
        }
        RAVELOG_VERBOSE(str(boost::format("executing traj with %d points\n")%ptraj->GetNumWaypoints()));
        if( ptraj->GetDOF() == robot->GetDOF() ) {
            robot->SetMotion(ptraj);
        }
        else if( ptraj->GetDOF() == robot->GetActiveDOF() ) {
            robot->SetActiveMotion(ptraj);
        }
        else {
            return false;
        }
        sout << "1";
        return true;
    }

    bool MoveHandStraight(ostream& sout, istream& sinput)
    {
        Vector direction = Vector(0,1,0);
        string strtrajfilename;
        bool bExecute = true;
        int minsteps = 0;
        int maxsteps = 10000;
        bool starteematrix = false;

        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        Transform Tee;

        WorkspaceTrajectoryParametersPtr params(new WorkspaceTrajectoryParameters(GetEnv()));
        boost::shared_ptr<ostream> pOutputTrajStream;
        params->ignorefirstcollision = 0.04;     // 0.04m?
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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
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

        params->minimumcompletetime = params->_fStepLength * minsteps;
        RAVELOG_DEBUG("Starting MoveHandStraight dir=(%f,%f,%f)...\n",(float)direction.x, (float)direction.y, (float)direction.z);
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
            ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D);
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
            vector<dReal> maxaccelerations(spec._vgroups[0].dof,10);
            planningutils::RetimeAffineTrajectory(params->workspacetraj,maxvelocities,maxaccelerations);
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
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
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

        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000;     // max iterations before failure

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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
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

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        // make sure the initial and goal configs are not in collision
        std::vector<dReal> originalvalues;
        robot->GetActiveDOFValues(originalvalues);
        vector<dReal> vonegoal(robot->GetActiveDOF());

        size_t writeindex=0;
        for(size_t i = 0; i < params->vgoalconfig.size(); i += robot->GetActiveDOF()) {
            std::copy(params->vgoalconfig.begin()+i,params->vgoalconfig.begin()+i+robot->GetActiveDOF(),vonegoal.begin());
            robot->SetActiveDOFValues(vonegoal, true);
            if( planningutils::JitterActiveDOF(robot) == 0 ) {
                RAVELOG_WARN(str(boost::format("jitter failed %d\n")%i));
            }
            else {
                robot->GetActiveDOFValues(vonegoal);
                std::copy(vonegoal.begin(),vonegoal.end(),params->vgoalconfig.begin()+writeindex);
                writeindex += robot->GetActiveDOF();
            }
        }
        if( writeindex == 0 ) {
            return false;
        }
        params->vgoalconfig.resize(writeindex);
        robot->SetActiveDOFValues(originalvalues);

        // jitter again for initial collision
        if( planningutils::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARN("jitter failed\n");
            return false;
        }
        robot->GetActiveDOFValues(params->vinitialconfig);

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

        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000;

        // constraint stuff
        boost::array<double,6> vconstraintfreedoms = { { 0,0,0,0,0,0}};
        Transform tConstraintTargetWorldFrame;
        double constrainterrorthresh=0;
        int goalsamples = 40;
        string cmd;
        dReal jitter = 0.03;
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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
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
            else if( cmd == "constrainterrorthresh" ) {
                sinput >> constrainterrorthresh;
            }
            else if( cmd == "jitter" ) {
                sinput >> jitter;
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

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        if( constrainterrorthresh > 0 ) {
            RAVELOG_DEBUG("setting jacobian constraint function in planner parameters\n");
            boost::shared_ptr<CM::GripperJacobianConstrains<double> > pconstraints(new CM::GripperJacobianConstrains<double>(robot->GetActiveManipulator(),tConstraintTargetWorldFrame,vconstraintfreedoms,constrainterrorthresh));
            pconstraints->_distmetricfn = params->_distmetricfn;
            params->_neighstatefn = boost::bind(&CM::GripperJacobianConstrains<double>::RetractionConstraint,pconstraints,_1,_2);
            // use linear interpolation!
            params->_sPostProcessingParameters ="<_nmaxiterations>100</_nmaxiterations><_postprocessing planner=\"lineartrajectoryretimer\"></_postprocessing>";
        }

        robot->SetActiveDOFs(pmanip->GetArmIndices(), 0);

        vector<dReal> vgoal;
        planningutils::ManipulatorIKGoalSampler goalsampler(pmanip, listgoals,goalsamples);
        params->vgoalconfig.reserve(nSeedIkSolutions*robot->GetActiveDOF());
        while(nSeedIkSolutions > 0) {
            if( goalsampler.Sample(vgoal) ) {
                if(( constrainterrorthresh > 0) &&( planningutils::JitterActiveDOF(robot,5000,jitter,params->_neighstatefn) == 0) ) {
                    RAVELOG_DEBUG("constraint function failed\n");
                    continue;
                }
                params->vgoalconfig.insert(params->vgoalconfig.end(), vgoal.begin(), vgoal.end());
                --nSeedIkSolutions;
            }
            else {
                --nSeedIkSolutions;
            }
        }
        goalsampler.SetSamplingProb(0.05);
        params->_samplegoalfn = boost::bind(&planningutils::ManipulatorIKGoalSampler::Sample,&goalsampler,_1);

        if( params->vgoalconfig.size() == 0 ) {
            RAVELOG_WARN("jitter failed for goal\n");
            return false;
        }

        // restore
        robot->SetActiveDOFValues(params->vinitialconfig);

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(params->_configurationspecification);
        Trajectory::TPOINT pt;
        pt.q = params->vinitialconfig;
        ptraj->AddPoint(pt);

        // jitter again for initial collision
        if( planningutils::JitterActiveDOF(robot,5000,jitter,params->_neighstatefn) == 0 ) {
            RAVELOG_WARN("jitter failed for initial\n");
            return false;
        }
        robot->GetActiveDOFValues(params->vinitialconfig);

        PlannerBasePtr rrtplanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( !rrtplanner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }

        bool bSuccess = false;
        RAVELOG_INFO("starting planning\n");

        for(int iter = 0; iter < nMaxTries; ++iter) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERROR("InitPlan failed\n");
                return false;
            }

            if( rrtplanner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_INFO("finished planning\n");
                break;
            }
            else {
                RAVELOG_WARN("PlanPath failed\n");
            }
        }

        rrtplanner.reset();     // have to destroy before environment

        if( !bSuccess ) {
            return false;
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params, ptraj);
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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
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
        uint32_t starttime = GetMilliTime();
        if( planningutils::JitterActiveDOF(robot) == 0 ) {
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
        sout << (GetMilliTime()-starttime)/1000.0f << " ";
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
        int nMaxIterations=5000;
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
                sinput >> nMaxIterations;
            }
            else if( cmd == "jitter" ) {
                sinput >> fJitter;
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
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

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch( planningutils::JitterActiveDOF(robot,nMaxIterations,fJitter) ) {
        case 0:
            RAVELOG_WARN("could not jitter out of collision\n");
            return false;
        case 1:
            robot->GetActiveDOFValues(ptfirst.q);
            ptraj->AddPoint(ptfirst);
        default:
            break;
        }

        if( bOutputFinal ) {
            FOREACH(itq,ptfirst.q) {
                sout << *itq << " ";
            }
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, "", pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool FindIKWithFilters(ostream& sout, istream& sinput)
    {
        bool bSolveAll = false;
        IkSolverBase::IkFilterCallbackFn filterfn;
        IkParameterization ikparam;
        int filteroptions = IKFO_CheckEnvCollisions;
        string cmd;
        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
        if( !pmanip->GetIkSolver() ) {
            throw openrave_exception(str(boost::format("FindIKWithFilters: manipulator %s has no ik solver set")%robot->GetActiveManipulator()->GetName()));
        }

        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "cone" ) {
                Vector vlocalaxis, vworldaxis;
                dReal anglelimit;
                sinput >> vlocalaxis.x >> vlocalaxis.y >> vlocalaxis.z >> vworldaxis.x >> vworldaxis.y >> vworldaxis.z >> anglelimit;
                filterfn = boost::bind(&BaseManipulation::_FilterWorldAxisIK,shared_problem(),_1,_2,_3, vlocalaxis, vworldaxis, RaveCos(anglelimit));
            }
            else if( cmd == "solveall" ) {
                bSolveAll = true;
            }
            else if( cmd == "ikparam" ) {
                sinput >> ikparam;
            }
            else if( cmd == "filteroptions" ) {
                sinput >> filteroptions;
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

        if( !filterfn ) {
            throw openrave_exception("FindIKWithFilters: no filter function set");
        }
        UserDataPtr filterhandle = robot->GetActiveManipulator()->GetIkSolver()->RegisterCustomFilter(0,filterfn);
        vector< vector<dReal> > vsolutions;
        if( bSolveAll ) {
            if( !pmanip->FindIKSolutions(ikparam,vsolutions,filteroptions)) {
                return false;
            }
        }
        else {
            vsolutions.resize(1);
            if( !pmanip->FindIKSolution(ikparam,vsolutions[0],filteroptions)) {
                return false;
            }
        }
        sout << vsolutions.size() << " ";
        FOREACH(itsol,vsolutions) {
            FOREACH(it, *itsol) {
                sout << *it << " ";
            }
        }
        return true;
    }

    bool _VerifyTrajectoryCommand(ostream& sout, istream& sinput)
    {
        TrajectoryBasePtr ptraj;
        dReal samplingstep = 0.001;
        bool bRecomputeTiming = false;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "stream" ) {
                ptraj = RaveCreateTrajectory(GetEnv(),"");
                ptraj->deserialize(sinput);
            }
            else if( cmd == "resettiming" ) {
                sinput >> bRecomputeTiming;
            }
            else if( cmd == "resettrans" ) {
                bool bReset = false;
                sinput >> bReset;
                if( bReset ) {
                    RAVELOG_WARN("resettiming not supported\n");
                }
            }
            else if( cmd == "samplingstep" ) {
                sinput >> samplingstep;
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

        if( !ptraj ) {
            return false;
        }

        if( bRecomputeTiming || ptraj->GetDuration() == 0 ) {
            planningutils::RetimeActiveDOFTrajectory(ptraj,robot);
        }

        RobotBase::RobotStateSaver saver(robot);
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->SetRobotActiveJoints(robot);
        try{
            planningutils::VerifyTrajectory(params,ptraj,samplingstep);
            return true;
        }
        catch(const openrave_exception& ex) {
            RAVELOG_WARN("%s\n",ex.what());
        }
        return false;
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
    IkFilterReturn _FilterWorldAxisIK(std::vector<dReal>& values, RobotBase::ManipulatorPtr pmanip, const IkParameterization& ikparam, const Vector& vlocalaxis, const Vector& vworldaxis, dReal coslimit)
    {
        if( RaveFabs(vworldaxis.dot3(pmanip->GetTransform().rotate(vlocalaxis))) < coslimit ) {
            return IKFR_Reject;
        }
        return IKFR_Success;
    }

    RobotBasePtr robot;
    string _strRRTPlannerName;
    dReal _fMaxVelMult;
};

ModuleBasePtr CreateBaseManipulation(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new BaseManipulation(penv));
}
