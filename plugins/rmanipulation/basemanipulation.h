// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_MANIPULATION_H
#define OPENRAVE_MANIPULATION_H

#include "commonmanipulation.h"

class BaseManipulation : public ProblemInstance
{
public:
 BaseManipulation(EnvironmentBasePtr penv) : ProblemInstance(penv) {
        __description = "Base Manipulation Problem - Rosen Diankov";
        RegisterCommand("SetActiveManip",boost::bind(&BaseManipulation::SetActiveManip,this,_1,_2),
                        "Set the active manipulator");
        RegisterCommand("Traj",boost::bind(&BaseManipulation::Traj,this,_1,_2),
                        "Execute a trajectory from a file on the local filesystem");
        RegisterCommand("GrabBody",boost::bind(&BaseManipulation::GrabBody,this,_1,_2),
                        "Robot calls ::Grab on a body with its current manipulator");
        RegisterCommand("ReleaseAll",boost::bind(&BaseManipulation::ReleaseAll,this,_1,_2),
                        "Releases all grabbed bodies (RobotBase::ReleaseAllGrabbed).");
        RegisterCommand("MoveHandStraight",boost::bind(&BaseManipulation::MoveHandStraight,this,_1,_2),
                        "Move the active end-effector in a straight line until collision or IK fails.");
        RegisterCommand("MoveManipulator",boost::bind(&BaseManipulation::MoveManipulator,this,_1,_2),
                        "Moves arm joints of active manipulator to a given set of joint values");
        RegisterCommand("MoveActiveJoints",boost::bind(&BaseManipulation::MoveActiveJoints,this,_1,_2),
                        "Moves the current active joints to a specified goal destination\n");
        RegisterCommand("MoveToHandPosition",boost::bind(&BaseManipulation::MoveToHandPosition,this,_1,_2),
                        "Move the manipulator's end effector to some 6D pose.");
        RegisterCommand("MoveUnsyncJoints",boost::bind(&BaseManipulation::MoveUnsyncJoints,this,_1,_2),
                        "Moves the active joints to a position where the inactive (hand) joints can\n"
                        "fully move to their goal. This is necessary because synchronization with arm\n"
                        "and hand isn't guaranteed.\n"
                        "Options: handjoints savetraj planner");
        RegisterCommand("CloseFingers",boost::bind(&BaseManipulation::CloseFingers,this,_1,_2),
                        "see TaskManipulation problem");
        RegisterCommand("ReleaseFingers",boost::bind(&BaseManipulation::ReleaseFingers,this,_1,_2),
                        "see TaskManipulation problem.");
        RegisterCommand("ReleaseActive",boost::bind(&BaseManipulation::ReleaseActive,this,_1,_2),
                        "see TaskManipulation problem.");
        RegisterCommand("JitterActive",boost::bind(&BaseManipulation::JitterActive,this,_1,_2),
                        "Jitters the active DOF for a collision-free position.");
        RegisterCommand("SmoothTrajectory",boost::bind(&BaseManipulation::SmoothTrajectory,this,_1,_2),
                        "Smooths a trajectory of points and returns the new trajectory such that\n"
                        "it is guaranteed to contain no co-linear points in configuration space\n");
        _fMaxVelMult=1;
    }

    virtual ~BaseManipulation() {}

    virtual void Destroy()
    {
        robot.reset();
        ProblemInstance::Destroy();
    }

    virtual void Reset()
    {
        ProblemInstance::Reset();
    }

    virtual int main(const std::string& args)
    {
        stringstream ss(args);
        ss >> _strRobotName;

        _fMaxVelMult=1;
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "planner" )
                ss >> _strRRTPlannerName;
            else if( cmd == "maxvelmult" )
                ss >> _fMaxVelMult;

            if( ss.fail() || !ss )
                break;
        }

        PlannerBasePtr planner;
        if( _strRRTPlannerName.size() > 0 )
            planner = GetEnv()->CreatePlanner(_strRRTPlannerName);
        if( !planner ) {
            _strRRTPlannerName = "BiRRT";
            planner = GetEnv()->CreatePlanner(_strRRTPlannerName);
            if( !planner )
                _strRRTPlannerName = "";
        }

        RAVELOG_DEBUGA(str(boost::format("BaseManipulation: using %s planner\n")%_strRRTPlannerName));
        return 0;
    }

    virtual bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        robot = GetEnv()->GetRobot(_strRobotName);
        return ProblemInstance::SendCommand(sout,sinput);
    }
protected:

    inline boost::shared_ptr<BaseManipulation> shared_problem() { return boost::static_pointer_cast<BaseManipulation>(shared_from_this()); }
    inline boost::shared_ptr<BaseManipulation const> shared_problem_const() const { return boost::static_pointer_cast<BaseManipulation const>(shared_from_this()); }

    bool SetActiveManip(ostream& sout, istream& sinput)
    {
        string manipname;
        int index = -1;

        if(!sinput.eof()) {
            sinput >> manipname;
            if( !sinput )
                return false;
        
            // find the manipulator with the right name
            index = 0;
            FOREACHC(itmanip, robot->GetManipulators()) {
                if( manipname == (*itmanip)->GetName() )
                    break;
                ++index;
            }

            if( index >= (int)robot->GetManipulators().size() ) {
                index = atoi(manipname.c_str());
            }
        }

        if( index >= 0 && index < (int)robot->GetManipulators().size() ) {
            robot->SetActiveManipulator(index);
            return true;
        }

        return false;
    }

    bool Traj(ostream& sout, istream& sinput)
    {
        string filename; sinput >> filename;
        if( !sinput )
            return false;

        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(robot->GetDOF());
    
        char sep = ' ';
        if( filename == "sep" ) {
            sinput >> sep;
            filename = getfilename_withseparator(sinput,sep);
        }

        if( filename == "stream" ) {
            // the trajectory is embedded in the stream
            RAVELOG_VERBOSEA("BaseManipulation: reading trajectory from stream\n");

            if( !ptraj->Read(sinput, robot) ) {
                RAVELOG_ERRORA("BaseManipulation: failed to get trajectory\n");
                return false;
            }
        }
        else {
            RAVELOG_VERBOSEA(str(boost::format("BaseManipulation: reading trajectory: %s\n")%filename));
            ifstream f(filename.c_str());
            if( !ptraj->Read(f, robot) ) {
                RAVELOG_ERRORA(str(boost::format("BaseManipulation: failed to read trajectory %s\n")%filename));
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

        RAVELOG_VERBOSEA(str(boost::format("executing traj with %d points\n")%ptraj->GetPoints().size()));
        robot->SetMotion(ptraj);
        sout << "1";
        return true;
    }

    bool GrabBody(ostream& sout, istream& sinput)
    {
        KinBodyPtr ptarget;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "name" ) {
                string name;
                sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else break;

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if(!ptarget) {
            RAVELOG_ERRORA("ERROR Manipulation::GrabBody - Invalid body name.\n");
            return false;
        }

        RAVELOG_DEBUGA(str(boost::format("robot %s:%s grabbing body %s...\n")%robot->GetName()%robot->GetActiveManipulator()->GetEndEffector()->GetName()%ptarget->GetName()));
        robot->Grab(ptarget);
        return true;
    }

    bool ReleaseAll(ostream& sout, istream& sinput)
    {
        if( !!robot ) {
            RAVELOG_DEBUGA("Releasing all bodies\n");
            robot->ReleaseAllGrabbed();
        }
        return true;
    }

    bool MoveHandStraight(ostream& sout, istream& sinput)
    {
        float stepsize = 0.003f;
        Vector direction = Vector(0,1,0);
        string strtrajfilename;
        bool bExecute = true;

        int minsteps = 0;
        int maxsteps = 10000;

        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
    
        dReal fjacobianerror=0;
        bool bIgnoreFirstCollision = true, bSearchAll = false;
        boost::shared_ptr<ostream> pOutputTrajStream;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "minsteps" )
                sinput >> minsteps;
            else if( cmd == "outputtraj")
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "maxsteps")
                sinput >> maxsteps;
            else if( cmd == "stepsize")
                sinput >> stepsize;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj")
                sinput >> strtrajfilename;
            else if( cmd == "direction")
                sinput >> direction.x >> direction.y >> direction.z;
            else if( cmd == "ignorefirstcollision")
                sinput >> bIgnoreFirstCollision;
            else if( cmd == "jacobian")
                sinput >> fjacobianerror;
            else if( cmd == "searchall" )
                sinput >> bSearchAll;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
    
        RAVELOG_DEBUGA("Starting MoveHandStraight dir=(%f,%f,%f)...\n",(float)direction.x, (float)direction.y, (float)direction.z);
        robot->RegrabAll();

        RobotBase::RobotStateSaver saver(robot);

        robot->SetActiveDOFs(pmanip->GetArmJoints());
        CM::JitterActiveDOF(robot,100); // try to jitter out, don't worry if it fails

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
        Trajectory::TPOINT point;
        vector<dReal> vPrevValues;
        bool bPrevInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot))||robot->CheckSelfCollision();
        robot->GetActiveDOFValues(vPrevValues);

        if( bPrevInCollision && !bIgnoreFirstCollision ) {
            RAVELOG_WARNA("MoveHandStraight: robot in collision\n");
            return false;
        }

        Transform handTr = robot->GetActiveManipulator()->GetEndEffectorTransform();

        boost::shared_ptr<CM::SimpleDistMetric> distmetricfn(new CM::SimpleDistMetric(robot));
        point.q = vPrevValues;
        ptraj->AddPoint(point);
        vector<vector<dReal> > vsolutions;

        // variables only for jacobians
        boost::multi_array<dReal,2> vjacobian;
        boost::shared_ptr<CM::GripperJacobianConstrains<double> > pconstraints;
        boost::numeric::ublas::matrix<double> J, Jt, invJJt, invJ, Jerror, qdelta;
        int i, eeindex=-1;
        for (i = 0; i < maxsteps;  i++) {
            if( fjacobianerror > 0 ) {
                // use jacobian
                if( !pconstraints ) {
                    boost::array<double,6> vconstraintfreedoms = {{1,1,0,1,1,0}}; // only rotate and translate across z
                    Transform tframe; tframe.rot = quatRotateDirection(direction,Vector(0,0,1));
                    pconstraints.reset(new CM::GripperJacobianConstrains<double>(robot->GetActiveManipulator(),tframe,vconstraintfreedoms,fjacobianerror));
                    pconstraints->_distmetricfn = boost::bind(&CM::SimpleDistMetric::Eval,distmetricfn,_1,_2);
                    eeindex = robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
                    J.resize(3,robot->GetActiveDOF());
                    invJJt.resize(3,3);
                    Jerror.resize(3,1);
                    Jerror(0,0) = direction.x*stepsize; Jerror(1,0) = direction.y*stepsize; Jerror(2,0) = direction.z*stepsize;
                }

                robot->CalculateActiveJacobian(eeindex,robot->GetActiveManipulator()->GetEndEffectorTransform().trans,vjacobian);
                const double lambda2 = 1e-8; // normalization constant
                for(size_t j = 0; j < 3; ++j)
                    std::copy(vjacobian[j].begin(),vjacobian[j].end(),J.find2(0,j,0));
                Jt = trans(J);
                invJJt = prod(J,Jt);
                for(int j = 0; j < 3; ++j)
                    invJJt(j,j) += lambda2;
                try {
                    if( !pconstraints->InvertMatrix(invJJt,invJJt) ) {
                        RAVELOG_WARN("failed to invert matrix\n");
                        break;
                    }
                }
                catch(...) {
                    RAVELOG_WARN("failed to invert matrix!!\n");
                    break;
                }
                invJ = prod(Jt,invJJt);
                qdelta = prod(invJ,Jerror);
                for(size_t j = 0; j < point.q.size(); ++j)
                    point.q[j] = vPrevValues[j] + qdelta(j,0);
                if( !pconstraints->RetractionConstraint(vPrevValues,point.q,0) )
                    break;
                robot->SetActiveDOFValues(point.q);
                bool bInCollision = robot->CheckSelfCollision();
                Transform tdelta = handTr.inverse()*robot->GetActiveManipulator()->GetEndEffectorTransform();
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    stringstream ss; ss << "transform: " << tdelta << endl;
                    RAVELOG_VERBOSE(ss.str());
                }
                robot->SetActiveDOFValues(vPrevValues);
                if( bInCollision )
                    break;
            }
            else {
                handTr.trans += stepsize*direction;
                bool bCheckCollision = !bPrevInCollision && i >= minsteps;
                if( bSearchAll ) {
                    if( !pmanip->FindIKSolutions(handTr,vsolutions,bCheckCollision)) {
                        RAVELOG_DEBUGA("Arm Lifting: broke due to ik\n");
                        break;
                    }
                    int minindex=0;
                    dReal fbestdist = distmetricfn->Eval(vPrevValues,vsolutions.at(0));
                    for(size_t j = 1; j < vsolutions.size(); ++j) {
                        dReal fdist = distmetricfn->Eval(vPrevValues,vsolutions[j]);
                        if( fbestdist > fdist ) {
                            fbestdist = fdist;
                            minindex = j;
                        }
                    }
                    point.q = vsolutions[minindex];
                }
                else {
                    if( !pmanip->FindIKSolution(handTr,point.q,bCheckCollision)) {
                        RAVELOG_DEBUGA("Arm Lifting: broke due to ik\n");
                        break;
                    }
                }
            }
        
            size_t j = 0;
            for(; j < point.q.size(); j++) {
                if(fabsf(point.q[j] - vPrevValues[j]) > 0.2)
                    break;
            }

            if( j < point.q.size()) {
                RAVELOG_DEBUGA(str(boost::format("Arm Lifting: broke due to discontinuity (%d:%f)\n")%j%(point.q[j] - vPrevValues[j])));
                break;
            }
        
            robot->SetActiveDOFValues(point.q);
        
            bool bInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot))||robot->CheckSelfCollision();
            if(bInCollision && !bPrevInCollision && i >= minsteps) {
                RAVELOG_DEBUGA("Arm Lifting: broke due to collision\n");
                break;
            }
        
            ptraj->AddPoint(point);
            vPrevValues = point.q;
            bPrevInCollision = bInCollision;
        }
    
        if( i > 0 ) {
            if( bPrevInCollision ) {
                RAVELOG_DEBUGA("hand failed to move out of collision\n");
                return false;
            }
            RAVELOG_DEBUGA("hand can move %f\n", (float)i*stepsize);
            if( i >= minsteps ) // only move if exceeded minsteps (otherwise user of this function would not have specified min steps)
                CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
            return i >= minsteps;
        }

        RAVELOG_DEBUGA("hand didn't move\n");
        return i >= minsteps;
    }

    bool MoveManipulator(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUGA("Starting MoveManipulator...\n");
        RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();

        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;
        std::vector<dReal> goals;
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000; // max iterations before failure

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "armvals" || cmd == "goal" ) {
                goals.resize(pmanip->GetArmJoints().size());
                FOREACH(it, goals)
                    sinput >> *it;
            }
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
    
        if( goals.size() != pmanip->GetArmJoints().size() )
            return false;

        RobotBase::RobotStateSaver saver(robot);

        robot->SetActiveDOFs(pmanip->GetArmJoints());
        params->SetRobotActiveJoints(robot);
        CM::JitterActiveDOF(robot);
    
        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

        std::vector<dReal> values;
        robot->GetActiveDOFValues(values);

        // make sure the initial and goal configs are not in collision
        robot->SetActiveDOFValues(goals, true);
        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("jitter failed\n");
            return false;
        }
        robot->GetActiveDOFValues(params->vgoalconfig);
        robot->SetActiveDOFValues(values);
    
        // jitter again for initial collision
        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("jitter failed\n");
            return false;
        }
        robot->GetActiveDOFValues(params->vinitialconfig);

        boost::shared_ptr<PlannerBase> rrtplanner = GetEnv()->CreatePlanner(_strRRTPlannerName);
        if( !rrtplanner ) {
            RAVELOG_WARNA("failed to create planner\n");
            return false;
        }
    
        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
    
        for(int iter = 0; iter < 3; ++iter) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERRORA("InitPlan failed\n");
                break;
            }
        
            if( rrtplanner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_INFOA("finished planning\n");
                break;
            }
            else RAVELOG_WARNA("PlanPath failed\n");
        }

        if( !bSuccess )
            return false;
    
        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        sout << "1";
        return true;
    }

    bool MoveActiveJoints(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        int nMaxTries = 1; // max tries for the planner
        boost::shared_ptr<ostream> pOutputTrajStream;
    
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000; // max iterations before failure

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "goal" ) {
                params->vgoalconfig.resize(robot->GetActiveDOF());
                FOREACH(it, params->vgoalconfig)
                    sinput >> *it;
            }
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "steplength" )
                sinput >> params->_fStepLength;
            else if( cmd == "maxtries" )
                sinput >> nMaxTries;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( (int)params->vgoalconfig.size() != robot->GetActiveDOF() )
            return false;
    
        RobotBase::RobotStateSaver saver(robot);

        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("failed\n");
            return false;
        }

        // restore
        params->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(params->vinitialconfig);
        robot->SetActiveDOFValues(params->vgoalconfig);
    
        // jitter again for goal
        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("failed\n");
            return false;
        }

        boost::shared_ptr<PlannerBase> rrtplanner = GetEnv()->CreatePlanner(_strRRTPlannerName);

        if( !rrtplanner ) {
            RAVELOG_ERRORA("failed to create BiRRTs\n");
            return false;
        }
    
        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    
        RAVELOG_DEBUGA("starting planning\n");
        bool bSuccess = false;
        for(int itry = 0; itry < nMaxTries; ++itry) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERRORA("InitPlan failed\n");
                return false;
            }
            
            if( !rrtplanner->PlanPath(ptraj) ) {
                RAVELOG_WARNA("PlanPath failed\n");
            }
            else {
                bSuccess = true;
                RAVELOG_DEBUGA("finished planning\n");
                break;
            }
        }

        if( !bSuccess )
            return false;
        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool MoveToHandPosition(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUGA("Starting MoveToHandPosition...\n");
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();

        list<IkParameterization> listgoals;
    
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        Vector vconstraintaxis, vconstraintpos;
        int affinedofs = 0;
        int nSeedIkSolutions = 0; // no extra solutions
        int nMaxTries = 3; // max tries for the planner

        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000;

        // constraint stuff
        boost::array<double,6> vconstraintfreedoms = {{0,0,0,0,0,0}};
        Transform tConstraintTargetWorldFrame;
        double constrainterrorthresh=0;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "translation" ) {
                Vector trans;
                sinput >> trans.x >> trans.y >> trans.z;
                listgoals.push_back(IkParameterization());
                listgoals.back().SetTranslation(trans);
            }
            else if( cmd == "rotation" ) {
                Vector q;
                sinput >> q.x >> q.y >> q.z >> q.w;
                listgoals.push_back(IkParameterization());
                listgoals.back().SetRotation(q);
            }
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "matrix" ) {
                TransformMatrix m;
                sinput >> m;
                listgoals.push_back(IkParameterization(Transform(m)));
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
            else if( cmd == "affinedofs" )
                sinput >> affinedofs;
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "maxtries" )
                sinput >> nMaxTries;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "seedik" )
                sinput >> nSeedIkSolutions;
            else if( cmd == "constraintfreedoms" )
                FOREACH(it,vconstraintfreedoms)
                    sinput >> *it;
            else if( cmd == "constraintmatrix" ) {
                TransformMatrix m; sinput >> m; tConstraintTargetWorldFrame = m;
            }
            else if( cmd == "constraintpose" )
                sinput >> tConstraintTargetWorldFrame;
            else if( cmd == "constrainterrorthresh" )
                sinput >> constrainterrorthresh;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
    
        robot->RegrabAll();
        RobotBase::RobotStateSaver saver(robot);

        std::vector<dReal> viksolution, armgoals;

        if( nSeedIkSolutions < 0 ) {
            vector<vector<dReal> > solutions;

            FOREACH(ittrans, listgoals) {
                pmanip->FindIKSolutions(*ittrans, solutions, true);
            
                armgoals.reserve(armgoals.size()+solutions.size()*pmanip->GetArmJoints().size());
                FOREACH(itsol, solutions)
                    armgoals.insert(armgoals.end(), itsol->begin(), itsol->end());
            }
        }
        else if( nSeedIkSolutions > 0 ) {
            FOREACH(ittrans, listgoals) {
                int nsampled = CM::SampleIkSolutions(robot, *ittrans, nSeedIkSolutions, armgoals);
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
                    RAVELOG_DEBUGA(s.str());
                    armgoals.insert(armgoals.end(), viksolution.begin(), viksolution.end());
                }
            }
        }

        if( armgoals.size() == 0 ) {
            RAVELOG_WARNA("No IK Solution found\n");
            return false;
        }

        RAVELOG_INFO(str(boost::format("MoveToHandPosition found %d solutions\n")%(armgoals.size()/pmanip->GetArmJoints().size())));
    
        robot->SetActiveDOFs(pmanip->GetArmJoints(), affinedofs);
        params->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(params->vinitialconfig);        
        robot->SetActiveDOFs(pmanip->GetArmJoints(), 0);

        vector<dReal> vgoals;
        params->vgoalconfig.reserve(armgoals.size());
        for(int i = 0; i < (int)armgoals.size(); i += pmanip->GetArmJoints().size()) {
            vector<dReal> v(armgoals.begin()+i,armgoals.begin()+i+pmanip->GetArmJoints().size());
            robot->SetActiveDOFValues(v);

            robot->SetActiveDOFs(pmanip->GetArmJoints(), affinedofs);

            if( CM::JitterActiveDOF(robot) ) {
                robot->GetActiveDOFValues(vgoals);
                params->vgoalconfig.insert(params->vgoalconfig.end(), vgoals.begin(), vgoals.end());
            }
        }

        if( params->vgoalconfig.size() == 0 ) {
            RAVELOG_WARNA("jitter failed for goal\n");
            return false;
        }

        // restore
        robot->SetActiveDOFValues(params->vinitialconfig);

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

        Trajectory::TPOINT pt;
        pt.q = params->vinitialconfig;
        ptraj->AddPoint(pt);
    
        // jitter again for initial collision
        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("jitter failed for initial\n");
            return false;
        }
        robot->GetActiveDOFValues(params->vinitialconfig);

        if( constrainterrorthresh > 0 ) {
            RAVELOG_DEBUG("setting jacobian constraint function in planner parameters\n");
            boost::shared_ptr<CM::GripperJacobianConstrains<double> > pconstraints(new CM::GripperJacobianConstrains<double>(robot->GetActiveManipulator(),tConstraintTargetWorldFrame,vconstraintfreedoms,constrainterrorthresh));
            pconstraints->_distmetricfn = params->_distmetricfn;
            params->_constraintfn = boost::bind(&CM::GripperJacobianConstrains<double>::RetractionConstraint,pconstraints,_1,_2,_3);
        }

        boost::shared_ptr<PlannerBase> rrtplanner = GetEnv()->CreatePlanner(_strRRTPlannerName);
        if( !rrtplanner ) {
            RAVELOG_ERRORA("failed to create BiRRTs\n");
            return false;
        }
    
        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
        
        for(int iter = 0; iter < nMaxTries; ++iter) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_ERRORA("InitPlan failed\n");
                return false;
            }
        
            if( rrtplanner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_INFOA("finished planning\n");
                break;
            }
            else
                RAVELOG_WARNA("PlanPath failed\n");
        }

        rrtplanner.reset(); // have to destroy before environment
    
        if( !bSuccess )
            return false;

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
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "writetraj" )
                sinput >> strsavetraj;
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "handjoints" ) {
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
            else if( cmd == "planner" )
                sinput >> strplanner;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "maxtries" )
                sinput >> nMaxTries;
            else if( cmd == "maxdivision" )
                sinput >> maxdivision;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        uint32_t starttime = timeGetTime();

        if( CM::JitterActiveDOF(robot) == 0 ) {
            RAVELOG_WARNA("failed to jitter robot out of collision\n");
        }

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
    
        bool bSuccess = false;
        for(int itry = 0; itry < nMaxTries; ++itry) {
            if( CM::MoveUnsync::_MoveUnsyncJoints(GetEnv(), robot, ptraj, vhandjoints, vhandgoal, strplanner,maxdivision) ) {
                bSuccess = true;
                break;
            }
        }
        if( !bSuccess )
            return false;

        BOOST_ASSERT(ptraj->GetPoints().size() > 0);

        bool bExecuted = CM::SetActiveTrajectory(robot, ptraj, bExecute, strsavetraj, pOutputTrajStream,_fMaxVelMult);
        sout << (int)bExecuted << " ";

        sout << (timeGetTime()-starttime)/1000.0f << " ";
        FOREACH(it, ptraj->GetPoints().back().q)
            sout << *it << " ";

        return true;
    }

    bool CloseFingers(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("CloseFingers moved to TaskManipulation...\n");
        bool bExecute = true, bOutputFinal=false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        Vector direction;
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));
        graspparams->vgoalconfig = pmanip->GetClosingDirection();

        vector<dReal> voffset;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "outputfinal" )
                bOutputFinal = true;
            else if( cmd == "offset" ) {
                voffset.resize(pmanip->GetGripperJoints().size());
                FOREACH(it, voffset)
                    sinput >> *it;
            }
            else if( cmd == "movingdir" ) {
                FOREACH(it,graspparams->vgoalconfig)
                    sinput >> *it;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        robot->SetActiveDOFs(pmanip->GetGripperJoints());

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        robot->GetActiveDOFValues(ptfirst.q);
 
        boost::shared_ptr<PlannerBase> graspplanner = GetEnv()->CreatePlanner("Grasper");
        if( !graspplanner ) {
            RAVELOG_ERRORA("grasping planner failure!\n");
            return false;
        }
    
        graspparams->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(graspparams->vinitialconfig);
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
        ptraj->AddPoint(ptfirst);

        if( !graspplanner->InitPlan(robot, graspparams) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARNA("PlanPath failed\n");
            return false;
        }   

        if( ptraj->GetPoints().size() == 0 )
            return false;

        if( bOutputFinal ) {
            FOREACH(itq,ptraj->GetPoints().back().q)
                sout << *itq << " ";
        }

        Trajectory::TPOINT p = ptraj->GetPoints().back();
        if(p.q.size() == voffset.size() ) {
            for(size_t i = 0; i < voffset.size(); ++i)
                p.q[i] += voffset[i]*pmanip->GetClosingDirection().at(i);
            robot->SetActiveDOFValues(p.q,true);
            robot->GetActiveDOFValues(p.q);
            ptraj->AddPoint(p);
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool ReleaseFingers(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("ReleaseFingers moved to TaskManipulation problem...\n");
        bool bExecute = true, bOutputFinal=false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        Vector direction;
        KinBodyPtr ptarget;
        RobotBase::ManipulatorConstPtr pmanip = robot->GetActiveManipulator();
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));
        graspparams->vgoalconfig = pmanip->GetClosingDirection();
        FOREACH(it,graspparams->vgoalconfig)
            *it = -*it;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "outputfinal" )
                bOutputFinal = true;
            else if( cmd == "movingdir" ) {
                FOREACH(it,graspparams->vgoalconfig)
                    sinput >> *it;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        robot->SetActiveDOFs(pmanip->GetGripperJoints());
        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
        // have to add the first point
        Trajectory::TPOINT ptfirst;
        robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch(CM::JitterActiveDOF(robot) ) {
        case 0:
            RAVELOG_WARNA("robot initially in collision\n");
            return false;
        case 1:
            robot->GetActiveDOFValues(ptfirst.q);
        default:
            break;
        }
 
        boost::shared_ptr<PlannerBase> graspplanner = GetEnv()->CreatePlanner("Grasper");
        if( !graspplanner ) {
            RAVELOG_ERRORA("grasping planner failure!\n");
            return false;
        }
    
        graspparams->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(graspparams->vinitialconfig);
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;
        graspparams->bavoidcontact = true;

        if( !graspplanner->InitPlan(robot, graspparams) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARNA("PlanPath failed\n");
            return false;
        }   

        if( ptraj->GetPoints().size() == 0 )
            return false;

        if( bOutputFinal ) {
            FOREACH(itq,ptraj->GetPoints().back().q)
                sout << *itq << " ";
        }

        {
            // check final trajectory for colliding points
            RobotBase::RobotStateSaver saver2(robot);
            robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
            if( CM::JitterActiveDOF(robot) > 0 ) {
                RAVELOG_WARNA("robot final configuration is in collision\n");
                Trajectory::TPOINT pt = ptraj->GetPoints().back();
                robot->GetActiveDOFValues(pt.q);
                ptraj->AddPoint(pt);
            }
        }

        if( !!ptarget )
            robot->Release(ptarget);

        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);

        return true;
    }

    bool ReleaseActive(ostream& sout, istream& sinput)
    {
        RAVELOG_WARN("ReleaseActive moved to TaskManipulation ...\n");

        bool bExecute = true, bOutputFinal = false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));

        // initialize the moving direction as the opposite of the closing direction defined in the manipulators
		vector<dReal> vclosingsign_full(robot->GetDOF(), 0);
        FOREACHC(itmanip, robot->GetManipulators()) {
            BOOST_ASSERT((*itmanip)->GetClosingDirection().size()==(*itmanip)->GetGripperJoints().size());
            for(size_t i = 0; i < (*itmanip)->GetClosingDirection().size(); ++i) {
                if( (*itmanip)->GetClosingDirection()[i] != 0 )
                    vclosingsign_full[(*itmanip)->GetGripperJoints()[i]] = (*itmanip)->GetClosingDirection()[i];
            }
        }

        graspparams->vgoalconfig.resize(robot->GetActiveDOF());
        int i = 0;
        FOREACHC(itindex,robot->GetActiveJointIndices()) {
            graspparams->vgoalconfig[i++] = -vclosingsign_full.at(*itindex);
        }

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "outputfinal" )
                bOutputFinal = true;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "movingdir" ) {
                // moving direction, has to be of size activedof
                FOREACH(it, graspparams->vgoalconfig)
                    sinput >> *it;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch( CM::JitterActiveDOF(robot) ) {
        case 0:
            RAVELOG_WARNA("robot initially in collision\n");
            return false;
        case 1:
            robot->GetActiveDOFValues(ptfirst.q);
        default:
            break;
        }
 
        boost::shared_ptr<PlannerBase> graspplanner = GetEnv()->CreatePlanner("Grasper");
        if( !graspplanner ) {
            RAVELOG_ERRORA("grasping planner failure!\n");
            return false;
        }
        
        robot->SetActiveManipulator(-1); // reset the manipulator
        graspparams->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(graspparams->vinitialconfig);  
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;
        graspparams->bavoidcontact = true;

        if( !graspplanner->InitPlan(robot, graspparams) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARNA("PlanPath failed\n");
            return false;
        }

        if( ptraj->GetPoints().size() == 0 )
            return false;


        if( bOutputFinal ) {
            FOREACH(itq,ptraj->GetPoints().back().q)
                sout << *itq << " ";
        }

        {
            // check final trajectory for colliding points
            RobotBase::RobotStateSaver saver2(robot);
            robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
            if( CM::JitterActiveDOF(robot) > 0 ) {
                RAVELOG_WARNA("robot final configuration is in collision\n");
                Trajectory::TPOINT pt = ptraj->GetPoints().back();
                robot->GetActiveDOFValues(pt.q);
                ptraj->AddPoint(pt);
            }
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool JitterActive(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting ReleaseFingers...\n");
        bool bExecute = true, bOutputFinal=false;
        boost::shared_ptr<ostream> pOutputTrajStream;
        string cmd;
        int nMaxIterations=5000;
        dReal fJitter=0.03f;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "maxiter" )
                sinput >> nMaxIterations;
            else if( cmd == "jitter" )
                sinput >> fJitter;
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "outputfinal" )
                bOutputFinal = true;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(robot);
        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch( CM::JitterActiveDOF(robot,nMaxIterations,fJitter) ) {
        case 0:
            RAVELOG_WARNA("could not jitter out of collision\n");
            return false;
        case 1:
            robot->GetActiveDOFValues(ptfirst.q);
            ptraj->AddPoint(ptfirst);
        default:
            break;
        }

        if( bOutputFinal ) {
            FOREACH(itq,ptfirst.q)
                sout << *itq << " ";
        }

        CM::SetActiveTrajectory(robot, ptraj, bExecute, "", pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    static bool CheckCollision(RobotBasePtr probot, const vector<dReal>& q0, const vector<dReal>& q1, const vector<dReal>& qresolutioninv)
    {
        BOOST_ASSERT( probot->GetDOF() == (int)q0.size() && probot->GetDOF()== (int)q1.size() && probot->GetDOF() == (int)qresolutioninv.size() );
    
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
        
            probot->SetJointValues(vtempconfig);
            if( probot->GetEnv()->CheckCollision(KinBodyConstPtr(probot)) || probot->CheckSelfCollision() )
                return true;
        }

        return false;
    }

    static void OptimizePathRandomized(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv, int _nMaxIterations)
    {
        if( path.size() <= 2 )
            return;

        list< Trajectory::TPOINT >::iterator itstart, itend;
    
        int nrejected = 0;
        int i = _nMaxIterations;
        while(i > 0 && nrejected < (int)path.size() ) {

            --i;

            // pick a random node on the path, and a random jump ahead
            int endIndex = 1+(RaveRandomInt()%((int)path.size()-1));
            int startIndex = RaveRandomInt()%endIndex;
        
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
    static void OptimizePathAll(RobotBasePtr probot, list< Trajectory::TPOINT >& path, const vector<dReal>& qresolutioninv)
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

    bool SmoothTrajectory(ostream& sout, istream& sinput)
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
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "traj" ) {
                if( !ptraj->Read(sinput,robot) ) {
                    RAVELOG_ERRORA("failed to read trajectory\n");
                    return false;
                }
            }
            else if( cmd == "trajfile" ) {
                string filename;
                sinput >> filename;
                ifstream f(filename.c_str());
                if( !ptraj->Read(f,robot) ) {
                    RAVELOG_ERRORA("failed to read trajectory\n");
                    return false;
                }
            }
            else if( cmd == "maxsmoothiter" ) {
                sinput >> nMaxSmoothIterations;
            }
            else if( cmd == "resolution" ) {
                FOREACH(it,qresolutioninv)
                    sinput >> *it;
            }
            else if( cmd == "execute" )
                sinput >> bExecute;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
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
            pnewtraj->CalcTrajTiming(robot,pnewtraj->GetInterpMethod(),true,false,_fMaxVelMult);
            robot->SetMotion(pnewtraj);
        }

        pnewtraj->Write(sout, Trajectory::TO_OneLine);
        return true;
    }

    RobotBasePtr robot;
    string _strRRTPlannerName;
    string _strRobotName; ///< name of the active robot
    dReal _fMaxVelMult;
};

#endif
