// Author: Achint Aggarwal
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

class DualManipulation : public ProblemInstance
{
 public:
 DualManipulation(EnvironmentBasePtr penv) : ProblemInstance(penv) {
        __description = "Dual Manipulation Problem - Achint Aggarwal";
        RegisterCommand("SetActiveManip",boost::bind(&DualManipulation::SetActiveManip,this,_1,_2),
                        "Set the active manipulator");
        RegisterCommand("GrabBody",boost::bind(&DualManipulation::GrabBody,this,_1,_2),
                        "Robot calls ::Grab on a body with its current manipulator");
        RegisterCommand("ReleaseAll",boost::bind(&DualManipulation::ReleaseAll,this,_1,_2),
                        "Releases all grabbed bodies (RobotBase::ReleaseAllGrabbed).");
        RegisterCommand("MoveAllJoints",boost::bind(&DualManipulation::MoveAllJoints,this,_1,_2),
                        "Moves the current active joints to a specified goal destination\n");
        RegisterCommand("MoveBothHandsStraight",boost::bind(&DualManipulation::MoveBothHandsStraight,this,_1,_2),
                        "Move both the end-effectors in straight lines until collision or IK fails.");
    }

    virtual ~DualManipulation() {}

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

        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "planner" )
                ss >> _strRRTPlannerName;

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

        RAVELOG_DEBUGA(str(boost::format("DualManipulation: using %s planner\n")%_strRRTPlannerName));
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

    inline boost::shared_ptr<DualManipulation> shared_problem() { return boost::static_pointer_cast<DualManipulation>(shared_from_this()); }
    inline boost::shared_ptr<DualManipulation const> shared_problem_const() const { return boost::static_pointer_cast<DualManipulation const>(shared_from_this()); }

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
            sout << "1";
        }
        else {
            RAVELOG_ERRORA("invaild manip %d\n", index);
            sout << "0";
        }

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

    bool MoveAllJoints(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;
        int nMaxTries = 3;
        bool bSuccess=true;
    
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000; // max iterations before failure

        // constraint stuff
        double constrainterrorthresh=0;

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

        if( (int)params->vgoalconfig.size() != robot->GetActiveDOF() )
            return false;
    
        RobotBase::RobotStateSaver saver(robot);

        if( !CM::JitterActiveDOF(robot) ) {
            RAVELOG_WARNA("failed\n");
            return false;
        }

        // restore
        params->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(params->vinitialconfig);
        robot->SetActiveDOFValues(params->vgoalconfig);
    
        // jitter again for goal
        if( !CM::JitterActiveDOF(robot) ) {
            RAVELOG_WARNA("failed\n");
            return false;
        }

        if( constrainterrorthresh > 0 ) {
            RAVELOG_DEBUG("setting DualArmConstrained function in planner parameters\n");
            boost::shared_ptr<CM::DualArmManipulation<double> > dplanner(new CM::DualArmManipulation<double>(robot));
            dplanner->_distmetricfn = params->_distmetricfn;
            params->_constraintfn = boost::bind(&CM::DualArmManipulation<double>::DualArmConstrained,dplanner,_1,_2,_3);//this juts means that the function is called upon pconstraints object and the function takes 3 arguments specified as _1,_2 and _3
            params->_nMaxIterations = 1000;
        
        }

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
        Trajectory::TPOINT pt;

        robot->SetActiveDOFValues(params->vinitialconfig);
        pt.q = params->vinitialconfig;
        ptraj->AddPoint(pt);

        boost::shared_ptr<PlannerBase> rrtplanner = GetEnv()->CreatePlanner(_strRRTPlannerName);
        
        if( !rrtplanner ) {
            RAVELOG_ERRORA("failed to create BiRRTs\n");
            return false;
        }    
        RAVELOG_DEBUGA("starting planning\n");
    
        for(int iter = 0; iter < nMaxTries; ++iter) {
            if( !rrtplanner->InitPlan(robot, params) ) {
                RAVELOG_INFOA("InitPlan failed\n");
                return false;
            }
    
            if(rrtplanner->PlanPath(ptraj) ) {
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
    
        RAVELOG_DEBUGA("finished planning\n");
        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream);


        return true;
    }

    bool MoveBothHandsStraight(ostream& sout, istream& sinput)
    {
        float stepsize = 0.003f;
        Vector directionL = Vector(0,1,0);
        Vector directionR = Vector(0,1,0);
        string strtrajfilename;
        bool bExecute = true;
        bool reachedR = false;
        bool reachedL = false;

        int minsteps = 0;
        int maxsteps = 10000;

        RobotBase::ManipulatorConstPtr pmanipR = robot->GetManipulators()[0];
        RobotBase::ManipulatorConstPtr pmanipL = robot->GetManipulators()[1];
    
    
        bool bIgnoreFirstCollision = true;
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
            else if( cmd == "directionl")
                sinput >> directionL.x >> directionL.y >> directionL.z;
            else if( cmd == "directionr")
                sinput >> directionR.x >> directionR.y >> directionR.z;
            else if( cmd == "ignorefirstcollision")
                sinput >> bIgnoreFirstCollision;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }
    
        RAVELOG_DEBUGA("Starting MoveBothHandsStraight dirL=(%f,%f,%f)...dirR=(%f,%f,%f)...\n",(float)directionL.x, (float)directionL.y, (float)directionL.z, (float)directionR.x, (float)directionR.y, (float)directionR.z);
        robot->RegrabAll();

        RobotBase::RobotStateSaver saver(robot);

        //robot->SetActiveDOFs(pmanip->GetArmJoints());
        CM::JitterActiveDOF(robot,100); // try to jitter out, don't worry if it fails

        boost::shared_ptr<Trajectory> ptraj(GetEnv()->CreateTrajectory(robot->GetActiveDOF()));
        Trajectory::TPOINT point;
        vector<dReal> vPrevValues,vRightJoints,vLeftJoints;
        bool bPrevInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot))||robot->CheckSelfCollision();
        robot->GetActiveDOFValues(vPrevValues);

        if( bPrevInCollision && !bIgnoreFirstCollision ) {
            RAVELOG_WARNA("MoveBothHandsStraight: robot in collision\n");
            return false;
        }

        Transform handTrR = robot->GetManipulators()[0]->GetEndEffectorTransform();
        Transform handTrL = robot->GetManipulators()[1]->GetEndEffectorTransform();

        point.q = vPrevValues;
        ptraj->AddPoint(point);

        int i;
        for (i = 0; i < maxsteps;  i++) {

            if (!reachedR){
                handTrR.trans += stepsize*directionR;
                if( !pmanipR->FindIKSolution(handTrR,vRightJoints,false)) {
                    RAVELOG_DEBUGA("Right Arm Lifting: broke due to ik\n");
                    reachedR=true;
                }
                else 
                    for(int a=pmanipR->GetArmJoints()[0], b = 0; a < (pmanipR->GetArmJoints()[0]+pmanipR->GetArmJoints().size()) && b<pmanipR->GetArmJoints().size(); a++,b++)
                        point.q[a]=vRightJoints[b];
                
            }
    
            if (!reachedL){
                handTrL.trans += stepsize*directionL;
                if( !pmanipL->FindIKSolution(handTrL,vLeftJoints,false)) {
                    RAVELOG_DEBUGA("Left Arm Lifting: broke due to ik\n");
                    reachedL=true;
                }
                else 
                    for(int a=pmanipL->GetArmJoints()[0], b=0; a < (pmanipL->GetArmJoints()[0]+pmanipL->GetArmJoints().size()) && b<pmanipL->GetArmJoints().size(); a++,b++)
                        point.q[a]=vLeftJoints[b];
                
            }
        
            size_t j = 0;
            for(; j < point.q.size(); j++) {
                if(fabsf(point.q[j] - vPrevValues[j]) > 0.2){
                    RAVELOG_DEBUGA("Breaking here %d\n",j);                 
                    break;
                }
            }

            if( j < point.q.size()) {
                RAVELOG_DEBUGA("Arm Lifting: broke due to discontinuity\n");
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
            RAVELOG_DEBUGA("hand moved %f\n", (float)i*stepsize);
            CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream);
            sout << "1";
            return i >= minsteps;
        }

        RAVELOG_DEBUGA("hand didn't move\n");
        return i >= minsteps;
    }

    RobotBasePtr robot;
    string _strRRTPlannerName;
    string _strRobotName; ///< name of the active robot
};

#endif
