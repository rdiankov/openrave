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

#include "dualcommonmanipulation.h"

class DualManipulation : public ModuleBase
{
public:
    DualManipulation(EnvironmentBasePtr penv) : ModuleBase(penv) {
        __description = ":Interface Author: Achint Aggarwal\n\nInterface for planners using more than one manipulator simultaneously.";
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

    virtual ~DualManipulation() {
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
            planner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
        if( !planner ) {
            _strRRTPlannerName = "BiRRT";
            planner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);
            if( !planner )
                _strRRTPlannerName = "";
        }

        RAVELOG_DEBUG(str(boost::format("DualManipulation: using %s planner\n")%_strRRTPlannerName));
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
        return ModuleBase::SendCommand(sout,sinput);
    }
protected:

    inline boost::shared_ptr<DualManipulation> shared_problem() {
        return boost::dynamic_pointer_cast<DualManipulation>(shared_from_this());
    }
    inline boost::shared_ptr<DualManipulation const> shared_problem_const() const {
        return boost::dynamic_pointer_cast<DualManipulation const>(shared_from_this());
    }

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

        if(( index >= 0) &&( index < (int)robot->GetManipulators().size()) ) {
            robot->SetActiveManipulator(index);
            sout << "1";
        }
        else {
            RAVELOG_ERROR("invaild manip %d\n", index);
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
        if( !!robot ) {
            RAVELOG_DEBUG("Releasing all bodies\n");
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
        params->_nMaxIterations = 4000;     // max iterations before failure

        // constraint stuff
        double constrainterrorthresh=0;
        RobotBase::ManipulatorPtr pmanipI, pmanipA;
        int activeManipIndex=robot->GetActiveManipulatorIndex();
        if (activeManipIndex==0) {
            pmanipA=robot->GetManipulators().at(0);    //Active Manipulator
            pmanipI=robot->GetManipulators().at(1);     //Inactive Manipulator
        }
        else{
            pmanipI =robot->GetManipulators().at(0);
            pmanipA =robot->GetManipulators().at(1);
        }

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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "constrainterrorthresh" )
                sinput >> constrainterrorthresh;
            else if( cmd == "manipA" ) {
                string name;
                sinput >> name;
                FOREACH(it, robot->GetManipulators()) {
                    if( (*it)->GetName() == name ) {
                        pmanipA = *it;
                        break;
                    }
                }
            }
            else if( cmd == "manipI" ) {
                string name;
                sinput >> name;
                FOREACH(it, robot->GetManipulators()) {
                    if( (*it)->GetName() == name ) {
                        pmanipI = *it;
                        break;
                    }
                }
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

        if( (int)params->vgoalconfig.size() != robot->GetActiveDOF() )
            return false;

        RobotBase::RobotStateSaver saver(robot);

        if( !planningutils::JitterActiveDOF(robot) ) {
            RAVELOG_WARN("failed\n");
            return false;
        }

        // restore
        params->SetRobotActiveJoints(robot);
        robot->GetActiveDOFValues(params->vinitialconfig);
        robot->SetActiveDOFValues(params->vgoalconfig);

        // jitter again for goal
        if( !planningutils::JitterActiveDOF(robot) ) {
            RAVELOG_WARN("failed\n");
            return false;
        }

        //set to initial config again before initializing the constraint fn
        robot->SetActiveDOFValues(params->vinitialconfig);

        if( constrainterrorthresh > 0 ) {
            RAVELOG_DEBUG("setting DualArmConstrained function in planner parameters\n");
            boost::shared_ptr<CM::DualArmManipulation<double> > dplanner(new CM::DualArmManipulation<double>(robot,pmanipA,pmanipI));
            dplanner->_distmetricfn = params->_distmetricfn;
            params->_neighstatefn = boost::bind(&CM::DualArmManipulation<double>::DualArmConstrained,dplanner,_1,_2);
            params->_nMaxIterations = 1000;
        }

        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF()));
        Trajectory::TPOINT pt;

        robot->SetActiveDOFValues(params->vinitialconfig);
        pt.q = params->vinitialconfig;
        ptraj->AddPoint(pt);

        boost::shared_ptr<PlannerBase> rrtplanner = RaveCreatePlanner(GetEnv(),_strRRTPlannerName);

        if( !rrtplanner ) {
            RAVELOG_ERROR("failed to create BiRRTs\n");
            return false;
        }
        RAVELOG_DEBUG("starting planning\n");

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
                RAVELOG_WARN("PlanPath failed\n");
        }

        rrtplanner.reset();     // have to destroy before environment

        if( !bSuccess )
            return false;

        RAVELOG_DEBUG("finished planning\n");
        CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream);
        return true;
    }

    bool MoveBothHandsStraight(ostream& sout, istream& sinput)
    {
        float stepsize = 0.003f;
        Vector direction0 = Vector(0,1,0);
        Vector direction1 = Vector(0,1,0);
        string strtrajfilename;
        bool bExecute = true;
        bool reached0 = false;
        bool reached1 = false;

        int minsteps = 0;
        int maxsteps = 10000;

        RobotBase::ManipulatorConstPtr pmanip0 = robot->GetManipulators().at(0);
        RobotBase::ManipulatorConstPtr pmanip1 = robot->GetManipulators().at(1);

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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            else if( cmd == "maxsteps")
                sinput >> maxsteps;
            else if( cmd == "stepsize")
                sinput >> stepsize;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj")
                sinput >> strtrajfilename;
            else if( cmd == "direction0")
                sinput >> direction0.x >> direction0.y >> direction0.z;
            else if( cmd == "direction1")
                sinput >> direction1.x >> direction1.y >> direction1.z;
            else if( cmd == "ignorefirstcollision")
                sinput >> bIgnoreFirstCollision;
            else if( cmd == "manip0" ) {
                string name;
                sinput >> name;
                FOREACH(it, robot->GetManipulators()) {
                    if( (*it)->GetName() == name ) {
                        pmanip0 = *it;
                        break;
                    }
                }
            }
            else if( cmd == "manip1" ) {
                string name;
                sinput >> name;
                FOREACH(it, robot->GetManipulators()) {
                    if( (*it)->GetName() == name ) {
                        pmanip1 = *it;
                        break;
                    }
                }
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

        RAVELOG_DEBUG("Starting MoveBothHandsStraight dir0=(%f,%f,%f)...dir1=(%f,%f,%f)...\n",(float)direction0.x, (float)direction0.y, (float)direction0.z, (float)direction1.x, (float)direction1.y, (float)direction1.z);
        robot->RegrabAll();

        RobotBase::RobotStateSaver saver(robot);

        //robot->SetActiveDOFs(pmanip->GetArmIndices());
        planningutils::JitterActiveDOF(robot,100);     // try to jitter out, don't worry if it fails

        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),robot->GetActiveDOF()));
        Trajectory::TPOINT point;
        vector<dReal> vPrevValues,v0Joints,v1Joints;
        bool bPrevInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot))||robot->CheckSelfCollision();
        robot->GetActiveDOFValues(vPrevValues);

        if( bPrevInCollision && !bIgnoreFirstCollision ) {
            RAVELOG_WARN("MoveBothHandsStraight: robot in collision\n");
            return false;
        }

        Transform handTr0 = pmanip0->GetTransform();
        Transform handTr1 = pmanip1->GetTransform();

        point.q = vPrevValues;
        ptraj->AddPoint(point);

        int i;
        for (i = 0; i < maxsteps; i++) {
            if (!reached0) {
                handTr0.trans += stepsize*direction0;
                if( !pmanip0->FindIKSolution(handTr0,v0Joints,false)) {
                    RAVELOG_DEBUG("Arm 0 Lifting: broke due to ik\n");
                    reached0=true;
                }
                else {
                    int index = 0;
                    FOREACHC(it, pmanip0->GetArmIndices())
                    point.q.at(*it) = v0Joints.at(index++);
                }
            }

            if (!reached1) {
                handTr1.trans += stepsize*direction1;
                if( !pmanip1->FindIKSolution(handTr1,v1Joints,false)) {
                    RAVELOG_DEBUG("Arm 1 Lifting: broke due to ik\n");
                    reached1=true;
                }
                else  {
                    int index = 0;
                    FOREACHC(it, pmanip1->GetArmIndices())
                    point.q.at(*it) = v1Joints.at(index++);
                }
            }

            size_t j = 0;
            for(; j < point.q.size(); j++) {
                if(RaveFabs(point.q[j] - vPrevValues[j]) > 0.2) {
                    RAVELOG_DEBUG("Breaking here %d\n",j);
                    break;
                }
            }

            if( j < point.q.size()) {
                RAVELOG_DEBUG("Arm Lifting: broke due to discontinuity\n");
                break;
            }

            robot->SetActiveDOFValues(point.q);

            bool bInCollision = GetEnv()->CheckCollision(KinBodyConstPtr(robot))||robot->CheckSelfCollision();
            if(bInCollision && !bPrevInCollision &&( i >= minsteps) ) {
                RAVELOG_DEBUG("Arm Lifting: broke due to collision\n");
                break;
            }


            ptraj->AddPoint(point);
            vPrevValues = point.q;
            bPrevInCollision = bInCollision;
        }

        if( i > 0 ) {
            if( bPrevInCollision ) {
                RAVELOG_DEBUG("hand failed to move out of collision\n");
                return false;
            }
            RAVELOG_DEBUG("hand moved %f\n", (float)i*stepsize);
            CM::SetActiveTrajectory(robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream);
            sout << "1";
            return i >= minsteps;
        }

        RAVELOG_DEBUG("hand didn't move\n");
        return i >= minsteps;
    }

    RobotBasePtr robot;
    string _strRRTPlannerName;
    string _strRobotName;     ///< name of the active robot
};

#endif
