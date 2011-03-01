// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_TASK_MANIPULATION_PROBLEM
#define OPENRAVE_TASK_MANIPULATION_PROBLEM

#include "commonmanipulation.h"

#ifdef HAVE_BOOST_REGEX
#include <boost/regex.hpp>
#endif

#define GRASPTHRESH2 dReal(0.002f)

struct GRASPGOAL
{
    vector<dReal> vpreshape;
    Transform tgrasp; ///< transform of the grasp
    vector<dReal> viksolution; ///< optional joint values for robot arm that achive the grasp goal
    list<TransformMatrix> listDests; ///< transform of the grasps at the destination
    int index; ///< index into global grasp table
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(GRASPGOAL)
#endif

class GraspVectorCompare : public RealVectorCompare
{
 public:
 GraspVectorCompare() : RealVectorCompare(GRASPTHRESH2) {}
};

class TaskManipulation : public ProblemInstance
{
 public:
    typedef std::map<vector<dReal>, TrajectoryBasePtr, GraspVectorCompare > PRESHAPETRAJMAP;

 TaskManipulation(EnvironmentBasePtr penv) : ProblemInstance(penv) {
        __description = ":Interface Author: Rosen Diankov\n\n\
Task-based manipulation planning involving target objects. A lot of the algorithms and theory are covered in:\n\
\n\
- Rosen Diankov. \"Automated Construction of Robotic Manipulation Programs\". PhD Thesis, CMU-RI-TR-10-29, Carnegie Mellon University, Robotics Institute, August 2010.\n";
        RegisterCommand("GraspPlanning",boost::bind(&TaskManipulation::GraspPlanning,this,_1,_2),
                        "Grasp planning is the ultimate function that puts together many planners achieving a robust and general pick and place motiotions with grasp sets. It first chooses grasps from a grasp set and use it for manipulation. In order for the arm to reach the grasps, it must have a Transform6D IK solver, anything with less DOF will not work.\nParameters:\n\n\
* grasps\n\
* outputtraj\n\
* execute\n\
* randomdests\n\
* writetraj\n\
* maxiter\n\
* graspindices\n\
* igraspdir\n\
* igrasppos\n\
* igrasproll\n\
* igraspstandoff\n\
* igrasppreshape\n\
* igrasptrans\n\
* target\n\
* approachoffset\n\
* quitafterfirstrun\n\
* matdests\n\
* posedests\n\
* seedgrasps\n\
* seeddests\n\
* seedik\n\
* savepreshapetraj\n\
");
        RegisterCommand("GraspPlanningAsCylinders",boost::bind(&TaskManipulation::GraspPlanningAsCylinders,this,_1,_2),
                        "Performs grasp planning treating the targets as cylinders (axis is along the target local z), this allows TranslationDirection5D IK to be used for arms with >= 5 joints");
        RegisterCommand("CloseFingers",boost::bind(&TaskManipulation::CloseFingers,this,_1,_2),
                        "Closes the active manipulator fingers using the grasp planner.");
        RegisterCommand("ReleaseFingers",boost::bind(&TaskManipulation::ReleaseFingers,this,_1,_2),
                        "Releases the active manipulator fingers using the grasp planner.\n"
                        "Also releases the given object.");
        RegisterCommand("ReleaseActive",boost::bind(&TaskManipulation::ReleaseActive,this,_1,_2),
                        "Moves the active DOF using the grasp planner.");
        RegisterCommand("CreateSystem",boost::bind(&TaskManipulation::CreateSystem,this,_1,_2),
                        "creates a sensor system and initializes it with the current bodies");
        RegisterCommand("EvaluateConstraints",boost::bind(&TaskManipulation::EvaluateConstraints,this,_1,_2),
                        "Instantiates a jacobian constraint function and runs it on several examples.\n"
                        "The constraints work on the active degress of freedom of the manipulator starting from the current configuration");
#ifdef HAVE_BOOST_REGEX
        RegisterCommand("SwitchModels",boost::bind(&TaskManipulation::SwitchModels,this,_1,_2),
                        "Switches between thin and fat models for planning.");
#endif
        _fMaxVelMult=1;
    }
    virtual ~TaskManipulation()
    {
        Destroy();
    }

    virtual void Destroy()
    {
        ProblemInstance::Destroy();
        listsystems.clear();
        _pGrasperPlanner.reset();
        _pRRTPlanner.reset();
        _robot.reset();
    }

    virtual void Reset()
    {
        _listSwitchModels.clear();
        ProblemInstance::Reset();
        listsystems.clear();
        // recreate the planners since they store state
        if( !!_pRRTPlanner )
            _pRRTPlanner = RaveCreatePlanner(GetEnv(),_pRRTPlanner->GetXMLId());
        if( !!_pGrasperPlanner )
            _pGrasperPlanner = RaveCreatePlanner(GetEnv(),_pGrasperPlanner->GetXMLId());
    }

    int main(const string& args)
    {
        string name;
        stringstream ss(args);
        _fMaxVelMult=1;
        ss >> _strRobotName;
    
        string plannername, graspername = "Grasper";
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "planner" ) {
                ss >> plannername;
            }
            else if( cmd == "maxvelmult" ) {
                ss >> _fMaxVelMult;
            }
            else if( cmd == "graspername" ) {
                ss >> graspername;
            }

            if( ss.fail() || !ss ) {
                break;
            }
        }

        if( plannername.size() > 0 ) {
            _pRRTPlanner = RaveCreatePlanner(GetEnv(),plannername);
        }
        if( !_pRRTPlanner ) {
            plannername = "BiRRT";
            _pRRTPlanner = RaveCreatePlanner(GetEnv(),plannername);
        }

        if( !_pRRTPlanner ) {
            RAVELOG_WARN("could not find an rrt planner\n");
            return -1;
        }
        RAVELOG_DEBUG(str(boost::format("using %s planner\n")%plannername));

        _pGrasperPlanner = RaveCreatePlanner(GetEnv(),graspername);
        if( !_pGrasperPlanner ) {
            RAVELOG_WARN(str(boost::format("Failed to create a grasper planner %s\n")%graspername));
        }
        _robot = GetEnv()->GetRobot(_strRobotName);
        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = GetEnv()->GetRobot(_strRobotName);
        return ProblemInstance::SendCommand(sout,sinput);
    }

    bool CreateSystem(ostream& sout, istream& sinput)
    {
        string systemname;
        sinput >> systemname;
        if( !sinput ) {
            return false;
        }
        SensorSystemBasePtr psystem = RaveCreateSensorSystem(GetEnv(),systemname);
        if( !psystem ) {
            return false;
        }
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        psystem->AddRegisteredBodies(vbodies);
        listsystems.push_back(psystem);

        RAVELOG_DEBUG(str(boost::format("added %s system\n")%systemname));
        sout << 1; // signal success
        return true;
    }

    class ActiveDistMetric
    {
    public:
    ActiveDistMetric(RobotBasePtr robot) : _robot(robot) {
            _robot->GetActiveDOFWeights(weights);
        }
        virtual dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1)
        {
            dReal out = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++)
                out += weights.at(i) * (c0.at(i)-c1.at(i))*(c0[i]-c1[i]);    
            return RaveSqrt(out);
        }

    protected:
        RobotBasePtr _robot;
        vector<dReal> weights;
    };

    bool EvaluateConstraints(ostream& sout, istream& sinput)
    {
        Transform tTargetWorldFrame;
        boost::array<double,6> vfreedoms = {{1,1,1,1,1,1}};
        string cmd;
        list< vector<dReal> > listconfigs;
        double errorthresh=1e-3;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "constraintfreedoms" ) {
                FOREACH(it,vfreedoms) {
                    sinput >> *it;
                }
            }
            else if( cmd == "constraintmatrix" ) {
                TransformMatrix m; sinput >> m; tTargetWorldFrame = m;
            }
            else if( cmd == "constraintpose" )
                sinput >> tTargetWorldFrame;
            else if( cmd == "config" ) {
                vector<dReal> vconfig(_robot->GetActiveDOF());
                FOREACH(it,vconfig) {
                    sinput >> *it;
                }
                listconfigs.push_back(vconfig);
            }
            else if( cmd == "constrainterrorthresh" ) {
                sinput >> errorthresh;
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

        RobotBase::RobotStateSaver saver(_robot);

        ActiveDistMetric distmetric(_robot);
        vector<dReal> vprev;
        _robot->GetActiveDOFValues(vprev);
        CM::GripperJacobianConstrains<double> constraints(_robot->GetActiveManipulator(),tTargetWorldFrame,vfreedoms,errorthresh);
        constraints._distmetricfn = boost::bind(&ActiveDistMetric::Eval,&distmetric,_1,_2);
        FOREACH(itconfig,listconfigs) {
            _robot->SetActiveDOFValues(*itconfig);
            constraints.RetractionConstraint(vprev,*itconfig,0);
            sout << constraints._iter << " ";
        }
        FOREACH(itconfig,listconfigs) {
            FOREACH(it,*itconfig)
                sout << *it << " ";
        }
        
        return true;
    }

    bool GraspPlanning(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("GraspPlanning...\n");
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();

        vector<dReal> vgrasps;
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));

        KinBodyPtr ptarget;
        int nNumGrasps=0, nGraspDim=0;
        dReal fApproachOffset=0.0f; // offset before approaching to the target
        string targetname;
        vector<Transform> vObjDestinations;
        string strpreshapetraj; // save the preshape trajectory
        bool bCombinePreShapeTraj = true;
        bool bExecute = true;
        string strtrajfilename;
        bool bRandomDests = true, bRandomGrasps = true; // if true, permute the grasps and destinations when iterating through them
        boost::shared_ptr<ostream> pOutputTrajStream;
        int nMaxSeedGrasps = 20, nMaxSeedDests = 5, nMaxSeedIkSolutions = 0;
        int nMaxIterations = 4000;
        bool bQuitAfterFirstRun = false;

        // indices into the grasp table
        int iGraspDir = -1, iGraspPos = -1, iGraspRoll = -1, iGraspPreshape = -1, iGraspStandoff = -1;
        int iGraspTransform = -1; // if >= 0, use the grasp transform to check for collisions

        string cmd;
        CollisionReportPtr report(new CollisionReport);
    
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "grasps" ) {
                sinput >> nNumGrasps >> nGraspDim;
                vgrasps.resize(nNumGrasps*nGraspDim);
                FOREACH(it, vgrasps) {
                    sinput >> *it;
                }
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "randomdests" ) {
                sinput >> bRandomDests;
            }
            else if( cmd == "randomgrasps" ) {
                sinput >> bRandomGrasps;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "maxiter" ) {
                sinput >> nMaxIterations;
            }
            else if( cmd == "graspindices" ) {
                sinput >> iGraspDir >> iGraspPos >> iGraspRoll >> iGraspStandoff >> iGraspPreshape;
            }
            else if( cmd == "igraspdir" ) {
                sinput >> iGraspDir;
            }
            else if( cmd == "igrasppos" ) {
                sinput >> iGraspPos;
            }
            else if( cmd == "igrasproll" ) {
                sinput >> iGraspRoll;
            }
            else if( cmd == "igraspstandoff" ) {
                sinput >> iGraspStandoff;
            }
            else if( cmd == "igrasppreshape" ) {
                sinput >> iGraspPreshape;
            }
            else if( cmd == "igrasptrans" ) {
                sinput >> iGraspTransform;
            }
            else if( cmd == "target" ) {
                sinput >> targetname;
                ptarget = GetEnv()->GetKinBody(targetname);
            }
            else if( cmd == "approachoffset" ) {
                sinput >> fApproachOffset;
            }
            else if( cmd == "quitafterfirstrun" ) {
                bQuitAfterFirstRun = true;
            }
            else if( cmd == "matdests" ) {
                int numdests = 0; sinput >> numdests;
                vObjDestinations.resize(numdests);
                FOREACH(ittrans, vObjDestinations) {
                    TransformMatrix m; sinput >> m;
                    *ittrans = m;
                }
            }
            else if( cmd == "posedests" ) {
                int numdests = 0; sinput >> numdests;
                vObjDestinations.resize(numdests);
                FOREACH(ittrans, vObjDestinations) {
                    sinput >> *ittrans;
                }
            }
            else if( cmd == "seedgrasps" ) {
                sinput >> nMaxSeedGrasps;
            }
            else if( cmd == "seeddests" ) {
                sinput >> nMaxSeedDests;
            }
            else if( cmd == "seedik" ) {
                sinput >> nMaxSeedIkSolutions;
            }
            else if( cmd == "savepreshapetraj" ) {
                sinput >> strpreshapetraj;
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
    
        if( !ptarget ) {
            RAVELOG_WARN(str(boost::format("Could not find target %s\n")%targetname));
            return false;
        }
    
        if( pmanip->IsGrabbing(ptarget) ) {
            throw openrave_exception(str(boost::format("TaskManipulaiton::GraspPlanning: manipulator %s is already grasping %s")%pmanip->GetName()%ptarget->GetName()));
        }
        if(!pmanip->GetIkSolver()->Supports(IkParameterization::Type_Transform6D)) {
            throw openrave_exception(str(boost::format("TaskManipulaiton::GraspPlanning: manipulator %s does not support Transform6D IK type")%pmanip->GetName()));
        }
        RobotBase::RobotStateSaver saver(_robot);

        bool bMobileBase = _robot->GetAffineDOF()!=0; // if mobile base, cannot rely on IK
        if( bMobileBase ) {
            RAVELOG_INFO("planning with mobile base!\n");
        }
        bool bInitialRobotChanged = false;
        vector<dReal> vCurHandValues, vCurRobotValues, vOrgRobotValues;
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());
        _robot->GetActiveDOFValues(vCurHandValues);
        _robot->GetDOFValues(vOrgRobotValues);

        SwitchModelState switchstate(shared_problem());
        _UpdateSwitchModels(true,true);
        // check if robot is in collision with padded models
        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) ) {
            _robot->SetActiveDOFs(pmanip->GetArmIndices());
            if( !CM::JitterActiveDOF(_robot) ) {
                RAVELOG_ERROR("failed to jitter robot\n");
                return false;
            }
            bInitialRobotChanged = true;
        }
        
        _robot->GetDOFValues(vCurRobotValues);

        string strResponse;
        Transform transTarg = ptarget->GetTransform();
        Transform transInvTarget = transTarg.inverse();
        Transform transDummy(Vector(1,0,0,0), Vector(100,0,0));
        vector<dReal> viksolution, vikgoal, vFinalGripperValues;
        
        PRESHAPETRAJMAP mapPreshapeTrajectories;
        {
            // fill with a trajectory with one point
            TrajectoryBasePtr pstarttraj = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());
            Trajectory::TPOINT tpstarthand;
            _robot->GetDOFValues(tpstarthand.q);
            tpstarthand.trans = _robot->GetTransform();
            pstarttraj->AddPoint(tpstarthand);
            mapPreshapeTrajectories[vCurHandValues] = pstarttraj;
        }

        TrajectoryBasePtr ptraj;
        GRASPGOAL goalFound;
        int iCountdown = 0;
        uint64_t nSearchTime = 0;

        list<GRASPGOAL> listGraspGoals;

        vector<int> vdestpermuation(vObjDestinations.size());
        for(int i = 0; i < (int)vObjDestinations.size(); ++i) {
            vdestpermuation[i] = i;
        }
        vector<int> vgrasppermuation(nNumGrasps);
        for(int i = 0; i < nNumGrasps; ++i) {
            vgrasppermuation[i] = i;
        }
        if( bRandomGrasps ) {
            PermutateRandomly(vgrasppermuation);
        }
        if( iGraspTransform < 0 ) {
            if( !_pGrasperPlanner ) {
                RAVELOG_ERROR("grasper problem not valid\n");
                return false;
            }
            if( iGraspDir < 0 || iGraspPos < 0 || iGraspRoll < 0 || iGraspStandoff < 0 ) {
                RAVELOG_ERROR("grasp indices not all initialized\n");
                return false;
            }
        }

        TrajectoryBasePtr phandtraj;
    
        for(int igraspperm = 0; igraspperm < (int)vgrasppermuation.size(); ++igraspperm) {
            int igrasp = vgrasppermuation[igraspperm];
            dReal* pgrasp = &vgrasps[igrasp*nGraspDim];

            if( listGraspGoals.size() > 0 && iCountdown-- <= 0 ) {
                // start planning
                _UpdateSwitchModels(true,false);

                RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
                uint64_t basestart = GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, nMaxSeedIkSolutions, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += GetMicroTime() - basestart;

                if( !!ptraj || bQuitAfterFirstRun )
                    break;
            }

            vector<dReal> vgoalpreshape(vCurHandValues.size());
            if( iGraspPreshape >= 0 ) {
                for(size_t j = 0; j < vCurHandValues.size(); ++j) {
                    vgoalpreshape[j] = pgrasp[iGraspPreshape+j];
                }
            }
            else {
                vgoalpreshape.resize(pmanip->GetGripperIndices().size());
                for(size_t j = 0; j < pmanip->GetGripperIndices().size(); ++j) {
                    vgoalpreshape[j] = vCurRobotValues[pmanip->GetGripperIndices()[j]];
                }
            }

            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vgoalpreshape);
            if( itpreshapetraj != mapPreshapeTrajectories.end() && !itpreshapetraj->second ) {
                // has failed to find a trajectory to open a hand on a previous attempt, so skip
                RAVELOG_DEBUG("grasp %d: skipping failed preshape\n", igrasp);
                continue;
            }

            _UpdateSwitchModels(false,false);

            Transform tGoalEndEffector;
            // set the goal preshape
            _robot->SetActiveDOFs(pmanip->GetGripperIndices(), RobotBase::DOF_NoTransform);
            _robot->SetActiveDOFValues(vgoalpreshape,true);
        
            if( !!_pGrasperPlanner ) {
                // set the preshape
                _robot->SetActiveDOFs(pmanip->GetGripperIndices(), RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z);
                if( !phandtraj ) {
                    phandtraj = RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF());
                }
                else {
                    phandtraj->Reset(_robot->GetActiveDOF());
                }
                graspparams->fstandoff = pgrasp[iGraspStandoff];
                graspparams->targetbody = ptarget;
                graspparams->ftargetroll = pgrasp[iGraspRoll];
                graspparams->vtargetdirection = Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
                graspparams->vtargetposition = Vector(pgrasp[iGraspPos], pgrasp[iGraspPos+1], pgrasp[iGraspPos+2]);
                graspparams->btransformrobot = true;
                graspparams->breturntrajectory = false;
                graspparams->bonlycontacttarget = true;
                graspparams->btightgrasp = false;
                graspparams->bavoidcontact = true;
                // TODO: in order to reproduce the same exact conditions as the original grasp, have to also transfer the step sizes

                if( !_pGrasperPlanner->InitPlan(_robot,graspparams) ) {
                    RAVELOG_DEBUG("grasper planner failed: %d\n", igrasp);
                    continue; // failed
                }

                if( !_pGrasperPlanner->PlanPath(phandtraj) ) {
                    RAVELOG_DEBUG("grasper planner failed: %d\n", igrasp);
                    continue; // failed
                }

                BOOST_ASSERT(phandtraj->GetPoints().size()>0);
                Transform t = phandtraj->GetPoints().back().trans;

                // move back a little if robot/target in collision
                if( !!ptarget ) {
                    RobotBase::RobotStateSaver saverlocal(_robot);
                    _robot->SetTransform(t);
                    Vector vglobalpalmdir;
                    if( iGraspDir >= 0 ) {
                        vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                    }
                    else {
                        vglobalpalmdir = pmanip->GetEndEffectorTransform().rotate(pmanip->GetDirection());
                    }

                    while(GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(ptarget))) {
                        t.trans -= vglobalpalmdir*0.001f;
                        _robot->SetTransform(t);
                    }
                }

                tGoalEndEffector = t * _robot->GetTransform().inverse() * pmanip->GetEndEffectorTransform(); // find the end effector transform
                vFinalGripperValues.resize(pmanip->GetGripperIndices().size());
                std::copy(phandtraj->GetPoints().back().q.begin(),phandtraj->GetPoints().back().q.begin()+vFinalGripperValues.size(),vFinalGripperValues.begin());
            }
            else if( iGraspTransform >= 0 ) {
                // use the grasp transform
                dReal* pm = pgrasp+iGraspTransform;
                TransformMatrix tm;
                tm.m[0] = pm[0]; tm.m[1] = pm[3]; tm.m[2] = pm[6]; tm.trans.x = pm[9];
                tm.m[4] = pm[1]; tm.m[5] = pm[4]; tm.m[6] = pm[7]; tm.trans.y = pm[10];
                tm.m[8] = pm[2]; tm.m[9] = pm[5]; tm.m[10] = pm[8]; tm.trans.z = pm[11];
                if( !ptarget ) {
                    tGoalEndEffector = tm;
                }
                else {
                    tGoalEndEffector = ptarget->GetTransform() * Transform(tm);
                }

                if( pmanip->CheckEndEffectorCollision(tGoalEndEffector,report) ) {
                    RAVELOG_DEBUG(str(boost::format("grasp %d: in collision (%s)\n")%igrasp%report->__str__()));
                    continue;
                }

                vFinalGripperValues.resize(0);
            }
            else {
                RAVELOG_ERROR("grasper problem not valid\n");
                return false;
            }

            // set the initial hand joints
            _robot->SetActiveDOFs(pmanip->GetGripperIndices());
            if( iGraspPreshape >= 0 ) {
                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
            }

            Transform tApproachEndEffector = tGoalEndEffector;
            if( !bMobileBase ) {
                // check ik
                Vector vglobalpalmdir;
                if( iGraspDir >= 0 ) {
                    vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                }
                else {
                    vglobalpalmdir = tApproachEndEffector.rotate(pmanip->GetDirection());
                }

                dReal fSmallOffset = 0.0001f;
                tApproachEndEffector.trans -= fSmallOffset * vglobalpalmdir;

                // first test the IK solution at the destination tGoalEndEffector
                if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, true) ) {
                    RAVELOG_DEBUG("grasp %d: No IK solution found (final)\n", igrasp);
                    continue;
                }

                _UpdateSwitchModels(true,true); // switch to fat models
                if( fApproachOffset != 0 ) {
                    // now test at the approach point (with offset)
                    tApproachEndEffector.trans -= (fApproachOffset-fSmallOffset) * vglobalpalmdir;
                    
                    // set the previous robot ik configuration to get the closest configuration!!
                    _robot->SetActiveDOFs(pmanip->GetArmIndices());
                    _robot->SetActiveDOFValues(viksolution);
                    if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, true) ) {
                        _robot->SetDOFValues(vCurRobotValues); // reset robot to original position
                        RAVELOG_DEBUG("grasp %d: No IK solution found (approach)\n", igrasp);
                        continue;
                    }
                    else {
                        stringstream ss;
                        ss << "IK found: "; 
                        FOREACH(it, viksolution) {
                            ss << *it << " ";
                        }
                        ss << endl;
                        RAVELOG_DEBUG(ss.str());
                    }
                }
                else {
                    // revert back?
                    //tApproachEndEffector.trans += fSmallOffset * vglobalpalmdir;
                }
            }

            // set the joints that the grasper plugin calculated
            _robot->SetActiveDOFs(pmanip->GetGripperIndices());
            if( vFinalGripperValues.size() > 0 )
                _robot->SetActiveDOFValues(vFinalGripperValues, true);

            //while (getchar() != '\n') usleep(1000);
            _UpdateSwitchModels(false,false); // should test destination with thin models

            list< TransformMatrix > listDests;
            if( bRandomDests ) {
                PermutateRandomly(vdestpermuation);
            }

            for(int idestperm = 0; idestperm < (int)vdestpermuation.size(); ++idestperm) {
                Transform& transDestTarget = vObjDestinations[vdestpermuation[idestperm]];
                Transform tDestEndEffector = transDestTarget * transInvTarget * tGoalEndEffector;
                ptarget->SetTransform(transDestTarget);
                bool bTargetCollision; 
                {
                    RobotBase::RobotStateSaver linksaver(_robot,KinBody::Save_LinkEnable);
                    _robot->Enable(false); // remove robot from target collisions 
                    bTargetCollision = GetEnv()->CheckCollision(KinBodyConstPtr(ptarget));
                }

                ptarget->SetTransform(transTarg);
                if( bTargetCollision ) {
                    RAVELOG_VERBOSE("target collision at dest\n");
                    continue;
                }
                
                if( !bMobileBase ) {
                    if( pmanip->FindIKSolution(tDestEndEffector, vikgoal, true) ) {
                        listDests.push_back(tDestEndEffector);
                    }
                }
                else
                    listDests.push_back(tDestEndEffector);

                if( (int)listDests.size() >= nMaxSeedDests )
                    break;
            }

            _robot->SetDOFValues(vCurRobotValues); // reset robot to original position

            if( vObjDestinations.size() > 0 && listDests.size() == 0 ) {
                RAVELOG_WARN("grasp %d: could not find destination\n", igrasp);
                continue;
            }

            // finally start planning
            _UpdateSwitchModels(true,false);

            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                // not present in map, so look for correct one
                // initial joint is far from desired preshape, have to plan to get to it
                // note that this changes trajectory of robot!
                _robot->SetActiveDOFValues(vCurHandValues, true);
                
                _robot->SetActiveDOFs(pmanip->GetArmIndices());
                TrajectoryBasePtr ptrajToPreshape = RaveCreateTrajectory(GetEnv(),pmanip->GetArmIndices().size());
                bool bSuccess = CM::MoveUnsync::_MoveUnsyncJoints(GetEnv(), _robot, ptrajToPreshape, pmanip->GetGripperIndices(), vgoalpreshape);
                
                if( !bSuccess ) {
                    mapPreshapeTrajectories[vgoalpreshape].reset(); // set to empty
                    RAVELOG_DEBUG("grasp %d: failed to find preshape\n", igrasp);
                    continue;
                }

                // get the full trajectory
                TrajectoryBasePtr ptrajToPreshapeFull = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());
                _robot->GetFullTrajectoryFromActive(ptrajToPreshapeFull, ptrajToPreshape);

                // add a grasp with the full preshape
                Trajectory::TPOINT tpopenhand;
                BOOST_ASSERT(ptrajToPreshapeFull->GetPoints().size()>0);
                _robot->SetDOFValues(ptrajToPreshapeFull->GetPoints().back().q);
                _robot->SetActiveDOFs(pmanip->GetGripperIndices());
                if( iGraspPreshape >= 0 ) {
                    _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
                }
                _robot->GetDOFValues(tpopenhand.q);
                tpopenhand.trans = _robot->GetTransform();
                ptrajToPreshapeFull->AddPoint(tpopenhand);
                ptrajToPreshapeFull->CalcTrajTiming(_robot, ptrajToPreshape->GetInterpMethod(), true, false,_fMaxVelMult);

                mapPreshapeTrajectories[vgoalpreshape] = ptrajToPreshapeFull;
            }

            if( iGraspPreshape >= 0 ) {
                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
            }
            listGraspGoals.push_back(GRASPGOAL());
            GRASPGOAL& goal = listGraspGoals.back();
            goal.index = igrasp;
            goal.tgrasp = tApproachEndEffector;
            goal.viksolution = viksolution;
            goal.listDests.swap(listDests);
            goal.vpreshape.resize(pmanip->GetGripperIndices().size());
            if( iGraspPreshape >= 0 ) {
                for(int j = 0; j < (int)goal.vpreshape.size(); ++j)
                    goal.vpreshape[j] = pgrasp[iGraspPreshape+j];
            }

            RAVELOG_DEBUG("grasp %d: adding to goals\n", igrasp);
            iCountdown = 40;

            if( (int)listGraspGoals.size() >= nMaxSeedGrasps ) {
                RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
                uint64_t basestart = GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += GetMicroTime() - basestart;

                if( bQuitAfterFirstRun )
                    break;
            }

            if( !!ptraj )
                break;
        }

        // if there's left over goal positions, start planning
        while( !ptraj && listGraspGoals.size() > 0 ) {
            //TODO have to update ptrajToPreshape
            RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
            uint64_t basestart = GetMicroTime();
            ptraj = _PlanGrasp(listGraspGoals, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
            nSearchTime += GetMicroTime() - basestart;
        }

        _UpdateSwitchModels(false,false);

        if( !!ptraj ) {
            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(goalFound.vpreshape);
            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                RAVELOG_ERROR("no preshape trajectory!\n");
                FOREACH(itpreshape,mapPreshapeTrajectories) {
                    RAVELOG_ERROR("%f %f %f %f %f %f\n",itpreshape->first[0],itpreshape->first[1],itpreshape->first[2],itpreshape->first[3],itpreshape->first[4],itpreshape->first[5]);
                }
                return false;
            }

            TrajectoryBasePtr ptrajfinal = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());

            if( bInitialRobotChanged )
                ptrajfinal->AddPoint(Trajectory::TPOINT(vOrgRobotValues,_robot->GetTransform(), 0));

            if( strpreshapetraj.size() > 0 ) { // write the preshape
                ofstream f(strpreshapetraj.c_str());
                itpreshapetraj->second->Write(f, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            }
            if( bCombinePreShapeTraj) { // add the preshape
                FOREACHC(itpoint, itpreshapetraj->second->GetPoints())
                    ptrajfinal->AddPoint(*itpoint);
            }

            FOREACHC(itpoint, ptraj->GetPoints())
                ptrajfinal->AddPoint(*itpoint);
            
            ptrajfinal->CalcTrajTiming(_robot, ptrajfinal->GetInterpMethod(), true, false,_fMaxVelMult);

            RAVELOG_DEBUG("grasp index %d\n",goalFound.index);
            sout << goalFound.listDests.size() << " ";
            FOREACH(itdest, goalFound.listDests)
                sout << *itdest << " ";
            sout << goalFound.index << " " << (float)nSearchTime/1000000.0f << " ";

            // set the trajectory
            CM::SetFullTrajectory(_robot,ptrajfinal, bExecute, strtrajfilename, pOutputTrajStream);
            return true;
        }

        return false; // couldn't not find for this cup
    }

    bool GraspPlanningAsCylinders(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("GraspPlanning...\n");
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));
        KinBodyPtr ptarget;
        dReal fApproachOffset=0.0f; // offset before approaching to the target
        string targetname;
        vector<Transform> vObjDestinations;
        string strpreshapetraj; // save the preshape trajectory
        //bool bCombinePreShapeTraj = true;
        bool bExecute = true;
        string strtrajfilename;
        bool bRandomDests = true; // if true, permute the destinations when iterating through them
        boost::shared_ptr<ostream> pOutputTrajStream;
        int nMaxSeedDests = 5, nMaxSeedIkSolutions = 0;
        int nMaxIterations = 4000;
        bool bQuitAfterFirstRun = false;

        string cmd;
        CollisionReportPtr report(new CollisionReport);
    
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            }
            else if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "randomdests" ) {
                sinput >> bRandomDests;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "maxiter" ) {
                sinput >> nMaxIterations;
            }
            else if( cmd == "target" ) {
                sinput >> targetname;
                ptarget = GetEnv()->GetKinBody(targetname);
            }
            else if( cmd == "approachoffset" ) {
                sinput >> fApproachOffset;
            }
            else if( cmd == "quitafterfirstrun" ) {
                bQuitAfterFirstRun = true;
            }
            else if( cmd == "matdests" ) {
                int numdests = 0; sinput >> numdests;
                vObjDestinations.resize(numdests);
                FOREACH(ittrans, vObjDestinations) {
                    TransformMatrix m; sinput >> m;
                    *ittrans = m;
                }
            }
            else if( cmd == "posedests" ) {
                int numdests = 0; sinput >> numdests;
                vObjDestinations.resize(numdests);
                FOREACH(ittrans, vObjDestinations)
                    sinput >> *ittrans;
            }
            else if( cmd == "seeddests" ) {
                sinput >> nMaxSeedDests;
            }
            else if( cmd == "seedik" ) {
                sinput >> nMaxSeedIkSolutions;
            }
            else if( cmd == "savepreshapetraj" ) {
                sinput >> strpreshapetraj;
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
    
        if( !ptarget ) {
            RAVELOG_WARN(str(boost::format("Could not find target %s\n")%targetname));
            return false;
        }
    
        if( pmanip->IsGrabbing(ptarget) ) {
            throw openrave_exception(str(boost::format("TaskManipulaiton::GraspPlanningAsCylinders: manipulator %s is already grasping %s")%pmanip->GetName()%ptarget->GetName()));
        }
        if(!pmanip->GetIkSolver()->Supports(IkParameterization::Type_TranslationDirection5D)) {
            throw openrave_exception(str(boost::format("TaskManipulaiton::GraspPlanningAsCylinders: manipulator %s does not support Transform6D IK type")%pmanip->GetName()));
        }
        RobotBase::RobotStateSaver saver(_robot);

        bool bMobileBase = _robot->GetAffineDOF()!=0; // if mobile base, cannot rely on IK
        if( bMobileBase ) {
            RAVELOG_INFO("planning with mobile base!\n");
        }
        bool bInitialRobotChanged = false;
        vector<dReal> vCurHandValues, vCurRobotValues, vOrgRobotValues;
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());
        _robot->GetActiveDOFValues(vCurHandValues);
        _robot->GetDOFValues(vOrgRobotValues);

        SwitchModelState switchstate(shared_problem());
        _UpdateSwitchModels(true,true);
        // check if robot is in collision with padded models
        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) ) {
            _robot->SetActiveDOFs(pmanip->GetArmIndices());
            if( !CM::JitterActiveDOF(_robot) ) {
                RAVELOG_ERROR("failed to jitter robot\n");
                return false;
            }

            bInitialRobotChanged = true;
        }
        
        _robot->GetDOFValues(vCurRobotValues);

        string strResponse;
        Transform transTarg = ptarget->GetTransform();
        Transform transInvTarget = transTarg.inverse();
        Transform transDummy(Vector(1,0,0,0), Vector(100,0,0));
        vector<dReal> viksolution, vikgoal, vFinalGripperValues;
        
        return false;
    }

    bool CloseFingers(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting CloseFingers...\n");
        bool bExecute = true, bOutputFinal=false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        Vector direction;
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));
        graspparams->vgoalconfig = pmanip->GetClosingDirection();

        vector<dReal> voffset;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            }
            else if( cmd == "outputfinal" ) {
                bOutputFinal = true;
            }
            else if( cmd == "offset" ) {
                voffset.resize(pmanip->GetGripperIndices().size());
                FOREACH(it, voffset)
                    sinput >> *it;
            }
            else if( cmd == "movingdir" ) {
                FOREACH(it,graspparams->vgoalconfig) {
                    sinput >> *it;
                }
            }
            else if( cmd == "coarsestep" ) {
                sinput >> graspparams->fcoarsestep;
                graspparams->ffinestep = graspparams->fcoarsestep * 0.01; // always make it 100x more accurate
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

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        _robot->GetActiveDOFValues(ptfirst.q);
 
        boost::shared_ptr<PlannerBase> graspplanner = RaveCreatePlanner(GetEnv(),"Grasper");
        if( !graspplanner ) {
            RAVELOG_ERROR("grasping planner failure!\n");
            return false;
        }
    
        graspparams->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(graspparams->vinitialconfig);
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;

        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));
        ptraj->AddPoint(ptfirst);

        if( !graspplanner->InitPlan(_robot, graspparams) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARN("PlanPath failed\n");
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
            _robot->SetActiveDOFValues(p.q,true);
            _robot->GetActiveDOFValues(p.q);
            ptraj->AddPoint(p);
        }

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool ReleaseFingers(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Starting ReleaseFingers...\n");
        bool bExecute = true, bOutputFinal=false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        Vector direction;
        KinBodyPtr ptarget;
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));
        graspparams->vgoalconfig = pmanip->GetClosingDirection();
        FOREACH(it,graspparams->vgoalconfig) {
            *it = -*it;
        }
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            }
            else if( cmd == "outputfinal" ) {
                bOutputFinal = true;
            }
            else if( cmd == "movingdir" ) {
                FOREACH(it,graspparams->vgoalconfig)
                    sinput >> *it;
            }
            else if( cmd == "coarsestep" ) {
                sinput >> graspparams->fcoarsestep;
                graspparams->ffinestep = graspparams->fcoarsestep * 0.01; // always make it 100x more accurate
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

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());
        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));
        // have to add the first point
        Trajectory::TPOINT ptfirst;
        _robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch(CM::JitterActiveDOF(_robot) ) {
        case 0:
            RAVELOG_WARN("robot initially in collision\n");
            return false;
        case 1:
            _robot->GetActiveDOFValues(ptfirst.q);
        default:
            break;
        }
 
        boost::shared_ptr<PlannerBase> graspplanner = RaveCreatePlanner(GetEnv(),"Grasper");
        if( !graspplanner ) {
            RAVELOG_ERROR("grasping planner failure!\n");
            return false;
        }
    
        graspparams->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(graspparams->vinitialconfig);
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;
        graspparams->bavoidcontact = true;

        if( !graspplanner->InitPlan(_robot, graspparams) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARN("PlanPath failed\n");
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
            RobotBase::RobotStateSaver saver2(_robot);
            _robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
            if( CM::JitterActiveDOF(_robot) > 0 ) {
                RAVELOG_WARN("robot final configuration is in collision\n");
                Trajectory::TPOINT pt = ptraj->GetPoints().back();
                _robot->GetActiveDOFValues(pt.q);
                ptraj->AddPoint(pt);
            }
        }

        if( !!ptarget )
            _robot->Release(ptarget);

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);

        return true;
    }

    bool ReleaseActive(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("Releasing active...\n");

        bool bExecute = true, bOutputFinal = false;
        string strtrajfilename;
        boost::shared_ptr<ostream> pOutputTrajStream;
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));

        // initialize the moving direction as the opposite of the closing direction defined in the manipulators
        vector<dReal> vclosingsign_full(_robot->GetDOF(), 0);
        FOREACHC(itmanip, _robot->GetManipulators()) {
            BOOST_ASSERT((*itmanip)->GetClosingDirection().size()==(*itmanip)->GetGripperIndices().size());
            for(size_t i = 0; i < (*itmanip)->GetClosingDirection().size(); ++i) {
                if( (*itmanip)->GetClosingDirection()[i] != 0 )
                    vclosingsign_full[(*itmanip)->GetGripperIndices()[i]] = (*itmanip)->GetClosingDirection()[i];
            }
        }

        graspparams->vgoalconfig.resize(_robot->GetActiveDOF());
        int i = 0;
        FOREACHC(itindex,_robot->GetActiveDOFIndices()) {
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
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));

        // have to add the first point
        Trajectory::TPOINT ptfirst;
        _robot->GetActiveDOFValues(ptfirst.q);
        ptraj->AddPoint(ptfirst);
        switch( CM::JitterActiveDOF(_robot) ) {
        case 0:
            RAVELOG_WARN("robot initially in collision\n");
            return false;
        case 1:
            _robot->GetActiveDOFValues(ptfirst.q);
        default:
            break;
        }
 
        boost::shared_ptr<PlannerBase> graspplanner = RaveCreatePlanner(GetEnv(),"Grasper");
        if( !graspplanner ) {
            RAVELOG_ERROR("grasping planner failure!\n");
            return false;
        }
        
        _robot->SetActiveManipulator(-1); // reset the manipulator
        graspparams->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(graspparams->vinitialconfig);  
        graspparams->btransformrobot = false;
        graspparams->breturntrajectory = false;
        graspparams->bonlycontacttarget = false;
        graspparams->bavoidcontact = true;

        if( !graspplanner->InitPlan(_robot, graspparams) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }
    
        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARN("PlanPath failed\n");
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
            RobotBase::RobotStateSaver saver2(_robot);
            _robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
            if( CM::JitterActiveDOF(_robot) > 0 ) {
                RAVELOG_WARN("robot final configuration is in collision\n");
                Trajectory::TPOINT pt = ptraj->GetPoints().back();
                _robot->GetActiveDOFValues(pt.q);
                ptraj->AddPoint(pt);
            }
        }

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    class SwitchModelContainer
    {
    public:
    SwitchModelContainer(KinBodyPtr pbody, const string& fatfilename) : _pbody(pbody) {
            RAVELOG_VERBOSE(str(boost::format("register %s body with %s fatfile\n")%pbody->GetName()%fatfilename));
            _pbodyfat = pbody->GetEnv()->ReadKinBodyXMLFile(fatfilename);
            // validate the fat body
            BOOST_ASSERT(pbody->GetLinks().size()==_pbodyfat->GetLinks().size());
            _bFat=false;
        }
        virtual ~SwitchModelContainer() {
            Switch(false);
        }

        void Switch(bool bSwitchToFat)
        {
            if( _bFat != bSwitchToFat ) {
                RAVELOG_DEBUG(str(boost::format("switching %s to fat: %d\n")%_pbody->GetName()%(int)bSwitchToFat));
                FOREACHC(itlink,_pbody->GetLinks()) {
                    KinBody::LinkPtr pswitchlink = _pbodyfat->GetLink((*itlink)->GetName());
                    list<KinBody::Link::GEOMPROPERTIES> listgeoms;
                    (*itlink)->SwapGeometries(listgeoms);
                    pswitchlink->SwapGeometries(listgeoms);
                    (*itlink)->SwapGeometries(listgeoms);
                }
                _bFat = bSwitchToFat;
            }
        }

        KinBodyPtr GetBody() const { return _pbody; }
        bool IsFat() const { return _bFat; }
    private:
        KinBodyPtr _pbody, _pbodyfat;
        bool _bFat;
    };

    class SwitchModelState
    {
    public:
    SwitchModelState(boost::shared_ptr<TaskManipulation> ptask) : _ptask(ptask) {
            _bFat = false;
            FOREACH(it, _ptask->_listSwitchModels) {
                if( (*it)->IsFat() ) {
                    _bFat = true;
                    break;
                }
            }
        }
        virtual ~SwitchModelState() { _ptask->_UpdateSwitchModels(_bFat); }
    private:
        boost::shared_ptr<TaskManipulation> _ptask;
        bool _bFat;
    };

    typedef boost::shared_ptr<SwitchModelContainer> SwitchModelContainerPtr;

    bool SwitchModels(ostream& sout, istream& sinput)
    {
        int doswitch=-1;
        string cmd;
        bool bUpdateBodies=true;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "register" ) {
                string pattern, fatfilename;
                sinput >> pattern >> fatfilename;
                if( !!sinput ) {
                    _listSwitchModelPatterns.push_back(make_pair(pattern,fatfilename));
                }
            }
            else if( cmd == "unregister" ) {
                string pattern;
                sinput >> pattern;
                list<pair<string,string> >::iterator it;
                FORIT(it,_listSwitchModelPatterns) {
                    if( it->first == pattern )
                        it = _listSwitchModelPatterns.erase(it);
                    else
                        ++it;
                }
            }
            else if( cmd == "update" )
                sinput >> bUpdateBodies;
            else if( cmd == "switchtofat" )
                sinput >> doswitch;
            else if( cmd == "clearpatterns" )
                _listSwitchModelPatterns.clear();
            else if( cmd == "clearmodels" )
                _listSwitchModels.clear();
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( doswitch >= 0 )
            _UpdateSwitchModels(doswitch>0,bUpdateBodies);
        return true;
    }

protected:
    inline boost::shared_ptr<TaskManipulation> shared_problem() { return boost::static_pointer_cast<TaskManipulation>(shared_from_this()); }
    inline boost::shared_ptr<TaskManipulation const> shared_problem_const() const { return boost::static_pointer_cast<TaskManipulation const>(shared_from_this()); }

    TrajectoryBasePtr _MoveArm(const vector<int>& activejoints, const vector<dReal>& activegoalconfig, int& nGoalIndex, int nMaxIterations)
    {
        RAVELOG_DEBUG("Starting MoveArm...\n");
        BOOST_ASSERT( !!_pRRTPlanner );
        TrajectoryBasePtr ptraj;
        RobotBase::RobotStateSaver _saver(_robot);

        if( activejoints.size() == 0 ) {
            RAVELOG_WARN("move arm failed\n");
            return ptraj;
        }

        if( (activegoalconfig.size()%activejoints.size()) != 0 ) {
            RAVELOG_WARN(str(boost::format("Active goal configurations not a multiple (%d/%d)\n")%activegoalconfig.size()%activejoints.size()));
            return ptraj;
        }

        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) )
            RAVELOG_WARN("Hand in collision\n");
    
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        _robot->SetActiveDOFs(activejoints);
        params->SetRobotActiveJoints(_robot);
        //params->_sPathOptimizationPlanner = ""; // no smoothing
        
        vector<dReal> pzero;
        _robot->GetActiveDOFValues(pzero);

        // make sure the initial and goal configs are not in collision
        params->vgoalconfig.reserve(activegoalconfig.size());
        vector<dReal> vgoals;

        for(int i = 0; i < (int)activegoalconfig.size(); i += activejoints.size()) {
            _robot->SetActiveDOFValues(vector<dReal>(activegoalconfig.begin()+i,activegoalconfig.begin()+i+_robot->GetActiveDOF()), true);

            // jitter only the manipulator! (jittering the hand causes big probs)
            if( CM::JitterActiveDOF(_robot) ) {
                _robot->GetActiveDOFValues(vgoals);
                params->vgoalconfig.insert(params->vgoalconfig.end(), vgoals.begin(), vgoals.end());
            }
        }

        if( params->vgoalconfig.size() == 0 )
            return ptraj;

        // restore
        _robot->SetActiveDOFValues(pzero);
    
        // jitter again for initial collision
        if( !CM::JitterActiveDOF(_robot) )
            return ptraj;

        _robot->GetActiveDOFValues(params->vinitialconfig);
        ptraj = RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF());

        params->_nMaxIterations = nMaxIterations; // max iterations before failure

        bool bSuccess = false;
        RAVELOG_VERBOSE("starting planning\n");
    
        stringstream ss;

        for(int iter = 0; iter < 3; ++iter) {

            if( !_pRRTPlanner->InitPlan(_robot, params) ) {
                RAVELOG_WARN("InitPlan failed\n");
                ptraj.reset();
                return ptraj;
            }
        
            if( _pRRTPlanner->PlanPath(ptraj, boost::shared_ptr<ostream>(&ss,null_deleter())) ) {
                ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true,_fMaxVelMult);
                ss >> nGoalIndex; // extract the goal index
                BOOST_ASSERT( nGoalIndex >= 0 && nGoalIndex < (int)params->vgoalconfig.size()/(int)activejoints.size() );
                bSuccess = true;
                RAVELOG_INFO("finished planning, goal index: %d\n", nGoalIndex);
                break;
            }
            else RAVELOG_WARN("PlanPath failed\n");
        }
    
        if( !bSuccess ) {
            ptraj.reset();
        }
        return ptraj;
    }

    /// grasps using the list of grasp goals. Removes all the goals that the planner planned with
    TrajectoryBasePtr _PlanGrasp(list<GRASPGOAL>& listGraspGoals, int nSeedIkSolutions, GRASPGOAL& goalfound, int nMaxIterations,PRESHAPETRAJMAP& mapPreshapeTrajectories)
    {
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
        TrajectoryBasePtr ptraj;

        // set all teh goals, be careful! not all goals have the same preshape!!!
        if( listGraspGoals.size() == 0 )
            return ptraj;

        RobotBase::RobotStateSaver _saver(_robot);

        // set back to the initial hand joints
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());
        vector<dReal> vpreshape = listGraspGoals.front().vpreshape;
        
        _robot->SetActiveDOFValues(vpreshape,true);

        list<GRASPGOAL>::iterator itgoals = listGraspGoals.begin();
        list<GRASPGOAL> listgraspsused;
    
        // take the first grasp
        listgraspsused.splice(listgraspsused.end(), listGraspGoals, itgoals++);
    
        while(itgoals != listGraspGoals.end()) {
            size_t ipreshape=0;
            for(ipreshape = 0; ipreshape < vpreshape.size(); ++ipreshape) {
                if( fabsf(vpreshape[ipreshape] - itgoals->vpreshape[ipreshape]) > 2.0*GRASPTHRESH2 )
                    break;
            }

            if( ipreshape == vpreshape.size() ) {
                // accept
                listgraspsused.splice(listgraspsused.end(), listGraspGoals, itgoals++);
            }
            else ++itgoals;
        }
        
        uint64_t tbase = GetMicroTime();

        PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vpreshape);
        if( itpreshapetraj != mapPreshapeTrajectories.end() ) {
            if( itpreshapetraj->second->GetPoints().size() > 0 )
                _robot->SetDOFValues(itpreshapetraj->second->GetPoints().back().q);
        }
        else {
            RAVELOG_WARN("no preshape trajectory!");
        }

        vector<dReal> vgoalconfigs;
        FOREACH(itgoal, listgraspsused) {
            BOOST_ASSERT( itgoal->viksolution.size() == pmanip->GetArmIndices().size() );
            vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());

            int nsampled = CM::SampleIkSolutions(_robot, itgoal->tgrasp, nSeedIkSolutions, vgoalconfigs);
            if( nsampled != nSeedIkSolutions ) {
                RAVELOG_WARN("warning, only found %d/%d ik solutions. goal indices will be wrong!\n", nsampled, nSeedIkSolutions);
                // fill the rest
                while(nsampled++ < nSeedIkSolutions)
                    vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());
            }
        }
                
        int nGraspIndex = 0;
        ptraj = _MoveArm(pmanip->GetArmIndices(), vgoalconfigs, nGraspIndex, nMaxIterations);
        if (!ptraj )
            return ptraj;

        list<GRASPGOAL>::iterator it = listgraspsused.begin();
        BOOST_ASSERT( nGraspIndex >= 0 && nGraspIndex/(1+nSeedIkSolutions) < (int)listgraspsused.size() );
        advance(it,nGraspIndex/(1+nSeedIkSolutions));
        goalfound = *it;

        TrajectoryBasePtr pfulltraj = RaveCreateTrajectory(GetEnv(),_robot->GetDOF());
        _robot->SetActiveDOFs(pmanip->GetArmIndices());
        _robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        
        RAVELOG_DEBUG("total planning time %d ms\n", (uint32_t)(GetMicroTime()-tbase)/1000);
        return pfulltraj;
    }

    void _UpdateSwitchModels(bool bSwitchToFat, bool bUpdateBodies=true)
    {
        if( bSwitchToFat && bUpdateBodies ) {
            // update model list
            string strcmd,strname;
#ifdef HAVE_BOOST_REGEX
            boost::regex re;
#endif

            vector<KinBodyPtr>::const_iterator itbody;
            list<SwitchModelContainerPtr>::iterator itmodel;
            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            FORIT(itbody, vbodies) {
                FORIT(itmodel,_listSwitchModels) {
                    if( (*itmodel)->GetBody() == *itbody )
                        break;
                }
                if( itmodel != _listSwitchModels.end() )
                    continue;
                FOREACH(itpattern, _listSwitchModelPatterns) {
                    bool bMatches=false;
#ifdef HAVE_BOOST_REGEX
                    try {
                        re.assign(itpattern->first, boost::regex_constants::icase);
                    }
                    catch (boost::regex_error& e) {
                        RAVELOG_ERROR(str(boost::format("%s is not a valid regular expression: %s")%itpattern->first%e.what()));
                        continue;
                    }
            
                    // convert
                    bMatches = boost::regex_match((*itbody)->GetName().c_str(), re);
#else
                    RAVELOG_DEBUG(str(boost::format("boost regex not enabled, cannot parse %s\n")%itpattern->first));
                    bMatches = (*itbody)->GetName() == itpattern->first;
#endif
                    if( bMatches ) {
                        _listSwitchModels.push_back(SwitchModelContainerPtr(new SwitchModelContainer(*itbody,itpattern->second)));
                    }
                }
            }
        }

        FOREACH(it,_listSwitchModels)
            (*it)->Switch(bSwitchToFat);
    }

    string _strRobotName; ///< name of the active robot
    RobotBasePtr _robot;
    dReal _fMaxVelMult;
    list<SensorSystemBasePtr > listsystems;
    PlannerBasePtr _pRRTPlanner, _pGrasperPlanner;
    list<pair<string,string> > _listSwitchModelPatterns;
    list<SwitchModelContainerPtr> _listSwitchModels;

    friend class SwitchModelState;
};
    
#endif
