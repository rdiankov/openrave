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
#ifndef OPENRAVE_TASK_MANIPULATION_PROBLEM
#define OPENRAVE_TASK_MANIPULATION_PROBLEM

#include "commonmanipulation.h"

#ifdef HAVE_BOOST_REGEX
#include <boost/regex.hpp>

#define SWITCHMODELS(tofat) \
{ \
    SwitchModelsInternal(vSwitchPatterns, tofat); \
    ptarget = GetEnv()->GetKinBody(targetname); \
    BOOST_ASSERT( !!ptarget ); \
} \

#else
#define SWITCHMODELS(tofat)
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
        RegisterCommand("createsystem",boost::bind(&TaskManipulation::CreateSystem,this,_1,_2),
                        "creates a sensor system and initializes it with the current bodies");
#ifdef HAVE_BOOST_REGEX
        RegisterCommand("switchmodels",boost::bind(&TaskManipulation::SwitchModels,this,_1,_2),
                        "Switches between thin and fat models for planning.");
#endif
        RegisterCommand("graspplanning",boost::bind(&TaskManipulation::GraspPlanning,this,_1,_2),
                        "Grasp planning, pick a grasp from a grasp set and use it for manipulation.\n"
                        "Can optionally use bispace for mobile platforms");
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
        ProblemInstance::Reset();
        listsystems.clear();
        // recreate the planners since they store state
        if( !!_pRRTPlanner )
            _pRRTPlanner = GetEnv()->CreatePlanner(_pRRTPlanner->GetXMLId());
        if( !!_pGrasperPlanner )
            _pGrasperPlanner = GetEnv()->CreatePlanner(_pGrasperPlanner->GetXMLId());
    }

    int main(const string& args)
    {
        string name;
        stringstream ss(args);

        ss >> _strRobotName;
    
        string plannername;
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "planner" )
                ss >> plannername;

            if( ss.fail() || !ss )
                break;
        }

        if( plannername.size() > 0 )
            _pRRTPlanner = GetEnv()->CreatePlanner(plannername);
        if( !_pRRTPlanner ) {
            plannername = "BiRRT";
            _pRRTPlanner = GetEnv()->CreatePlanner(plannername);
        }

        if( !_pRRTPlanner ) {
            RAVELOG_WARNA("could not find an rrt planner\n");
            return -1;
        }
        RAVELOG_DEBUGA(str(boost::format("using %s planner\n")%plannername));

        _pGrasperPlanner = GetEnv()->CreatePlanner("Grasper");
        if( !_pGrasperPlanner )
            RAVELOG_WARNA("Failed to create a grasper planner\n");
            
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
        if( !sinput )
            return false;

        SensorSystemBasePtr psystem = GetEnv()->CreateSensorSystem(systemname);
        if( !psystem )
            return false;

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        psystem->AddRegisteredBodies(vbodies);
        listsystems.push_back(psystem);

        RAVELOG_DEBUGA(str(boost::format("added %s system\n")%systemname));
        sout << 1; // signal success
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
        vector<pair<string, string> > vSwitchPatterns;
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
    
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "grasps" ) {
                sinput >> nNumGrasps >> nGraspDim;
                vgrasps.resize(nNumGrasps*nGraspDim);
                FOREACH(it, vgrasps)
                    sinput >> *it;
            }
            else if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "randomdests" )
                sinput >> bRandomDests;
            else if( cmd == "randomgrasps" )
                sinput >> bRandomGrasps;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "maxiter" )
                sinput >> nMaxIterations;
            else if( cmd == "graspindices" )
                sinput >> iGraspDir >> iGraspPos >> iGraspRoll >> iGraspStandoff >> iGraspPreshape;
            else if( cmd == "igraspdir" )
                sinput >> iGraspDir;
            else if( cmd == "igrasppos" )
                sinput >> iGraspPos;
            else if( cmd == "igrasproll" )
                sinput >> iGraspRoll;
            else if( cmd == "igraspstandoff" )
                sinput >> iGraspStandoff;
            else if( cmd == "igrasppreshape" )
                sinput >> iGraspPreshape;
            else if( cmd == "igrasptrans" )
                sinput >> iGraspTransform;
            else if( cmd == "target" ) {
                sinput >> targetname;
                ptarget = GetEnv()->GetKinBody(targetname);
            }
            else if( cmd == "approachoffset" )
                sinput >> fApproachOffset;
            else if( cmd == "quitafterfirstrun" )
                bQuitAfterFirstRun = true;
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
            else if( cmd == "seedgrasps" )
                sinput >> nMaxSeedGrasps;
            else if( cmd == "seeddests" )
                sinput >> nMaxSeedDests;
            else if( cmd == "seedik" )
                sinput >> nMaxSeedIkSolutions;
            else if( cmd == "switch" ) {
                string pattern, fatfilename;
                sinput >> pattern >> fatfilename;
                vSwitchPatterns.push_back(pair<string, string>(pattern, fatfilename));
            }
            else if( cmd == "savepreshapetraj" ) {
                sinput >> strpreshapetraj;
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
    
        if( !ptarget ) {
            RAVELOG_WARNA(str(boost::format("Could not find target %s\n")%targetname));
            return false;
        }
    
        BOOST_ASSERT(!pmanip->IsGrabbing(ptarget));
        RobotBase::RobotStateSaver saver(_robot);

        bool bMobileBase = _robot->GetAffineDOF()!=0; // if mobile base, cannot rely on IK
        if( bMobileBase )
            RAVELOG_INFOA("planning with mobile base!\n");

        bool bInitialRobotChanged = false;
        vector<dReal> vCurHandValues, vCurRobotValues, vOrgRobotValues;
        _robot->SetActiveDOFs(pmanip->GetGripperJoints());
        _robot->GetActiveDOFValues(vCurHandValues);
        _robot->GetJointValues(vOrgRobotValues);

        SWITCHMODELS(true);
        // check if robot is in collision with padded models
        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) ) {
            _robot->SetActiveDOFs(pmanip->GetArmJoints());
            if( !CM::JitterActiveDOF(_robot) ) {
                RAVELOG_ERRORA("failed to jitter robot\n");
                return false;
            }

            bInitialRobotChanged = true;
        }
        
        _robot->GetJointValues(vCurRobotValues);

        string strResponse;
        Transform transTarg = ptarget->GetTransform();
        Transform transInvTarget = transTarg.inverse();
        Transform transDummy(Vector(1,0,0,0), Vector(100,0,0));
        vector<dReal> viksolution, vikgoal, vFinalGripperValues;
        
        PRESHAPETRAJMAP mapPreshapeTrajectories;
        {
            // fill with a trajectory with one point
            TrajectoryBasePtr pstarttraj = GetEnv()->CreateTrajectory(_robot->GetDOF());
            Trajectory::TPOINT tpstarthand;
            _robot->GetJointValues(tpstarthand.q);
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
        for(int i = 0; i < (int)vObjDestinations.size(); ++i)
            vdestpermuation[i] = i;

        vector<int> vgrasppermuation(nNumGrasps);
        for(int i = 0; i < nNumGrasps; ++i)
            vgrasppermuation[i] = i;

        if( bRandomGrasps )
            PermutateRandomly(vgrasppermuation);

        if( iGraspTransform < 0 ) {
            if( !_pGrasperPlanner ) {
                RAVELOG_ERRORA("grasper problem not valid\n");
                return false;
            }
            if( iGraspDir < 0 || iGraspPos < 0 || iGraspRoll < 0 || iGraspPreshape < 0 || iGraspStandoff < 0 ) {
                RAVELOG_ERRORA("grasp indices not all initialized\n");
                return false;
            }
        }
        else {
            if( iGraspPreshape < 0 ) {
                RAVELOG_ERRORA("grasp indices not all initialized\n");
                return false;
            }
        }

        TrajectoryBasePtr phandtraj;
    
        for(int igraspperm = 0; igraspperm < (int)vgrasppermuation.size(); ++igraspperm) {
            int igrasp = vgrasppermuation[igraspperm];
            dReal* pgrasp = &vgrasps[igrasp*nGraspDim];

            if( listGraspGoals.size() > 0 && iCountdown-- <= 0 ) {
                // start planning
                SWITCHMODELS(true);

                RAVELOG_VERBOSEA(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
                uint64_t basestart = GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, nMaxSeedIkSolutions, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += GetMicroTime() - basestart;

                if( !!ptraj || bQuitAfterFirstRun )
                    break;
            }

            vector<dReal> vgoalpreshape(vCurHandValues.size());
            if( iGraspPreshape >= 0 ) {
                for(size_t j = 0; j < vCurHandValues.size(); ++j)
                    vgoalpreshape[j] = pgrasp[iGraspPreshape+j];
            }
            else {
                vgoalpreshape.resize(pmanip->GetGripperJoints().size());
                for(size_t j = 0; j < pmanip->GetGripperJoints().size(); ++j)
                    vgoalpreshape[j] = vCurRobotValues[pmanip->GetGripperJoints()[j]];
            }

            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vgoalpreshape);
            if( itpreshapetraj != mapPreshapeTrajectories.end() && !itpreshapetraj->second ) {
                // has failed to find a trajectory to open a hand on a previous attempt, so skip
                RAVELOG_DEBUGA("grasp %d: skipping failed preshape\n", igrasp);
                continue;
            }

            SWITCHMODELS(false);

            Transform tGoalEndEffector;
        
            if( iGraspTransform >= 0 ) {
                // use the grasp transform
                dReal* pm = pgrasp+iGraspTransform;
                TransformMatrix tm;
                tm.m[0] = pm[0]; tm.m[1] = pm[3]; tm.m[2] = pm[6]; tm.trans.x = pm[9];
                tm.m[4] = pm[1]; tm.m[5] = pm[4]; tm.m[6] = pm[7]; tm.trans.y = pm[10];
                tm.m[8] = pm[2]; tm.m[9] = pm[5]; tm.m[10] = pm[8]; tm.trans.z = pm[11];
                tGoalEndEffector = tm;

                if( pmanip->CheckEndEffectorCollision(tGoalEndEffector) ) {
                    RAVELOG_DEBUGA("grasp %d: in collision\n", igrasp);
                    continue;
                }
            }
            else if( !_pGrasperPlanner ) {
                RAVELOG_ERRORA("grasper problem not valid\n");
                return false;
            }

            if( !!_pGrasperPlanner ) {
                // set the preshape
                if( vgoalpreshape.size() > 0 ) {
                    _robot->SetActiveDOFs(pmanip->GetGripperJoints(), RobotBase::DOF_NoTransform);
                    _robot->SetActiveDOFValues(vgoalpreshape,true);
                }
                _robot->SetActiveDOFs(pmanip->GetGripperJoints(), RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z);

                if( !phandtraj )
                    phandtraj = GetEnv()->CreateTrajectory(_robot->GetActiveDOF());
                else
                    phandtraj->Reset(_robot->GetActiveDOF());

                graspparams->fstandoff = pgrasp[iGraspStandoff];
                graspparams->targetbody = ptarget;
                graspparams->ftargetroll = pgrasp[iGraspRoll];
                graspparams->vtargetdirection = Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
                graspparams->vtargetposition = Vector(pgrasp[iGraspPos], pgrasp[iGraspPos+1], pgrasp[iGraspPos+2]);
                graspparams->btransformrobot = true;
                graspparams->breturntrajectory = false;
                graspparams->bonlycontacttarget = true;
                graspparams->btightgrasp = false;

                if( !_pGrasperPlanner->InitPlan(_robot,graspparams) ) {
                    RAVELOG_DEBUGA("grasp planner failed: %d\n", igrasp);
                    continue; // failed
                }

                if( !_pGrasperPlanner->PlanPath(phandtraj) ) {
                    RAVELOG_DEBUGA("grasp planner failed: %d\n", igrasp);
                    continue; // failed
                }

                Transform t = phandtraj->GetPoints().back().trans;

                // move back a little if robot/target in collision
                if( !!ptarget ) {
                    KinBody::KinBodyStateSaver saverlocal(_robot);
                    _robot->SetTransform(t);
                    Vector vglobalpalmdir;
                    if( iGraspDir >= 0 )
                        vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                    else
                        vglobalpalmdir = pmanip->GetEndEffectorTransform().rotate(pmanip->GetPalmDirection());

                    while(GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(ptarget))) {
                        t.trans -= vglobalpalmdir*0.001f;
                        _robot->SetTransform(t);
                    }
                }

                tGoalEndEffector = t * _robot->GetTransform().inverse() * pmanip->GetEndEffectorTransform(); // find the end effector transform
                vFinalGripperValues.resize(pmanip->GetGripperJoints().size());
                std::copy(phandtraj->GetPoints().back().q.begin(),phandtraj->GetPoints().back().q.begin()+vFinalGripperValues.size(),vFinalGripperValues.begin());
            }
            else
                vFinalGripperValues.resize(0);

            // set the initial hand joints
            _robot->SetActiveDOFs(pmanip->GetGripperJoints());
            if( iGraspPreshape >= 0 )
                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);

            Transform tApproachEndEffector = tGoalEndEffector;
            if( !bMobileBase ) {
                // check ik
                Vector vglobalpalmdir;
                if( iGraspDir >= 0 )
                    vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                else
                    vglobalpalmdir = tApproachEndEffector.rotate(pmanip->GetPalmDirection());

                dReal fSmallOffset = 0.002f;
                tApproachEndEffector.trans -= fSmallOffset * vglobalpalmdir;

                // first test the IK solution at the destination tGoalEndEffector
                if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, true) ) {
                    RAVELOG_DEBUGA("grasp %d: No IK solution found (final)\n", igrasp);
                    continue;
                }

                // switch to fat models
                SWITCHMODELS(true);

                if( fApproachOffset != 0 ) {
                    // now test at the approach point (with offset)
                    tApproachEndEffector.trans -= (fApproachOffset-fSmallOffset) * vglobalpalmdir;
                    
                    // set the previous robot ik configuration to get the closest configuration!!
                    _robot->SetActiveDOFs(pmanip->GetArmJoints());
                    _robot->SetActiveDOFValues(viksolution);
                    if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, true) ) {
                        _robot->SetJointValues(vCurRobotValues); // reset robot to original position
                        RAVELOG_DEBUGA("grasp %d: No IK solution found (approach)\n", igrasp);
                        continue;
                    }
                    else {
                        stringstream ss;
                        ss << "IK found: "; 
                        FOREACH(it, viksolution)
                            ss << *it << " ";
                        ss << endl;
                        RAVELOG_DEBUGA(ss.str());
                    }
                }
            }

            // set the joints that the grasper plugin calculated
            _robot->SetActiveDOFs(pmanip->GetGripperJoints());
            if( vFinalGripperValues.size() > 0 )
                _robot->SetActiveDOFValues(vFinalGripperValues, true);

            //while (getchar() != '\n') usleep(1000);

            // should test destination with thin models
            SWITCHMODELS(false);

            list< TransformMatrix > listDests;
            if( bRandomDests )
                PermutateRandomly(vdestpermuation);

            for(int idestperm = 0; idestperm < (int)vdestpermuation.size(); ++idestperm) {
                Transform& transDestTarget = vObjDestinations[vdestpermuation[idestperm]];
                Transform tDestEndEffector = transDestTarget * transInvTarget * tGoalEndEffector;
                 
                ptarget->SetTransform(transDestTarget);
                _robot->Enable(false); // remove from target collisions
                bool bTargetCollision = GetEnv()->CheckCollision(KinBodyConstPtr(ptarget));
                _robot->Enable(true); // remove from target collisions
                ptarget->SetTransform(transTarg);
                if( bTargetCollision ) {
                    RAVELOG_VERBOSEA("target collision at dest\n");
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

            _robot->SetJointValues(vCurRobotValues); // reset robot to original position

            if( vObjDestinations.size() > 0 && listDests.size() == 0 ) {
                RAVELOG_WARNA("grasp %d: could not find destination\n", igrasp);
                continue;
            }

            // finally start planning
            SWITCHMODELS(true);

            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                // not present in map, so look for correct one
                // initial joint is far from desired preshape, have to plan to get to it
                // note that this changes trajectory of robot!
                _robot->SetActiveDOFValues(vCurHandValues, true);
                
                _robot->SetActiveDOFs(pmanip->GetArmJoints());
                TrajectoryBasePtr ptrajToPreshape = GetEnv()->CreateTrajectory(pmanip->GetArmJoints().size());
                bool bSuccess = CM::MoveUnsync::_MoveUnsyncJoints(GetEnv(), _robot, ptrajToPreshape, pmanip->GetGripperJoints(), vgoalpreshape);
                
                if( !bSuccess ) {
                    mapPreshapeTrajectories[vgoalpreshape].reset(); // set to empty
                    RAVELOG_DEBUGA("grasp %d: failed to find preshape\n", igrasp);
                    continue;
                }

                // get the full trajectory
                TrajectoryBasePtr ptrajToPreshapeFull = GetEnv()->CreateTrajectory(_robot->GetDOF());
                _robot->GetFullTrajectoryFromActive(ptrajToPreshapeFull, ptrajToPreshape);

                // add a grasp with the full preshape
                Trajectory::TPOINT tpopenhand;
                _robot->SetJointValues(ptrajToPreshapeFull->GetPoints().back().q);
                _robot->SetActiveDOFs(pmanip->GetGripperJoints());
                if( iGraspPreshape >= 0 )
                    _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
                _robot->GetJointValues(tpopenhand.q);
                tpopenhand.trans = _robot->GetTransform();
                ptrajToPreshapeFull->AddPoint(tpopenhand);
                ptrajToPreshapeFull->CalcTrajTiming(_robot, ptrajToPreshape->GetInterpMethod(), true, false);

                mapPreshapeTrajectories[vgoalpreshape] = ptrajToPreshapeFull;
            }

            if( iGraspPreshape >= 0 )
                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);

            listGraspGoals.push_back(GRASPGOAL());
            GRASPGOAL& goal = listGraspGoals.back();
            goal.index = igrasp;
            goal.tgrasp = tApproachEndEffector;
            goal.viksolution = viksolution;
            goal.listDests.swap(listDests);
            goal.vpreshape.resize(pmanip->GetGripperJoints().size());
            if( iGraspPreshape >= 0 ) {
                for(int j = 0; j < (int)goal.vpreshape.size(); ++j)
                    goal.vpreshape[j] = pgrasp[iGraspPreshape+j];
            }

            RAVELOG_DEBUGA("grasp %d: adding to goals\n", igrasp);
            iCountdown = 40;

            if( (int)listGraspGoals.size() >= nMaxSeedGrasps ) {
                RAVELOG_VERBOSEA(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
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
            RAVELOG_VERBOSEA(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
            uint64_t basestart = GetMicroTime();
            ptraj = _PlanGrasp(listGraspGoals, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
            nSearchTime += GetMicroTime() - basestart;
        }

        SWITCHMODELS(false);

        if( !!ptraj ) {
            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(goalFound.vpreshape);
            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                RAVELOG_ERRORA("no preshape trajectory!\n");
                FOREACH(itpreshape,mapPreshapeTrajectories) {
                    RAVELOG_ERRORA("%f %f %f %f %f %f\n",itpreshape->first[0],itpreshape->first[1],itpreshape->first[2],itpreshape->first[3],itpreshape->first[4],itpreshape->first[5]);
                }
                return false;
            }

            TrajectoryBasePtr ptrajfinal = GetEnv()->CreateTrajectory(_robot->GetDOF());

            if( bInitialRobotChanged )
                ptrajfinal->AddPoint(Trajectory::TPOINT(vOrgRobotValues,_robot->GetTransform(), 0));

            if( strpreshapetraj.size() > 0 ) // write the preshape
                itpreshapetraj->second->Write(strpreshapetraj, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
            if( bCombinePreShapeTraj) { // add the preshape
                FOREACHC(itpoint, itpreshapetraj->second->GetPoints())
                    ptrajfinal->AddPoint(*itpoint);
            }

            FOREACHC(itpoint, ptraj->GetPoints())
                ptrajfinal->AddPoint(*itpoint);
            
            ptrajfinal->CalcTrajTiming(_robot, ptrajfinal->GetInterpMethod(), true, false);
                
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

#ifdef HAVE_BOOST_REGEX
    bool SwitchModelsInternal(vector<pair<string, string> >& vpatterns, bool tofat)
    {
        string strcmd;
        boost::regex re;
        string strname;

        vector<KinBodyPtr>::const_iterator itbody, itbody2;
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FORIT(itbody, vbodies) {
        
            FOREACH(itpattern, vpatterns) {
                try {
                    re.assign(itpattern->first, boost::regex_constants::icase);
                }
                catch (boost::regex_error& e) {
                    stringstream ss;
                    ss << itpattern->first << " is not a valid regular expression: \""
                         << e.what() << "\"" << endl;
                    RAVELOG_ERRORA(ss.str());
                    continue;
                }
            
                // convert
                if( boost::regex_match((*itbody)->GetName().c_str(), re) ) {
                
                    // check if already created
                    bool bCreated = false;
                    strname = (*itbody)->GetName(); strname += "thin";

                    FORIT(itbody2, vbodies) {
                        if( (*itbody2)->GetName() == strname ) {
                            bCreated = true;
                            break;
                        }
                    }
                
                    if( !bCreated ) {
                        strname = (*itbody)->GetName(); strname += "fat";

                        FORIT(itbody2, vbodies) {
                            if( (*itbody2)->GetName() == strname) {
                                bCreated = true;
                                break;
                            }
                        }
                    }

                    if( tofat && !bCreated ) {
                        RAVELOG_DEBUGA(str(boost::format("creating %s\n")%strname));
                    
                        // create fat body
                        KinBodyPtr pfatbody = GetEnv()->CreateKinBody();
                        if( !pfatbody->InitFromFile(itpattern->second, std::list<std::pair<std::string,std::string> >()) ) {
                            RAVELOG_WARNA(str(boost::format("failed to open file: %s\n")%itpattern->second));
                            continue;
                        }

                        pfatbody->SetName(strname); // should be %sfat
                        if( !GetEnv()->AddKinBody(pfatbody) )
                            continue;

                        pfatbody->SetTransform(Transform(Vector(1,0,0,0),Vector(0,100,0)));
                    }
                
                    if( !SwitchModel((*itbody)->GetName(), tofat) )
                        RAVELOG_WARNA(str(boost::format("SwitchModel with %s failed\n")%(*itbody)->GetName()));
                    break;
                }
            }
        }

        return true;
    }

    bool SwitchModels(ostream& sout, istream& sinput)
    {
        vector<KinBodyPtr> vbodies;
        vector<bool> vpadded;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "name" ) {
                string name;
                sinput >> name;
                vbodies.push_back(GetEnv()->GetKinBody(name));
            }
            else if( cmd == "padded" ) {
                int padded;
                sinput >> padded;
                vpadded.push_back(padded>0);
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

        if(vbodies.size() != vpadded.size() ) {    
            RAVELOG_ERRORA("SwitchModels - Number of models must equal number of padding states.\n");
            return false;
        }
    
        for(size_t i = 0; i < vbodies.size(); i++) {
            if( SwitchModel(vbodies[i]->GetName(), vpadded[i]) != 0 ) {
                RAVELOG_DEBUGA(str(boost::format("failed switching: %s\n")%vbodies[i]->GetName()));
            }
        }

        return true;
    }
    
    bool SwitchModel(const string& bodyname, bool bToFatModel)
    {
        KinBodyPtr pbody = GetEnv()->GetKinBody(bodyname);
        if( !pbody )
            return false;

        string oldname, newname;
        
        if( bToFatModel ) {
            oldname = bodyname + "fat";
            newname = bodyname + "thin";
        }
        else {
            oldname = bodyname + "thin";
            newname = bodyname + "fat";
        }

        KinBodyPtr pswitch = GetEnv()->GetKinBody(oldname);
        if( !pswitch ) {
            RAVELOG_VERBOSEA(str(boost::format("Model %s doesn't need switching\n")%bodyname));
            return true;
        }

        KinBody::LinkPtr pGrabLink;
        if( !!_robot ) {
            pGrabLink = _robot->IsGrabbing(pbody);
            BOOST_ASSERT( !!pGrabLink );
            _robot->Release(pbody);
        }
            
        Transform tinit = pbody->GetTransform(); 
        vector<dReal> vjoints; pbody->GetJointValues(vjoints);
    
        Transform tprevtrans = tinit;
        
        pswitch->SetName(bodyname);
        pswitch->SetTransform(tinit);
        if( vjoints.size() > 0 )
            pswitch->SetJointValues(vjoints);
    
        pbody->SetName(newname);
        Transform temp; temp.trans.y = 100.0f;
        pbody->SetTransform(temp);

        FOREACH(itsys, listsystems)
            (*itsys)->SwitchBody(pbody,pswitch);

        if( !!pGrabLink ) {
            RAVELOG_VERBOSEA(str(boost::format("regrabbing %s\n")%pswitch->GetName()));
            _robot->Grab(pswitch, pGrabLink);
        }

        return true;
    }
#endif

protected:
    inline boost::shared_ptr<TaskManipulation> shared_problem() { return boost::static_pointer_cast<TaskManipulation>(shared_from_this()); }
    inline boost::shared_ptr<TaskManipulation const> shared_problem_const() const { return boost::static_pointer_cast<TaskManipulation const>(shared_from_this()); }

    TrajectoryBasePtr _MoveArm(const vector<int>& activejoints, const vector<dReal>& activegoalconfig, int& nGoalIndex, int nMaxIterations)
    {
        RAVELOG_DEBUGA("Starting MoveArm...\n");
        BOOST_ASSERT( !!_pRRTPlanner );
        TrajectoryBasePtr ptraj;
        RobotBase::RobotStateSaver _saver(_robot);

        if( activejoints.size() == 0 ) {
            RAVELOG_WARNA("move arm failed\n");
            return ptraj;
        }

        if( (activegoalconfig.size()%activejoints.size()) != 0 ) {
            RAVELOG_WARNA(str(boost::format("Active goal configurations not a multiple (%d/%d)\n")%activegoalconfig.size()%activejoints.size()));
            return ptraj;
        }

        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) )
            RAVELOG_WARNA("Hand in collision\n");
    
        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        _robot->SetActiveDOFs(activejoints);
        params->SetRobotActiveJoints(_robot);

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
        ptraj = GetEnv()->CreateTrajectory(_robot->GetActiveDOF());

        params->_nMaxIterations = nMaxIterations; // max iterations before failure

        bool bSuccess = false;
        RAVELOG_VERBOSEA("starting planning\n");
    
        stringstream ss;

        for(int iter = 0; iter < 3; ++iter) {

            if( !_pRRTPlanner->InitPlan(_robot, params) ) {
                RAVELOG_WARNA("InitPlan failed\n");
                ptraj.reset();
                return ptraj;
            }
        
            if( _pRRTPlanner->PlanPath(ptraj, boost::shared_ptr<ostream>(&ss,null_deleter())) ) {
                ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);
                ss >> nGoalIndex; // extract the goal index
                BOOST_ASSERT( nGoalIndex >= 0 && nGoalIndex < (int)params->vgoalconfig.size()/(int)activejoints.size() );
                bSuccess = true;
                RAVELOG_INFOA("finished planning, goal index: %d\n", nGoalIndex);
                break;
            }
            else RAVELOG_WARNA("PlanPath failed\n");
        }
    
        if( !bSuccess )
            ptraj.reset();

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
        _robot->SetActiveDOFs(pmanip->GetGripperJoints());
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
                _robot->SetJointValues(itpreshapetraj->second->GetPoints().back().q);
        }
        else {
            RAVELOG_WARNA("no preshape trajectory!");
        }

        vector<dReal> vgoalconfigs;
        FOREACH(itgoal, listgraspsused) {
            BOOST_ASSERT( itgoal->viksolution.size() == pmanip->GetArmJoints().size() );
            vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());

            int nsampled = CM::SampleIkSolutions(_robot, itgoal->tgrasp, nSeedIkSolutions, vgoalconfigs);
            if( nsampled != nSeedIkSolutions ) {
                RAVELOG_WARNA("warning, only found %d/%d ik solutions. goal indices will be wrong!\n", nsampled, nSeedIkSolutions);
                // fill the rest
                while(nsampled++ < nSeedIkSolutions)
                    vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());
            }
        }
                
        int nGraspIndex = 0;
        ptraj = _MoveArm(pmanip->GetArmJoints(), vgoalconfigs, nGraspIndex, nMaxIterations);
        if (!ptraj )
            return ptraj;

        list<GRASPGOAL>::iterator it = listgraspsused.begin();
        BOOST_ASSERT( nGraspIndex >= 0 && nGraspIndex/(1+nSeedIkSolutions) < (int)listgraspsused.size() );
        advance(it,nGraspIndex/(1+nSeedIkSolutions));
        goalfound = *it;

        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(_robot->GetDOF());
        _robot->SetActiveDOFs(pmanip->GetArmJoints());
        _robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
        
        RAVELOG_DEBUGA("total planning time %d ms\n", (uint32_t)(GetMicroTime()-tbase)/1000);
        return pfulltraj;
    }

    string _strRobotName; ///< name of the active robot
    RobotBasePtr _robot;
    list<SensorSystemBasePtr > listsystems;
    PlannerBasePtr _pRRTPlanner, _pGrasperPlanner;
};
    
#endif
