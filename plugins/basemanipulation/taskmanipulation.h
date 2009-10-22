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

#include "basemanipulation.h"

#ifdef HAVE_BOOST_REGEX
#include <boost/regex.hpp>

#define SWITCHMODELS(tofat) \
{ \
    SwitchModelsInternal(vSwitchPatterns, tofat); \
    ptarget = GetEnv()->GetKinBody(targetname.c_str()); \
    assert( ptarget != NULL ); \
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

class TaskManipulationProblem : public CmdProblemInstance
{
public:
    class RealVectorCompare
    {
    public:
        bool operator()(const vector<dReal> & v1, const vector<dReal>& v2) const
        {
            if( v1.size() != v2.size() )
                return true;

            for(size_t i = 0; i < v1.size(); ++i) {
                if( v1[i] < v2[i]-GRASPTHRESH2 )
                    return true;
                else if( v1[i] > v2[i]+GRASPTHRESH2 )
                    return false;
            }

            return false;
        }
    };

    typedef std::map<vector<dReal>, boost::shared_ptr<Trajectory>, RealVectorCompare > PRESHAPETRAJMAP;

    TaskManipulationProblem(EnvironmentBase* penv) : CmdProblemInstance(penv), _robot(NULL)
    {
        RegisterCommand("createsystem",(CommandFn)&TaskManipulationProblem::CreateSystem,
                        "creates a sensor system and initializes it with the current bodies");
//        RegisterCommand("HeadLookAt",(CommandFn)&TaskManipulationProblem::HeadLookAt,
//                        "Calculates the joint angles for the head to look at a specific target.\n"
//                        "Can optionally move the head there");
        RegisterCommand("Help", (CommandFn)&TaskManipulationProblem::Help,"Help message");
#ifdef HAVE_BOOST_REGEX
        RegisterCommand("switchmodels",(CommandFn)&TaskManipulationProblem::SwitchModels,
                        "Switches between thin and fat models for planning.");
#endif
        RegisterCommand("TestAllGrasps",(CommandFn)&TaskManipulationProblem::TestAllGrasps,
                        "Grasp planning, pick a grasp from a grasp set and use it for manipulation.\n"
                        "Can optionally use bispace for mobile platforms");
    }
    virtual ~TaskManipulationProblem()
    {
        Destroy();
    }

    virtual void Destroy()
    {
        CmdProblemInstance::Destroy();
        listsystems.clear();
        _pGrasperProblem.reset();
        _pRRTPlanner.reset();
        _robot = NULL;
    }

    int main(const char* args)
    {
        string name;
        stringstream ss(args);
        ss >> name;
        _strRobotName = _ravembstowcs(name.c_str());

        _pGrasperProblem.reset(GetEnv()->CreateProblem("GrasperProblem"));
        if( !_pGrasperProblem )
            RAVELOG_WARNA("Failed to create GrasperProblem\n");
        else if( _pGrasperProblem->main(NULL) < 0 )
            return -1;
        
        
        string plannername = "rBIRRT";

        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
        
            if( stricmp(cmd.c_str(), "planner") == 0 ) {
                ss >> plannername;
            }
            else break;

            if( !ss ) {
                RAVELOG_ERRORA("failed to parse arguments\n");
                return -1;
            }
        }

        _pRRTPlanner.reset(GetEnv()->CreatePlanner(plannername.c_str()));
            
        return 0;
    }

    void SetActiveRobots(const std::vector<RobotBase*>& robots)
    {
        RobotBase* probot = NULL;
        vector<RobotBase*>::const_iterator itrobot;
        FORIT(itrobot, robots) {
            if( wcsicmp((*itrobot)->GetName(), _strRobotName.c_str() ) == 0  ) {
                probot = *itrobot;
                break;
            }
        }

        if( probot == NULL ) {
            RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
            return;
        }

        if( _robot == probot )
            return;

        _robot = probot;

        // get all child links of all the manipualtor
        _vvManipChildLinks.clear();
        FOREACH(itmanip, _robot->GetManipulators()) {
            vector<pair<KinBody::Link*,Transform> > _vChildLinks;
            set<KinBody::Link*> setChildLinks;
            Transform tbaseinv = itmanip->GetEndEffectorTransform().inverse();
            itmanip->GetChildLinks(setChildLinks);
            FOREACH(itlink,setChildLinks)
                _vChildLinks.push_back(make_pair(*itlink,tbaseinv*(*itlink)->GetTransform()));
            _vvManipChildLinks.push_back(_vChildLinks);
        }
    }

    bool IsGripperCollision(int imanip, const Transform& tgripper)
    {
        if( _robot == NULL )
            return false;
        if( imanip < 0 || imanip >= (int)_vvManipChildLinks.size() )
            return false;
        
        FOREACH(itlink,_vvManipChildLinks[imanip]) {
            Transform torg = itlink->first->GetTransform();
            itlink->first->SetTransform(tgripper*itlink->second);
            bool bCollision = _robot->GetEnv()->CheckCollision(itlink->first);
            itlink->first->SetTransform(torg);
            if( bCollision )
                return true;
        }
        
        return false;
    }

    bool SendCommand(const char* cmd, string& response)
    {
        SetActiveRobots(GetEnv()->GetRobots());
        if( _robot == NULL ) {
            RAVELOG_ERRORA("robot is NULL, send command failed\n");
            return false;
        }
        return CmdProblemInstance::SendCommand(cmd, response);
    }

    bool CreateSystem(ostream& sout, istream& sinput)
    {
        string systemname;
        sinput >> systemname;
        if( !sinput )
            return false;

        boost::shared_ptr<SensorSystemBase> psystem(GetEnv()->CreateSensorSystem(systemname.c_str()));
        if( !psystem )
            return false;

        if( !psystem->Init(sinput) )
            return false;

        psystem->AddRegisteredBodies(GetEnv()->GetBodies());
        listsystems.push_back(psystem);

        RAVELOG_DEBUGA("added %s system\n", systemname.c_str());
        sout << 1; // signal success
        return true;
    }

    bool TestAllGrasps(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG(L"TestingAllGrasps...\n");
        const RobotBase::Manipulator* pmanip = _robot->GetActiveManipulator();
        if( pmanip == NULL )
            return false;

        vector<dReal> vgrasps;
        vector<int> vHandJoints, vHandJointsRobot; // one is for the indices of the test hand, the other for the real robot 
        
        KinBody* ptarget = NULL;
        RobotBase* probotHand = NULL;
        int nNumGrasps=0, nGraspDim=0;
        Vector vpalmdir; // normal of plam dir (in local coord system of robot hand)
        dReal fOffset=0.0f; // offset before approaching to the target
        vector<pair<string, string> > vSwitchPatterns;
        wstring targetname;
        vector<Transform> vObjDestinations;
        string strpreshapetraj; // save the preshape trajectory
        bool bCombinePreShapeTraj = true;
        bool bExecute = true, bOutputTraj = false;
        string strtrajfilename;
        bool bRandomDests = true, bRandomGrasps = true; // if true, permute the grasps and destinations when iterating through them

        int nMaxSeedGrasps = 20, nMaxSeedDests = 5, nMaxSeedIkSolutions = 0;
        int nMaxIterations = 4000;

        //bool bBiSpace = false; // use the bispace planner and plan with translation/rotation
        bool bQuitAfterFirstRun = false;

        // indices into the grasp table
        int iGraspDir = -1, iGraspPos = -1, iGraspRoll = -1, iGraspPreshape = -1, iGraspStandoff = -1;
        int iGraspTransform = -1; // if >= 0, use the grasp transform directly without executing the grasper planner

        string cmd;
    
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            if( stricmp(cmd.c_str(), "grasps") == 0 ) {
                sinput >> nNumGrasps >> nGraspDim;
                vgrasps.resize(nNumGrasps*nGraspDim);
                FOREACH(it, vgrasps)
                    sinput >> *it;
            }
            else if( stricmp(cmd.c_str(), "outputtraj") == 0 )
                bOutputTraj = true;
            else if( stricmp(cmd.c_str(), "execute") == 0 )
                sinput >> bExecute;
            else if( stricmp(cmd.c_str(), "randomdests") == 0 )
                sinput >> bRandomDests;
            else if( stricmp(cmd.c_str(), "randomgrasps") == 0 )
                sinput >> bRandomGrasps;
            else if( stricmp(cmd.c_str(), "writetraj") == 0 )
                sinput >> strtrajfilename;
            else if( stricmp(cmd.c_str(), "maxiter") == 0 )
                sinput >> nMaxIterations;

            else if( stricmp(cmd.c_str(), "graspindices") == 0 ) {
                sinput >> iGraspDir >> iGraspPos >> iGraspRoll >> iGraspStandoff >> iGraspPreshape;
            }
            else if( stricmp(cmd.c_str(), "igrasppreshape") == 0 ) {
                sinput >> iGraspPreshape;
            }
            else if( stricmp(cmd.c_str(), "igrasptrans") == 0 ) {
                sinput >> iGraspTransform;
            }
            else if( stricmp(cmd.c_str(), "target") == 0 ) {
                string name; sinput >> name;
                targetname = _ravembstowcs(name.c_str());
                ptarget = GetEnv()->GetKinBody(targetname.c_str());
            }
            else if( stricmp(cmd.c_str(), "robothand") == 0 ) {
                string name; sinput >> name;
                KinBody* ptemp = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
                if( ptemp != NULL && ptemp->IsRobot() )
                    probotHand = (RobotBase*)ptemp;
            }
            else if( stricmp(cmd.c_str(), "handjoints") == 0 ) {
                int n = 0; sinput >> n;
                vHandJoints.resize(n);
                FOREACH(it, vHandJoints)
                    sinput >> *it;
            }
            else if( stricmp(cmd.c_str(), "robothandjoints") == 0 ) {
                int n = 0; sinput >> n;
                vHandJointsRobot.resize(n);
                FOREACH(it, vHandJointsRobot)
                    sinput >> *it;
            }
            else if( stricmp(cmd.c_str(), "palmdir") == 0 ) {
                sinput >> vpalmdir.x >> vpalmdir.y >> vpalmdir.z;
            }
            else if( stricmp(cmd.c_str(), "offset") == 0 )
                sinput >> fOffset;
            else if( stricmp(cmd.c_str(), "quitafterfirstrun") == 0 )
                bQuitAfterFirstRun = true;
            else if( stricmp(cmd.c_str(), "destposes") == 0 ) {
                int numdests = 0; sinput >> numdests;
                vObjDestinations.resize(numdests);
                FOREACH(ittrans, vObjDestinations) {
                    TransformMatrix m; sinput >> m;
                    *ittrans = m;
                }
            }
            else if( stricmp(cmd.c_str(), "seedgrasps") == 0 )
                sinput >> nMaxSeedGrasps;
            else if( stricmp(cmd.c_str(), "seeddests") == 0 )
                sinput >> nMaxSeedDests;
            else if( stricmp(cmd.c_str(), "seedik") == 0 )
                sinput >> nMaxSeedIkSolutions;
            else if( stricmp(cmd.c_str(), "switch") == 0 ) {
                string pattern, fatfilename;
                sinput >> pattern >> fatfilename;
                vSwitchPatterns.push_back(pair<string, string>(pattern, fatfilename));
            }
            else if( stricmp(cmd.c_str(), "savepreshapetraj") == 0 ) {
                sinput >> strpreshapetraj;
            }
//            else if( stricmp(cmd.c_str(), "combinepreshapetraj") == 0 ) {
//                bCombinePreShapeTraj = true;
//            }
            else break;

            if( !sinput ) {
                RAVELOG(L"failed\n");
                return false;
            }
        }
    
        if( ptarget == NULL ) {
            RAVEPRINT(L"Could not find target %S\n", targetname.c_str());
            return false;
        }
    
        RobotBase::RobotStateSaver saver(_robot);

        bool bMobileBase = _robot->GetAffineDOF()!=0; // if mobile base, cannot rely on IK
        if( bMobileBase )
            RAVELOG_INFOA("planning with mobile base!\n");

        bool bInitialRobotChanged = false;
        vector<dReal> vCurHandValues, vCurRobotValues, vOrgRobotValues;
        _robot->SetActiveDOFs(vHandJointsRobot);
        _robot->GetActiveDOFValues(vCurHandValues);
        _robot->GetJointValues(vOrgRobotValues);

        SWITCHMODELS(true);
        // check if robot is in collision with padded models
        if( GetEnv()->CheckCollision(_robot) ) {
            _robot->SetActiveDOFs(pmanip->_vecarmjoints);
            if( !JitterActiveDOF(_robot) ) {
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
        vector<dReal> viksolution, vikgoal, vjointsvalues;
        
        PRESHAPETRAJMAP mapPreshapeTrajectories;
        {
            // fill with a trajectory with one point
            boost::shared_ptr<Trajectory> pstarttraj(GetEnv()->CreateTrajectory(_robot->GetDOF()));
            Trajectory::TPOINT tpstarthand;
            _robot->GetJointValues(tpstarthand.q);
            tpstarthand.trans = _robot->GetTransform();
            pstarttraj->AddPoint(tpstarthand);
            mapPreshapeTrajectories[vCurHandValues] = pstarttraj;
        }

        boost::shared_ptr<Trajectory> ptraj;
        GRASPGOAL goalFound;
        Transform transDestHand;
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
            if( _pGrasperProblem == NULL ) {
                RAVELOG_ERRORA("grasper problem not valid\n");
                return false;
            }
            if( iGraspDir < 0 || iGraspPos < 0 || iGraspRoll < 0 || iGraspPreshape < 0 || iGraspStandoff < 0 ) {
                RAVELOG_ERRORA("grasp indices not all initialized\n");
                return false;
            }

            if( probotHand == NULL ) {
                RAVEPRINT(L"Couldn't not find test hand\n");
                return false;
            }

            if( (int)vHandJointsRobot.size() != probotHand->GetDOF() ) {
                RAVELOG_ERRORA("robot hand joints (%"PRIdS") not equal to hand dof (%d)\n", vHandJointsRobot.size(), probotHand->GetDOF());
                return false;
            }
        }
        else {
            if( iGraspPreshape < 0 ) {
                RAVELOG_ERRORA("grasp indices not all initialized\n");
                return false;
            }
        }

        for(int igraspperm = 0; igraspperm < (int)vgrasppermuation.size(); ++igraspperm) {
            int igrasp = vgrasppermuation[igraspperm];
            dReal* pgrasp = &vgrasps[igrasp*nGraspDim];

            if( listGraspGoals.size() > 0 && iCountdown-- <= 0 ) {
                // start planning
                SWITCHMODELS(true);

                RAVELOG_VERBOSEA("planning grasps %"PRIdS"\n",listGraspGoals.size());
                uint64_t basestart = GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, vHandJointsRobot, nMaxSeedIkSolutions, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += GetMicroTime() - basestart;

                if( ptraj.get() != NULL || bQuitAfterFirstRun )
                    break;
            }

            vector<dReal> vgoalpreshape(vCurHandValues.size());
            if( iGraspPreshape >= 0 ) {
                for(size_t j = 0; j < vCurHandValues.size(); ++j)
                    vgoalpreshape[j] = pgrasp[iGraspPreshape+j];
            }
            else {
                vgoalpreshape.resize(vHandJointsRobot.size());
                for(size_t j = 0; j < vHandJointsRobot.size(); ++j)
                    vgoalpreshape[j] = vCurRobotValues[vHandJointsRobot[j]];
            }

            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vgoalpreshape);
            if( itpreshapetraj != mapPreshapeTrajectories.end() && !itpreshapetraj->second ) {
                // has failed to find a trajectory to open a hand on a previous attempt, so skip
                RAVELOG_DEBUGA("grasp %d: skipping failed preshape\n", igrasp);
                continue;
            }

            SWITCHMODELS(false);

            Transform transRobot;
        
            if( iGraspTransform >= 0 ) {
                // use the grasp transform
                dReal* pm = pgrasp+iGraspTransform;
                TransformMatrix tm;
                tm.m[0] = pm[0]; tm.m[1] = pm[3]; tm.m[2] = pm[6]; tm.trans.x = pm[9];
                tm.m[4] = pm[1]; tm.m[5] = pm[4]; tm.m[6] = pm[7]; tm.trans.y = pm[10];
                tm.m[8] = pm[2]; tm.m[9] = pm[5]; tm.m[10] = pm[8]; tm.trans.z = pm[11];
                transRobot = tm;

                if( IsGripperCollision(_robot->GetActiveManipulatorIndex(),transRobot) ) {
                    RAVELOG_DEBUGA("grasp %d: in collision\n", igrasp);
                    continue;
                }
            }
            else {
                // set the hand joints
                if( vgoalpreshape.size() > 0 )
                    probotHand->SetJointValues(NULL,NULL, &vgoalpreshape[0],true);

                if( _pGrasperProblem == NULL ) {
                    RAVELOG_ERRORA("grasper problem not valid\n");
                    return false;
                }

                _robot->Enable(false);
                probotHand->Enable(true);
        
                probotHand->SetActiveDOFs(vHandJoints, RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z);

                stringstream ss;
                ss << "exec direction " << pgrasp[iGraspDir] << " " << pgrasp[iGraspDir+1] << " " << pgrasp[iGraspDir+2]
                   << " bodyid " << ptarget->GetNetworkId() << " robot " << probotHand->GetNetworkId()
                   << " roll " << pgrasp[iGraspRoll] << " standoff " << pgrasp[iGraspStandoff]
                   << " centeroffset " << pgrasp[iGraspPos]-transTarg.trans.x << " " << pgrasp[iGraspPos+1]-transTarg.trans.y << " " << pgrasp[iGraspPos+2]-transTarg.trans.z
                   << " palmdir " << vpalmdir.x << " " << vpalmdir.y << " " << vpalmdir.z;

                RAVELOG_VERBOSEA("grasper cmd: %s\n", ss.str().c_str());
                
                Transform t = _robot->GetTransform();
                _robot->SetTransform(transDummy);
                _pGrasperProblem->SendCommand(ss.str().c_str(), strResponse);
                _robot->SetTransform(t);

                probotHand->Enable(false);
                _robot->Enable(true);

                if( strResponse.size() == 0 ) {
                    RAVEPRINT(L"grasp planner failed: %d\n", igrasp);
                    continue; // failed
                }

                transRobot = probotHand->GetTransform();

                // send the robot somewhere
                probotHand->GetController()->SetPath(NULL); // reset
                probotHand->SetTransform(transDummy);
            }

            // set the initial hand joints
            _robot->SetActiveDOFs(vHandJointsRobot);
            if( iGraspPreshape >= 0 )
                _robot->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape,true);

            Transform tnewrobot = transRobot;

            if( !bMobileBase ) {
                // check ik
                Vector vglobalpalmdir;
                if( iGraspDir >= 0 )
                    vglobalpalmdir = Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
                else
                    vglobalpalmdir = tnewrobot.rotate(vpalmdir);

                dReal fSmallOffset = 0.002f;
                tnewrobot.trans -= fSmallOffset * vglobalpalmdir;

                // first test the IK solution at the destination transRobot
                // don't check for collisions since grasper plugins should have done that
                if( !pmanip->FindIKSolution(tnewrobot, viksolution, iGraspTransform >= 0 || _pGrasperProblem == NULL) ) {
                    RAVELOG_DEBUGA("grasp %d: No IK solution found (final)\n", igrasp);
                    continue;
                }

                // switch to fat models
                SWITCHMODELS(true);

                if( fOffset != 0 ) {
                    // now test at the approach point (with offset)
                    tnewrobot.trans -= (fOffset-fSmallOffset) * vglobalpalmdir;
                    
                    // set the previous robot ik configuration to get the closest configuration!!
                    _robot->SetActiveDOFs(pmanip->_vecarmjoints);
                    _robot->SetActiveDOFValues(NULL,&viksolution[0]);
                    if( !pmanip->FindIKSolution(tnewrobot, viksolution, true) ) {
                        _robot->SetJointValues(NULL, NULL, &vCurRobotValues[0]); // reset robot to original position
                        RAVELOG_DEBUGA("grasp %d: No IK solution found (approach)\n", igrasp);
                        continue;
                    }
                    else {
                        stringstream ss;
                        ss << "IK found: "; 
                        FOREACH(it, viksolution)
                            ss << *it << " ";
                        ss << endl;
                        RAVELOG_DEBUGA(ss.str().c_str());
                    }
                }
            }

            // set the joints that the grasper plugin calculated
            // DON'T: gets in collision, and vHandJoints.size is not necessarily equal to vHandJointsRobot.size
            //probotHand->SetActiveDOFs(vHandJoints, 0);
            //probotHand->GetActiveDOFValues(vjointsvalues);
            _robot->SetActiveDOFs(vHandJointsRobot);
            if( probotHand != NULL ) {
                probotHand->GetJointValues(vjointsvalues);
                _robot->SetActiveDOFValues(NULL, &vjointsvalues[0], true);
            }

            //while (getchar() != '\n') usleep(1000);

            // should test destination with thin models
            SWITCHMODELS(false);

            // Disable destination checking
            list< TransformMatrix > listDests;

            if( bRandomDests )
                PermutateRandomly(vdestpermuation);

            for(int idestperm = 0; idestperm < (int)vdestpermuation.size(); ++idestperm) {
                Transform& transDestTarget = vObjDestinations[vdestpermuation[idestperm]];
                transDestHand = transDestTarget * transInvTarget * transRobot;
                 
                ptarget->SetTransform(transDestTarget);
                _robot->Enable(false); // remove from target collisions
                bool bTargetCollision = GetEnv()->CheckCollision(ptarget);
                _robot->Enable(true); // remove from target collisions
                ptarget->SetTransform(transTarg);
                if( bTargetCollision ) {
                    RAVELOG_VERBOSE(L"target collision at dest\n");
                    continue;
                }
                
                if( !bMobileBase ) {
                    if( pmanip->FindIKSolution(transDestHand, vikgoal, true) ) {
                        listDests.push_back(transDestHand);
                    }
                }
                else
                    listDests.push_back(transDestHand);

                if( (int)listDests.size() >= nMaxSeedDests )
                    break;
            }

            _robot->Enable(true);

            _robot->SetJointValues(NULL, NULL, &vCurRobotValues[0]); // reset robot to original position

            if( vObjDestinations.size() > 0 && listDests.size() == 0 ) {
                RAVEPRINT(L"grasp %d: could not find destination\n", igrasp);
                continue;
            }

            // finally start planning
            SWITCHMODELS(true);

            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                // not present in map, so look for correct one
                // initial joint is far from desired preshape, have to plan to get to it
                // note that this changes trajectory of robot!
                _robot->SetActiveDOFValues(NULL, &vCurHandValues[0], true);
                
                _robot->SetActiveDOFs(pmanip->_vecarmjoints);
                boost::shared_ptr<Trajectory> ptrajToPreshape(GetEnv()->CreateTrajectory(pmanip->_vecarmjoints.size()));
                bool bSuccess = MoveUnsyncGoalFunction::_MoveUnsyncJoints(GetEnv(), _robot, ptrajToPreshape.get(), vHandJointsRobot, vgoalpreshape, NULL);
                
                if( !bSuccess ) {
                    mapPreshapeTrajectories[vgoalpreshape].reset(); // set to empty
                    RAVELOG_DEBUGA("grasp %d: failed to find preshape\n", igrasp);
                    continue;
                }

                // get the full trajectory
                boost::shared_ptr<Trajectory> ptrajToPreshapeFull(GetEnv()->CreateTrajectory(_robot->GetDOF()));
                _robot->GetFullTrajectoryFromActive(ptrajToPreshapeFull.get(), ptrajToPreshape.get());

                // add a grasp with the full preshape
                Trajectory::TPOINT tpopenhand;
                _robot->SetJointValues(NULL,NULL,&ptrajToPreshapeFull->GetPoints().back().q[0]);
                _robot->SetActiveDOFs(vHandJointsRobot);
                if( iGraspPreshape >= 0 )
                    _robot->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape,true);
                _robot->GetJointValues(tpopenhand.q);
                tpopenhand.trans = _robot->GetTransform();
                ptrajToPreshapeFull->AddPoint(tpopenhand);
                ptrajToPreshapeFull->CalcTrajTiming(_robot, ptrajToPreshape->GetInterpMethod(), true, false);

                mapPreshapeTrajectories[vgoalpreshape] = ptrajToPreshapeFull;
            }

            if( iGraspPreshape >= 0 )
                _robot->SetActiveDOFValues(NULL, pgrasp+iGraspPreshape,true);

            listGraspGoals.push_back(GRASPGOAL());
            GRASPGOAL& goal = listGraspGoals.back();
            goal.index = igrasp;
            goal.tgrasp = tnewrobot;
            goal.viksolution = viksolution;
            goal.listDests.swap(listDests);
            goal.vpreshape.resize(vHandJointsRobot.size());
            if( iGraspPreshape >= 0 ) {
                for(int j = 0; j < (int)goal.vpreshape.size(); ++j)
                    goal.vpreshape[j] = pgrasp[iGraspPreshape+j];
            }

            RAVELOG_DEBUG(L"grasp %d: adding to goals\n", igrasp);
            iCountdown = 40;

            if( (int)listGraspGoals.size() >= nMaxSeedGrasps ) {
                RAVELOG_VERBOSEA("planning grasps %"PRIdS"\n",listGraspGoals.size());
                uint64_t basestart = GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, vHandJointsRobot, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
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
            RAVELOG_VERBOSEA("planning grasps %"PRIdS"\n",listGraspGoals.size());
            uint64_t basestart = GetMicroTime();
            ptraj = _PlanGrasp(listGraspGoals, vHandJointsRobot, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
            nSearchTime += GetMicroTime() - basestart;
        }

        if( probotHand != NULL ) {
            // send the hand somewhere
            probotHand->GetController()->SetPath(NULL); // reset
            probotHand->SetTransform(transDummy);
            probotHand->Enable(false);
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

            boost::shared_ptr<Trajectory> ptrajfinal(GetEnv()->CreateTrajectory(_robot->GetDOF()));

            if( bInitialRobotChanged )
                ptrajfinal->AddPoint(Trajectory::TPOINT(vOrgRobotValues,_robot->GetTransform(), 0));

            if( strpreshapetraj.size() > 0 ) // write the preshape
                itpreshapetraj->second->Write(strpreshapetraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
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
            _SetFullTrajectory(ptrajfinal.get(), bExecute, strtrajfilename, bOutputTraj?&sout:NULL);

            if( probotHand != NULL ) {
                probotHand->SetTransform(transDummy);
                probotHand->Enable(true);
            }
            return true;
        }

        return false; // couldn't not find for this cup
    }

//    bool HeadLookAt(ostream& sout, istream& sinput)
//    {
//        Vector targetpt;
//        KinBody* pbody = NULL;
//
//        string cmd;
//    
//        while(!sinput.eof()) {
//            sinput >> cmd;
//            if( !sinput )
//                break;
//        
//            if( stricmp(cmd.c_str(), "targetpt") == 0 ) {
//                sinput >> targetpt.x >> targetpt.y >> targetpt.z;
//            }
//            else if( stricmp(cmd.c_str(), "body") == 0 ) {
//                string name; sinput >> name;
//                pbody = GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str());
//                if( pbody != NULL )
//                    targetpt = pbody->GetTransform().trans;
//            }
//            else break;
//
//            if( !sinput ) {
//                RAVELOG(L"failed\n");
//                return false;
//            }
//        }
//
//        // Will store head angles in here
//        dReal fheadangles[2];
//    
//        //Vector vcamerapos = Vector(0.115, 0.045, 0.085); // Old SVS camera
//        Vector vcamerapos = Vector(0.105f, 0.0, 0.085f);
//
//        const RobotBase::Manipulator& head = _robot->GetManipulators()[0];
//        Transform tNeck = head.pBase->GetTransform();
//        Vector localpt = tNeck.inverse() * targetpt;
//    
//        Vector destdir = localpt - vcamerapos;
//    
//        normalize3(destdir, destdir);
//    
//        double yaw = atan2f(destdir.y, destdir.x);
//        double pitch = -atan2f(destdir.z, sqrtf(destdir.x*destdir.x+destdir.y*destdir.y));
//    
//        yaw = CLAMP_ON_RANGE<double>(yaw, YAWMIN*0.0174532925, YAWMAX*0.0174532925);
//        pitch = CLAMP_ON_RANGE<double>(pitch, PITCHMIN*0.0174532925, PITCHMAX*0.0174532925);
//    
//        fheadangles[0] = yaw;
//        fheadangles[1] = pitch;
//    
//        vector<dReal> values;
//        _robot->GetJointValues(values);
//    
//        for(size_t i=0; i<head._vecarmjoints.size(); i++) {
//            values[head._vecarmjoints[i]] = fheadangles[i];
//        }
//    
//        _robot->SetJointValues(NULL, NULL, &values[0], true);
//        _robot->GetController()->SetDesired(&values[0]);
//        return true;
//    }

#ifdef HAVE_BOOST_REGEX
    bool SwitchModelsInternal(vector<pair<string, string> >& vpatterns, bool tofat)
    {
        string strcmd;
        boost::regex re;
        char str[128];
        wchar_t strname[128];

        vector<KinBody*>::const_iterator itbody, itbody2;
        vector<KinBody*> vbodies = GetEnv()->GetBodies(); // copy
        FORIT(itbody, vbodies) {
        
            FOREACH(itpattern, vpatterns) {
                try {
                    re.assign(itpattern->first, boost::regex_constants::icase);
                }
                catch (boost::regex_error& e) {
                    stringstream ss;
                    ss << itpattern->first << " is not a valid regular expression: \""
                         << e.what() << "\"" << endl;
                    RAVELOG_ERRORA("%s", ss.str().c_str());
                    continue;
                }
            
                // convert
                sprintf(str, "%S", (*itbody)->GetName());
                if( boost::regex_match(str, re) ) {
                
                    // check if already created
                    bool bCreated = false;
                    wcscpy(strname, (*itbody)->GetName());
                    wcscat(strname, L"thin");

                    FORIT(itbody2, vbodies) {
                        if( wcscmp((*itbody2)->GetName(), strname) == 0 ) {
                            bCreated = true;
                            break;
                        }
                    }
                
                    if( !bCreated ) {
                        wcscpy(strname, (*itbody)->GetName());
                        wcscat(strname, L"fat");

                        FORIT(itbody2, vbodies) {
                            if( wcscmp((*itbody2)->GetName(), strname) == 0 ) {
                                bCreated = true;
                                break;
                            }
                        }
                    }

                    if( tofat && !bCreated ) {
                    
                        GetEnv()->LockPhysics(true);
                        RAVELOG_DEBUGA("creating %S\n", strname);
                    
                        // create fat body
                        KinBody* pfatbody = GetEnv()->CreateKinBody();
                        if( !pfatbody->Init(itpattern->second.c_str(), NULL) ) {
                            RAVELOG_WARNA("failed to open file: %s\n", itpattern->second.c_str());
                            GetEnv()->LockPhysics(false);
                            continue;
                        }
                        pfatbody->SetName(strname); // should be %Sfat
                        if( !GetEnv()->AddKinBody(pfatbody) ) {
                            delete pfatbody;
                            GetEnv()->LockPhysics(false);
                            continue;
                        }

                        pfatbody->SetTransform(Transform(Vector(1,0,0,0),Vector(0,100,0)));
                        GetEnv()->LockPhysics(false);
                    }
                
                    if( !SwitchModel((*itbody)->GetName(), tofat) )
                        RAVELOG_WARNA("SwitchModel with %S failed\n", (*itbody)->GetName());
                    break;
                }
            }
        }

        return true;
    }

    bool SwitchModels(ostream& sout, istream& sinput)
    {
        vector<KinBody*> vbodies;
        vector<bool> vpadded;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
        
            if( stricmp(cmd.c_str(), "name") == 0 ) {
                string name;
                sinput >> name;
                vbodies.push_back(GetEnv()->GetKinBody(_ravembstowcs(name.c_str()).c_str()));
            }
            else if( stricmp(cmd.c_str(), "padded") == 0 ) {
                int padded;
                sinput >> padded;
                vpadded.push_back(padded>0);
            }
            else break;

            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }

        if(vbodies.size() != vpadded.size() ) {    
            RAVELOG_ERRORA("SwitchModels - Number of models must equal number of padding states.\n");
            return false;
        }
    
        for(size_t i = 0; i < vbodies.size(); i++) {
            if( SwitchModel(vbodies[i]->GetName(), vpadded[i]) != 0 ) {
                RAVELOG_DEBUGA("failed switching: %S\n", vbodies[i]->GetName());
            }
        }

        return true;
    }
    
    bool SwitchModel(const wstring& bodyname, bool bToFatModel)
    {
        KinBody* pbody = GetEnv()->GetKinBody(bodyname.c_str());
        if( pbody == NULL )
            return false;

        wstring oldname, newname;

        if( bToFatModel ) {
            oldname = bodyname + L"fat";
            newname = bodyname + L"thin";
        }
        else {
            oldname = bodyname + L"thin";
            newname = bodyname + L"fat";
        }

        KinBody* pswitch = GetEnv()->GetKinBody(oldname.c_str());
        if( pswitch == NULL ) {
            RAVELOG_VERBOSEA("Model %S doesn't need switching\n",bodyname.c_str());
            return true;
        }

        KinBody::Link* pGrabLink = NULL;
        if( _robot != NULL && _robot->IsGrabbing(pbody) ) {
            FOREACH(it, _robot->GetGrabbed()) {
                if( it->pbody == pbody ) {
                    pGrabLink = it->plinkrobot;
                    break;
                }
            }

            assert( pGrabLink != NULL );
            _robot->Release(pbody);
        }
            
        Transform tinit = pbody->GetTransform(); 
        vector<dReal> vjoints; pbody->GetJointValues(vjoints);
    
        Transform tprevtrans = tinit;
        
        pswitch->SetName(bodyname.c_str());
        pswitch->SetTransform(tinit);
        if( vjoints.size() > 0 )
            pswitch->SetJointValues(NULL,NULL,&vjoints[0]);
    
        pbody->SetName(newname.c_str());
        Transform temp; temp.trans.y = 100.0f;
        pbody->SetTransform(temp);

        FOREACH(itsys, listsystems)
            (*itsys)->SwitchBody(pbody,pswitch);

        if( pGrabLink != NULL ) {
            RAVELOG_VERBOSEA("regrabbing %S\n", pswitch->GetName());
            _robot->Grab(pswitch, pGrabLink->GetIndex(), NULL);
        }

        return true;
    }
#endif

    bool Help(ostream& sout, istream& sinput)
    {
        sout << "----------------------------------" << endl
             << "TaskManipulation Problem Commands:" << endl;
        GetCommandHelp(sout);
        sout << "----------------------------------" << endl;
        return true;
    }


protected:
    
    boost::shared_ptr<Trajectory> _MoveArm(const vector<int>& activejoints, const vector<dReal>& activegoalconfig, int& nGoalIndex, int nMaxIterations)
    {
        RAVELOG_DEBUGA("Starting MoveArm...\n");
        assert( !!_pRRTPlanner );
        boost::shared_ptr<Trajectory> ptraj;
        RobotBase::RobotStateSaver _saver(_robot);

        if( activejoints.size() == 0 ) {
            RAVELOG_WARNA("move arm failed\n");
            return ptraj;
        }

        if( (activegoalconfig.size()%activejoints.size()) != 0 ) {
            RAVELOG_WARNA("Active goal configurations not a multiple (%"PRIdS"/%"PRIdS")\n", activegoalconfig.size(), activejoints.size());
            return ptraj;
        }

        if( GetEnv()->CheckCollision(_robot) )
            RAVELOG_WARNA("Hand in collision\n");
    
        PlannerBase::PlannerParameters params;
    
        _robot->SetActiveDOFs(activejoints);
        vector<dReal> pzero;
        _robot->GetActiveDOFValues(pzero);

        // make sure the initial and goal configs are not in collision
        params.vgoalconfig.reserve(activegoalconfig.size());
        vector<dReal> vgoals;

        for(int i = 0; i < (int)activegoalconfig.size(); i += activejoints.size()) {
            _robot->SetActiveDOFValues(NULL, &activegoalconfig[i], true);

            // jitter only the manipulator! (jittering the hand causes big probs)
            if( JitterActiveDOF(_robot) ) {
                _robot->GetActiveDOFValues(vgoals);
                params.vgoalconfig.insert(params.vgoalconfig.end(), vgoals.begin(), vgoals.end());
            }
        }

        if( params.vgoalconfig.size() == 0 )
            return ptraj;

        // restore
        _robot->SetActiveDOFValues(NULL, &pzero[0]);
    
        // jitter again for initial collision
        if( !JitterActiveDOF(_robot) )
            return ptraj;

        _robot->GetActiveDOFValues(params.vinitialconfig);
        ptraj.reset(GetEnv()->CreateTrajectory(_robot->GetActiveDOF()));

        params.nMaxIterations = nMaxIterations; // max iterations before failure

        bool bSuccess = false;
        RAVEPRINT(L"starting planning\n");
    
        stringstream ss;

        for(int iter = 0; iter < 3; ++iter) {

            if( !_pRRTPlanner->InitPlan(_robot, &params) ) {
                RAVELOG_WARNA("InitPlan failed\n");
                ptraj.reset();
                return ptraj;
            }
        
            if( _pRRTPlanner->PlanPath(ptraj.get(), &ss) ) {
                ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);
                ss >> nGoalIndex; // extract the goal index
                assert( nGoalIndex >= 0 && nGoalIndex < (int)params.vgoalconfig.size()/(int)activejoints.size() );
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

    bool _SetFullTrajectory(Trajectory* pfulltraj, bool bExecute, const string& strsavetraj, ostream* pout)
    {
        assert( pfulltraj != NULL );
        if( pfulltraj->GetPoints().size() == 0 )
            return false;

        bool bExecuted = false;
        if( bExecute ) {
            if( pfulltraj->GetPoints().size() > 1 ) {
                _robot->SetMotion(pfulltraj);
                bExecute = true;
            }
            // have to set anyway since calling script will orEnvWait!
            else if( _robot->GetController() != NULL ) {
                if( _robot->GetController()->SetDesired(&pfulltraj->GetPoints()[0].q[0]))
                    bExecuted = true;
            }
        }

        if( strsavetraj.size() || pout != NULL ) {
            if( strsavetraj.size() > 0 )
                pfulltraj->Write(strsavetraj.c_str(), Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);

            if( pout != NULL )
                pfulltraj->Write(*pout, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation|Trajectory::TO_OneLine);
        }
    
        return bExecuted;
    }

    /// grasps using the list of grasp goals. Removes all the goals that the planner planned with
    boost::shared_ptr<Trajectory> _PlanGrasp(list<GRASPGOAL>& listGraspGoals, const vector<int>& vHandJointsRobot, int nSeedIkSolutions, GRASPGOAL& goalfound, int nMaxIterations,PRESHAPETRAJMAP& mapPreshapeTrajectories)
    {
        const RobotBase::Manipulator* pmanip = _robot->GetActiveManipulator();
        assert( _robot != NULL && pmanip != NULL );

        boost::shared_ptr<Trajectory> ptraj;

        // set all teh goals, be careful! not all goals have the same preshape!!!
        if( listGraspGoals.size() == 0 )
            return ptraj;

        RobotBase::RobotStateSaver _saver(_robot);

        // set back to the initial hand joints
        _robot->SetActiveDOFs(vHandJointsRobot);
        vector<dReal> vpreshape = listGraspGoals.front().vpreshape;
        
        _robot->SetActiveDOFValues(NULL, &vpreshape[0],true);

        list<GRASPGOAL>::iterator itgoals = listGraspGoals.begin();
        list<GRASPGOAL> listgraspsused;
    
        // take the first grasp
        listgraspsused.splice(listgraspsused.end(), listGraspGoals, itgoals++);
    
        while(itgoals != listGraspGoals.end()) {
            int ipreshape=0;
            for(ipreshape = 0; ipreshape < (int)vpreshape.size(); ++ipreshape) {
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

        vector<dReal> vgoalconfigs;
        FOREACH(itgoal, listgraspsused) {
            assert( itgoal->viksolution.size() == pmanip->_vecarmjoints.size() );
            vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());
            int nsampled = SampleIkSolutions(_robot, itgoal->tgrasp, nSeedIkSolutions, vgoalconfigs);
            if( nsampled != nSeedIkSolutions ) {
                RAVELOG_WARNA("warning, only found %d/%d ik solutions. goal indices will be wrong!\n", nsampled, nSeedIkSolutions);
                // fill the rest
                while(nsampled++ < nSeedIkSolutions)
                    vgoalconfigs.insert(vgoalconfigs.end(), itgoal->viksolution.begin(), itgoal->viksolution.end());
            }
        }

        PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vpreshape);
        if( itpreshapetraj != mapPreshapeTrajectories.end() ) {
            if( itpreshapetraj->second->GetPoints().size() > 0 )
                _robot->SetJointValues(NULL,NULL,&itpreshapetraj->second->GetPoints().back().q[0]);
        }
        else {
            RAVELOG_WARNA("no preshape trajectory!");
        }
                
        int nGraspIndex = 0;
        ptraj = _MoveArm(pmanip->_vecarmjoints, vgoalconfigs, nGraspIndex, nMaxIterations);
        if (!ptraj )
            return ptraj;

        list<GRASPGOAL>::iterator it = listgraspsused.begin();
        assert( nGraspIndex >= 0 && nGraspIndex/(1+nSeedIkSolutions) < (int)listgraspsused.size() );
        advance(it,nGraspIndex/(1+nSeedIkSolutions));
        goalfound = *it;

        boost::shared_ptr<Trajectory> pfulltraj(GetEnv()->CreateTrajectory(_robot->GetDOF()));
        _robot->SetActiveDOFs(pmanip->_vecarmjoints);
        _robot->GetFullTrajectoryFromActive(pfulltraj.get(), ptraj.get());

        RAVELOG_DEBUGA("total planning time %d ms\n", (uint32_t)(GetMicroTime()-tbase)/1000);
        return pfulltraj;
    }

    wstring _strRobotName; ///< name of the active robot
    RobotBase* _robot;
    list<boost::shared_ptr<SensorSystemBase> > listsystems;
    boost::shared_ptr<ProblemInstance> _pGrasperProblem;
    boost::shared_ptr<PlannerBase> _pRRTPlanner;
    vector< vector<pair<KinBody::Link*,Transform> > > _vvManipChildLinks;
};

#endif
