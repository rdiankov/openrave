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
#include "commonmanipulation.h"

#ifdef HAVE_BOOST_REGEX
#include <boost/regex.hpp>
#endif

#define GRASPTHRESH2 dReal(0.002f)

struct GRASPGOAL
{
    vector<dReal> vpreshape;
    IkParameterization tgrasp; ///< transform of the grasp
    vector<dReal> viksolution; ///< optional joint values for robot arm that achive the grasp goal
    list<IkParameterization> listDests; ///< transform of the grasps at the destination
    int graspindex;
    vector<dReal> vgoalconfiguration; ///< contains the configuration if this grasp was picked
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(GRASPGOAL)
#endif

class GraspVectorCompare : public RealVectorCompare
{
public:
    GraspVectorCompare() : RealVectorCompare(GRASPTHRESH2) {
    }
};

class TaskManipulation : public ModuleBase
{
public:
    typedef std::map<vector<dReal>, TrajectoryBasePtr, GraspVectorCompare > PRESHAPETRAJMAP;

    TaskManipulation(EnvironmentBasePtr penv) : ModuleBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\n\
Task-based manipulation planning involving target objects. A lot of the algorithms and theory are covered in:\n\
\n\
- Rosen Diankov. \"Automated Construction of Robotic Manipulation Programs\". PhD Thesis, CMU-RI-TR-10-29, Carnegie Mellon University, Robotics Institute, August 2010.\n";
        RegisterCommand("GraspPlanning",boost::bind(&TaskManipulation::GraspPlanning,this,_1,_2),
                        "Grasp planning is the ultimate function that puts together many planners achieving a robust and general pick and place motiotions with grasp sets. It first chooses grasps from a grasp set and use it for manipulation. In order for the arm to reach the grasps, it must have a Transform6D or TranslationDirection5D IK solver, anything with less DOF will not work.\nParameters:\n\n\
* grasps\n\
* outputtraj\n\
* execute\n\
* randomdests\n\
* writetraj\n\
* maxiter\n\
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
* grasptranslationstepmult\n\
* graspfinestep\n\
");
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
        RegisterCommand("SetMinimumGoalPaths",boost::bind(&TaskManipulation::SetMinimumGoalPathsCommand,this,_1,_2),
                        "Sets _minimumgoalpaths for all planner parameters.");
#ifdef HAVE_BOOST_REGEX
        RegisterCommand("SwitchModels",boost::bind(&TaskManipulation::SwitchModels,this,_1,_2),
                        "Switches between thin and fat models for planning.");
#endif
        _fMaxVelMult=1;
        _minimumgoalpaths=1;
    }
    virtual ~TaskManipulation()
    {
        Destroy();
    }

    virtual void Destroy()
    {
        ModuleBase::Destroy();
        listsystems.clear();
        _pGrasperPlanner.reset();
        _pRRTPlanner.reset();
        _robot.reset();
    }

    virtual void Reset()
    {
        _listSwitchModels.clear();
        ModuleBase::Reset();
        listsystems.clear();
        // recreate the planners since they store state
        if( !!_pRRTPlanner ) {
            _pRRTPlanner = RaveCreatePlanner(GetEnv(),_pRRTPlanner->GetXMLId());
        }
        if( !!_pGrasperPlanner ) {
            _pGrasperPlanner = RaveCreatePlanner(GetEnv(),_pGrasperPlanner->GetXMLId());
        }
    }

    int main(const string& args)
    {
        string name;
        stringstream ss(args);
        _fMaxVelMult=1;
        _minimumgoalpaths=1;
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
        return ModuleBase::SendCommand(sout,sinput);
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
        sout << 1;     // signal success
        return true;
    }

    class ActiveDistMetric
    {
public:
        ActiveDistMetric(RobotBasePtr robot) : _robot(robot) {
            _robot->GetActiveDOFWeights(weights);
            FOREACH(it,weights) {
                *it *= *it;
            }
        }
        virtual dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1)
        {
            dReal out = 0;
            for(int i=0; i < _robot->GetActiveDOF(); i++) {
                out += weights.at(i) * (c0.at(i)-c1.at(i))*(c0[i]-c1[i]);
            }
            return RaveSqrt(out);
        }

protected:
        RobotBasePtr _robot;
        vector<dReal> weights;
    };

    bool EvaluateConstraints(ostream& sout, istream& sinput)
    {
        Transform tTargetWorldFrame, tConstraintTaskFrame;
        boost::array<double,6> vfreedoms = { { 1,1,1,1,1,1}};
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
            else if( cmd == "constraintpose" ) {
                sinput >> tTargetWorldFrame;
            }
            else if( cmd == "constrainttaskmatrix" ) {
                TransformMatrix m; sinput >> m; tConstraintTaskFrame = m;
            }
            else if( cmd == "constrainttaskpose" ) {
                sinput >> tConstraintTaskFrame;
            }
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
        vector<dReal> vprev, vdelta;
        _robot->GetActiveDOFValues(vprev);
        CM::GripperJacobianConstrains<double> constraints(_robot->GetActiveManipulator(),tTargetWorldFrame,tConstraintTaskFrame, vfreedoms,errorthresh);
        constraints._distmetricfn = boost::bind(&ActiveDistMetric::Eval,&distmetric,_1,_2);
        vdelta.resize(vprev.size());
        FOREACH(itconfig,listconfigs) {
            _robot->SetActiveDOFValues(*itconfig);
            for(size_t j = 0; j < vprev.size(); ++j) {
                vdelta[j] = (*itconfig)[j] - vprev[j];
            }
            constraints.RetractionConstraint(vprev,vdelta);
            sout << constraints._iter << " ";
        }
        FOREACH(itconfig,listconfigs) {
            FOREACH(it,*itconfig) {
                sout << *it << " ";
            }
        }

        return true;
    }

    bool GraspPlanning(ostream& sout, istream& sinput)
    {
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();

        vector<dReal> vgrasps;
        boost::shared_ptr<GraspParameters> graspparams(new GraspParameters(GetEnv()));

        KinBodyPtr ptarget;
        int nNumGrasps=0, nGraspDim=0;
        dReal fApproachOffset=0.0f;     // offset before approaching to the target
        string targetname;
        vector<Transform> vObjDestinations;
        string strpreshapetraj;     // save the preshape trajectory
        bool bCombinePreShapeTraj = true;
        bool bExecute = true;
        string strtrajfilename;
        bool bRandomDests = true, bRandomGrasps = true;     // if true, permute the grasps and destinations when iterating through them
        boost::shared_ptr<ostream> pOutputTrajStream;
        int nMaxSeedGrasps = 20, nMaxSeedDests = 5, nMaxSeedIkSolutions = 0;
        int nMaxIterations = 4000;
        bool bQuitAfterFirstRun = false;
        dReal jitter = 0.03;
        int nJitterIterations = 5000;

        // indices into the grasp table
        int iGraspDir = -1, iGraspPos = -1, iGraspRoll = -1, iGraspPreshape = -1, iGraspStandoff = -1, imanipulatordirection = -1;
        int iGraspTransform = -1;     // if >= 0, use the grasp transform to check for collisions
        int iGraspTransformNoCol = -1;
        int iStartCountdown = 40;
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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
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
            else if( cmd == "igraspdir" ) {
                sinput >> iGraspDir;
            }
            else if( cmd == "igrasppos" ) {
                sinput >> iGraspPos;
            }
            else if( cmd == "imanipulatordirection" ) {
                sinput >> imanipulatordirection;
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
            else if( cmd == "grasptrans_nocol" ) {
                sinput >> iGraspTransformNoCol;
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
            else if( cmd == "countdowngrasps" ) {
                sinput >> iStartCountdown;
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
            else if( cmd == "grasptranslationstepmult" ) {
                sinput >> graspparams->ftranslationstepmult;
            }
            else if( cmd == "graspfinestep" ) {
                sinput >> graspparams->ffinestep;
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
            throw OPENRAVE_EXCEPTION_FORMAT("manipulator %s is already grasping %s", pmanip->GetName()%ptarget->GetName(),ORE_InvalidArguments);
        }
        RobotBase::RobotStateSaver saver(_robot);

        int nMobileAffine = _robot->GetAffineDOF();     // if mobile base, cannot rely on IK
        if( nMobileAffine ) {
            RAVELOG_INFO("planning with mobile base!\n");
        }

        vector<int> vindices(pmanip->GetArmIndices().size()+pmanip->GetGripperIndices().size());
        std::copy(pmanip->GetArmIndices().begin(),pmanip->GetArmIndices().end(),vindices.begin());
        std::copy(pmanip->GetGripperIndices().begin(),pmanip->GetGripperIndices().end(),vindices.begin()+pmanip->GetArmIndices().size());
        _robot->SetActiveDOFs(vindices, nMobileAffine, _robot->GetAffineRotationAxis());
        ConfigurationSpecification specfinal = _robot->GetActiveConfigurationSpecification();
        specfinal.AddDerivativeGroups(1,true);
        specfinal.AddDeltaTimeGroup();

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        vector<dReal> vinsertconfiguration; // configuration to add at the beginning of the trajectory, usually it is in collision
        // jitter again for initial collision

        vector<dReal> vCurHandValues, vCurRobotValues, vHandLowerLimits, vHandUpperLimits;
        _robot->SetActiveDOFs(pmanip->GetGripperIndices());
        _robot->GetActiveDOFValues(vCurHandValues);
        _robot->GetActiveDOFLimits(vHandLowerLimits,vHandUpperLimits);
        _robot->GetDOFValues(vCurRobotValues);

        SwitchModelState switchstate(shared_problem());
        _UpdateSwitchModels(true,true);

        // check if robot is in collision with padded models
        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) ) {
            _robot->SetActiveDOFs(pmanip->GetArmIndices());
            switch( planningutils::JitterActiveDOF(_robot,nJitterIterations,jitter) ) {
            case 0:
                RAVELOG_WARN("jitter failed for initial\n");
                return false;
            case 1:
                RAVELOG_DEBUG("original robot position in collision, so jittered out of it\n");
                vinsertconfiguration = vCurRobotValues;
                _robot->GetDOFValues(vCurRobotValues);
                break;
            }
        }

        string strResponse;
        Transform transTarg = ptarget->GetTransform();
        Transform transInvTarget = transTarg.inverse();
        Transform transDummy(Vector(1,0,0,0), Vector(100,0,0));
        vector<dReal> viksolution, vikgoal, vFinalGripperValues;

        PRESHAPETRAJMAP mapPreshapeTrajectories;
        {
            // fill with a trajectory with one point
            TrajectoryBasePtr pstarttraj = RaveCreateTrajectory(GetEnv(),"");
            pstarttraj->Init(_robot->GetConfigurationSpecification());
            vector<dReal> v;
            _robot->GetConfigurationValues(v);
            pstarttraj->Insert(0,v);
            mapPreshapeTrajectories[vCurHandValues] = pstarttraj;
        }

        IkReturnPtr ikreturn(new IkReturn(IKRA_Success));
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
        if( iGraspTransformNoCol < 0 ) {
            if( !_pGrasperPlanner ) {
                RAVELOG_ERROR("grasper problem not valid\n");
                return false;
            }
            if(( iGraspDir < 0) ||( iGraspPos < 0) ||( iGraspRoll < 0) ||( iGraspStandoff < 0) ) {
                RAVELOG_ERROR("grasp indices not all initialized\n");
                return false;
            }
        }

        _phandtraj.reset();

        vector<dReal> vtrajdata;
        UserDataPtr ikfilter;
        if( pmanip->GetIkSolver()->Supports(IKP_TranslationDirection5D) ) {
            // if 5D, have to set a filter
            ikfilter = pmanip->GetIkSolver()->RegisterCustomFilter(0,boost::bind(&TaskManipulation::_FilterIkForGrasping,shared_problem(),_1,_2,_3,ptarget));
            fApproachOffset = 0; // cannot approach
        }

        for(int igraspperm = 0; igraspperm < (int)vgrasppermuation.size(); ++igraspperm) {
            int igrasp = vgrasppermuation[igraspperm];
            dReal* pgrasp = &vgrasps[igrasp*nGraspDim];

            if(( listGraspGoals.size() > 0) &&( iCountdown-- <= 0) ) {
                // start planning
                _UpdateSwitchModels(true,false);

                RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
                uint64_t basestart = utils::GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, nMaxSeedIkSolutions, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += utils::GetMicroTime() - basestart;

                if( !!ptraj || bQuitAfterFirstRun ) {
                    break;
                }
            }

            vector<dReal> vgoalpreshape(vCurHandValues.size());
            if( iGraspPreshape >= 0 ) {
                bool badpreshape = false;
                for(size_t j = 0; j < vCurHandValues.size(); ++j) {
                    vgoalpreshape[j] = pgrasp[iGraspPreshape+j];
                    if( vHandLowerLimits.at(j) > vgoalpreshape[j]+0.001 || vHandUpperLimits.at(j) < vgoalpreshape[j]-0.001 ) {
                        RAVELOG_WARN(str(boost::format("bad preshape index %d (%f)!")%j%vgoalpreshape[j]));
                        badpreshape = true;
                        break;
                    }
                }
                if( badpreshape ) {
                    continue;
                }
            }
            else {
                vgoalpreshape.resize(pmanip->GetGripperIndices().size());
                for(size_t j = 0; j < pmanip->GetGripperIndices().size(); ++j) {
                    vgoalpreshape[j] = vCurRobotValues[pmanip->GetGripperIndices()[j]];
                }
            }

            PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vgoalpreshape);
            if(( itpreshapetraj != mapPreshapeTrajectories.end()) && !itpreshapetraj->second ) {
                // has failed to find a trajectory to open a hand on a previous attempt, so skip
                RAVELOG_DEBUG("grasp %d: skipping failed preshape\n", igrasp);
                continue;
            }

            _UpdateSwitchModels(false,false);

            IkParameterization tGoalEndEffector;
            // set the goal preshape
            _robot->SetActiveDOFs(pmanip->GetGripperIndices(), DOF_NoTransform);
            _robot->SetActiveDOFValues(vgoalpreshape,true);

            dReal fGraspApproachOffset = fApproachOffset;

            if( !!_pGrasperPlanner && pmanip->GetIkSolver()->Supports(IKP_Transform6D) ) {
                _robot->SetActiveDOFs(pmanip->GetGripperIndices(), DOF_X|DOF_Y|DOF_Z);
                if( !_phandtraj ) {
                    _phandtraj = RaveCreateTrajectory(GetEnv(),"");
                }
                _phandtraj->Init(_robot->GetActiveConfigurationSpecification());
                graspparams->fstandoff = pgrasp[iGraspStandoff];
                graspparams->targetbody = ptarget;
                graspparams->ftargetroll = pgrasp[iGraspRoll];
                graspparams->vtargetdirection = Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]);
                graspparams->vtargetposition = Vector(pgrasp[iGraspPos], pgrasp[iGraspPos+1], pgrasp[iGraspPos+2]);
                if( imanipulatordirection >= 0 ) {
                    graspparams->vmanipulatordirection = Vector(pgrasp[imanipulatordirection], pgrasp[imanipulatordirection+1], pgrasp[imanipulatordirection+2]);
                }
                else {
                    graspparams->vmanipulatordirection = pmanip->GetDirection();
                }
                graspparams->btransformrobot = true;
                graspparams->breturntrajectory = false;
                graspparams->bonlycontacttarget = true;
                graspparams->btightgrasp = false;
                graspparams->bavoidcontact = true;
                // TODO: in order to reproduce the same exact conditions as the original grasp, have to also transfer the step sizes

                if( !_pGrasperPlanner->InitPlan(_robot,graspparams) ) {
                    RAVELOG_DEBUG("grasper planner failed: %d\n", igrasp);
                    continue;
                }

                if( !_pGrasperPlanner->PlanPath(_phandtraj) ) {
                    RAVELOG_DEBUG("grasper planner failed: %d\n", igrasp);
                    continue;
                }

                BOOST_ASSERT(_phandtraj->GetNumWaypoints()>0);
                _phandtraj->GetWaypoint(-1,vtrajdata);
                Transform t = _robot->GetTransform();
                _phandtraj->GetConfigurationSpecification().ExtractTransform(t,vtrajdata.begin(),_robot);

                Vector vglobalpalmdir;
                if( iGraspDir >= 0 ) {
                    vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                }
                else {
                    vglobalpalmdir = pmanip->GetTransform().rotate(pmanip->GetDirection());
                }

                // move back a little if robot/target in collision
                if( !!ptarget ) {
                    RobotBase::RobotStateSaver saverlocal(_robot);
                    _robot->SetTransform(t);
                    dReal fstep=0;
                    dReal fstepbacksize = 0.001f;
                    while(GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(ptarget))) {
                        t.trans -= vglobalpalmdir*fstepbacksize;
                        fGraspApproachOffset -= fstepbacksize;
                        fstep += fstepbacksize;
                        _robot->SetTransform(t);
                    }
                    if( fstep > 0 ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: moved %f along direction=[%f,%f,%f]")%igrasp%fstep% -vglobalpalmdir.x% -vglobalpalmdir.y% -vglobalpalmdir.z));
                    }
                }

                // find the end effector transform
                tGoalEndEffector.SetTransform6D(t * _robot->GetTransform().inverse() * pmanip->GetTransform());

                if( iGraspTransform >= 0 ) {
                    // use the grasp transform to figure out how much backing to compensate for, this is just a sanity check
                    dReal* pm = pgrasp+iGraspTransform;
                    TransformMatrix tmexpected;
                    tmexpected.m[0] = pm[0]; tmexpected.m[1] = pm[3]; tmexpected.m[2] = pm[6]; tmexpected.trans.x = pm[9];
                    tmexpected.m[4] = pm[1]; tmexpected.m[5] = pm[4]; tmexpected.m[6] = pm[7]; tmexpected.trans.y = pm[10];
                    tmexpected.m[8] = pm[2]; tmexpected.m[9] = pm[5]; tmexpected.m[10] = pm[8]; tmexpected.trans.z = pm[11];
                    Transform texpectedglobal = ptarget->GetTransform() * Transform(tmexpected);
                    dReal dist = vglobalpalmdir.dot3(tGoalEndEffector.GetTransform6D().trans-texpectedglobal.trans);
                    fGraspApproachOffset = fApproachOffset+dist;
                }

                if( fGraspApproachOffset < 0 ) {
                    RAVELOG_WARN(str(boost::format("grasp %d: moved too far back to avoid collision, approach offset is now negative (%f) and cannot recover. Should increase approachoffset")%igrasp%fGraspApproachOffset));
                }

                vFinalGripperValues.resize(pmanip->GetGripperIndices().size(),0);
                _phandtraj->GetConfigurationSpecification().ExtractJointValues(vFinalGripperValues.begin(),vtrajdata.begin(),_robot,pmanip->GetGripperIndices());
            }
            else if( iGraspTransformNoCol >= 0 ) {
                vFinalGripperValues.resize(0);

                // use the grasp transform
                dReal* pm = pgrasp+iGraspTransformNoCol;
                TransformMatrix tm, tgoal;
                tm.m[0] = pm[0]; tm.m[1] = pm[3]; tm.m[2] = pm[6]; tm.trans.x = pm[9];
                tm.m[4] = pm[1]; tm.m[5] = pm[4]; tm.m[6] = pm[7]; tm.trans.y = pm[10];
                tm.m[8] = pm[2]; tm.m[9] = pm[5]; tm.m[10] = pm[8]; tm.trans.z = pm[11];
                if( !ptarget ) {
                    tgoal = tm;
                }
                else {
                    tgoal = ptarget->GetTransform() * Transform(tm);
                }

                if( pmanip->GetIkSolver()->Supports(IKP_TranslationDirection5D) ) {
                    // get a valid transformation
                    tGoalEndEffector.SetTranslationDirection5D(RAY(tgoal.trans,tgoal.rotate(pmanip->GetDirection())));
                    if( !pmanip->FindIKSolution(tGoalEndEffector,IKFO_CheckEnvCollisions, ikreturn) ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: ik 5d failed reason 0x%x")%igrasp%ikreturn->_action));
                        continue; // failed
                    }
                    vFinalGripperValues = _vFinalGripperValues;
                }
                else if( pmanip->GetIkSolver()->Supports(IKP_Transform6D) ) {
                    tGoalEndEffector.SetTransform6D(tgoal);
                    KinBody::KinBodyStateSaver saver(ptarget,KinBody::Save_LinkEnable);
                    ptarget->Enable(false);
                    if( pmanip->CheckEndEffectorCollision(tgoal,report) ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: in collision (%s)\n")%igrasp%report->__str__()));
                        continue;
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("manipulator %s does not support ik types transform6d or translationdirection5d necessing for grasp planning\n", pmanip->GetName(), ORE_InvalidArguments);
                }
            }
            else {
                RAVELOG_ERROR(str(boost::format("grasp %d: grasper problem not valid")%igrasp));
                return false;
            }

            // set the initial hand joints
            _robot->SetActiveDOFs(pmanip->GetGripperIndices());
            if( pmanip->GetGripperIndices().size() > 0 && iGraspPreshape >= 0 ) {
                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
            }

            IkParameterization tApproachEndEffector = tGoalEndEffector;
            if( !nMobileAffine ) {
                // check ik
                Vector vglobalpalmdir;
                if( iGraspDir >= 0 ) {
                    vglobalpalmdir = transTarg.rotate(Vector(pgrasp[iGraspDir], pgrasp[iGraspDir+1], pgrasp[iGraspDir+2]));
                }
                else {
                    if( tApproachEndEffector.GetType() == IKP_Transform6D ) {
                        vglobalpalmdir = tApproachEndEffector.GetTransform6D().rotate(pmanip->GetDirection());
                    }
                    else {
                        vglobalpalmdir = tApproachEndEffector.GetTranslationDirection5D().dir;
                    }
                }

                // first test the IK solution at the destination tGoalEndEffector
                if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, IKFO_CheckEnvCollisions) ) {
                    RAVELOG_DEBUG("grasp %d: No IK solution found (final)\n", igrasp);
                    continue;
                }

                _UpdateSwitchModels(true,true);     // switch to fat models
                if( fGraspApproachOffset > 0 ) {
                    Transform tsmalloffset;
                    // now test at the approach point (with offset)
                    tsmalloffset.trans = -fGraspApproachOffset * vglobalpalmdir;
                    tApproachEndEffector = tsmalloffset*tApproachEndEffector;

                    // set the previous robot ik configuration to get the closest configuration!!
                    _robot->SetActiveDOFs(pmanip->GetArmIndices());
                    _robot->SetActiveDOFValues(viksolution);
                    if( !pmanip->FindIKSolution(tApproachEndEffector, viksolution, IKFO_CheckEnvCollisions) ) {
                        _robot->SetDOFValues(vCurRobotValues);     // reset robot to original position
                        RAVELOG_DEBUG("grasp %d: No IK solution found (approach)\n", igrasp);
                        continue;
                    }
                    else {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        ss << "ikfound = [";
                        FOREACH(it, viksolution) {
                            ss << *it << ", ";
                        }
                        ss << "]" << endl;
                        RAVELOG_DEBUG(ss.str());
                    }
                }
            }

            // set the joints that the grasper plugin calculated
            _robot->SetActiveDOFs(pmanip->GetGripperIndices());
            if( vFinalGripperValues.size() > 0 ) {
                _robot->SetActiveDOFValues(vFinalGripperValues, true);
            }

            //while (getchar() != '\n') usleep(1000);
            _UpdateSwitchModels(false,false);     // should test destination with thin models

            list< IkParameterization > listDests;
            if( bRandomDests ) {
                PermutateRandomly(vdestpermuation);
            }

            for(int idestperm = 0; idestperm < (int)vdestpermuation.size(); ++idestperm) {
                Transform& transDestTarget = vObjDestinations[vdestpermuation[idestperm]];
                IkParameterization tDestEndEffector = (transDestTarget * transInvTarget) * tGoalEndEffector;
                ptarget->SetTransform(transDestTarget);
                bool bTargetCollision;
                {
                    RobotBase::RobotStateSaver linksaver(_robot,KinBody::Save_LinkEnable);
                    _robot->Enable(false);     // remove robot from target collisions
                    bTargetCollision = GetEnv()->CheckCollision(KinBodyConstPtr(ptarget),report);
                }

                ptarget->SetTransform(transTarg);
                if( bTargetCollision ) {
                    RAVELOG_VERBOSE(str(boost::format("target collision at dest %d: %s")%vdestpermuation[idestperm]%report->__str__()));
                    continue;
                }

                if( !nMobileAffine ) {
                    if( pmanip->FindIKSolution(tDestEndEffector, vikgoal, IKFO_CheckEnvCollisions) ) {
                        listDests.push_back(tDestEndEffector);
                    }
                }
                else {
                    listDests.push_back(tDestEndEffector);
                }

                if( (int)listDests.size() >= nMaxSeedDests ) {
                    break;
                }
            }

            _robot->SetDOFValues(vCurRobotValues);     // reset robot to original position

            if(( vObjDestinations.size() > 0) &&( listDests.size() == 0) ) {
                RAVELOG_WARN("grasp %d: could not find destination\n", igrasp);
                continue;
            }

            // finally start planning
            _UpdateSwitchModels(true,false);

            if( pmanip->GetGripperIndices().size() > 0 && itpreshapetraj == mapPreshapeTrajectories.end() ) {
                // not present in map, so look for correct one
                // initial joint is far from desired preshape, have to plan to get to it
                // note that this changes trajectory of robot!
                _robot->SetActiveDOFValues(vCurHandValues, true);

                _robot->SetActiveDOFs(pmanip->GetArmIndices());
                TrajectoryBasePtr ptrajToPreshape = RaveCreateTrajectory(GetEnv(),"");
                ptrajToPreshape->Init(specfinal);
                bool bSuccess = CM::MoveUnsync::_MoveUnsyncJoints(GetEnv(), _robot, ptrajToPreshape, pmanip->GetGripperIndices(), vgoalpreshape);

                if( !bSuccess ) {
                    mapPreshapeTrajectories[vgoalpreshape].reset();     // set to empty
                    RAVELOG_DEBUG("grasp %d: failed to find preshape\n", igrasp);
                    continue;
                }

                // add a grasp with the full preshape
                specfinal += ptrajToPreshape->GetConfigurationSpecification(); //. don't want to lose any extra information the smoothers might have added
                if( iGraspPreshape >= 0 ) {
                    planningutils::ConvertTrajectorySpecification(ptrajToPreshape,specfinal);
                    vector<dReal> vpreshapevalues(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+pmanip->GetGripperIndices().size());
                    _robot->SetActiveDOFs(pmanip->GetGripperIndices());
                    // actually should use _phandtraj
                    planningutils::InsertActiveDOFWaypointWithRetiming(ptrajToPreshape->GetNumWaypoints(), vpreshapevalues, std::vector<dReal>(), ptrajToPreshape, _robot, _fMaxVelMult);
                }

                mapPreshapeTrajectories[vgoalpreshape] = ptrajToPreshape;
            }

//            itpreshapetraj = mapPreshapeTrajectories.find(vgoalpreshape);
//            BOOST_ASSERT(itpreshapetraj != mapPreshapeTrajectories.end());

//            if( iGraspPreshape >= 0 ) {
//                _robot->SetActiveDOFValues(vector<dReal>(pgrasp+iGraspPreshape,pgrasp+iGraspPreshape+_robot->GetActiveDOF()),true);
//            }

            listGraspGoals.push_back(GRASPGOAL());
            GRASPGOAL& goal = listGraspGoals.back();
            goal.graspindex = igrasp;
            goal.tgrasp = tApproachEndEffector;
            goal.viksolution = viksolution;
            goal.listDests.swap(listDests);
            goal.vpreshape.resize(pmanip->GetGripperIndices().size());
            if( iGraspPreshape >= 0 ) {
                for(int j = 0; j < (int)goal.vpreshape.size(); ++j) {
                    goal.vpreshape[j] = pgrasp[iGraspPreshape+j];
                }
            }

            RAVELOG_DEBUG("grasp %d: adding to goals\n", igrasp);
            iCountdown = iStartCountdown;

            if( (int)listGraspGoals.size() >= nMaxSeedGrasps ) {
                RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
                uint64_t basestart = utils::GetMicroTime();
                ptraj = _PlanGrasp(listGraspGoals, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
                nSearchTime += utils::GetMicroTime() - basestart;
                if( bQuitAfterFirstRun ) {
                    break;
                }
            }

            if( !!ptraj ) {
                break;
            }
        }

        // if there's left over goal positions, start planning
        while( !ptraj && listGraspGoals.size() > 0 ) {
            //TODO have to update ptrajToPreshape
            RAVELOG_VERBOSE(str(boost::format("planning grasps %d\n")%listGraspGoals.size()));
            uint64_t basestart = utils::GetMicroTime();
            ptraj = _PlanGrasp(listGraspGoals, nMaxSeedGrasps, goalFound, nMaxIterations,mapPreshapeTrajectories);
            nSearchTime += utils::GetMicroTime() - basestart;
        }

        _UpdateSwitchModels(false,false);

        if( !ptraj ) {
            return false;     // couldn't not find any grasps
        }

        PRESHAPETRAJMAP::iterator itpreshapetraj;
        if( pmanip->GetGripperIndices().size() > 0 ) {
            itpreshapetraj = mapPreshapeTrajectories.find(goalFound.vpreshape);
            if( itpreshapetraj == mapPreshapeTrajectories.end() ) {
                std::stringstream ss;
                ss << "no preshape trajectory where there should have been one!" << endl;
                RAVELOG_ERROR("no preshape trajectory!\n");
                FOREACH(itpreshape,mapPreshapeTrajectories) {
                    FOREACHC(itvalue,itpreshape->first) {
                        ss << *itvalue << ", ";
                    }
                    ss << endl;
                }
                throw openrave_exception(ss.str(),ORE_InconsistentConstraints);
            }
        }

        specfinal += ptraj->GetConfigurationSpecification();
        TrajectoryBasePtr ptrajfinal = RaveCreateTrajectory(GetEnv(),"");
        ptrajfinal->Init(specfinal);

        if( vinsertconfiguration.size() > 0 ) {
            _robot->SetActiveDOFs(pmanip->GetArmIndices());
            _robot->SetDOFValues(vinsertconfiguration);
            _robot->GetActiveDOFValues(vtrajdata);

            vector<int> vindices(_robot->GetDOF());
            for(size_t i = 0; i < vindices.size(); ++i) {
                vindices[i] = i;
            }
            ptrajfinal->Insert(0,vCurRobotValues,_robot->GetConfigurationSpecificationIndices(vindices));
            planningutils::InsertActiveDOFWaypointWithRetiming(0, vtrajdata, std::vector<dReal>(), ptrajfinal, _robot, _fMaxVelMult);
        }

        _robot->SetDOFValues(vCurRobotValues);

        if( bCombinePreShapeTraj && pmanip->GetGripperIndices().size() > 0 ) {     // add the preshape
            RAVELOG_DEBUG(str(boost::format("combine preshape trajectory, duration=%f")%itpreshapetraj->second->GetDuration()));
            itpreshapetraj->second->GetWaypoints(0,itpreshapetraj->second->GetNumWaypoints(),vtrajdata,specfinal);
            ptrajfinal->Insert(ptrajfinal->GetNumWaypoints(),vtrajdata);
            // set the last point so the converters can pick it up
            ptrajfinal->GetWaypoint(-1,vtrajdata,_robot->GetConfigurationSpecification());
            _robot->SetConfigurationValues(vtrajdata.begin(),true);
        }

        ptraj->GetWaypoints(0,ptraj->GetNumWaypoints(),vtrajdata,specfinal);
        ptrajfinal->Insert(ptrajfinal->GetNumWaypoints(),vtrajdata);

        if( strpreshapetraj.size() > 0 ) {     // write the preshape
            ofstream f(strpreshapetraj.c_str());
            itpreshapetraj->second->serialize(f);
        }

        RAVELOG_DEBUG("grasp index %d\n",goalFound.graspindex);
        sout << goalFound.listDests.size() << " ";
        FOREACH(itdest, goalFound.listDests) {
            sout << *itdest << " ";
        }
        sout << goalFound.graspindex << " " << (float)nSearchTime/1000000.0f << " ";
        CM::SetActiveTrajectory(_robot,ptrajfinal, bExecute, strtrajfilename, pOutputTrajStream);
        return true;
    }

    bool CloseFingers(ostream& sout, istream& sinput)
    {
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
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
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
                graspparams->ffinestep = graspparams->fcoarsestep * 0.01;     // always make it 100x more accurate
            }
            else if( cmd == "finestep" ) {
                sinput >> graspparams->ffinestep;
            }
            else if( cmd == "translationstepmult" ) {
                sinput >> graspparams->ftranslationstepmult;
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

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_robot->GetActiveConfigurationSpecification());
        ptraj->Insert(0,graspparams->vinitialconfig); // have to add the first point

        if( !graspplanner->InitPlan(_robot, graspparams) ) {
            RAVELOG_ERROR("InitPlan failed\n");
            return false;
        }

        if( !graspplanner->PlanPath(ptraj) ) {
            RAVELOG_WARN("PlanPath failed\n");
            return false;
        }

        if( ptraj->GetNumWaypoints() == 0 ) {
            return false;
        }
        if( bOutputFinal ) {
            vector<dReal> q;
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            FOREACH(itq,q) {
                sout << *itq << " ";
            }
        }

        vector<dReal> vlastvalues;
        ptraj->GetWaypoint(-1,vlastvalues,_robot->GetActiveConfigurationSpecification());
        if(vlastvalues.size() == voffset.size() ) {
            for(size_t i = 0; i < voffset.size(); ++i) {
                vlastvalues[i] += voffset[i]*pmanip->GetClosingDirection().at(i);
            }
            _robot->SetActiveDOFValues(vlastvalues,true);
            _robot->GetActiveDOFValues(vlastvalues);
            ptraj->Insert(ptraj->GetNumWaypoints(),vlastvalues,_robot->GetActiveConfigurationSpecification());
        }

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool ReleaseFingers(ostream& sout, istream& sinput)
    {
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
            else if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "outputfinal" ) {
                bOutputFinal = true;
            }
            else if( cmd == "movingdir" ) {
                FOREACH(it,graspparams->vgoalconfig) {
                    sinput >> *it;
                }
            }
            else if( cmd == "coarsestep" ) {
                sinput >> graspparams->fcoarsestep;
                graspparams->ffinestep = graspparams->fcoarsestep * 0.01;     // always make it 100x more accurate
            }
            else if( cmd == "finestep" ) {
                sinput >> graspparams->ffinestep;
            }
            else if( cmd == "translationstepmult" ) {
                sinput >> graspparams->ftranslationstepmult;
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
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_robot->GetActiveConfigurationSpecification());

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        // have to add the first point
        vector<dReal> vinitialconfig;
        _robot->GetActiveDOFValues(vinitialconfig);
        ptraj->Insert(0,vinitialconfig);
        switch(planningutils::JitterActiveDOF(_robot) ) {
        case 0:
            RAVELOG_WARN("robot initially in collision\n");
            return false;
        case 1:
            _robot->GetActiveDOFValues(vinitialconfig);
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

        if( ptraj->GetNumWaypoints() == 0 ) {
            return false;
        }

        vector<dReal> q;
        if( bOutputFinal ) {
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            FOREACH(itq,q) {
                sout << *itq << " ";
            }
        }

        bool bForceRetime = false;
        {
            // check final trajectory for colliding points
            RobotBase::RobotStateSaver saver2(_robot);
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            _robot->SetActiveDOFValues(q);
            if( planningutils::JitterActiveDOF(_robot) > 0 ) {
                RAVELOG_WARN("robot final configuration is in collision\n");
                _robot->GetActiveDOFValues(q);
                ptraj->Insert(ptraj->GetNumWaypoints(),q,_robot->GetActiveConfigurationSpecification());
                bForceRetime = true;
            }
        }

        if( !!ptarget ) {
            _robot->Release(ptarget);
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            _robot->SetActiveDOFValues(q);
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot),KinBodyConstPtr(ptarget)) ) {
                RAVELOG_WARN(str(boost::format("even after releasing, in collision with target %s")%_robot->GetName()));
            }
        }
        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool ReleaseActive(ostream& sout, istream& sinput)
    {
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
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "execute" ) {
                sinput >> bExecute;
            }
            else if( cmd == "outputtraj" ) {
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,utils::null_deleter());
            }
            else if( cmd == "outputfinal" ) {
                bOutputFinal = true;
            }
            else if( cmd == "writetraj" ) {
                sinput >> strtrajfilename;
            }
            else if( cmd == "finestep" ) {
                sinput >> graspparams->ffinestep;
            }
            else if( cmd == "translationstepmult" ) {
                sinput >> graspparams->ftranslationstepmult;
            }
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
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        ptraj->Init(_robot->GetActiveConfigurationSpecification());

        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);

        // have to add the first point
        vector<dReal> vinitialconfig;
        _robot->GetActiveDOFValues(vinitialconfig);
        ptraj->Insert(0,vinitialconfig);
        switch( planningutils::JitterActiveDOF(_robot) ) {
        case 0:
            RAVELOG_WARN("robot initially in collision\n");
            return false;
        case 1:
            _robot->GetActiveDOFValues(vinitialconfig);
            break;
        }

        PlannerBasePtr graspplanner = RaveCreatePlanner(GetEnv(),"Grasper");
        if( !graspplanner ) {
            RAVELOG_ERROR("grasping planner failure!\n");
            return false;
        }

        _robot->SetActiveManipulator(RobotBase::ManipulatorPtr());     // reset the manipulator
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

        if( ptraj->GetNumWaypoints() == 0 ) {
            return false;
        }

        vector<dReal> q;
        if( bOutputFinal ) {
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            FOREACH(itq,q) {
                sout << *itq << " ";
            }
        }

        bool bForceRetime = false;
        {
            // check final trajectory for colliding points
            RobotBase::RobotStateSaver saver2(_robot);
            ptraj->GetWaypoint(-1,q,_robot->GetActiveConfigurationSpecification());
            _robot->SetActiveDOFValues(q);
            if( planningutils::JitterActiveDOF(_robot) > 0 ) {
                RAVELOG_WARN("robot final configuration is in collision\n");
                _robot->GetActiveDOFValues(q);
                ptraj->Insert(ptraj->GetNumWaypoints(),q,_robot->GetActiveConfigurationSpecification());
                bForceRetime = true;
            }
        }

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    class SwitchModelContainer
    {
public:
        SwitchModelContainer(KinBodyPtr pbody, const string &fatfilename) : _pbody(pbody) {
            RAVELOG_VERBOSE(str(boost::format("register %s body with %s fatfile\n")%pbody->GetName()%fatfilename));
            _pbodyfat = pbody->GetEnv()->ReadKinBodyURI(fatfilename);
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
                    (*itlink)->SwapGeometries(pswitchlink);
                }
                _bFat = bSwitchToFat;
            }
        }

        KinBodyPtr GetBody() const {
            return _pbody;
        }
        bool IsFat() const {
            return _bFat;
        }
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
        virtual ~SwitchModelState() {
            _ptask->_UpdateSwitchModels(_bFat);
        }
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
    inline boost::shared_ptr<TaskManipulation> shared_problem() {
        return boost::dynamic_pointer_cast<TaskManipulation>(shared_from_this());
    }
    inline boost::shared_ptr<TaskManipulation const> shared_problem_const() const {
        return boost::dynamic_pointer_cast<TaskManipulation const>(shared_from_this());
    }

    TrajectoryBasePtr _MoveArm(const vector<int>&activejoints, planningutils::ManipulatorIKGoalSampler& goalsampler, int& nGoalIndex, int nMaxIterations)
    {
        int nSeedIkSolutions = 8;
        int nJitterIterations = 5000;
        dReal jitter = 0.03;
        int nMaxTries = 3;
        dReal fGoalSamplingProb = 0.05;
        RAVELOG_DEBUG("Starting MoveArm...\n");
        BOOST_ASSERT( !!_pRRTPlanner );
        TrajectoryBasePtr ptraj;
        RobotBase::RobotStateSaver _saver(_robot);

        if( activejoints.size() == 0 ) {
            RAVELOG_WARN("move arm failed\n");
            return ptraj;
        }

        if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) )
            RAVELOG_WARN("Hand in collision\n");

        RRTParametersPtr params(new RRTParameters());
        params->_minimumgoalpaths = _minimumgoalpaths;
        _robot->SetActiveDOFs(activejoints);
        params->SetRobotActiveJoints(_robot);
        //params->_sPathOptimizationPlanner = ""; // no smoothing

        _robot->GetActiveDOFValues(params->vinitialconfig);

        // make sure the initial and goal configs are not in collision
        vector<dReal> vgoal;
        params->vgoalconfig.reserve(nSeedIkSolutions*_robot->GetActiveDOF());
        goalsampler.SetSamplingProb(1);
        while(nSeedIkSolutions > 0) {
            if( goalsampler.Sample(vgoal) ) {
                params->vgoalconfig.insert(params->vgoalconfig.end(), vgoal.begin(), vgoal.end());
                --nSeedIkSolutions;
            }
            else {
                --nSeedIkSolutions;
            }
        }

        if( params->vgoalconfig.size() == 0 ) {
            return ptraj;
        }

        goalsampler.SetSamplingProb(fGoalSamplingProb);
        params->_samplegoalfn = boost::bind(&planningutils::ManipulatorIKGoalSampler::Sample,&goalsampler,_1);

        // restore
        _robot->SetActiveDOFValues(params->vinitialconfig);

        vector<dReal> vinsertconfiguration; // configuration to add at the beginning of the trajectory, usually it is in collision
        // jitter again for initial collision
        switch( planningutils::JitterActiveDOF(_robot,nJitterIterations,jitter) ) {
        case 0:
            RAVELOG_WARN("jitter failed for initial\n");
            return TrajectoryBasePtr();
        case 1:
            RAVELOG_DEBUG("original robot position in collision, so jittered out of it\n");
            vinsertconfiguration = params->vinitialconfig;
            _robot->GetActiveDOFValues(params->vinitialconfig);
            break;
        }

        ptraj = RaveCreateTrajectory(GetEnv(),"");
        params->_nMaxIterations = nMaxIterations;     // max iterations before failure

        bool bSuccess = false;
        RAVELOG_VERBOSE("starting planning\n");

        stringstream ss;
        for(int iter = 0; iter < nMaxTries; ++iter) {
            if( !_pRRTPlanner->InitPlan(_robot, params) ) {
                RAVELOG_WARN("InitPlan failed\n");
                ptraj.reset();
                return ptraj;
            }

            if( _pRRTPlanner->PlanPath(ptraj) ) {
                stringstream sinput; sinput << "GetGoalIndex";
                _pRRTPlanner->SendCommand(ss,sinput);
                ss >> nGoalIndex;     // extract the goal index
                if( !ss ) {
                    RAVELOG_WARN("failed to extract goal index\n");
                }
                else {
                    RAVELOG_INFO(str(boost::format("finished planning, goal index: %d")%nGoalIndex));
                }
                bSuccess = true;
                break;
            }
            else {
                RAVELOG_WARN("PlanPath failed\n");
            }
        }

        if( !bSuccess ) {
            ptraj.reset();
        }
        if( RaveGetDebugLevel() & Level_VerifyPlans ) {
            planningutils::VerifyTrajectory(params, ptraj);
        }
        if( vinsertconfiguration.size() > 0 ) {
            planningutils::InsertActiveDOFWaypointWithRetiming(0,vinsertconfiguration,vector<dReal>(), ptraj, _robot, _fMaxVelMult);
        }
        return ptraj;
    }

    /// grasps using the list of grasp goals. Removes all the goals that the planner planned with
    TrajectoryBasePtr _PlanGrasp(list<GRASPGOAL>&listGraspGoals, int nSeedIkSolutions, GRASPGOAL& goalfound, int nMaxIterations,PRESHAPETRAJMAP& mapPreshapeTrajectories)
    {
        RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
        TrajectoryBasePtr ptraj;

        // set all teh goals, be careful! not all goals have the same preshape!!!
        if( listGraspGoals.size() == 0 ) {
            return ptraj;
        }
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
                if( RaveFabs(vpreshape[ipreshape] - itgoals->vpreshape[ipreshape]) > 2.0*GRASPTHRESH2 ) {
                    break;
                }
            }

            if( ipreshape == vpreshape.size() ) {
                // accept
                listgraspsused.splice(listgraspsused.end(), listGraspGoals, itgoals++);
            }
            else {
                ++itgoals;
            }
        }

        uint64_t tbase = utils::GetMicroTime();

        PRESHAPETRAJMAP::iterator itpreshapetraj = mapPreshapeTrajectories.find(vpreshape);
        if( itpreshapetraj != mapPreshapeTrajectories.end() ) {
            if( itpreshapetraj->second->GetNumWaypoints() > 0 ) {
                vector<dReal> vconfig;
                _robot->GetConfigurationValues(vconfig);
                itpreshapetraj->second->GetWaypoint(-1,vconfig,_robot->GetConfigurationSpecification());
                _robot->SetConfigurationValues(vconfig.begin(),true);
            }
        }
        else {
            RAVELOG_WARN("no preshape trajectory!");
        }

        std::list<IkParameterization> listgoals;
        FOREACH(itgoal, listgraspsused) {
            listgoals.push_back(itgoal->tgrasp);
        }
        planningutils::ManipulatorIKGoalSampler goalsampler(pmanip, listgoals);

        int nGoalIndex = -1;
        ptraj = _MoveArm(pmanip->GetArmIndices(), goalsampler, nGoalIndex, nMaxIterations);
        if (!!ptraj) {
            int nGraspIndex = goalsampler.GetIkParameterizationIndex(nGoalIndex);
            BOOST_ASSERT( nGraspIndex >= 0 && nGraspIndex < (int)listgraspsused.size() );
            list<GRASPGOAL>::iterator it = listgraspsused.begin();
            advance(it,nGraspIndex);
            goalfound = *it;
            RAVELOG_DEBUG("total planning time %d ms\n", (uint32_t)(utils::GetMicroTime()-tbase)/1000);
        }

        return ptraj;
    }

    IkReturn _FilterIkForGrasping(std::vector<dReal>&vsolution, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization &ikparam, KinBodyPtr ptarget)
    {
        if( _robot->IsGrabbing(ptarget) ) {
            return IKRA_Success;
        }
        if( ikparam.GetType() != IKP_Transform6D ) {
            // only check end effector if not trasform 6d
            if( pmanip->CheckEndEffectorCollision(pmanip->GetEndEffectorTransform()) ) {
                RAVELOG_DEBUG("grasper planner CheckEndEffectorCollision\n");
                return IKRA_Reject;
            }
        }

        if( pmanip->GetGripperIndices().size() > 0 ) {
            RobotBase::RobotStateSaver saver(_robot);
            _robot->SetActiveDOFs(pmanip->GetGripperIndices());
            if( !_phandtraj ) {
                _phandtraj = RaveCreateTrajectory(GetEnv(),"");
            }

            GraspParametersPtr graspparams(new GraspParameters(GetEnv()));
            graspparams->targetbody = ptarget;
            graspparams->btransformrobot = false;
            graspparams->breturntrajectory = false;
            graspparams->bonlycontacttarget = true;
            graspparams->btightgrasp = false;
            graspparams->bavoidcontact = true;
            // TODO: in order to reproduce the same exact conditions as the original grasp, have to also transfer the step sizes
            if( !_pGrasperPlanner->InitPlan(_robot,graspparams) ) {
                RAVELOG_DEBUG("grasper planner InitPlan failed\n");
                return IKRA_Reject;
            }

            if( !_pGrasperPlanner->PlanPath(_phandtraj) ) {
                RAVELOG_DEBUG("grasper planner PlanPath failed\n");
                return IKRA_Reject;
            }

            _phandtraj->GetWaypoint(-1,_vFinalGripperValues, _robot->GetConfigurationSpecificationIndices(pmanip->GetGripperIndices()));
        }
        else {
            _phandtraj.reset();
        }
        return IKRA_Success;
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
                    if( (*itmodel)->GetBody() == *itbody ) {
                        break;
                    }
                }
                if( itmodel != _listSwitchModels.end() ) {
                    continue;
                }
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

        FOREACH(it,_listSwitchModels) {
            (*it)->Switch(bSwitchToFat);
        }
    }

    bool SetMinimumGoalPathsCommand(ostream& sout, istream& sinput)
    {
        sinput >> _minimumgoalpaths;
        BOOST_ASSERT(_minimumgoalpaths>=0);
        return !!sinput;
    }

    string _strRobotName;     ///< name of the active robot
    RobotBasePtr _robot;
    dReal _fMaxVelMult;
    list<SensorSystemBasePtr > listsystems;
    PlannerBasePtr _pRRTPlanner, _pGrasperPlanner;
    list<pair<string,string> > _listSwitchModelPatterns;
    list<SwitchModelContainerPtr> _listSwitchModels;
    TrajectoryBasePtr _phandtraj;
    vector<dReal> _vFinalGripperValues;
    int _minimumgoalpaths;
    friend class SwitchModelState;
};

ModuleBasePtr CreateTaskManipulation(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new TaskManipulation(penv));
}
