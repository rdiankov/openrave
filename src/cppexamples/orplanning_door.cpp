/** \example orplanning_door.cpp
    \author Rosen Diankov

    Shows how to use a planner two plan for a robot opening a door.
    The configuration space consists of the robot's manipulator joints and the door joint.
    A generic planner is just passed in this configuration space inside its planner parameters.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <openrave/planningutils.h>

using namespace OpenRAVE;
using namespace std;

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#define usleep(micro) Sleep(micro/1000)
#endif

void SetViewer(EnvironmentBasePtr penv, const string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer);
    viewer->main(true);
}

/// \brief builds up the configuration space of a robot and a door
class DoorConfiguration : public boost::enable_shared_from_this<DoorConfiguration>
{
public:
    DoorConfiguration(RobotBase::ManipulatorPtr pmanip, KinBody::JointPtr pdoorjoint) : _pmanip(pmanip), _pdoorjoint(pdoorjoint) {
        _probot = pmanip->GetRobot();
        _ptarget = RaveInterfaceCast<RobotBase>(pdoorjoint->GetParent());
        _pdoorlink = _pdoorjoint->GetHierarchyChildLink();
        _tgrasp = Transform(Vector(0.65839364,  0.68616871, -0.22320624, -0.21417118),Vector(0.23126595, -0.01218956,  0.1084143));
        BOOST_ASSERT(_pdoorjoint->GetDOF()==1);
    }

    virtual ~DoorConfiguration() {
    }

    int GetDOF() {
        return (int)_pmanip->GetArmIndices().size()+1;
    }

    dReal ComputeDistance(const std::vector<dReal>& c0, const std::vector<dReal>& c1) {
        dReal d1 = _robotdistmetric->Eval(c0,c1);
        std::vector<dReal> door0(1), door1(1);
        door0[0] = c0.back();
        door1[0] = c1.back();
        dReal d2 = _doordistmetric->Eval(door0,door1);
        return RaveSqrt(d1*d1+d2*d2);
    }

    bool Sample(std::vector<dReal>&v) {
        for(int i = 0; i < 50; ++i) {
            _robotsamplefn->Sample(v);
            vector<dReal> vdoor;
            _doorsamplefn->Sample(vdoor);

            v.resize(GetDOF());
            v.back() = vdoor.at(0);
            if( _SetState(v) ) {
                return true;
            }
        }
        return false;
    }

    bool _SetState(std::vector<dReal>& v) {
        vector<dReal> vdoor(1); vdoor[0] = v.back();
        _ptarget->SetActiveDOFValues(vdoor);

        _probot->SetActiveDOFValues(v);
        vector<dReal> vsolution;
        bool bsuccess = _pmanip->FindIKSolution(_pdoorlink->GetTransform() * _tgrasp, vsolution, IKFO_CheckEnvCollisions);
        if( bsuccess ) {
            _probot->SetActiveDOFValues(vsolution);
            std::copy(vsolution.begin(),vsolution.end(),v.begin());
        }
        return bsuccess;
    }

    void SetState(const std::vector<dReal>& v)
    {
        vector<dReal> vtemp = v;
        if( !_SetState(vtemp) ) {
            RAVELOG_WARN("could not set state\n");
        }
    }

    void GetState(std::vector<dReal>& v)
    {
        _probot->GetActiveDOFValues(v);
        v.resize(GetDOF());
        v.back() = _pdoorjoint->GetValue(0);
    }

    void DiffState(std::vector<dReal>& v1,const std::vector<dReal>& v2)
    {
        _probot->SubtractActiveDOFValues(v1,v2);
        v1.back() -= v2.back();
    }

    bool NeightState(std::vector<dReal>& v,const std::vector<dReal>& vdelta, int fromgoal)
    {
        for(int i = 0; i < GetDOF(); ++i) {
            v.at(i) += vdelta.at(i);
        }
        return _SetState(v);
    }

    void SetPlannerParameters(PlannerBase::PlannerParametersPtr params)
    {
        //RobotBase::RobotStateSaver saver1(_probot);
        //RobotBase::RobotStateSaver saver2(_ptarget);
        _probot->SetActiveDOFs(_pmanip->GetArmIndices());
        vector<int> v(1); v[0] = _pdoorjoint->GetDOFIndex();
        _ptarget->SetActiveDOFs(v);

        params->_configurationspecification = _probot->GetActiveConfigurationSpecification() + _ptarget->GetActiveConfigurationSpecification();
        _probot->GetActiveDOFLimits(params->_vConfigLowerLimit,params->_vConfigUpperLimit);
        _pdoorjoint->GetLimits(params->_vConfigLowerLimit,params->_vConfigUpperLimit,true);

        _probot->GetActiveDOFVelocityLimits(params->_vConfigVelocityLimit);
        params->_vConfigVelocityLimit.push_back(100);

        _probot->GetActiveDOFAccelerationLimits(params->_vConfigAccelerationLimit);
        params->_vConfigAccelerationLimit.push_back(100);

        _probot->GetActiveDOFResolutions(params->_vConfigResolution);
        params->_vConfigResolution.push_back(0.05);

        //robot->GetActiveDOFValues(params->vinitialconfig);

        _robotdistmetric.reset(new planningutils::SimpleDistanceMetric(_probot));
        _doordistmetric.reset(new planningutils::SimpleDistanceMetric(_ptarget));
        params->_distmetricfn = boost::bind(&DoorConfiguration::ComputeDistance,shared_from_this(),_1,_2);

        SpaceSamplerBasePtr pconfigsampler1 = RaveCreateSpaceSampler(_probot->GetEnv(),str(boost::format("robotconfiguration %s")%_probot->GetName()));
        _robotsamplefn.reset(new planningutils::SimpleNeighborhoodSampler(pconfigsampler1,boost::bind(&planningutils::SimpleDistanceMetric::Eval,_robotdistmetric,_1,_2)));
        SpaceSamplerBasePtr pconfigsampler2 = RaveCreateSpaceSampler(_probot->GetEnv(),str(boost::format("robotconfiguration %s")%_ptarget->GetName()));
        _doorsamplefn.reset(new planningutils::SimpleNeighborhoodSampler(pconfigsampler2,boost::bind(&planningutils::SimpleDistanceMetric::Eval,_doordistmetric,_1,_2)));
        params->_samplefn = boost::bind(&DoorConfiguration::Sample,shared_from_this(),_1);
        params->_sampleneighfn.clear(); // won't be using it

        params->_setstatefn = boost::bind(&DoorConfiguration::SetState,shared_from_this(),_1);
        params->_getstatefn = boost::bind(&DoorConfiguration::GetState,shared_from_this(),_1);
        params->_diffstatefn = boost::bind(&DoorConfiguration::DiffState,shared_from_this(),_1,_2);
        params->_neighstatefn = boost::bind(&DoorConfiguration::NeightState,shared_from_this(),_1,_2,_3);

        _collision.reset(new planningutils::LineCollisionConstraint());
        params->_checkpathconstraintsfn = boost::bind(&planningutils::LineCollisionConstraint::Check,_collision,params, _probot, _1, _2, _3, _4);
    }

    RobotBase::ManipulatorPtr _pmanip;
    KinBody::LinkPtr _pdoorlink;
    KinBody::JointPtr _pdoorjoint;
    Transform _tgrasp; ///< the grasp transform in the door link frame
    RobotBasePtr _probot, _ptarget;

    boost::shared_ptr<planningutils::SimpleDistanceMetric> _robotdistmetric, _doordistmetric;
    boost::shared_ptr<planningutils::SimpleNeighborhoodSampler> _robotsamplefn, _doorsamplefn;
    boost::shared_ptr<planningutils::LineCollisionConstraint> _collision;
};

typedef boost::shared_ptr<DoorConfiguration> DoorConfigurationPtr;

int main(int argc, char ** argv)
{
    string scenefilename = "data/wam_cabinet.env.xml";
    string viewername = "qtcoin";
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    RaveSetDebugLevel(Level_Debug);

    boost::thread thviewer(boost::bind(SetViewer,penv,viewername)); // create the viewer
    usleep(200000); // wait for the viewer to init
    penv->Load(scenefilename);
    usleep(100000); // wait for the viewer to init

    RobotBasePtr probot = penv->GetRobot("BarrettWAM");
    // find the longest manipulator chain to move
    RobotBase::ManipulatorPtr pmanip = probot->GetManipulators().at(0);
    for(size_t i = 1; i < probot->GetManipulators().size(); ++i) {
        if( pmanip->GetArmIndices().size() < probot->GetManipulators()[i]->GetArmIndices().size() ) {
            pmanip = probot->GetManipulators()[i];
        }
    }
    RAVELOG_INFO(str(boost::format("planning with manipulator %s\n")%pmanip->GetName()));

    RobotBasePtr target = penv->GetRobot("Cabinet");
    KinBody::JointPtr pdoorjoint = target->GetJoint("J_right");

    std::vector<dReal> vpreshape(4);
    vpreshape[0] = 2.3; vpreshape[1] = 2.3; vpreshape[2] = 0.8; vpreshape[3] = 0;

    // create the configuration space
    DoorConfigurationPtr doorconfig(new DoorConfiguration(pmanip,pdoorjoint));
    PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
    Transform trobotorig;
    {
        EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

        probot->SetActiveDOFs(pmanip->GetGripperIndices());
        probot->SetActiveDOFValues(vpreshape);

        doorconfig->SetPlannerParameters(params);
        params->_nMaxIterations = 4000; // max iterations before failure

        trobotorig = probot->GetTransform();
    }

    PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");
    TrajectoryBasePtr ptraj;

//    for(dReal fangle = 0; fangle <= 1; fangle += 0.1) {
//        vector<dReal> v(doorconfig->GetDOF());
//        {
//            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
//            params->_getstatefn(v);
//            v.back()=fangle;
//            params->_setstatefn(v);
//            params->_getstatefn(v);
//        }
//        int n;
//        cin >> n;
//    }

    while(1) {
        GraphHandlePtr pgraph;
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

            params->_getstatefn(params->vinitialconfig);
            params->_setstatefn(params->vinitialconfig);
            params->_getstatefn(params->vinitialconfig);

            params->vgoalconfig = params->vinitialconfig;
            params->vgoalconfig.back() = RaveRandomFloat()*PI/2; // in radians
            params->_setstatefn(params->vgoalconfig);
            params->_getstatefn(params->vgoalconfig);

//            // find a set of free joint values for the robot
//            {
//                RobotBase::RobotStateSaver saver(probot); // save the state
//                while(1) {
//                    for(size_t i = 0; i < vlower.size(); ++i) {
//                        params->vgoalconfig[i] = vlower[i] + (vupper[i]-vlower[i])*RaveRandomFloat();
//                    }
//                    probot->SetActiveDOFValues(params->vgoalconfig);
//                    if( !penv->CheckCollision(probot) && !probot->CheckSelfCollision() ) {
//                        break;
//                    }
//                }
//                // robot state is restored
//            }

            if( !planner->InitPlan(probot,params) ) {
                RAVELOG_WARN("plan failed to init\n");
                continue;
            }

            // create a new output trajectory
            ptraj = RaveCreateTrajectory(penv,"");
            if( !planner->PlanPath(ptraj) ) {
                RAVELOG_WARN("plan failed, trying again\n");
                continue;
            }

            // draw the end effector of the trajectory
            {
                RobotBase::RobotStateSaver saver(probot); // save the state of the robot since will be setting joint values
                vector<RaveVector<float> > vpoints;
                vector<dReal> vtrajdata;
                for(dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01) {
                    ptraj->Sample(vtrajdata,ftime,probot->GetActiveConfigurationSpecification());
                    probot->SetActiveDOFValues(vtrajdata);
                    vpoints.push_back(pmanip->GetEndEffectorTransform().trans);
                }
                pgraph = penv->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),1.0f);
            }

            // send the trajectory to the robot
            probot->GetController()->SetPath(ptraj);
            target->GetController()->SetPath(ptraj);
        }


        // wait for the robot to finish
        while(!probot->GetController()->IsDone()) {
            usleep(1000);
        }

    }

    thviewer.join(); // wait for the viewer thread to exit
    penv->Destroy(); // destroy
    return 0;
}
