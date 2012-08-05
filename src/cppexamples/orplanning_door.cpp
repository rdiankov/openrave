/** \example orplanning_door.cpp
    \author Rosen Diankov

    \image html cppexample_orplanning_door.jpg "Robot opening door for different initial/goal configurations."
    \image latex cppexample_orplanning_door.jpg "Robot opening door for different initial/goal configurations." width=10cm

    Shows how to use a planner two plan for a robot opening a door.
    The configuration space consists of the robot's manipulator joints and the door joint.
    A generic planner is just passed in this configuration space inside its planner parameters.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/format.hpp>

#include <openrave/planningutils.h>

#include "orexample.h"

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

/// \brief builds up the configuration space of a robot and a door
class DoorConfiguration : public boost::enable_shared_from_this<DoorConfiguration>
{
    static dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
    {
        //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
        dReal facos = min((t1.rot-t2.rot).lengthsqr4(),(t1.rot+t2.rot).lengthsqr4());
        return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos; //*facos;
    }


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

    bool _SetState(std::vector<dReal>& v, int filteroptions=IKFO_CheckEnvCollisions|IKFO_IgnoreCustomFilters) {
        // save state before modifying it
        RobotBase::RobotStateSaverPtr savestate1(new RobotBase::RobotStateSaver(_probot));
        RobotBase::RobotStateSaverPtr savestate2(new RobotBase::RobotStateSaver(_ptarget));

        vector<dReal> vdoor(1); vdoor[0] = v.back();
        _ptarget->SetActiveDOFValues(vdoor);
        _probot->SetActiveDOFValues(v);

        vector<dReal> vsolution;
        bool bsuccess = _pmanip->FindIKSolution(_pdoorlink->GetTransform() * _tgrasp, vsolution, filteroptions);
        if( bsuccess ) {
            savestate1.reset();
            savestate2.reset();
            _probot->SetActiveDOFValues(vsolution);
            std::copy(vsolution.begin(),vsolution.end(),v.begin());
            _ptarget->SetActiveDOFValues(vdoor);
        }
        return bsuccess;
    }

    void SetState(const std::vector<dReal>& v)
    {
        vector<dReal> vtemp = v;
        if( !_SetState(vtemp) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("could not set state",ORE_InvalidArguments);
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
        _vprevsolution = v;
        // the previous solution should already be set on the robot, so do a sanity check
        _tmanipprev = _pmanip->GetTransform();
        Transform tdoorprev = _pdoorlink->GetTransform();
        BOOST_ASSERT( TransformDistance2(tdoorprev*_tgrasp,_tmanipprev) <= g_fEpsilon );

        {
            KinBody::KinBodyStateSaver statesaver(_ptarget);
            vector<dReal> vdoor(1);
            vdoor[0] = v.back()+0.5*vdelta.back();
            _ptarget->SetActiveDOFValues(vdoor);
            _tmanipmidreal = _pdoorlink->GetTransform()*_tgrasp;
        }

        for(int i = 0; i < GetDOF(); ++i) {
            v.at(i) += vdelta.at(i);
        }

        return _SetState(v,IKFO_CheckEnvCollisions);
    }

    // due to discontinues check that the robot midpoint is also along the door's expected trajectory
    // take the midpoint of the solutions and ikparameterization and see if they are close
    IkReturn _CheckContinuityFilter(std::vector<dReal>& vsolution, RobotBase::ManipulatorConstPtr pmanip, const IkParameterization& ikp)
    {
        Transform tmanipnew = ikp.GetTransform6D();
        std::vector<dReal> vmidsolution(_probot->GetActiveDOF());
        dReal realdist2 = TransformDistance2(_tmanipprev, tmanipnew);
        const dReal ikmidpointmaxdist2mult = 0.5;

        RobotBase::RobotStateSaver savestate(_probot);
        for(int i = 0; i < _probot->GetActiveDOF(); ++i) {
            vmidsolution.at(i) = 0.5*(_vprevsolution.at(i)+vsolution.at(i));
        }
        _probot->SetActiveDOFValues(vmidsolution);

        Transform tmanipmid = _pmanip->GetTransform();
        dReal middist2 = TransformDistance2(tmanipmid, _tmanipmidreal);
        if( middist2 > g_fEpsilon && middist2 > ikmidpointmaxdist2mult*realdist2 ) {
            RAVELOG_VERBOSE(str(boost::format("rejected due to discontinuity at mid-point %e > %e")%middist2%(ikmidpointmaxdist2mult*realdist2)));
            return IKRA_Reject;
        }
        return IKRA_Success;
    }

    void SetPlannerParameters(PlannerBase::PlannerParametersPtr params)
    {
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

        std::list<KinBodyPtr> listCheckCollisions;
        listCheckCollisions.push_back(_probot);
        _collision.reset(new planningutils::LineCollisionConstraint(listCheckCollisions));
        params->_checkpathconstraintsfn = boost::bind(&planningutils::LineCollisionConstraint::Check,_collision,params, _probot, _1, _2, _3, _4);

        _ikfilter = _pmanip->GetIkSolver()->RegisterCustomFilter(0, boost::bind(&DoorConfiguration::_CheckContinuityFilter, shared_from_this(), _1, _2, _3));
    }

    RobotBase::ManipulatorPtr _pmanip;
    KinBody::LinkPtr _pdoorlink;
    KinBody::JointPtr _pdoorjoint;
    Transform _tgrasp; ///< the grasp transform in the door link frame
    RobotBasePtr _probot, _ptarget;
    vector<dReal> _vprevsolution;
    Transform _tmanipprev, _tmanipmidreal;
    UserDataPtr _ikfilter;

    boost::shared_ptr<planningutils::SimpleDistanceMetric> _robotdistmetric, _doordistmetric;
    boost::shared_ptr<planningutils::SimpleNeighborhoodSampler> _robotsamplefn, _doorsamplefn;
    boost::shared_ptr<planningutils::LineCollisionConstraint> _collision;
};

typedef boost::shared_ptr<DoorConfiguration> DoorConfigurationPtr;

class PlanningDoorExample : public OpenRAVEExample
{
public:
    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "data/wam_cabinet.env.xml";
        RaveSetDebugLevel(Level_Debug);
        penv->Load(scenefilename);

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
            params->_nMaxIterations = 150; // max iterations before failure

            trobotorig = probot->GetTransform();
        }

        PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");
        TrajectoryBasePtr ptraj;
        int iter = 0;

        while(IsOk()) {
            iter += 1;
            GraphHandlePtr pgraph;
            {
                EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

                if( (iter%5) == 0 ) {
                    RAVELOG_INFO("find a new position for the robot\n");
                    for(int i = 0; i < 100; ++i) {
                        Transform tnew = trobotorig;
                        tnew.trans.x += 0.5*(RaveRandomFloat()-0.5);
                        tnew.trans.y += 0.5*(RaveRandomFloat()-0.5);
                        probot->SetTransform(tnew);

                        try {
                            params->_getstatefn(params->vinitialconfig);
                            params->_setstatefn(params->vinitialconfig);
                            params->_getstatefn(params->vinitialconfig);

                            params->vgoalconfig = params->vinitialconfig;
                            params->vgoalconfig.back() = RaveRandomFloat()*1.5; // in radians
                            params->_setstatefn(params->vgoalconfig);
                            params->_getstatefn(params->vgoalconfig);
                            break;
                        }
                        catch(const openrave_exception& ex) {
                            probot->SetTransform(trobotorig);
                        }
                    }
                }
                else {
                    params->_getstatefn(params->vinitialconfig);
                    params->_setstatefn(params->vinitialconfig);
                    params->_getstatefn(params->vinitialconfig);

                    params->vgoalconfig = params->vinitialconfig;
                    params->vgoalconfig.back() = RaveRandomFloat()*1.5; // in radians
                    params->_setstatefn(params->vgoalconfig);
                    params->_getstatefn(params->vgoalconfig);
                }

                //params->_sPostProcessingPlanner = "lineartrajectoryretimer";
                ptraj = RaveCreateTrajectory(penv,"");
                if( !planner->InitPlan(probot,params) ) {
                    RAVELOG_WARN("plan failed to init\n");
                    continue;
                }

                // create a new output trajectory
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
            while(!probot->GetController()->IsDone() && IsOk()) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            }
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PlanningDoorExample example;
    return example.main(argc,argv);
}
