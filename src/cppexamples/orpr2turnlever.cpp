/** \example orpr2turnlever.cpp
    \author Rosen Diankov

    Shows how to set a workspace trajectory for the hand and have a robot plan it.

    <b>Full Example Code:</b>
 */
#include <openrave-core.h>
#include <vector>
#include <sstream>
#include <boost/format.hpp>

#include "orexample.h"
#include <openrave/plannerparameters.h>
#include <openrave/planningutils.h>

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PR2TurnLevelExample : public OpenRAVEExample
{
public:

    PlannerAction PlanCallback(const PlannerBase::PlannerProgress& progress)
    {
        // plan callback
        return PA_None;
    }

    virtual void WaitRobot(RobotBasePtr probot) {
        // unlock the environment and wait for the robot to finish
        while(!probot->GetController()->IsDone() && IsOk()) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
        }
    }

    virtual void demothread(int argc, char ** argv) {
        string scenefilename = "data/pr2test1.env.xml";
        RaveSetDebugLevel(Level_Debug);
        penv->Load(scenefilename);
        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        RobotBasePtr probot = vrobots.at(0);

        KinBodyPtr target = penv->GetKinBody("lever");
        if( !target ) {
            target = RaveCreateKinBody(penv,"");
            std::vector<AABB> boxes(1);
            boxes[0].pos = Vector(0,0.1,0);
            boxes[0].extents = Vector(0.01,0.1,0.01);
            target->InitFromBoxes(boxes,true);
            target->SetName("lever");
            penv->Add(target);
            Transform t;
            t.trans = Vector(-0.2,-0.2,1);
            target->SetTransform(t);
        }

        RobotBase::ManipulatorPtr pmanip = probot->SetActiveManipulator("rightarm");

        // load inverse kinematics using ikfast
        ModuleBasePtr pikfast = RaveCreateModule(penv,"ikfast");
        penv->Add(pikfast,true,"");
        stringstream ssin,ssout;
        vector<dReal> vsolution;
        ssin << "LoadIKFastSolver " << probot->GetName() << " " << (int)IKP_Transform6D;
        if( !pikfast->SendCommand(ssout,ssin) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("failed to load iksolver", ORE_Assert);
        }

        // create the planner parameters
        PlannerBasePtr planner = RaveCreatePlanner(penv,"birrt");

        ModuleBasePtr basemodule = RaveCreateModule(penv,"BaseManipulation");
        penv->Add(basemodule,true,probot->GetName());
        ModuleBasePtr taskmodule = RaveCreateModule(penv,"TaskManipulation");
        penv->Add(taskmodule,true,probot->GetName());

        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            stringstream ssin, ssout; ssin << "ReleaseFingers";
            taskmodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);

        TrajectoryBasePtr workspacetraj;
        Transform Tgrasp0;
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            Transform Toffset;
            Toffset.trans = Vector(0,0.2,0);
            Transform Ttarget0 = target->GetTransform();
            Transform Ttarget1 = Ttarget0 * matrixFromAxisAngle(Vector(PI/2,0,0));

            Tgrasp0 = matrixFromAxisAngle(Vector(PI/2,0,0))*matrixFromAxisAngle(Vector(0,PI/2,0));
            Tgrasp0.trans = Ttarget0*Toffset.trans;
            Transform Tgraspoffset = Ttarget0.inverse() * Tgrasp0;
            Transform Tgrasp1 = Ttarget1 * Tgraspoffset;

            ConfigurationSpecification spec = IkParameterization::GetConfigurationSpecification(IKP_Transform6D,"linear");
            workspacetraj = RaveCreateTrajectory(penv,"");
            vector<dReal> values;
            workspacetraj->Init(spec);
            for(size_t i = 0; i < 32; ++i) {
                dReal angle = i*0.05;
                Transform Ttarget = Ttarget0 * matrixFromAxisAngle(Vector(angle,0,0));
                Transform Tgrasp = Ttarget*Tgraspoffset;
                IkParameterization ikparam(Tgrasp,IKP_Transform6D);
                values.resize(ikparam.GetNumberOfValues());
                ikparam.GetValues(values.begin());
                workspacetraj->Insert(workspacetraj->GetNumWaypoints(),values);
            }

            std::vector<dReal> maxvelocities(7,1.0);
            std::vector<dReal> maxaccelerations(7,5.0);
            planningutils::RetimeAffineTrajectory(workspacetraj,maxvelocities,maxaccelerations);
            RAVELOG_INFO(str(boost::format("duration=%f, points=%d")%workspacetraj->GetDuration()%workspacetraj->GetNumWaypoints()));
        }

        {
            stringstream ssout, ssin; ssin << "MoveToHandPosition poses 1 " << Tgrasp0;
            basemodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);
        {
            stringstream ssin, ssout; ssin << "CloseFingers";
            taskmodule->SendCommand(ssout,ssin);
        }
        WaitRobot(probot);

        list<TrajectoryBasePtr> listtrajectories;
        {
            EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment
            probot->SetActiveDOFs(pmanip->GetArmIndices());
            probot->Grab(target);
            PlannerBasePtr planner = RaveCreatePlanner(penv,"workspacetrajectorytracker");
            WorkspaceTrajectoryParametersPtr params(new WorkspaceTrajectoryParameters(penv));
            params->SetRobotActiveJoints(probot); // set planning configuration space to current active dofs
            params->workspacetraj = workspacetraj;

            RAVELOG_INFO("starting to plan\n");
            if( !planner->InitPlan(probot,params) ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("plan init failed",ORE_Assert);
            }

            // create a new output trajectory
            TrajectoryBasePtr outputtraj = RaveCreateTrajectory(penv,"");
            if( !planner->PlanPath(outputtraj) ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("plan failed",ORE_Assert);
            }
            listtrajectories.push_back(outputtraj);
            listtrajectories.push_back(planningutils::ReverseTrajectory(outputtraj));
        }

        while(IsOk()) {
            for(list<TrajectoryBasePtr>::iterator it = listtrajectories.begin(); it != listtrajectories.end(); ++it) {
                probot->GetController()->SetPath(*it);
                WaitRobot(probot);
            }
        }
    }
};

} // end namespace cppexamples

int main(int argc, char ** argv)
{
    cppexamples::PR2TurnLevelExample example;
    return example.main(argc,argv);
}

