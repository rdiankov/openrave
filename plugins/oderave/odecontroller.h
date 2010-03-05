// Copyright (c) 2008-2010 Rosen Diankov (rosen.diankov@gmail.com), Juan Gonzalez
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// A simple openrave plugin that registers a custom XML reader. This allows users
/// to extend the XML files to add their own tags. 
/// Once the plugin is compiled, cd to where the plugin was installed and open customreader.env.xml with openrave
#ifndef OPENRAVE_ODE_CONTROLLER
#define OPENRAVE_ODE_CONTROLLER

#include "plugindefs.h"

class ODEVelocityController : public ControllerBase
{
public:
 ODEVelocityController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = "ODE Velocity controller by Juan Gonzalez and Rosen Diankov";
    }

    virtual bool Init(RobotBasePtr robot, const std::string& args)
    {
        _probot = robot;
        Reset(0);
        return true;
    }

    virtual void Reset(int options)
    {
        if( !!_probot ) {
            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            ODESpace::KinBodyInfoPtr pinfo = GetODESpace();
            if( !!pinfo ) {
                boost::shared_ptr<ODESpace> odespace(pinfo->_odespace);
                FOREACHC(itjoint,_probot->GetJoints()) {
                    dJointID jointid = pinfo->vjoints.at((*itjoint)->GetJointIndex());
                    for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                        odespace->_jointset[dJointGetType(jointid)](jointid,dParamFMax+dParamGroup*i, 0);
                    }
                }
            }
        }
        _bVelocityMode = false;
    }

    virtual bool SetDesired(const std::vector<dReal>& values) {
        Reset(0);
        return false;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) {
        Reset(0);
        return false;
    }
    virtual bool SimulationStep(dReal fTimeElapsed) { return false; }
    virtual bool IsDone() { return !_bVelocityMode; }
    virtual dReal GetTime() const { return 0; }
    virtual RobotBasePtr GetRobot() const { return _probot; }

    virtual ODESpace::KinBodyInfoPtr GetODESpace() {
        if( GetEnv()->GetPhysicsEngine()->GetXMLId() == "ode" )
            return boost::static_pointer_cast<ODESpace::KinBodyInfo>(_probot->GetPhysicsData());
        return ODESpace::KinBodyInfoPtr();
    }

    bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "setvelocity" ) {
            vector<dReal> velocities(_probot->GetDOF());
            for(size_t i = 0; i < velocities.size(); ++i) {
                is >> velocities[i];
                if( !is )
                    return false;
            }

            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            ODESpace::KinBodyInfoPtr pinfo = GetODESpace();
            if( !pinfo ) {
                RAVELOG_WARN("need to set ode physics engine\n");
                return false;
            }
            boost::shared_ptr<ODESpace> odespace(pinfo->_odespace);
            FOREACHC(itjoint,_probot->GetJoints()) {
                dJointID jointid = pinfo->vjoints.at((*itjoint)->GetJointIndex());
                for(int i = 0; i < (*itjoint)->GetDOF(); ++i) {
                    odespace->_jointset[dJointGetType(jointid)](jointid,dParamFMax+dParamGroup*i, (*itjoint)->GetMaxTorque());
                    odespace->_jointset[dJointGetType(jointid)](jointid,dParamVel+dParamGroup*i, velocities.at((*itjoint)->GetDOFIndex()+i));
                }
            }
            _bVelocityMode = true;
            return true;
        }

        throw openrave_exception(str(boost::format("command %s supported")%cmd),OpenRAVE::ORE_CommandNotSupported);
        return false;
    }
    
protected:
    RobotBasePtr _probot;
    bool _bVelocityMode;
};

#endif
