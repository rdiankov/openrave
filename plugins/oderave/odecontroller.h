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
#ifndef OPENRAVE_ODE_CONTROLLER
#define OPENRAVE_ODE_CONTROLLER

#include "plugindefs.h"

class ODEVelocityController : public ControllerBase
{
public:
    ODEVelocityController(EnvironmentBasePtr penv) : ControllerBase(penv)
    {
        __description = ":Interface Authors: Juan Gonzalez and Rosen Diankov\n\nODE Velocity controller.";
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        _dofindices = dofindices;
        if( nControlTransformation ) {
            RAVELOG_WARN("odevelocity controller cannot control transformation\n");
        }
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
                FOREACHC(it, _dofindices) {
                    KinBody::JointConstPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                    dJointID jointid = pinfo->vjoints.at(pjoint->GetJointIndex());
                    int index = *it-pjoint->GetJointIndex();
                    BOOST_ASSERT(index>=0);
                    odespace->_jointset[dJointGetType(jointid)](jointid,dParamFMax+dParamGroup*index, 0);
                }
            }
        }
        _bVelocityMode = false;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return 0;
    }

    virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans) {
        Reset(0);
        return false;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) {
        Reset(0);
        return false;
    }
    virtual void SimulationStep(OpenRAVE::dReal fTimeElapsed) {
    }
    virtual bool IsDone() {
        return !_bVelocityMode;
    }
    virtual OpenRAVE::dReal GetTime() const {
        return 0;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

    virtual ODESpace::KinBodyInfoPtr GetODESpace() {
        return boost::dynamic_pointer_cast<ODESpace::KinBodyInfo>(_probot->GetPhysicsData());
    }

    bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

        if( cmd == "setvelocity" ) {
            vector<OpenRAVE::dReal> velocities(_dofindices.size());
            for(size_t i = 0; i < velocities.size(); ++i) {
                is >> velocities[i];
                if( !is ) {
                    RAVELOG_WARN("setvelocity bad command\n");
                    return false;
                }
            }

            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
            ODESpace::KinBodyInfoPtr pinfo = GetODESpace();
            if( !pinfo ) {
                RAVELOG_WARN("need to set ode physics engine\n");
                return false;
            }
            RobotBase::RobotStateSaver robotsaver(_probot,KinBody::Save_LinkVelocities);
            boost::shared_ptr<ODESpace> odespace(pinfo->_odespace);
            int dofindex = 0;
            std::vector<OpenRAVE::dReal> valldofvelocities;
            _probot->GetDOFVelocities(valldofvelocities);
            FOREACHC(it, _dofindices) {
                KinBody::JointConstPtr pjoint = _probot->GetJointFromDOFIndex(*it);
                dJointID jointid = pinfo->vjoints.at(pjoint->GetJointIndex());
                int iaxis = *it-pjoint->GetDOFIndex();
                BOOST_ASSERT(iaxis >= 0);
                valldofvelocities.at(*it) = velocities.at(dofindex);
                odespace->_jointset[dJointGetType(jointid)](jointid,dParamFMax+dParamGroup*iaxis, pjoint->GetMaxTorque(iaxis));
                odespace->_jointset[dJointGetType(jointid)](jointid,dParamVel+dParamGroup*iaxis, velocities.at(dofindex++));
            }
            _probot->SetDOFVelocities(valldofvelocities);

            // go through all passive joints and set their velocity if they are dependent on the current controlled indices
            std::vector<int> vmimicdofs;
            std::vector<OpenRAVE::dReal> values;
            size_t ipassiveindex = _probot->GetJoints().size();
            FOREACHC(itjoint, _probot->GetPassiveJoints()) {
                values.resize(0);
                for(int iaxis = 0; iaxis < (*itjoint)->GetDOF(); ++iaxis) {
                    if( (*itjoint)->IsMimic(iaxis) ) {
                        bool bset = false;
                        (*itjoint)->GetMimicDOFIndices(vmimicdofs,iaxis);
                        FOREACH(itdof, vmimicdofs) {
                            if( find(_dofindices.begin(),_dofindices.end(),*itdof) != _dofindices.end() ) {
                                bset = true;
                            }
                        }
                        if( bset ) {
                            if( values.size() == 0 ) {
                                (*itjoint)->GetVelocities(values);
                            }
                            dJointID jointid = pinfo->vjoints.at(ipassiveindex);
                            odespace->_jointset[dJointGetType(jointid)](jointid,dParamFMax+dParamGroup*iaxis, (*itjoint)->GetMaxTorque(iaxis));
                            odespace->_jointset[dJointGetType(jointid)](jointid,dParamVel+dParamGroup*iaxis, values.at(iaxis));
                        }
                    }
                }
                ipassiveindex++;
            }
            _bVelocityMode = true;
            return true;
        }

        throw openrave_exception(str(boost::format("command %s supported")%cmd),OpenRAVE::ORE_CommandNotSupported);
        return false;
    }

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    bool _bVelocityMode;
};

#endif
