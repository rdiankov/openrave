// -*- coding: utf-8 -*-
// Copyright (C) 2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "plugindefs.h"

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>

class IdealVelocityController : public ControllerBase
{
public:
    IdealVelocityController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv)
    {
        __description = ":Interface Authors: Rosen Diankov\n\nIdeal Velocity controller.";
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
//        if( !!_probot ) {
//            EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
//            //_probot->GetDOFVelocities(_vPreviousVelocities,_dofindices);
//        }
        _bVelocityMode = false;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return 0;
    }

    virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans) {
        OPENRAVE_ASSERT_OP(values.size(),==,_dofindices.size());
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        _vDesiredVelocities = values;
        std::vector<dReal> vallvelocities;
        _probot->GetDOFVelocities(vallvelocities);
        for(size_t i = 0; i < _dofindices.size(); ++i) {
            vallvelocities.at(_dofindices[i]) = _vDesiredVelocities.at(i);
        }
        _probot->SetDOFVelocities(vallvelocities);
        _bVelocityMode = true;
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj) {
        return false;
    }
    virtual void SimulationStep(OpenRAVE::dReal fTimeElapsed) {
        if( _bVelocityMode ) {
            std::vector<dReal> vallvelocities;
            _probot->GetDOFVelocities(vallvelocities);
            for(size_t i = 0; i < _dofindices.size(); ++i) {
                vallvelocities.at(_dofindices[i]) = _vDesiredVelocities.at(i);
            }

            vector<dReal> vprevvalues;
            _probot->GetDOFValues(vprevvalues,_dofindices);
            for(size_t i = 0; i < _dofindices.size(); ++i) {
                vprevvalues[i] += fTimeElapsed*_vDesiredVelocities[i];
            }
            _probot->SetDOFValues(vprevvalues,true,_dofindices);
            _probot->SetDOFVelocities(vallvelocities); // set after SetDOFValues in order to get correct link velocities
        }
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

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices;
    std::vector<dReal> _vDesiredVelocities; //, _vPreviousVelocities;
    bool _bVelocityMode;
    OpenRAVE::UserDataPtr _torquechangedhandle;
};

ControllerBasePtr CreateIdealVelocityController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new IdealVelocityController(penv,sinput));
}
