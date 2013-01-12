// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "ravep.h"

namespace OpenRAVE {

class GenericRobot : public RobotBase
{
public:
    GenericRobot(EnvironmentBasePtr penv, std::istream& sinput) : RobotBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nSimplest robot possible that just passes the trajectories to the controller";
    }
    virtual ~GenericRobot() {
    }

    virtual bool SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
    {
        _pController = controller;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),jointindices,nControlTransformation) ) {
                RAVELOG_WARN(str(boost::format("GenericRobot %s: Failed to init controller %s\n")%GetName()%controller->GetXMLId()));
                _pController.reset();
                return false;
            }
        }
        return true;
    }

    virtual ControllerBasePtr GetController() const {
        return _pController;
    }

    virtual void SimulationStep(dReal fElapsedTime)
    {
        RobotBase::SimulationStep(fElapsedTime);
        if( !!_pController ) {
            try {
                _pController->SimulationStep(fElapsedTime);
            }
            catch(const std::exception& ex) {
                RAVELOG_ERROR(str(boost::format("robot %s controller %s failed with exception, so resetting: %s")%GetName()%_pController->GetXMLId()%ex.what()));
                _pController->Reset(0);
            }
        }
    }

protected:
    ControllerBasePtr _pController;
};

RobotBasePtr CreateGenericRobot(EnvironmentBasePtr penv, std::istream& sinput)
{
    return RobotBasePtr(new GenericRobot(penv,sinput));
}

}
