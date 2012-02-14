// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com)
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
#include "libopenrave.h"

namespace OpenRAVE {

class CustomIkSolverFilterData : public boost::enable_shared_from_this<CustomIkSolverFilterData>, public UserData
{
public:
    CustomIkSolverFilterData(int priority, const IkSolverBase::IkFilterCallbackFn& filterfn, IkSolverBasePtr iksolver) : _priority(priority), _filterfn(filterfn), _iksolverweak(iksolver) {
    }
    virtual ~CustomIkSolverFilterData() {
        IkSolverBasePtr iksolver = _iksolverweak.lock();
        if( !!iksolver ) {
            iksolver->__listRegisteredFilters.erase(_iterator);
        }
    }

    int _priority;
    IkSolverBase::IkFilterCallbackFn _filterfn;
    IkSolverBaseWeakPtr _iksolverweak;
    std::list<UserDataWeakPtr>::iterator _iterator;
};

typedef boost::shared_ptr<CustomIkSolverFilterData> CustomIkSolverFilterDataPtr;

bool CustomIkSolverFilterDataCompare(UserDataPtr data0, UserDataPtr data1)
{
    return boost::dynamic_pointer_cast<CustomIkSolverFilterData>(data0)->_priority > boost::dynamic_pointer_cast<CustomIkSolverFilterData>(data1)->_priority;
}

UserDataPtr IkSolverBase::RegisterCustomFilter(int priority, const IkSolverBase::IkFilterCallbackFn &filterfn)
{
    CustomIkSolverFilterDataPtr pdata(new CustomIkSolverFilterData(priority,filterfn,shared_iksolver()));
    std::list<UserDataWeakPtr>::iterator it;
    FORIT(it, __listRegisteredFilters) {
        CustomIkSolverFilterDataPtr pitdata = boost::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata && pdata->_priority > pitdata->_priority ) {
            break;
        }
    }
    pdata->_iterator = __listRegisteredFilters.insert(it,pdata);
    return pdata;
}

IkFilterReturn IkSolverBase::_CallFilters(std::vector<dReal>& solution, RobotBase::ManipulatorPtr manipulator, const IkParameterization& param)
{
    vector<dReal> vtestsolution,vtestsolution2;
    if( IS_DEBUGLEVEL(Level_Debug) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        RobotBasePtr robot = manipulator->GetRobot();
        robot->GetConfigurationValues(vtestsolution);
        for(size_t i = 0; i < manipulator->GetArmIndices().size(); ++i) {
            int dofindex = manipulator->GetArmIndices()[i];
            dReal fdiff = 0;
            KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(dofindex);
            if( pjoint->GetDOF() == 1 ) {
                // use subtract values
                vector<dReal> v1(1), v2(1); v1[0] = vtestsolution.at(dofindex); v2[0] = solution.at(i);
                pjoint->SubtractValues(v1,v2);
                fdiff = RaveFabs(v1.at(0));
            }
            else {
                fdiff = RaveFabs(vtestsolution.at(dofindex) - solution.at(i));
            }
            if( fdiff > g_fEpsilonJointLimit ) {
                throw OPENRAVE_EXCEPTION_FORMAT("_CallFilters on robot %s manip %s need to start with robot configuration set to the solution. manip dof %d (%f != %f)",manipulator->GetRobot()->GetName()%manipulator->GetName()%dofindex%vtestsolution.at(dofindex)%solution.at(i), ORE_InconsistentConstraints);
            }
        }
    }

    FOREACHC(it,__listRegisteredFilters) {
        CustomIkSolverFilterDataPtr pitdata = boost::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata) {
            IkFilterReturn ret = pitdata->_filterfn(solution,manipulator,param);
            if( ret != IKFR_Success ) {
                return ret;
            }
            if( vtestsolution.size() > 0 ) {
                // check that the robot is set to solution
                vector<dReal> vtestsolution2;
                manipulator->GetRobot()->GetConfigurationValues(vtestsolution2);
                for(size_t i = 0; i < manipulator->GetArmIndices().size(); ++i) {
                    vtestsolution.at(manipulator->GetArmIndices()[i]) = solution.at(i);
                }
                for(size_t i = 0; i < vtestsolution.size(); ++i) {
                    if( RaveFabs(vtestsolution.at(i)-vtestsolution2.at(i)) > g_fEpsilonJointLimit ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("one of the filters set on robot %s manip %s did not restore the robot configuraiton. config dof %d (%f -> %f)",manipulator->GetRobot()->GetName()%manipulator->GetName()%i%vtestsolution.at(i)%vtestsolution2.at(i), ORE_InconsistentConstraints);
                    }
                }
            }
        }
    }
    return IKFR_Success;
}

}
