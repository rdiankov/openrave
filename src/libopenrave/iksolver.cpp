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

bool IkReturn::Append(const IkReturn& r)
{
    bool bclashing = false;
    if( !!r._userdata ) {
        if( !!_userdata ) {
            RAVELOG_WARN("IkReturn already has _userdata set, but overwriting anyway\n");
            bclashing = true;
        }
        _userdata = r._userdata;
    }
    if( _mapdata.size() == 0 ) {
        _mapdata = r._mapdata;
    }
    else {
        FOREACHC(itr,r._mapdata) {
            if( !_mapdata.insert(*itr).second ) {
                RAVELOG_WARN(str(boost::format("IkReturn _mapdata %s overwritten")%itr->first));
                bclashing = true;
            }
        }
    }
    if( r._vsolution.size() > 0 ) {
        if( _vsolution.size() > 0 ) {
            RAVELOG_WARN("IkReturn already has _vsolution set, but overwriting anyway\n");
            bclashing = true;
        }
        _vsolution = r._vsolution;
    }
    return bclashing;
}

void IkReturn::Clear()
{
    _mapdata.clear();
    _userdata.reset();
    _vsolution.resize(0);
}

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

bool IkSolverBase::Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, IkReturnPtr ikreturn)
{
    if( !ikreturn ) {
        return Solve(param,q0,filteroptions,boost::shared_ptr< vector<dReal> >());
    }
    ikreturn->Clear();
    boost::shared_ptr< vector<dReal> > psolution(&ikreturn->_vsolution, utils::null_deleter());
    if( !Solve(param,q0,filteroptions,psolution) ) {
        ikreturn->_action = IKRA_Reject;
        return false;
    }
    ikreturn->_action = IKRA_Success;
    return true;
}

bool IkSolverBase::SolveAll(const IkParameterization& param, int filteroptions, std::vector<IkReturnPtr>& ikreturns)
{
    ikreturns.resize(0);
    std::vector< std::vector<dReal> > vsolutions;
    if( !SolveAll(param,filteroptions,vsolutions) ) {
        return false;
    }
    ikreturns.resize(vsolutions.size());
    for(size_t i = 0; i < ikreturns.size(); ++i) {
        ikreturns[i].reset(new IkReturn(IKRA_Success));
        ikreturns[i]->_vsolution = vsolutions[i];
        ikreturns[i]->_action = IKRA_Success;
    }
    return vsolutions.size() > 0;
}

bool IkSolverBase::Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn)
{
    if( !ikreturn ) {
        return Solve(param,q0,vFreeParameters,filteroptions,boost::shared_ptr< vector<dReal> >());
    }
    ikreturn->Clear();
    boost::shared_ptr< vector<dReal> > psolution(&ikreturn->_vsolution, utils::null_deleter());
    if( !Solve(param,q0,vFreeParameters,filteroptions,psolution) ) {
        ikreturn->_action = IKRA_Reject;
        return false;
    }
    ikreturn->_action = IKRA_Success;
    return true;
}

bool IkSolverBase::SolveAll(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& ikreturns)
{
    ikreturns.resize(0);
    std::vector< std::vector<dReal> > vsolutions;
    if( !SolveAll(param,vFreeParameters,filteroptions,vsolutions) ) {
        return false;
    }
    ikreturns.resize(vsolutions.size());
    for(size_t i = 0; i < ikreturns.size(); ++i) {
        ikreturns[i].reset(new IkReturn(IKRA_Success));
        ikreturns[i]->_vsolution = vsolutions[i];
        ikreturns[i]->_action = IKRA_Success;
    }
    return vsolutions.size() > 0;
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

IkReturnAction IkSolverBase::_CallFilters(std::vector<dReal>& solution, RobotBase::ManipulatorPtr manipulator, const IkParameterization& param, IkReturnPtr filterreturn)
{
    vector<dReal> vtestsolution,vtestsolution2;
    if( IS_DEBUGLEVEL(Level_Debug) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
        RobotBasePtr robot = manipulator->GetRobot();
        robot->GetConfigurationValues(vtestsolution);
        for(size_t i = 0; i < manipulator->GetArmIndices().size(); ++i) {
            int dofindex = manipulator->GetArmIndices()[i];
            dReal fdiff = 0;
            KinBody::JointPtr pjoint = robot->GetJointFromDOFIndex(dofindex);
            fdiff = pjoint->SubtractValue(vtestsolution.at(dofindex), solution.at(i), dofindex-pjoint->GetDOFIndex());
            if( fdiff > g_fEpsilonJointLimit ) {
                throw OPENRAVE_EXCEPTION_FORMAT("_CallFilters on robot %s manip %s need to start with robot configuration set to the solution. manip dof %d (%f != %f)",manipulator->GetRobot()->GetName()%manipulator->GetName()%dofindex%vtestsolution.at(dofindex)%solution.at(i), ORE_InconsistentConstraints);
            }
        }
    }

    FOREACHC(it,__listRegisteredFilters) {
        CustomIkSolverFilterDataPtr pitdata = boost::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata) {
            IkReturn ret = pitdata->_filterfn(solution,manipulator,param);
            if( ret != IKRA_Success ) {
                return ret._action; // just return the action
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
            if( !!filterreturn ) {
                filterreturn->Append(ret);
            }
        }
    }
    if( !!filterreturn ) {
        filterreturn->_action = IKRA_Success;
    }
    return IKRA_Success;
}

}
