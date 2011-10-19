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
    FOREACHC(it,__listRegisteredFilters) {
        CustomIkSolverFilterDataPtr pitdata = boost::dynamic_pointer_cast<CustomIkSolverFilterData>(it->lock());
        if( !!pitdata) {
            IkFilterReturn ret = pitdata->_filterfn(solution,manipulator,param);
            if( ret != IKFR_Success ) {
                return ret;
            }
        }
    }
    return IKFR_Success;
}

}
