// Copyright (C) 2006-2013 Rosen Diankov <rosen.diankov@gmail.com>
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

class MultiController : public MultiControllerBase
{
public:
    MultiController(EnvironmentBasePtr penv) : MultiControllerBase(penv), _nControlTransformation(0) {
    }

    virtual ~MultiController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _probot=robot;
        if( !_probot ) {
            return false;
        }
        _listcontrollers.clear();
        _dofindices=dofindices;
        // reverse the mapping
        _dofreverseindices.resize(_probot->GetDOF());
        FOREACH(it,_dofreverseindices) {
            *it = -1;
        }
        int index = 0;
        FOREACH(it,_dofindices) {
            _dofreverseindices.at(*it) = index++;
        }
        _vcontrollersbydofs.resize(0); _vcontrollersbydofs.resize(_dofindices.size());
        _nControlTransformation = nControlTransformation;
        _ptransformcontroller.reset();
        return true;
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return _nControlTransformation;
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

    virtual bool AttachController(ControllerBasePtr controller, const std::vector<int>& dofindices, int nControlTransformation)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( nControlTransformation && !!_ptransformcontroller ) {
            throw openrave_exception(_("controller already attached for transformation"),ORE_InvalidArguments);
        }
        FOREACHC(it,dofindices) {
            if( !!_vcontrollersbydofs.at(*it) ) {
                throw openrave_exception(str(boost::format(_("controller already attached to dof %d"))%*it));
            }
        }
        if( !controller->Init(_probot,dofindices,nControlTransformation) ) {
            return false;
        }
        if( nControlTransformation ) {
            _ptransformcontroller = controller;
        }
        FOREACHC(it,dofindices) {
            _vcontrollersbydofs.at(*it) = controller;
        }
        _listcontrollers.push_back(controller);
        return true;
    }

    virtual void RemoveController(ControllerBasePtr controller)
    {
        boost::mutex::scoped_lock lock(_mutex);
        _listcontrollers.remove(controller);
        if( _ptransformcontroller == controller ) {
            _ptransformcontroller.reset();
        }
        FOREACH(it,_vcontrollersbydofs) {
            if( *it == controller ) {
                it->reset();
            }
        }
    }

    virtual ControllerBasePtr GetController(int dof) const
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( dof < 0 ) {
            return _ptransformcontroller;
        }
        int index=0;
        FOREACHC(it,_dofindices) {
            if( *it == dof ) {
                return _vcontrollersbydofs.at(index);
            }
            index++;
        }
        return ControllerBasePtr();
    }

    virtual void Reset(int options=0)
    {
        boost::mutex::scoped_lock lock(_mutex);
        FOREACH(itcontroller,_listcontrollers) {
            (*itcontroller)->Reset(options);
        }
    }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans=TransformConstPtr())
    {
        boost::mutex::scoped_lock lock(_mutex);
        vector<dReal> v;
        bool bsuccess = true;
        FOREACH(itcontroller,_listcontrollers) {
            v.resize(0);
            FOREACHC(it, (*itcontroller)->GetControlDOFIndices()) {
                v.push_back(values.at(_dofreverseindices.at(*it)));
            }
            bsuccess &= (*itcontroller)->SetDesired(v,trans);
        }
        return bsuccess;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        boost::mutex::scoped_lock lock(_mutex);
        bool bsuccess = true;
        FOREACH(itcontroller,_listcontrollers) {
            bsuccess &= (*itcontroller)->SetPath(ptraj);
        }
        return bsuccess;
    }

    virtual void SimulationStep(dReal fTimeElapsed) {
        boost::mutex::scoped_lock lock(_mutex);
        FOREACH(it,_listcontrollers) {
            (*it)->SimulationStep(fTimeElapsed);
        }
    }

    virtual bool IsDone()
    {
        boost::mutex::scoped_lock lock(_mutex);
        bool bdone=true;
        FOREACH(it,_listcontrollers) {
            bdone &= (*it)->IsDone();
        }
        return bdone;
    }

    virtual dReal GetTime() const
    {
        boost::mutex::scoped_lock lock(_mutex);
        dReal t = 0;
        FOREACHC(it,_listcontrollers) {
            if( it == _listcontrollers.begin() ) {
                t = (*it)->GetTime();
            }
            else {
                dReal tnew = (*it)->GetTime();
                if( RaveFabs(t-tnew) > 0.000001 ) {
                    RAVELOG_WARN(str(boost::format("multi-controller time is different! %f!=%f\n")%t%tnew));
                }
                t = max(t,tnew);
            }
        }
        return t;
    }

    virtual void GetVelocity(std::vector<dReal>& vel) const
    {
        boost::mutex::scoped_lock lock(_mutex);
        vel.resize(_dofindices.size());
        FOREACH(it,vel) {
            *it = 0;
        }
        vector<dReal> v;
        FOREACHC(itcontroller,_listcontrollers) {
            (*itcontroller)->GetVelocity(v);
            int index=0;
            FOREACH(it,v) {
                vel.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
            }
        }
    }

    /// get torque/current/strain values
    /// \param torque [out] - returns the current torque/current/strain exerted by each of the dofs from outside forces.
    /// The feedforward and friction terms should be subtracted out already
    virtual void GetTorque(std::vector<dReal>& torque) const
    {
        boost::mutex::scoped_lock lock(_mutex);
        torque.resize(_dofindices.size());
        FOREACH(it,torque) {
            *it = 0;
        }
        vector<dReal> v;
        FOREACHC(itcontroller,_listcontrollers) {
            (*itcontroller)->GetTorque(v);
            int index=0;
            FOREACH(it,v) {
                torque.at(_dofreverseindices.at((*itcontroller)->GetControlDOFIndices().at(index++))) = *it;
            }
        }
    }

protected:
    RobotBasePtr _probot;
    std::vector<int> _dofindices, _dofreverseindices;
    int _nControlTransformation;
    std::list<ControllerBasePtr> _listcontrollers;
    std::vector<ControllerBasePtr> _vcontrollersbydofs;
    ControllerBasePtr _ptransformcontroller;
    TrajectoryBasePtr _ptraj;
    mutable boost::mutex _mutex;
};

MultiControllerBasePtr CreateMultiController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return MultiControllerBasePtr(new MultiController(penv));
}

}
