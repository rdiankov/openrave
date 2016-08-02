// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

class RedirectController : public ControllerBase
{
public:
    RedirectController(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv), _bAutoSync(true) {
        __description = ":Interface Author: Rosen Diankov\n\nRedirects all input and output to another controller (this avoides cloning the other controller while still allowing it to be used from cloned environments)";
    }
    virtual ~RedirectController() {
    }

    virtual bool Init(RobotBasePtr robot, const std::vector<int>&dofindices, int nControlTransformation)
    {
        _dofindices.clear();
        _pcontroller.reset();
        _probot = GetEnv()->GetRobot(robot->GetName());
        if( _probot != robot ) {
            _pcontroller = robot->GetController();
            if( !!_pcontroller ) {
                _dofindices = _pcontroller->GetControlDOFIndices();
            }
        }
        if( _bAutoSync ) {
            _sync();
        }
        return true;
    }

    // don't touch the referenced controller, since could be just destroying clones
    virtual void Reset(int options) {
    }

    virtual bool SetDesired(const std::vector<dReal>&values, TransformConstPtr trans)
    {
        if( !_pcontroller->SetDesired(values, trans) ) {
            return false;
        }
        if(_bAutoSync) {
            _sync();
        }
        return true;
    }
    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( !_pcontroller->SetPath(ptraj) ) {
            return false;
        }
        if(_bAutoSync) {
            _sync();
        }
        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed) {
        if( !!_pcontroller ) {
            _pcontroller->SimulationStep(fTimeElapsed);
            if(_bAutoSync) {
                _sync();
            }
        }
    }

    virtual const std::vector<int>& GetControlDOFIndices() const {
        return _dofindices;
    }
    virtual int IsControlTransformation() const {
        return !_pcontroller ? 0 : _pcontroller->IsControlTransformation();
    }
    virtual bool IsDone() {
        return _bAutoSync ? _bSyncDone&&_pcontroller->IsDone() : _pcontroller->IsDone();
    }

    virtual dReal GetTime() const {
        return _pcontroller->GetTime();
    }
    virtual void GetVelocity(std::vector<dReal>&vel) const {
        return _pcontroller->GetVelocity(vel);
    }
    virtual void GetTorque(std::vector<dReal>&torque) const {
        return _pcontroller->GetTorque(torque);
    }
    virtual RobotBasePtr GetRobot() const {
        return _probot;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        ControllerBase::Clone(preference,cloningoptions);
        std::shared_ptr<RedirectController const> r = std::dynamic_pointer_cast<RedirectController const>(preference);
        _probot = GetEnv()->GetRobot(r->_probot->GetName());
        _pcontroller = r->_pcontroller;     // hmm......... this requires some thought
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        streampos pos = is.tellg();
        is >> cmd;
        if( !is ) {
            throw openrave_exception("invalid argument",ORE_InvalidArguments);
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "sync" ) {
            _sync();
            return true;
        }
        else if( cmd == "autosync" ) {
            is >> _bAutoSync;
            if( !is ) {
                return false;
            }
            if( _bAutoSync ) {
                _sync();
            }
            return true;
        }

        is.seekg(pos);
        if( !_pcontroller ) {
            return false;
        }
        return _pcontroller->SendCommand(os,is);
    }

private:
    virtual void _sync()
    {
        if( !!_pcontroller ) {
            vector<Transform> vtrans;
            _pcontroller->GetRobot()->GetLinkTransformations(vtrans);
            _probot->SetLinkTransformations(vtrans);
            _bSyncDone = _pcontroller->IsDone();
        }
    }

    std::vector<int> _dofindices;
    bool _bAutoSync, _bSyncDone;
    RobotBasePtr _probot;               ///< controlled body
    ControllerBasePtr _pcontroller;
};

ControllerBasePtr CreateRedirectController(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ControllerBasePtr(new RedirectController(penv,sinput));
}
