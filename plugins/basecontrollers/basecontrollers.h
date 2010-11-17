// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_CONTROLLERS_H
#define RAVE_CONTROLLERS_H

class IdealController : public ControllerBase
{
 public:
    IdealController(EnvironmentBasePtr penv) : ControllerBase(penv), cmdid(0), _bPause(false), _bIsDone(true) {
        __description = ":Interface Author: Rosen Diankov\nIdeal controller used for planning and non-physics simulations. Forces exact robot positions.";
        fTime = 0;
        _fSpeed = 1;
        _nControlTransformation = 0;
    }
    virtual ~IdealController() {}

    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
    {
        _probot = robot;
        if( flog.is_open() ) {
            flog.close();
        }
        if( !!_probot ) {
            string filename = RaveGetHomeDirectory() + string("/") + _probot->GetName() + string(".traj");
            flog.open(filename.c_str());
            if( !flog )
                RAVELOG_WARN(str(boost::format("failed to open %s\n")%filename));
            flog << GetXMLId() << " " << _probot->GetName() << endl << endl;
            _dofindices = dofindices;
            _nControlTransformation = nControlTransformation;
        }
        _bPause = false;
        return true;
    }

    virtual void Reset(int options)
    {
        _ptraj.reset();
        _vecdesired.resize(0);
        if( flog.is_open() ) {
            flog.close();
        }
    }

    virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
    virtual int IsControlTransformation() const { return _nControlTransformation; }

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
    {
        if( values.size() != _dofindices.size() ) {
            throw openrave_exception(str(boost::format("wrong desired dimensions %d!=%d")%values.size()%_dofindices.size()),ORE_InvalidArguments);
        }
        fTime = 0;
        _ptraj.reset();
        // do not set done to true here! let it be picked up by the simulation thread.
        // this will also let it have consistent mechanics as SetPath
        // (there's a race condition we're avoiding where a user calls SetDesired and then state savers revert the robot)
        if( !_bPause ) {
            EnvironmentMutex::scoped_lock lockenv(_probot->GetEnv()->GetMutex());            
            _vecdesired = values;
            if( _nControlTransformation ) {
                if( !!trans ) {
                    _tdesired = *trans;
                }
                else {
                    _tdesired = _probot->GetTransform();
                }
                _SetDOFValues(_vecdesired,_tdesired);
            }
            else {
                _SetDOFValues(_vecdesired);
            }
            _bIsDone = false; // set after _vecdesired has changed
        }
        return true;
    }

    virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
    {
        if( _bPause ) {
            RAVELOG_DEBUGA("IdealController cannot player trajectories when paused\n");
            _ptraj.reset();
            _bIsDone = true;
            return false;
        }
        if( !!ptraj && ptraj->GetDOF() != (int)_dofindices.size() ) {
            throw openrave_exception(str(boost::format("wrong path dimensions %d!=%d")%ptraj->GetDOF()%_dofindices.size()),ORE_InvalidArguments);
        }
        _ptraj = ptraj;
        fTime = 0;
        _bIsDone = !_ptraj;
        _vecdesired.resize(0);

        if( !!_ptraj && !!flog ) {
            flog << endl << "trajectory: " << ++cmdid << endl;
            _ptraj->Write(flog, Trajectory::TO_IncludeTimestamps|Trajectory::TO_IncludeBaseTransformation);
        }

        return true;
    }

    virtual void SimulationStep(dReal fTimeElapsed)
    {
        if( _bPause ) {
            return;
        }
        if( !!_ptraj ) {
            Trajectory::TPOINT tp;
            if( !_ptraj->SampleTrajectory(fTime, tp) ) {
                return;
            }
            if( tp.q.size() > 0 ) {
                if( _nControlTransformation ) {
                    _SetDOFValues(tp.q,tp.trans);
                }
                else {
                    _SetDOFValues(tp.q);
                }
            }
            else if( _nControlTransformation ) {
                _probot->SetTransform(tp.trans);
            }

            if( fTime > _ptraj->GetTotalDuration() ) {
                fTime = _ptraj->GetTotalDuration();
                _bIsDone = true;
            }

            fTime += _fSpeed * fTimeElapsed;
        }

        if( _vecdesired.size() > 0 ) {
            if( _nControlTransformation ) {
                _SetDOFValues(_vecdesired,_tdesired);
            }
            else {
                _SetDOFValues(_vecdesired);
            }
            _bIsDone = true;
        }
    }

    virtual bool SendCommand(std::ostream& os, std::istream& is)
    {
        string cmd;
        is >> cmd;
        if( !is ) {
            throw openrave_exception("invalid argument",ORE_InvalidArguments);
        }
        std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        if( cmd == "pause" ) {
            is >> _bPause;
        }
        else {
            throw openrave_exception("not commands supported",ORE_CommandNotSupported);
        }
        return true;
    }
    virtual bool IsDone() { return _bIsDone; }
    virtual dReal GetTime() const { return fTime; }
    virtual RobotBasePtr GetRobot() const { return _probot; }

private:
    virtual void _SetDOFValues(const std::vector<dReal>& values)
    {
        vector<dReal> curvalues;
        _probot->GetDOFValues(curvalues);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
        }
//        vector<Vector> vLinearVelocities, vAngularVelocities;
//        GetEnv()->GetPhysicsEngine()->GetBodyVelocity(KinBodyPtr(_probot),vLinearVelocities,vAngularVelocities);
//        vector<Vector> vForces(_probot->GetLinks().size()), vTorques(_probot->GetLinks().size());
//        FOREACH(it,_probot->GetLinks()) {
//            GetEnv()->GetPhysicsEngine()->GetLinkForceTorque(KinBody::LinkConstPtr(*it), vForces.at((*it)->GetIndex()), vTorques.at((*it)->GetIndex()));
//        }
        _probot->SetJointValues(curvalues,true);
//        GetEnv()->GetPhysicsEngine()->SetBodyVelocity(KinBodyPtr(_probot),vLinearVelocities,vAngularVelocities);
//        FOREACH(it,_probot->GetLinks()) {
//            (*it)->SetForce(vForces.at((*it)->GetIndex()), (*it)->GetTransform()*(*it)->GetCOMOffset(), false);
//            (*it)->SetTorque(vTorques.at((*it)->GetIndex()), false);
//        }
    }
    virtual void _SetDOFValues(const std::vector<dReal>& values, const Transform& t)
    {
        BOOST_ASSERT(_nControlTransformation);
        vector<dReal> curvalues;
        _probot->GetDOFValues(curvalues);
        int i = 0;
        FOREACH(it,_dofindices) {
            curvalues.at(*it) = values.at(i++);
        }
        _probot->SetJointValues(curvalues,t, true);
    }

    /// reset velocities of all controlling links
    /// must do this in order to operating along side a running physics engine
//    virtual void _ResetPhysics(vector<Vector>& vLinearVelocities, vector<Vector>& vAngularVelocities)
//    {
//        FOREACH(it,_dofindices) {
//            KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(*it);
//            int index = _GetChildLink(pjoint)->GetIndex();
//            vLinearVelocities.at(index) = Vector();
//            vAngularVelocities.at(index) = Vector();
//        }
//        if( _nControlTransformation ) {
//            vLinearVelocities.at(0) = Vector();
//            vAngularVelocities.at(0) = Vector();
//        }
//        GetEnv()->GetPhysicsEngine()->SetBodyVelocity(KinBodyPtr(_probot),vLinearVelocities,vAngularVelocities);
//        
////        vector<Vector> vForces(_probot->GetLinks().size()), vTorques(_probot->GetLinks().size())
////        FOREACH(it,_probot->GetLinks()) {
////            GetEnv()->GetPhysicsEngine()->GetLinkForceTorque(KinBody::LinkConstPtr(*it), vForces.at((*it)->GetIndex()), vTorques.at((*it)->GetIndex()));
////        }
////        FOREACH(it,_probot->GetLinks()) {
////            (*it)->SetForce(vForces.at((*it)->GetIndex()), (*it)->GetTransform()*(*it)->GetCOMOffset(), false);
////            (*it)->SetTorque(vTorques.at((*it)->GetIndex()), false);
////        }
//    }
//
//    virtual KinBody::LinkPtr _GetChildLink(KinBody::JointConstPtr pjoint) {
//        if( !!pjoint->GetFirstAttached() && !!pjoint->GetSecondAttached() ) {
//            if( pjoint->GetFirstAttached()->GetParentLink() == pjoint->GetSecondAttached() )
//                return pjoint->GetFirstAttached();
//            else if( pjoint->GetSecondAttached()->GetParentLink() == pjoint->GetFirstAttached() )
//                return pjoint->GetSecondAttached();
//        }
//        else if( !!pjoint->GetFirstAttached() ) {
//            return pjoint->GetFirstAttached();
//        }
//        else if( !!pjoint->GetSecondAttached() ) {
//            return pjoint->GetSecondAttached();
//        }
//        RAVELOG_WARN(str(boost::format("joint %s cannot find child link\n")%pjoint->GetName()));
//        return KinBody::LinkPtr();
//    }

    RobotBasePtr _probot;           ///< controlled body
    dReal _fSpeed;                ///< how fast the robot should go
    TrajectoryBaseConstPtr _ptraj;     ///< computed trajectory robot needs to follow in chunks of _pbody->GetDOF()

    dReal fTime;

    std::vector<dReal> _vecdesired;     ///< desired values of the joints    
    Transform _tdesired;

    std::vector<int> _dofindices;
    int _nControlTransformation;
    ofstream flog;
    int cmdid;
    bool _bPause;
    bool _bIsDone;
};

class RedirectController : public ControllerBase
{
 public:
 RedirectController(EnvironmentBasePtr penv) : ControllerBase(penv), _bAutoSync(true) {
        __description = ":Interface Author: Rosen Diankov\nRedirects all input and output to another controller (this avoides cloning the other controller while still allowing it to be used from cloned environments)";
    }
    virtual ~RedirectController() {}
    
    virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
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
    virtual void Reset(int options) {}

    virtual bool SetDesired(const std::vector<dReal>& values, TransformConstPtr trans)
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

    virtual const std::vector<int>& GetControlDOFIndices() const { return _dofindices; }
    virtual int IsControlTransformation() const { return !_pcontroller ? 0 : _pcontroller->IsControlTransformation(); }
    virtual bool IsDone() { return _bAutoSync ? _bSyncDone&&_pcontroller->IsDone() : _pcontroller->IsDone(); }

    virtual dReal GetTime() const { return _pcontroller->GetTime(); }
    virtual void GetVelocity(std::vector<dReal>& vel) const { return _pcontroller->GetVelocity(vel); }
    virtual void GetTorque(std::vector<dReal>& torque) const { return _pcontroller->GetTorque(torque); }
    virtual RobotBasePtr GetRobot() const { return _probot; }

    virtual bool Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        boost::shared_ptr<RedirectController const> r = boost::dynamic_pointer_cast<RedirectController const>(preference);
        if( !r ) {
            return false;
        }
        if( !ControllerBase::Clone(preference,cloningoptions) ) {
            return false;
        }
        _probot = GetEnv()->GetRobot(r->_probot->GetName());
        _pcontroller = r->_pcontroller; // hmm......... this requires some thought
        return true;
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
        return _pcontroller->SendCommand(os,is);
    }
    
private:
    virtual void _sync()
    {
        if( !!_pcontroller ) {
            vector<Transform> vtrans;
            _pcontroller->GetRobot()->GetBodyTransformations(vtrans);
            _probot->SetBodyTransformations(vtrans);
            _bSyncDone = _pcontroller->IsDone();
        }
    }

    std::vector<int> _dofindices;
    bool _bAutoSync, _bSyncDone;
    RobotBasePtr _probot;           ///< controlled body
    ControllerBasePtr _pcontroller;
};

#endif
