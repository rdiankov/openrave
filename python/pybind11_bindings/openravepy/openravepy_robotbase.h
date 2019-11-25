// -*- coding: utf-8 -*-
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
#ifndef OPENRAVEPY_INTERNAL_ROBOT_H
#define OPENRAVEPY_INTERNAL_ROBOT_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_manipulatorinfo.h>
#include <openravepy/openravepy_configurationspecification.h>
#include <openravepy/openravepy_jointinfo.h>

namespace openravepy {
using py::object;

class PyRobotBase : public PyKinBody
{
protected:
    RobotBasePtr _probot;
public:
    RobotBasePtr GetRobot();
    class PyManipulator
    {
        RobotBase::ManipulatorPtr _pmanip;
        PyEnvironmentBasePtr _pyenv;
public:
        PyManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv);
        virtual ~PyManipulator();

        RobotBase::ManipulatorPtr GetManipulator() const;

        object GetTransform() const;

        object GetTransformPose() const;

        object GetVelocity() const;

        object GetName() const;

        void SetName(const std::string& s);

        PyRobotBasePtr GetRobot();

        bool SetIkSolver(PyIkSolverBasePtr iksolver);
        object GetIkSolver();
        object GetBase();
        object GetEndEffector();
        void ReleaseAllGrabbed();
        object GetGraspTransform();
        object GetLocalToolTransform();
        object GetLocalToolTransformPose();
        void SetLocalToolTransform(object otrans);
        void SetLocalToolDirection(object odirection);
        void SetClosingDirection(object oclosingdirection);
        void SetChuckingDirection(object ochuckingdirection);
        object GetGripperJoints();
        object GetGripperIndices();
        object GetArmJoints();
        object GetArmIndices();
        object GetArmDOFValues();
        object GetGripperDOFValues();
        int GetArmDOF();
        int GetGripperDOF();
        object GetClosingDirection();
        object GetChuckingDirection();
        object GetDirection();
        object GetLocalToolDirection();
        bool IsGrabbing(PyKinBodyPtr pbody);

        int GetNumFreeParameters() const;

        object GetFreeParameters() const;

        bool _FindIKSolution(const IkParameterization& ikparam, std::vector<dReal>& solution, int filteroptions, bool releasegil) const;
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions, bool releasegil) const;
        bool _FindIKSolution(const IkParameterization& ikparam, int filteroptions, IkReturn& ikreturn, bool releasegil) const;
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturn& ikreturn, bool releasegil) const;

        bool _FindIKSolutions(const IkParameterization& ikparam, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const;
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const;
        bool _FindIKSolutions(const IkParameterization& ikparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const;
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const;

        object FindIKSolution(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const;

        object FindIKSolution(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const;

        object FindIKSolutions(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const;

        object FindIKSolutions(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const;

        object GetIkParameterization(object oparam, bool inworld=true);

        object GetChildJoints();
        object GetChildDOFIndices();

        object GetChildLinks();

        bool IsChildLink(object pylink);

        object GetIndependentLinks();

        object GetArmConfigurationSpecification(const std::string& interpolation="") const;
        object GetIkConfigurationSpecification(IkParameterizationType iktype, const std::string& interpolation="") const;
        bool CheckEndEffectorCollision(PyCollisionReportPtr pyreport) const;

        bool CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pyreport=PyCollisionReportPtr(), int numredundantsamples=0) const;

        bool CheckEndEffectorSelfCollision(PyCollisionReportPtr pyreport) const;
        bool CheckEndEffectorSelfCollision(object otrans, PyCollisionReportPtr pyreport=PyCollisionReportPtr(), int numredundantsamples=0, bool ignoreManipulatorLinks=false) const;
        bool CheckIndependentCollision() const;

        object CalculateJacobian();
        object CalculateRotationJacobian();

        object CalculateAngularVelocityJacobian();

        object GetInfo();
        string GetStructureHash() const;
        string GetKinematicsStructureHash() const;
        string GetInverseKinematicsStructureHash(IkParameterizationType iktype) const;

        string __repr__();
        string __str__();
        object __unicode__();
        bool __eq__(OPENRAVE_SHARED_PTR<PyManipulator> p);
        bool __ne__(OPENRAVE_SHARED_PTR<PyManipulator> p);
        long __hash__();
    };
    typedef OPENRAVE_SHARED_PTR<PyManipulator> PyManipulatorPtr;
    PyManipulatorPtr _GetManipulator(RobotBase::ManipulatorPtr pmanip);

    class PyAttachedSensor
    {
        RobotBase::AttachedSensorPtr _pattached;
        PyEnvironmentBasePtr _pyenv;
public:
        PyAttachedSensor(RobotBase::AttachedSensorPtr pattached, PyEnvironmentBasePtr pyenv);
        virtual ~PyAttachedSensor();

        RobotBase::AttachedSensorPtr GetAttachedSensor() const;
        object GetSensor();
        object GetAttachingLink() const;
        object GetRelativeTransform() const {
            return ReturnTransform(_pattached->GetRelativeTransform());
        }
        object GetTransform() const {
            return ReturnTransform(_pattached->GetTransform());
        }
        object GetTransformPose() const {
            return toPyArray(_pattached->GetTransform());
        }
        PyRobotBasePtr GetRobot() const {
            return _pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv));
        }
        object GetName() const {
            return ConvertStringToUnicode(_pattached->GetName());
        }

        object GetData()
        {
            return openravepy::toPySensorData(_pattached->GetSensor(),_pyenv);
        }

        void SetRelativeTransform(object transform) {
            _pattached->SetRelativeTransform(ExtractTransform(transform));
        }
        string GetStructureHash() const {
            return _pattached->GetStructureHash();
        }

        void UpdateInfo(SensorBase::SensorType type=SensorBase::ST_Invalid) {
            _pattached->UpdateInfo(type);
        }

        object UpdateAndGetInfo(SensorBase::SensorType type=SensorBase::ST_Invalid) {
            return py::to_object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->UpdateAndGetInfo(type))));
        }

        object GetInfo() {
            return py::to_object(PyAttachedSensorInfoPtr(new PyAttachedSensorInfo(_pattached->GetInfo())));
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetAttachedSensor('%s')")%RaveGetEnvironmentId(_pattached->GetRobot()->GetEnv())%_pattached->GetRobot()->GetName()%_pattached->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<attachedsensor:%s, parent=%s>")%_pattached->GetName()%_pattached->GetRobot()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p) {
            return !!p && _pattached==p->_pattached;
        }
        bool __ne__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p) {
            return !p || _pattached!=p->_pattached;
        }
        long __hash__() {
            return static_cast<long>(uintptr_t(_pattached.get()));
        }
    };

    typedef OPENRAVE_SHARED_PTR<PyAttachedSensor> PyAttachedSensorPtr;
    OPENRAVE_SHARED_PTR<PyAttachedSensor> _GetAttachedSensor(RobotBase::AttachedSensorPtr pattachedsensor)
    {
        return !pattachedsensor ? PyAttachedSensorPtr() : PyAttachedSensorPtr(new PyAttachedSensor(pattachedsensor, _pyenv));
    }

    class PyConnectedBody {
        RobotBase::ConnectedBodyPtr _pconnected;
        PyEnvironmentBasePtr _pyenv;
public:
        PyConnectedBody(RobotBase::ConnectedBodyPtr pconnected, PyEnvironmentBasePtr pyenv) : _pconnected(pconnected),
            _pyenv(pyenv) {
        }

        virtual ~PyConnectedBody() {
        }

        RobotBase::ConnectedBodyPtr GetConnectedBody() const {
            return _pconnected;
        }

        object GetName() {
            return ConvertStringToUnicode(_pconnected->GetName());
        }

        object GetInfo() {
            return py::to_object(PyConnectedBodyInfoPtr(new PyConnectedBodyInfo(_pconnected->GetInfo(), _pyenv)));
        }

        bool SetActive(bool active) {
            return _pconnected->SetActive(active);
        }

        bool IsActive() {
            return _pconnected->IsActive();
        }
        object GetTransform() const {
            return ReturnTransform(_pconnected->GetTransform());
        }
        object GetTransformPose() const {
            return toPyArray(_pconnected->GetTransform());
        }

        object GetRelativeTransform() const {
            return ReturnTransform(_pconnected->GetRelativeTransform());
        }
        object GetRelativeTransformPose() const {
            return toPyArray(_pconnected->GetRelativeTransform());
        }

        void SetLinkEnable(bool enable) {
            _pconnected->SetLinkEnable(enable);
        }

        void SetLinkVisible(bool visible) {
            _pconnected->SetLinkVisible(visible);
        }

        object GetResolvedLinks()
        {
            py::list olinks;
            std::vector<KinBody::LinkPtr> vlinks;
            _pconnected->GetResolvedLinks(vlinks);
            FOREACH(itlink, vlinks) {
                olinks.append(toPyLink(*itlink,_pyenv));
            }
            return olinks;
        }

        object GetResolvedJoints()
        {
            py::list ojoints;
            std::vector<KinBody::JointPtr> vjoints;
            _pconnected->GetResolvedJoints(vjoints);
            FOREACH(itjoint, vjoints) {
                ojoints.append(toPyJoint(*itjoint, _pyenv));
            }
            return ojoints;
        }

        object GetResolvedManipulators()
        {
            py::list omanips;
            std::vector<RobotBase::ManipulatorPtr> vmanips;
            _pconnected->GetResolvedManipulators(vmanips);
            FOREACH(itmanip, vmanips) {
                omanips.append(toPyRobotManipulator(*itmanip, _pyenv));
            }
            return omanips;
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetConnectedBody('%s')") %
                              RaveGetEnvironmentId(_pconnected->GetRobot()->GetEnv()) %
                              _pconnected->GetRobot()->GetName() % _pconnected->GetName());
        }

        string __str__() {
            return boost::str(boost::format("<attachedbody:%s, parent=%s>") % _pconnected->GetName() %
                              _pconnected->GetRobot()->GetName());
        }

        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }

        bool __eq__(OPENRAVE_SHARED_PTR<PyConnectedBody> p) {
            return !!p && _pconnected == p->_pconnected;
        }

        bool __ne__(OPENRAVE_SHARED_PTR<PyConnectedBody> p) {
            return !p || _pconnected != p->_pconnected;
        }

        long __hash__() {
            return static_cast<long>(uintptr_t(_pconnected.get()));
        }
    };

    typedef OPENRAVE_SHARED_PTR<PyConnectedBody> PyConnectedBodyPtr;
    OPENRAVE_SHARED_PTR<PyConnectedBody> _GetConnectedBody(RobotBase::ConnectedBodyPtr pConnectedBody)
    {
        return !pConnectedBody ? PyConnectedBodyPtr() : PyConnectedBodyPtr(new PyConnectedBody(pConnectedBody, _pyenv));
    }

    class PyRobotStateSaver
    {
        PyEnvironmentBasePtr _pyenv;
        RobotBase::RobotStateSaver _state;
public:
        PyRobotStateSaver(PyRobotBasePtr pyrobot) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot()) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);

        }
        PyRobotStateSaver(PyRobotBasePtr pyrobot, object options) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot(),pyGetIntFromPy(options,0)) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(probot) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(probot,pyGetIntFromPy(options,0)) {
            // python should not support restoring on destruction since there's garbage collection
            _state.SetRestoreOnDestructor(false);
        }
        virtual ~PyRobotStateSaver() {
        }

        object GetBody() const {
            return py::to_object(toPyRobot(RaveInterfaceCast<RobotBase>(_state.GetBody()),_pyenv));
        }

        void Restore(PyRobotBasePtr pyrobot=PyRobotBasePtr()) {
            _state.Restore(!pyrobot ? RobotBasePtr() : pyrobot->GetRobot());
        }

        void Release() {
            _state.Release();
        }

        std::string __str__() {
            KinBodyPtr pbody = _state.GetBody();
            if( !pbody ) {
                return "robot state empty";
            }
            return boost::str(boost::format("robot state for %s")%pbody->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
    };
    typedef OPENRAVE_SHARED_PTR<PyRobotStateSaver> PyRobotStateSaverPtr;

    PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : PyKinBody(probot,pyenv), _probot(probot) {
    }
    PyRobotBase(const PyRobotBase &r) : PyKinBody(r._probot,r._pyenv) {
        _probot = r._probot;
    }
    virtual ~PyRobotBase() {
    }

    bool Init(object olinkinfos, object ojointinfos, object omanipinfos, object oattachedsensorinfos, const std::string& uri=std::string()) {
        std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
        _ParseLinkInfos(olinkinfos, vlinkinfos);
        std::vector<KinBody::JointInfoConstPtr> vjointinfos;
        _ParseJointInfos(ojointinfos, vjointinfos);
        std::vector<RobotBase::ManipulatorInfoConstPtr> vmanipinfos(len(omanipinfos));
        for(size_t i = 0; i < vmanipinfos.size(); ++i) {
            PyManipulatorInfoPtr pymanip = py::extract<PyManipulatorInfoPtr>(omanipinfos[i]);
            if( !pymanip ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.ManipInfo"),ORE_InvalidArguments);
            }
            vmanipinfos[i] = pymanip->GetManipulatorInfo();
        }
        std::vector<RobotBase::AttachedSensorInfoConstPtr> vattachedsensorinfos(len(oattachedsensorinfos));
        for(size_t i = 0; i < vattachedsensorinfos.size(); ++i) {
            PyAttachedSensorInfoPtr pyattachedsensor = py::extract<PyAttachedSensorInfoPtr>(oattachedsensorinfos[i]);
            if( !pyattachedsensor ) {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot cast to KinBody.AttachedsensorInfo"),ORE_InvalidArguments);
            }
            vattachedsensorinfos[i] = pyattachedsensor->GetAttachedSensorInfo();
        }
        return _probot->Init(vlinkinfos, vjointinfos, vmanipinfos, vattachedsensorinfos, uri);
    }

    object GetManipulators()
    {
        py::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            manips.append(_GetManipulator(*it));
        }
        return manips;
    }

    object GetManipulators(const string& manipname)
    {
        py::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                manips.append(_GetManipulator(*it));
            }
        }
        return manips;
    }
    PyManipulatorPtr GetManipulator(const string& manipname)
    {
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                return _GetManipulator(*it);
            }
        }
        return PyManipulatorPtr();
    }

    PyManipulatorPtr SetActiveManipulator(const std::string& manipname) {
        _probot->SetActiveManipulator(manipname);
        return GetActiveManipulator();
    }
    PyManipulatorPtr SetActiveManipulator(PyManipulatorPtr pmanip) {
        _probot->SetActiveManipulator(pmanip->GetManipulator());
        return GetActiveManipulator();
    }
    PyManipulatorPtr GetActiveManipulator() {
        return _GetManipulator(_probot->GetActiveManipulator());
    }

    PyManipulatorPtr AddManipulator(PyManipulatorInfoPtr pmanipinfo, bool removeduplicate=false) {
        return _GetManipulator(_probot->AddManipulator(*pmanipinfo->GetManipulatorInfo(), removeduplicate));
    }
    bool RemoveManipulator(PyManipulatorPtr pmanip) {
        return _probot->RemoveManipulator(pmanip->GetManipulator());
    }

    PyAttachedSensorPtr AddAttachedSensor(PyAttachedSensorInfoPtr pattsensorinfo, bool removeduplicate=false) {
        return _GetAttachedSensor(_probot->AddAttachedSensor(*pattsensorinfo->GetAttachedSensorInfo(), removeduplicate));
    }
    bool RemoveAttachedSensor(PyAttachedSensorPtr pyattsensor) {
        return _probot->RemoveAttachedSensor(*pyattsensor->GetAttachedSensor());
    }

    object GetSensors()
    {
        RAVELOG_WARN("GetSensors is deprecated, please use GetAttachedSensors\n");
        return GetAttachedSensors();
    }

    object GetAttachedSensors()
    {
        py::list sensors;
        FOREACH(itsensor, _probot->GetAttachedSensors()) {
            sensors.append(OPENRAVE_SHARED_PTR<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
        }
        return sensors;
    }
    OPENRAVE_SHARED_PTR<PyAttachedSensor> GetSensor(const string& sensorname)
    {
        RAVELOG_WARN("GetSensor is deprecated, please use GetAttachedSensor\n");
        return GetAttachedSensor(sensorname);
    }

    OPENRAVE_SHARED_PTR<PyAttachedSensor> GetAttachedSensor(const string& sensorname)
    {
        return _GetAttachedSensor(_probot->GetAttachedSensor(sensorname));
    }

    PyConnectedBodyPtr AddConnectedBody(PyConnectedBodyInfoPtr pConnectedBodyInfo, bool removeduplicate=false) {
        return _GetConnectedBody(_probot->AddConnectedBody(*pConnectedBodyInfo->GetConnectedBodyInfo(), removeduplicate));
    }

    bool RemoveConnectedBody(PyConnectedBodyPtr pConnectedBody) {
        return _probot->RemoveConnectedBody(*pConnectedBody->GetConnectedBody());
    }

    object GetConnectedBodies()
    {
        py::list bodies;
        FOREACH(itbody, _probot->GetConnectedBodies()) {
            bodies.append(OPENRAVE_SHARED_PTR<PyConnectedBody>(new PyConnectedBody(*itbody, _pyenv)));
        }
        return bodies;
    }

    PyConnectedBodyPtr GetConnectedBody(const string& bodyname)
    {
        FOREACH(itbody, _probot->GetConnectedBodies()) {
            if( (*itbody)->GetName() == bodyname ) {
                return _GetConnectedBody(*itbody);
            }
        }
        return PyConnectedBodyPtr();
    }

    object GetConnectedBodyActiveStates() const
    {
        std::vector<uint8_t> activestates;
        _probot->GetConnectedBodyActiveStates(activestates);
        return toPyArray(activestates);
    }

    void SetConnectedBodyActiveStates(object oactivestates)
    {
        std::vector<uint8_t> activestates = ExtractArray<uint8_t>(oactivestates);
        _probot->SetConnectedBodyActiveStates(activestates);
    }

    object GetController() const {
        CHECK_POINTER(_probot);
        return py::to_object(openravepy::toPyController(_probot->GetController(),_pyenv));
    }

    bool SetController(PyControllerBasePtr pController, const string& PY_ARGS) {
        RAVELOG_WARN("RobotBase::SetController(PyControllerBasePtr,PY_ARGS) is deprecated\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    bool SetController(PyControllerBasePtr pController, object odofindices, int nControlTransformation) {
        CHECK_POINTER(pController);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _probot->SetController(openravepy::GetController(pController),dofindices,nControlTransformation);
    }

    bool SetController(PyControllerBasePtr pController) {
        RAVELOG_VERBOSE("RobotBase::SetController(PyControllerBasePtr) will control all DOFs and transformation\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    void SetActiveDOFs(const object& dofindices) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices));
    }
    void SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask);
    }
    void SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask, object rotationaxis) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
    }

    int GetActiveDOF() const {
        return _probot->GetActiveDOF();
    }
    int GetAffineDOF() const {
        return _probot->GetAffineDOF();
    }
    int GetAffineDOFIndex(DOFAffine dof) const {
        return _probot->GetAffineDOFIndex(dof);
    }

    object GetAffineRotationAxis() const {
        return toPyVector3(_probot->GetAffineRotationAxis());
    }
    void SetAffineTranslationLimits(object lower, object upper) {
        return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationAxisLimits(object lower, object upper) {
        return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotation3DLimits(object lower, object upper) {
        return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationQuatLimits(object quatangle) {
        return _probot->SetAffineRotationQuatLimits(ExtractVector4(quatangle));
    }
    void SetAffineTranslationMaxVels(object vels) {
        _probot->SetAffineTranslationMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationAxisMaxVels(object vels) {
        _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotation3DMaxVels(object vels) {
        _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationQuatMaxVels(dReal vels) {
        _probot->SetAffineRotationQuatMaxVels(vels);
    }
    void SetAffineTranslationResolution(object resolution) {
        _probot->SetAffineTranslationResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationAxisResolution(object resolution) {
        _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution));
    }
    void SetAffineRotation3DResolution(object resolution) {
        _probot->SetAffineRotation3DResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationQuatResolution(dReal resolution) {
        _probot->SetAffineRotationQuatResolution(resolution);
    }
    void SetAffineTranslationWeights(object weights) {
        _probot->SetAffineTranslationWeights(ExtractVector3(weights));
    }
    void SetAffineRotationAxisWeights(object weights) {
        _probot->SetAffineRotationAxisWeights(ExtractVector4(weights));
    }
    void SetAffineRotation3DWeights(object weights) {
        _probot->SetAffineRotation3DWeights(ExtractVector3(weights));
    }
    void SetAffineRotationQuatWeights(dReal weights) {
        _probot->SetAffineRotationQuatWeights(weights);
    }

    object GetAffineTranslationLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineTranslationLimits(lower,upper);
        return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationAxisLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotationAxisLimits(lower,upper);
        return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotation3DLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotation3DLimits(lower,upper);
        return py::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationQuatLimits() const
    {
        return toPyVector4(_probot->GetAffineRotationQuatLimits());
    }
    object GetAffineTranslationMaxVels() const {
        return toPyVector3(_probot->GetAffineTranslationMaxVels());
    }
    object GetAffineRotationAxisMaxVels() const {
        return toPyVector3(_probot->GetAffineRotationAxisMaxVels());
    }
    object GetAffineRotation3DMaxVels() const {
        return toPyVector3(_probot->GetAffineRotation3DMaxVels());
    }
    dReal GetAffineRotationQuatMaxVels() const {
        return _probot->GetAffineRotationQuatMaxVels();
    }
    object GetAffineTranslationResolution() const {
        return toPyVector3(_probot->GetAffineTranslationResolution());
    }
    object GetAffineRotationAxisResolution() const {
        return toPyVector4(_probot->GetAffineRotationAxisResolution());
    }
    object GetAffineRotation3DResolution() const {
        return toPyVector3(_probot->GetAffineRotation3DResolution());
    }
    dReal GetAffineRotationQuatResolution() const {
        return _probot->GetAffineRotationQuatResolution();
    }
    object GetAffineTranslationWeights() const {
        return toPyVector3(_probot->GetAffineTranslationWeights());
    }
    object GetAffineRotationAxisWeights() const {
        return toPyVector4(_probot->GetAffineRotationAxisWeights());
    }
    object GetAffineRotation3DWeights() const {
        return toPyVector3(_probot->GetAffineRotation3DWeights());
    }
    dReal GetAffineRotationQuatWeights() const {
        return _probot->GetAffineRotationQuatWeights();
    }

    void SetActiveDOFValues(object values, uint32_t checklimits=KinBody::CLA_CheckLimits) const
    {
        vector<dReal> vvalues = ExtractArray<dReal>(values);
        if( vvalues.size() > 0 ) {
            _probot->SetActiveDOFValues(vvalues,checklimits);
        }
        else {
            OPENRAVE_ASSERT_OP_FORMAT((int)vvalues.size(),>=,_probot->GetActiveDOF(), "not enough values %d<%d",vvalues.size()%_probot->GetActiveDOF(),ORE_InvalidArguments);
        }
    }
    object GetActiveDOFValues() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFValues(values);
        return toPyArray(values);
    }

    object GetActiveDOFWeights() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> weights;
        _probot->GetActiveDOFWeights(weights);
        return toPyArray(weights);
    }

    void SetActiveDOFVelocities(object velocities, uint32_t checklimits=KinBody::CLA_CheckLimits)
    {
        _probot->SetActiveDOFVelocities(ExtractArray<dReal>(velocities), checklimits);
    }
    object GetActiveDOFVelocities() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFVelocities(values);
        return toPyArray(values);
    }

    object GetActiveDOFLimits() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::make_tuple(py::empty_array(), py::empty_array()); // always need 2 since users can do lower, upper = GetDOFLimits()
        }
        vector<dReal> lower, upper;
        _probot->GetActiveDOFLimits(lower,upper);
        return py::make_tuple(toPyArray(lower),toPyArray(upper));
    }

    object GetActiveDOFMaxVel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxVel(values);
        return toPyArray(values);
    }

    object GetActiveDOFMaxAccel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxAccel(values);
        return toPyArray(values);
    }

    object GetActiveDOFMaxJerk() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxJerk(values);
        return toPyArray(values);
    }

    object GetActiveDOFHardMaxVel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFHardMaxVel(values);
        return toPyArray(values);
    }

    object GetActiveDOFHardMaxAccel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFHardMaxAccel(values);
        return toPyArray(values);
    }

    object GetActiveDOFHardMaxJerk() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFHardMaxJerk(values);
        return toPyArray(values);
    }

    object GetActiveDOFResolutions() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return py::empty_array();
        }
        vector<dReal> values;
        _probot->GetActiveDOFResolutions(values);
        return toPyArray(values);
    }

    object GetActiveConfigurationSpecification(const std::string& interpolation="") const {
        return py::to_object(openravepy::toPyConfigurationSpecification(_probot->GetActiveConfigurationSpecification(interpolation)));
    }

    object GetActiveJointIndices() {
        RAVELOG_WARN("GetActiveJointIndices deprecated. Use GetActiveDOFIndices\n"); return toPyArray(_probot->GetActiveDOFIndices());
    }
    object GetActiveDOFIndices() {
        return toPyArray(_probot->GetActiveDOFIndices());
    }

    object SubtractActiveDOFValues(object ovalues0, object ovalues1)
    {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetActiveDOF() );
        BOOST_ASSERT((int)values1.size() == GetActiveDOF() );
        _probot->SubtractActiveDOFValues(values0,values1);
        return toPyArray(values0);
    }

    object CalculateActiveJacobian(int index, object offset) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveJacobian(index,ExtractVector3(offset),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveRotationJacobian(int index, object q) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveRotationJacobian(index,ExtractVector4(q),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveAngularVelocityJacobian(int index) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveAngularVelocityJacobian(index,vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    bool Grab(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody());
    }

    // since PyKinBody::Grab is overloaded with (pbody, plink) parameters, have to support both...?
    bool Grab(PyKinBodyPtr pbody, object pylink_or_linkstoignore)
    {
        CHECK_POINTER(pbody);
        CHECK_POINTER(pylink_or_linkstoignore);
        KinBody::LinkPtr plink = GetKinBodyLink(pylink_or_linkstoignore);
        if( !!plink ) {
            return _probot->Grab(pbody->GetBody(), plink);
        }
        // maybe it is a set?
        std::set<int> setlinkstoignore = ExtractSet<int>(pylink_or_linkstoignore);
        return _probot->Grab(pbody->GetBody(), setlinkstoignore);
    }

    bool CheckLinkSelfCollision(int ilinkindex, object olinktrans, PyCollisionReportPtr pyreport=PyCollisionReportPtr())
    {
        return _probot->CheckLinkSelfCollision(ilinkindex, ExtractTransform(olinktrans), !pyreport ? CollisionReportPtr() : openravepy::GetCollisionReport(pyreport));
    }

    bool WaitForController(float ftimeout)
    {
        ControllerBasePtr pcontroller = _probot->GetController();
        if( !pcontroller ) {
            return false;
        }
        if( pcontroller->IsDone() ) {
            return true;
        }
        bool bSuccess = true;
        Py_BEGIN_ALLOW_THREADS;

        try {
            uint64_t starttime = GetMicroTime();
            uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
            while( !pcontroller->IsDone() ) {
                Sleep(1);
                if(( deltatime > 0) &&( (GetMicroTime()-starttime)>deltatime) ) {
                    bSuccess = false;
                    break;
                }
            }
        }
        catch(...) {
            RAVELOG_ERROR("exception raised inside WaitForController:\n");
            PyErr_Print();
            bSuccess = false;
        }

        Py_END_ALLOW_THREADS;
        return bSuccess;
    }

    string GetRobotStructureHash() const {
        return _probot->GetRobotStructureHash();
    }

    virtual PyStateRestoreContextBase* CreateStateSaver(object options) {
        PyRobotStateSaverPtr saver;
        if( IS_PYTHONOBJECT_NONE(options) ) {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv));
        }
        else {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv,options));
        }
        return new PyStateRestoreContext<PyRobotStateSaverPtr, PyRobotBasePtr>(saver);
    }

    PyStateRestoreContextBase* CreateRobotStateSaver(object options=py::none_()) {
        return CreateStateSaver(options);
    }

    virtual string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s')")%RaveGetEnvironmentId(_probot->GetEnv())%_probot->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_probot->GetInterfaceType())%_probot->GetXMLId()%_probot->GetName()%_probot->GetRobotStructureHash());
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    virtual void __enter__()
    {
        // necessary to lock physics to prevent multiple threads from interfering
        if( _listStateSavers.size() == 0 ) {
            openravepy::LockEnvironment(_pyenv);
        }
        _listStateSavers.push_back(OPENRAVE_SHARED_PTR<void>(new RobotBase::RobotStateSaver(_probot)));
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ROBOT_H
