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
        py::array_int GetGripperJoints();
        py::array_int GetGripperIndices();
        py::array_int GetArmJoints();
        py::array_int GetArmIndices();
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
        bool CheckIndependentCollision(PyCollisionReportPtr pReport) const;

        object CalculateJacobian();
        object CalculateRotationJacobian();

        object CalculateAngularVelocityJacobian();

        object GetInfo();
        std::string GetStructureHash() const;
        std::string GetKinematicsStructureHash() const;
        std::string GetInverseKinematicsStructureHash(IkParameterizationType iktype) const;

        std::string __repr__();
        std::string __str__();
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
        object GetRelativeTransform() const;
        object GetTransform() const;
        object GetTransformPose() const;
        PyRobotBasePtr GetRobot() const;
        object GetName() const;

        object GetData();

        void SetRelativeTransform(object transform);
        std::string GetStructureHash() const;

        void UpdateInfo(SensorBase::SensorType type=SensorBase::ST_Invalid);

        object UpdateAndGetInfo(SensorBase::SensorType type=SensorBase::ST_Invalid);

        object GetInfo();

        std::string __repr__();
        std::string __str__();
        object __unicode__();
        bool __eq__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p);
        bool __ne__(OPENRAVE_SHARED_PTR<PyAttachedSensor> p);
        long __hash__();
    };

    typedef OPENRAVE_SHARED_PTR<PyAttachedSensor> PyAttachedSensorPtr;
    OPENRAVE_SHARED_PTR<PyAttachedSensor> _GetAttachedSensor(RobotBase::AttachedSensorPtr pattachedsensor);

    class PyConnectedBody {
        RobotBase::ConnectedBodyPtr _pconnected;
        PyEnvironmentBasePtr _pyenv;
public:
        PyConnectedBody(RobotBase::ConnectedBodyPtr pconnected, PyEnvironmentBasePtr pyenv);

        virtual ~PyConnectedBody();
        RobotBase::ConnectedBodyPtr GetConnectedBody() const;

        object GetName();

        object GetInfo();

        bool SetActive(bool active);

        bool IsActive();
        object GetTransform() const;
        object GetTransformPose() const;

        object GetRelativeTransform() const;
        object GetRelativeTransformPose() const;

        void SetLinkEnable(bool enable);

        void SetLinkVisible(bool visible);

        object GetResolvedLinks();

        object GetResolvedJoints();

        object GetResolvedManipulators();

        std::string __repr__();

        std::string __str__();

        object __unicode__();

        bool __eq__(OPENRAVE_SHARED_PTR<PyConnectedBody> p);

        bool __ne__(OPENRAVE_SHARED_PTR<PyConnectedBody> p);

        long __hash__();
    };

    typedef OPENRAVE_SHARED_PTR<PyConnectedBody> PyConnectedBodyPtr;
    OPENRAVE_SHARED_PTR<PyConnectedBody> _GetConnectedBody(RobotBase::ConnectedBodyPtr pConnectedBody);

    class PyRobotStateSaver
    {
        PyEnvironmentBasePtr _pyenv;
        RobotBase::RobotStateSaver _state;
public:
        PyRobotStateSaver(PyRobotBasePtr pyrobot);
        PyRobotStateSaver(PyRobotBasePtr pyrobot, object options);
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv);
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv, object options);
        virtual ~PyRobotStateSaver();
        object GetBody() const;

        void Restore(PyRobotBasePtr pyrobot=PyRobotBasePtr());

        void Release();

        std::string __str__();
        object __unicode__();
    };
    typedef OPENRAVE_SHARED_PTR<PyRobotStateSaver> PyRobotStateSaverPtr;

    PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv);
    PyRobotBase(const PyRobotBase &r);
    virtual ~PyRobotBase();

    bool Init(object olinkinfos, object ojointinfos, object omanipinfos, object oattachedsensorinfos, object oconnectedbodyinfos, const std::string& uri=std::string());

    object GetManipulators();

    object GetManipulators(const std::string& manipname);
    PyManipulatorPtr GetManipulator(const std::string& manipname);
    PyManipulatorPtr SetActiveManipulator(const std::string& manipname);
    PyManipulatorPtr SetActiveManipulator(PyManipulatorPtr pmanip);
    PyManipulatorPtr GetActiveManipulator();

    PyManipulatorPtr AddManipulator(PyManipulatorInfoPtr pmanipinfo, bool removeduplicate=false);
    bool RemoveManipulator(PyManipulatorPtr pmanip);

    PyAttachedSensorPtr AddAttachedSensor(PyAttachedSensorInfoPtr pattsensorinfo, bool removeduplicate=false);
    bool RemoveAttachedSensor(PyAttachedSensorPtr pyattsensor);

    object GetSensors();

    object GetAttachedSensors();
    OPENRAVE_SHARED_PTR<PyAttachedSensor> GetSensor(const std::string& sensorname);

    OPENRAVE_SHARED_PTR<PyAttachedSensor> GetAttachedSensor(const std::string& sensorname);

    PyConnectedBodyPtr AddConnectedBody(PyConnectedBodyInfoPtr pConnectedBodyInfo, bool removeduplicate=false);

    bool RemoveConnectedBody(PyConnectedBodyPtr pConnectedBody);

    object GetConnectedBodies();

    PyConnectedBodyPtr GetConnectedBody(const std::string& bodyname);

    object GetConnectedBodyActiveStates() const;

    void SetConnectedBodyActiveStates(object oactivestates);

    object GetController() const;

    bool SetController(PyControllerBasePtr pController, const std::string& args);

    bool SetController(PyControllerBasePtr pController, object odofindices, int nControlTransformation);

    bool SetController(PyControllerBasePtr pController);

    void SetActiveDOFs(const object& dofindices);
    void SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask);
    void SetActiveDOFs(const object& dofindices, int nAffineDOsBitmask, object rotationaxis);

    int GetActiveDOF() const ;
    int GetAffineDOF() const;
    int GetAffineDOFIndex(DOFAffine dof) const;

    object GetAffineRotationAxis() const;
    void SetAffineTranslationLimits(object lower, object upper);
    void SetAffineRotationAxisLimits(object lower, object upper);
    void SetAffineRotation3DLimits(object lower, object upper);
    void SetAffineRotationQuatLimits(object quatangle);
    void SetAffineTranslationMaxVels(object vels);
    void SetAffineRotationAxisMaxVels(object vels);
    void SetAffineRotation3DMaxVels(object vels);
    void SetAffineRotationQuatMaxVels(dReal vels);
    void SetAffineTranslationResolution(object resolution);
    void SetAffineRotationAxisResolution(object resolution);
    void SetAffineRotation3DResolution(object resolution);
    void SetAffineRotationQuatResolution(dReal resolution);
    void SetAffineTranslationWeights(object weights);
    void SetAffineRotationAxisWeights(object weights);
    void SetAffineRotation3DWeights(object weights);
    void SetAffineRotationQuatWeights(dReal weights);

    object GetAffineTranslationLimits() const;

    object GetAffineRotationAxisLimits() const;
    object GetAffineRotation3DLimits() const;
    object GetAffineRotationQuatLimits() const;
    object GetAffineTranslationMaxVels() const;
    object GetAffineRotationAxisMaxVels() const;
    object GetAffineRotation3DMaxVels() const;
    dReal GetAffineRotationQuatMaxVels() const;
    object GetAffineTranslationResolution() const;
    object GetAffineRotationAxisResolution() const;
    object GetAffineRotation3DResolution() const;
    dReal GetAffineRotationQuatResolution() const;
    object GetAffineTranslationWeights() const;
    object GetAffineRotationAxisWeights() const;
    object GetAffineRotation3DWeights() const;
    dReal GetAffineRotationQuatWeights() const;

    void SetActiveDOFValues(object values, uint32_t checklimits=KinBody::CLA_CheckLimits) const;
    object GetActiveDOFValues() const;

    object GetActiveDOFWeights() const;

    void SetActiveDOFVelocities(object velocities, uint32_t checklimits=KinBody::CLA_CheckLimits);
    object GetActiveDOFVelocities() const;

    object GetActiveDOFLimits() const;

    object GetActiveDOFMaxVel() const;

    object GetActiveDOFMaxAccel() const;

    object GetActiveDOFMaxJerk() const;

    object GetActiveDOFHardMaxVel() const;

    object GetActiveDOFHardMaxAccel() const;

    object GetActiveDOFHardMaxJerk() const;

    object GetActiveDOFResolutions() const;

    object GetActiveConfigurationSpecification(const std::string& interpolation="") const;

    object GetActiveJointIndices();
    object GetActiveDOFIndices();

    object SubtractActiveDOFValues(object ovalues0, object ovalues1);

    object CalculateActiveJacobian(int index, object offset) const;

    object CalculateActiveRotationJacobian(int index, object q) const;

    object CalculateActiveAngularVelocityJacobian(int index) const;

    bool Grab(PyKinBodyPtr pbody);

    // since PyKinBody::Grab is overloaded with (pbody, plink) parameters, have to support both...?
    bool Grab(PyKinBodyPtr pbody, object pylink_or_linkstoignore);
    bool Grab(PyKinBodyPtr pbody, object pylink, object linkstoignore);

    bool CheckLinkSelfCollision(int ilinkindex, object olinktrans, PyCollisionReportPtr pyreport=PyCollisionReportPtr());

    bool WaitForController(float ftimeout);

    std::string GetRobotStructureHash() const;

    virtual PyStateRestoreContextBase* CreateStateSaver(object options);

    PyStateRestoreContextBase* CreateRobotStateSaver(object options=py::none_());

    virtual std::string __repr__();
    virtual std::string __str__();
    virtual object __unicode__();
    virtual void __enter__();
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_ROBOT_H
