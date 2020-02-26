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
#ifndef OPENRAVEPY_INTERNAL_SENSORBASE_H
#define OPENRAVEPY_INTERNAL_SENSORBASE_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class PyCameraIntrinsics
{
public:
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics = geometry::RaveCameraIntrinsics<float>());
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics);

    virtual ~PyCameraIntrinsics();

    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics();
    object K = py::none_();
    std::string distortion_model;
    object distortion_coeffs = py::none_();
    dReal focal_length;
};

class PyCameraGeomData : public PySensorGeometry
{
public:
    PyCameraGeomData();
    PyCameraGeomData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom);
    virtual ~PyCameraGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    virtual object SerializeJSON(dReal fUnitScale=1.0, py::object options=py::none_());
    virtual void DeserializeJSON(py::object obj, dReal fUnitScale=1.0);

    std::string hardware_id;
    PyCameraIntrinsics intrinsics;
    int width = 0;
    int height = 0;
    std::string sensor_reference;
    std::string target_region;
    dReal measurement_time = 1.0;
    dReal gain = 1.0;

private:
    void _Update(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom);
};

class PyLaserGeomData : public PySensorGeometry
{
public:
    PyLaserGeomData();
    PyLaserGeomData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom);
    virtual ~PyLaserGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();
    py::tuple min_angle = py::make_tuple(0.0, 0.0);
    py::tuple max_angle = py::make_tuple(0.0, 0.0);
    py::tuple resolution;
    dReal min_range = 0.0;
    dReal max_range = 0.0;
    dReal time_increment = 0.0;
    dReal time_scan = 0.0;
};

class PyJointEncoderGeomData : public PySensorGeometry
{
public:
    PyJointEncoderGeomData();
    PyJointEncoderGeomData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom);
    virtual ~PyJointEncoderGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    object resolution = py::none_();
};

class PyForce6DGeomData : public PySensorGeometry
{
public:
    PyForce6DGeomData();
    PyForce6DGeomData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom);
    virtual ~PyForce6DGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();
};

class PyIMUGeomData : public PySensorGeometry
{
public:
    PyIMUGeomData();
    PyIMUGeomData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom);
    virtual ~PyIMUGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    dReal time_measurement = 0.0;
};

class PyOdometryGeomData : public PySensorGeometry
{
public:
    PyOdometryGeomData();
    PyOdometryGeomData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom);
    virtual ~PyOdometryGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    std::string targetid;
};

// TODO fill rest of fields
class PyTactileGeomData : public PySensorGeometry
{
public:
    PyTactileGeomData();
    PyTactileGeomData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom);
    ~PyTactileGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    dReal thickness = 0.0;
};

class PyActuatorGeomData : public PySensorGeometry
{
public:
    PyActuatorGeomData();
    PyActuatorGeomData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom);
    virtual ~PyActuatorGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    dReal maxtorque = 0.0;
    dReal maxcurrent = 0.0;
    dReal nominalcurrent = 0.0;
    dReal maxvelocity = 0.0;
    dReal maxacceleration = 0.0;
    dReal maxjerk = 0.0;
    dReal staticfriction = 0.0;
    dReal viscousfriction = 0.0;
};

class PySensorBase : public PyInterfaceBase
{
protected:
    SensorBasePtr _psensor;
    std::map<SensorBase::SensorType, SensorBase::SensorDataPtr> _mapsensordata;

public:
    class PySensorData
    {
public:
        PySensorData(SensorBase::SensorType type);
        PySensorData(SensorBase::SensorDataPtr pdata);
        virtual ~PySensorData();

        SensorBase::SensorType type;
        uint64_t stamp;
        object transform = py::none_();
    };

    class PyLaserSensorData : public PySensorData
    {
public:
        PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::LaserSensorData> pdata);
        PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom);
        virtual ~PyLaserSensorData();
        object positions = py::none_();
        object ranges = py::none_();
        object intensity = py::none_();
    };

    class PyCameraSensorData : public PySensorData
    {
public:
        PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::CameraSensorData> pdata);
        PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom);
        virtual ~PyCameraSensorData();
        object imagedata = py::none_();
        object KK = py::none_();
        PyCameraIntrinsics intrinsics;
    };

    class PyJointEncoderSensorData : public PySensorData
    {
public:
        PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::JointEncoderSensorData> pdata);
        PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom);
        virtual ~PyJointEncoderSensorData();
        object encoderValues = py::none_();
        object encoderVelocity = py::none_();
        object resolution = py::none_();
    };

    class PyForce6DSensorData : public PySensorData
    {
public:
        PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::Force6DSensorData> pdata);
        PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom);
        virtual ~PyForce6DSensorData();
        object force = py::none_();
        object torque = py::none_();
    };

    class PyIMUSensorData : public PySensorData
    {
public:
        PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::IMUSensorData> pdata);
        PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom);
        virtual ~PyIMUSensorData();
        object rotation = py::none_();
        object angular_velocity = py::none_();
        object linear_acceleration = py::none_();
        object rotation_covariance = py::none_();
        object angular_velocity_covariance = py::none_();
        object linear_acceleration_covariance = py::none_();
    };

    class PyOdometrySensorData : public PySensorData
    {
public:
        PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::OdometrySensorData> pdata);
        PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom);
        virtual ~PyOdometrySensorData();
        object pose = py::none_();
        object linear_velocity = py::none_();
        object angular_velocity = py::none_();
        object pose_covariance = py::none_();
        object velocity_covariance = py::none_();
        std::string targetid;
    };

    class PyTactileSensorData : public PySensorData
    {
public:
        PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::TactileSensorData> pdata);
        PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom);
        virtual ~PyTactileSensorData();
        object forces = py::none_();
        object force_covariance = py::none_();
        object positions = py::none_();
        dReal thickness;
    };

    class PyActuatorSensorData : public PySensorData
    {
public:
        PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::ActuatorSensorData> pdata);
        PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom);
        virtual ~PyActuatorSensorData();
        SensorBase::ActuatorSensorData::ActuatorState state;
        dReal measuredcurrent, measuredtemperature, appliedcurrent;
        dReal maxtorque, maxcurrent, nominalcurrent, maxvelocity, maxacceleration, maxjerk, staticfriction, viscousfriction;
    };

    PySensorBase(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv);
    virtual ~PySensorBase();

    SensorBasePtr GetSensor();
    int Configure(SensorBase::ConfigureCommand command, bool blocking=false);

    bool SimulationStep(dReal timeelapsed);

    OPENRAVE_SHARED_PTR<PySensorGeometry> GetSensorGeometry(SensorBase::SensorType type);

    OPENRAVE_SHARED_PTR<PySensorData> CreateSensorData(SensorBase::SensorType type);

    OPENRAVE_SHARED_PTR<PySensorData> GetSensorData();
    OPENRAVE_SHARED_PTR<PySensorData> GetSensorData(SensorBase::SensorType type);

    OPENRAVE_SHARED_PTR<PySensorData> ConvertToPySensorData(SensorBase::SensorDataPtr psensordata);

    bool Supports(SensorBase::SensorType type);
    void SetSensorGeometry(PySensorGeometryPtr pygeometry);

    void SetTransform(object transform);
    object GetTransform();
    object GetTransformPose();

    object GetName();

    void SetName(const std::string& name);

    virtual std::string __repr__();
    virtual std::string __str__();
    virtual object __unicode__();
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_SENSORBASE_H
