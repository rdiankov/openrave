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
#include "../openravepy_int.h"

namespace openravepy {
using py::object;

class PyCameraIntrinsics
{
public:
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics = geometry::RaveCameraIntrinsics<float>());
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics);

    virtual ~PyCameraIntrinsics();

    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics();
    object K;
    string distortion_model;
    object distortion_coeffs;
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

    std::string hardware_id;
    PyCameraIntrinsics intrinsics;
    int width, height;
    std::string sensor_reference;
    std::string target_region;
    dReal measurement_time;
    dReal gain;
};

class PyLaserGeomData : public PySensorGeometry
{
public:
    PyLaserGeomData();
    PyLaserGeomData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom);
    virtual ~PyLaserGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();
    py::tuple min_angle, max_angle, resolution;
    dReal min_range, max_range, time_increment, time_scan;
};

class PyJointEncoderGeomData : public PySensorGeometry
{
public:
    PyJointEncoderGeomData();
    PyJointEncoderGeomData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom);
    virtual ~PyJointEncoderGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    object resolution;
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

    dReal time_measurement;
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
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    dReal thickness;
};

class PyActuatorGeomData : public PySensorGeometry
{
public:
    PyActuatorGeomData();
    PyActuatorGeomData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom);
    virtual ~PyActuatorGeomData();
    virtual SensorBase::SensorType GetType();
    virtual SensorBase::SensorGeometryPtr GetGeometry();

    dReal maxtorque, maxcurrent, nominalcurrent, maxvelocity, maxacceleration, maxjerk, staticfriction, viscousfriction;
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
        PySensorData(SensorBase::SensorType type) : type(type), stamp(0) {
        }
        PySensorData(SensorBase::SensorDataPtr pdata)
        {
            type = pdata->GetType();
            stamp = pdata->__stamp;
            transform = ReturnTransform(pdata->__trans);
        }
        virtual ~PySensorData() {
        }


        SensorBase::SensorType type;
        uint64_t stamp;
        object transform;
    };

    class PyLaserSensorData : public PySensorData
    {
public:
        PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::LaserSensorData> pdata) : PySensorData(pdata)
        {
            positions = toPyArray3(pdata->positions);
            ranges = toPyArray3(pdata->ranges);
            intensity = toPyArrayN(pdata->intensity.size()>0 ? &pdata->intensity[0] : NULL,pdata->intensity.size());
        }
        PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom) : PySensorData(SensorBase::ST_Laser) {
        }
        virtual ~PyLaserSensorData() {
        }
        object positions, ranges, intensity;
    };

    class PyCameraSensorData : public PySensorData
    {
public:
        PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::CameraSensorData> pdata) : PySensorData(pdata), intrinsics(pgeom->intrinsics)
        {
            if( (int)pdata->vimagedata.size() != pgeom->height*pgeom->width*3 ) {
                throw openrave_exception(_("bad image data"));
            }
            {
                npy_intp dims[] = { pgeom->height,pgeom->width,3};
                PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
                if( pdata->vimagedata.size() > 0 ) {
                    memcpy(PyArray_DATA(pyvalues),&pdata->vimagedata[0],pdata->vimagedata.size());
                }
                imagedata = py::to_array(pyvalues);
            }
        }
        PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom);
        virtual ~PyCameraSensorData();
        object imagedata, KK;
        PyCameraIntrinsics intrinsics;
    };

    class PyJointEncoderSensorData : public PySensorData
    {
public:
        PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::JointEncoderSensorData> pdata) : PySensorData(pdata)
        {
            encoderValues = toPyArray(pdata->encoderValues);
            encoderVelocity = toPyArray(pdata->encoderVelocity);
            resolution = toPyArray(pgeom->resolution);
        }
        PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom) : PySensorData(SensorBase::ST_JointEncoder)
        {
            resolution = toPyArray(pgeom->resolution);
        }
        virtual ~PyJointEncoderSensorData() {
        }
        object encoderValues, encoderVelocity;
        object resolution;
    };

    class PyForce6DSensorData : public PySensorData
    {
public:
        PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::Force6DSensorData> pdata) : PySensorData(pdata)
        {
            force = toPyVector3(pdata->force);
            torque = toPyVector3(pdata->torque);
        }
        PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom) : PySensorData(SensorBase::ST_Force6D)
        {
        }
        virtual ~PyForce6DSensorData() {
        }
        object force, torque;
    };

    class PyIMUSensorData : public PySensorData
    {
public:
        PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::IMUSensorData> pdata);
        PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom);
        virtual ~PyIMUSensorData();
        object rotation, angular_velocity, linear_acceleration, rotation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
    };

    class PyOdometrySensorData : public PySensorData
    {
public:
        PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::OdometrySensorData> pdata);
        PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom);
        virtual ~PyOdometrySensorData();
        object pose, linear_velocity, angular_velocity, pose_covariance, velocity_covariance;
        std::string targetid;
    };

    class PyTactileSensorData : public PySensorData
    {
public:
        PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::TactileSensorData> pdata);
        PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom);
        virtual ~PyTactileSensorData();
        object forces, force_covariance, positions;
        dReal thickness;
    };

    class PyActuatorSensorData : public PySensorData
    {
public:
        PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::ActuatorSensorData> pdata) : PySensorData(pdata)
        {
            state = pdata->state;
            appliedcurrent = pdata->appliedcurrent;
            measuredcurrent = pdata->measuredcurrent;
            measuredtemperature = pdata->measuredtemperature;
            maxtorque = pgeom->maxtorque;
            maxcurrent = pgeom->maxcurrent;
            nominalcurrent = pgeom->nominalcurrent;
            maxvelocity = pgeom->maxvelocity;
            maxacceleration = pgeom->maxacceleration;
            maxjerk = pgeom->maxjerk;
            staticfriction = pgeom->staticfriction;
            viscousfriction = pgeom->viscousfriction;
        }
        PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom) : PySensorData(SensorBase::ST_Actuator)        {
            maxtorque = pgeom->maxtorque;
            maxcurrent = pgeom->maxcurrent;
            nominalcurrent = pgeom->nominalcurrent;
            maxvelocity = pgeom->maxvelocity;
            maxacceleration = pgeom->maxacceleration;
            maxjerk = pgeom->maxjerk;
            staticfriction = pgeom->staticfriction;
            viscousfriction = pgeom->viscousfriction;
        }
        virtual ~PyActuatorSensorData() {
        }
        SensorBase::ActuatorSensorData::ActuatorState state;
        dReal measuredcurrent, measuredtemperature, appliedcurrent;
        dReal maxtorque, maxcurrent, nominalcurrent, maxvelocity, maxacceleration, maxjerk, staticfriction, viscousfriction;
    };

    PySensorBase(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensor, pyenv), _psensor(psensor)
    {
    }
    virtual ~PySensorBase() {
    }

    SensorBasePtr GetSensor() {
        return _psensor;
    }

    int Configure(SensorBase::ConfigureCommand command, bool blocking=false)
    {
        return _psensor->Configure(command,blocking);
    }

    bool SimulationStep(dReal timeelapsed)
    {
        return _psensor->SimulationStep(timeelapsed);
    }

    OPENRAVE_SHARED_PTR<PySensorGeometry> GetSensorGeometry(SensorBase::SensorType type)
    {
        switch(type) {
        case SensorBase::ST_Laser:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyLaserGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Camera:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyCameraGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_JointEncoder:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyJointEncoderGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::JointEncoderGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Force6D:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyForce6DGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::Force6DGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_IMU:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyIMUGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::IMUGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Odometry:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyOdometryGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::OdometryGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Tactile:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyTactileGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::TactileGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Actuator:
            return OPENRAVE_SHARED_PTR<PySensorGeometry>(new PyActuatorGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::ActuatorGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Invalid:
            break;
        }
        throw openrave_exception(boost::str(boost::format(_("unknown sensor data type %d\n"))%type));
    }

    OPENRAVE_SHARED_PTR<PySensorData> CreateSensorData(SensorBase::SensorType type)
    {
        return ConvertToPySensorData(_psensor->CreateSensorData(type));
    }

    OPENRAVE_SHARED_PTR<PySensorData> GetSensorData()
    {
        return GetSensorData(SensorBase::ST_Invalid);
    }
    OPENRAVE_SHARED_PTR<PySensorData> GetSensorData(SensorBase::SensorType type)
    {
        SensorBase::SensorDataPtr psensordata;
        if( _mapsensordata.find(type) == _mapsensordata.end() ) {
            psensordata = _psensor->CreateSensorData(type);
            _mapsensordata[type] = psensordata;
        }
        else {
            psensordata = _mapsensordata[type];
        }
        if( !_psensor->GetSensorData(psensordata) ) {
            throw openrave_exception(_("SensorData failed"));
        }
        return ConvertToPySensorData(psensordata);
    }

    OPENRAVE_SHARED_PTR<PySensorData> ConvertToPySensorData(SensorBase::SensorDataPtr psensordata)
    {
        if( !psensordata ) {
            return OPENRAVE_SHARED_PTR<PySensorData>();
        }
        switch(psensordata->GetType()) {
        case SensorBase::ST_Laser:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyLaserSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserSensorData>(psensordata)));
        case SensorBase::ST_Camera:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyCameraSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraSensorData>(psensordata)));
        case SensorBase::ST_JointEncoder:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyJointEncoderSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::JointEncoderGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::JointEncoderSensorData>(psensordata)));
        case SensorBase::ST_Force6D:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyForce6DSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::Force6DGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::Force6DSensorData>(psensordata)));
        case SensorBase::ST_IMU:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyIMUSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::IMUGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::IMUSensorData>(psensordata)));
        case SensorBase::ST_Odometry:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyOdometrySensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::OdometryGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::OdometrySensorData>(psensordata)));
        case SensorBase::ST_Tactile:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyTactileSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::TactileGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::TactileSensorData>(psensordata)));
        case SensorBase::ST_Actuator:
            return OPENRAVE_SHARED_PTR<PySensorData>(new PyActuatorSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::ActuatorGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::ActuatorSensorData>(psensordata)));
        case SensorBase::ST_Invalid:
            break;
        }
        throw openrave_exception(boost::str(boost::format(_("unknown sensor data type %d\n"))%psensordata->GetType()));
    }

    bool Supports(SensorBase::SensorType type) {
        return _psensor->Supports(type);
    }

    void SetSensorGeometry(PySensorGeometryPtr pygeometry) {
        _psensor->SetSensorGeometry(pygeometry->GetGeometry());
    }

    void SetTransform(object transform) {
        _psensor->SetTransform(ExtractTransform(transform));
    }
    object GetTransform() {
        return ReturnTransform(_psensor->GetTransform());
    }
    object GetTransformPose() {
        return toPyArray(_psensor->GetTransform());
    }

    object GetName() {
        return ConvertStringToUnicode(_psensor->GetName());
    }

    void SetName(const std::string& name)
    {
        return _psensor->SetName(name);
    }

    virtual string __repr__() {
        return boost::str(boost::format("<RaveGetEnvironment(%d).GetSensor('%s')>")%RaveGetEnvironmentId(_psensor->GetEnv())%_psensor->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s>")%RaveGetInterfaceName(_psensor->GetInterfaceType())%_psensor->GetXMLId()%_psensor->GetName());
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_SENSORBASE_H