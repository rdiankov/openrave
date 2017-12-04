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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

class PyCameraIntrinsics
{
public:
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics = geometry::RaveCameraIntrinsics<float>())
    {
        numeric::array arr(boost::python::make_tuple(intrinsics.fx,0,intrinsics.cx,0,intrinsics.fy,intrinsics.cy,0,0,1));
        arr.resize(3,3);
        K = arr;
        distortion_model = intrinsics.distortion_model;
        distortion_coeffs = toPyArray(intrinsics.distortion_coeffs);
        focal_length = intrinsics.focal_length;
    }
    PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics)
    {
        numeric::array arr(boost::python::make_tuple(intrinsics.fx,0,intrinsics.cx,0,intrinsics.fy,intrinsics.cy,0,0,1));
        arr.resize(3,3);
        K = arr;
        distortion_model = intrinsics.distortion_model;
        distortion_coeffs = toPyArray(intrinsics.distortion_coeffs);
        focal_length = intrinsics.focal_length;
    }

    virtual ~PyCameraIntrinsics() {
    }

    virtual SensorBase::CameraIntrinsics GetCameraIntrinsics()
    {
        SensorBase::CameraIntrinsics intrinsics;
        if( IS_PYTHONOBJECT_NONE(K) ) {
            intrinsics.fx = 0;
            intrinsics.fy = 0;
            intrinsics.cx = 0;
            intrinsics.cy = 0;
        }
        else {
            intrinsics.fx = boost::python::extract<dReal>(K[0][0]);
            intrinsics.fy = boost::python::extract<dReal>(K[1][1]);
            intrinsics.cx = boost::python::extract<dReal>(K[0][2]);
            intrinsics.cy = boost::python::extract<dReal>(K[1][2]);
        }
        intrinsics.distortion_model = distortion_model;
        intrinsics.distortion_coeffs = ExtractArray<dReal>(distortion_coeffs);
        intrinsics.focal_length = focal_length;
        return intrinsics;
    }
    object K;
    string distortion_model;
    object distortion_coeffs;
    dReal focal_length;
};

class PyCameraGeomData : public PySensorGeometry
{
public:
    PyCameraGeomData() {
        width = 0;
        height = 0;
        measurement_time = 1;
        gain = 1;
    }
    PyCameraGeomData(boost::shared_ptr<SensorBase::CameraGeomData const> pgeom) : intrinsics(pgeom->intrinsics)
    {
        hardware_id = pgeom->hardware_id;
        width = pgeom->width;
        height = pgeom->height;
        sensor_reference = pgeom->sensor_reference;
        target_region = pgeom->target_region;
        measurement_time = pgeom->measurement_time;
        gain = pgeom->gain;
    }
    virtual ~PyCameraGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Camera;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::CameraGeomData> geom(new SensorBase::CameraGeomData());
        geom->hardware_id = hardware_id;
        geom->width = width;
        geom->height = height;
        geom->intrinsics = intrinsics.GetCameraIntrinsics();
        geom->sensor_reference = sensor_reference;
        geom->target_region = target_region;
        geom->measurement_time = measurement_time;
        geom->gain = gain;
        return geom;
    }

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
    PyLaserGeomData() {
        min_angle = boost::python::make_tuple(0.0, 0.0);
        max_angle = boost::python::make_tuple(0.0, 0.0);
        min_range = 0.0;
        max_range = 0.0;
        time_increment = 0.0;
        time_scan = 0.0;
    }
    PyLaserGeomData(boost::shared_ptr<SensorBase::LaserGeomData const> pgeom)
    {
        min_angle = boost::python::make_tuple(pgeom->min_angle[0], pgeom->min_angle[1]);
        max_angle = boost::python::make_tuple(pgeom->max_angle[0], pgeom->max_angle[1]);
        min_range = pgeom->min_range;
        max_range = pgeom->max_range;
        time_increment = pgeom->time_increment;
        time_scan = pgeom->time_scan;
    }
    virtual ~PyLaserGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Laser;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::LaserGeomData> geom(new SensorBase::LaserGeomData());
        geom->min_angle[0] = (dReal)boost::python::extract<dReal>(min_angle[0]);
        geom->min_angle[1] = (dReal)boost::python::extract<dReal>(min_angle[1]);
        geom->max_angle[0] = (dReal)boost::python::extract<dReal>(max_angle[0]);
        geom->max_angle[1] = (dReal)boost::python::extract<dReal>(max_angle[1]);
        geom->min_range = min_range;
        geom->max_range = max_range;
        geom->time_increment = time_increment;
        geom->time_scan = time_scan;
        return geom;
    }

    boost::python::tuple min_angle, max_angle, resolution;
    dReal min_range, max_range, time_increment, time_scan;
};

class PyJointEncoderGeomData : public PySensorGeometry
{
public:
    PyJointEncoderGeomData() {
        resolution = toPyArray(std::vector<dReal>());
    }
    PyJointEncoderGeomData(boost::shared_ptr<SensorBase::JointEncoderGeomData const> pgeom)
    {
        resolution = toPyArray(pgeom->resolution);
    }
    virtual ~PyJointEncoderGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_JointEncoder;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::JointEncoderGeomData> geom(new SensorBase::JointEncoderGeomData());
        geom->resolution = ExtractArray<dReal>(resolution);
        return geom;
    }

    object resolution;
};

class PyForce6DGeomData : public PySensorGeometry
{
public:
    PyForce6DGeomData() {
    }
    PyForce6DGeomData(boost::shared_ptr<SensorBase::Force6DGeomData const> pgeom)
    {
    }
    virtual ~PyForce6DGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Force6D;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::Force6DGeomData> geom(new SensorBase::Force6DGeomData());
        return geom;
    }
};

class PyIMUGeomData : public PySensorGeometry
{
public:
    PyIMUGeomData() {
        time_measurement = 0.0;
    }
    PyIMUGeomData(boost::shared_ptr<SensorBase::IMUGeomData const> pgeom)
    {
        time_measurement = pgeom->time_measurement;
    }
    virtual ~PyIMUGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_IMU;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::IMUGeomData> geom(new SensorBase::IMUGeomData());
        geom->time_measurement = time_measurement;
        return geom;
    }

    dReal time_measurement;
};

class PyOdometryGeomData : public PySensorGeometry
{
public:
    PyOdometryGeomData() {
    }
    PyOdometryGeomData(boost::shared_ptr<SensorBase::OdometryGeomData const> pgeom)
    {
        targetid = pgeom->targetid;
    }
    virtual ~PyOdometryGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Odometry;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::OdometryGeomData> geom(new SensorBase::OdometryGeomData());
        geom->targetid = targetid;
        return geom;
    }

    std::string targetid;
};

// TODO fill rest of fields
class PyTactileGeomData : public PySensorGeometry
{
public:
    PyTactileGeomData() {
        thickness = 0.0;
    }
    PyTactileGeomData(boost::shared_ptr<SensorBase::TactileGeomData const> pgeom)
    {
        thickness = pgeom->thickness;
    }
    virtual ~PyTactileGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Tactile;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::TactileGeomData> geom(new SensorBase::TactileGeomData());
        geom->thickness = thickness;
        return geom;
    }

    dReal thickness;
};

class PyActuatorGeomData : public PySensorGeometry
{
public:
    PyActuatorGeomData() {
        maxtorque = 0.0;
        maxcurrent = 0.0;
        nominalcurrent = 0.0;
        maxvelocity = 0.0;
        maxacceleration = 0.0;
        maxjerk = 0.0;
        staticfriction = 0.0;
        viscousfriction = 0.0;
    }
    PyActuatorGeomData(boost::shared_ptr<SensorBase::ActuatorGeomData const> pgeom)
    {
        maxtorque = pgeom->maxtorque;
        maxcurrent = pgeom->maxcurrent;
        nominalcurrent = pgeom->nominalcurrent;
        maxvelocity = pgeom->maxvelocity;
        maxacceleration = pgeom->maxacceleration;
        maxjerk = pgeom->maxjerk;
        staticfriction = pgeom->staticfriction;
        viscousfriction = pgeom->viscousfriction;
    }
    virtual ~PyActuatorGeomData() {
    }
    virtual SensorBase::SensorType GetType() {
        return SensorBase::ST_Actuator;
    }
    virtual SensorBase::SensorGeometryPtr GetGeometry() {
        boost::shared_ptr<SensorBase::ActuatorGeomData> geom(new SensorBase::ActuatorGeomData());
        geom->maxtorque = maxtorque;
        geom->maxcurrent = maxcurrent;
        geom->nominalcurrent = nominalcurrent;
        geom->maxvelocity = maxvelocity;
        geom->maxacceleration = maxacceleration;
        geom->maxjerk = maxjerk;
        geom->staticfriction = staticfriction;
        geom->viscousfriction = viscousfriction;
        return geom;
    }

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
        PyLaserSensorData(boost::shared_ptr<SensorBase::LaserGeomData const> pgeom, boost::shared_ptr<SensorBase::LaserSensorData> pdata) : PySensorData(pdata)
        {
            positions = toPyArray3(pdata->positions);
            ranges = toPyArray3(pdata->ranges);
            intensity = toPyArrayN(pdata->intensity.size()>0 ? &pdata->intensity[0] : NULL,pdata->intensity.size());
        }
        PyLaserSensorData(boost::shared_ptr<SensorBase::LaserGeomData const> pgeom) : PySensorData(SensorBase::ST_Laser) {
        }
        virtual ~PyLaserSensorData() {
        }
        object positions, ranges, intensity;
    };

    class PyCameraSensorData : public PySensorData
    {
public:
        PyCameraSensorData(boost::shared_ptr<SensorBase::CameraGeomData const> pgeom, boost::shared_ptr<SensorBase::CameraSensorData> pdata) : PySensorData(pdata), intrinsics(pgeom->intrinsics)
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
                imagedata = static_cast<numeric::array>(handle<>(pyvalues));
            }
        }
        PyCameraSensorData(boost::shared_ptr<SensorBase::CameraGeomData const> pgeom) : PySensorData(SensorBase::ST_Camera), intrinsics(pgeom->intrinsics)
        {
            {
                npy_intp dims[] = { pgeom->height,pgeom->width,3};
                PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
                memset(PyArray_DATA(pyvalues),0,pgeom->height*pgeom->width*3);
                imagedata = static_cast<numeric::array>(handle<>(pyvalues));
            }
            {
                numeric::array arr(boost::python::make_tuple(pgeom->intrinsics.fx,0,pgeom->intrinsics.cx,0,pgeom->intrinsics.fy,pgeom->intrinsics.cy,0,0,1));
                arr.resize(3,3);
                KK = arr;
            }
        }
        virtual ~PyCameraSensorData() {
        }
        object imagedata, KK;
        PyCameraIntrinsics intrinsics;
    };

    class PyJointEncoderSensorData : public PySensorData
    {
public:
        PyJointEncoderSensorData(boost::shared_ptr<SensorBase::JointEncoderGeomData const> pgeom, boost::shared_ptr<SensorBase::JointEncoderSensorData> pdata) : PySensorData(pdata)
        {
            encoderValues = toPyArray(pdata->encoderValues);
            encoderVelocity = toPyArray(pdata->encoderVelocity);
            resolution = toPyArray(pgeom->resolution);
        }
        PyJointEncoderSensorData(boost::shared_ptr<SensorBase::JointEncoderGeomData const> pgeom) : PySensorData(SensorBase::ST_JointEncoder)
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
        PyForce6DSensorData(boost::shared_ptr<SensorBase::Force6DGeomData const> pgeom, boost::shared_ptr<SensorBase::Force6DSensorData> pdata) : PySensorData(pdata)
        {
            force = toPyVector3(pdata->force);
            torque = toPyVector3(pdata->torque);
        }
        PyForce6DSensorData(boost::shared_ptr<SensorBase::Force6DGeomData const> pgeom) : PySensorData(SensorBase::ST_Force6D)
        {
        }
        virtual ~PyForce6DSensorData() {
        }
        object force, torque;
    };

    class PyIMUSensorData : public PySensorData
    {
public:
        PyIMUSensorData(boost::shared_ptr<SensorBase::IMUGeomData const> pgeom, boost::shared_ptr<SensorBase::IMUSensorData> pdata) : PySensorData(pdata)
        {
            rotation = toPyVector4(pdata->rotation);
            angular_velocity = toPyVector3(pdata->angular_velocity);
            linear_acceleration = toPyVector3(pdata->linear_acceleration);
            numeric::array arr = toPyArrayN(&pdata->rotation_covariance[0],pdata->rotation_covariance.size());
            arr.resize(3,3);
            rotation_covariance = arr;
            arr = toPyArrayN(&pdata->angular_velocity_covariance[0],pdata->angular_velocity_covariance.size());
            arr.resize(3,3);
            angular_velocity_covariance = arr;
            arr = toPyArrayN(&pdata->linear_acceleration_covariance[0],pdata->linear_acceleration_covariance.size());
            arr.resize(3,3);
            linear_acceleration_covariance = arr;
        }
        PyIMUSensorData(boost::shared_ptr<SensorBase::IMUGeomData const> pgeom) : PySensorData(SensorBase::ST_IMU)
        {
        }
        virtual ~PyIMUSensorData() {
        }
        object rotation, angular_velocity, linear_acceleration, rotation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
    };

    class PyOdometrySensorData : public PySensorData
    {
public:
        PyOdometrySensorData(boost::shared_ptr<SensorBase::OdometryGeomData const> pgeom, boost::shared_ptr<SensorBase::OdometrySensorData> pdata) : PySensorData(pdata)
        {
            pose = toPyArray(pdata->pose);
            linear_velocity = toPyVector3(pdata->linear_velocity);
            angular_velocity = toPyVector3(pdata->angular_velocity);
            numeric::array arr = toPyArrayN(&pdata->pose_covariance[0],pdata->pose_covariance.size());
            arr.resize(3,3);
            pose_covariance = arr;
            arr = toPyArrayN(&pdata->velocity_covariance[0],pdata->velocity_covariance.size());
            arr.resize(3,3);
            velocity_covariance = arr;
            targetid = pgeom->targetid;

        }
        PyOdometrySensorData(boost::shared_ptr<SensorBase::OdometryGeomData const> pgeom) : PySensorData(SensorBase::ST_Odometry)
        {
            targetid = pgeom->targetid;
        }
        virtual ~PyOdometrySensorData() {
        }
        object pose, linear_velocity, angular_velocity, pose_covariance, velocity_covariance;
        std::string targetid;
    };

    class PyTactileSensorData : public PySensorData
    {
public:
        PyTactileSensorData(boost::shared_ptr<SensorBase::TactileGeomData const> pgeom, boost::shared_ptr<SensorBase::TactileSensorData> pdata) : PySensorData(pdata)
        {
            forces = toPyArray3(pdata->forces);
            numeric::array arr = toPyArrayN(&pdata->force_covariance[0],pdata->force_covariance.size());
            arr.resize(3,3);
            force_covariance = arr;
            positions = toPyArray3(pgeom->positions);
            thickness = pgeom->thickness;
        }
        PyTactileSensorData(boost::shared_ptr<SensorBase::TactileGeomData const> pgeom) : PySensorData(SensorBase::ST_Tactile)
        {
            positions = toPyArray3(pgeom->positions);
            thickness = pgeom->thickness;
        }
        virtual ~PyTactileSensorData() {
        }
        object forces, force_covariance, positions;
        dReal thickness;
    };

    class PyActuatorSensorData : public PySensorData
    {
public:
        PyActuatorSensorData(boost::shared_ptr<SensorBase::ActuatorGeomData const> pgeom, boost::shared_ptr<SensorBase::ActuatorSensorData> pdata) : PySensorData(pdata)
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
        PyActuatorSensorData(boost::shared_ptr<SensorBase::ActuatorGeomData const> pgeom) : PySensorData(SensorBase::ST_Actuator)        {
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

    boost::shared_ptr<PySensorGeometry> GetSensorGeometry(SensorBase::SensorType type)
    {
        switch(type) {
        case SensorBase::ST_Laser:
            return boost::shared_ptr<PySensorGeometry>(new PyLaserGeomData(boost::static_pointer_cast<SensorBase::LaserGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Camera:
            return boost::shared_ptr<PySensorGeometry>(new PyCameraGeomData(boost::static_pointer_cast<SensorBase::CameraGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_JointEncoder:
            return boost::shared_ptr<PySensorGeometry>(new PyJointEncoderGeomData(boost::static_pointer_cast<SensorBase::JointEncoderGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Force6D:
            return boost::shared_ptr<PySensorGeometry>(new PyForce6DGeomData(boost::static_pointer_cast<SensorBase::Force6DGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_IMU:
            return boost::shared_ptr<PySensorGeometry>(new PyIMUGeomData(boost::static_pointer_cast<SensorBase::IMUGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Odometry:
            return boost::shared_ptr<PySensorGeometry>(new PyOdometryGeomData(boost::static_pointer_cast<SensorBase::OdometryGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Tactile:
            return boost::shared_ptr<PySensorGeometry>(new PyTactileGeomData(boost::static_pointer_cast<SensorBase::TactileGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Actuator:
            return boost::shared_ptr<PySensorGeometry>(new PyActuatorGeomData(boost::static_pointer_cast<SensorBase::ActuatorGeomData const>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Invalid:
            break;
        }
        throw openrave_exception(boost::str(boost::format(_("unknown sensor data type %d\n"))%type));
    }

    boost::shared_ptr<PySensorData> CreateSensorData(SensorBase::SensorType type)
    {
        return ConvertToPySensorData(_psensor->CreateSensorData(type));
    }

    boost::shared_ptr<PySensorData> GetSensorData()
    {
        return GetSensorData(SensorBase::ST_Invalid);
    }
    boost::shared_ptr<PySensorData> GetSensorData(SensorBase::SensorType type)
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

    boost::shared_ptr<PySensorData> ConvertToPySensorData(SensorBase::SensorDataPtr psensordata)
    {
        if( !psensordata ) {
            return boost::shared_ptr<PySensorData>();
        }
        switch(psensordata->GetType()) {
        case SensorBase::ST_Laser:
            return boost::shared_ptr<PySensorData>(new PyLaserSensorData(boost::static_pointer_cast<SensorBase::LaserGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::LaserSensorData>(psensordata)));
        case SensorBase::ST_Camera:
            return boost::shared_ptr<PySensorData>(new PyCameraSensorData(boost::static_pointer_cast<SensorBase::CameraGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::CameraSensorData>(psensordata)));
        case SensorBase::ST_JointEncoder:
            return boost::shared_ptr<PySensorData>(new PyJointEncoderSensorData(boost::static_pointer_cast<SensorBase::JointEncoderGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::JointEncoderSensorData>(psensordata)));
        case SensorBase::ST_Force6D:
            return boost::shared_ptr<PySensorData>(new PyForce6DSensorData(boost::static_pointer_cast<SensorBase::Force6DGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::Force6DSensorData>(psensordata)));
        case SensorBase::ST_IMU:
            return boost::shared_ptr<PySensorData>(new PyIMUSensorData(boost::static_pointer_cast<SensorBase::IMUGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::IMUSensorData>(psensordata)));
        case SensorBase::ST_Odometry:
            return boost::shared_ptr<PySensorData>(new PyOdometrySensorData(boost::static_pointer_cast<SensorBase::OdometryGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::OdometrySensorData>(psensordata)));
        case SensorBase::ST_Tactile:
            return boost::shared_ptr<PySensorData>(new PyTactileSensorData(boost::static_pointer_cast<SensorBase::TactileGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::TactileSensorData>(psensordata)));
        case SensorBase::ST_Actuator:
            return boost::shared_ptr<PySensorData>(new PyActuatorSensorData(boost::static_pointer_cast<SensorBase::ActuatorGeomData const>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::ActuatorSensorData>(psensordata)));
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

SensorBasePtr GetSensor(PySensorBasePtr pysensor)
{
    return !pysensor ? SensorBasePtr() : pysensor->GetSensor();
}

PyInterfaceBasePtr toPySensor(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv)
{
    return !psensor ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PySensorBase(psensor,pyenv));
}

object toPySensorData(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv)
{
    if( !psensor ) {
        return object();
    }
    return object(PySensorBase(psensor,pyenv).GetSensorData());
}

PySensorBasePtr RaveCreateSensor(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SensorBasePtr p = OpenRAVE::RaveCreateSensor(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySensorBasePtr();
    }
    return PySensorBasePtr(new PySensorBase(p,pyenv));
}

PySensorGeometryPtr toPySensorGeometry(SensorBase::SensorGeometryPtr pgeom)
{
    if( !!pgeom ) {
        if( pgeom->GetType() == SensorBase::ST_Camera ) {
            return PySensorGeometryPtr(new PyCameraGeomData(boost::static_pointer_cast<SensorBase::CameraGeomData const>(pgeom)));
        }
        else if( pgeom->GetType() == SensorBase::ST_Laser ) {
            return PySensorGeometryPtr(new PyLaserGeomData(boost::static_pointer_cast<SensorBase::LaserGeomData const>(pgeom)));
        }

    }
    return PySensorGeometryPtr();
}

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics)
{
    return PyCameraIntrinsicsPtr(new PyCameraIntrinsics(intrinsics));
}

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics)
{
    return PyCameraIntrinsicsPtr(new PyCameraIntrinsics(intrinsics));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Configure_overloads, Configure, 1, 2)

void init_openravepy_sensor()
{
    {
        boost::shared_ptr<PySensorBase::PySensorData> (PySensorBase::*GetSensorData1)() = &PySensorBase::GetSensorData;
        boost::shared_ptr<PySensorBase::PySensorData> (PySensorBase::*GetSensorData2)(SensorBase::SensorType) = &PySensorBase::GetSensorData;
        scope sensor = class_<PySensorBase, boost::shared_ptr<PySensorBase>, bases<PyInterfaceBase> >("Sensor", DOXY_CLASS(SensorBase), no_init)
                       .def("Configure",&PySensorBase::Configure, Configure_overloads(args("command","blocking"), DOXY_FN(SensorBase,Configure)))
                       .def("SimulationStep",&PySensorBase::SimulationStep, args("timeelapsed"), DOXY_FN(SensorBase,SimulationStep))
                       .def("GetSensorData",GetSensorData1, DOXY_FN(SensorBase,GetSensorData))
                       .def("GetSensorData",GetSensorData2, DOXY_FN(SensorBase,GetSensorData))
                       .def("CreateSensorData",&PySensorBase::CreateSensorData, DOXY_FN(SensorBase,CreateSensorData))
                       .def("GetSensorGeometry",&PySensorBase::GetSensorGeometry,DOXY_FN(SensorBase,GetSensorGeometry))
                       .def("SetSensorGeometry",&PySensorBase::SetSensorGeometry, DOXY_FN(SensorBase,SetSensorGeometry))
                       .def("SetTransform",&PySensorBase::SetTransform, DOXY_FN(SensorBase,SetTransform))
                       .def("GetTransform",&PySensorBase::GetTransform, DOXY_FN(SensorBase,GetTransform))
                       .def("GetTransformPose",&PySensorBase::GetTransformPose, DOXY_FN(SensorBase,GetTransform))
                       .def("GetName",&PySensorBase::GetName, DOXY_FN(SensorBase,GetName))
                       .def("SetName",&PySensorBase::SetName, DOXY_FN(SensorBase,SetName))
                       .def("Supports",&PySensorBase::Supports, DOXY_FN(SensorBase,Supports))
                       .def("__str__",&PySensorBase::__str__)
                       .def("__unicode__",&PySensorBase::__unicode__)
                       .def("__repr__",&PySensorBase::__repr__)
        ;

        class_<PyCameraIntrinsics, boost::shared_ptr<PyCameraIntrinsics> >("CameraIntrinsics", DOXY_CLASS(geometry::RaveCameraIntrinsics))
        .def_readwrite("K",&PyCameraIntrinsics::K)
        .def_readwrite("distortion_model",&PyCameraIntrinsics::distortion_model)
        .def_readwrite("distortion_coeffs",&PyCameraIntrinsics::distortion_coeffs)
        .def_readwrite("focal_length",&PyCameraIntrinsics::focal_length)
        ;

        class_<PySensorBase::PySensorData, boost::shared_ptr<PySensorBase::PySensorData> >("SensorData", DOXY_CLASS(SensorBase::SensorData),no_init)
        .def_readonly("type",&PySensorBase::PySensorData::type)
        .def_readonly("stamp",&PySensorBase::PySensorData::stamp)
        ;
        class_<PySensorBase::PyLaserSensorData, boost::shared_ptr<PySensorBase::PyLaserSensorData>, bases<PySensorBase::PySensorData> >("LaserSensorData", DOXY_CLASS(SensorBase::LaserSensorData),no_init)
        .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
        .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
        .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
        ;
        class_<PySensorBase::PyCameraSensorData, boost::shared_ptr<PySensorBase::PyCameraSensorData>, bases<PySensorBase::PySensorData> >("CameraSensorData", DOXY_CLASS(SensorBase::CameraSensorData),no_init)
        .def_readonly("transform",&PySensorBase::PyCameraSensorData::transform)
        .def_readonly("imagedata",&PySensorBase::PyCameraSensorData::imagedata)
        .def_readonly("KK",&PySensorBase::PyCameraSensorData::KK)
        .def_readonly("intrinsics",&PySensorBase::PyCameraSensorData::intrinsics)
        ;
        class_<PySensorBase::PyJointEncoderSensorData, boost::shared_ptr<PySensorBase::PyJointEncoderSensorData>, bases<PySensorBase::PySensorData> >("JointEncoderSensorData", DOXY_CLASS(SensorBase::JointEncoderSensorData),no_init)
        .def_readonly("encoderValues",&PySensorBase::PyJointEncoderSensorData::encoderValues)
        .def_readonly("encoderVelocity",&PySensorBase::PyJointEncoderSensorData::encoderVelocity)
        .def_readonly("resolution",&PySensorBase::PyJointEncoderSensorData::resolution)
        ;
        class_<PySensorBase::PyForce6DSensorData, boost::shared_ptr<PySensorBase::PyForce6DSensorData>, bases<PySensorBase::PySensorData> >("Force6DSensorData", DOXY_CLASS(SensorBase::Force6DSensorData),no_init)
        .def_readonly("force",&PySensorBase::PyForce6DSensorData::force)
        .def_readonly("torque",&PySensorBase::PyForce6DSensorData::torque)
        ;
        class_<PySensorBase::PyIMUSensorData, boost::shared_ptr<PySensorBase::PyIMUSensorData>, bases<PySensorBase::PySensorData> >("IMUSensorData", DOXY_CLASS(SensorBase::IMUSensorData),no_init)
        .def_readonly("rotation",&PySensorBase::PyIMUSensorData::rotation)
        .def_readonly("angular_velocity",&PySensorBase::PyIMUSensorData::angular_velocity)
        .def_readonly("linear_acceleration",&PySensorBase::PyIMUSensorData::linear_acceleration)
        .def_readonly("rotation_covariance",&PySensorBase::PyIMUSensorData::rotation_covariance)
        .def_readonly("angular_velocity_covariance",&PySensorBase::PyIMUSensorData::angular_velocity_covariance)
        .def_readonly("linear_acceleration_covariance",&PySensorBase::PyIMUSensorData::linear_acceleration_covariance)
        ;
        class_<PySensorBase::PyOdometrySensorData, boost::shared_ptr<PySensorBase::PyOdometrySensorData>, bases<PySensorBase::PySensorData> >("OdometrySensorData", DOXY_CLASS(SensorBase::OdometrySensorData),no_init)
        .def_readonly("pose",&PySensorBase::PyOdometrySensorData::pose)
        .def_readonly("linear_velocity",&PySensorBase::PyOdometrySensorData::linear_velocity)
        .def_readonly("angular_velocity",&PySensorBase::PyOdometrySensorData::angular_velocity)
        .def_readonly("pose_covariance",&PySensorBase::PyOdometrySensorData::pose_covariance)
        .def_readonly("velocity_covariance",&PySensorBase::PyOdometrySensorData::velocity_covariance)
        .def_readonly("targetid",&PySensorBase::PyOdometrySensorData::targetid)
        ;
        class_<PySensorBase::PyTactileSensorData, boost::shared_ptr<PySensorBase::PyTactileSensorData>, bases<PySensorBase::PySensorData> >("TactileSensorData", DOXY_CLASS(SensorBase::TactileSensorData),no_init)
        .def_readonly("forces",&PySensorBase::PyTactileSensorData::forces)
        .def_readonly("force_covariance",&PySensorBase::PyTactileSensorData::force_covariance)
        .def_readonly("positions",&PySensorBase::PyTactileSensorData::positions)
        .def_readonly("thickness",&PySensorBase::PyTactileSensorData::thickness)
        ;
        {
            class_<PySensorBase::PyActuatorSensorData, boost::shared_ptr<PySensorBase::PyActuatorSensorData>, bases<PySensorBase::PySensorData> >("ActuatorSensorData", DOXY_CLASS(SensorBase::ActuatorSensorData),no_init)
            .def_readonly("state",&PySensorBase::PyActuatorSensorData::state)
            .def_readonly("measuredcurrent",&PySensorBase::PyActuatorSensorData::measuredcurrent)
            .def_readonly("measuredtemperature",&PySensorBase::PyActuatorSensorData::measuredtemperature)
            .def_readonly("appliedcurrent",&PySensorBase::PyActuatorSensorData::appliedcurrent)
            .def_readonly("maxtorque",&PySensorBase::PyActuatorSensorData::maxtorque)
            .def_readonly("maxcurrent",&PySensorBase::PyActuatorSensorData::maxcurrent)
            .def_readonly("nominalcurrent",&PySensorBase::PyActuatorSensorData::nominalcurrent)
            .def_readonly("maxvelocity",&PySensorBase::PyActuatorSensorData::maxvelocity)
            .def_readonly("maxacceleration",&PySensorBase::PyActuatorSensorData::maxacceleration)
            .def_readonly("maxjerk",&PySensorBase::PyActuatorSensorData::maxjerk)
            .def_readonly("staticfriction",&PySensorBase::PyActuatorSensorData::staticfriction)
            .def_readonly("viscousfriction",&PySensorBase::PyActuatorSensorData::viscousfriction)
            ;
            enum_<SensorBase::ActuatorSensorData::ActuatorState>("ActuatorState" DOXY_ENUM(ActuatorState))
            .value("Undefined",SensorBase::ActuatorSensorData::AS_Undefined)
            .value("Idle",SensorBase::ActuatorSensorData::AS_Idle)
            .value("Moving",SensorBase::ActuatorSensorData::AS_Moving)
            .value("Stalled",SensorBase::ActuatorSensorData::AS_Stalled)
            .value("Braked",SensorBase::ActuatorSensorData::AS_Braked)
            ;
        }

        enum_<SensorBase::SensorType>("Type" DOXY_ENUM(SensorType))
        .value("Invalid",SensorBase::ST_Invalid)
        .value("Laser",SensorBase::ST_Laser)
        .value("Camera",SensorBase::ST_Camera)
        .value("JointEncoder",SensorBase::ST_JointEncoder)
        .value("Force6D",SensorBase::ST_Force6D)
        .value("IMU",SensorBase::ST_IMU)
        .value("Odometry",SensorBase::ST_Odometry)
        .value("Tactile",SensorBase::ST_Tactile)
        .value("Actuator",SensorBase::ST_Actuator)
        ;
        enum_<SensorBase::ConfigureCommand>("ConfigureCommand" DOXY_ENUM(ConfigureCommand))
        .value("PowerOn",SensorBase::CC_PowerOn)
        .value("PowerOff",SensorBase::CC_PowerOff)
        .value("PowerCheck",SensorBase::CC_PowerCheck)
        .value("RenderDataOn",SensorBase::CC_RenderDataOn)
        .value("RenderDataOff",SensorBase::CC_RenderDataOff)
        .value("RenderDataCheck",SensorBase::CC_RenderDataCheck)
        .value("RenderGeometryOn",SensorBase::CC_RenderGeometryOn)
        .value("RenderGeometryOff",SensorBase::CC_RenderGeometryOff)
        .value("RenderGeometryCheck",SensorBase::CC_RenderGeometryCheck)
        ;
    }

    class_<PySensorGeometry, boost::shared_ptr<PySensorGeometry>, boost::noncopyable >("SensorGeometry", DOXY_CLASS(PySensorGeometry),no_init)
    .def("GetType",boost::python::pure_virtual(&PySensorGeometry::GetType))
    ;
    class_<PyCameraGeomData, boost::shared_ptr<PyCameraGeomData>, bases<PySensorGeometry> >("CameraGeomData", DOXY_CLASS(SensorBase::CameraGeomData))
    .def_readwrite("intrinsics",&PyCameraGeomData::intrinsics)
    .def_readwrite("hardware_id",&PyCameraGeomData::hardware_id)
    .def_readwrite("width",&PyCameraGeomData::width)
    .def_readwrite("height",&PyCameraGeomData::height)
    .def_readwrite("sensor_reference",&PyCameraGeomData::sensor_reference)
    .def_readwrite("target_region",&PyCameraGeomData::target_region)
    .def_readwrite("measurement_time",&PyCameraGeomData::measurement_time)
    .def_readwrite("gain",&PyCameraGeomData::gain)
    .def_readwrite("KK",&PyCameraGeomData::intrinsics) // deprecated
    ;

    class_<PyLaserGeomData, boost::shared_ptr<PyLaserGeomData>, bases<PySensorGeometry> >("LaserGeomData", DOXY_CLASS(SensorBase::LaserGeomData))
    .def_readwrite("min_angle",&PyLaserGeomData::min_angle)
    .def_readwrite("max_angle",&PyLaserGeomData::max_angle)
    .def_readwrite("min_range",&PyLaserGeomData::min_range)
    .def_readwrite("max_range",&PyLaserGeomData::max_range)
    .def_readwrite("time_increment",&PyLaserGeomData::time_increment)
    .def_readwrite("time_scan",&PyLaserGeomData::time_scan)
    ;

    class_<PyJointEncoderGeomData, boost::shared_ptr<PyJointEncoderGeomData>, bases<PySensorGeometry> >("JointEncoderGeomData", DOXY_CLASS(SensorBase::JointEncoderGeomData))
    .def_readwrite("resolution",&PyJointEncoderGeomData::resolution)
    ;

    class_<PyForce6DGeomData, boost::shared_ptr<PyForce6DGeomData>, bases<PySensorGeometry> >("Force6DGeomData", DOXY_CLASS(SensorBase::Force6DGeomData))
    ;

    class_<PyIMUGeomData, boost::shared_ptr<PyIMUGeomData>, bases<PySensorGeometry> >("IMUGeomData", DOXY_CLASS(SensorBase::IMUGeomData))
    .def_readwrite("time_measurement",&PyIMUGeomData::time_measurement)
    ;

    class_<PyOdometryGeomData, boost::shared_ptr<PyOdometryGeomData>, bases<PySensorGeometry> >("OdometryGeomData", DOXY_CLASS(SensorBase::OdometryGeomData))
    .def_readwrite("targetid",&PyOdometryGeomData::targetid)
    ;

    class_<PyTactileGeomData, boost::shared_ptr<PyTactileGeomData>, bases<PySensorGeometry> >("TactileGeomData", DOXY_CLASS(SensorBase::TactileGeomData))
    .def_readwrite("thickness",&PyTactileGeomData::thickness)
    ;

    class_<PyActuatorGeomData, boost::shared_ptr<PyActuatorGeomData>, bases<PySensorGeometry> >("ActuatorGeomData", DOXY_CLASS(SensorBase::ActuatorGeomData))
    .def_readwrite("maxtorque",&PyActuatorGeomData::maxtorque)
    .def_readwrite("maxcurrent",&PyActuatorGeomData::maxcurrent)
    .def_readwrite("nominalcurrent",&PyActuatorGeomData::nominalcurrent)
    .def_readwrite("maxvelocity",&PyActuatorGeomData::maxvelocity)
    .def_readwrite("maxacceleration",&PyActuatorGeomData::maxacceleration)
    .def_readwrite("maxjerk",&PyActuatorGeomData::maxjerk)
    .def_readwrite("staticfriction",&PyActuatorGeomData::staticfriction)
    .def_readwrite("viscousfriction",&PyActuatorGeomData::viscousfriction)
    ;

    def("RaveCreateSensor",openravepy::RaveCreateSensor,args("env","name"),DOXY_FN1(RaveCreateSensor));
}

}
