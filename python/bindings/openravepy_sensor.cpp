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
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_sensorbase.h>

namespace openravepy {

using py::object;
using py::extract;
using py::extract_;
using py::handle;
using py::dict;
using py::enum_;
using py::class_;
using py::init;
using py::scope_; // py::object if USE_PYBIND11_PYTHON_BINDINGS
using py::scope;
using py::args;
using py::return_value_policy;

#ifndef USE_PYBIND11_PYTHON_BINDINGS
using py::no_init;
using py::bases;
using py::copy_const_reference;
using py::docstring_options;
using py::pickle_suite;
using py::manage_new_object;
using py::def;
#endif // USE_PYBIND11_PYTHON_BINDINGS

namespace numeric = py::numeric;

inline void resize_3x3(numeric::array& arr) {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    arr.resize({3, 3});
#else
    arr.resize(3, 3);
#endif
}

PyCameraIntrinsics::PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>& intrinsics)
{
    numeric::array arr(py::make_tuple(intrinsics.fx,0,intrinsics.cx,0,intrinsics.fy,intrinsics.cy,0,0,1));
    resize_3x3(arr);
    K = arr;
    distortion_model = intrinsics.distortion_model;
    distortion_coeffs = toPyArray(intrinsics.distortion_coeffs);
    focal_length = intrinsics.focal_length;
}

PyCameraIntrinsics::PyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>& intrinsics)
{
    numeric::array arr(py::make_tuple(intrinsics.fx,0,intrinsics.cx,0,intrinsics.fy,intrinsics.cy,0,0,1));
    resize_3x3(arr);
    K = arr;
    distortion_model = intrinsics.distortion_model;
    distortion_coeffs = toPyArray(intrinsics.distortion_coeffs);
    focal_length = intrinsics.focal_length;
}

PyCameraIntrinsics::~PyCameraIntrinsics() {
}

SensorBase::CameraIntrinsics PyCameraIntrinsics::GetCameraIntrinsics()
{
    SensorBase::CameraIntrinsics intrinsics;
    if( IS_PYTHONOBJECT_NONE(K) ) {
        intrinsics.fx = 0;
        intrinsics.fy = 0;
        intrinsics.cx = 0;
        intrinsics.cy = 0;
    }
    else {
        intrinsics.fx = py::extract<dReal>(K[0][0]);
        intrinsics.fy = py::extract<dReal>(K[1][1]);
        intrinsics.cx = py::extract<dReal>(K[0][2]);
        intrinsics.cy = py::extract<dReal>(K[1][2]);
    }
    intrinsics.distortion_model = distortion_model;
    intrinsics.distortion_coeffs = ExtractArray<dReal>(distortion_coeffs);
    intrinsics.focal_length = focal_length;
    return intrinsics;
}

PyCameraGeomData::PyCameraGeomData() {
}
PyCameraGeomData::PyCameraGeomData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom)
{
    _Update(pgeom);
}
PyCameraGeomData::~PyCameraGeomData() {
}
SensorBase::SensorType PyCameraGeomData::GetType() {
    return SensorBase::ST_Camera;
}

object PyCameraGeomData::SerializeJSON(dReal fUnitScale, object options) {
    rapidjson::Document doc;
    SensorBase::SensorGeometryPtr pgeom = GetGeometry();
    pgeom->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, pyGetIntFromPy(options, 0));
    return toPyObject(doc);
}

void PyCameraGeomData::DeserializeJSON(object obj, dReal fUnitScale) {
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    SensorBase::CameraGeomDataPtr pgeom(new SensorBase::CameraGeomData());
    pgeom->DeserializeJSON(doc, fUnitScale);
    _Update(pgeom);
}

void PyCameraGeomData::_Update(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom)
{
    intrinsics = pgeom->intrinsics;
    hardware_id = pgeom->hardware_id;
    width = pgeom->width;
    height = pgeom->height;
    sensor_reference = pgeom->sensor_reference;
    target_region = pgeom->target_region;
    measurement_time = pgeom->measurement_time;
    gain = pgeom->gain;
}

SensorBase::SensorGeometryPtr PyCameraGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData> geom(new SensorBase::CameraGeomData());
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

PyLaserGeomData::PyLaserGeomData() {
}
PyLaserGeomData::PyLaserGeomData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom)
{
    min_angle = py::make_tuple(pgeom->min_angle[0], pgeom->min_angle[1]);
    max_angle = py::make_tuple(pgeom->max_angle[0], pgeom->max_angle[1]);
    min_range = pgeom->min_range;
    max_range = pgeom->max_range;
    time_increment = pgeom->time_increment;
    time_scan = pgeom->time_scan;
}
PyLaserGeomData::~PyLaserGeomData() {
}
SensorBase::SensorType PyLaserGeomData::GetType() {
    return SensorBase::ST_Laser;
}
SensorBase::SensorGeometryPtr PyLaserGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData> geom(new SensorBase::LaserGeomData());
    geom->min_angle[0] = (dReal)py::extract<dReal>(min_angle[0]);
    geom->min_angle[1] = (dReal)py::extract<dReal>(min_angle[1]);
    geom->max_angle[0] = (dReal)py::extract<dReal>(max_angle[0]);
    geom->max_angle[1] = (dReal)py::extract<dReal>(max_angle[1]);
    geom->min_range = min_range;
    geom->max_range = max_range;
    geom->time_increment = time_increment;
    geom->time_scan = time_scan;
    return geom;
}

PyJointEncoderGeomData::PyJointEncoderGeomData() {
    resolution = toPyArray(std::vector<dReal>());
}
PyJointEncoderGeomData::PyJointEncoderGeomData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom)
{
    resolution = toPyArray(pgeom->resolution);
}
PyJointEncoderGeomData::~PyJointEncoderGeomData() {
}
SensorBase::SensorType PyJointEncoderGeomData::GetType() {
    return SensorBase::ST_JointEncoder;
}
SensorBase::SensorGeometryPtr PyJointEncoderGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData> geom(new SensorBase::JointEncoderGeomData());
    geom->resolution = ExtractArray<dReal>(resolution);
    return geom;
}

PyForce6DGeomData::PyForce6DGeomData() {
}
PyForce6DGeomData::PyForce6DGeomData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom)
{
}
PyForce6DGeomData::~PyForce6DGeomData() {
}
SensorBase::SensorType PyForce6DGeomData::GetType() {
    return SensorBase::ST_Force6D;
}
SensorBase::SensorGeometryPtr PyForce6DGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData> geom(new SensorBase::Force6DGeomData());
    return geom;
}

PyIMUGeomData::PyIMUGeomData() {
}
PyIMUGeomData::PyIMUGeomData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom)
{
    time_measurement = pgeom->time_measurement;
}
PyIMUGeomData::~PyIMUGeomData() {
}
SensorBase::SensorType PyIMUGeomData::GetType() {
    return SensorBase::ST_IMU;
}
SensorBase::SensorGeometryPtr PyIMUGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData> geom(new SensorBase::IMUGeomData());
    geom->time_measurement = time_measurement;
    return geom;
}

PyOdometryGeomData::PyOdometryGeomData() {
}
PyOdometryGeomData::PyOdometryGeomData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom)
{
    targetid = pgeom->targetid;
}
PyOdometryGeomData::~PyOdometryGeomData() {
}
SensorBase::SensorType PyOdometryGeomData::GetType() {
    return SensorBase::ST_Odometry;
}
SensorBase::SensorGeometryPtr PyOdometryGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData> geom(new SensorBase::OdometryGeomData());
    geom->targetid = targetid;
    return geom;
}

// TODO fill rest of fields
PyTactileGeomData::PyTactileGeomData() {
}
PyTactileGeomData::PyTactileGeomData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom)
{
    thickness = pgeom->thickness;
}
PyTactileGeomData::~PyTactileGeomData() {
}
SensorBase::SensorType PyTactileGeomData::GetType() {
    return SensorBase::ST_Tactile;
}
SensorBase::SensorGeometryPtr PyTactileGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData> geom(new SensorBase::TactileGeomData());
    geom->thickness = thickness;
    return geom;
}

PyActuatorGeomData::PyActuatorGeomData() {
}
PyActuatorGeomData::PyActuatorGeomData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom)
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
PyActuatorGeomData::~PyActuatorGeomData() {
}
SensorBase::SensorType PyActuatorGeomData::GetType() {
    return SensorBase::ST_Actuator;
}
SensorBase::SensorGeometryPtr PyActuatorGeomData::GetGeometry() {
    OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData> geom(new SensorBase::ActuatorGeomData());
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


PySensorBase::PySensorData::PySensorData(SensorBase::SensorType type) : type(type), stamp(0) {
}
PySensorBase::PySensorData::PySensorData(SensorBase::SensorDataPtr pdata)
{
    type = pdata->GetType();
    stamp = pdata->__stamp;
    transform = ReturnTransform(pdata->__trans);
}
PySensorBase::PySensorData::~PySensorData() {
}

PySensorBase::PyLaserSensorData::PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::LaserSensorData> pdata) : PySensorData(pdata)
{
    positions = toPyArray3(pdata->positions);
    ranges = toPyArray3(pdata->ranges);
    intensity = toPyArrayN(pdata->intensity.data(), pdata->intensity.size());
}
PySensorBase::PyLaserSensorData::PyLaserSensorData(OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const> pgeom) : PySensorData(SensorBase::ST_Laser) {
}
PySensorBase::PyLaserSensorData::~PyLaserSensorData() {
}

PySensorBase::PyCameraSensorData::PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::CameraSensorData> pdata) : PySensorData(pdata), intrinsics(pgeom->intrinsics)
{
    const std::vector<uint8_t>& vimagedata = pdata->vimagedata;
    const size_t numel = pgeom->height * pgeom->width * 3;
    if( vimagedata.size() != numel ) {
        throw openrave_exception(_("bad image data"));
    }

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        py::array_t<uint8_t> pyimagedata = toPyArray(vimagedata);
        pyimagedata.resize({pgeom->height, pgeom->width, 3});
        imagedata = pyimagedata;
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = {npy_intp(pgeom->height), npy_intp(pgeom->width), npy_intp(3)};
        PyObject *pyvalues = PyArray_SimpleNew(3, dims, PyArray_UINT8);
        if( !vimagedata.empty() ) {
            memcpy(PyArray_DATA(pyvalues), vimagedata.data(), numel);
        }
        imagedata = py::to_array_astype<uint8_t>(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
}
PySensorBase::PyCameraSensorData::PyCameraSensorData(OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const> pgeom) : PySensorData(SensorBase::ST_Camera), intrinsics(pgeom->intrinsics)
{
    {
        const size_t numel = pgeom->height * pgeom->width * 3;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        py::array_t<uint8_t> pyimagedata({pgeom->height, pgeom->width, 3});
        py::buffer_info buf = pyimagedata.request();
        uint8_t* pimagedata = (uint8_t*) buf.ptr;
        memset(pimagedata, 0, numel * sizeof(uint8_t)); // although we know sizeof(uint8_t)==1
        imagedata = pyimagedata;
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = { pgeom->height,pgeom->width,3};
        PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
        memset(PyArray_DATA(pyvalues), 0, numel * sizeof(uint8_t));
        imagedata = py::to_array_astype<uint8_t>(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }
    {
        numeric::array arr(py::make_tuple(pgeom->intrinsics.fx,0,pgeom->intrinsics.cx,0,pgeom->intrinsics.fy,pgeom->intrinsics.cy,0,0,1));
        resize_3x3(arr);
        KK = arr;
    }
}
PySensorBase::PyCameraSensorData::~PyCameraSensorData() {
}

PySensorBase::PyJointEncoderSensorData::PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::JointEncoderSensorData> pdata) : PySensorData(pdata)
{
    encoderValues = toPyArray(pdata->encoderValues);
    encoderVelocity = toPyArray(pdata->encoderVelocity);
    resolution = toPyArray(pgeom->resolution);
}
PySensorBase::PyJointEncoderSensorData::PyJointEncoderSensorData(OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const> pgeom) : PySensorData(SensorBase::ST_JointEncoder)
{
    resolution = toPyArray(pgeom->resolution);
}
PySensorBase::PyJointEncoderSensorData::~PyJointEncoderSensorData() {
}

PySensorBase::PyForce6DSensorData::PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::Force6DSensorData> pdata) : PySensorData(pdata)
{
    force = toPyVector3(pdata->force);
    torque = toPyVector3(pdata->torque);
}
PySensorBase::PyForce6DSensorData::PyForce6DSensorData(OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const> pgeom) : PySensorData(SensorBase::ST_Force6D)
{
}
PySensorBase::PyForce6DSensorData::~PyForce6DSensorData() {
}

PySensorBase::PyIMUSensorData::PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::IMUSensorData> pdata) : PySensorData(pdata)
{
    rotation = toPyVector4(pdata->rotation);
    angular_velocity = toPyVector3(pdata->angular_velocity);
    linear_acceleration = toPyVector3(pdata->linear_acceleration);
    numeric::array arr = toPyArrayN(pdata->rotation_covariance.data(), pdata->rotation_covariance.size());
    resize_3x3(arr);
    rotation_covariance = arr;
    arr = toPyArrayN(pdata->angular_velocity_covariance.data(), pdata->angular_velocity_covariance.size());
    resize_3x3(arr);
    angular_velocity_covariance = arr;
    arr = toPyArrayN(pdata->linear_acceleration_covariance.data(), pdata->linear_acceleration_covariance.size());
    resize_3x3(arr);
    linear_acceleration_covariance = arr;
}
PySensorBase::PyIMUSensorData::PyIMUSensorData(OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const> pgeom) : PySensorData(SensorBase::ST_IMU)
{
}
PySensorBase::PyIMUSensorData::~PyIMUSensorData() {
}

PySensorBase::PyOdometrySensorData::PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::OdometrySensorData> pdata) : PySensorData(pdata)
{
    pose = toPyArray(pdata->pose);
    linear_velocity = toPyVector3(pdata->linear_velocity);
    angular_velocity = toPyVector3(pdata->angular_velocity);
    numeric::array arr = toPyArrayN(pdata->pose_covariance.data(), pdata->pose_covariance.size());
    resize_3x3(arr);
    pose_covariance = arr;
    arr = toPyArrayN(pdata->velocity_covariance.data(), pdata->velocity_covariance.size());
    resize_3x3(arr);
    velocity_covariance = arr;
    targetid = pgeom->targetid;

}
PySensorBase::PyOdometrySensorData::PyOdometrySensorData(OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const> pgeom) : PySensorData(SensorBase::ST_Odometry)
{
    targetid = pgeom->targetid;
}
PySensorBase::PyOdometrySensorData::~PyOdometrySensorData() {
}

PySensorBase::PyTactileSensorData::PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::TactileSensorData> pdata) : PySensorData(pdata)
{
    forces = toPyArray3(pdata->forces);
    numeric::array arr = toPyArrayN(pdata->force_covariance.data(), pdata->force_covariance.size());
    resize_3x3(arr);
    force_covariance = arr;
    positions = toPyArray3(pgeom->positions);
    thickness = pgeom->thickness;
}
PySensorBase::PyTactileSensorData::PyTactileSensorData(OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const> pgeom) : PySensorData(SensorBase::ST_Tactile)
{
    positions = toPyArray3(pgeom->positions);
    thickness = pgeom->thickness;
}
PySensorBase::PyTactileSensorData::~PyTactileSensorData() {
}

PySensorBase::PyActuatorSensorData::PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom, OPENRAVE_SHARED_PTR<SensorBase::ActuatorSensorData> pdata) : PySensorData(pdata)
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
PySensorBase::PyActuatorSensorData::PyActuatorSensorData(OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const> pgeom) : PySensorData(SensorBase::ST_Actuator) {
    maxtorque = pgeom->maxtorque;
    maxcurrent = pgeom->maxcurrent;
    nominalcurrent = pgeom->nominalcurrent;
    maxvelocity = pgeom->maxvelocity;
    maxacceleration = pgeom->maxacceleration;
    maxjerk = pgeom->maxjerk;
    staticfriction = pgeom->staticfriction;
    viscousfriction = pgeom->viscousfriction;
}
PySensorBase::PyActuatorSensorData::~PyActuatorSensorData() {
}

PySensorBase::PySensorBase(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensor, pyenv), _psensor(psensor)
{
}
PySensorBase::~PySensorBase() {
}

SensorBasePtr PySensorBase::GetSensor() {
    return _psensor;
}

int PySensorBase::Configure(SensorBase::ConfigureCommand command, bool blocking)
{
    return _psensor->Configure(command,blocking);
}

bool PySensorBase::SimulationStep(dReal timeelapsed)
{
    return _psensor->SimulationStep(timeelapsed);
}

OPENRAVE_SHARED_PTR<PySensorGeometry> PySensorBase::GetSensorGeometry(SensorBase::SensorType type)
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

OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> PySensorBase::CreateSensorData(SensorBase::SensorType type)
{
    return ConvertToPySensorData(_psensor->CreateSensorData(type));
}

OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> PySensorBase::GetSensorData()
{
    return GetSensorData(SensorBase::ST_Invalid);
}
OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> PySensorBase::GetSensorData(SensorBase::SensorType type)
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

OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> PySensorBase::ConvertToPySensorData(SensorBase::SensorDataPtr psensordata)
{
    if( !psensordata ) {
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>();
    }
    switch(psensordata->GetType()) {
    case SensorBase::ST_Laser:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyLaserSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserSensorData>(psensordata)));
    case SensorBase::ST_Camera:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyCameraSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraSensorData>(psensordata)));
    case SensorBase::ST_JointEncoder:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyJointEncoderSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::JointEncoderGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::JointEncoderSensorData>(psensordata)));
    case SensorBase::ST_Force6D:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyForce6DSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::Force6DGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::Force6DSensorData>(psensordata)));
    case SensorBase::ST_IMU:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyIMUSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::IMUGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::IMUSensorData>(psensordata)));
    case SensorBase::ST_Odometry:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyOdometrySensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::OdometryGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::OdometrySensorData>(psensordata)));
    case SensorBase::ST_Tactile:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyTactileSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::TactileGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::TactileSensorData>(psensordata)));
    case SensorBase::ST_Actuator:
        return OPENRAVE_SHARED_PTR<PySensorBase::PySensorData>(new PyActuatorSensorData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::ActuatorGeomData const>(_psensor->GetSensorGeometry()), OPENRAVE_STATIC_POINTER_CAST<SensorBase::ActuatorSensorData>(psensordata)));
    case SensorBase::ST_Invalid:
        break;
    }
    throw openrave_exception(boost::str(boost::format(_("unknown sensor data type %d\n"))%psensordata->GetType()));
}

bool PySensorBase::Supports(SensorBase::SensorType type) {
    return _psensor->Supports(type);
}

void PySensorBase::SetSensorGeometry(PySensorGeometryPtr pygeometry) {
    _psensor->SetSensorGeometry(pygeometry->GetGeometry());
}

void PySensorBase::SetTransform(object transform) {
    _psensor->SetTransform(ExtractTransform(transform));
}
object PySensorBase::GetTransform() {
    return ReturnTransform(_psensor->GetTransform());
}
object PySensorBase::GetTransformPose() {
    return toPyArray(_psensor->GetTransform());
}

object PySensorBase::GetName() {
    return ConvertStringToUnicode(_psensor->GetName());
}

void PySensorBase::SetName(const std::string& name)
{
    return _psensor->SetName(name);
}

std::string PySensorBase::__repr__() {
    return boost::str(boost::format("<RaveGetEnvironment(%d).GetSensor('%s')>")%RaveGetEnvironmentId(_psensor->GetEnv())%_psensor->GetName());
}
std::string PySensorBase::__str__() {
    return boost::str(boost::format("<%s:%s - %s>")%RaveGetInterfaceName(_psensor->GetInterfaceType())%_psensor->GetXMLId()%_psensor->GetName());
}
object PySensorBase::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

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
        return py::none_();
    }
    return py::to_object(PySensorBase(psensor,pyenv).GetSensorData());
}

PySensorBasePtr RaveCreateSensor(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    SensorBasePtr p = OpenRAVE::RaveCreateSensor(GetEnvironment(pyenv), name);
    if( !p ) {
        return PySensorBasePtr();
    }
    return PySensorBasePtr(new PySensorBase(p,pyenv));
}

PySensorGeometryPtr toPySensorGeometry(const std::string& sensorname, const rapidjson::Document& docSensorGeometry)
{
    SensorBase::SensorGeometryPtr pgeom;
    BaseJSONReaderPtr pReader = RaveCallJSONReader(PT_Sensor, sensorname, InterfaceBasePtr(), AttributesList());
    if (!!pReader) {
        pReader->DeserializeJSON(docSensorGeometry);
        ReadablePtr pReadable = pReader->GetReadable();
        if (!!pReadable) {
            pgeom = OPENRAVE_DYNAMIC_POINTER_CAST<SensorBase::SensorGeometry>(pReadable);
        }
    } else {
        RAVELOG_WARN_FORMAT("failed to get json reader for sensor type \"%s\"", sensorname);
    }
    if( !!pgeom ) {
        if( pgeom->GetType() == SensorBase::ST_Camera ) {
            return PySensorGeometryPtr(new PyCameraGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::CameraGeomData const>(pgeom)));
        }
        else if( pgeom->GetType() == SensorBase::ST_Laser ) {
            return PySensorGeometryPtr(new PyLaserGeomData(OPENRAVE_STATIC_POINTER_CAST<SensorBase::LaserGeomData const>(pgeom)));
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

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Configure_overloads, Configure, 1, 2)
// SerializeJSON
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PySensorGeometry_SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyCameraGeomData_SerializeJSON_overloads, SerializeJSON, 0, 2)
// DeserializeJSON
// BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PySensorGeometry_DeserializeJSON_overloads, DeserializeJSON, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PyCameraGeomData_DeserializeJSON_overloads, DeserializeJSON, 1, 2)
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_sensor(py::module& m)
#else
void init_openravepy_sensor()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals; // "..."_a
#endif
    {
        OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> (PySensorBase::*GetSensorData1)() = &PySensorBase::GetSensorData;
        OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> (PySensorBase::*GetSensorData2)(SensorBase::SensorType) = &PySensorBase::GetSensorData;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ sensor = class_<PySensorBase, OPENRAVE_SHARED_PTR<PySensorBase>, PyInterfaceBase>(m, "Sensor", DOXY_CLASS(SensorBase))
                        .def(init<SensorBasePtr, PyEnvironmentBasePtr>(), "psensor"_a, "pyenv"_a)
#else
        scope_ sensor = class_<PySensorBase, OPENRAVE_SHARED_PTR<PySensorBase>, bases<PyInterfaceBase> >("Sensor", DOXY_CLASS(SensorBase), no_init)
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                       .def("Configure", &PySensorBase::Configure,
                        "command"_a,
                        "blocking"_a = false,
                        DOXY_FN(SensorBase, Configure)
                        )
#else   
                       .def("Configure",&PySensorBase::Configure, Configure_overloads(PY_ARGS("command","blocking") DOXY_FN(SensorBase,Configure)))
#endif
                       .def("SimulationStep",&PySensorBase::SimulationStep, PY_ARGS("timeelapsed") DOXY_FN(SensorBase,SimulationStep))
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

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        class_<PyCameraIntrinsics, OPENRAVE_SHARED_PTR<PyCameraIntrinsics> >(sensor, "CameraIntrinsics", DOXY_CLASS(geometry::RaveCameraIntrinsics))
#else
        class_<PyCameraIntrinsics, OPENRAVE_SHARED_PTR<PyCameraIntrinsics> >("CameraIntrinsics", DOXY_CLASS(geometry::RaveCameraIntrinsics))
#endif
        .def_readwrite("K",&PyCameraIntrinsics::K)
        .def_readwrite("distortion_model",&PyCameraIntrinsics::distortion_model)
        .def_readwrite("distortion_coeffs",&PyCameraIntrinsics::distortion_coeffs)
        .def_readwrite("focal_length",&PyCameraIntrinsics::focal_length)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // SensorData is inside SensorBase
        class_<PySensorBase::PySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> >(sensor, "SensorData", DOXY_CLASS(SensorBase::SensorData))
        .def(init<SensorBase::SensorType>(), "type"_a)
        .def(init<SensorBase::SensorDataPtr>(), "pdata"_a)
#else
        class_<PySensorBase::PySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PySensorData> >("SensorData", DOXY_CLASS(SensorBase::SensorData),no_init)
#endif
        .def_readonly("type",&PySensorBase::PySensorData::type)
        .def_readonly("stamp",&PySensorBase::PySensorData::stamp)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // LaserSensorData is inside SensorBase
        class_<PySensorBase::PyLaserSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyLaserSensorData>, PySensorBase::PySensorData>(sensor, "LaserSensorData", DOXY_CLASS(SensorBase::LaserSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::LaserSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyLaserSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyLaserSensorData>, bases<PySensorBase::PySensorData> >("LaserSensorData", DOXY_CLASS(SensorBase::LaserSensorData),no_init)
#endif
        .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
        .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
        .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // CameraSensorData is inside SensorBase
        class_<PySensorBase::PyCameraSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyCameraSensorData>, PySensorBase::PySensorData>(sensor, "CameraSensorData", DOXY_CLASS(SensorBase::CameraSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::CameraSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyCameraSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyCameraSensorData>, bases<PySensorBase::PySensorData> >("CameraSensorData", DOXY_CLASS(SensorBase::CameraSensorData),no_init)
#endif
        .def_readonly("transform",&PySensorBase::PyCameraSensorData::transform)
        .def_readonly("imagedata",&PySensorBase::PyCameraSensorData::imagedata)
        .def_readonly("KK",&PySensorBase::PyCameraSensorData::KK)
        .def_readonly("intrinsics",&PySensorBase::PyCameraSensorData::intrinsics)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // JointEncoderSensorData is inside SensorBase
        class_<PySensorBase::PyJointEncoderSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyJointEncoderSensorData>, PySensorBase::PySensorData>(sensor, "JointEncoderSensorData", DOXY_CLASS(SensorBase::JointEncoderSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::JointEncoderSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyJointEncoderSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyJointEncoderSensorData>, bases<PySensorBase::PySensorData> >("JointEncoderSensorData", DOXY_CLASS(SensorBase::JointEncoderSensorData),no_init)
#endif
        .def_readonly("encoderValues",&PySensorBase::PyJointEncoderSensorData::encoderValues)
        .def_readonly("encoderVelocity",&PySensorBase::PyJointEncoderSensorData::encoderVelocity)
        .def_readonly("resolution",&PySensorBase::PyJointEncoderSensorData::resolution)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // Force6DSensorData is inside SensorBase
        class_<PySensorBase::PyForce6DSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyForce6DSensorData>, PySensorBase::PySensorData>(sensor, "Force6DSensorData", DOXY_CLASS(SensorBase::Force6DSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::Force6DSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyForce6DSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyForce6DSensorData>, bases<PySensorBase::PySensorData> >("Force6DSensorData", DOXY_CLASS(SensorBase::Force6DSensorData),no_init)
#endif
        .def_readonly("force",&PySensorBase::PyForce6DSensorData::force)
        .def_readonly("torque",&PySensorBase::PyForce6DSensorData::torque)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // IMUSensorData is inside SensorBase
        class_<PySensorBase::PyIMUSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyIMUSensorData>, PySensorBase::PySensorData>(sensor, "IMUSensorData", DOXY_CLASS(SensorBase::IMUSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::IMUSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyIMUSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyIMUSensorData>, bases<PySensorBase::PySensorData> >("IMUSensorData", DOXY_CLASS(SensorBase::IMUSensorData),no_init)
#endif
        .def_readonly("rotation",&PySensorBase::PyIMUSensorData::rotation)
        .def_readonly("angular_velocity",&PySensorBase::PyIMUSensorData::angular_velocity)
        .def_readonly("linear_acceleration",&PySensorBase::PyIMUSensorData::linear_acceleration)
        .def_readonly("rotation_covariance",&PySensorBase::PyIMUSensorData::rotation_covariance)
        .def_readonly("angular_velocity_covariance",&PySensorBase::PyIMUSensorData::angular_velocity_covariance)
        .def_readonly("linear_acceleration_covariance",&PySensorBase::PyIMUSensorData::linear_acceleration_covariance)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // OdometrySensorData is inside SensorBase
        class_<PySensorBase::PyOdometrySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyOdometrySensorData>, PySensorBase::PySensorData>(sensor, "OdometrySensorData", DOXY_CLASS(SensorBase::OdometrySensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::OdometrySensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyOdometrySensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyOdometrySensorData>, bases<PySensorBase::PySensorData> >("OdometrySensorData", DOXY_CLASS(SensorBase::OdometrySensorData),no_init)
#endif
        .def_readonly("pose",&PySensorBase::PyOdometrySensorData::pose)
        .def_readonly("linear_velocity",&PySensorBase::PyOdometrySensorData::linear_velocity)
        .def_readonly("angular_velocity",&PySensorBase::PyOdometrySensorData::angular_velocity)
        .def_readonly("pose_covariance",&PySensorBase::PyOdometrySensorData::pose_covariance)
        .def_readonly("velocity_covariance",&PySensorBase::PyOdometrySensorData::velocity_covariance)
        .def_readonly("targetid",&PySensorBase::PyOdometrySensorData::targetid)
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // TactileSensorData is inside SensorBase
        class_<PySensorBase::PyTactileSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyTactileSensorData>, PySensorBase::PySensorData>(sensor, "TactileSensorData", DOXY_CLASS(SensorBase::TactileSensorData))
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::TactileSensorData>>(), "pgeom"_a, "pdata"_a)
        .def(init<OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const>>(), "pgeom"_a)
#else
        class_<PySensorBase::PyTactileSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyTactileSensorData>, bases<PySensorBase::PySensorData> >("TactileSensorData", DOXY_CLASS(SensorBase::TactileSensorData),no_init)
#endif
        .def_readonly("forces",&PySensorBase::PyTactileSensorData::forces)
        .def_readonly("force_covariance",&PySensorBase::PyTactileSensorData::force_covariance)
        .def_readonly("positions",&PySensorBase::PyTactileSensorData::positions)
        .def_readonly("thickness",&PySensorBase::PyTactileSensorData::thickness)
        ;

        {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            scope_ actuatorsensordata = 
            // ActuatorSensorData is inside SensorBase
            class_<PySensorBase::PyActuatorSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyActuatorSensorData>, PySensorBase::PySensorData>(sensor, "ActuatorSensorData", DOXY_CLASS(SensorBase::ActuatorSensorData))
            .def(init<OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const>, OPENRAVE_SHARED_PTR<SensorBase::ActuatorSensorData>>(), "pgeom"_a, "pdata"_a)
            .def(init<OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const>>(), "pdata"_a)
#else
            class_<PySensorBase::PyActuatorSensorData, OPENRAVE_SHARED_PTR<PySensorBase::PyActuatorSensorData>, bases<PySensorBase::PySensorData> >("ActuatorSensorData", DOXY_CLASS(SensorBase::ActuatorSensorData),no_init)
#endif
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

#ifdef USE_PYBIND11_PYTHON_BINDINGS
            // ActuatorState belongs to ActuatorSensorData
            enum_<SensorBase::ActuatorSensorData::ActuatorState>(actuatorsensordata, "ActuatorState", py::arithmetic() DOXY_ENUM(ActuatorState))
#else
            enum_<SensorBase::ActuatorSensorData::ActuatorState>("ActuatorState" DOXY_ENUM(ActuatorState))
#endif
            .value("Undefined",SensorBase::ActuatorSensorData::AS_Undefined)
            .value("Idle",SensorBase::ActuatorSensorData::AS_Idle)
            .value("Moving",SensorBase::ActuatorSensorData::AS_Moving)
            .value("Stalled",SensorBase::ActuatorSensorData::AS_Stalled)
            .value("Braked",SensorBase::ActuatorSensorData::AS_Braked)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .export_values()
#endif
            ;
        }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // SensorType belongs to Sensor
        enum_<SensorBase::SensorType>(sensor, "Type", py::arithmetic() DOXY_ENUM(SensorType))
#else
        enum_<SensorBase::SensorType>("Type" DOXY_ENUM(SensorType))
#endif
        .value("Invalid",SensorBase::ST_Invalid)
        .value("Laser",SensorBase::ST_Laser)
        .value("Camera",SensorBase::ST_Camera)
        .value("JointEncoder",SensorBase::ST_JointEncoder)
        .value("Force6D",SensorBase::ST_Force6D)
        .value("IMU",SensorBase::ST_IMU)
        .value("Odometry",SensorBase::ST_Odometry)
        .value("Tactile",SensorBase::ST_Tactile)
        .value("Actuator",SensorBase::ST_Actuator)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .export_values()
#endif
        ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
        enum_<SensorBase::ConfigureCommand>(sensor, "ConfigureCommand", py::arithmetic() DOXY_ENUM(ConfigureCommand))
#else
        enum_<SensorBase::ConfigureCommand>("ConfigureCommand" DOXY_ENUM(ConfigureCommand))
#endif
        .value("PowerOn",SensorBase::CC_PowerOn)
        .value("PowerOff",SensorBase::CC_PowerOff)
        .value("PowerCheck",SensorBase::CC_PowerCheck)
        .value("RenderDataOn",SensorBase::CC_RenderDataOn)
        .value("RenderDataOff",SensorBase::CC_RenderDataOff)
        .value("RenderDataCheck",SensorBase::CC_RenderDataCheck)
        .value("RenderGeometryOn",SensorBase::CC_RenderGeometryOn)
        .value("RenderGeometryOff",SensorBase::CC_RenderGeometryOff)
        .value("RenderGeometryCheck",SensorBase::CC_RenderGeometryCheck)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        .export_values()
#endif
        ;
    }

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PySensorGeometry, OPENRAVE_SHARED_PTR<PySensorGeometry>>(m, "SensorGeometry", DOXY_CLASS(PySensorGeometry))
    // how to handle pure virtual
    // see p.62 (book page) https://buildmedia.readthedocs.org/media/pdf/pybind11/stable/pybind11.pdf
    .def("GetType", &PySensorGeometry::GetType)
#else
    class_<PySensorGeometry, OPENRAVE_SHARED_PTR<PySensorGeometry>, boost::noncopyable >("SensorGeometry", DOXY_CLASS(PySensorGeometry),no_init)
    .def("GetType",py::pure_virtual(&PySensorGeometry::GetType))
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyCameraGeomData, OPENRAVE_SHARED_PTR<PyCameraGeomData>, PySensorGeometry>(m, "CameraGeomData", DOXY_CLASS(SensorBase::CameraGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::CameraGeomData const>>(), "pgeom"_a)
#else
    class_<PyCameraGeomData, OPENRAVE_SHARED_PTR<PyCameraGeomData>, bases<PySensorGeometry> >("CameraGeomData", DOXY_CLASS(SensorBase::CameraGeomData))
#endif
    .def_readwrite("intrinsics",&PyCameraGeomData::intrinsics)
    .def_readwrite("hardware_id",&PyCameraGeomData::hardware_id)
    .def_readwrite("width",&PyCameraGeomData::width)
    .def_readwrite("height",&PyCameraGeomData::height)
    .def_readwrite("sensor_reference",&PyCameraGeomData::sensor_reference)
    .def_readwrite("target_region",&PyCameraGeomData::target_region)
    .def_readwrite("measurement_time",&PyCameraGeomData::measurement_time)
    .def_readwrite("gain",&PyCameraGeomData::gain)
    .def_readwrite("KK",&PyCameraGeomData::intrinsics) // deprecated
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def("SerializeJSON", &PyCameraGeomData::SerializeJSON,
        "unitScale"_a = 1.0,
        "options"_a = py::none_(),
        DOXY_FN(SensorBase::CameraGeomData, SerializeJSON)
    )
    .def("DeserializeJSON", &PyCameraGeomData::DeserializeJSON,
        "obj"_a,
        "unitScale"_a = 1.0,
        DOXY_FN(SensorBase::CameraGeomData, DeserializeJSON)
    )
#else
    .def("SerializeJSON", &PyCameraGeomData::SerializeJSON, PyCameraGeomData_SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(SensorBase::CameraGeomData, SerializeJSON)))
    .def("DeserializeJSON", &PyCameraGeomData::DeserializeJSON, PyCameraGeomData_DeserializeJSON_overloads(PY_ARGS("obj", "unitScale") DOXY_FN(SensorBase::CameraGeomData, DeserializeJSON)))
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyLaserGeomData, OPENRAVE_SHARED_PTR<PyLaserGeomData>, PySensorGeometry>(m, "LaserGeomData", DOXY_CLASS(SensorBase::LaserGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::LaserGeomData const>>(), "pgeom"_a)
#else
    class_<PyLaserGeomData, OPENRAVE_SHARED_PTR<PyLaserGeomData>, bases<PySensorGeometry> >("LaserGeomData", DOXY_CLASS(SensorBase::LaserGeomData))
#endif
    .def_readwrite("min_angle",&PyLaserGeomData::min_angle)
    .def_readwrite("max_angle",&PyLaserGeomData::max_angle)
    .def_readwrite("min_range",&PyLaserGeomData::min_range)
    .def_readwrite("max_range",&PyLaserGeomData::max_range)
    .def_readwrite("time_increment",&PyLaserGeomData::time_increment)
    .def_readwrite("time_scan",&PyLaserGeomData::time_scan)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyJointEncoderGeomData, OPENRAVE_SHARED_PTR<PyJointEncoderGeomData>, PySensorGeometry>(m, "JointEncoderGeomData", DOXY_CLASS(SensorBase::JointEncoderGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::JointEncoderGeomData const>>(), "pgeom"_a)
#else
    class_<PyJointEncoderGeomData, OPENRAVE_SHARED_PTR<PyJointEncoderGeomData>, bases<PySensorGeometry> >("JointEncoderGeomData", DOXY_CLASS(SensorBase::JointEncoderGeomData))
#endif
    .def_readwrite("resolution",&PyJointEncoderGeomData::resolution)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyForce6DGeomData, OPENRAVE_SHARED_PTR<PyForce6DGeomData>, PySensorGeometry>(m, "Force6DGeomData", DOXY_CLASS(SensorBase::Force6DGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::Force6DGeomData const>>(), "pgeom"_a)
#else
    class_<PyForce6DGeomData, OPENRAVE_SHARED_PTR<PyForce6DGeomData>, bases<PySensorGeometry> >("Force6DGeomData", DOXY_CLASS(SensorBase::Force6DGeomData))
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyIMUGeomData, OPENRAVE_SHARED_PTR<PyIMUGeomData>, PySensorGeometry>(m, "IMUGeomData", DOXY_CLASS(SensorBase::IMUGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::IMUGeomData const>>(), "pgeom"_a)
#else
    class_<PyIMUGeomData, OPENRAVE_SHARED_PTR<PyIMUGeomData>, bases<PySensorGeometry> >("IMUGeomData", DOXY_CLASS(SensorBase::IMUGeomData))
#endif
    .def_readwrite("time_measurement",&PyIMUGeomData::time_measurement)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyOdometryGeomData, OPENRAVE_SHARED_PTR<PyOdometryGeomData>, PySensorGeometry>(m, "OdometryGeomData", DOXY_CLASS(SensorBase::OdometryGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::OdometryGeomData const>>(), "pgeom"_a)
#else
    class_<PyOdometryGeomData, OPENRAVE_SHARED_PTR<PyOdometryGeomData>, bases<PySensorGeometry> >("OdometryGeomData", DOXY_CLASS(SensorBase::OdometryGeomData))
#endif
    .def_readwrite("targetid",&PyOdometryGeomData::targetid)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyTactileGeomData, OPENRAVE_SHARED_PTR<PyTactileGeomData>, PySensorGeometry>(m, "TactileGeomData", DOXY_CLASS(SensorBase::TactileGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::TactileGeomData const>>(), "pgeom"_a)
#else
    class_<PyTactileGeomData, OPENRAVE_SHARED_PTR<PyTactileGeomData>, bases<PySensorGeometry> >("TactileGeomData", DOXY_CLASS(SensorBase::TactileGeomData))
#endif
    .def_readwrite("thickness",&PyTactileGeomData::thickness)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyActuatorGeomData, OPENRAVE_SHARED_PTR<PyActuatorGeomData>, PySensorGeometry>(m, "ActuatorGeomData", DOXY_CLASS(SensorBase::ActuatorGeomData))
    .def(init<>())
    .def(init<OPENRAVE_SHARED_PTR<SensorBase::ActuatorGeomData const>>(), "pgeom"_a)
#else
    class_<PyActuatorGeomData, OPENRAVE_SHARED_PTR<PyActuatorGeomData>, bases<PySensorGeometry> >("ActuatorGeomData", DOXY_CLASS(SensorBase::ActuatorGeomData))
#endif
    .def_readwrite("maxtorque",&PyActuatorGeomData::maxtorque)
    .def_readwrite("maxcurrent",&PyActuatorGeomData::maxcurrent)
    .def_readwrite("nominalcurrent",&PyActuatorGeomData::nominalcurrent)
    .def_readwrite("maxvelocity",&PyActuatorGeomData::maxvelocity)
    .def_readwrite("maxacceleration",&PyActuatorGeomData::maxacceleration)
    .def_readwrite("maxjerk",&PyActuatorGeomData::maxjerk)
    .def_readwrite("staticfriction",&PyActuatorGeomData::staticfriction)
    .def_readwrite("viscousfriction",&PyActuatorGeomData::viscousfriction)
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateSensor",openravepy::RaveCreateSensor, PY_ARGS("env","name") DOXY_FN1(RaveCreateSensor));
#else
    def("RaveCreateSensor",openravepy::RaveCreateSensor, PY_ARGS("env","name") DOXY_FN1(RaveCreateSensor));
#endif
}

}
