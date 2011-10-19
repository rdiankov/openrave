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
        PyLaserSensorData(boost::shared_ptr<SensorBase::LaserGeomData> pgeom, boost::shared_ptr<SensorBase::LaserSensorData> pdata) : PySensorData(pdata)
        {
            positions = toPyArray3(pdata->positions);
            ranges = toPyArray3(pdata->ranges);
            intensity = toPyArrayN(pdata->intensity.size()>0 ? &pdata->intensity[0] : NULL,pdata->intensity.size());
        }
        PyLaserSensorData(boost::shared_ptr<SensorBase::LaserGeomData> pgeom) : PySensorData(SensorBase::ST_Laser) {
        }
        virtual ~PyLaserSensorData() {
        }

        object positions, ranges, intensity;
    };

    class PyCameraIntrinsics
    {
public:
        PyCameraIntrinsics(const SensorBase::CameraIntrinsics& intrinsics = SensorBase::CameraIntrinsics())
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

        object K;
        string distortion_model;
        object distortion_coeffs;
        dReal focal_length;
    };

    class PyCameraSensorData : public PySensorData
    {
public:
        PyCameraSensorData(boost::shared_ptr<SensorBase::CameraGeomData> pgeom, boost::shared_ptr<SensorBase::CameraSensorData> pdata) : PySensorData(pdata), intrinsics(pgeom->KK)
        {
            if( (int)pdata->vimagedata.size() != pgeom->height*pgeom->width*3 ) {
                throw openrave_exception("bad image data");
            }
            {
                npy_intp dims[] = { pgeom->height,pgeom->width,3};
                PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
                if( pdata->vimagedata.size() > 0 ) {
                    memcpy(PyArray_DATA(pyvalues),&pdata->vimagedata[0],pdata->vimagedata.size());
                }
                imagedata = static_cast<numeric::array>(handle<>(pyvalues));
            }
            KK = intrinsics.K;
        }
        PyCameraSensorData(boost::shared_ptr<SensorBase::CameraGeomData> pgeom) : PySensorData(SensorBase::ST_Camera)
        {
            {
                npy_intp dims[] = { pgeom->height,pgeom->width,3};
                PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
                memset(PyArray_DATA(pyvalues),0,pgeom->height*pgeom->width*3);
                imagedata = static_cast<numeric::array>(handle<>(pyvalues));
            }
            {
                numeric::array arr(boost::python::make_tuple(pgeom->KK.fx,0,pgeom->KK.cx,0,pgeom->KK.fy,pgeom->KK.cy,0,0,1));
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
        PyJointEncoderSensorData(boost::shared_ptr<SensorBase::JointEncoderGeomData> pgeom, boost::shared_ptr<SensorBase::JointEncoderSensorData> pdata) : PySensorData(pdata)
        {
            encoderValues = toPyArray(pdata->encoderValues);
            encoderVelocity = toPyArray(pdata->encoderVelocity);
            resolution = toPyArray(pgeom->resolution);
        }
        PyJointEncoderSensorData(boost::shared_ptr<SensorBase::JointEncoderGeomData> pgeom) : PySensorData(SensorBase::ST_JointEncoder)
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
        PyForce6DSensorData(boost::shared_ptr<SensorBase::Force6DGeomData> pgeom, boost::shared_ptr<SensorBase::Force6DSensorData> pdata) : PySensorData(pdata)
        {
            force = toPyVector3(pdata->force);
            torque = toPyVector3(pdata->torque);
        }
        PyForce6DSensorData(boost::shared_ptr<SensorBase::Force6DGeomData> pgeom) : PySensorData(SensorBase::ST_Force6D)
        {
        }
        virtual ~PyForce6DSensorData() {
        }
        object force, torque;
    };

    class PyIMUSensorData : public PySensorData
    {
public:
        PyIMUSensorData(boost::shared_ptr<SensorBase::IMUGeomData> pgeom, boost::shared_ptr<SensorBase::IMUSensorData> pdata) : PySensorData(pdata)
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
        PyIMUSensorData(boost::shared_ptr<SensorBase::IMUGeomData> pgeom) : PySensorData(SensorBase::ST_IMU)
        {
        }
        virtual ~PyIMUSensorData() {
        }
        object rotation, angular_velocity, linear_acceleration, rotation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
    };

    class PyOdometrySensorData : public PySensorData
    {
public:
        PyOdometrySensorData(boost::shared_ptr<SensorBase::OdometryGeomData> pgeom, boost::shared_ptr<SensorBase::OdometrySensorData> pdata) : PySensorData(pdata)
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
        PyOdometrySensorData(boost::shared_ptr<SensorBase::OdometryGeomData> pgeom) : PySensorData(SensorBase::ST_Odometry)
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
        PyTactileSensorData(boost::shared_ptr<SensorBase::TactileGeomData> pgeom, boost::shared_ptr<SensorBase::TactileSensorData> pdata) : PySensorData(pdata)
        {
            forces = toPyArray3(pdata->forces);
            numeric::array arr = toPyArrayN(&pdata->force_covariance[0],pdata->force_covariance.size());
            arr.resize(3,3);
            force_covariance = arr;
            positions = toPyArray3(pgeom->positions);
            thickness = pgeom->thickness;
        }
        PyTactileSensorData(boost::shared_ptr<SensorBase::TactileGeomData> pgeom) : PySensorData(SensorBase::ST_Tactile)
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
        PyActuatorSensorData(boost::shared_ptr<SensorBase::ActuatorGeomData> pgeom, boost::shared_ptr<SensorBase::ActuatorSensorData> pdata) : PySensorData(pdata)
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
        PyActuatorSensorData(boost::shared_ptr<SensorBase::ActuatorGeomData> pgeom) : PySensorData(SensorBase::ST_Actuator)        {
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
            throw openrave_exception("SensorData failed");
        }
        switch(psensordata->GetType()) {
        case SensorBase::ST_Laser:
            return boost::shared_ptr<PySensorData>(new PyLaserSensorData(boost::static_pointer_cast<SensorBase::LaserGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::LaserSensorData>(psensordata)));
        case SensorBase::ST_Camera:
            return boost::shared_ptr<PySensorData>(new PyCameraSensorData(boost::static_pointer_cast<SensorBase::CameraGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::CameraSensorData>(psensordata)));
        case SensorBase::ST_JointEncoder:
            return boost::shared_ptr<PySensorData>(new PyJointEncoderSensorData(boost::static_pointer_cast<SensorBase::JointEncoderGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::JointEncoderSensorData>(psensordata)));
        case SensorBase::ST_Force6D:
            return boost::shared_ptr<PySensorData>(new PyForce6DSensorData(boost::static_pointer_cast<SensorBase::Force6DGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::Force6DSensorData>(psensordata)));
        case SensorBase::ST_IMU:
            return boost::shared_ptr<PySensorData>(new PyIMUSensorData(boost::static_pointer_cast<SensorBase::IMUGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::IMUSensorData>(psensordata)));
        case SensorBase::ST_Odometry:
            return boost::shared_ptr<PySensorData>(new PyOdometrySensorData(boost::static_pointer_cast<SensorBase::OdometryGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::OdometrySensorData>(psensordata)));
        case SensorBase::ST_Tactile:
            return boost::shared_ptr<PySensorData>(new PyTactileSensorData(boost::static_pointer_cast<SensorBase::TactileGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::TactileSensorData>(psensordata)));
        case SensorBase::ST_Actuator:
            return boost::shared_ptr<PySensorData>(new PyActuatorSensorData(boost::static_pointer_cast<SensorBase::ActuatorGeomData>(_psensor->GetSensorGeometry()), boost::static_pointer_cast<SensorBase::ActuatorSensorData>(psensordata)));
        case SensorBase::ST_Invalid:
            break;
        }
        throw openrave_exception(boost::str(boost::format("unknown sensor data type %d\n")%psensordata->GetType()));
    }
    boost::shared_ptr<PySensorData> GetSensorGeometry(SensorBase::SensorType type)
    {
        switch(type) {
        case SensorBase::ST_Laser:
            return boost::shared_ptr<PySensorData>(new PyLaserSensorData(boost::static_pointer_cast<SensorBase::LaserGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Camera:
            return boost::shared_ptr<PySensorData>(new PyCameraSensorData(boost::static_pointer_cast<SensorBase::CameraGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_JointEncoder:
            return boost::shared_ptr<PySensorData>(new PyJointEncoderSensorData(boost::static_pointer_cast<SensorBase::JointEncoderGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Force6D:
            return boost::shared_ptr<PySensorData>(new PyForce6DSensorData(boost::static_pointer_cast<SensorBase::Force6DGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_IMU:
            return boost::shared_ptr<PySensorData>(new PyIMUSensorData(boost::static_pointer_cast<SensorBase::IMUGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Odometry:
            return boost::shared_ptr<PySensorData>(new PyOdometrySensorData(boost::static_pointer_cast<SensorBase::OdometryGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Tactile:
            return boost::shared_ptr<PySensorData>(new PyTactileSensorData(boost::static_pointer_cast<SensorBase::TactileGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Actuator:
            return boost::shared_ptr<PySensorData>(new PyActuatorSensorData(boost::static_pointer_cast<SensorBase::ActuatorGeomData>(_psensor->GetSensorGeometry())));
        case SensorBase::ST_Invalid:
            break;
        }
        throw openrave_exception(boost::str(boost::format("unknown sensor data type %d\n")%type));
    }

    bool Supports(SensorBase::SensorType type) {
        return _psensor->Supports(type);
    }

    void SetTransform(object transform) {
        _psensor->SetTransform(ExtractTransform(transform));
    }
    object GetTransform() {
        return ReturnTransform(_psensor->GetTransform());
    }

    string GetName() {
        return _psensor->GetName();
    }

    virtual string __repr__() {
        return boost::str(boost::format("<RaveGetEnvironment(%d).GetSensor('%s')>")%RaveGetEnvironmentId(_psensor->GetEnv())%_psensor->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s>")%RaveGetInterfaceName(_psensor->GetInterfaceType())%_psensor->GetXMLId()%_psensor->GetName());
    }
};

namespace openravepy {

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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Configure_overloads, Configure, 1, 2)

void init_openravepy_sensor()
{
    {
        boost::shared_ptr<PySensorBase::PySensorData> (PySensorBase::*GetSensorData1)() = &PySensorBase::GetSensorData;
        boost::shared_ptr<PySensorBase::PySensorData> (PySensorBase::*GetSensorData2)(SensorBase::SensorType) = &PySensorBase::GetSensorData;
        scope sensor = class_<PySensorBase, boost::shared_ptr<PySensorBase>, bases<PyInterfaceBase> >("Sensor", DOXY_CLASS(SensorBase), no_init)
                       .def("Configure",&PySensorBase::Configure, Configure_overloads(args("command","blocking"), DOXY_FN(SensorBase,Configure)))
                       .def("GetSensorData",GetSensorData1, DOXY_FN(SensorBase,GetSensorData))
                       .def("GetSensorData",GetSensorData2, DOXY_FN(SensorBase,GetSensorData))
                       .def("GetSensorGeometry",&PySensorBase::GetSensorGeometry,DOXY_FN(SensorBase,GetSensorGeometry))
                       .def("SetTransform",&PySensorBase::SetTransform, DOXY_FN(SensorBase,SetTransform))
                       .def("GetTransform",&PySensorBase::GetTransform, DOXY_FN(SensorBase,GetTransform))
                       .def("GetName",&PySensorBase::GetName, DOXY_FN(SensorBase,GetName))
                       .def("Supports",&PySensorBase::Supports, DOXY_FN(SensorBase,Supports))
                       .def("__str__",&PySensorBase::__str__)
                       .def("__repr__",&PySensorBase::__repr__)
        ;

        class_<PySensorBase::PyCameraIntrinsics, boost::shared_ptr<PySensorBase::PyCameraIntrinsics> >("CameraIntrinsics", DOXY_CLASS(SensorBase::CameraIntrinsics))
        .def_readonly("K",&PySensorBase::PyCameraIntrinsics::K)
        .def_readonly("distortion_model",&PySensorBase::PyCameraIntrinsics::distortion_model)
        .def_readonly("distortion_coeffs",&PySensorBase::PyCameraIntrinsics::distortion_coeffs)
        .def_readonly("focal_length",&PySensorBase::PyCameraIntrinsics::focal_length)
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

    def("RaveCreateSensor",openravepy::RaveCreateSensor,args("env","name"),DOXY_FN1(RaveCreateSensor));
}

}
