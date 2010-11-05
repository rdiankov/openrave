// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
// -*- coding: utf-8 -*-
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
#ifndef OPENRAVEPY_INTERNAL_H
#define OPENRAVEPY_INTERNAL_H

#include "openrave-core.h"

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#include <Python.h>

#include <sstream>
#include <exception>

#include <boost/array.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/format.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/version.hpp>

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/docstring_options.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"
#include "docstrings.h"

#define CHECK_POINTER(p) { \
        if( !(p) ) throw openrave_exception(boost::str(boost::format("[%s:%d]: invalid pointer")%__PRETTY_FUNCTION__%__LINE__)); \
}

using namespace boost::python;
using namespace std;
using namespace OpenRAVE;

struct DummyStruct {};

class PyInterfaceBase;
class PyKinBody;
class PyRobotBase;
class PyEnvironmentBase;
class PyCollisionReport;
class PyPhysicsEngineBase;
class PyCollisionCheckerBase;
class PyIkSolverBase;
class PyPlannerBase;
class PySensorBase;
class PySensorSystemBase;
class PyControllerBase;
class PyTrajectoryBase;
class PyProblemInstance;
class PyViewerBase;

typedef boost::shared_ptr<PyInterfaceBase> PyInterfaceBasePtr;
typedef boost::shared_ptr<PyInterfaceBase const> PyInterfaceBaseConstPtr;
typedef boost::shared_ptr<PyKinBody> PyKinBodyPtr;
typedef boost::shared_ptr<PyKinBody const> PyKinBodyConstPtr;
typedef boost::shared_ptr<PyRobotBase> PyRobotBasePtr;
typedef boost::shared_ptr<PyRobotBase const> PyRobotBaseConstPtr;
typedef boost::shared_ptr<PyEnvironmentBase> PyEnvironmentBasePtr;
typedef boost::shared_ptr<PyEnvironmentBase const> PyEnvironmentBaseConstPtr;
typedef boost::shared_ptr<PyIkSolverBase> PyIkSolverBasePtr;
typedef boost::shared_ptr<PyIkSolverBase const> PyIkSolverBaseConstPtr;
typedef boost::shared_ptr<PyTrajectoryBase> PyTrajectoryBasePtr;
typedef boost::shared_ptr<PyTrajectoryBase const> PyTrajectoryBaseConstPtr;
typedef boost::shared_ptr<PyCollisionReport> PyCollisionReportPtr;
typedef boost::shared_ptr<PyCollisionReport const> PyCollisionReportConstPtr;
typedef boost::shared_ptr<PyPhysicsEngineBase> PyPhysicsEngineBasePtr;
typedef boost::shared_ptr<PyPhysicsEngineBase const> PyPhysicsEngineBaseConstPtr;
typedef boost::shared_ptr<PyCollisionCheckerBase> PyCollisionCheckerBasePtr;
typedef boost::shared_ptr<PyCollisionCheckerBase const> PyCollisionCheckerBaseConstPtr;
typedef boost::shared_ptr<PyPlannerBase> PyPlannerBasePtr;
typedef boost::shared_ptr<PyPlannerBase const> PyPlannerBaseConstPtr;
typedef boost::shared_ptr<PySensorBase> PySensorBasePtr;
typedef boost::shared_ptr<PySensorBase const> PySensorBaseConstPtr;
typedef boost::shared_ptr<PySensorSystemBase> PySensorSystemBasePtr;
typedef boost::shared_ptr<PySensorSystemBase const> PySensorSystemBaseConstPtr;
typedef boost::shared_ptr<PyControllerBase> PyControllerBasePtr;
typedef boost::shared_ptr<PyControllerBase const> PyControllerBaseConstPtr;
typedef boost::shared_ptr<PyProblemInstance> PyProblemInstancePtr;
typedef boost::shared_ptr<PyProblemInstance const> PyProblemInstanceConstPtr;
typedef boost::shared_ptr<PyViewerBase> PyViewerBasePtr;
typedef boost::shared_ptr<PyViewerBase const> PyViewerBaseConstPtr;

inline uint64_t GetMicroTime()
{
#ifdef _WIN32
    LARGE_INTEGER count, freq;
    QueryPerformanceCounter(&count);
    QueryPerformanceFrequency(&freq);
    return (count.QuadPart * 1000000) / freq.QuadPart;
#else
    struct timeval t;
    gettimeofday(&t, NULL);
    return (uint64_t)t.tv_sec*1000000+t.tv_usec;
#endif
}

struct null_deleter { void operator()(void const *) const {} };

inline RaveVector<float> ExtractFloat3(const object& o)
{
    return RaveVector<float>(extract<float>(o[0]), extract<float>(o[1]), extract<float>(o[2]));
}

template <typename T>
inline Vector ExtractVector3Type(const object& o)
{
    return Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]));
}

template <typename T>
inline Vector ExtractVector4Type(const object& o)
{
    return Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3]));
}

inline Vector ExtractVector3(const object& oraw)
{
    return ExtractVector3Type<dReal>(oraw);
}

inline Vector ExtractVector4(const object& oraw)
{
    return ExtractVector4Type<dReal>(oraw);
}

template <typename T>
inline RaveVector<T> ExtractVector34(const object& oraw,T fdefaultw)
{
    int n = len(oraw);
    if( n == 3 ) {
        RaveVector<T> v = ExtractVector3Type<T>(oraw);
        v.w = fdefaultw;
        return v;
    }
    else if( n == 4 )
        return ExtractVector4Type<T>(oraw);
    throw openrave_exception("unexpected vector size");
}

template <typename T>
inline Transform ExtractTransformType(const object& o)
{
    if( len(o) == 7 )
        return Transform(Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])),
                         Vector(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        object orow = o[i];
        t.m[4*i+0] = extract<T>(orow[0]);
        t.m[4*i+1] = extract<T>(orow[1]);
        t.m[4*i+2] = extract<T>(orow[2]);
        t.trans[i] = extract<T>(orow[3]);
    }
    return t;
}

template <typename T>
inline TransformMatrix ExtractTransformMatrixType(const object& o)
{
    if( len(o) == 7 )
        return Transform(Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])),
                         Vector(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        object orow = o[i];
        t.m[4*i+0] = extract<T>(orow[0]);
        t.m[4*i+1] = extract<T>(orow[1]);
        t.m[4*i+2] = extract<T>(orow[2]);
        t.trans[i] = extract<T>(orow[3]);
    }
    return t;
}

inline Transform ExtractTransform(const object& oraw)
{
    return ExtractTransformType<dReal>(oraw);
}

inline TransformMatrix ExtractTransformMatrix(const object& oraw)
{
    return ExtractTransformMatrixType<dReal>(oraw);
}

inline object toPyArray(const TransformMatrix& t)
{
    npy_intp dims[] = {4,4};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2]; pdata[3] = t.trans.x;
    pdata[4] = t.m[4]; pdata[5] = t.m[5]; pdata[6] = t.m[6]; pdata[7] = t.trans.y;
    pdata[8] = t.m[8]; pdata[9] = t.m[9]; pdata[10] = t.m[10]; pdata[11] = t.trans.z;
    pdata[12] = 0; pdata[13] = 0; pdata[14] = 0; pdata[15] = 1;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArrayRotation(const TransformMatrix& t)
{
    npy_intp dims[] = {3,3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
    pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
    pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray(const Transform& t)
{
    npy_intp dims[] = {7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray3(const std::vector<RaveVector<float> >& v)
{
    npy_intp dims[] = {v.size(),3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_FLOAT);
    if( v.size() > 0 ) {
        float* pf = (float*)PyArray_DATA(pyvalues);
        FOREACHC(it,v) {
            *pf++ = it->x;
            *pf++ = it->y;
            *pf++ = it->z;
        }
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray3(const std::vector<RaveVector<double> >& v)
{
    npy_intp dims[] = {v.size(),3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_DOUBLE);
    if( v.size() > 0 ) {
        double* pf = (double*)PyArray_DATA(pyvalues);
        FOREACHC(it,v) {
            *pf++ = it->x;
            *pf++ = it->y;
            *pf++ = it->z;
        }
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyVector3(Vector v)
{
    return numeric::array(boost::python::make_tuple(v.x,v.y,v.z));
}

inline object toPyVector4(Vector v)
{
    return numeric::array(boost::python::make_tuple(v.x,v.y,v.z,v.w));
}

class PyPluginInfo
{
public:
    PyPluginInfo(const PLUGININFO& info)
    {
        FOREACHC(it, info.interfacenames) {
            boost::python::list names;
            FOREACHC(itname,it->second)
                names.append(*itname);
            interfacenames.append(boost::python::make_tuple(it->first,names));
        }
        version = OPENRAVE_VERSION_STRING_FORMAT(info.version);
    }

    boost::python::list interfacenames;
    string version;
};

class PyGraphHandle
{
public:
    PyGraphHandle() {}
    PyGraphHandle(GraphHandlePtr handle) : _handle(handle) {}
    virtual ~PyGraphHandle() {}

    void SetTransform(object otrans) { _handle->SetTransform(RaveTransform<float>(ExtractTransformMatrixType<float>(otrans))); }
    void SetShow(bool bshow) { _handle->SetShow(bshow); }
private:
    GraphHandlePtr _handle;
};

namespace openravepy
{
    object RaveGetPluginInfo();
    object RaveGetLoadedInterfaces();
    PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name);
    PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyPlannerBasePtr RaveCreatePlanner(PyEnvironmentBasePtr pyenv, const std::string& name);
    PySensorSystemBasePtr RaveCreateSensorSystem(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyControllerBasePtr RaveCreateController(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyProblemInstancePtr RaveCreateProblem(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyIkSolverBasePtr RaveCreateIkSolver(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyPhysicsEngineBasePtr RaveCreatePhysicsEngine(PyEnvironmentBasePtr pyenv, const std::string& name);
    PySensorBasePtr RaveCreateSensor(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyCollisionCheckerBasePtr RaveCreateCollisionChecker(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyViewerBasePtr RaveCreateViewer(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyKinBodyPtr RaveCreateKinBody(PyEnvironmentBasePtr pyenv, const std::string& name);
    PyTrajectoryBasePtr RaveCreateTrajectory(PyEnvironmentBasePtr pyenv, const std::string& name);
    void init_openravepy_global();
}

#endif
