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
#ifndef OPENRAVEPY_INTERNAL_H
#define OPENRAVEPY_INTERNAL_H

#include <Python.h>

#include "openrave-core.h"

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

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
        if( !(p) ) { throw openrave_exception(boost::str(boost::format("[%s:%d]: invalid pointer")%__PRETTY_FUNCTION__%__LINE__)); } \
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
class PyModuleBase;
class PyViewerBase;
class PySpaceSamplerBase;
class PyConfigurationSpecification;

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
typedef boost::shared_ptr<PyPhysicsEngineBase> PyPhysicsEngineBasePtr;
typedef boost::shared_ptr<PyPhysicsEngineBase const> PyPhysicsEngineBaseConstPtr;
typedef boost::shared_ptr<PyCollisionCheckerBase> PyCollisionCheckerBasePtr;
typedef boost::shared_ptr<PyCollisionCheckerBase const> PyCollisionCheckerBaseConstPtr;
typedef boost::shared_ptr<PyCollisionReport> PyCollisionReportPtr;
typedef boost::shared_ptr<PyCollisionReport const> PyCollisionReportConstPtr;
typedef boost::shared_ptr<PyPlannerBase> PyPlannerBasePtr;
typedef boost::shared_ptr<PyPlannerBase const> PyPlannerBaseConstPtr;
typedef boost::shared_ptr<PySensorBase> PySensorBasePtr;
typedef boost::shared_ptr<PySensorBase const> PySensorBaseConstPtr;
typedef boost::shared_ptr<PySensorSystemBase> PySensorSystemBasePtr;
typedef boost::shared_ptr<PySensorSystemBase const> PySensorSystemBaseConstPtr;
typedef boost::shared_ptr<PyControllerBase> PyControllerBasePtr;
typedef boost::shared_ptr<PyControllerBase const> PyControllerBaseConstPtr;
typedef boost::shared_ptr<PyModuleBase> PyModuleBasePtr;
typedef boost::shared_ptr<PyModuleBase const> PyModuleBaseConstPtr;
typedef boost::shared_ptr<PyViewerBase> PyViewerBasePtr;
typedef boost::shared_ptr<PyViewerBase const> PyViewerBaseConstPtr;
typedef boost::shared_ptr<PySpaceSamplerBase> PySpaceSamplerBasePtr;
typedef boost::shared_ptr<PySpaceSamplerBase const> PySpaceSamplerBaseConstPtr;
typedef boost::shared_ptr<PyConfigurationSpecification> PyConfigurationSpecificationPtr;
typedef boost::shared_ptr<PyConfigurationSpecification const> PyConfigurationSpecificationConstPtr;

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

struct null_deleter { void operator()(void const *) const {
                      }
};

inline RaveVector<float> ExtractFloat3(const object& o)
{
    return RaveVector<float>(extract<float>(o[0]), extract<float>(o[1]), extract<float>(o[2]));
}

template <typename T>
inline Vector ExtractVector2Type(const object& o)
{
    return Vector(extract<T>(o[0]), extract<T>(o[1]),0);
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

inline Vector ExtractVector2(const object& oraw)
{
    return ExtractVector2Type<dReal>(oraw);
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
    else if( n == 4 ) {
        return ExtractVector4Type<T>(oraw);
    }
    throw openrave_exception("unexpected vector size");
}

template <typename T>
inline Transform ExtractTransformType(const object& o)
{
    if( len(o) == 7 ) {
        return Transform(Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])), Vector(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    }
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
    if( len(o) == 7 ) {
        return Transform(Vector(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])), Vector(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    }
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
    npy_intp dims[] = { 4,4};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2]; pdata[3] = t.trans.x;
    pdata[4] = t.m[4]; pdata[5] = t.m[5]; pdata[6] = t.m[6]; pdata[7] = t.trans.y;
    pdata[8] = t.m[8]; pdata[9] = t.m[9]; pdata[10] = t.m[10]; pdata[11] = t.trans.z;
    pdata[12] = 0; pdata[13] = 0; pdata[14] = 0; pdata[15] = 1;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArrayRotation(const TransformMatrix& t)
{
    npy_intp dims[] = { 3,3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
    pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
    pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray(const Transform& t)
{
    npy_intp dims[] = { 7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray3(const std::vector<RaveVector<float> >& v)
{
    npy_intp dims[] = { v.size(),3};
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
    npy_intp dims[] = { v.size(),3};
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

inline object toPyVector2(Vector v)
{
    return numeric::array(boost::python::make_tuple(v.x,v.y));
}

inline object toPyVector3(Vector v)
{
    return numeric::array(boost::python::make_tuple(v.x,v.y,v.z));
}

inline object toPyVector4(Vector v)
{
    return numeric::array(boost::python::make_tuple(v.x,v.y,v.z,v.w));
}

inline AttributesList toAttributesList(boost::python::dict odict)
{
    AttributesList atts;
    if( !!odict ) {
        boost::python::list iterkeys = (boost::python::list)odict.iterkeys();
        for (int i = 0; i < boost::python::len(iterkeys); i++) {
            // Because we know they're strings, we can do this
            std::string key = boost::python::extract<std::string>(iterkeys[i]);
            std::string value = boost::python::extract<std::string>(odict[iterkeys[i]]);
            atts.push_back(make_pair(key,value));
        }
    }
    return atts;
}

bool GetReturnTransformQuaternions();

template <typename T>
inline object ReturnTransform(T t)
{
    if( GetReturnTransformQuaternions() ) {
        return toPyArray(Transform(t));
    }
    else {
        return toPyArray(TransformMatrix(t));
    }
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
    PyGraphHandle() {
    }
    PyGraphHandle(GraphHandlePtr handle) : _handle(handle) {
    }
    virtual ~PyGraphHandle() {
    }

    void SetTransform(object otrans) {
        _handle->SetTransform(RaveTransform<float>(ExtractTransformMatrixType<float>(otrans)));
    }
    void SetShow(bool bshow) {
        _handle->SetShow(bshow);
    }
private:
    GraphHandlePtr _handle;
};


class PyUserData
{
public:
    PyUserData() {
    }
    PyUserData(UserDataPtr handle) : _handle(handle) {
    }
    void close() {
        _handle.reset();
    }
    UserDataPtr _handle;
};

class PyUserObject : public UserData
{
public:
    PyUserObject(object o) : _o(o) {
    }
    object _o;
};

class PyRay
{
public:
    PyRay() {
    }
    PyRay(object newpos, object newdir);
    PyRay(const RAY& newr) : r(newr) {
    }
    object dir();
    object pos();
    virtual string __repr__();
    virtual string __str__();
    RAY r;
};

object toPyGraphHandle(const GraphHandlePtr p);
object toPyUserData(UserDataPtr p);
bool ExtractIkParameterization(object o, IkParameterization& ikparam);
object toPyIkParameterization(const IkParameterization& ikparam);
object toPyAABB(const AABB& ab);
object toPyRay(const RAY& r);
RAY ExtractRay(object o);
bool ExtractRay(object o, RAY& r);
object toPyTriMesh(const KinBody::Link::TRIMESH& mesh);
bool ExtractTriMesh(object o, KinBody::Link::TRIMESH& mesh);

class PyInterfaceBase
{
protected:
    InterfaceBasePtr _pbase;
    PyEnvironmentBasePtr _pyenv;
public:
    PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv);
    virtual ~PyInterfaceBase() {
    }

    InterfaceType GetInterfaceType() const {
        return _pbase->GetInterfaceType();
    }
    string GetXMLId() const {
        return _pbase->GetXMLId();
    }
    string GetPluginName() const {
        return _pbase->GetPluginName();
    }
    string GetDescription() const {
        return _pbase->GetDescription();
    }
    void SetDescription(const std::string& s) {
        _pbase->SetDescription(s);
    }
    PyEnvironmentBasePtr GetEnv() const {
        return _pyenv;
    }

    void Clone(PyInterfaceBasePtr preference, int cloningoptions) {
        CHECK_POINTER(preference);
        _pbase->Clone(preference->GetInterfaceBase(),cloningoptions);
    }

    void SetUserData(PyUserData pdata) {
        _pbase->SetUserData(pdata._handle);
    }
    void SetUserData(object o) {
        _pbase->SetUserData(boost::shared_ptr<UserData>(new PyUserObject(o)));
    }
    object GetUserData() const {
        boost::shared_ptr<PyUserObject> po = boost::dynamic_pointer_cast<PyUserObject>(_pbase->GetUserData());
        if( !po ) {
            return object(PyUserData(_pbase->GetUserData()));
        }
        else {
            return po->_o;
        }
    }

    class PythonThreadSaver
    {
public:
        PythonThreadSaver() {
            _save = PyEval_SaveThread();
        }
        virtual ~PythonThreadSaver() {
            PyEval_RestoreThread(_save);
        }
protected:
        PyThreadState *_save;
    };

    object SendCommand(const string& in, bool releasegil=false) {
        boost::shared_ptr<PythonThreadSaver> statesaver;
        if( releasegil ) {
            statesaver.reset(new PythonThreadSaver());
        }
        stringstream sin(in), sout;
        sout << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        if( !_pbase->SendCommand(sout,sin) ) {
            return object();
        }
        return object(sout.str());
    }

    virtual string __repr__() {
        return boost::str(boost::format("<RaveCreateInterface(RaveGetEnvironment(%d),InterfaceType.%s,'%s')>")%RaveGetEnvironmentId(_pbase->GetEnv())%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual bool __eq__(PyInterfaceBasePtr p) {
        return !!p && _pbase == p->GetInterfaceBase();
    }
    virtual bool __ne__(PyInterfaceBasePtr p) {
        return !p || _pbase != p->GetInterfaceBase();
    }
    virtual InterfaceBasePtr GetInterfaceBase() {
        return _pbase;
    }
};

namespace openravepy
{

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr);
void LockEnvironment(PyEnvironmentBasePtr);
void UnlockEnvironment(PyEnvironmentBasePtr);

void init_openravepy_collisionchecker();
CollisionCheckerBasePtr GetCollisionChecker(PyCollisionCheckerBasePtr);
PyInterfaceBasePtr toPyCollisionChecker(CollisionCheckerBasePtr, PyEnvironmentBasePtr);
CollisionReportPtr GetCollisionReport(object);
CollisionReportPtr GetCollisionReport(PyCollisionReportPtr);
PyCollisionReportPtr toPyCollisionReport(CollisionReportPtr, PyEnvironmentBasePtr);
void UpdateCollisionReport(PyCollisionReportPtr, PyEnvironmentBasePtr);
void UpdateCollisionReport(object, PyEnvironmentBasePtr);
void init_openravepy_controller();
ControllerBasePtr GetController(PyControllerBasePtr);
PyInterfaceBasePtr toPyController(ControllerBasePtr, PyEnvironmentBasePtr);
void init_openravepy_iksolver();
IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr);
PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr, PyEnvironmentBasePtr);
void init_openravepy_kinbody();
KinBodyPtr GetKinBody(object);
KinBodyPtr GetKinBody(PyKinBodyPtr);
PyInterfaceBasePtr toPyKinBody(KinBodyPtr, PyEnvironmentBasePtr);
object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr);
KinBody::LinkPtr GetKinBodyLink(object);
KinBody::LinkConstPtr GetKinBodyLinkConst(object);
KinBody::JointPtr GetKinBodyJoint(object);
void init_openravepy_module();
ModuleBasePtr GetModule(PyModuleBasePtr);
PyInterfaceBasePtr toPyModule(ModuleBasePtr, PyEnvironmentBasePtr);
void init_openravepy_physicsengine();
PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr);
PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr, PyEnvironmentBasePtr);
void init_openravepy_planner();
PlannerBasePtr GetPlanner(PyPlannerBasePtr);
PyInterfaceBasePtr toPyPlanner(PlannerBasePtr, PyEnvironmentBasePtr);
//void init_openravepy_robot();
RobotBasePtr GetRobot(PyRobotBasePtr);
PyInterfaceBasePtr toPyRobot(RobotBasePtr, PyEnvironmentBasePtr);
object toPyRobotManipulator(RobotBase::ManipulatorPtr, PyEnvironmentBasePtr);
void init_openravepy_sensor();
SensorBasePtr GetSensor(PySensorBasePtr);
PyInterfaceBasePtr toPySensor(SensorBasePtr, PyEnvironmentBasePtr);
object toPySensorData(SensorBasePtr, PyEnvironmentBasePtr);
void init_openravepy_sensorsystem();
SensorSystemBasePtr GetSensorSystem(PySensorSystemBasePtr);
PyInterfaceBasePtr toPySensorSystem(SensorSystemBasePtr, PyEnvironmentBasePtr);
void init_openravepy_spacesampler();
SpaceSamplerBasePtr GetSpaceSampler(PySpaceSamplerBasePtr);
PyInterfaceBasePtr toPySpaceSampler(SpaceSamplerBasePtr, PyEnvironmentBasePtr);
void init_openravepy_trajectory();
TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr);
PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr, PyEnvironmentBasePtr);
PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr);
void init_openravepy_viewer();
ViewerBasePtr GetViewer(PyViewerBasePtr);
PyInterfaceBasePtr toPyViewer(ViewerBasePtr, PyEnvironmentBasePtr);

PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification&);
const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr);

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name);
void init_openravepy_global();

}

#endif
