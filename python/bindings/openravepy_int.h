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

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)

#define CHECK_POINTER(p) { \
        if( !(p) ) { throw openrave_exception(boost::str(boost::format(_("[%s:%d]: invalid pointer"))%__PRETTY_FUNCTION__%__LINE__)); } \
}

using namespace boost::python;
using namespace std;
using namespace OpenRAVE;

namespace openravepy {

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
class PyMultiControllerBase;
class PyTrajectoryBase;
class PyModuleBase;
class PyViewerBase;
class PySpaceSamplerBase;
class PyConfigurationSpecification;
class PyIkParameterization;
class PyXMLReadable;
class PyCameraIntrinsics;

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
typedef boost::shared_ptr<PyMultiControllerBase> PyMultiControllerBasePtr;
typedef boost::shared_ptr<PyMultiControllerBase const> PyMultiControllerBaseConstPtr;
typedef boost::shared_ptr<PyModuleBase> PyModuleBasePtr;
typedef boost::shared_ptr<PyModuleBase const> PyModuleBaseConstPtr;
typedef boost::shared_ptr<PyViewerBase> PyViewerBasePtr;
typedef boost::shared_ptr<PyViewerBase const> PyViewerBaseConstPtr;
typedef boost::shared_ptr<PySpaceSamplerBase> PySpaceSamplerBasePtr;
typedef boost::shared_ptr<PySpaceSamplerBase const> PySpaceSamplerBaseConstPtr;
typedef boost::shared_ptr<PyConfigurationSpecification> PyConfigurationSpecificationPtr;
typedef boost::shared_ptr<PyConfigurationSpecification const> PyConfigurationSpecificationConstPtr;
typedef boost::shared_ptr<PyIkParameterization> PyIkParameterizationPtr;
typedef boost::shared_ptr<PyXMLReadable> PyXMLReadablePtr;
typedef boost::shared_ptr<PyCameraIntrinsics> PyCameraIntrinsicsPtr;

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

#if OPENRAVE_RAPIDJSON
/// conversion between rapidjson value and pyobject
object toPyObject(const rapidjson::Value& value);
void toRapidJSONValue(object &obj, rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator);
#endif // OPENRAVE_RAPIDJSON

/// used externally, don't change definitions
//@{
Transform ExtractTransform(const object& oraw);
TransformMatrix ExtractTransformMatrix(const object& oraw);
object toPyArray(const TransformMatrix& t);
object toPyArray(const Transform& t);

XMLReadablePtr ExtractXMLReadable(object o);
object toPyXMLReadable(XMLReadablePtr p);
bool ExtractIkParameterization(object o, IkParameterization& ikparam);
object toPyIkParameterization(const IkParameterization& ikparam);
object toPyIkParameterization(const std::string& serializeddata);
//@}

struct null_deleter {
    void operator()(void const *) const {
    }
};

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

/// \brief release and restore the python GIL... no thread state saved..?
class PythonGILSaver
{
public:
    PythonGILSaver() {
        PyEval_ReleaseLock();
    }
    virtual ~PythonGILSaver() {
        PyEval_AcquireLock();
    }
};

class AutoPyArrayObjectDereferencer
{
public:
    AutoPyArrayObjectDereferencer(PyArrayObject* pyarrobj) : _pyarrobj(pyarrobj) {
    }
    ~AutoPyArrayObjectDereferencer() {
        Py_DECREF(_pyarrobj);
    }

private:
    PyArrayObject* _pyarrobj;
};

typedef boost::shared_ptr<PythonThreadSaver> PythonThreadSaverPtr;

inline RaveVector<float> ExtractFloat3(const object& o)
{
    return RaveVector<float>(extract<float>(o[0]), extract<float>(o[1]), extract<float>(o[2]));
}

template <typename T>
inline RaveVector<T> ExtractVector2Type(const object& o)
{
    return RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]),0);
}

template <typename T>
inline RaveVector<T> ExtractVector3Type(const object& o)
{
    return RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]));
}

template <typename T>
inline RaveVector<T> ExtractVector4Type(const object& o)
{
    return RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3]));
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
    throw openrave_exception(_("unexpected vector size"));
}

template <typename T>
inline RaveVector<T> ExtractVector(const object& oraw)
{
    int n = len(oraw);
    if( n > 4 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("unexpected vector size %d"),n,ORE_InvalidArguments);
    }
    Vector v;
    for(int i = 0; i < n; ++i) {
        v[i] = (T)extract<T>(oraw[i]);
    }
    return v;
}

template <typename T>
inline RaveTransform<T> ExtractTransformType(const object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])), RaveVector<T>(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    }
    RaveTransformMatrix<T> t;
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
inline RaveTransformMatrix<T> ExtractTransformMatrixType(const object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(extract<T>(o[0]), extract<T>(o[1]), extract<T>(o[2]), extract<T>(o[3])), RaveVector<T>(extract<T>(o[4]), extract<T>(o[5]), extract<T>(o[6])));
    }
    RaveTransformMatrix<T> t;
    for(int i = 0; i < 3; ++i) {
        object orow = o[i];
        t.m[4*i+0] = extract<T>(orow[0]);
        t.m[4*i+1] = extract<T>(orow[1]);
        t.m[4*i+2] = extract<T>(orow[2]);
        t.trans[i] = extract<T>(orow[3]);
    }
    return t;
}

inline object toPyArrayRotation(const TransformMatrix& t)
{
    npy_intp dims[] = {3,3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
    pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
    pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray3(const std::vector<RaveVector<float> >& v)
{
    npy_intp dims[] = { npy_intp(v.size()), npy_intp(3) };
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
    npy_intp dims[] = { npy_intp(v.size()), npy_intp(3) };
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

/// \brief converts dictionary of keyvalue pairs
AttributesList toAttributesList(boost::python::dict odict);
/// \brief converts list of tuples [(key,value),(key,value)], it is possible for keys to repeat
AttributesList toAttributesList(boost::python::list olist);
AttributesList toAttributesList(boost::python::object oattributes);

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
    void Close()
    {
        _handle.reset();
    }

private:
    GraphHandlePtr _handle;
};

class PyEnvironmentLockSaver
{
public:
    PyEnvironmentLockSaver(PyEnvironmentBasePtr pyenv, bool braw);
    ~PyEnvironmentLockSaver();
protected:
    PyEnvironmentBasePtr _pyenv;
};

typedef boost::shared_ptr<PyEnvironmentLockSaver> PyEnvironmentLockSaverPtr;

class PyUserData
{
public:
    PyUserData() {
    }
    PyUserData(UserDataPtr handle) : _handle(handle) {
    }
    virtual ~PyUserData() {
    }
    virtual void Close() {
        _handle.reset();
    }
    UserDataPtr _handle;
};

class PySerializableData : public PyUserData
{
public:
    class StringSerializableData : public SerializableData
    {
public:
        StringSerializableData(const std::string& data) : _data(data) {
        }

        virtual void Serialize(std::ostream& O, int options=0) const {
            O << _data;
        }

        virtual void Deserialize(std::istream& I) {
            // need to read the entire input
            stringbuf buf;
            I.get(buf, 0);
            _data = buf.str();
        }

        std::string _data;
    };

    PySerializableData() {
    }
    PySerializableData(const std::string& data) {
        _handle.reset(new StringSerializableData(data));
    }
    PySerializableData(SerializableDataPtr handle) : _handle(handle) {
    }
    void Close() {
        _handle.reset();
    }
    object Serialize(int options) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _handle->Serialize(ss,options);
        return object(ss.str());
    }
    void Deserialize(const std::string& s) {
        std::stringstream ss(s);
        _handle->Deserialize(ss);
    }
    SerializableDataPtr _handle;
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
    virtual boost::python::object __unicode__();
    RAY r;
};

object toPyGraphHandle(const GraphHandlePtr p);
object toPyUserData(UserDataPtr p);
void init_openravepy_ikparameterization();
object toPyAABB(const AABB& ab);
object toPyRay(const RAY& r);
RAY ExtractRay(object o);

/// \brief PyAABB -> AABB
AABB ExtractAABB(object o);
bool ExtractRay(object o, RAY& r);
object toPyTriMesh(const TriMesh& mesh);
bool ExtractTriMesh(object o, TriMesh& mesh);

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
    object GetDescription() const {
        return ConvertStringToUnicode(_pbase->GetDescription());
    }
    void SetDescription(const std::string& s) {
        _pbase->SetDescription(s);
    }
    PyEnvironmentBasePtr GetEnv() const;

    void Clone(PyInterfaceBasePtr preference, int cloningoptions) {
        CHECK_POINTER(preference);
        _pbase->Clone(preference->GetInterfaceBase(),cloningoptions);
    }

    void SetUserData(PyUserData pdata) {
        _pbase->SetUserData(std::string(), pdata._handle);
    }
    void SetUserData(const std::string& key, PyUserData pdata) {
        _pbase->SetUserData(key, pdata._handle);
    }
    void SetUserData(object o) {
        _pbase->SetUserData(std::string(), boost::shared_ptr<UserData>(new PyUserObject(o)));
    }
    void SetUserData(const std::string& key, object o) {
        _pbase->SetUserData(key, boost::shared_ptr<UserData>(new PyUserObject(o)));
    }
    bool RemoveUserData(const std::string& key) {
        return _pbase->RemoveUserData(key);
    }
    object GetUserData(const std::string& key=std::string()) const;

    bool SupportsCommand(const string& cmd);
    object SendCommand(const string& in, bool releasegil=false, bool lockenv=false);

#if OPENRAVE_RAPIDJSON
    bool SupportsJSONCommand(const string& cmd);
    object SendJSONCommand(const string& cmd, object input, bool releasegil=false, bool lockenv=false);
#endif // OPENRAVE_RAPIDJSON

    virtual object GetReadableInterfaces();
    virtual object GetReadableInterface(const std::string& xmltag);

    virtual void SetReadableInterface(const std::string& xmltag, object oreadable);

    virtual string __repr__() {
        return boost::str(boost::format("RaveCreateInterface(RaveGetEnvironment(%d),InterfaceType.%s,'%s')")%RaveGetEnvironmentId(_pbase->GetEnv())%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual boost::python::object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    virtual int __hash__() {
        return static_cast<int>(uintptr_t(_pbase.get()));
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

class PySensorGeometry
{
public:
    virtual ~PySensorGeometry() {
    }
    virtual SensorBase::SensorType GetType()=0;
    virtual SensorBase::SensorGeometryPtr GetGeometry()=0;
};

typedef boost::shared_ptr<PySensorGeometry> PySensorGeometryPtr;

PySensorGeometryPtr toPySensorGeometry(SensorBase::SensorGeometryPtr);

bool ExtractIkReturn(object o, IkReturn& ikfr);
object toPyIkReturn(const IkReturn& ret);

object GetUserData(UserDataPtr pdata);

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr);
EnvironmentBasePtr GetEnvironment(object);
void LockEnvironment(PyEnvironmentBasePtr);
void UnlockEnvironment(PyEnvironmentBasePtr);
int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv);
PyEnvironmentBasePtr RaveGetEnvironment(int id);

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
IkSolverBasePtr GetIkSolver(object);
IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr);
PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr, PyEnvironmentBasePtr);
object toPyIkSolver(IkSolverBasePtr, object);
void init_openravepy_kinbody();
KinBodyPtr GetKinBody(object);
KinBodyPtr GetKinBody(PyKinBodyPtr);
PyEnvironmentBasePtr GetPyEnvFromPyKinBody(object okinbody);
PyInterfaceBasePtr toPyKinBody(KinBodyPtr, PyEnvironmentBasePtr);
object toPyKinBody(KinBodyPtr, object opyenv);
object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr);
object toPyKinBodyLink(KinBody::LinkPtr plink, object opyenv);
KinBody::LinkPtr GetKinBodyLink(object);
KinBody::LinkConstPtr GetKinBodyLinkConst(object);
object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr);
KinBody::JointPtr GetKinBodyJoint(object);
std::string reprPyKinBodyJoint(object);
std::string strPyKinBodyJoint(object);
void init_openravepy_module();
ModuleBasePtr GetModule(PyModuleBasePtr);
PyInterfaceBasePtr toPyModule(ModuleBasePtr, PyEnvironmentBasePtr);
void init_openravepy_physicsengine();
PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr);
PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr, PyEnvironmentBasePtr);
void init_openravepy_planner();
PlannerBasePtr GetPlanner(PyPlannerBasePtr);
PyInterfaceBasePtr toPyPlanner(PlannerBasePtr, PyEnvironmentBasePtr);
PlannerBase::PlannerParametersPtr GetPlannerParameters(object);
PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(object);

object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params);
void init_openravepy_robot();
RobotBasePtr GetRobot(object);
RobotBasePtr GetRobot(PyRobotBasePtr);
PyInterfaceBasePtr toPyRobot(RobotBasePtr, PyEnvironmentBasePtr);
RobotBase::ManipulatorPtr GetRobotManipulator(object);
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
TrajectoryBasePtr GetTrajectory(object);
TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr);
PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr, PyEnvironmentBasePtr);
object toPyTrajectory(TrajectoryBasePtr, object opyenv);
PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr);
// input can be class derived from PyInterfaceBase
object toPyEnvironment(object opyinterface);
PyEnvironmentBasePtr toPyEnvironment(PyKinBodyPtr);
void init_openravepy_viewer();
ViewerBasePtr GetViewer(PyViewerBasePtr);
PyInterfaceBasePtr toPyViewer(ViewerBasePtr, PyEnvironmentBasePtr);

int pyGetIntFromPy(object olevel, int defaultvalue);

PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification&);
const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr);

PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>&);
PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>&);

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name);
void init_openravepy_global();
void InitPlanningUtils();

}

#endif
