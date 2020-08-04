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

#include <openrave-core.h>

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif // _WIN32

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
#ifndef USE_PYBIND11_PYTHON_BINDINGS
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/docstring_options.hpp>
#include <boost/python/slice.hpp>
#endif // USE_PYBIND11_PYTHON_BINDINGS
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#define OPENRAVE_BINDINGS_PYARRAY
#include <openravepy/bindings.h>
#include <openravepy/docstrings.h>

#define _(msgid) OpenRAVE::RaveGetLocalizedTextForDomain("openrave", msgid)

#define CHECK_POINTER(p) { \
        if( !(p) ) { throw openrave_exception(boost::str(boost::format(_("[%s:%d]: invalid pointer"))%__PRETTY_FUNCTION__%__LINE__)); } \
}

using namespace std;
using namespace OpenRAVE;

namespace openravepy {

/// conversion between rapidjson value and py::object
OPENRAVEPY_API py::object toPyObject(const rapidjson::Value& value);
OPENRAVEPY_API void toRapidJSONValue(const py::object &obj, rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator);

/// used externally, don't change definitions
//@{
OPENRAVEPY_API Transform ExtractTransform(const py::object& oraw);
OPENRAVEPY_API TransformMatrix ExtractTransformMatrix(const py::object& oraw);
OPENRAVEPY_API py::object toPyArray(const TransformMatrix& t);
OPENRAVEPY_API py::object toPyArray(const Transform& t);
OPENRAVEPY_API py::object toPyArray(const std::vector<KinBody::GeometryInfoPtr>& infos);
// OPENRAVEPY_API py::object toPyArray(std::vector<KinBody::GeometryInfoPtr>& infos);
OPENRAVEPY_API ReadablePtr ExtractReadable(py::object o);
OPENRAVEPY_API py::object toPyReadable(ReadablePtr p);
OPENRAVEPY_API bool ExtractIkParameterization(py::object o, IkParameterization& ikparam);
OPENRAVEPY_API py::object toPyIkParameterization(const IkParameterization& ikparam);
OPENRAVEPY_API py::object toPyIkParameterization(const std::string& serializeddata);
//@}


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
class PyReadable;
class PyCameraIntrinsics;
class PyLinkInfo;
class PyJointInfo;
class PyGeometryInfo;
class PyManipulatorInfo;
class PyAttachedSensorInfo;
class PyConnectedBodyInfo;
class PyLink;
class PyJoint;

typedef OPENRAVE_SHARED_PTR<PyInterfaceBase> PyInterfaceBasePtr;
typedef OPENRAVE_SHARED_PTR<PyInterfaceBase const> PyInterfaceBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyKinBody> PyKinBodyPtr;
typedef OPENRAVE_SHARED_PTR<PyKinBody const> PyKinBodyConstPtr;
typedef OPENRAVE_SHARED_PTR<PyRobotBase> PyRobotBasePtr;
typedef OPENRAVE_SHARED_PTR<PyRobotBase const> PyRobotBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyEnvironmentBase> PyEnvironmentBasePtr;
typedef OPENRAVE_SHARED_PTR<PyEnvironmentBase const> PyEnvironmentBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyIkSolverBase> PyIkSolverBasePtr;
typedef OPENRAVE_SHARED_PTR<PyIkSolverBase const> PyIkSolverBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyTrajectoryBase> PyTrajectoryBasePtr;
typedef OPENRAVE_SHARED_PTR<PyTrajectoryBase const> PyTrajectoryBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyPhysicsEngineBase> PyPhysicsEngineBasePtr;
typedef OPENRAVE_SHARED_PTR<PyPhysicsEngineBase const> PyPhysicsEngineBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionCheckerBase> PyCollisionCheckerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionCheckerBase const> PyCollisionCheckerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionReport> PyCollisionReportPtr;
typedef OPENRAVE_SHARED_PTR<PyCollisionReport const> PyCollisionReportConstPtr;
typedef OPENRAVE_SHARED_PTR<PyPlannerBase> PyPlannerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyPlannerBase const> PyPlannerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySensorBase> PySensorBasePtr;
typedef OPENRAVE_SHARED_PTR<PySensorBase const> PySensorBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySensorSystemBase> PySensorSystemBasePtr;
typedef OPENRAVE_SHARED_PTR<PySensorSystemBase const> PySensorSystemBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyControllerBase> PyControllerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyControllerBase const> PyControllerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyMultiControllerBase> PyMultiControllerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyMultiControllerBase const> PyMultiControllerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyModuleBase> PyModuleBasePtr;
typedef OPENRAVE_SHARED_PTR<PyModuleBase const> PyModuleBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyViewerBase> PyViewerBasePtr;
typedef OPENRAVE_SHARED_PTR<PyViewerBase const> PyViewerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PySpaceSamplerBase> PySpaceSamplerBasePtr;
typedef OPENRAVE_SHARED_PTR<PySpaceSamplerBase const> PySpaceSamplerBaseConstPtr;
typedef OPENRAVE_SHARED_PTR<PyConfigurationSpecification> PyConfigurationSpecificationPtr;
typedef OPENRAVE_SHARED_PTR<PyConfigurationSpecification const> PyConfigurationSpecificationConstPtr;
typedef OPENRAVE_SHARED_PTR<PyIkParameterization> PyIkParameterizationPtr;
typedef OPENRAVE_SHARED_PTR<PyReadable> PyReadablePtr;
typedef OPENRAVE_SHARED_PTR<PyCameraIntrinsics> PyCameraIntrinsicsPtr;
typedef OPENRAVE_SHARED_PTR<PyLinkInfo> PyLinkInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyJointInfo> PyJointInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyGeometryInfo> PyGeometryInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyManipulatorInfo> PyManipulatorInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyAttachedSensorInfo> PyAttachedSensorInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyConnectedBodyInfo> PyConnectedBodyInfoPtr;
typedef OPENRAVE_SHARED_PTR<PyLink> PyLinkPtr;
typedef OPENRAVE_SHARED_PTR<PyLink const> PyLinkConstPtr;
typedef OPENRAVE_SHARED_PTR<PyJoint> PyJointPtr;
typedef OPENRAVE_SHARED_PTR<PyJoint const> PyJointConstPtr;


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


struct OPENRAVEPY_API null_deleter {
    void operator()(void const *) const {
    }
};

class OPENRAVEPY_API PythonThreadSaver
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
class OPENRAVEPY_API PythonGILSaver
{
public:
    PythonGILSaver() {
        PyEval_ReleaseLock();
    }
    virtual ~PythonGILSaver() {
        PyEval_AcquireLock();
    }
};

class OPENRAVEPY_API AutoPyArrayObjectDereferencer
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

typedef OPENRAVE_SHARED_PTR<PythonThreadSaver> PythonThreadSaverPtr;

inline RaveVector<float> ExtractFloat3(const py::object& o)
{
    return RaveVector<float>(py::extract<float>(o[0]), py::extract<float>(o[1]), py::extract<float>(o[2]));
}

template <typename T>
inline RaveVector<T> ExtractVector2Type(const py::object& o)
{
    return RaveVector<T>(py::extract<T>(o[0]), py::extract<T>(o[1]),0);
}

template <typename T>
inline RaveVector<T> ExtractVector3Type(const py::object& o)
{
    return RaveVector<T>(py::extract<T>(o[0]), py::extract<T>(o[1]), py::extract<T>(o[2]));
}

template <typename T>
inline RaveVector<T> ExtractVector4Type(const py::object& o)
{
    return RaveVector<T>(py::extract<T>(o[0]), py::extract<T>(o[1]), py::extract<T>(o[2]), py::extract<T>(o[3]));
}

inline Vector ExtractVector2(const py::object& oraw)
{
    return ExtractVector2Type<dReal>(oraw);
}

inline Vector ExtractVector3(const py::object& oraw)
{
    return ExtractVector3Type<dReal>(oraw);
}

inline Vector ExtractVector4(const py::object& oraw)
{
    return ExtractVector4Type<dReal>(oraw);
}

template <typename T>
inline RaveVector<T> ExtractVector34(const py::object& oraw,T fdefaultw)
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
inline RaveVector<T> ExtractVector(const py::object& oraw)
{
    int n = len(oraw);
    if( n > 4 ) {
        throw OPENRAVE_EXCEPTION_FORMAT(_("unexpected vector size %d"),n,ORE_InvalidArguments);
    }
    Vector v;
    for(int i = 0; i < n; ++i) {
        v[i] = (T)py::extract<T>(oraw[i]);
    }
    return v;
}

template <typename T>
inline RaveTransform<T> ExtractTransformType(const py::object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(py::extract<T>(o[0]), py::extract<T>(o[1]), py::extract<T>(o[2]), py::extract<T>(o[3])), RaveVector<T>(py::extract<T>(o[4]), py::extract<T>(o[5]), py::extract<T>(o[6])));
    }
    RaveTransformMatrix<T> t;
    for(int i = 0; i < 3; ++i) {
        py::object orow = o[i];
        t.m[4*i+0] = py::extract<T>(orow[0]);
        t.m[4*i+1] = py::extract<T>(orow[1]);
        t.m[4*i+2] = py::extract<T>(orow[2]);
        t.trans[i] = py::extract<T>(orow[3]);
    }
    return t;
}

template <typename T>
inline RaveTransformMatrix<T> ExtractTransformMatrixType(const py::object& o)
{
    if( len(o) == 7 ) {
        return RaveTransform<T>(RaveVector<T>(py::extract<T>(o[0]), py::extract<T>(o[1]), py::extract<T>(o[2]), py::extract<T>(o[3])), RaveVector<T>(py::extract<T>(o[4]), py::extract<T>(o[5]), py::extract<T>(o[6])));
    }
    RaveTransformMatrix<T> t;
    for(int i = 0; i < 3; ++i) {
        py::object orow = o[i];
        t.m[4*i+0] = py::extract<T>(orow[0]);
        t.m[4*i+1] = py::extract<T>(orow[1]);
        t.m[4*i+2] = py::extract<T>(orow[2]);
        t.trans[i] = py::extract<T>(orow[3]);
    }
    return t;
}

inline py::object toPyArrayRotation(const TransformMatrix& t)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvalues({3, 3});
    py::buffer_info buf = pyvalues.request();
    dReal* pvalue = (dReal*) buf.ptr;
    pvalue[0] = t.m[0];
    pvalue[1] = t.m[1];
    pvalue[2] = t.m[2];
    pvalue[3] = t.m[4];
    pvalue[4] = t.m[5];
    pvalue[5] = t.m[6];
    pvalue[6] = t.m[8];
    pvalue[7] = t.m[9];
    pvalue[8] = t.m[10];
    return pyvalues;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = {3,3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
    pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
    pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
    return py::to_array_astype<dReal>(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

template <typename T>
inline py::object toPyArray3(const std::vector<RaveVector<T> >& v)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvalues({(int) v.size(), 3});
    py::buffer_info buf = pyvalues.request();
    dReal* pvalue = (dReal*) buf.ptr;
    for(const RaveVector<T>& vi : v) {
        *pvalue++ = vi.x;
        *pvalue++ = vi.y;
        *pvalue++ = vi.z;
    }
    return pyvalues;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { npy_intp(v.size()), npy_intp(3) };
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(T)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    if( v.size() > 0 ) {
        T* pf = (T*)PyArray_DATA(pyvalues);
        FOREACHC(it,v) {
            *pf++ = it->x;
            *pf++ = it->y;
            *pf++ = it->z;
        }
    }
    return py::to_array_astype<T>(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

inline py::object toPyVector2(Vector v)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvec({2});
    py::buffer_info buf = pyvec.request();
    dReal* pvec = (dReal*) buf.ptr;
    pvec[0] = v.x;
    pvec[1] = v.y;
    return pyvec;
#else
    const dReal arr[2] {v.x, v.y};
    return toPyArrayN(arr, 2);
#endif
}

inline py::object toPyVector3(Vector v)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvec({3});
    py::buffer_info buf = pyvec.request();
    dReal* pvec = (dReal*) buf.ptr;
    pvec[0] = v.x;
    pvec[1] = v.y;
    pvec[2] = v.z;
    return pyvec;
#else
    const dReal arr[3] {v.x, v.y, v.z};
    return toPyArrayN(arr, 3);
#endif
}

inline py::object toPyVector4(Vector v)
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvec({4});
    py::buffer_info buf = pyvec.request();
    dReal* pvec = (dReal*) buf.ptr;
    pvec[0] = v.x;
    pvec[1] = v.y;
    pvec[2] = v.z;
    pvec[3] = v.w;
    return pyvec;
#else
    const dReal arr[4] {v.x, v.y, v.z, v.w};
    return toPyArrayN(arr, 4);
#endif
}

/// \brief converts dictionary of keyvalue pairs
AttributesList toAttributesList(py::dict odict);
/// \brief converts list of tuples [(key,value),(key,value)], it is possible for keys to repeat
AttributesList toAttributesList(py::list olist);
AttributesList toAttributesList(py::object oattributes);

bool GetReturnTransformQuaternions();

template <typename T>
inline py::object ReturnTransform(T t)
{
    if( GetReturnTransformQuaternions() ) {
        return toPyArray(Transform(t));
    }
    else {
        return toPyArray(TransformMatrix(t));
    }
}

class OPENRAVEPY_API PyPluginInfo
{
public:
    PyPluginInfo(const PLUGININFO& info)
    {
        FOREACHC(it, info.interfacenames) {
            py::list names;
            FOREACHC(itname,it->second)
            names.append(*itname);
            interfacenames.append(py::make_tuple(it->first,names));
        }
        version = OPENRAVE_VERSION_STRING_FORMAT(info.version);
    }

    py::list interfacenames;
    std::string version;
};

class OPENRAVEPY_API PyGraphHandle
{
public:
    PyGraphHandle() {
    }
    PyGraphHandle(GraphHandlePtr handle) : _handle(handle) {
    }
    virtual ~PyGraphHandle() {
    }

    void SetTransform(py::object otrans) {
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

class OPENRAVEPY_API PyEnvironmentLockSaver
{
public:
    PyEnvironmentLockSaver(PyEnvironmentBasePtr pyenv, bool braw);
    ~PyEnvironmentLockSaver();
protected:
    PyEnvironmentBasePtr _pyenv;
};

typedef OPENRAVE_SHARED_PTR<PyEnvironmentLockSaver> PyEnvironmentLockSaverPtr;

class OPENRAVEPY_API PyUserData
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

class OPENRAVEPY_API PySerializableData : public PyUserData
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
    py::object Serialize(int options) {
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        _handle->Serialize(ss,options);
        return py::to_object(ss.str());
    }
    void Deserialize(const std::string& s) {
        std::stringstream ss(s);
        _handle->Deserialize(ss);
    }
    SerializableDataPtr _handle;
};

class OPENRAVEPY_API PyUserObject : public UserData
{
public:
    PyUserObject(py::object o) : _o(o) {
    }
    py::object _o;
};

class OPENRAVEPY_API PyRay
{
public:
    PyRay() {
    }
    PyRay(py::object newpos, py::object newdir);
    PyRay(const RAY& newr) : r(newr) {
    }
    py::object dir();
    py::object pos();
    virtual std::string __repr__();
    virtual std::string __str__();
    virtual py::object __unicode__();
    RAY r;
};

OPENRAVEPY_API py::object toPyGraphHandle(const GraphHandlePtr p);
OPENRAVEPY_API py::object toPyUserData(UserDataPtr p);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_ikparameterization(py::module& m);
#else
void init_openravepy_ikparameterization();
#endif
OPENRAVEPY_API py::object toPyAABB(const AABB& ab);
OPENRAVEPY_API py::object toPyRay(const RAY& r);
OPENRAVEPY_API RAY ExtractRay(py::object o);

/// \brief PyAABB -> AABB
OPENRAVEPY_API AABB ExtractAABB(py::object o);
OPENRAVEPY_API bool ExtractRay(py::object o, RAY& r);
OPENRAVEPY_API py::object toPyTriMesh(const TriMesh& mesh);
OPENRAVEPY_API bool ExtractTriMesh(py::object o, TriMesh& mesh);
OPENRAVEPY_API std::vector<KinBody::LinkInfoPtr> ExtractLinkInfoArray(py::object pyLinkInfoList);
OPENRAVEPY_API std::vector<KinBody::JointInfoPtr> ExtractJointInfoArray(py::object pyJointInfoList);
OPENRAVEPY_API KinBody::GrabbedInfoPtr ExtractGrabbedInfo(py::object pyGrabbedInfo);
OPENRAVEPY_API std::vector<KinBody::GrabbedInfoPtr> ExtractGrabbedInfoArray(py::object pyGrabbedInfoList);
OPENRAVEPY_API std::vector< std::pair< std::pair<std::string, int>, dReal>> ExtractDOFValuesArray(py::object pyDOFValuesList);
OPENRAVEPY_API std::map<std::string, ReadablePtr> ExtractReadableInterfaces(py::object pyReadableInterfaces);
OPENRAVEPY_API std::vector<RobotBase::AttachedSensorInfoPtr> ExtractAttachedSensorInfoArray(py::object pyAttachedSensorInfoList);
OPENRAVEPY_API std::vector<RobotBase::ManipulatorInfoPtr> ExtractManipulatorInfoArray(py::object pyManipList);
OPENRAVEPY_API std::vector<RobotBase::ConnectedBodyInfoPtr> ExtractConnectedBodyInfoArray(py::object pyConnectedBodyInfoList);
OPENRAVEPY_API py::object ReturnDOFValues(const std::vector<std::pair<std::pair<std::string, int>, dReal>>& vDOFValues);

class OPENRAVEPY_API PyInterfaceBase
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
    std::string GetXMLId() const {
        return _pbase->GetXMLId();
    }
    std::string GetPluginName() const {
        return _pbase->GetPluginName();
    }
    py::object GetDescription() const {
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
    void SetUserData(py::object o) {
        _pbase->SetUserData(std::string(), OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
    }
    void SetUserData(const std::string& key, py::object o) {
        _pbase->SetUserData(key, OPENRAVE_SHARED_PTR<UserData>(new PyUserObject(o)));
    }
    bool RemoveUserData(const std::string& key) {
        return _pbase->RemoveUserData(key);
    }
    py::object GetUserData(const std::string& key=std::string()) const;

    bool SupportsCommand(const string& cmd);
    py::object SendCommand(const string& in, bool releasegil=false, bool lockenv=false);

    bool SupportsJSONCommand(const string& cmd);
    py::object SendJSONCommand(const string& cmd, py::object input, bool releasegil=false, bool lockenv=false);

    virtual py::object GetReadableInterfaces();
    virtual py::object GetReadableInterface(const std::string& xmltag);

    virtual void SetReadableInterface(const std::string& xmltag, py::object oreadable);

    virtual string __repr__() {
        return boost::str(boost::format("RaveCreateInterface(RaveGetEnvironment(%d),InterfaceType.%s,'%s')")%RaveGetEnvironmentId(_pbase->GetEnv())%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId());
    }
    virtual py::object __unicode__() {
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

class OPENRAVEPY_API PySensorGeometry
{
public:
    virtual ~PySensorGeometry() {
    }
    virtual SensorBase::SensorType GetType()=0;
    virtual SensorBase::SensorGeometryPtr GetGeometry()=0;
};

typedef OPENRAVE_SHARED_PTR<PySensorGeometry> PySensorGeometryPtr;

OPENRAVEPY_API PySensorGeometryPtr toPySensorGeometry(const std::string& sensorname, const rapidjson::Document& docSensorGeometry);

OPENRAVEPY_API bool ExtractIkReturn(py::object o, IkReturn& ikfr);
OPENRAVEPY_API py::object toPyIkReturn(const IkReturn& ret);

OPENRAVEPY_API py::object GetUserData(UserDataPtr pdata);

OPENRAVEPY_API EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr);
OPENRAVEPY_API EnvironmentBasePtr GetEnvironment(py::object);
OPENRAVEPY_API void LockEnvironment(PyEnvironmentBasePtr);
OPENRAVEPY_API void UnlockEnvironment(PyEnvironmentBasePtr);
OPENRAVEPY_API int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv);
OPENRAVEPY_API PyEnvironmentBasePtr RaveGetEnvironment(int id);

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_collisionchecker(py::module& m);
#else
void init_openravepy_collisionchecker();
#endif
OPENRAVEPY_API CollisionCheckerBasePtr GetCollisionChecker(PyCollisionCheckerBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyCollisionChecker(CollisionCheckerBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API CollisionReportPtr GetCollisionReport(py::object);
OPENRAVEPY_API CollisionReportPtr GetCollisionReport(PyCollisionReportPtr);
OPENRAVEPY_API PyCollisionReportPtr toPyCollisionReport(CollisionReportPtr, PyEnvironmentBasePtr);
OPENRAVEPY_API void UpdateCollisionReport(PyCollisionReportPtr, PyEnvironmentBasePtr);
OPENRAVEPY_API void UpdateCollisionReport(py::object, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_controller(py::module& m);
#else
void init_openravepy_controller();
#endif
OPENRAVEPY_API ControllerBasePtr GetController(PyControllerBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyController(ControllerBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_iksolver(py::module& m);
#else
void init_openravepy_iksolver();
#endif
OPENRAVEPY_API IkSolverBasePtr GetIkSolver(py::object);
OPENRAVEPY_API IkSolverBasePtr GetIkSolver(PyIkSolverBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyIkSolver(IkSolverBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API py::object toPyIkSolver(IkSolverBasePtr, py::object);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_kinbody(py::module& m);
#else
void init_openravepy_kinbody();
#endif
OPENRAVEPY_API KinBodyPtr GetKinBody(py::object);
OPENRAVEPY_API KinBodyPtr GetKinBody(PyKinBodyPtr);
OPENRAVEPY_API PyEnvironmentBasePtr GetPyEnvFromPyKinBody(py::object okinbody);
OPENRAVEPY_API PyInterfaceBasePtr toPyKinBody(KinBodyPtr, PyEnvironmentBasePtr);
OPENRAVEPY_API py::object toPyKinBody(KinBodyPtr, py::object opyenv);
OPENRAVEPY_API py::object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr);
OPENRAVEPY_API py::object toPyKinBodyLink(KinBody::LinkPtr plink, py::object opyenv);
OPENRAVEPY_API KinBody::LinkPtr GetKinBodyLink(py::object);
OPENRAVEPY_API KinBody::LinkConstPtr GetKinBodyLinkConst(py::object);
OPENRAVEPY_API py::object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr);
OPENRAVEPY_API KinBody::JointPtr GetKinBodyJoint(py::object);
OPENRAVEPY_API std::string reprPyKinBodyJoint(py::object);
OPENRAVEPY_API std::string strPyKinBodyJoint(py::object);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_module(py::module& m);
#else
void init_openravepy_module();
#endif
OPENRAVEPY_API ModuleBasePtr GetModule(PyModuleBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyModule(ModuleBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_physicsengine(py::module& m);
#else
void init_openravepy_physicsengine();
#endif
OPENRAVEPY_API PhysicsEngineBasePtr GetPhysicsEngine(PyPhysicsEngineBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyPhysicsEngine(PhysicsEngineBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_planner(py::module& m);
#else
void init_openravepy_planner();
#endif
OPENRAVEPY_API PlannerBasePtr GetPlanner(PyPlannerBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyPlanner(PlannerBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API PlannerBase::PlannerParametersPtr GetPlannerParameters(py::object);
OPENRAVEPY_API PlannerBase::PlannerParametersConstPtr GetPlannerParametersConst(py::object);

OPENRAVEPY_API py::object toPyPlannerParameters(PlannerBase::PlannerParametersPtr params);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_robot(py::module& m);
#else
void init_openravepy_robot();
#endif
OPENRAVEPY_API RobotBasePtr GetRobot(py::object);
OPENRAVEPY_API RobotBasePtr GetRobot(PyRobotBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyRobot(RobotBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API RobotBase::ManipulatorPtr GetRobotManipulator(py::object);
OPENRAVEPY_API py::object toPyRobotManipulator(RobotBase::ManipulatorPtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_sensor(py::module& m);
#else
void init_openravepy_sensor();
#endif
OPENRAVEPY_API SensorBasePtr GetSensor(PySensorBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPySensor(SensorBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API py::object toPySensorData(SensorBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_sensorsystem(py::module& m);
#else
void init_openravepy_sensorsystem();
#endif
OPENRAVEPY_API SensorSystemBasePtr GetSensorSystem(PySensorSystemBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPySensorSystem(SensorSystemBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_spacesampler(py::module& m);
#else
void init_openravepy_spacesampler();
#endif
OPENRAVEPY_API SpaceSamplerBasePtr GetSpaceSampler(PySpaceSamplerBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPySpaceSampler(SpaceSamplerBasePtr, PyEnvironmentBasePtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_trajectory(py::module& m);
#else
void init_openravepy_trajectory();
#endif
OPENRAVEPY_API TrajectoryBasePtr GetTrajectory(py::object);
OPENRAVEPY_API TrajectoryBasePtr GetTrajectory(PyTrajectoryBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyTrajectory(TrajectoryBasePtr, PyEnvironmentBasePtr);
OPENRAVEPY_API py::object toPyTrajectory(TrajectoryBasePtr, py::object opyenv);
OPENRAVEPY_API PyEnvironmentBasePtr toPyEnvironment(PyTrajectoryBasePtr);
// input can be class derived from PyInterfaceBase
OPENRAVEPY_API py::object toPyEnvironment(py::object opyinterface);
OPENRAVEPY_API PyEnvironmentBasePtr toPyEnvironment(PyKinBodyPtr);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_viewer(py::module& m);
#else
void init_openravepy_viewer();
#endif
OPENRAVEPY_API ViewerBasePtr GetViewer(PyViewerBasePtr);
OPENRAVEPY_API PyInterfaceBasePtr toPyViewer(ViewerBasePtr, PyEnvironmentBasePtr);

OPENRAVEPY_API int pyGetIntFromPy(py::object olevel, int defaultvalue);
OPENRAVEPY_API py::object toPyPlannerStatus(const PlannerStatus&);
    
OPENRAVEPY_API PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification&);
OPENRAVEPY_API const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr);

OPENRAVEPY_API PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<float>&);
OPENRAVEPY_API PyCameraIntrinsicsPtr toPyCameraIntrinsics(const geometry::RaveCameraIntrinsics<double>&);

OPENRAVEPY_API PyLinkPtr toPyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv);
OPENRAVEPY_API PyJointPtr toPyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv);
OPENRAVEPY_API PyLinkInfoPtr toPyLinkInfo(const KinBody::LinkInfo& linkinfo);
OPENRAVEPY_API PyJointInfoPtr toPyJointInfo(const KinBody::JointInfo& jointinfo);
OPENRAVEPY_API PyGeometryInfoPtr toPyGeometryInfo(const KinBody::GeometryInfo& geominfo);
OPENRAVEPY_API PyManipulatorInfoPtr toPyManipulatorInfo(const RobotBase::ManipulatorInfo& manipulatorinfo);
OPENRAVEPY_API PyAttachedSensorInfoPtr toPyAttachedSensorInfo(const RobotBase::AttachedSensorInfo& attachedSensorinfo);
OPENRAVEPY_API PyConnectedBodyInfoPtr toPyConnectedBodyInfo(const RobotBase::ConnectedBodyInfo& connectedBodyInfo);

OPENRAVEPY_API PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_global(py::module& m);
#else
void init_openravepy_global();
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
void InitPlanningUtils(py::module& m);
#else
void InitPlanningUtils();
#endif

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_H
