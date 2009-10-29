// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#include "libopenrave-core/openrave-core.h"
#include <Python.h>

#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>
#include <exception>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <sstream>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

#define PY_ARRAY_UNIQUE_SYMBOL PyArrayHandle
#include <boost/python.hpp>
#include <boost/python/exception_translator.hpp>
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#define CHECK_POINTER(p) { \
    if( (p) == NULL ) throw openrave_exception("invalid pointer"); \
}

using namespace boost::python;
using namespace std;
using namespace OpenRAVE;

inline std::wstring _stdmbstowcs(const char* pstr)
{
    size_t len = mbstowcs(NULL, pstr, 0);
    std::wstring w; w.resize(len);
    mbstowcs(&w[0], pstr, len);
    return w;
}

inline std::string _stdwcstombs(const wchar_t* pname)
{
    if( pname == NULL )
        return std::string();

    std::string s;
    size_t len = wcstombs(NULL, pname, 0);
    if( len != (size_t)-1 ) {
        s.resize(len);
        wcstombs(&s[0], pname, len);
    }

    return s;
}

uint64_t GetMicroTime()
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

/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
struct DummyStruct {};

class PyEnvironmentBase;
class PyCollisionReport;
class PyIkSolverBase;
class PySensorBase;
class PyControllerBase;
class PyTrajectoryBase;

struct openrave_exception : std::exception
{
    openrave_exception() : std::exception(), _s("unknown exception") {}
    openrave_exception(const string& s) : std::exception() { _s = "OpenRAVE: " + s; }
    virtual ~openrave_exception() throw() {}
    char const* what() const throw() { return _s.c_str(); }
    string _s;
};

void translate_openrave_exception(openrave_exception const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

class LockEnvironment
{
public:
    LockEnvironment(EnvironmentBase* penv) : _penv(penv) { _penv->LockPhysics(true); }
    ~LockEnvironment() { _penv->LockPhysics(false); }
private:
    EnvironmentBase* _penv;
};

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
    object o = oraw.attr("flat");

    // check the types of o
    extract<dReal> xr(o[0]);
    if( xr.check() )
        return ExtractVector3Type<dReal>(o);
#if OPENRAVE_PRECISION
    extract<float> xf(o[0]);
    if( xf.check() )
        return ExtractVector3Type<float>(o);
#else
    extract<double> xd(o[0]);
    if( xd.check() )
        return ExtractVector3Type<double>(o);
#endif
    // continue
    extract<int> xi(o[0]);
    if( xi.check() )
        return ExtractVector3Type<int>(o);
    extract<unsigned int> xu(o[0]);
    if( xu.check() )
        return ExtractVector3Type<unsigned int>(o);

    object onew = ((numeric::array)oraw).astype("f8").attr("flat");
    return ExtractVector3Type<double>(onew); // python should raise exception
}

inline Vector ExtractVector4(const object& oraw)
{
    object o = oraw.attr("flat");

    // check the types of o
    extract<dReal> xr(o[0]);
    if( xr.check() )
        return ExtractVector4Type<dReal>(o);
#if OPENRAVE_PRECISION
    extract<float> xf(o[0]);
    if( xf.check() )
        return ExtractVector4Type<float>(o);
#else
    extract<double> xd(o[0]);
    if( xd.check() )
        return ExtractVector4Type<double>(o);
#endif
    // continue
    extract<int> xi(o[0]);
    if( xi.check() )
        return ExtractVector4Type<int>(o);
    extract<unsigned int> xu(o[0]);
    if( xu.check() )
        return ExtractVector4Type<unsigned int>(o);

    object onew = ((numeric::array)oraw).astype("f8").attr("flat");
    return ExtractVector4Type<double>(onew); // python should raise exception
}

inline Vector ExtractVector34(const object& oraw)
{
    int n = len(oraw);
    if( n == 3 )
        return ExtractVector3(oraw);
    else if( n == 4 )
        return ExtractVector4(oraw);
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
        t.m[4*i+0] = extract<T>(o[4*i+0]);
        t.m[4*i+1] = extract<T>(o[4*i+1]);
        t.m[4*i+2] = extract<T>(o[4*i+2]);
        t.trans[i] = extract<T>(o[4*i+3]);
    }
    return t;
}

inline Transform ExtractTransform(const object& oraw)
{
    object o = oraw.attr("flat");
    // check the types of o
    extract<dReal> xr(o[0]);
    if( xr.check() )
        return ExtractTransformType<dReal>(o);
#if OPENRAVE_PRECISION
    extract<float> xf(o[0]);
    if( xf.check() )
        return ExtractTransformType<float>(o);
#else
    extract<double> xd(o[0]);
    if( xd.check() )
        return ExtractTransformType<double>(o);
#endif
    // continue
    extract<int> xi(o[0]);
    if( xi.check() )
        return ExtractTransformType<int>(o);
    extract<unsigned int> xu(o[0]);
    if( xu.check() )
        return ExtractTransformType<unsigned int>(o);

    object onew = ((numeric::array)oraw).astype("f8").attr("flat");
    return ExtractTransformType<double>(onew);
}

template <typename T>
inline vector<T> ExtractArray(const object& o)
{
    vector<T> v(len(o));
    for(size_t i = 0; i < v.size(); ++i)
        v[i] = extract<T>(o[i]);
    return v;
}

inline vector<dReal> ExtractRealArray(const object& o)
{
    // check the types of o
    extract<dReal> xr(o[0]);
    if( xr.check() )
        return ExtractArray<dReal>(o);

    vector<dReal> v(len(o));
#if OPENRAVE_PRECISION
    extract<float> xf(o[0]);
    if( xf.check() ) {
        for(size_t i = 0; i < v.size(); ++i)
            v[i] = (dReal)(extract<float>(o[i]));
        return v;
    }
#else
    extract<double> xd(o[0]);
    if( xd.check() ) {
        for(size_t i = 0; i < v.size(); ++i)
            v[i] = (dReal)(extract<double>(o[i]));
        return v;
    }
#endif
    
    object onew = ((numeric::array)o).astype("f8");
    for(size_t i = 0; i < v.size(); ++i)
        v[i] = (dReal)(extract<double>(onew[i]));
    return v;
}

inline vector<float> ExtractFloatArray(const object& o)
{
    // check the types of o
    extract<float> xr(o[0]);
    if( xr.check() )
        return ExtractArray<float>(o);

    vector<float> v(len(o));
    object onew = ((numeric::array)o).astype("f8");
    for(size_t i = 0; i < v.size(); ++i)
        v[i] = (float)(extract<double>(onew[i]));
    return v;
}

inline vector<int> ExtractArrayInt(const object& o)
{
    // check the types of o
    extract<int> xi(o[0]);
    if( xi.check() )
        return ExtractArray<int>(o);
    extract<int64_t> xi64(o[0]);
    if( xi64.check() ) {
        vector<int> v(len(o));
        for(size_t i = 0; i < v.size(); ++i)
            v[i] = extract<int64_t>(o[i]);
    }
    return ExtractArray<int>(o);
}

template <typename T>
inline set<T> ExtractSet(const object& o)
{
    set<T> v;
    size_t nlen = len(o);
    for(size_t i = 0; i < nlen; ++i)
        v.insert(extract<T>(o[i]));
    return v;
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

inline object toPyArray(const Transform& t)
{
    npy_intp dims[] = {7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArray3(const vector<RaveVector<float> >& v)
{
    npy_intp dims[] = {v.size(),3};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, PyArray_FLOAT);
    if( v.size() > 0 ) {
        float* pf = (float*)PyArray_DATA(pyvalues);
        FOREACH(it,v) {
            *pf++ = it->x;
            *pf++ = it->y;
            *pf++ = it->z;
        }
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

template <typename T>
inline object ReturnTransform(T t)
{
    if( s_bReturnTransformQuaternions) 
        return toPyArray(Transform(t));
    else
        return toPyArray(TransformMatrix(t));
}

inline object toPyVector3(Vector v)
{
    numeric::array arr(make_tuple(v.x,v.y,v.z));
    arr.resize(3,1);
    return arr;
}

inline object toPyVector4(Vector v)
{
    numeric::array arr(make_tuple(v.x,v.y,v.z,v.w));
    arr.resize(4,1);
    return arr;
}

inline object toPyArrayN(const float* pvalues, int N)
{
    npy_intp dims[] = {N};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_FLOAT);
    if( pvalues != NULL )
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(float));
    return static_cast<numeric::array>(handle<>(pyvalues));
}

inline object toPyArrayN(const double* pvalues, int N)
{
    npy_intp dims[] = {N};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, PyArray_DOUBLE);
    if( pvalues != NULL )
        memcpy(PyArray_DATA(pyvalues),pvalues,N*sizeof(double));
    return static_cast<numeric::array>(handle<>(pyvalues));
}

template <typename T>
inline object toPyList(const vector<T>& v)
{
    boost::python::list lvalues;
    FOREACH(it,v)
        lvalues.append(object(*it));
    return lvalues;
}

template <typename T>
inline object toPyArray(const vector<T>& v)
{
    if( v.size() == 0 )
        return object();
    return toPyArrayN(&v[0],v.size());
}

class PyPluginInfo
{
public:
    PyPluginInfo(const PLUGININFO& info)
    {
        FOREACH(it, info.robots)
            robots.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.planners)
            planners.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.sensorsystems)
            sensorsystems.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.controllers)
            controllers.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.problems)
            problems.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.iksolvers)
            iksolvers.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.physicsengines)
            physicsengines.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.sensors)
            sensors.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.collisioncheckers)
            collisioncheckers.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.trajectories)
            trajectories.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.viewers)
            viewers.append(_stdwcstombs(it->c_str()));
        FOREACH(it, info.servers)
            servers.append(_stdwcstombs(it->c_str()));
    }

    boost::python::list robots, planners, sensorsystems, controllers, problems, iksolvers, physicsengines,sensors, collisioncheckers, trajectories, viewers, servers;
};

class PyGraphHandle
{
public:
    PyGraphHandle() : _handle(NULL) {}
    PyGraphHandle(void* handle) : _handle(handle) {}
    void* _handle;
};

class PyRay
{
public:
    PyRay() {}
    PyRay(object newpos, object newdir) {
        r.pos = ExtractVector3(newpos);
        r.dir = ExtractVector3(newdir);
    }
    PyRay(const RAY& newr) : r(newr) {}

    object dir() { return toPyVector3(r.dir); }
    object pos() { return toPyVector3(r.pos); }

    RAY r;
};

class Ray_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyRay& r)
    {
        return make_tuple(toPyVector3(r.r.pos),toPyVector3(r.r.dir));
    }
};
class PyAABB
{
public:
    PyAABB() {}
    PyAABB(object newpos, object newextents) {
        ab.pos = ExtractVector3(newpos);
        ab.extents = ExtractVector3(newextents);
    }
    PyAABB(const AABB& newab) : ab(newab) {}

    object extents() { return toPyVector3(ab.extents); }
    object pos() { return toPyVector3(ab.pos); }

    AABB ab;
};

class AABB_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyAABB& ab)
    {
        return make_tuple(toPyVector3(ab.ab.pos),toPyVector3(ab.ab.extents));
    }
};

class PyInterfaceBase
{
    friend class PyEnvironmentBase;
protected:
    InterfaceBase* _pbase;
    PyEnvironmentBase* _pyenv;
    bool _bOwnObject;
public:
    PyInterfaceBase(InterfaceBase* pbase, PyEnvironmentBase* pyenv, bool bOwnObject);
    virtual ~PyInterfaceBase();

    string GetXMLId() const { CHECK_POINTER(_pbase); return _pbase->GetXMLId(); }
    string GetPluginName() const {  CHECK_POINTER(_pbase); return _pbase->GetPluginName(); }
    PyEnvironmentBase* GetEnv() const { return _pyenv; }
    
    bool Clone(PyInterfaceBase* preference, int cloningoptions) {
        CHECK_POINTER(_pbase); 
        return _pbase->Clone(preference->GetInterfaceBase(),cloningoptions);
    }

    void SetUserData(uintptr_t pdata) { CHECK_POINTER(_pbase); return _pbase->SetUserData((void*)pdata); }
    uintptr_t GetUserData() const { CHECK_POINTER(_pbase); return (uintptr_t)_pbase->GetUserData(); }

    virtual InterfaceBase* GetInterfaceBase() { return _pbase; }
    virtual void Invalidate() { _pbase = NULL; _bOwnObject = false; }
};

class PyKinBody : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
protected:
    KinBody* _pbody;
    virtual void Invalidate() { _pbody = NULL; PyInterfaceBase::Invalidate(); }
public:
    class PyLink
    {
        friend class PyEnvironmentBase;
        KinBody::Link* _plink;
        PyEnvironmentBase* _pyenv;
    public:
        class PyTriMesh
        {
        public:
            PyTriMesh(const KinBody::Link::TRIMESH& mesh) {
                npy_intp dims[] = {mesh.vertices.size(),3};
                PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
                dReal* pvdata = (dReal*)PyArray_DATA(pyvertices);
                FOREACH(itv, mesh.vertices) {
                    *pvdata++ = itv->x;
                    *pvdata++ = itv->y;
                    *pvdata++ = itv->z;
                }
                vertices = static_cast<numeric::array>(handle<>(pyvertices));

                dims[0] = mesh.indices.size()/3;
                dims[1] = 3;
                PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT);
                int* pidata = (int*)PyArray_DATA(pyindices);
                FOREACH(it, mesh.indices)
                    *pidata++ = *it;
                indices = static_cast<numeric::array>(handle<>(pyindices));
            }
            object vertices,indices;
        };

        PyLink(KinBody::Link* plink, PyEnvironmentBase* pyenv) : _plink(plink), _pyenv(pyenv) {}
        virtual ~PyLink() {}

        KinBody::Link* GetLink() { return _plink; }

        void init(KinBody::Link* plink, PyEnvironmentBase* pyenv) { _plink=plink; _pyenv=pyenv; }
        string GetName() { return _stdwcstombs(_plink->GetName()); }
        int GetIndex() { return _plink->GetIndex(); }
        bool IsEnabled() const { return _plink->IsEnabled(); }
        bool IsStatic() const { return _plink->IsStatic(); }
        void Enable(bool bEnable) { _plink->Enable(bEnable); }

        PyKinBody* GetParent() const { return new PyKinBody(_plink->GetParent(),_pyenv,false); }
        
        PyTriMesh* GetCollisionData() { return new PyTriMesh(_plink->GetCollisionData()); }
        PyAABB* ComputeAABB() const { return new PyAABB(_plink->ComputeAABB()); }
        object GetTransform() const { return ReturnTransform(_plink->GetTransform()); }
        
        object GetCOMOffset() const { return toPyVector3(_plink->GetCOMOffset()); }
        object GetInertia() const { return ReturnTransform(_plink->GetInertia()); }
        dReal GetMass() const { return _plink->GetMass(); }

        void SetTransform(object otrans) { _plink->SetTransform(ExtractTransform(otrans)); }
        void SetForce(object oforce, object opos, bool bAdd) { return _plink->SetForce(ExtractVector3(oforce),ExtractVector3(opos),bAdd); }
        void SetTorque(object otorque, bool bAdd) { return _plink->SetTorque(ExtractVector3(otorque),bAdd); }
    };

    class PyJoint
    {
        friend class PyEnvironmentBase;
        KinBody::Joint* _pjoint;
        PyEnvironmentBase* _pyenv;
    public:
        PyJoint(KinBody::Joint* pjoint, PyEnvironmentBase* pyenv) : _pjoint(pjoint), _pyenv(pyenv) {}
        virtual ~PyJoint() {}

        KinBody::Joint* GetJoint() { return _pjoint; }

        string GetName() { return _stdwcstombs(_pjoint->GetName()); }
        int GetMimicJointIndex() const { return _pjoint->GetMimicJointIndex(); }
        object GetMimicCoeffs() const { return toPyArrayN(_pjoint->GetMimicCoeffs(),2); }

        dReal GetMaxVel() const { return _pjoint->GetMaxVel(); }
        dReal GetMaxAccel() const { return _pjoint->GetMaxAccel(); }
        dReal GetMaxTorque() const { return _pjoint->GetMaxTorque(); }

        int GetDOFIndex() const { return _pjoint->GetDOFIndex(); }
        int GetJointIndex() const { return _pjoint->GetJointIndex(); }
        
        PyKinBody* GetParent() const { return new PyKinBody(_pjoint->GetParent(),_pyenv,false); }

        PyLink* GetFirstAttached() const { return new PyLink(_pjoint->GetFirstAttached(), _pyenv); }
        PyLink* GetSecondAttached() const { return new PyLink(_pjoint->GetSecondAttached(), _pyenv); }

        int GetType() const { return _pjoint->GetType(); }

        int GetDOF() const { return _pjoint->GetDOF(); }
        object GetValues() const
        {
            if( GetDOF() == 0 )
                return object();
            npy_intp dims[] = {GetDOF()};
            PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            _pjoint->GetValues((dReal*)PyArray_DATA(pyvalues));
            return static_cast<numeric::array>(handle<>(pyvalues));
        }
        object GetVelocities() const
        {
            if( GetDOF() == 0 )
                return object();
            npy_intp dims[] = {GetDOF()};
            PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            _pjoint->GetVelocities((dReal*)PyArray_DATA(pyvalues));
            return static_cast<numeric::array>(handle<>(pyvalues));
        }

        object GetAnchor() const { return toPyVector3(_pjoint->GetAnchor()); }
        object GetAxis(int iaxis) { return toPyVector3(_pjoint->GetAxis(iaxis)); }
        object GetLimits() const {
            if( GetDOF() == 0 )
                return object();
            npy_intp dims[] = {GetDOF()};
            PyObject *pyvalues1 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            PyObject *pyvalues2 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            _pjoint->GetLimits((dReal*)PyArray_DATA(pyvalues1), (dReal*)PyArray_DATA(pyvalues2));
            return make_tuple(static_cast<numeric::array>(handle<>(pyvalues1)), static_cast<numeric::array>(handle<>(pyvalues2)));
        }
    };

    PyKinBody(KinBody* pbody, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pbody,pyenv, bOwnObject), _pbody(pbody) {}
    PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv,false) { _pbody = r._pbody; }
    virtual ~PyKinBody();
    KinBody* GetBody() { return _pbody; }

    bool Init(const string& filename)
    {
        CHECK_POINTER(_pbody);
        return _pbody->Init(filename.c_str(),NULL);
    }

    string GetName() const { CHECK_POINTER(_pbody); return _stdwcstombs(_pbody->GetName()); }
    int GetDOF() const { CHECK_POINTER(_pbody); return _pbody->GetDOF(); }

    object GetJointValues() const
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return object();
        npy_intp dims[] = {_pbody->GetDOF()};
        PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _pbody->GetJointValues((dReal*)PyArray_DATA(pyvalues));
        return static_cast<numeric::array>(handle<>(pyvalues));
    }
    object GetJointLimits() const
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return object();
        npy_intp dims[] = {_pbody->GetDOF()};
        PyObject *pyvalues1 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        PyObject *pyvalues2 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _pbody->GetJointLimits((dReal*)PyArray_DATA(pyvalues1), (dReal*)PyArray_DATA(pyvalues2));
        return make_tuple(static_cast<numeric::array>(handle<>(pyvalues1)), static_cast<numeric::array>(handle<>(pyvalues2)));
    }
    
    object GetLinks()
    {
        CHECK_POINTER(_pbody);
        boost::python::list links;
        FOREACH(itlink, _pbody->GetLinks())
            links.append(PyLink(*itlink, GetEnv()));
        return links;
    }
    object GetJoints()
    {
        CHECK_POINTER(_pbody);
        boost::python::list joints;
        FOREACH(itjoint, _pbody->GetJoints())
            joints.append(PyJoint(*itjoint, GetEnv()));
        return joints;
    }

    object GetTransform() const { CHECK_POINTER(_pbody); return ReturnTransform(_pbody->GetTransform()); }
    
    object GetBodyTransformations() const
    {
        CHECK_POINTER(_pbody);
        boost::python::list transforms;
        FOREACHC(itlink, _pbody->GetLinks())
            transforms.append(ReturnTransform((*itlink)->GetTransform()));
        return transforms;
    }

    void SetBodyTransformations(object transforms)
    {
        CHECK_POINTER(_pbody);
        size_t numtransforms = len(transforms);
        if( numtransforms != _pbody->GetLinks().size() )
            throw openrave_exception("number of input transforms not equal to links");

        std::vector<Transform> vtransforms(numtransforms);
        for(size_t i = 0; i < numtransforms; ++i)
            vtransforms[i] = ExtractTransform(transforms[i]);
        _pbody->SetBodyTransformations(vtransforms);
    }

    object GetLinkVelocities() const
    {
        CHECK_POINTER(_pbody);
        std::vector<std::pair<Vector,Vector> > velocities;
        _pbody->GetLinkVelocities(velocities);

        npy_intp dims[] = {velocities.size(),3};
        PyObject *pylinear = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        PyObject *pyangular = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        dReal* plinear = (dReal*)PyArray_DATA(pylinear);
        dReal* pangular = (dReal*)PyArray_DATA(pyangular);
        FOREACH(it,velocities) {
            *plinear++ = it->first.x; *plinear++ = it->first.y; *plinear++ = it->first.z;
            *pangular++ = it->second.x; *pangular++ = it->second.y; *pangular++ = it->second.z;
        }
        return make_tuple(static_cast<numeric::array>(handle<>(pylinear)),static_cast<numeric::array>(handle<>(pyangular)));
    }

    PyAABB* ComputeAABB() { CHECK_POINTER(_pbody); return new PyAABB(_pbody->ComputeAABB()); }
    void Enable(bool bEnable) { CHECK_POINTER(_pbody); _pbody->Enable(bEnable); }
    bool IsEnabled() const { CHECK_POINTER(_pbody); return _pbody->IsEnabled(); }

    void SetTransform(object transform) { CHECK_POINTER(_pbody); _pbody->SetTransform(ExtractTransform(transform)); }
    void SetJointValues(object o)
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return;

        vector<dReal> values = ExtractRealArray(o);
        if( (int)values.size() != GetDOF() )
            throw openrave_exception("values do not equal to body degrees of freedom");
        _pbody->SetJointValues(NULL,NULL,&values[0],true);
    }
    void SetTransformWithJointValues(object otrans,object ojoints)
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 ) {
            _pbody->SetTransform(ExtractTransform(otrans));
            return;
        }

        vector<dReal> values = ExtractRealArray(ojoints);
        if( (int)values.size() != GetDOF() )
            throw openrave_exception("values do not equal to body degrees of freedom");

        Transform t = ExtractTransform(otrans);
        _pbody->SetJointValues(NULL,&t,&values[0],true);
    }

    void SetJointValues(object o,object indices)
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 || len(indices) == 0 )
            return;

        vector<dReal> vsetvalues = ExtractRealArray(o);
        vector<int> vindices = ExtractArrayInt(indices);
        if( vsetvalues.size() != vindices.size() )
            throw openrave_exception("sizes do not match");

        vector<dReal> values;
        _pbody->GetJointValues(values);
        vector<dReal>::iterator itv = vsetvalues.begin();
        FOREACH(it, vindices) {
            if( *it < 0 || *it >= _pbody->GetDOF() )
                throw openrave_exception("bad index passed");

            values[*it] = *itv++;
        }

        _pbody->SetJointValues(NULL,NULL,&values[0],true);
    }

    object CalculateJacobian(int index, object offset)
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return object();
        npy_intp dims[] = {3,_pbody->GetDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _pbody->CalculateJacobian(index,ExtractVector3(offset),(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    object CalculateRotationJacobian(int index, object q) const
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return object();
        npy_intp dims[] = {4,_pbody->GetDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _pbody->CalculateRotationJacobian(index,ExtractVector4(q),(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    object CalculateAngularVelocityJacobian(int index) const
    {
        CHECK_POINTER(_pbody);
        if( _pbody->GetDOF() == 0 )
            return object();
        npy_intp dims[] = {3,_pbody->GetDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _pbody->CalculateAngularVelocityJacobian(index,(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    bool CheckSelfCollision()
    {
        CHECK_POINTER(_pbody);
        return _pbody->CheckSelfCollision();
    }
    bool CheckSelfCollision(PyCollisionReport* pReport);

    void AttachBody(PyKinBody* pattachbody) {
        CHECK_POINTER(_pbody);
        CHECK_POINTER(pattachbody); CHECK_POINTER(pattachbody->GetBody());
        _pbody->AttachBody(pattachbody->GetBody());
    }
    void RemoveBody(PyKinBody* pbody) {
        CHECK_POINTER(_pbody);
        if( pbody == NULL )
            _pbody->RemoveBody(NULL);
        else {
            CHECK_POINTER(pbody->GetBody());
            _pbody->RemoveBody(pbody->GetBody());
        }
    }
    bool IsAttached(PyKinBody* pattachbody) {
        CHECK_POINTER(_pbody);
        CHECK_POINTER(pattachbody); CHECK_POINTER(pattachbody->GetBody());
        return _pbody->IsAttached(pattachbody->GetBody());
    }
    object GetAttached() const {
        CHECK_POINTER(_pbody);
        boost::python::list attached;
        FOREACHC(it,_pbody->GetAttached())
            attached.append(new PyKinBody(*it,_pyenv,false));
        return attached;
    }
            
    bool IsRobot() const { CHECK_POINTER(_pbody); return _pbody->IsRobot(); }
    int GetNetworkId() const { CHECK_POINTER(_pbody); return _pbody->GetNetworkId(); }

    int DoesAffect(int jointindex, int linkindex ) const {
        CHECK_POINTER(_pbody); 
        return _pbody->DoesAffect(jointindex,linkindex);
    }

    std::string GetForwardKinematics() const {
        CHECK_POINTER(_pbody); 
        stringstream ss;
        _pbody->WriteForwardKinematics(ss);
        return ss.str();
    }

    void SetGuiData(uintptr_t pdata) { CHECK_POINTER(_pbody); _pbody->SetGuiData((void*)pdata); }
    uintptr_t GetGuiData() const { CHECK_POINTER(_pbody); return (uintptr_t)_pbody->GetGuiData(); }

    std::string GetXMLFilename() const { CHECK_POINTER(_pbody); return _pbody->GetXMLFilename(); }

    object GetNonAdjacentLinks() const {
        CHECK_POINTER(_pbody);
        boost::python::list nonadjacent;
        FOREACHC(it,_pbody->GetNonAdjacentLinks())
            nonadjacent.append(make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        return nonadjacent;
    }
    object GetAdjacentLinks() const {
        CHECK_POINTER(_pbody);
        boost::python::list adjacent;
        FOREACHC(it,_pbody->GetAdjacentLinks())
            adjacent.append(make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        return adjacent;
    }
    
    uintptr_t GetPhysicsData() const { CHECK_POINTER(_pbody); return (uintptr_t)_pbody->GetPhysicsData(); }
    uintptr_t GetCollisionData() const { CHECK_POINTER(_pbody); return (uintptr_t)_pbody->GetCollisionData(); }
    int GetUpdateStamp() const { CHECK_POINTER(_pbody); return _pbody->GetUpdateStamp(); }
};

class PyCollisionReport
{
public:
    PyCollisionReport() : plink1(NULL,NULL),plink2(NULL,NULL) {}
    virtual ~PyCollisionReport() {}

    struct PYCONTACT
    {
        PYCONTACT() {}
        PYCONTACT(const COLLISIONREPORT::CONTACT& c)
        {
            pos = toPyVector3(c.pos);
            norm = toPyVector3(c.norm);
        }

        object pos, norm;
    };

    void init(const COLLISIONREPORT& r, PyEnvironmentBase* pyenv)
    {
        options = r.options;
        numCols = r.numCols;
        minDistance = r.minDistance;
        numWithinTol = r.numWithinTol;
        if( r.plink1 != NULL )
            plink1.init(r.plink1, pyenv);
        else
            plink1.init(NULL,NULL);
        if( r.plink2 != NULL )
            plink2.init(r.plink2, pyenv);
        else
            plink2.init(NULL,NULL);
        boost::python::list newcontacts;
        FOREACH(itc, r.contacts)
            newcontacts.append(PYCONTACT(*itc));
        contacts = newcontacts;
    }

    int options;
    PyKinBody::PyLink plink1, plink2;
    int numCols;
    //std::vector<KinBody::Link*> vLinkColliding;
    dReal minDistance;
    int numWithinTol;
    boost::python::list contacts;
};

bool PyKinBody::CheckSelfCollision(PyCollisionReport* pReport)
{
    CHECK_POINTER(_pbody);
    COLLISIONREPORT report;
    bool bSuccess = _pbody->CheckSelfCollision(&report);
    pReport->init(report,GetEnv());
    return bSuccess;
}

class PyRobotBase : public PyKinBody
{
    friend class PyEnvironmentBase;
protected:
    RobotBase* _probot;
    virtual void Invalidate() { _probot = NULL; PyKinBody::Invalidate(); }
public:

    RobotBase* GetRobot() { return _probot; }

    class PyManipulator
    {
        RobotBase::Manipulator* _pmanip;
        PyEnvironmentBase* _pyenv;
    public:
        PyManipulator(RobotBase::Manipulator* pmanip, PyEnvironmentBase* pyenv) : _pmanip(pmanip),_pyenv(pyenv) {}

        object GetEndEffectorTransform() const { return ReturnTransform(_pmanip->GetEndEffectorTransform()); }

		void SetIKSolver(PyIkSolverBase* iksolver);
        bool InitIKSolver() { return _pmanip->InitIKSolver(); }
        string GetIKSolverName() const { return _pmanip->GetIKSolverName(); }
        bool HasIKSolver() const { return _pmanip->HasIKSolver(); }
        string GetName() const { return _pmanip->GetName(); }

        object GetFreeParameters() const {
            if( _pmanip->GetNumFreeParameters() == 0 )
                return object();
            npy_intp dims[] = {_pmanip->GetNumFreeParameters()};
            PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            _pmanip->GetFreeParameters((dReal*)PyArray_DATA(pyvalues));
            return static_cast<numeric::array>(handle<>(pyvalues));
        }

        object FindIKSolution(object transform, bool bColCheck) const
        {
            vector<dReal> solution;
            if( !_pmanip->FindIKSolution(ExtractTransform(transform),solution,bColCheck) )
                return object();
            return toPyArrayN(&solution[0],solution.size());
        }

        object FindIKSolution(object transform, object freeparams, bool bColCheck) const
        {
            vector<dReal> solution, vfreeparams = ExtractRealArray(freeparams);
            if( !_pmanip->FindIKSolution(ExtractTransform(transform),vfreeparams.size()>0?&vfreeparams[0]:NULL, solution,bColCheck) )
                return object();
            return toPyArrayN(&solution[0],solution.size());
        }

        object FindIKSolutions(object transform, bool bColCheck) const
        {
            std::vector<std::vector<dReal> > vsolutions;
            if( !_pmanip->FindIKSolutions(ExtractTransform(transform),vsolutions,bColCheck) )
                return object();
            boost::python::list solutions;
            FOREACH(itsol,vsolutions)
                solutions.append(toPyArrayN(&(*itsol)[0],itsol->size()));
            return solutions;
        }

        object FindIKSolutions(object transform, object freeparams, bool bColCheck) const
        {
            std::vector<std::vector<dReal> > vsolutions;
            vector<dReal> vfreeparams = ExtractRealArray(freeparams);
            if( !_pmanip->FindIKSolutions(ExtractTransform(transform),vfreeparams.size()>0?&vfreeparams[0]:NULL, vsolutions,bColCheck) )
                return object();
            boost::python::list solutions;
            FOREACH(itsol,vsolutions)
                solutions.append(toPyArrayN(&(*itsol)[0],itsol->size()));
            return solutions;
        }

        PyLink* GetBase() { CHECK_POINTER(_pmanip->pBase); return new PyLink(_pmanip->pBase,_pyenv); }
        PyLink* GetEndEffector() { CHECK_POINTER(_pmanip->pEndEffector); return new PyLink(_pmanip->pEndEffector,_pyenv); }
        object GetGraspTransform() { return ReturnTransform(_pmanip->tGrasp); }
        object GetJoints() { return toPyList(_pmanip->_vecjoints); }
        object GetArmJoints() { return toPyList(_pmanip->_vecarmjoints); }
        
        object GetChildJoints() {
            std::set<KinBody::Joint*> vjoints;
            _pmanip->GetChildJoints(vjoints);
            boost::python::list joints;
            FOREACH(itjoint,vjoints)
                joints.append(PyJoint(*itjoint, _pyenv));
            return joints;
        }
        object GetChildDOFIndices() {
            std::set<int> vdofindices;
            _pmanip->GetChildDOFIndices(vdofindices);
            boost::python::list dofindices;
            FOREACH(itindex,vdofindices)
                dofindices.append(*itindex);
            return dofindices;
        }

        object GetChildLinks() {
            std::set<KinBody::Link*> vlinks;
            _pmanip->GetChildLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks)
                links.append(PyLink(*itlink,_pyenv));
            return links;
        }
    };

    class PyAttachedSensor
    {
        RobotBase::AttachedSensor* _pattached;
        PyEnvironmentBase* _pyenv;
    public:
        PyAttachedSensor(RobotBase::AttachedSensor* pattached, PyEnvironmentBase* pyenv) : _pattached(pattached),_pyenv(pyenv) {}
        
        PySensorBase* GetSensor();
        PyLink* GetAttachingLink() const { return new PyLink(_pattached->GetAttachingLink(), _pyenv); }
        object GetRelativeTransform() const { return ReturnTransform(_pattached->GetRelativeTransform()); }
        PyRobotBase* GetRobot() const { return new PyRobotBase(_pattached->GetRobot(), _pyenv, false); }
        string GetName() const { return _pattached->GetName(); }
    };

    class PyGrabbed
    {
    public:
        PyGrabbed(const RobotBase::GRABBED& grabbed, PyEnvironmentBase* pyenv) {
            grabbedbody = object(PyKinBody(grabbed.pbody,pyenv,false));
            linkrobot = object(PyLink(grabbed.plinkrobot,pyenv));

            FOREACH(it, grabbed.sValidColLinks)
                validColLinks.append(PyLink(*it,pyenv));
            
            troot = ReturnTransform(grabbed.troot);
        }

        object grabbedbody;
        object linkrobot;
        boost::python::list validColLinks;
        object troot;
    };

    PyRobotBase(RobotBase* probot, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyKinBody(probot,pyenv,bOwnObject), _probot(probot) {}
    PyRobotBase(const PyRobotBase& r) : PyKinBody(r._probot,r._pyenv,false) { _probot = r._probot; }
    virtual ~PyRobotBase();

    object GetManipulators()
    {
        CHECK_POINTER(_probot);
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators())
            manips.append(PyManipulator(&(*it),_pyenv));
        return manips;
    }

    void SetActiveManipulator(int index) { CHECK_POINTER(_probot); _probot->SetActiveManipulator(index); }
    PyManipulator* GetActiveManipulator() { CHECK_POINTER(_probot); return new PyManipulator(_probot->GetActiveManipulator(),_pyenv); }
    int GetActiveManipulatorIndex() const { CHECK_POINTER(_probot); return _probot->GetActiveManipulatorIndex(); }

    object GetSensors()
    {
        CHECK_POINTER(_probot);
        boost::python::list sensors;
        FOREACH(itsensor, _probot->GetSensors())
            sensors.append(PyAttachedSensor(&(*itsensor),_pyenv));
        return sensors;
    }
    
    PyControllerBase* GetController() const;
    bool SetController(const string& name, const string& args);
    bool SetController(PyControllerBase * pController, const string& args);
    
    void SetActiveDOFs(object jointindices) { CHECK_POINTER(_probot); _probot->SetActiveDOFs(ExtractArrayInt(jointindices)); }
    void SetActiveDOFs(object jointindices, int nAffineDOsBitmask) { CHECK_POINTER(_probot); _probot->SetActiveDOFs(ExtractArrayInt(jointindices), nAffineDOsBitmask); }
    void SetActiveDOFs(object jointindices, int nAffineDOsBitmask, object rotationaxis) {
        CHECK_POINTER(_probot);
        Vector vaxis = ExtractVector3(rotationaxis);
        _probot->SetActiveDOFs(ExtractArrayInt(jointindices), nAffineDOsBitmask, &vaxis);
    }

    int GetActiveDOF() const { CHECK_POINTER(_probot); return _probot->GetActiveDOF(); }
    int GetAffineDOF() const { CHECK_POINTER(_probot); return _probot->GetAffineDOF(); }
    int GetAffineDOFIndex(RobotBase::DOFAffine dof) const { CHECK_POINTER(_probot); return _probot->GetAffineDOFIndex(dof); }

    object GetAffineRotationAxis() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineRotationAxis()); }
    void SetAffineTranslationLimits(object lower, object upper) { CHECK_POINTER(_probot); return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotationAxisLimits(object lower, object upper) { CHECK_POINTER(_probot); return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotation3DLimits(object lower, object upper) { CHECK_POINTER(_probot); return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotationQuatLimits(object lower, object upper) { CHECK_POINTER(_probot); return _probot->SetAffineRotationQuatLimits(ExtractVector4(lower),ExtractVector4(upper)); }
    void SetAffineTranslationMaxVels(object vels) { CHECK_POINTER(_probot); _probot->SetAffineTranslationMaxVels(ExtractVector3(vels)); }
    void SetAffineRotationAxisMaxVels(object vels) { CHECK_POINTER(_probot); _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels)); }
    void SetAffineRotation3DMaxVels(object vels) { CHECK_POINTER(_probot); _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels)); }
    void SetAffineRotationQuatMaxVels(object vels) { CHECK_POINTER(_probot); _probot->SetAffineRotationQuatMaxVels(ExtractVector4(vels)); }
    void SetAffineTranslationResolution(object resolution) { CHECK_POINTER(_probot); _probot->SetAffineTranslationResolution(ExtractVector3(resolution)); }
    void SetAffineRotationAxisResolution(object resolution) { CHECK_POINTER(_probot); _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution)); }
    void SetAffineRotation3DResolution(object resolution) { CHECK_POINTER(_probot); _probot->SetAffineRotation3DResolution(ExtractVector3(resolution)); }
    void SetAffineRotationQuatResolution(object resolution) { CHECK_POINTER(_probot); _probot->SetAffineRotationQuatResolution(ExtractVector4(resolution)); }

    object GetAffineTranslationLimits() const
    {
        CHECK_POINTER(_probot);
        Vector lower, upper;
        _probot->GetAffineTranslationLimits(lower,upper);
        return make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationAxisLimits() const
    {
        CHECK_POINTER(_probot);
        Vector lower, upper;
        _probot->GetAffineRotationAxisLimits(lower,upper);
        return make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotation3DLimits() const
    {
        CHECK_POINTER(_probot);
        Vector lower, upper;
        _probot->GetAffineRotation3DLimits(lower,upper);
        return make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationQuatLimits() const
    {
        CHECK_POINTER(_probot);
        Vector lower, upper;
        _probot->GetAffineRotationQuatLimits(lower,upper);
        return make_tuple(toPyVector4(lower),toPyVector4(upper));
    }
    object GetAffineTranslationMaxVels() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineTranslationMaxVels()); }
    object GetAffineRotationAxisMaxVels() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineRotationAxisMaxVels()); }
    object GetAffineRotation3DMaxVels() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineRotation3DMaxVels()); }
    object GetAffineRotationQuatMaxVels() const { CHECK_POINTER(_probot); return toPyVector4(_probot->GetAffineRotationQuatMaxVels()); }
    object GetAffineTranslationResolution() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineTranslationResolution()); }
    object GetAffineRotationAxisResolution() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineRotationAxisResolution()); }
    object GetAffineRotation3DResolution() const { CHECK_POINTER(_probot); return toPyVector3(_probot->GetAffineRotation3DResolution()); }
    object GetAffineRotationQuatResolution() const { CHECK_POINTER(_probot); return toPyVector4(_probot->GetAffineRotationQuatResolution()); }

    void SetActiveDOFValues(object values) const
    {
        CHECK_POINTER(_probot);
        vector<dReal> vvalues = ExtractRealArray(values);
        if( vvalues.size() > 0 )
            _probot->SetActiveDOFValues(NULL,&vvalues[0],true);
    }
    object GetActiveDOFValues() const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {_probot->GetActiveDOF()};
        PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->GetActiveDOFValues((dReal*)PyArray_DATA(pyvalues));
        return static_cast<numeric::array>(handle<>(pyvalues));
    }

    void SetActiveDOFVelocities(object velocities)
    {
        CHECK_POINTER(_probot);
        vector<dReal> vvelocities = ExtractRealArray(velocities);
        if( vvelocities.size() > 0 )
            _probot->SetActiveDOFVelocities(&vvelocities[0]);
    }
    object GetActiveDOFVelocities() const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {_probot->GetActiveDOF()};
        PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->GetActiveDOFVelocities((dReal*)PyArray_DATA(pyvalues));
        return static_cast<numeric::array>(handle<>(pyvalues));
    }

    object GetActiveDOFLimits() const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {_probot->GetActiveDOF()};
        PyObject *pyvalues1 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        PyObject *pyvalues2 = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->GetActiveDOFLimits((dReal*)PyArray_DATA(pyvalues1), (dReal*)PyArray_DATA(pyvalues1));
        return make_tuple(static_cast<numeric::array>(handle<>(pyvalues1)), static_cast<numeric::array>(handle<>(pyvalues2)));
    }

//    void GetActiveDOFResolutions(dReal* pResolution) const;
//    void GetActiveDOFResolutions(std::vector<dReal>& v) const;
//    void GetActiveDOFMaxVel(dReal* pMaxVel) const;
//    void GetActiveDOFMaxVel(std::vector<dReal>& v) const;
//    void GetActiveDOFMaxAccel(dReal* pMaxAccel) const;
//    void GetActiveDOFMaxAccel(std::vector<dReal>& v) const;

//    int GetControlDOF() const { return GetActiveDOF(); }
//    void GetControlMaxTorques(dReal* pMaxTorque) const;
//    void GetControlMaxTorques(std::vector<dReal>& vmaxtorque) const;
//    void SetControlTorques(dReal* pTorques);
    
//    void GetFullTrajectoryFromActive(PyTrajectory* pFullTraj, PyTrajectory* pActiveTraj, bool bOverwriteTransforms);
//    void SetActiveMotion(PyTrajectory* ptraj);

    int GetActiveJointIndex(int active_index) const { CHECK_POINTER(_probot); return _probot->GetActiveJointIndex(active_index); }
    object GetActiveJointIndices() { CHECK_POINTER(_probot); return toPyList(_probot->GetActiveJointIndices()); }

    object CalculateActiveJacobian(int index, object offset) const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {3,_probot->GetActiveDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->CalculateActiveJacobian(index,ExtractVector3(offset),(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    object CalculateActiveRotationJacobian(int index, object q) const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {4,_probot->GetActiveDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->CalculateActiveJacobian(index,ExtractVector4(q),(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    object CalculateActiveAngularVelocityJacobian(int index) const
    {
        CHECK_POINTER(_probot);
        if( _probot->GetActiveDOF() == 0 )
            return object();
        npy_intp dims[] = {3,_probot->GetActiveDOF()};
        PyObject *pyjacobian = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        _probot->CalculateActiveAngularVelocityJacobian(index,(dReal*)PyArray_DATA(pyjacobian));
        return static_cast<numeric::array>(handle<>(pyjacobian));
    }

    bool Grab(PyKinBody* pbody) { CHECK_POINTER(_probot); CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody()); }
    bool Grab(PyKinBody* pbody, object linkstoignore)
    {
        CHECK_POINTER(_probot);
        std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
        return _probot->Grab(pbody->GetBody(), &setlinkstoignore);
    }
    bool Grab(PyKinBody* pbody, int linkindex, object linkstoignore)
    {
        CHECK_POINTER(_probot);
        std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
        return _probot->Grab(pbody->GetBody(), linkindex, &setlinkstoignore);
    }
    void Release(PyKinBody* pbody) { CHECK_POINTER(_probot); _probot->Release(pbody->GetBody()); }
    void ReleaseAllGrabbed() { CHECK_POINTER(_probot); _probot->ReleaseAllGrabbed(); }
    void RegrabAll() { CHECK_POINTER(_probot); _probot->RegrabAll(); }
    bool IsGrabbing(PyKinBody* pbody) const { CHECK_POINTER(_probot); return _probot->IsGrabbing(pbody->GetBody()); }
    object GetGrabbed() const {
        CHECK_POINTER(_probot);
        boost::python::list grabbed;
        FOREACHC(it, _probot->GetGrabbed())
            grabbed.append(object(PyGrabbed(*it,_pyenv)));
        return grabbed;
    }

    bool WaitForController(float ftimeout)
    {
        CHECK_POINTER(_probot);
        if( _probot->GetController() == NULL )
            return false;

        uint64_t starttime = GetMicroTime();
        uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
        while( !_probot->GetController()->IsDone() ) {
            Sleep(1);
            
            if( deltatime > 0 && (GetMicroTime()-starttime)>deltatime  )
                return false;
        }
        
        return true;
    }
};

class PyPlannerBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
protected:
    PlannerBase* _pplanner;
    virtual void Invalidate() { _pplanner = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyPlannerBase(PlannerBase* pplanner, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pplanner, pyenv, bOwnObject), _pplanner(pplanner) {}
    virtual ~PyPlannerBase() { if( _bOwnObject ) { _bOwnObject = false; delete _pplanner; } }
};

class PySensorSystemBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
private:
    SensorSystemBase* _psensorsystem;
    virtual void Invalidate() { _psensorsystem = NULL; PyInterfaceBase::Invalidate(); }
public:
    PySensorSystemBase(SensorSystemBase* psensorsystem, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(psensorsystem, pyenv, bOwnObject), _psensorsystem(psensorsystem) {}
    virtual ~PySensorSystemBase() { if( _bOwnObject ) { _bOwnObject = false; delete _psensorsystem; } }
};

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    Trajectory* _ptrajectory;
    virtual void Invalidate() { _ptrajectory = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyTrajectoryBase(Trajectory* pTrajectory, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pTrajectory, pyenv, bOwnObject),_ptrajectory(pTrajectory) {}
    virtual ~PyTrajectoryBase() { if( _bOwnObject ) { _bOwnObject = false; delete _ptrajectory; } }

    Trajectory* GetTrajectory() { return _ptrajectory; }
};

class PyControllerBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
protected:
    ControllerBase* _pcontroller;
    virtual void Invalidate() { _pcontroller = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyControllerBase(ControllerBase* pcontroller, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pcontroller, pyenv, bOwnObject), _pcontroller(pcontroller) {}
    virtual ~PyControllerBase() { if( _bOwnObject ) { _bOwnObject = false; delete _pcontroller; } }

    ControllerBase* GetController() { return _pcontroller; }

    bool Init(PyRobotBase* robot, const string& args) { CHECK_POINTER(_pcontroller); return _pcontroller->Init(robot->GetRobot(), args.c_str()); }
    void Reset(int options) { CHECK_POINTER(_pcontroller); _pcontroller->Reset(options); }

    bool SetDesired(object o)
    {
        CHECK_POINTER(_pcontroller);
        vector<dReal> values = ExtractRealArray(o);
        if( values.size() == 0 )
            throw openrave_exception("no values specified");
        return _pcontroller->SetDesired(&values[0]);
    }
    
    bool SetPath(PyTrajectoryBase* ptraj) { CHECK_POINTER(_pcontroller); return _pcontroller->SetPath(ptraj != NULL ? ptraj->GetTrajectory() : NULL); }
    //bool SetPath(PyTrajectoryBase* ptraj, int nTrajectoryId, float fDivergenceTime) = 0;
    bool SimulationStep(dReal fTimeElapsed) { CHECK_POINTER(_pcontroller); return _pcontroller->SimulationStep(fTimeElapsed); }

    bool IsDone() { CHECK_POINTER(_pcontroller); return _pcontroller->IsDone(); }
    dReal GetTime() { CHECK_POINTER(_pcontroller); return _pcontroller->GetTime(); }

    object GetVelocity()
    {
        CHECK_POINTER(_pcontroller);
        vector<dReal> velocity;
        _pcontroller->GetVelocity(velocity);
        return toPyArray(velocity);
    }

    object GetTorque()
    {
        CHECK_POINTER(_pcontroller);
        vector<dReal> torque;
        _pcontroller->GetTorque(torque);
        return toPyArray(torque);
    }
    
    bool SendCmd(const string& cmd) { CHECK_POINTER(_pcontroller); return _pcontroller->SendCmd(cmd.c_str()); }
    bool SupportsCmd(const string& cmd) { CHECK_POINTER(_pcontroller); return _pcontroller->SupportsCmd(cmd.c_str()); }
};

class PyProblemInstance : public PyInterfaceBase
{
protected:
    ProblemInstance* _pproblem;
    virtual void Invalidate() { _pproblem = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyProblemInstance(ProblemInstance* pproblem, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pproblem, pyenv, bOwnObject), _pproblem(pproblem) {}
    virtual ~PyProblemInstance();
    ProblemInstance* GetProblem() { return _pproblem; }

    bool SimulationStep(dReal fElapsedTime) { CHECK_POINTER(_pproblem); return _pproblem->SimulationStep(fElapsedTime); }
    object SendCommand(const string& cmd) {
        CHECK_POINTER(_pproblem);
        //LockEnvironment envlock(_pproblem->GetEnv());
        string response;
        if( !_pproblem->SendCommand(cmd.c_str(),response) )
            return object();
        return object(response);
    }

//    void Query(const char* cmd, std::string& response) {}
};

class PyIkSolverBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
    friend class RobotBase;
protected:
    IkSolverBase* _pIkSolver;
    virtual void Invalidate() { _pIkSolver = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyIkSolverBase(IkSolverBase* pIkSolver, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pIkSolver, pyenv, bOwnObject), _pIkSolver(pIkSolver) {}
    virtual ~PyIkSolverBase() { if( _bOwnObject ) { _bOwnObject = false; delete _pIkSolver; } }

    IkSolverBase* GetIkSolver() { return _pIkSolver; }
};

class PyPhysicsEngineBase : public PyInterfaceBase
{
protected:
    PhysicsEngineBase* _pPhysicsEngine;
    virtual void Invalidate() { _pPhysicsEngine = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyPhysicsEngineBase(PhysicsEngineBase* pPhysicsEngine, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pPhysicsEngine, pyenv, bOwnObject),_pPhysicsEngine(pPhysicsEngine) {}
    virtual ~PyPhysicsEngineBase();

    PhysicsEngineBase* GetPhysicsEngine() { return _pPhysicsEngine; }

    bool SetPhysicsOptions(int physicsoptions) { CHECK_POINTER(_pPhysicsEngine); return _pPhysicsEngine->SetPhysicsOptions(physicsoptions); }
    int GetPhysicsOptions() const { CHECK_POINTER(_pPhysicsEngine); return _pPhysicsEngine->GetPhysicsOptions(); }

    object SetPhysicsOptions(const string& s) {
        CHECK_POINTER(_pPhysicsEngine);
        stringstream sinput(s), sout;
        if( !_pPhysicsEngine->SetPhysicsOptions(sout,sinput) )
            return object();
        return object(sout.str());
    }
    bool InitEnvironment() { CHECK_POINTER(_pPhysicsEngine); return _pPhysicsEngine->InitEnvironment(); }
    void DestroyEnvironment() { CHECK_POINTER(_pPhysicsEngine); _pPhysicsEngine->DestroyEnvironment(); }
    bool InitKinBody(PyKinBody* pbody) { CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody()); return _pPhysicsEngine->InitKinBody(pbody->GetBody()); }
    bool DestroyKinBody(PyKinBody* pbody) { CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody()); return _pPhysicsEngine->DestroyKinBody(pbody->GetBody()); }
    bool SetBodyVelocity(PyKinBody* pbody, object linearvel, object angularvel, object jointvelocity)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody());
        vector<dReal> vJointVelocity = ExtractRealArray(jointvelocity);
        return _pPhysicsEngine->SetBodyVelocity(pbody->GetBody(),ExtractVector3(linearvel),ExtractVector3(angularvel),vJointVelocity.size()>0?&vJointVelocity[0]:NULL);
    }
    bool SetBodyVelocity(PyKinBody* pbody, object LinearVelocities, object AngularVelocities)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody());
        vector<dReal> vLinearVelocities = ExtractRealArray(LinearVelocities);
        vector<dReal> vAngularVelocities = ExtractRealArray(AngularVelocities);
        vector<Vector> linearvel(vLinearVelocities.size()/3);
        for(size_t i = 0; i < vLinearVelocities.size()/3; ++i)
            linearvel[i] = Vector(vLinearVelocities[3*i],vLinearVelocities[3*i+1],vLinearVelocities[3*i+2]);
        vector<Vector> angularvel(vAngularVelocities.size()/3);
        for(size_t i = 0; i < vAngularVelocities.size()/3; ++i)
            angularvel[i] = Vector(vAngularVelocities[3*i],vAngularVelocities[3*i+1],vAngularVelocities[3*i+2]);
        return _pPhysicsEngine->SetBodyVelocity(pbody->GetBody(),linearvel.size()>0?&linearvel[0]:NULL,
                                                angularvel.size()>0?&angularvel[0]:NULL);
    }

    object GetBodyVelocityJoints(PyKinBody* pbody)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody());
        Vector linearvel, angularvel;
        npy_intp dims[] = {pbody->GetBody()->GetDOF()};
        PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        if( !_pPhysicsEngine->GetBodyVelocity(pbody->GetBody(),linearvel,angularvel,(dReal*)PyArray_DATA(pyvalues)) ) {
            decref(pyvalues);
            return make_tuple(object(),object(),object());
        }

        return make_tuple(toPyVector3(linearvel),toPyVector3(angularvel),static_cast<numeric::array>(handle<>(pyvalues)));
    }

    object GetBodyVelocityLinks(PyKinBody* pbody, Vector* pLinearVelocities, Vector* pAngularVelocities)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody());
        if( pbody->GetBody()->GetDOF() == 0 )
            return make_tuple(object(),object());
        vector<Vector> linearvel(pbody->GetBody()->GetDOF()),angularvel(pbody->GetBody()->GetDOF());
        if( !_pPhysicsEngine->GetBodyVelocity(pbody->GetBody(),&linearvel[0],&angularvel[0]) )
            return make_tuple(object(),object());

        npy_intp dims[] = {pbody->GetBody()->GetDOF(),3};
        PyObject *pylinear = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        PyObject *pyangular = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        dReal* pflinear = (dReal*)PyArray_DATA(pylinear);
        dReal* pfangular = (dReal*)PyArray_DATA(pyangular);
        for(size_t i = 0; i < linearvel.size(); ++i) {
            pflinear[3*i+0] = linearvel[i].x; pflinear[3*i+1] = linearvel[i].y; pflinear[3*i+2] = linearvel[i].z;
            pfangular[3*i+0] = angularvel[i].x; pfangular[3*i+1] = angularvel[i].y; pfangular[3*i+2] = angularvel[i].z;
        }
        return make_tuple(static_cast<numeric::array>(handle<>(pylinear)),static_cast<numeric::array>(handle<>(pyangular)));
    }

    bool SetJointVelocity(PyKinBody::PyJoint* pjoint, object jointvelocity)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pjoint); CHECK_POINTER(pjoint->GetParent()); CHECK_POINTER(pjoint->GetParent()->GetBody());
        vector<dReal> velocity = ExtractRealArray(jointvelocity);
        return _pPhysicsEngine->SetJointVelocity(pjoint->GetJoint(),velocity.size()>0?&velocity[0]:NULL);
    }

    object GetJointVelocity(PyKinBody::PyJoint* pjoint)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pjoint); CHECK_POINTER(pjoint->GetParent()); CHECK_POINTER(pjoint->GetParent()->GetBody());
        Vector linearvel, angularvel;
        npy_intp dims[] = {pjoint->GetJoint()->GetDOF()};
        PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        if( !_pPhysicsEngine->GetJointVelocity(pjoint->GetJoint(),(dReal*)PyArray_DATA(pyvalues)) ) {
            decref(pyvalues);
            return object();
        }

        return static_cast<numeric::array>(handle<>(pyvalues));
    }

    bool SetBodyForce(PyKinBody::PyLink* plink, object force, object position, bool bAdd)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(plink); CHECK_POINTER(plink->GetParent()); CHECK_POINTER(plink->GetParent()->GetBody());
        return _pPhysicsEngine->SetBodyForce(plink->GetLink(),ExtractVector3(force),ExtractVector3(position),bAdd);
    }

    bool SetBodyTorque(PyKinBody::PyLink* plink, object torque, bool bAdd)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(plink); CHECK_POINTER(plink->GetParent()); CHECK_POINTER(plink->GetParent()->GetBody());
        return _pPhysicsEngine->SetBodyTorque(plink->GetLink(),ExtractVector3(torque),bAdd);
    }

    bool AddJointTorque(PyKinBody::PyJoint* pjoint, object torques)
    {
        CHECK_POINTER(_pPhysicsEngine); CHECK_POINTER(pjoint); CHECK_POINTER(pjoint->GetParent()); CHECK_POINTER(pjoint->GetParent()->GetBody());
        vector<dReal> vtorques = ExtractRealArray(torques);
        return _pPhysicsEngine->AddJointTorque(pjoint->GetJoint(),vtorques.size()>0?&vtorques[0]:NULL);
    }

    void SetGravity(object gravity)
    {
        CHECK_POINTER(_pPhysicsEngine);
        _pPhysicsEngine->SetGravity(ExtractVector3(gravity));
    }
    object GetGravity() { CHECK_POINTER(_pPhysicsEngine); return toPyVector3(_pPhysicsEngine->GetGravity()); }

    void SimulateStep(dReal fTimeElapsed)
    {
        CHECK_POINTER(_pPhysicsEngine);
        _pPhysicsEngine->SimulateStep(fTimeElapsed);
    }
};

class PySensorBase : public PyInterfaceBase
{
protected:
    SensorBase* _psensor;
    boost::shared_ptr<SensorBase::SensorData> _psensordata;
    virtual void Invalidate() { _psensor = NULL; PyInterfaceBase::Invalidate(); }
public:
    class PySensorData
    {
    public:
        PySensorData(SensorBase::SensorData* pdata)
        {
            type = pdata->GetType();
        }
        virtual ~PySensorData() {}
        
        SensorBase::SensorType type;
    };

    class PyLaserSensorData : public PySensorData
    {
    public:
        PyLaserSensorData(SensorBase::LaserGeomData* pgeom, SensorBase::LaserSensorData* pdata) : PySensorData(pdata)
        {
            transform = ReturnTransform(pdata->t);
            positions = toPyArray3(pdata->positions);
            ranges = toPyArray3(pdata->ranges);
            intensity = toPyArrayN(pdata->intensity.size()>0?&pdata->intensity[0]:NULL,pdata->intensity.size());
            id = pdata->id;
        }
        virtual ~PyLaserSensorData() {}

        object transform, positions, ranges, intensity;
        int id;
    };

    class PyCameraSensorData : public PySensorData
    {
    public:
        PyCameraSensorData(SensorBase::CameraGeomData* pgeom, SensorBase::CameraSensorData* pdata) : PySensorData(pdata)
        {
            if( (int)pdata->vimagedata.size() != pgeom->height*pgeom->width*3 )
                throw openrave_exception("bad image data");

            transform = ReturnTransform(pdata->t);
            {
                npy_intp dims[] = {pgeom->height,pgeom->width,3};
                PyObject *pyvalues = PyArray_SimpleNew(3,dims, PyArray_UINT8);
                if( pdata->vimagedata.size() > 0 )
                    memcpy(PyArray_DATA(pyvalues),&pdata->vimagedata[0],pdata->vimagedata.size());
                imagedata = static_cast<numeric::array>(handle<>(pyvalues));
            }
            {
                numeric::array arr(make_tuple(pgeom->KK[0],0,pgeom->KK[2],0,pgeom->KK[1],pgeom->KK[3],0,0,1));
                arr.resize(3,3);
                KK = arr;
            }
            id = pdata->id;
        }
        virtual ~PyCameraSensorData() {}

        object transform, imagedata;
        object KK;
        int id;
    };

    PySensorBase(SensorBase* psensor, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(psensor, pyenv, bOwnObject), _psensor(psensor)
    {
        _psensordata.reset(_psensor->CreateSensorData());
    }
    virtual ~PySensorBase() { if( _bOwnObject ) { _bOwnObject = false; delete _psensor; } }

    SensorBase* GetSensor() { return _psensor; }

    PySensorData* GetSensorData()
    {
        CHECK_POINTER(_psensor);
        if( !_psensor->GetSensorData(_psensordata.get()) )
            throw openrave_exception("SensorData failed");
        switch(_psensordata->GetType()) {
        case SensorBase::ST_Laser:
            return new PyLaserSensorData((SensorBase::LaserGeomData*)_psensor->GetSensorGeometry(),
                                                (SensorBase::LaserSensorData*)_psensordata.get());

        case SensorBase::ST_Camera:
            return new PyCameraSensorData((SensorBase::CameraGeomData*)_psensor->GetSensorGeometry(),
                                                 (SensorBase::CameraSensorData*)_psensordata.get());
        default: {
            stringstream ss;
            ss << "unknown sensor data type: " << _psensordata->GetType() << endl;
            throw openrave_exception(ss.str());
        }
        }
    }

    object SendCmd(const string& cmd)
    {
        CHECK_POINTER(_psensor);
        std::stringstream ss(cmd), ssout;
        if( !_psensor->SendCmd(ss,ssout) )
            return object();
        return object(ssout.str());
    }
    
    bool SupportsCmd(const string& cmd) { CHECK_POINTER(_psensor); return _psensor->SupportsCmd(cmd.c_str()); }
    
    void SetTransform(object transform) { CHECK_POINTER(_psensor); _psensor->SetTransform(ExtractTransform(transform)); }
    object GetTransform() { CHECK_POINTER(_psensor); return ReturnTransform(_psensor->GetTransform()); }

    string GetName() { CHECK_POINTER(_psensor); return _stdwcstombs(_psensor->GetName()); }
};

class PyCollisionCheckerBase : public PyInterfaceBase
{
protected:
    CollisionCheckerBase* _pCollisionChecker;
    virtual void Invalidate() { _pCollisionChecker = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyCollisionCheckerBase(CollisionCheckerBase* pCollisionChecker, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pCollisionChecker, pyenv, bOwnObject), _pCollisionChecker(pCollisionChecker) {}
    virtual ~PyCollisionCheckerBase();

    CollisionCheckerBase* GetCollisionChecker() { return _pCollisionChecker; }
};

class PyRaveViewerBase : public PyInterfaceBase
{
protected:
    RaveViewerBase* _pviewer;
    virtual void Invalidate() { _pviewer = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyRaveViewerBase(RaveViewerBase* pviewer, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pviewer, pyenv, bOwnObject), _pviewer(pviewer) {}
    virtual ~PyRaveViewerBase() { if( _bOwnObject ) { _bOwnObject = false; delete _pviewer; } }

    RaveViewerBase* GetViewer() { return _pviewer; }

    int main(bool bShow) { CHECK_POINTER(_pviewer); return _pviewer->main(bShow); }
    void quitmainloop() { CHECK_POINTER(_pviewer); return _pviewer->quitmainloop(); }

    void ViewerSetSize(int w, int h) { CHECK_POINTER(_pviewer); _pviewer->ViewerSetSize(w,h); }
    void ViewerMove(int x, int y) { CHECK_POINTER(_pviewer); _pviewer->ViewerMove(x,y); }
    void ViewerSetTitle(const string& title) { CHECK_POINTER(_pviewer); _pviewer->ViewerSetTitle(title.c_str()); }
    bool LoadModel(const string& filename) { CHECK_POINTER(_pviewer); return _pviewer->LoadModel(filename.c_str()); }
};

class PyRaveServerBase : public PyInterfaceBase
{
protected:
    RaveServerBase* _pserver;
    virtual void Invalidate() { _pserver = NULL; PyInterfaceBase::Invalidate(); }
public:
    PyRaveServerBase(RaveServerBase* pserver, PyEnvironmentBase* pyenv, bool bOwnObject=true) : PyInterfaceBase(pserver, pyenv, bOwnObject), _pserver(pserver) {}
    virtual ~PyRaveServerBase() { if( _bOwnObject ) { _bOwnObject = false; delete _pserver; } }

    RaveServerBase* GetServer() { return _pserver; }
};

class PyEnvironmentBase
{
protected:
    boost::shared_ptr<EnvironmentBase> _penv;
    boost::shared_ptr<RaveViewerBase> _pviewer;
    boost::shared_ptr<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;
    bool _bShutdown;

    void _ViewerThread(const string& strviewer, bool bShowViewer)
    {
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _pviewer.reset(_penv->CreateViewer(strviewer.c_str()));
            if( !!_pviewer ) {
                _penv->AttachViewer(_pviewer.get());
            }
            _conditionViewer.notify_all();
        }

        if( !_pviewer )
            return;

        _pviewer->main(bShowViewer); // spin until quitfrommainloop is called
        RAVELOG_DEBUGA("destroying viewer\n");
        _pviewer.reset();
    }

    PyKinBody* toPyKinBody(KinBody* pbody, bool bOwnObject) { return pbody != NULL ? new PyKinBody(pbody,this,bOwnObject) : NULL; }
    PyRobotBase* toPyRobot(RobotBase* probot, bool bOwnObject) { return probot != NULL ? new PyRobotBase(probot,this,bOwnObject) : NULL; }
    PyPlannerBase* toPyPlanner(PlannerBase* pplanner, bool bOwnObject) { return pplanner != NULL ? new PyPlannerBase(pplanner,this,bOwnObject) : NULL; }
    PySensorSystemBase* toPySensorSystem(SensorSystemBase* pSensorSystem, bool bOwnObject) { return pSensorSystem != NULL ? new PySensorSystemBase(pSensorSystem,this,bOwnObject) : NULL; }
    PyControllerBase* toPyController(ControllerBase* pcontroller, bool bOwnObject) { return pcontroller != NULL ? new PyControllerBase(pcontroller,this,bOwnObject) : NULL; }
    PyProblemInstance* toPyProblemInstance(ProblemInstance* pProblemInstance, bool bOwnObject) { return pProblemInstance != NULL ? new PyProblemInstance(pProblemInstance,this,bOwnObject) : NULL; }
    PyIkSolverBase* toPyIkSolver(IkSolverBase* pIkSolver, bool bOwnObject) { return pIkSolver != NULL ? new PyIkSolverBase(pIkSolver,this,bOwnObject) : NULL; }
    PyPhysicsEngineBase* toPyPhysicsEngine(PhysicsEngineBase* pPhysicsEngine, bool bOwnObject) { return pPhysicsEngine != NULL ? new PyPhysicsEngineBase(pPhysicsEngine,this,bOwnObject) : NULL; }
    PySensorBase* toPySensor(SensorBase* psensor, bool bOwnObject) { return psensor != NULL ? new PySensorBase(psensor,this,bOwnObject) : NULL; }
    PyCollisionCheckerBase* toPyCollisionChecker(CollisionCheckerBase* pCollisionChecker, bool bOwnObject) { return pCollisionChecker != NULL ? new PyCollisionCheckerBase(pCollisionChecker,this,bOwnObject) : NULL; }
    PyRaveViewerBase* toPyRaveViewer(RaveViewerBase* pRaveViewer, bool bOwnObject) { return pRaveViewer != NULL ? new PyRaveViewerBase(pRaveViewer,this,bOwnObject) : NULL; }
    PyRaveServerBase* toPyServer(RaveServerBase* pserver, bool bOwnObject) { return pserver != NULL ? new PyRaveServerBase(pserver,this,bOwnObject) : NULL; }


public:
    PyEnvironmentBase()
    {
        _penv.reset(CreateEnvironment());
        _bShutdown = false;
    }
    PyEnvironmentBase(const PyEnvironmentBase& pyenv)
    {
        _bShutdown = false;
        _penv = pyenv._penv;
        if( !!_penv && _penv->GetViewer() != NULL )
            SetViewer(_penv->GetViewer()->GetXMLId());
    }
    PyEnvironmentBase(EnvironmentBase* penv) : _penv(penv), _bShutdown(false) {}
    virtual ~PyEnvironmentBase()
    {
        {
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            _penv->AttachViewer(NULL);
        }
        _bShutdown = true;
        if( !!_threadviewer ) {
            _threadviewer->join();
            _threadviewer.reset();
        }
        _pviewer.reset();

        // get rid of any pointers
        _penv->SetCollisionChecker(NULL);
        _penv->SetPhysicsEngine(NULL);

        // destroy all allocated objects
        FOREACH(it, _setInterfaces) {
            if( (*it)->_bOwnObject ) {
                (*it)->_bOwnObject = false;
                // have to remove anything from deleting
                if( dynamic_cast<PyKinBody*>(*it) )
                    _penv->RemoveKinBody(dynamic_cast<PyKinBody*>(*it)->GetBody(),false);
                if( dynamic_cast<PyProblemInstance*>(*it) )
                    _penv->RemoveProblem(dynamic_cast<PyProblemInstance*>(*it)->GetProblem());
                delete (*it)->_pbase;
            }
            (*it)->Invalidate();
        }
        _setInterfaces.clear();
    }

    void Reset() {
        _penv->Reset();

        // have to invalidate all bodies/robots
        std::set<PyInterfaceBase*>::iterator it = _setInterfaces.begin();
        while( it != _setInterfaces.end()) {
            if( dynamic_cast<PyKinBody*>(*it) != NULL || dynamic_cast<PyRobotBase*>(*it) != NULL ) {
                (*it)->Invalidate();
                _setInterfaces.erase(it++);
            }
            else
                ++it;
        }
    }
//    void Destroy()
//    {
//        FOREACH(it, _setInterfaces) {
//            (*it)->_bOwnObject = false;
//        _penv->Destroy();
//    }

    object GetPluginInfo()
    {
        boost::python::list plugins;
        std::list< std::pair<std::string, PLUGININFO> > listplugins;
        _penv->GetPluginInfo(listplugins);
        FOREACH(itplugin, listplugins)
            plugins.append(make_tuple(itplugin->first,object(PyPluginInfo(itplugin->second))));
        return plugins;
    }

    PyPluginInfo* GetLoadedInterfaces()
    {
        PLUGININFO info;
        _penv->GetLoadedInterfaces(info);
        return new PyPluginInfo(info);
    }

    bool LoadPlugin(const string& name) { return _penv->LoadPlugin(name.c_str()); }

    PyRobotBase* CreateRobot(const string& name) { return new PyRobotBase(_penv->CreateRobot(name.c_str()), this, true); }
    PyPlannerBase* CreatePlanner(const string& name) { return new PyPlannerBase(_penv->CreatePlanner(name.c_str()), this, true); }
    PySensorSystemBase* CreateSensorSystem(const string& name) { return new PySensorSystemBase(_penv->CreateSensorSystem(name.c_str()), this, true); }
    PyControllerBase* CreateController(const string& name) { return new PyControllerBase(_penv->CreateController(name.c_str()), this, true); }
    PyProblemInstance* CreateProblem(const string& name) { return new PyProblemInstance(_penv->CreateProblem(name.c_str()), this, true); }
    PyIkSolverBase* CreateIkSolver(const string& name) { return new PyIkSolverBase(_penv->CreateIkSolver(name.c_str()), this, true); }
    PyPhysicsEngineBase* CreatePhysicsEngine(const string& name) { return new PyPhysicsEngineBase(_penv->CreatePhysicsEngine(name.c_str()), this, true); }
    PySensorBase* CreateSensor(const string& name) { return new PySensorBase(_penv->CreateSensor(name.c_str()), this, true); }
    PyCollisionCheckerBase* CreateCollisionChecker(const string& name) { return new PyCollisionCheckerBase(_penv->CreateCollisionChecker(name.c_str()), this, true); }
    PyRaveViewerBase* CreateViewer(const string& name) { return new PyRaveViewerBase(_penv->CreateViewer(name.c_str()), this, true); }
    PyRaveServerBase* CreateServer(const string& name) { return new PyRaveServerBase(_penv->CreateServer(name.c_str()), this, true); }

    PyEnvironmentBase* CloneSelf(int options)
    {
        //RAVELOG_WARNA("cloning environment without permission!\n");
        string strviewer;
        if( options & Clone_Viewer ) {
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            if( _penv->GetViewer() != NULL )
                strviewer = _penv->GetViewer()->GetXMLId();
        }
        PyEnvironmentBase* pnewenv = new PyEnvironmentBase(_penv->CloneSelf(options));
        if( strviewer.size() > 0 )
            pnewenv->SetViewer(strviewer);
        return pnewenv;
    }

    bool SetCollisionChecker(PyCollisionCheckerBase* pchecker) {
        if( pchecker == NULL )
            return _penv->SetCollisionChecker(NULL);
        CHECK_POINTER(pchecker->GetCollisionChecker());
        return _penv->SetCollisionChecker(pchecker->GetCollisionChecker());
    }
    PyCollisionCheckerBase* GetCollisionChecker() { return new PyCollisionCheckerBase(_penv->GetCollisionChecker(), this, false); }

    bool SetCollisionOptions(int options) { return _penv->SetCollisionOptions(options); }
    int GetCollisionOptions() const { return _penv->GetCollisionOptions(); }

    bool CheckCollision(PyKinBody* pbody1)
    {
        return _penv->CheckCollision(pbody1->_pbody);
    }
    bool CheckCollision(PyKinBody* pbody1, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(pbody1->_pbody, &report);
        pReport->init(report,this);
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBody* pbody1, PyKinBody* pbody2)
    {
        return _penv->CheckCollision(pbody1->_pbody, pbody2->_pbody);
    }

    bool CheckCollision(PyKinBody* pbody1, PyKinBody* pbody2, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(pbody1->_pbody, pbody2->_pbody, &report);
        pReport->init(report,this);
        return bSuccess;
    }

    bool CheckCollision(PyKinBody::PyLink* plink)
    {
        return _penv->CheckCollision(plink->_plink);
    }

    bool CheckCollision(PyKinBody::PyLink* plink, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(plink->_plink, &report);
        pReport->init(report,this);
        return bSuccess;
    }

    bool CheckCollision(PyKinBody::PyLink* plink1, PyKinBody::PyLink* plink2)
    {
        return _penv->CheckCollision(plink1->_plink, plink2->_plink);
    }
    bool CheckCollision(PyKinBody::PyLink* plink1, PyKinBody::PyLink* plink2, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(plink1->_plink, plink2->_plink, &report);
        pReport->init(report,this);
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBody::PyLink* plink, PyKinBody* pbody)
    {
        return _penv->CheckCollision(plink->_plink, pbody->_pbody);
    }
    bool CheckCollision(PyKinBody::PyLink* plink, PyKinBody* pbody, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(plink->_plink, pbody->_pbody, &report);
        pReport->init(report,this);
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBody::PyLink* plink, object bodyexcluded, object linkexcluded)
    {
        std::set<KinBody*> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody* pbody = extract<PyKinBody*>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.insert(pbody->_pbody);
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::set<KinBody::Link*> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLink* plink = extract<PyKinBody::PyLink*>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.insert(plink->_plink);
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        return _penv->CheckCollision(plink->_plink,vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBody::PyLink* plink, object bodyexcluded, object linkexcluded, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        std::set<KinBody*> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody* pbody = extract<PyKinBody*>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.insert(pbody->_pbody);
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::set<KinBody::Link*> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLink* plink = extract<PyKinBody::PyLink*>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.insert(plink->_plink);
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        bool bSuccess = _penv->CheckCollision(plink->_plink, vbodyexcluded, vlinkexcluded, &report);
        pReport->init(report,this);
        return bSuccess;
    }

    bool CheckCollision(PyKinBody* pbody, object bodyexcluded, object linkexcluded)
    {
        std::set<KinBody*> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody* pbody = extract<PyKinBody*>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.insert(pbody->_pbody);
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::set<KinBody::Link*> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLink* plink = extract<PyKinBody::PyLink*>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.insert(plink->_plink);
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        return _penv->CheckCollision(pbody->_pbody,vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBody* pbody, object bodyexcluded, object linkexcluded, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        std::set<KinBody*> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody* pbody = extract<PyKinBody*>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.insert(pbody->_pbody);
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::set<KinBody::Link*> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLink* plink = extract<PyKinBody::PyLink*>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.insert(plink->_plink);
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        bool bSuccess = _penv->CheckCollision(pbody->_pbody, vbodyexcluded, vlinkexcluded, &report);
        pReport->init(report,this);
        return bSuccess;
    }

    bool CheckCollision(PyRay* pyray, PyKinBody* pbody)
    {
        return _penv->CheckCollision(pyray->r,pbody->_pbody);
    }

    bool CheckCollision(PyRay* pyray, PyKinBody* pbody, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(pyray->r, pbody->_pbody, &report);
        pReport->init(report,this);
        return bSuccess;
    }

    /// check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not
    object CheckCollisionRays(object rays, PyKinBody* pbody,bool bFrontFacingOnly=false)
    {
        object shape = rays.attr("shape");
        int num = extract<int>(shape[1]);
        if( extract<int>(shape[0]) != 6 )
            throw openrave_exception("rays object needs to be a 6xN vector\n");

        COLLISIONREPORT report;
        RAY r;
        npy_intp dims[] = {3,num};
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pypos);
        PyObject* pycollision = PyArray_SimpleNew(1,&dims[1], PyArray_BOOL);
        bool* pcollision = (bool*)PyArray_DATA(pycollision);
        for(int i = 0; i < num; ++i) {
            r.pos.x = extract<dReal>(rays[0][i]);
            r.pos.y = extract<dReal>(rays[1][i]);
            r.pos.z = extract<dReal>(rays[2][i]);
            r.dir.x = extract<dReal>(rays[3][i]);
            r.dir.y = extract<dReal>(rays[4][i]);
            r.dir.z = extract<dReal>(rays[5][i]);
            bool bCollision = _penv->CheckCollision(r, pbody->_pbody, &report);
            pcollision[i] = false;
            if( bCollision && report.contacts.size() > 0 ) {
                if( !bFrontFacingOnly || dot3(report.contacts[0].norm,r.dir)<0 ) {
                    pcollision[i] = true;
                    ppos[i] = report.contacts[0].pos.x;
                    ppos[i+num] = report.contacts[0].pos.y;
                    ppos[i+2*num] = report.contacts[0].pos.z;
                }
            }
        }

        return make_tuple(static_cast<numeric::array>(handle<>(pycollision)),static_cast<numeric::array>(handle<>(pypos)));
    }

    bool CheckCollision(PyRay* pyray)
    {
        return _penv->CheckCollision(pyray->r);
    }
    
    bool CheckCollision(PyRay* pyray, PyCollisionReport* pReport)
    {
        COLLISIONREPORT report;
        bool bSuccess = _penv->CheckCollision(pyray->r, &report);
        pReport->init(report,this);
        return bSuccess;
    }
	
    bool Load(const string& filename, bool bLockPhysics=true)
    {
        boost::shared_ptr<LockEnvironment> envlock;
        if(bLockPhysics)
            envlock.reset(new LockEnvironment(_penv.get()));
        return _penv->Load(filename.c_str());
    }
    bool Save(const string& filename, bool bLockPhysics=true)
    {
        boost::shared_ptr<LockEnvironment> envlock;
        if(bLockPhysics)
            envlock.reset(new LockEnvironment(_penv.get()));
        return _penv->Save(filename.c_str());
    }

    PyRobotBase* ReadRobotXML(const string& filename)
    {
        LockEnvironment envlock(_penv.get());
        return toPyRobot(_penv->ReadRobotXML(NULL,filename.c_str(),NULL),true);
    }
    PyRobotBase* ReadRobotXML(const string& filename, bool lockphysics=true)
    {
        boost::shared_ptr<LockEnvironment> envlock;
        if(lockphysics)
            envlock.reset(new LockEnvironment(_penv.get()));
        return toPyRobot(_penv->ReadRobotXML(NULL,filename.c_str(),NULL),true);
    }

    PyKinBody* ReadKinBodyXML(const string& filename)
    {
        LockEnvironment envlock(_penv.get());
        return toPyKinBody(_penv->ReadKinBodyXML(NULL,filename.c_str(),NULL),true);
    }

    PyKinBody* ReadKinBodyXML(const string& filename, bool lockphysics=true)
    {
        boost::shared_ptr<LockEnvironment> envlock;
        if(lockphysics)
            envlock.reset(new LockEnvironment(_penv.get()));
        return toPyKinBody(_penv->ReadKinBodyXML(NULL,filename.c_str(),NULL),true);
    }

    bool AddKinBody(PyKinBody* pbody) { CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody()); return _penv->AddKinBody(pbody->GetBody()); }
    bool AddRobot(PyRobotBase* robot) { CHECK_POINTER(robot); CHECK_POINTER(robot->GetRobot()); return _penv->AddRobot(robot->GetRobot()); }
    bool RemoveKinBody(PyKinBody* pbody) { CHECK_POINTER(pbody); CHECK_POINTER(pbody->GetBody()); return _penv->RemoveKinBody(pbody->GetBody(),false); }
    
    PyKinBody* GetKinBody(const string& name) { return toPyKinBody(_penv->GetKinBody(_stdmbstowcs(name.c_str()).c_str()),false); }
    PyKinBody* GetBodyFromNetworkId(int id) { return toPyKinBody(_penv->GetBodyFromNetworkId(id),false); }

    PyKinBody* CreateKinBody()
    {
        LockEnvironment envlock(_penv.get());
        return toPyKinBody(_penv->CreateKinBody(),true);
    }
    PyKinBody* CreateKinBody(bool lockphysics=true)
    {
        boost::shared_ptr<LockEnvironment> envlock;
        if(lockphysics)
            envlock.reset(new LockEnvironment(_penv.get()));
        return toPyKinBody(_penv->CreateKinBody(),true);
    }

//    Trajectory* CreateTrajectory(int nDOF) {}
//
    int LoadProblem(PyProblemInstance* prob, const string& args) { CHECK_POINTER(prob); CHECK_POINTER(prob->GetProblem()); return _penv->LoadProblem(prob->GetProblem(),args.c_str()); }
    bool RemoveProblem(PyProblemInstance* prob) { CHECK_POINTER(prob); CHECK_POINTER(prob->GetProblem()); return _penv->RemoveProblem(prob->GetProblem()); }
    object GetProblems()
    {
        boost::python::list problems;
        FOREACHC(itprob, _penv->GetProblems())
            problems.append(object(PyProblemInstance(*itprob,this,false)));
        return problems;
    }

    bool SetPhysicsEngine(PyPhysicsEngineBase* pengine)
    {
        if( pengine == NULL )
            return _penv->SetPhysicsEngine(NULL);
        CHECK_POINTER(pengine->GetPhysicsEngine());
        return _penv->SetPhysicsEngine(pengine->GetPhysicsEngine());
    }
    PyPhysicsEngineBase* GetPhysicsEngine() { return new PyPhysicsEngineBase(_penv->GetPhysicsEngine(),this,false); }

    void StepSimulation(dReal timeStep) { _penv->StepSimulation(timeStep); }
    void StartSimulation(dReal fDeltaTime) { _penv->StartSimulation(fDeltaTime); }
    void StopSimulation() { _penv->StopSimulation(); }
    uint64_t GetSimulationTime() { return _penv->GetSimulationTime(); }

    void LockPhysics(bool bLock)
    {
        if( !_penv->LockPhysics(bLock) )
            throw openrave_exception(bLock?"failed to lock physics\n":"failed to unlock physics");
    }
    void LockPhysics(bool bLock, float timeout)
    {
        if( !_penv->LockPhysics(bLock, timeout) )
            throw openrave_exception(bLock?"failed to lock physics\n":"failed to unlock physics");
    }

    bool SetViewer(const string& viewername, bool showviewer=true)
    {
        if( !!_threadviewer ) { // wait for the viewer
            _threadviewer->join();
            _threadviewer.reset();
        }
        
        
        if( viewername.size() > 0 ) {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _threadviewer.reset(new boost::thread(boost::bind(&PyEnvironmentBase::_ViewerThread, this, viewername, showviewer)));
            _conditionViewer.wait(lock);
            
            if( !_pviewer ) {
                RAVELOG_WARNA("failed to create viewer %s\n", viewername.c_str());
                _threadviewer->join();
                _threadviewer.reset();
                return false;
            }
            else
                RAVELOG_INFOA("viewer %s successfully attached\n", viewername.c_str());
        }

        return true;
    }

    PyRaveViewerBase* GetViewer() { return new PyRaveViewerBase(_penv->GetViewer(),this,false); }

    object plot3(object opoints,float pointsize,object ocolors=object(),int drawstyle=0)
    {
        object shape = opoints.attr("shape");
        int num = extract<int>(shape[0]);
        if( num <= 0 )
            throw openrave_exception("points cannot be empty");

        if( len(shape) > 1 ) {
            int dim = extract<int>(shape[1]);
            if( dim != 3 )
                throw openrave_exception("points array needs to be Nx3 matrix");
        }

        vector<float> vpoints = ExtractFloatArray(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors);
            else {
                vector<float> vcolors = ExtractFloatArray(ocolors.attr("flat"));
                if( vcolors.size() != vpoints.size() )
                    throw openrave_exception("colors needs to be Nx3 matrix");
                return object(PyGraphHandle(_penv->plot3(&vpoints[0],vpoints.size()/3,sizeof(float)*3,pointsize,&vcolors[0],drawstyle)));
            }
        }
        return object(PyGraphHandle(_penv->plot3(&vpoints[0],vpoints.size()/3,sizeof(float)*3,pointsize,vcolor,drawstyle)));
    }

    object drawlinestrip(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        object shape = opoints.attr("shape");
        int num = extract<int>(shape[0]);
        if( num <= 0 )
            throw openrave_exception("points cannot be empty");

        if( len(shape) > 1 ) {
            int dim = extract<int>(shape[1]);
            if( dim != 3 )
                throw openrave_exception("points array needs to be Nx3 matrix");
        }

        vector<float> vpoints = ExtractFloatArray(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors);
            else {
                vector<float> vcolors = ExtractFloatArray(ocolors.attr("flat"));
                if( vcolors.size() != vpoints.size() )
                    throw openrave_exception("colors needs to be Nx3 matrix");
                return object(PyGraphHandle(_penv->drawlinestrip(&vpoints[0],vpoints.size()/3,sizeof(float)*3,linewidth,&vcolors[0])));
            }
        }
        return object(PyGraphHandle(_penv->drawlinestrip(&vpoints[0],vpoints.size()/3,sizeof(float)*3,linewidth,vcolor)));
    }

    object drawlinelist(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        object shape = opoints.attr("shape");
        int num = extract<int>(shape[0]);
        if( num <= 0 )
            throw openrave_exception("points cannot be empty");

        if( len(shape) > 1 ) {
            int dim = extract<int>(shape[1]);
            if( dim != 3 )
                throw openrave_exception("points array needs to be Nx3 matrix");
        }

        vector<float> vpoints = ExtractFloatArray(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors);
            else {
                vector<float> vcolors = ExtractFloatArray(ocolors.attr("flat"));
                if( vcolors.size() != vpoints.size() )
                    throw openrave_exception("colors needs to be Nx3 matrix");
                return object(PyGraphHandle(_penv->drawlinelist(&vpoints[0],vpoints.size()/3,sizeof(float)*3,linewidth,&vcolors[0])));
            }
        }
        return object(PyGraphHandle(_penv->drawlinelist(&vpoints[0],vpoints.size()/3,sizeof(float)*3,linewidth,vcolor)));
    }
    
    object drawarrow(object op1, object op2, float linewidth=0.002, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolor != object() )
            vcolor = ExtractVector34(ocolor);
        return object(PyGraphHandle(_penv->drawarrow(ExtractVector3(op1),ExtractVector3(op2),linewidth,vcolor)));
    }

    object drawbox(object opos, object oextents, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolor != object() )
            vcolor = ExtractVector34(ocolor);
        return object(PyGraphHandle(_penv->drawbox(ExtractVector3(opos),ExtractVector3(oextents))));
    }

    object drawtrimesh(object opoints, object oindices=object(), object ocolors=object())
    {
        object shape = opoints.attr("shape");
        int num = extract<int>(shape[0]);
        if( num <= 0 )
            throw openrave_exception("points cannot be empty");
        int dim = extract<int>(shape[1]);
        if( dim != 3 )
            throw openrave_exception("points array needs to be Nx3 matrix");
        vector<float> vpoints = ExtractFloatArray(opoints.attr("flat"));

        vector<int> vindices;
        int* pindices = NULL;
        int numTriangles = vpoints.size()/9;
        if( oindices != object() ) {
            vindices = ExtractArray<int>(oindices.attr("flat"));
            if( vindices.size() > 0 ) {
                numTriangles = vindices.size()/3;
                pindices = &vindices[0];
            }
        }

        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors);
            else {
                vector<float> vcolors = ExtractFloatArray(ocolors.attr("flat"));
                if( vcolors.size() != vpoints.size() )
                    throw openrave_exception("colors needs to be Nx3 matrix (same size as points)");
                vcolor = RaveVector<float>(vcolors[0],vcolors[1],vcolors[2],1);
            }
        }
        
        return object(PyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,vcolor)));
    }
    
    void closegraph(PyGraphHandle& phandle)
    {
        _penv->closegraph(phandle._handle);
    }

    void SetCamera(object transform) { _penv->SetCamera(ExtractTransform(transform)); }

    void SetCameraLookAt(object lookat, object campos, object camup) {
        _penv->SetCameraLookAt(ExtractFloat3(lookat), ExtractFloat3(campos), ExtractFloat3(camup));
    }
    object GetCameraTransform() { return ReturnTransform(_penv->GetCameraTransform()); }

//    object GetFractionOccluded(PyKinBody* pbody, int width, int height, float nearPlane, float farPlane, object extrinsic, object KK, double& fracOccluded)
//    {
//        PyKinBody* pbody, int width, int height, float nearPlane, float farPlane, object extrinsic, object KK, double& fracOccluded
//    }

    object GetCameraImage(int width, int height, object extrinsic, object oKK)
    {
        vector<float> vKK = ExtractFloatArray(oKK);
        if( vKK.size() != 4 )
            throw openrave_exception("KK needs to be of size 4");
        npy_intp dims[] = {height,width,3};
        PyObject *pymem = PyArray_SimpleNew(3,dims, PyArray_UBYTE);
        if( !_penv->GetCameraImage(PyArray_DATA(pymem), width,height,RaveTransform<float>(ExtractTransform(extrinsic)), &vKK[0]) ) {
            decref(pymem);
            return object();
        }
        return static_cast<numeric::array>(handle<>(pymem));
    }

    bool WriteCameraImage(int width, int height, object extrinsic, object oKK, const string& filename, const string& extension)
    {
        vector<float> vKK = ExtractFloatArray(oKK);
        if( vKK.size() != 4 )
            throw openrave_exception("KK needs to be of size 4");
        return _penv->WriteCameraImage(width,height,RaveTransform<float>(ExtractTransform(extrinsic)), &vKK[0], filename.c_str(), extension.c_str());
    }

    object GetRobots()
    {
        boost::python::list robots;
        FOREACHC(itrobot, _penv->GetRobots())
            robots.append(PyRobotBase(*itrobot,this,false));
        return robots;
    }

    object GetBodies()
    {
        boost::python::list bodies;
        FOREACHC(itbody, _penv->GetBodies())
            bodies.append(PyKinBody(*itbody,this,false));
        return bodies;
    }

//    void GetPublishedBodies(std::vector<EnvironmentBase::BODYSTATE>& vbodies) {}
    void SetPublishBodiesAnytime(bool bAnytime) { _penv->SetPublishBodiesAnytime(bAnytime); }
    bool GetPublishBodiesAnytime() const { return _penv->GetPublishBodiesAnytime(); }

//    EnvironmentBase::EnvLock* GetLockedBodies(std::vector<KinBody*>& bodies) const {}
//    EnvironmentBase::EnvLock* GetLockedRobots(std::vector<RobotBase*>& robots) const {}

    PyKinBody::PyLink::PyTriMesh* Triangulate(PyKinBody* pbody)
    {
        KinBody::Link::TRIMESH mesh;
        if( !_penv->Triangulate(mesh,pbody->GetBody()) )
            return NULL;
        return new PyKinBody::PyLink::PyTriMesh(mesh);
    }
    PyKinBody::PyLink::PyTriMesh* TriangulateScene(EnvironmentBase::TriangulateOptions opts, const string& name)
    {
        KinBody::Link::TRIMESH mesh;
        if( !_penv->TriangulateScene(mesh,opts,_stdmbstowcs(name.c_str()).c_str()) )
            return NULL;
        return new PyKinBody::PyLink::PyTriMesh(mesh);
    }

    void SetDebugLevel(DebugLevel level) { _penv->SetDebugLevel(level); }
    DebugLevel GetDebugLevel() const { return _penv->GetDebugLevel(); }

    bool AttachServer(PyRaveServerBase* pserver) { return _penv->AttachServer(pserver->GetServer()); }
    PyRaveServerBase* GetServer() { return new PyRaveServerBase(_penv->GetServer(),this,false); }

    string GetHomeDirectory() { return _penv->GetHomeDirectory(); }

    // private interface stuff
    void AddInterface(PyInterfaceBase* p) { _setInterfaces.insert(p); }
    void RemoveInterface(PyInterfaceBase* p) { _setInterfaces.erase(p); }

protected:
    std::set<PyInterfaceBase*> _setInterfaces;
};

PyInterfaceBase::PyInterfaceBase(InterfaceBase* pbase, PyEnvironmentBase* pyenv, bool bOwnObject) : _pbase(pbase), _pyenv(pyenv), _bOwnObject(bOwnObject)
{
    _pyenv->AddInterface(this);
}

PyInterfaceBase::~PyInterfaceBase()
{
    if( _bOwnObject ) {
        _bOwnObject = false;
        delete _pbase;
    }

    if( _pbase != NULL )
        _pyenv->RemoveInterface(this);
}

PyKinBody::~PyKinBody()
{
    if( _bOwnObject ) {
        LockEnvironment envlock(_pbody->GetEnv());
        _pbody->GetEnv()->RemoveKinBody(_pbody,false);
        _bOwnObject = false;
        delete _pbody;
    }
}

PyRobotBase::~PyRobotBase()
{
    if( _bOwnObject ) {
        LockEnvironment envlock(_pbody->GetEnv());
        _pbody->GetEnv()->RemoveKinBody(_probot,false);
        _bOwnObject = false;
        delete _probot;
    }
}

void PyRobotBase::PyManipulator::SetIKSolver(PyIkSolverBase* iksolver)
{
    _pmanip->SetIKSolver(iksolver->GetIkSolver());
}

PySensorBase* PyRobotBase::PyAttachedSensor::GetSensor()
{
    return new PySensorBase(_pattached->GetSensor(),_pyenv,false);
}

PyControllerBase* PyRobotBase::GetController() const
{
    return new PyControllerBase(_probot->GetController(),_pyenv,false);
}

bool PyRobotBase::SetController(const string& name, const string& args)
{
    return _probot->SetController(_stdmbstowcs(name.c_str()).c_str(),args.c_str(),false);
}

bool PyRobotBase::SetController(PyControllerBase * pController, const string& args)
{
    return _probot->SetController(pController->GetController(),args.c_str(),false);
}

PyProblemInstance::~PyProblemInstance()
{
    if( _bOwnObject ) {
        LockEnvironment envlock(_pproblem->GetEnv());
        _pproblem->GetEnv()->RemoveProblem(_pproblem);
        _bOwnObject = false;
        delete _pproblem; _pproblem = NULL;
    }
}

PyCollisionCheckerBase::~PyCollisionCheckerBase()
{
    if( _bOwnObject ) {
        _bOwnObject = false;
        LockEnvironment envlock(_pCollisionChecker->GetEnv());
        if( _pCollisionChecker->GetEnv()->GetCollisionChecker() == _pCollisionChecker ) {
            RAVELOG_WARNA("resetting environment collision checker\n");
            _pCollisionChecker->GetEnv()->SetCollisionChecker(NULL);
        }
        delete _pCollisionChecker; _pCollisionChecker = NULL;
    }
}

PyPhysicsEngineBase::~PyPhysicsEngineBase()
{
    if( _bOwnObject ) {
        _bOwnObject = false;
        LockEnvironment envlock(_pPhysicsEngine->GetEnv());
        if( _pPhysicsEngine->GetEnv()->GetPhysicsEngine() == _pPhysicsEngine ) {
            RAVELOG_WARNA("resetting environment physics engine\n");
            _pPhysicsEngine->GetEnv()->SetCollisionOptions(NULL);
        }
        delete _pPhysicsEngine; _pPhysicsEngine = NULL;
    }
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetViewer_overloads, SetViewer, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(plot3_overloads, plot3, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinestrip_overloads, drawlinestrip, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinelist_overloads, drawlinelist, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawarrow_overloads, drawarrow, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawbox_overloads, drawbox, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawtrimesh_overloads, drawtrimesh, 1, 3)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Load_overloads, Load, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Save_overloads, Save, 1, 2)

BOOST_PYTHON_MODULE(openravepy)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    register_exception_translator<openrave_exception>(&translate_openrave_exception);

#ifndef _WIN32
    mkdir("~/.openrave",644);
#endif

    enum_<DebugLevel>("DebugLevel")
        .value("Fatal",Level_Fatal)
        .value("Error",Level_Error)
        .value("Warn",Level_Warn)
        .value("Info",Level_Info)
        .value("Debug",Level_Debug)
        .value("Verbose",Level_Verbose)
        ;
    enum_<PluginType>("PluginType")
        .value("Planner",PT_Planner)
        .value("Robot",PT_Robot)
        .value("SensorSystem",PT_SensorSystem)
        .value("Controller",PT_Controller)
        .value("ProblemInstance",PT_ProblemInstance)
        .value("InverseKinematicsSolver",PT_InverseKinematicsSolver)
        .value("KinBody",PT_KinBody)
        .value("PhysicsEngine",PT_PhysicsEngine)
        .value("Sensor",PT_Sensor)
        .value("CollisionChecker",PT_CollisionChecker)
        .value("Trajectory",PT_Trajectory)
        .value("Viewer",PT_Viewer)
        .value("Server",PT_Server)
        ;
    enum_<CollisionOptions>("CollisionOptions")
        .value("Distance",CO_Distance)
        .value("UseTolerance",CO_UseTolerance)
        .value("Contacts",CO_Contacts)
        .value("RayAnyHit",CO_RayAnyHit)
        ;

    enum_<CloningOptions>("CloningOptions")
        .value("Bodies",Clone_Bodies)
        .value("Viewer",Clone_Viewer)
        .value("Simulation",Clone_Simulation)
        .value("RealControllers",Clone_RealControllers)
        ;

    enum_<PhysicsEngineOptions>("PhysicsEngineOptions")
        .value("SelfCollisions",PEO_SelfCollisions)
        ;

    class_<PyEnvironmentBase> classenv("Environment");
    class_<PyGraphHandle>("GraphHandle");
    class_<PyRay>("Ray")
        .def(init<object,object>())
        .def("dir",&PyRay::dir)
        .def("pos",&PyRay::pos)
        .def_pickle(Ray_pickle_suite())
        ;
    class_<PyAABB>("AABB")
        .def(init<object,object>())
        .def("extents",&PyAABB::extents)
        .def("pos",&PyAABB::pos)
        .def_pickle(AABB_pickle_suite())
        ;
    class_<PyInterfaceBase>("Interface", no_init)
        .def("GetXMLId",&PyInterfaceBase::GetXMLId)
        .def("GetPluginName",&PyInterfaceBase::GetPluginName)
        .def("GetEnv",&PyInterfaceBase::GetEnv, return_internal_reference<1>())
        .def("Clone",&PyInterfaceBase::Clone)
        .def("SetUserData",&PyInterfaceBase::SetUserData)
        .def("GetUserData",&PyInterfaceBase::GetUserData)
        ;

    class_<PyPluginInfo>("PluginInfo",no_init)
        .def_readonly("robots",&PyPluginInfo::robots)
        .def_readonly("planners",&PyPluginInfo::planners)
        .def_readonly("sensorsystems",&PyPluginInfo::sensorsystems)
        .def_readonly("controllers",&PyPluginInfo::controllers)
        .def_readonly("problems",&PyPluginInfo::problems)
        .def_readonly("iksolvers",&PyPluginInfo::iksolvers)
        .def_readonly("physicsengines",&PyPluginInfo::physicsengines)
        .def_readonly("sensors",&PyPluginInfo::sensors)
        .def_readonly("collisioncheckers",&PyPluginInfo::collisioncheckers)
        .def_readonly("trajectories",&PyPluginInfo::trajectories)
        .def_readonly("viewers",&PyPluginInfo::viewers)
        .def_readonly("servers",&PyPluginInfo::servers)
        ;

    {    
        bool (PyKinBody::*pkinbodyself)() = &PyKinBody::CheckSelfCollision;
        bool (PyKinBody::*pkinbodyselfr)(PyCollisionReport*) = &PyKinBody::CheckSelfCollision;
        void (PyKinBody::*psetjointvalues1)(object) = &PyKinBody::SetJointValues;
        void (PyKinBody::*psetjointvalues2)(object,object) = &PyKinBody::SetJointValues;
        scope kinbody = class_<PyKinBody, bases<PyInterfaceBase> >("KinBody", no_init)
            .def("Init",&PyKinBody::Init)
            .def("GetName",&PyKinBody::GetName)
            .def("GetDOF",&PyKinBody::GetDOF)
            .def("GetJointValues",&PyKinBody::GetJointValues)
            .def("GetJointLimits",&PyKinBody::GetJointLimits)
            .def("GetLinks",&PyKinBody::GetLinks)
            .def("GetJoints",&PyKinBody::GetJoints)
            .def("GetTransform",&PyKinBody::GetTransform)
            .def("GetBodyTransformations",&PyKinBody::GetBodyTransformations)
            .def("SetBodyTransformations",&PyKinBody::SetBodyTransformations)
            .def("GetLinkVelocities",&PyKinBody::GetLinkVelocities)
            .def("ComputeAABB",&PyKinBody::ComputeAABB, return_value_policy<manage_new_object>())
            .def("Enable",&PyKinBody::Enable)
            .def("IsEnabled",&PyKinBody::IsEnabled)
            .def("SetTransform",&PyKinBody::SetTransform)
            .def("SetJointValues",psetjointvalues1)
            .def("SetJointValues",psetjointvalues2)
            .def("SetTransformWithJointValues",&PyKinBody::SetTransformWithJointValues)
            .def("CalculateJacobian",&PyKinBody::CalculateJacobian)
            .def("CalculateRotationJacobian",&PyKinBody::CalculateRotationJacobian)
            .def("CalculateAngularVelocityJacobian",&PyKinBody::CalculateAngularVelocityJacobian)
            .def("CheckSelfCollision",pkinbodyself)
            .def("CheckSelfCollision",pkinbodyselfr)
            .def("AttachBody",&PyKinBody::AttachBody)
            .def("RemoveBody",&PyKinBody::RemoveBody)
            .def("IsAttached",&PyKinBody::IsAttached)
            .def("GetAttached",&PyKinBody::GetAttached)
            .def("IsRobot",&PyKinBody::IsRobot)
            .def("GetNetworkId",&PyKinBody::GetNetworkId)
            .def("DoesAffect",&PyKinBody::DoesAffect)
            .def("GetForwardKinematics",&PyKinBody::GetForwardKinematics)
            .def("SetGuiData",&PyKinBody::SetGuiData)
            .def("GetGuiData",&PyKinBody::GetGuiData)
            .def("GetXMLFilename",&PyKinBody::GetXMLFilename)
            .def("GetNonAdjacentLinks",&PyKinBody::GetNonAdjacentLinks)
            .def("GetAdjacentLinks",&PyKinBody::GetAdjacentLinks)
            .def("GetPhysicsData",&PyKinBody::GetPhysicsData)
            .def("GetCollisionData",&PyKinBody::GetCollisionData)
            .def("GetUpdateStamp",&PyKinBody::GetUpdateStamp)
            ;

        {        
            scope link = class_<PyKinBody::PyLink>("Link", no_init)
                .def("GetName",&PyKinBody::PyLink::GetName)
                .def("GetIndex",&PyKinBody::PyLink::GetIndex)
                .def("IsEnabled",&PyKinBody::PyLink::IsEnabled)
                .def("IsStatic",&PyKinBody::PyLink::IsStatic)
                .def("Enable",&PyKinBody::PyLink::Enable)
                .def("GetParent",&PyKinBody::PyLink::GetParent, return_value_policy<manage_new_object>())
                .def("GetCollisionData",&PyKinBody::PyLink::GetCollisionData, return_value_policy<manage_new_object>())
                .def("ComputeAABB",&PyKinBody::PyLink::ComputeAABB, return_value_policy<manage_new_object>())
                .def("GetTransform",&PyKinBody::PyLink::GetTransform)
                .def("GetCOMOffset",&PyKinBody::PyLink::GetCOMOffset)
                .def("GetInertia",&PyKinBody::PyLink::GetInertia)
                .def("GetMass",&PyKinBody::PyLink::GetMass)
                .def("SetTransform",&PyKinBody::PyLink::SetTransform)
                .def("SetForce",&PyKinBody::PyLink::SetForce)
                .def("SetTorque",&PyKinBody::PyLink::SetTorque)
                ;

            class_<PyKinBody::PyLink::PyTriMesh>("TriMesh",no_init)
                .def_readwrite("vertices",&PyKinBody::PyLink::PyTriMesh::vertices)
                .def_readwrite("indices",&PyKinBody::PyLink::PyTriMesh::indices)
                ;
        }

        class_<PyKinBody::PyJoint>("Joint",no_init)
            .def("GetName", &PyKinBody::PyJoint::GetName)
            .def("GetMimicJointIndex", &PyKinBody::PyJoint::GetMimicJointIndex)
            .def("GetMimicCoeffs", &PyKinBody::PyJoint::GetMimicCoeffs)
            .def("GetMaxVel", &PyKinBody::PyJoint::GetMaxVel)
            .def("GetMaxAccel", &PyKinBody::PyJoint::GetMaxAccel)
            .def("GetMaxTorque", &PyKinBody::PyJoint::GetMaxTorque)
            .def("GetDOFIndex", &PyKinBody::PyJoint::GetDOFIndex)
            .def("GetJointIndex", &PyKinBody::PyJoint::GetJointIndex)
            .def("GetParent", &PyKinBody::PyJoint::GetParent, return_value_policy<manage_new_object>())
            .def("GetFirstAttached", &PyKinBody::PyJoint::GetFirstAttached, return_value_policy<manage_new_object>())
            .def("GetSecondAttached", &PyKinBody::PyJoint::GetSecondAttached, return_value_policy<manage_new_object>())
            .def("GetType", &PyKinBody::PyJoint::GetType)
            .def("GetDOF", &PyKinBody::PyJoint::GetDOF)
            .def("GetValues", &PyKinBody::PyJoint::GetValues)
            .def("GetVelocities", &PyKinBody::PyJoint::GetVelocities)
            .def("GetAnchor", &PyKinBody::PyJoint::GetAnchor)
            .def("GetAxis", &PyKinBody::PyJoint::GetAxis)
            .def("GetLimits", &PyKinBody::PyJoint::GetLimits)
            ;
    }

    class_<PyCollisionReport::PYCONTACT>("Contact")
        .def_readwrite("pos",&PyCollisionReport::PYCONTACT::pos)
        .def_readwrite("norm",&PyCollisionReport::PYCONTACT::norm)
        ;
    class_<PyCollisionReport>("CollisionReport")
        .def_readwrite("options",&PyCollisionReport::options)
        .def_readwrite("plink1",&PyCollisionReport::plink1)
        .def_readwrite("plink2",&PyCollisionReport::plink2)
        .def_readwrite("numCols",&PyCollisionReport::numCols)
        .def_readwrite("minDistance",&PyCollisionReport::minDistance)
        .def_readwrite("numWithinTol",&PyCollisionReport::numWithinTol)
        .def_readwrite("contacts",&PyCollisionReport::contacts)
        ;

    {
        bool (PyRobotBase::*psetcontroller1)(const string&,const string&) = &PyRobotBase::SetController;
        bool (PyRobotBase::*psetcontroller2)(PyControllerBase*,const string&) = &PyRobotBase::SetController;

        void (PyRobotBase::*psetactivedofs1)(object) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(object, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(object, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBody*) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBody*,object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab3)(PyKinBody*,int,object) = &PyRobotBase::Grab;

        scope robot = class_<PyRobotBase, bases<PyKinBody, PyInterfaceBase> >("Robot", no_init)
            .def("GetManipulators",&PyRobotBase::GetManipulators)
            .def("SetActiveManipulator",&PyRobotBase::SetActiveManipulator)
            .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, return_value_policy<manage_new_object>())
            .def("GetActiveManipulatorIndex",&PyRobotBase::GetActiveManipulatorIndex)
            .def("GetSensors",&PyRobotBase::GetSensors)
            .def("GetController",&PyRobotBase::GetController, return_value_policy<manage_new_object>())
            .def("SetController",psetcontroller1)
            .def("SetController",psetcontroller2)
            .def("SetActiveDOFs",psetactivedofs1)
            .def("SetActiveDOFs",psetactivedofs2)
            .def("SetActiveDOFs",psetactivedofs3)
            .def("GetActiveDOF",&PyRobotBase::GetActiveDOF)
            .def("GetAffineDOF",&PyRobotBase::GetAffineDOF)
            .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis)
            .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits)
            .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits)
            .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits)
            .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits)
            .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels)
            .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels)
            .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels)
            .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels)
            .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution)
            .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution)
            .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution)
            .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution)
            .def("GetAffineTranslationLimits",&PyRobotBase::GetAffineTranslationLimits)
            .def("GetAffineRotationAxisLimits",&PyRobotBase::GetAffineRotationAxisLimits)
            .def("GetAffineRotation3DLimits",&PyRobotBase::GetAffineRotation3DLimits)
            .def("GetAffineRotationQuatLimits",&PyRobotBase::GetAffineRotationQuatLimits)
            .def("GetAffineTranslationMaxVels",&PyRobotBase::GetAffineTranslationMaxVels)
            .def("GetAffineRotationAxisMaxVels",&PyRobotBase::GetAffineRotationAxisMaxVels)
            .def("GetAffineRotation3DMaxVels",&PyRobotBase::GetAffineRotation3DMaxVels)
            .def("GetAffineRotationQuatMaxVels",&PyRobotBase::GetAffineRotationQuatMaxVels)
            .def("GetAffineTranslationResolution",&PyRobotBase::GetAffineTranslationResolution)
            .def("GetAffineRotationAxisResolution",&PyRobotBase::GetAffineRotationAxisResolution)
            .def("GetAffineRotation3DResolution",&PyRobotBase::GetAffineRotation3DResolution)
            .def("GetAffineRotationQuatResolution",&PyRobotBase::GetAffineRotationQuatResolution)
            .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues)
            .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues)
            .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities)
            .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities)
            .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits)
            .def("GetActiveJointIndex",&PyRobotBase::GetActiveJointIndex)
            .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
            .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian)
            .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian)
            .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian)
            .def("Grab",pgrab1)
            .def("Grab",pgrab2)
            .def("Grab",pgrab3)
            .def("Release",&PyRobotBase::Release)
            .def("ReleaseAllGrabbed",&PyRobotBase::ReleaseAllGrabbed)
            .def("RegrabAll",&PyRobotBase::RegrabAll)
            .def("IsGrabbing",&PyRobotBase::IsGrabbing)
            .def("GetGrabbed",&PyRobotBase::GetGrabbed)
            .def("WaitForController",&PyRobotBase::WaitForController)
            ;
        
        object (PyRobotBase::PyManipulator::*pmanipik)(object, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        class_<PyRobotBase::PyManipulator>("Manipulator", no_init)
            .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetEndEffectorTransform)
            .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIKSolver)
            .def("InitIKSolver",&PyRobotBase::PyManipulator::InitIKSolver)
            .def("GetIKSolverName",&PyRobotBase::PyManipulator::GetIKSolverName)
            .def("HasIKSolver",&PyRobotBase::PyManipulator::HasIKSolver)
            .def("GetName",&PyRobotBase::PyManipulator::GetName)
            .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters)
            .def("FindIKSolution",pmanipik)
            .def("FindIKSolution",pmanipikf)
            .def("FindIKSolutions",pmanipiks)
            .def("FindIKSolutions",pmanipiksf)
            .def("GetBase",&PyRobotBase::PyManipulator::GetBase, return_value_policy<manage_new_object>())
            .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, return_value_policy<manage_new_object>())
            .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform)
            .def("GetJoints",&PyRobotBase::PyManipulator::GetJoints)
            .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints)
            .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints)
            .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices)
            .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks)
            ;

        class_<PyRobotBase::PyAttachedSensor>("AttachedSensor", no_init)
            .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, return_value_policy<manage_new_object>())
            .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, return_value_policy<manage_new_object>())
            .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform)
            .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot,return_value_policy<manage_new_object>())
            .def("GetName",&PyRobotBase::PyAttachedSensor::GetName)
            ;

        class_<PyRobotBase::PyGrabbed>("Grabbed",no_init)
            .def_readwrite("grabbedbody",&PyRobotBase::PyGrabbed::grabbedbody)
            .def_readwrite("linkrobot",&PyRobotBase::PyGrabbed::linkrobot)
            .def_readwrite("validColLinks",&PyRobotBase::PyGrabbed::validColLinks)
            .def_readwrite("troot",&PyRobotBase::PyGrabbed::troot)
            ;

        enum_<RobotBase::DOFAffine>("DOFAffine")
            .value("NoTransform",RobotBase::DOF_NoTransform)
            .value("X",RobotBase::DOF_X)
            .value("Y",RobotBase::DOF_Y)
            .value("Z",RobotBase::DOF_Z)
            .value("RotationAxis",RobotBase::DOF_RotationAxis)
            .value("Rotation3D",RobotBase::DOF_Rotation3D)
            .value("RotationQuat",RobotBase::DOF_RotationQuat)
            ;
    }

    class_<PyPlannerBase, bases<PyInterfaceBase> >("Planner", no_init);
    class_<PySensorSystemBase, bases<PyInterfaceBase> >("SensorSystem", no_init);
    class_<PyTrajectoryBase, bases<PyInterfaceBase> >("Trajectory", no_init);
    class_<PyControllerBase, bases<PyInterfaceBase> >("Controller", no_init)
        .def("Init",&PyControllerBase::Init)
        .def("Reset",&PyControllerBase::Reset)
        .def("SetDesired",&PyControllerBase::SetDesired)
        .def("SetPath",&PyControllerBase::SetPath)
        .def("SimulationStep",&PyControllerBase::SimulationStep)
        .def("IsDone",&PyControllerBase::IsDone)
        .def("GetTime",&PyControllerBase::GetTime)
        .def("GetVelocity",&PyControllerBase::GetVelocity)
        .def("GetTorque",&PyControllerBase::GetTorque)
        .def("SendCmd",&PyControllerBase::SendCmd)
        .def("SupportsCmd",&PyControllerBase::SupportsCmd)
        ;
    class_<PyProblemInstance, bases<PyInterfaceBase> >("Problem", no_init)
        .def("SimulationStep",&PyProblemInstance::SimulationStep)
        .def("SendCommand",&PyProblemInstance::SendCommand)
        ;
    class_<PyIkSolverBase, bases<PyInterfaceBase> >("IkSolver", no_init);

    object (PyPhysicsEngineBase::*SetPhysicsOptions1)(const string&) = &PyPhysicsEngineBase::SetPhysicsOptions;
    bool (PyPhysicsEngineBase::*SetPhysicsOptions2)(int) = &PyPhysicsEngineBase::SetPhysicsOptions;
    bool (PyPhysicsEngineBase::*SetBodyVelocity1)(PyKinBody*, object, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    bool (PyPhysicsEngineBase::*SetBodyVelocity2)(PyKinBody*, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    class_<PyPhysicsEngineBase, bases<PyInterfaceBase> >("PhysicsEngine", no_init)
        .def("GetPhysicsOptions",&PyPhysicsEngineBase::GetPhysicsOptions)
        .def("GetPhysicsOptions",SetPhysicsOptions1)
        .def("SetPhysicsOptions",SetPhysicsOptions2)
        .def("InitEnvironment",&PyPhysicsEngineBase::InitEnvironment)
        .def("DestroyEnvironment",&PyPhysicsEngineBase::DestroyEnvironment)
        .def("InitKinBody",&PyPhysicsEngineBase::InitKinBody)
        .def("DestroyKinBody",&PyPhysicsEngineBase::DestroyKinBody)
        .def("SetBodyVelocity",SetBodyVelocity1)
        .def("SetBodyVelocity",SetBodyVelocity2)
        .def("GetBodyVelocityJoints",&PyPhysicsEngineBase::GetBodyVelocityJoints)
        .def("GetBodyVelocityLinks",&PyPhysicsEngineBase::GetBodyVelocityLinks)
        .def("SetJointVelocity",&PyPhysicsEngineBase::SetJointVelocity)
        .def("GetJointVelocity",&PyPhysicsEngineBase::GetJointVelocity)
        .def("SetBodyForce",&PyPhysicsEngineBase::SetBodyForce)
        .def("SetBodyTorque",&PyPhysicsEngineBase::SetBodyTorque)
        .def("AddJointTorque",&PyPhysicsEngineBase::AddJointTorque)
        .def("SetGravity",&PyPhysicsEngineBase::SetGravity)
        .def("GetGravity",&PyPhysicsEngineBase::GetGravity)
        .def("SimulateStep",&PyPhysicsEngineBase::SimulateStep)
        ;
    {
        scope sensor = class_<PySensorBase, bases<PyInterfaceBase> >("Sensor", no_init)
            .def("GetSensorData",&PySensorBase::GetSensorData,return_value_policy<manage_new_object>())
            .def("SendCmd",&PySensorBase::SendCmd)
            .def("SupportsCmd",&PySensorBase::SupportsCmd)
            .def("SetTransform",&PySensorBase::SetTransform)
            .def("GetTransform",&PySensorBase::GetTransform)
            .def("GetName",&PySensorBase::GetName)
            ;

        class_<PySensorBase::PySensorData>("SensorData",no_init)
            .def_readonly("type",&PySensorBase::PySensorData::type)
            ;
        class_<PySensorBase::PyLaserSensorData, bases<PySensorBase::PySensorData> >("LaserSensorData",no_init)
            .def_readonly("transform",&PySensorBase::PyLaserSensorData::transform)
            .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
            .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
            .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
            .def_readonly("id",&PySensorBase::PyLaserSensorData::id)
            ;
        class_<PySensorBase::PyCameraSensorData, bases<PySensorBase::PySensorData> >("CameraSensorData",no_init)
            .def_readonly("transform",&PySensorBase::PyCameraSensorData::transform)
            .def_readonly("imagedata",&PySensorBase::PyCameraSensorData::imagedata)
            .def_readonly("KK",&PySensorBase::PyCameraSensorData::KK)
            .def_readonly("id",&PySensorBase::PyCameraSensorData::id)
            ;

        enum_<SensorBase::SensorType>("SensorType")
            .value("Invalid",SensorBase::ST_Invalid)
            .value("Laser",SensorBase::ST_Laser)
            .value("Camera",SensorBase::ST_Camera)
            .value("JointEncoder",SensorBase::ST_JointEncoder)
            .value("Force6D",SensorBase::ST_Force6D)
            ;
    }

    class_<PyCollisionCheckerBase, bases<PyInterfaceBase> >("CollisionChecker", no_init);
    class_<PyRaveViewerBase, bases<PyInterfaceBase> >("Viewer", no_init)
        .def("main",&PyRaveViewerBase::main)
        .def("quitmainloop",&PyRaveViewerBase::quitmainloop)
        .def("SetSize",&PyRaveViewerBase::ViewerSetSize)
        .def("Move",&PyRaveViewerBase::ViewerMove)
        .def("SetTitle",&PyRaveViewerBase::ViewerSetTitle)
        .def("LoadModel",&PyRaveViewerBase::LoadModel)
        ;
    class_<PyRaveServerBase, bases<PyInterfaceBase> >("Server", no_init);

    bool (PyEnvironmentBase::*pcolb)(PyKinBody*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbr)(PyKinBody*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbb)(PyKinBody*,PyKinBody*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbbr)(PyKinBody*, PyKinBody*,PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoll)(PyKinBody::PyLink*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollr)(PyKinBody::PyLink*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolll)(PyKinBody::PyLink*,PyKinBody::PyLink*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolllr)(PyKinBody::PyLink*,PyKinBody::PyLink*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollb)(PyKinBody::PyLink*, PyKinBody*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollbr)(PyKinBody::PyLink*, PyKinBody*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolle)(PyKinBody::PyLink*,object,object) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoller)(PyKinBody::PyLink*, object,object,PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbe)(PyKinBody*,object,object) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolber)(PyKinBody*, object,object,PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolyb)(PyRay*,PyKinBody*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolybr)(PyRay*, PyKinBody*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoly)(PyRay*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolyr)(PyRay*, PyCollisionReport*) = &PyEnvironmentBase::CheckCollision;

    PyRobotBase* (PyEnvironmentBase::*ReadRobotXML1)(const string&) = &PyEnvironmentBase::ReadRobotXML;
    PyRobotBase* (PyEnvironmentBase::*ReadRobotXML2)(const string&,bool) = &PyEnvironmentBase::ReadRobotXML;
    PyKinBody* (PyEnvironmentBase::*ReadKinBodyXML1)(const string&) = &PyEnvironmentBase::ReadKinBodyXML;
    PyKinBody* (PyEnvironmentBase::*ReadKinBodyXML2)(const string&,bool) = &PyEnvironmentBase::ReadKinBodyXML;
    PyKinBody* (PyEnvironmentBase::*CreateKinBody0)() = &PyEnvironmentBase::CreateKinBody;
    PyKinBody* (PyEnvironmentBase::*CreateKinBody1)(bool) = &PyEnvironmentBase::CreateKinBody;

    void (PyEnvironmentBase::*LockPhysics1)(bool) = &PyEnvironmentBase::LockPhysics;
    void (PyEnvironmentBase::*LockPhysics2)(bool, float) = &PyEnvironmentBase::LockPhysics;

    {
        scope env = classenv
            .def("Reset",&PyEnvironmentBase::Reset)
            .def("GetPluginInfo",&PyEnvironmentBase::GetPluginInfo)
            .def("GetLoadedInterfaces",&PyEnvironmentBase::GetLoadedInterfaces, return_value_policy<manage_new_object>())
            .def("LoadPlugin",&PyEnvironmentBase::LoadPlugin)
            .def("CreateRobot", &PyEnvironmentBase::CreateRobot, return_value_policy<manage_new_object>() )
            .def("CreatePlanner", &PyEnvironmentBase::CreatePlanner, return_value_policy<manage_new_object>() )
            .def("CreateSensorSystem", &PyEnvironmentBase::CreateSensorSystem, return_value_policy<manage_new_object>() )
            .def("CreateController", &PyEnvironmentBase::CreateController, return_value_policy<manage_new_object>() )
            .def("CreateProblem", &PyEnvironmentBase::CreateProblem, return_value_policy<manage_new_object>() )
            .def("CreateIkSolver", &PyEnvironmentBase::CreateIkSolver, return_value_policy<manage_new_object>() )
            .def("CreatePhysicsEngine", &PyEnvironmentBase::CreatePhysicsEngine, return_value_policy<manage_new_object>() )
            .def("CreateSensor", &PyEnvironmentBase::CreateSensor, return_value_policy<manage_new_object>() )
            .def("CreateCollisionChecker", &PyEnvironmentBase::CreateCollisionChecker, return_value_policy<manage_new_object>() )
            .def("CreateViewer", &PyEnvironmentBase::CreateViewer, return_value_policy<manage_new_object>() )
            .def("CreateServer", &PyEnvironmentBase::CreateServer, return_value_policy<manage_new_object>() )
        
            .def("CloneSelf",&PyEnvironmentBase::CloneSelf, return_value_policy<manage_new_object>())
            .def("SetCollisionChecker",&PyEnvironmentBase::SetCollisionChecker)
            .def("GetCollisionChecker",&PyEnvironmentBase::GetCollisionChecker, return_value_policy<manage_new_object>() )

            .def("CheckCollision",pcolb)
            .def("CheckCollision",pcolbr )
            .def("CheckCollision",pcolbb)
            .def("CheckCollision",pcolbbr )
            .def("CheckCollision",pcoll)
            .def("CheckCollision",pcollr )
            .def("CheckCollision",pcolll)
            .def("CheckCollision",pcolllr )
            .def("CheckCollision",pcollb)
            .def("CheckCollision",pcollbr )
            .def("CheckCollision",pcolle)
            .def("CheckCollision",pcoller )
            .def("CheckCollision",pcolbe)
            .def("CheckCollision",pcolber )
            .def("CheckCollision",pcolyb)
            .def("CheckCollision",pcolybr )
            .def("CheckCollision",pcoly)
            .def("CheckCollision",pcolyr)
            .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,CheckCollisionRays_overloads(args("rays","body","front_facing_only")))
        
            .def("Load",&PyEnvironmentBase::Load, Load_overloads(args("filename","lockphysics")))
            .def("Save",&PyEnvironmentBase::Save, Save_overloads(args("filename","lockphysics")))
            .def("ReadRobotXML",ReadRobotXML1, return_value_policy<manage_new_object>())
            .def("ReadRobotXML",ReadRobotXML2, return_value_policy<manage_new_object>())
            .def("ReadKinBodyXML",ReadKinBodyXML1, return_value_policy<manage_new_object>())
            .def("ReadKinBodyXML",ReadKinBodyXML2, return_value_policy<manage_new_object>())
            .def("AddKinBody",&PyEnvironmentBase::AddKinBody)
            .def("AddRobot",&PyEnvironmentBase::AddRobot)
            .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody)
            .def("GetKinBody",&PyEnvironmentBase::GetKinBody, return_value_policy<manage_new_object>())
            .def("GetBodyFromNetworkId",&PyEnvironmentBase::GetBodyFromNetworkId, return_value_policy<manage_new_object>() )
            .def("CreateKinBody",CreateKinBody0, return_value_policy<manage_new_object>() )
            .def("CreateKinBody",CreateKinBody1, return_value_policy<manage_new_object>() )

            .def("LoadProblem",&PyEnvironmentBase::LoadProblem)
            .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem)
            .def("GetProblems",&PyEnvironmentBase::GetProblems)
            .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine)
            .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine, return_value_policy<manage_new_object>())
            .def("StepSimulation",&PyEnvironmentBase::StepSimulation)
            .def("StartSimulation",&PyEnvironmentBase::StartSimulation)
            .def("StopSimulation",&PyEnvironmentBase::StopSimulation)
            .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime)
            .def("LockPhysics",LockPhysics1)
            .def("LockPhysics",LockPhysics2)
            .def("SetViewer",&PyEnvironmentBase::SetViewer,SetViewer_overloads(args("viewername","showviewer")))
            .def("GetViewer",&PyEnvironmentBase::GetViewer, return_value_policy<manage_new_object>())
            .def("plot3",&PyEnvironmentBase::plot3,plot3_overloads(args("points","pointsize","colors","drawstyle")))
            .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,drawlinestrip_overloads(args("points","linewidth","colors","drawstyle")))
            .def("drawlinelist",&PyEnvironmentBase::drawlinelist,drawlinelist_overloads(args("points","linewidth","colors","drawstyle")))
            .def("drawarrow",&PyEnvironmentBase::drawarrow,drawarrow_overloads(args("p1","p2","linewidth","color")))
            .def("drawbox",&PyEnvironmentBase::drawbox,drawbox_overloads(args("pos","extents","color")))
            .def("drawtrimesh",&PyEnvironmentBase::drawtrimesh,drawtrimesh_overloads(args("points","indices","colors")))
            .def("closegraph",&PyEnvironmentBase::closegraph)
            .def("SetCamera",&PyEnvironmentBase::SetCamera)
            .def("SetCameraLookAt",&PyEnvironmentBase::SetCameraLookAt)
            .def("GetCameraTransform",&PyEnvironmentBase::GetCameraTransform)
            .def("GetCameraImage",&PyEnvironmentBase::GetCameraImage)
            .def("WriteCameraImage",&PyEnvironmentBase::WriteCameraImage)
            .def("GetRobots",&PyEnvironmentBase::GetRobots)
            .def("GetBodies",&PyEnvironmentBase::GetBodies)
            .def("SetPublishBodiesAnytime",&PyEnvironmentBase::SetPublishBodiesAnytime)
            .def("GetPublishBodiesAnytime",&PyEnvironmentBase::GetPublishBodiesAnytime)
            .def("Triangulate",&PyEnvironmentBase::Triangulate, return_value_policy<manage_new_object>())
            .def("TriangulateScene",&PyEnvironmentBase::TriangulateScene, return_value_policy<manage_new_object>())
            .def("SetDebugLevel",&PyEnvironmentBase::SetDebugLevel)
            .def("GetDebugLevel",&PyEnvironmentBase::GetDebugLevel)
            .def("AttachServer",&PyEnvironmentBase::AttachServer)
            .def("GetServer",&PyEnvironmentBase::GetServer, return_value_policy<manage_new_object>())
            .def("GetHomeDirectory",&PyEnvironmentBase::GetHomeDirectory)
            ;

        enum_<EnvironmentBase::TriangulateOptions>("TriangulateOptions")
            .value("Obstacles",EnvironmentBase::TO_Obstacles)
            .value("Robots",EnvironmentBase::TO_Robots)
            .value("Everything",EnvironmentBase::TO_Everything)
            .value("Body",EnvironmentBase::TO_Body)
            .value("AllExceptBody",EnvironmentBase::TO_AllExceptBody)
            ;
    }
    
    {
        scope options = class_<DummyStruct>("options")
            .def_readwrite("ReturnTransformQuaternions",&s_bReturnTransformQuaternions);
    }
}
