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
#ifndef _WIN32
#include <sys/time.h>
#define Sleep(milli) usleep(1000*milli)
#else
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#endif

#include "openrave-core.h"
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
#include <pyconfig.h>
#include <numpy/arrayobject.h>

#define OPENRAVE_BININGS_PYARRAY
#include "bindings.h"

#define CHECK_POINTER(p) { \
        if( !(p) ) throw openrave_exception(boost::str(boost::format("[%s:%d]: invalid pointer")%__PRETTY_FUNCTION__%__LINE__)); \
}

using namespace boost::python;
using namespace std;
using namespace OpenRAVE;

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

struct null_deleter { void operator()(void const *) const {} };

/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
bool GetReturnTransformQuaternions() { return s_bReturnTransformQuaternions; }
void SetReturnTransformQuaternions(bool bset) { s_bReturnTransformQuaternions = bset; }

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
class PyRaveViewerBase;

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
typedef boost::shared_ptr<PyRaveViewerBase> PyRaveViewerBasePtr;
typedef boost::shared_ptr<PyRaveViewerBase const> PyRaveViewerBaseConstPtr;

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
    return ExtractVector3Type<dReal>(oraw);//.attr("flat"));
}

inline Vector ExtractVector4(const object& oraw)
{
    return ExtractVector4Type<dReal>(oraw);//.attr("flat"));
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
        t.m[4*i+0] = extract<T>(o[4*i+0]);
        t.m[4*i+1] = extract<T>(o[4*i+1]);
        t.m[4*i+2] = extract<T>(o[4*i+2]);
        t.trans[i] = extract<T>(o[4*i+3]);
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
        t.m[4*i+0] = extract<T>(o[4*i+0]);
        t.m[4*i+1] = extract<T>(o[4*i+1]);
        t.m[4*i+2] = extract<T>(o[4*i+2]);
        t.trans[i] = extract<T>(o[4*i+3]);
    }
    return t;
}

inline Transform ExtractTransform(const object& oraw)
{
    return ExtractTransformType<dReal>(oraw.attr("flat"));
}

inline TransformMatrix ExtractTransformMatrix(const object& oraw)
{
    return ExtractTransformMatrixType<dReal>(oraw.attr("flat"));
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

inline object toPyArray3(const vector<RaveVector<float> >& v)
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
    }

    boost::python::list interfacenames;
};

class PyGraphHandle
{
public:
    PyGraphHandle() {}
    PyGraphHandle(EnvironmentBase::GraphHandlePtr handle) : _handle(handle) {}
    virtual ~PyGraphHandle() {}

private:
    EnvironmentBase::GraphHandlePtr _handle;
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
        return boost::python::make_tuple(toPyVector3(r.r.pos),toPyVector3(r.r.dir));
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
        return boost::python::make_tuple(toPyVector3(ab.ab.pos),toPyVector3(ab.ab.extents));
    }
};

class PyInterfaceBase
{
protected:
    InterfaceBasePtr _pbase;
    PyEnvironmentBasePtr _pyenv;
public:
    PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv) : _pbase(pbase), _pyenv(pyenv)
    {
        CHECK_POINTER(_pbase);
        CHECK_POINTER(_pyenv);
    }
    virtual ~PyInterfaceBase() {}

    PluginType GetInterfaceType() const { return _pbase->GetInterfaceType(); }
    string GetXMLId() const { return _pbase->GetXMLId(); }
    string GetPluginName() const { return _pbase->GetPluginName(); }
    string GetDescription() const { return _pbase->GetDescription(); }
    PyEnvironmentBasePtr GetEnv() const { return _pyenv; }
    
    bool Clone(PyInterfaceBasePtr preference, int cloningoptions) {
        CHECK_POINTER(preference); 
        return _pbase->Clone(preference->GetInterfaceBase(),cloningoptions);
    }

    void SetUserData(PyVoidHandle pdata) { _pbase->SetUserData(pdata._handle); }
    PyVoidHandle GetUserData() const { return PyVoidHandle(_pbase->GetUserData()); }

    object SendCommand(const string& in) {
        stringstream sin(in), sout;
        if( !_pbase->SendCommand(sout,sin) )
            return object();
        return object(sout.str());
    }

    virtual InterfaceBasePtr GetInterfaceBase() { return _pbase; }
};

class PyKinBody : public PyInterfaceBase
{
protected:
    KinBodyPtr _pbody;
    std::list<boost::shared_ptr<void> > _listStateSavers;

public:
    class PyLink
    {
        KinBody::LinkPtr _plink;
        PyEnvironmentBasePtr _pyenv;
    public:
        class PyTriMesh
        {
        public:
            PyTriMesh() {}
            PyTriMesh(object vertices, object indices) : vertices(vertices), indices(indices) {}
            PyTriMesh(const KinBody::Link::TRIMESH& mesh) {
                npy_intp dims[] = {mesh.vertices.size(),3};
                PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
                dReal* pvdata = (dReal*)PyArray_DATA(pyvertices);
                FOREACHC(itv, mesh.vertices) {
                    *pvdata++ = itv->x;
                    *pvdata++ = itv->y;
                    *pvdata++ = itv->z;
                }
                vertices = static_cast<numeric::array>(handle<>(pyvertices));

                dims[0] = mesh.indices.size()/3;
                dims[1] = 3;
                PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT);
                int* pidata = (int*)PyArray_DATA(pyindices);
                FOREACHC(it, mesh.indices)
                    *pidata++ = *it;
                indices = static_cast<numeric::array>(handle<>(pyindices));
            }

            void GetTriMesh(KinBody::Link::TRIMESH& mesh) {
                int numverts = len(vertices);
                mesh.vertices.resize(numverts);
                for(int i = 0; i < numverts; ++i) {
                    object ov = vertices[i];
                    mesh.vertices[i].x = extract<dReal>(ov[0]);
                    mesh.vertices[i].y = extract<dReal>(ov[1]);
                    mesh.vertices[i].z = extract<dReal>(ov[2]);
                }

                int numtris = len(indices);
                mesh.indices.resize(3*numtris);
                for(int i = 0; i < numtris; ++i) {
                    object oi = indices[i];
                    mesh.indices[3*i+0] = extract<int>(oi[0]);
                    mesh.indices[3*i+1] = extract<int>(oi[1]);
                    mesh.indices[3*i+2] = extract<int>(oi[2]);
                }
            }

            object vertices,indices;
        };

        class PyGeomProperties
        {
            KinBody::LinkPtr _plink;
            int _geomindex;
        public:
            PyGeomProperties(KinBody::LinkPtr plink, int geomindex) : _plink(plink), _geomindex(geomindex) {}

            virtual void SetCollisionMesh(boost::shared_ptr<PyTriMesh> pytrimesh)
            {
                KinBody::Link::TRIMESH mesh;
                pytrimesh->GetTriMesh(mesh);
                _plink->GetGeometry(_geomindex).SetCollisionMesh(mesh);
            }

            boost::shared_ptr<PyTriMesh> GetCollisionMesh() { return boost::shared_ptr<PyTriMesh>(new PyTriMesh(_plink->GetGeometry(_geomindex).GetCollisionMesh())); }
            void SetDraw(bool bDraw) { _plink->GetGeometry(_geomindex).SetDraw(bDraw); }
            void SetTransparency(float f) { _plink->GetGeometry(_geomindex).SetTransparency(f); }
            bool IsDraw() { return _plink->GetGeometry(_geomindex).IsDraw(); }
            bool IsModifiable() { return _plink->GetGeometry(_geomindex).IsModifiable(); }
            KinBody::Link::GEOMPROPERTIES::GeomType GetType() { return _plink->GetGeometry(_geomindex).GetType(); }
            object GetTransform() { return ReturnTransform(_plink->GetGeometry(_geomindex).GetTransform()); }
            dReal GetSphereRadius() const { return _plink->GetGeometry(_geomindex).GetSphereRadius(); }
            dReal GetCylinderRadius() const { return _plink->GetGeometry(_geomindex).GetCylinderRadius(); }
            dReal GetCylinderHeight() const { return _plink->GetGeometry(_geomindex).GetCylinderHeight(); }
            object GetBoxExtents() const { return toPyVector3(_plink->GetGeometry(_geomindex).GetBoxExtents()); }
        };

        PyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv) : _plink(plink), _pyenv(pyenv) {}
        virtual ~PyLink() {}

        KinBody::LinkPtr GetLink() { return _plink; }

        string GetName() { return _plink->GetName(); }
        int GetIndex() { return _plink->GetIndex(); }
        bool IsEnabled() const { return _plink->IsEnabled(); }
        bool IsStatic() const { return _plink->IsStatic(); }
        void Enable(bool bEnable) { _plink->Enable(bEnable); }

        PyKinBodyPtr GetParent() const { return PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)); }
        
        boost::shared_ptr<PyTriMesh> GetCollisionData() { return boost::shared_ptr<PyTriMesh>(new PyTriMesh(_plink->GetCollisionData())); }
        boost::shared_ptr<PyAABB> ComputeAABB() const { return boost::shared_ptr<PyAABB>(new PyAABB(_plink->ComputeAABB())); }
        object GetTransform() const { return ReturnTransform(_plink->GetTransform()); }
        
        object GetCOMOffset() const { return toPyVector3(_plink->GetCOMOffset()); }
        object GetInertia() const
        {
            TransformMatrix t = _plink->GetInertia();
            npy_intp dims[] = {3,4};
            PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
            dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
            pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2]; pdata[3] = t.trans[0];
            pdata[4] = t.m[4]; pdata[5] = t.m[5]; pdata[6] = t.m[6]; pdata[7] = t.trans[1];
            pdata[8] = t.m[8]; pdata[9] = t.m[9]; pdata[10] = t.m[10]; pdata[11] = t.trans[2];
            return static_cast<numeric::array>(handle<>(pyvalues));
        }
        dReal GetMass() const { return _plink->GetMass(); }

        void SetTransform(object otrans) { _plink->SetTransform(ExtractTransform(otrans)); }
        void SetForce(object oforce, object opos, bool bAdd) { return _plink->SetForce(ExtractVector3(oforce),ExtractVector3(opos),bAdd); }
        void SetTorque(object otorque, bool bAdd) { return _plink->SetTorque(ExtractVector3(otorque),bAdd); }

        object GetGeometries()
        {
            boost::python::list geoms;
            size_t N = _plink->GetGeometries().size();
            for(size_t i = 0; i < N; ++i)
                geoms.append(boost::shared_ptr<PyGeomProperties>(new PyGeomProperties(_plink, i)));
            return geoms;
        }
    };

    typedef boost::shared_ptr<PyLink> PyLinkPtr;
    typedef boost::shared_ptr<PyLink const> PyLinkConstPtr;

    class PyJoint
    {
        KinBody::JointPtr _pjoint;
        PyEnvironmentBasePtr _pyenv;
    public:
        PyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv) : _pjoint(pjoint), _pyenv(pyenv) {}
        virtual ~PyJoint() {}

        KinBody::JointPtr GetJoint() { return _pjoint; }

        string GetName() { return _pjoint->GetName(); }
        int GetMimicJointIndex() const { return _pjoint->GetMimicJointIndex(); }
        object GetMimicCoeffs() const { return toPyArray(_pjoint->GetMimicCoeffs()); }

        dReal GetMaxVel() const { return _pjoint->GetMaxVel(); }
        dReal GetMaxAccel() const { return _pjoint->GetMaxAccel(); }
        dReal GetMaxTorque() const { return _pjoint->GetMaxTorque(); }

        int GetDOFIndex() const { return _pjoint->GetDOFIndex(); }
        int GetJointIndex() const { return _pjoint->GetJointIndex(); }
        
        PyKinBodyPtr GetParent() const { return PyKinBodyPtr(new PyKinBody(_pjoint->GetParent(),_pyenv)); }

        PyLinkPtr GetFirstAttached() const { return !_pjoint->GetFirstAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetFirstAttached(), _pyenv)); }
        PyLinkPtr GetSecondAttached() const { return !_pjoint->GetSecondAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetSecondAttached(), _pyenv)); }

        KinBody::Joint::JointType GetType() const { return _pjoint->GetType(); }

        int GetDOF() const { return _pjoint->GetDOF(); }
        object GetValues() const
        {
            vector<dReal> values;
            _pjoint->GetValues(values);
            return toPyArray(values);
        }
        object GetVelocities() const
        {
            vector<dReal> values;
            _pjoint->GetVelocities(values);
            return toPyArray(values);
        }

        object GetAnchor() const { return toPyVector3(_pjoint->GetAnchor()); }
        object GetAxis(int iaxis) { return toPyVector3(_pjoint->GetAxis(iaxis)); }
        object GetInternalHierarchyAnchor() const { return toPyVector3(_pjoint->GetInternalHierarchyAnchor()); }
        object GetInternalHierarchyAxis(int iaxis) { return toPyVector3(_pjoint->GetInternalHierarchyAxis(iaxis)); }
        object GetInternalHierarchyLeftTransform() { return ReturnTransform(_pjoint->GetInternalHierarchyLeftTransform()); }
        object GetInternalHierarchyRightTransform() { return ReturnTransform(_pjoint->GetInternalHierarchyRightTransform()); }

        object GetLimits() const {
            vector<dReal> lower, upper;
            _pjoint->GetLimits(lower,upper);
            return boost::python::make_tuple(toPyArray(lower),toPyArray(upper));
        }

        void SetJointOffset(dReal offset) { _pjoint->SetJointOffset(offset); }
        void SetJointLimits(object olower, object oupper) {
            vector<dReal> vlower = ExtractArray<dReal>(olower);
            vector<dReal> vupper = ExtractArray<dReal>(oupper);
            if( vlower.size() != vupper.size() || (int)vlower.size() != _pjoint->GetDOF() )
                throw openrave_exception("limits are wrong dimensions");
            _pjoint->SetJointLimits(vlower,vupper);
        }
        void SetResolution(dReal resolution) { _pjoint->SetResolution(resolution); }
        void SetWeights(object o) { _pjoint->SetWeights(ExtractArray<dReal>(o)); }

        void AddTorque(object otorques) {
            vector<dReal> vtorques = ExtractArray<dReal>(otorques);
            return _pjoint->AddTorque(vtorques);
        }
    };
    typedef boost::shared_ptr<PyJoint> PyJointPtr;
    typedef boost::shared_ptr<PyJoint const> PyJointConstPtr;

    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pbody,pyenv), _pbody(pbody) {}
    PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv) { _pbody = r._pbody; }
    virtual ~PyKinBody() {}
    KinBodyPtr GetBody() { return _pbody; }

    bool InitFromFile(const string& filename) { return _pbody->InitFromFile(filename,std::list<std::pair<std::string,std::string> >()); }
    bool InitFromData(const string& data) { return _pbody->InitFromData(data,std::list<std::pair<std::string,std::string> >()); }

    void SetName(const string& name) { _pbody->SetName(name); }
    string GetName() const { return _pbody->GetName(); }
    int GetDOF() const { return _pbody->GetDOF(); }

    object GetJointValues() const
    {
        vector<dReal> values;
        _pbody->GetJointValues(values);
        return toPyArray(values);
    }
    object GetJointLimits() const
    {
        vector<dReal> vlower, vupper;
        _pbody->GetJointLimits(vlower,vupper);
        return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }
    
    object GetLinks()
    {
        boost::python::list links;
        FOREACHC(itlink, _pbody->GetLinks())
            links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        return links;
    }

    PyLinkPtr GetLink(const std::string& linkname) const
    {
        KinBody::LinkPtr plink = _pbody->GetLink(linkname);
        return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,GetEnv()));
    }

    object GetJoints()
    {
        boost::python::list joints;
        FOREACHC(itjoint, _pbody->GetJoints())
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        return joints;
    }

    object GetPassiveJoints()
    {
        boost::python::list joints;
        FOREACHC(itjoint, _pbody->GetPassiveJoints())
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        return joints;
    }

    object GetDependencyOrderedJoints()
    {
        boost::python::list joints;
        FOREACHC(itjoint, _pbody->GetDependencyOrderedJoints())
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        return joints;
    }

    PyJointPtr GetJoint(const std::string& jointname) const
    {
        KinBody::JointPtr pjoint = _pbody->GetJoint(jointname);
        return !pjoint ? PyJointPtr() : PyJointPtr(new PyJoint(pjoint,GetEnv()));
    }

    object GetTransform() const { return ReturnTransform(_pbody->GetTransform()); }
    
    object GetBodyTransformations() const
    {
        boost::python::list transforms;
        FOREACHC(itlink, _pbody->GetLinks())
            transforms.append(ReturnTransform((*itlink)->GetTransform()));
        return transforms;
    }

    void SetBodyTransformations(object transforms)
    {
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
        return boost::python::make_tuple(static_cast<numeric::array>(handle<>(pylinear)),static_cast<numeric::array>(handle<>(pyangular)));
    }

    boost::shared_ptr<PyAABB> ComputeAABB() { return boost::shared_ptr<PyAABB>(new PyAABB(_pbody->ComputeAABB())); }
    void Enable(bool bEnable) { _pbody->Enable(bEnable); }
    bool IsEnabled() const { return _pbody->IsEnabled(); }

    void SetTransform(object transform) { _pbody->SetTransform(ExtractTransform(transform)); }
    void SetJointValues(object o)
    {
        if( _pbody->GetDOF() == 0 )
            return;

        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() )
            throw openrave_exception("values do not equal to body degrees of freedom");
        _pbody->SetJointValues(values,true);
    }
    void SetTransformWithJointValues(object otrans,object ojoints)
    {
        if( _pbody->GetDOF() == 0 ) {
            _pbody->SetTransform(ExtractTransform(otrans));
            return;
        }

        vector<dReal> values = ExtractArray<dReal>(ojoints);
        if( (int)values.size() != GetDOF() )
            throw openrave_exception("values do not equal to body degrees of freedom");

        _pbody->SetJointValues(values,ExtractTransform(otrans),true);
    }

    void SetJointValues(object o,object indices)
    {
        if( _pbody->GetDOF() == 0 || len(indices) == 0 )
            return;

        vector<dReal> vsetvalues = ExtractArray<dReal>(o);
        vector<int> vindices = ExtractArray<int>(indices);
        if( vsetvalues.size() != vindices.size() )
            throw openrave_exception("sizes do not match");

        vector<dReal> values;
        _pbody->GetJointValues(values);
        vector<dReal>::iterator itv = vsetvalues.begin();
        FOREACH(it, vindices) {
            if( *it < 0 || *it >= _pbody->GetDOF() )
                throw openrave_exception(boost::str(boost::format("bad index passed")%(*it)));

            values[*it] = *itv++;
        }

        _pbody->SetJointValues(values,true);
    }

    void SetJointTorques(object otorques, bool bAdd)
    {
        if( _pbody->GetDOF() == 0 )
            return;

        vector<dReal> vtorques = ExtractArray<dReal>(otorques);
        BOOST_ASSERT((int)vtorques.size() != GetDOF() );
        _pbody->SetJointValues(vtorques,bAdd);
    }

    boost::multi_array<dReal,2> CalculateJacobian(int index, object offset)
    {
        boost::multi_array<dReal,2> mjacobian;
        _pbody->CalculateJacobian(index,ExtractVector3(offset),mjacobian);
        return mjacobian;
    }

    boost::multi_array<dReal,2> CalculateRotationJacobian(int index, object q) const
    {
        boost::multi_array<dReal,2> mjacobian;
        _pbody->CalculateRotationJacobian(index,ExtractVector4(q),mjacobian);
        return mjacobian;
    }

    boost::multi_array<dReal,2> CalculateAngularVelocityJacobian(int index) const
    {
        boost::multi_array<dReal,2> mjacobian;
        _pbody->CalculateAngularVelocityJacobian(index,mjacobian);
        return mjacobian;
    }

    bool CheckSelfCollision() { return _pbody->CheckSelfCollision(); }
    bool CheckSelfCollision(PyCollisionReportPtr pReport);

    bool IsAttached(PyKinBodyPtr pattachbody) {
        CHECK_POINTER(pattachbody);
        return _pbody->IsAttached(pattachbody->GetBody());
    }
    object GetAttached() const {
        boost::python::list attached;
        std::set<KinBodyPtr> vattached;
        _pbody->GetAttached(vattached);
        FOREACHC(it,vattached)
            attached.append(PyKinBodyPtr(new PyKinBody(*it,_pyenv)));
        return attached;
    }

    bool IsRobot() const { return _pbody->IsRobot(); }
    int GetNetworkId() const { return _pbody->GetNetworkId(); }

    int DoesAffect(int jointindex, int linkindex ) const { return _pbody->DoesAffect(jointindex,linkindex); }

    std::string GetForwardKinematics() const {
        stringstream ss;
        _pbody->WriteForwardKinematics(ss);
        return ss.str();
    }

    void SetGuiData(PyVoidHandle pdata) { _pbody->SetGuiData(pdata._handle); }
    PyVoidHandle GetGuiData() const { return PyVoidHandle(_pbody->GetGuiData()); }

    std::string GetXMLFilename() const { return _pbody->GetXMLFilename(); }

    object GetNonAdjacentLinks() const {
        boost::python::list nonadjacent;
        FOREACHC(it,_pbody->GetNonAdjacentLinks())
            nonadjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        return nonadjacent;
    }
    object GetAdjacentLinks() const {
        boost::python::list adjacent;
        FOREACHC(it,_pbody->GetAdjacentLinks())
            adjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        return adjacent;
    }
    
    PyVoidHandle GetPhysicsData() const { return PyVoidHandle(_pbody->GetPhysicsData()); }
    PyVoidHandle GetCollisionData() const { return PyVoidHandle(_pbody->GetCollisionData()); }
    int GetUpdateStamp() const { return _pbody->GetUpdateStamp(); }

    string serialize(int options) const {
        stringstream ss;
        _pbody->serialize(ss,options);
        return ss.str();
    }

    string GetKinematicsGeometryHash() const { return _pbody->GetKinematicsGeometryHash(); }
    PyVoidHandle CreateKinBodyStateSaver() { return PyVoidHandle(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody))); }

    virtual void __enter__();
    virtual void __exit__(object type, object value, object traceback);
};

class PyCollisionReport
{
public:
    PyCollisionReport() : report(new COLLISIONREPORT()) {}
    PyCollisionReport(CollisionReportPtr report) : report(report) {}
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

    void init(PyEnvironmentBasePtr pyenv)
    {
        options = report->options;
        numCols = report->numCols;
        minDistance = report->minDistance;
        numWithinTol = report->numWithinTol;
        if( !!report->plink1 )
            plink1.reset(new PyKinBody::PyLink(boost::const_pointer_cast<KinBody::Link>(report->plink1), pyenv));
        else
            plink1.reset();
        if( !!report->plink2 )
            plink2.reset(new PyKinBody::PyLink(boost::const_pointer_cast<KinBody::Link>(report->plink2), pyenv));
        else
            plink2.reset();
        boost::python::list newcontacts;
        FOREACH(itc, report->contacts)
            newcontacts.append(PYCONTACT(*itc));
        contacts = newcontacts;
    }

    int options;
    boost::shared_ptr<PyKinBody::PyLink> plink1, plink2;
    int numCols;
    //std::vector<KinBody::Link*> vLinkColliding;
    dReal minDistance;
    int numWithinTol;
    boost::python::list contacts;

    CollisionReportPtr report;
};

bool PyKinBody::CheckSelfCollision(PyCollisionReportPtr pReport)
{
    if( !pReport )
        return _pbody->CheckSelfCollision();

    bool bSuccess = _pbody->CheckSelfCollision(pReport->report);
    pReport->init(GetEnv());
    return bSuccess;
}

class PyControllerBase : public PyInterfaceBase
{
protected:
    ControllerBasePtr _pcontroller;
public:
    PyControllerBase(ControllerBasePtr pcontroller, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pcontroller, pyenv), _pcontroller(pcontroller) {}
    virtual ~PyControllerBase() {}

    ControllerBasePtr GetController() { return _pcontroller; }

    bool Init(PyRobotBasePtr robot, const string& args);
    void Reset(int options) { _pcontroller->Reset(options); }

    bool SetDesired(object o)
    {
        vector<dReal> values = ExtractArray<dReal>(o);
        if( values.size() == 0 )
            throw openrave_exception("no values specified");
        return _pcontroller->SetDesired(values);
    }
    
    bool SetPath(PyTrajectoryBasePtr ptraj);
    bool SimulationStep(dReal fTimeElapsed) { return _pcontroller->SimulationStep(fTimeElapsed); }

    bool IsDone() { return _pcontroller->IsDone(); }
    dReal GetTime() { return _pcontroller->GetTime(); }

    object GetVelocity()
    {
        vector<dReal> velocity;
        _pcontroller->GetVelocity(velocity);
        return toPyArray(velocity);
    }

    object GetTorque()
    {
        vector<dReal> torque;
        _pcontroller->GetTorque(torque);
        return toPyArray(torque);
    }
};

class PySensorBase : public PyInterfaceBase
{
protected:
    SensorBasePtr _psensor;
    SensorBase::SensorDataPtr _psensordata;
public:
    class PySensorData
    {
    public:
        PySensorData(SensorBase::SensorDataPtr pdata)
        {
            type = pdata->GetType();
        }
        virtual ~PySensorData() {}
        
        SensorBase::SensorType type;
    };

    class PyLaserSensorData : public PySensorData
    {
    public:
        PyLaserSensorData(boost::shared_ptr<SensorBase::LaserGeomData> pgeom, boost::shared_ptr<SensorBase::LaserSensorData> pdata) : PySensorData(pdata)
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
        PyCameraSensorData(boost::shared_ptr<SensorBase::CameraGeomData> pgeom, boost::shared_ptr<SensorBase::CameraSensorData> pdata) : PySensorData(pdata)
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
                numeric::array arr(boost::python::make_tuple(pgeom->KK.fx,0,pgeom->KK.cx,0,pgeom->KK.fy,pgeom->KK.cy,0,0,1));
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

    PySensorBase(SensorBasePtr psensor, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensor, pyenv), _psensor(psensor)
    {
        _psensordata = _psensor->CreateSensorData();
    }
    virtual ~PySensorBase() { }

    SensorBasePtr GetSensor() { return _psensor; }
    boost::shared_ptr<PySensorData> GetSensorData()
    {
        if( !_psensor->GetSensorData(_psensordata) )
            throw openrave_exception("SensorData failed");
        switch(_psensordata->GetType()) {
        case SensorBase::ST_Laser:
            return boost::shared_ptr<PySensorData>(new PyLaserSensorData(boost::static_pointer_cast<SensorBase::LaserGeomData>(_psensor->GetSensorGeometry()),
                                                                         boost::static_pointer_cast<SensorBase::LaserSensorData>(_psensordata)));
            
        case SensorBase::ST_Camera:
            return boost::shared_ptr<PySensorData>(new PyCameraSensorData(boost::static_pointer_cast<SensorBase::CameraGeomData>(_psensor->GetSensorGeometry()),
                                                                          boost::static_pointer_cast<SensorBase::CameraSensorData>(_psensordata)));
        default: {
            stringstream ss;
            ss << "unknown sensor data type: " << _psensordata->GetType() << endl;
            throw openrave_exception(ss.str());
        }
        }
    }
    
    void SetTransform(object transform) { _psensor->SetTransform(ExtractTransform(transform)); }
    object GetTransform() { return ReturnTransform(_psensor->GetTransform()); }

    string GetName() { return _psensor->GetName(); }
};

class PyIkSolverBase : public PyInterfaceBase
{
protected:
    IkSolverBasePtr _pIkSolver;
public:
    PyIkSolverBase(IkSolverBasePtr pIkSolver, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pIkSolver, pyenv), _pIkSolver(pIkSolver) {}
    virtual ~PyIkSolverBase() {}

    IkSolverBasePtr GetIkSolver() { return _pIkSolver; }
};

class PyRobotBase : public PyKinBody
{
protected:
    RobotBasePtr _probot;
public:
    RobotBasePtr GetRobot() { return _probot; }

    class PyManipulator
    {
        RobotBase::ManipulatorPtr _pmanip;
        PyEnvironmentBasePtr _pyenv;
    public:
        PyManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv) : _pmanip(pmanip),_pyenv(pyenv) {}
        virtual ~PyManipulator() {}

        object GetEndEffectorTransform() const { return ReturnTransform(_pmanip->GetEndEffectorTransform()); }

        string GetName() const { return _pmanip->GetName(); }
        PyRobotBasePtr GetRobot() { return PyRobotBasePtr(new PyRobotBase(_pmanip->GetRobot(),_pyenv)); }

		void SetIKSolver(PyIkSolverBasePtr iksolver) { CHECK_POINTER(iksolver); _pmanip->SetIKSolver(iksolver->GetIkSolver()); }
        bool InitIKSolver() { return _pmanip->InitIKSolver(); }
        string GetIKSolverName() const { return _pmanip->GetIKSolverName(); }
        bool HasIKSolver() const { return _pmanip->HasIKSolver(); }

        boost::shared_ptr<PyLink> GetBase() { return !_pmanip->GetBase() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetBase(),_pyenv)); }
        boost::shared_ptr<PyLink> GetEndEffector() { return !_pmanip->GetEndEffector() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetEndEffector(),_pyenv)); }
        object GetGraspTransform() { return ReturnTransform(_pmanip->GetGraspTransform()); }
        object GetGripperJoints() { return toPyArray(_pmanip->GetGripperJoints()); }
        object GetArmJoints() { return toPyArray(_pmanip->GetArmJoints()); }
        object GetClosingDirection() { return toPyArray(_pmanip->GetClosingDirection()); }
        object GetPalmDirection() { return toPyVector3(_pmanip->GetPalmDirection()); }
        bool IsGrabbing(PyKinBodyPtr pbody) { return _pmanip->IsGrabbing(pbody->GetBody()); }

        int GetNumFreeParameters() const { return _pmanip->GetNumFreeParameters(); }

        object GetFreeParameters() const {
            if( _pmanip->GetNumFreeParameters() == 0 )
                return object();
            vector<dReal> values;
            _pmanip->GetFreeParameters(values);
            return toPyArray(values);
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
            vector<dReal> solution, vfreeparams = ExtractArray<dReal>(freeparams);
            if( !_pmanip->FindIKSolution(ExtractTransform(transform),vfreeparams, solution,bColCheck) )
                return object();
            return toPyArray(solution);
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
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            if( !_pmanip->FindIKSolutions(ExtractTransform(transform),vfreeparams, vsolutions,bColCheck) )
                return object();
            boost::python::list solutions;
            FOREACH(itsol,vsolutions)
                solutions.append(toPyArray(*itsol));
            return solutions;
        }
        
        object GetChildJoints() {
            std::vector<KinBody::JointPtr> vjoints;
            _pmanip->GetChildJoints(vjoints);
            boost::python::list joints;
            FOREACH(itjoint,vjoints)
                joints.append(PyJointPtr(new PyJoint(*itjoint, _pyenv)));
            return joints;
        }
        object GetChildDOFIndices() {
            std::vector<int> vdofindices;
            _pmanip->GetChildDOFIndices(vdofindices);
            boost::python::list dofindices;
            FOREACH(itindex,vdofindices)
                dofindices.append(*itindex);
            return dofindices;
        }

        object GetChildLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetChildLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks)
                links.append(PyLinkPtr(new PyLink(*itlink,_pyenv)));
            return links;
        }

        object GetIndependentLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetIndependentLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks)
                links.append(PyLinkPtr(new PyLink(*itlink,_pyenv)));
            return links;
        }

        bool CheckEndEffectorCollision(object otrans) const
        {
            return _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans));
        }
        bool CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pReport) const
        {
            return _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans),!pReport ? CollisionReportPtr() : pReport->report);
        }
        bool CheckIndependentCollision() const
        {
            return _pmanip->CheckIndependentCollision();
        }
        bool CheckIndependentCollision(PyCollisionReportPtr pReport) const
        {
            return _pmanip->CheckIndependentCollision(!pReport ? CollisionReportPtr() : pReport->report);
        }
    };

    class PyAttachedSensor
    {
        RobotBase::AttachedSensorPtr _pattached;
        PyEnvironmentBasePtr _pyenv;
    public:
        PyAttachedSensor(RobotBase::AttachedSensorPtr pattached, PyEnvironmentBasePtr pyenv) : _pattached(pattached),_pyenv(pyenv) {}
        virtual ~PyAttachedSensor() {}
        
        PySensorBasePtr GetSensor() { return !_pattached->GetSensor() ? PySensorBasePtr() : PySensorBasePtr(new PySensorBase(_pattached->GetSensor(),_pyenv)); }
        PyLinkPtr GetAttachingLink() const { return !_pattached->GetAttachingLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pattached->GetAttachingLink(), _pyenv)); }
        object GetRelativeTransform() const { return ReturnTransform(_pattached->GetRelativeTransform()); }
        object GetTransform() const { return ReturnTransform(_pattached->GetTransform()); }
        PyRobotBasePtr GetRobot() const { return _pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv)); }
        string GetName() const { return _pattached->GetName(); }

        void SetRelativeTransform(object transform) { _pattached->SetRelativeTransform(ExtractTransform(transform)); }
    };

    class PyGrabbed
    {
    public:
        PyGrabbed(const RobotBase::GRABBED& grabbed, PyEnvironmentBasePtr pyenv) {
            grabbedbody.reset(new PyKinBody(KinBodyPtr(grabbed.pbody),pyenv));
            linkrobot.reset(new PyLink(KinBody::LinkPtr(grabbed.plinkrobot),pyenv));

            FOREACHC(it, grabbed.vCollidingLinks)
                validColLinks.append(PyLinkPtr(new PyLink(boost::const_pointer_cast<KinBody::Link>(*it),pyenv)));
            
            troot = ReturnTransform(grabbed.troot);
        }
        virtual ~PyGrabbed() {}

        PyKinBodyPtr grabbedbody;
        PyLinkPtr linkrobot;
        boost::python::list validColLinks;
        object troot;
    };

    PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : PyKinBody(probot,pyenv), _probot(probot) {}
    PyRobotBase(const PyRobotBase& r) : PyKinBody(r._probot,r._pyenv) { _probot = r._probot; }
    virtual ~PyRobotBase() {}

    object GetManipulators()
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators())
            manips.append(boost::shared_ptr<PyManipulator>(new PyManipulator(*it,_pyenv)));
        return manips;
    }

    void SetActiveManipulator(int index) { _probot->SetActiveManipulator(index); }
    boost::shared_ptr<PyManipulator> GetActiveManipulator() { return boost::shared_ptr<PyManipulator>(new PyManipulator(_probot->GetActiveManipulator(),_pyenv)); }
    int GetActiveManipulatorIndex() const { return _probot->GetActiveManipulatorIndex(); }

    object GetSensors()
    {
        boost::python::list sensors;
        FOREACH(itsensor, _probot->GetSensors())
            sensors.append(boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
        return sensors;
    }
    
    PyControllerBasePtr GetController() const { return !_probot->GetController() ? PyControllerBasePtr() : PyControllerBasePtr(new PyControllerBase(_probot->GetController(),_pyenv)); }
    bool SetController(PyControllerBasePtr pController, const string& args=string("")) { CHECK_POINTER(pController); return _probot->SetController(pController->GetController(),args.c_str()); }
    
    void SetActiveDOFs(object jointindices) { _probot->SetActiveDOFs(ExtractArray<int>(jointindices)); }
    void SetActiveDOFs(object jointindices, int nAffineDOsBitmask) { _probot->SetActiveDOFs(ExtractArray<int>(jointindices), nAffineDOsBitmask); }
    void SetActiveDOFs(object jointindices, int nAffineDOsBitmask, object rotationaxis) {
        _probot->SetActiveDOFs(ExtractArray<int>(jointindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
    }

    int GetActiveDOF() const { return _probot->GetActiveDOF(); }
    int GetAffineDOF() const { return _probot->GetAffineDOF(); }
    int GetAffineDOFIndex(RobotBase::DOFAffine dof) const { return _probot->GetAffineDOFIndex(dof); }

    object GetAffineRotationAxis() const { return toPyVector3(_probot->GetAffineRotationAxis()); }
    void SetAffineTranslationLimits(object lower, object upper) { return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotationAxisLimits(object lower, object upper) { return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotation3DLimits(object lower, object upper) { return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper)); }
    void SetAffineRotationQuatLimits(object lower, object upper) { return _probot->SetAffineRotationQuatLimits(ExtractVector4(lower),ExtractVector4(upper)); }
    void SetAffineTranslationMaxVels(object vels) { _probot->SetAffineTranslationMaxVels(ExtractVector3(vels)); }
    void SetAffineRotationAxisMaxVels(object vels) { _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels)); }
    void SetAffineRotation3DMaxVels(object vels) { _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels)); }
    void SetAffineRotationQuatMaxVels(object vels) { _probot->SetAffineRotationQuatMaxVels(ExtractVector4(vels)); }
    void SetAffineTranslationResolution(object resolution) { _probot->SetAffineTranslationResolution(ExtractVector4(resolution)); }
    void SetAffineRotationAxisResolution(object resolution) { _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution)); }
    void SetAffineRotation3DResolution(object resolution) { _probot->SetAffineRotation3DResolution(ExtractVector3(resolution)); }
    void SetAffineRotationQuatResolution(object resolution) { _probot->SetAffineRotationQuatResolution(ExtractVector4(resolution)); }
    void SetAffineTranslationWeights(object weights) { _probot->SetAffineTranslationWeights(ExtractVector3(weights)); }
    void SetAffineRotationAxisWeights(object weights) { _probot->SetAffineRotationAxisWeights(ExtractVector4(weights)); }
    void SetAffineRotation3DWeights(object weights) { _probot->SetAffineRotation3DWeights(ExtractVector3(weights)); }
    void SetAffineRotationQuatWeights(object weights) { _probot->SetAffineRotationQuatWeights(ExtractVector4(weights)); }

    object GetAffineTranslationLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineTranslationLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationAxisLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotationAxisLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotation3DLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotation3DLimits(lower,upper);
        return boost::python::make_tuple(toPyVector3(lower),toPyVector3(upper));
    }
    object GetAffineRotationQuatLimits() const
    {
        Vector lower, upper;
        _probot->GetAffineRotationQuatLimits(lower,upper);
        return boost::python::make_tuple(toPyVector4(lower),toPyVector4(upper));
    }
    object GetAffineTranslationMaxVels() const { return toPyVector3(_probot->GetAffineTranslationMaxVels()); }
    object GetAffineRotationAxisMaxVels() const { return toPyVector3(_probot->GetAffineRotationAxisMaxVels()); }
    object GetAffineRotation3DMaxVels() const { return toPyVector3(_probot->GetAffineRotation3DMaxVels()); }
    object GetAffineRotationQuatMaxVels() const { return toPyVector4(_probot->GetAffineRotationQuatMaxVels()); }
    object GetAffineTranslationResolution() const { return toPyVector3(_probot->GetAffineTranslationResolution()); }
    object GetAffineRotationAxisResolution() const { return toPyVector4(_probot->GetAffineRotationAxisResolution()); }
    object GetAffineRotation3DResolution() const { return toPyVector3(_probot->GetAffineRotation3DResolution()); }
    object GetAffineRotationQuatResolution() const { return toPyVector4(_probot->GetAffineRotationQuatResolution()); }
    object GetAffineTranslationWeights() const { return toPyVector3(_probot->GetAffineTranslationWeights()); }
    object GetAffineRotationAxisWeights() const { return toPyVector4(_probot->GetAffineRotationAxisWeights()); }
    object GetAffineRotation3DWeights() const { return toPyVector3(_probot->GetAffineRotation3DWeights()); }
    object GetAffineRotationQuatWeights() const { return toPyVector4(_probot->GetAffineRotationQuatWeights()); }

    void SetActiveDOFValues(object values) const
    {
        vector<dReal> vvalues = ExtractArray<dReal>(values);
        if( vvalues.size() > 0 )
            _probot->SetActiveDOFValues(vvalues,true);
    }
    object GetActiveDOFValues() const
    {
        if( _probot->GetActiveDOF() == 0 )
            return object();
        vector<dReal> values;
        _probot->GetActiveDOFValues(values);
        return toPyArray(values);
    }

    void SetActiveDOFVelocities(object velocities)
    {
        _probot->SetActiveDOFVelocities(ExtractArray<dReal>(velocities));
    }
    object GetActiveDOFVelocities() const
    {
        if( _probot->GetActiveDOF() == 0 )
            return object();
        vector<dReal> values;
        _probot->GetActiveDOFVelocities(values);
        return toPyArray(values);
    }

    object GetActiveDOFLimits() const
    {
        if( _probot->GetActiveDOF() == 0 )
            return object();
        vector<dReal> lower, upper;
        _probot->GetActiveDOFLimits(lower,upper);
        return boost::python::make_tuple(toPyArray(lower),toPyArray(upper));
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

    int GetActiveJointIndex(int active_index) const { return _probot->GetActiveJointIndex(active_index); }
    object GetActiveJointIndices() { return toPyArray(_probot->GetActiveJointIndices()); }

    boost::multi_array<dReal,2> CalculateActiveJacobian(int index, object offset) const
    {
        boost::multi_array<dReal,2> mjacobian;
        _probot->CalculateActiveJacobian(index,ExtractVector3(offset),mjacobian);
        return mjacobian;
    }

    boost::multi_array<dReal,2> CalculateActiveRotationJacobian(int index, object q) const
    {
        boost::multi_array<dReal,2> mjacobian;
        _probot->CalculateActiveJacobian(index,ExtractVector4(q),mjacobian);
        return mjacobian;
    }

    boost::multi_array<dReal,2> CalculateActiveAngularVelocityJacobian(int index) const
    {
        boost::multi_array<dReal,2> mjacobian;
        _probot->CalculateActiveAngularVelocityJacobian(index,mjacobian);
        return mjacobian;
    }

    bool Grab(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody()); }
    bool Grab(PyKinBodyPtr pbody, object linkstoignore)
    {
        CHECK_POINTER(pbody);
        std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
        return _probot->Grab(pbody->GetBody(), setlinkstoignore);
    }
    bool Grab(PyKinBodyPtr pbody, PyKinBody::PyLinkPtr plink)
    {
        CHECK_POINTER(pbody);
        CHECK_POINTER(plink);
        return _probot->Grab(pbody->GetBody(), plink->GetLink());
    }
    bool Grab(PyKinBodyPtr pbody, PyKinBody::PyLinkPtr plink, object linkstoignore)
    {
        CHECK_POINTER(pbody);
        CHECK_POINTER(plink);
        std::set<int> setlinkstoignore = ExtractSet<int>(linkstoignore);
        return _probot->Grab(pbody->GetBody(), plink->GetLink(), setlinkstoignore);
    }
    void Release(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); _probot->Release(pbody->GetBody()); }
    void ReleaseAllGrabbed() { _probot->ReleaseAllGrabbed(); }
    void RegrabAll() { _probot->RegrabAll(); }
    PyLinkPtr IsGrabbing(PyKinBodyPtr pbody) const {
        CHECK_POINTER(pbody);
        KinBody::LinkPtr plink = _probot->IsGrabbing(pbody->GetBody());
        if( !plink )
            return PyLinkPtr();
        else
            return PyLinkPtr(new PyLink(plink,_pyenv));
    }

    object GetGrabbed() const
    {
        boost::python::list bodies;
        std::vector<KinBodyPtr> vbodies;
        _probot->GetGrabbed(vbodies);
        FOREACH(itbody, vbodies)
            bodies.append(PyKinBodyPtr(new PyKinBody(*itbody,_pyenv)));
        return bodies;
    }

    bool WaitForController(float ftimeout)
    {
        ControllerBasePtr pcontroller = _probot->GetController();
        if( !pcontroller )
            return false;
        if( pcontroller->IsDone() )
            return true;

        bool bSuccess = true;
        Py_BEGIN_ALLOW_THREADS

        try {
            uint64_t starttime = GetMicroTime();
            uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
            while( !pcontroller->IsDone() ) {
                Sleep(1);            
                if( deltatime > 0 && (GetMicroTime()-starttime)>deltatime  ) {
                    bSuccess = false;
                    break;
                }
            }
        }
        catch(...) {
            RAVELOG_ERROR("exception raised inside WaitForController\n");
            bSuccess = false;
        }
        
        Py_END_ALLOW_THREADS;


        return bSuccess;
    }

    string GetRobotStructureHash() const { return _probot->GetRobotStructureHash(); }
    PyVoidHandle CreateRobotStateSaver() { return PyVoidHandle(boost::shared_ptr<void>(new RobotBase::RobotStateSaver(_probot))); }

    virtual void __enter__();
};

bool PyControllerBase::Init(PyRobotBasePtr robot, const string& args)
{
    CHECK_POINTER(robot);
    return _pcontroller->Init(robot->GetRobot(), args);
}

class PyPlannerBase : public PyInterfaceBase
{
protected:
    PlannerBasePtr _pplanner;
public:
    PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pplanner, pyenv), _pplanner(pplanner) {}
    virtual ~PyPlannerBase() {}
};

class PySensorSystemBase : public PyInterfaceBase
{
    friend class PyEnvironmentBase;
private:
    SensorSystemBasePtr _psensorsystem;
public:
    PySensorSystemBase(SensorSystemBasePtr psensorsystem, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(psensorsystem, pyenv), _psensorsystem(psensorsystem) {}
    virtual ~PySensorSystemBase() {}
};

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pTrajectory, pyenv),_ptrajectory(pTrajectory) {}
    virtual ~PyTrajectoryBase() {}

    TrajectoryBasePtr GetTrajectory() { return _ptrajectory; }
};

bool PyControllerBase::SetPath(PyTrajectoryBasePtr ptraj)
{
    CHECK_POINTER(ptraj);
    return _pcontroller->SetPath(!ptraj ? TrajectoryBasePtr() : ptraj->GetTrajectory());
}

class PyProblemInstance : public PyInterfaceBase
{
protected:
    ProblemInstancePtr _pproblem;
public:
    PyProblemInstance(ProblemInstancePtr pproblem, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pproblem, pyenv), _pproblem(pproblem) {}
    virtual ~PyProblemInstance() {}
    ProblemInstancePtr GetProblem() { return _pproblem; }

    bool SimulationStep(dReal fElapsedTime) { return _pproblem->SimulationStep(fElapsedTime); }
};

class PyPhysicsEngineBase : public PyInterfaceBase
{
protected:
    PhysicsEngineBasePtr _pPhysicsEngine;
public:
    PyPhysicsEngineBase(PhysicsEngineBasePtr pPhysicsEngine, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pPhysicsEngine, pyenv),_pPhysicsEngine(pPhysicsEngine) {}
    virtual ~PyPhysicsEngineBase() {}

    PhysicsEngineBasePtr GetPhysicsEngine() { return _pPhysicsEngine; }

    bool SetPhysicsOptions(int physicsoptions) { return _pPhysicsEngine->SetPhysicsOptions(physicsoptions); }
    int GetPhysicsOptions() const { return _pPhysicsEngine->GetPhysicsOptions(); }

    object SetPhysicsOptions(const string& s) {
        stringstream sinput(s), sout;
        if( !_pPhysicsEngine->SetPhysicsOptions(sout,sinput) )
            return object();
        return object(sout.str());
    }
    bool InitEnvironment() { return _pPhysicsEngine->InitEnvironment(); }
    void DestroyEnvironment() { _pPhysicsEngine->DestroyEnvironment(); }
    bool InitKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _pPhysicsEngine->InitKinBody(pbody->GetBody()); }

    bool SetBodyVelocity(PyKinBodyPtr pbody, object linearvel, object angularvel, object jointvelocity)
    {
        CHECK_POINTER(pbody);
        if( !jointvelocity )
            return _pPhysicsEngine->SetBodyVelocity(pbody->GetBody(),ExtractVector3(linearvel),ExtractVector3(angularvel));
        return _pPhysicsEngine->SetBodyVelocity(pbody->GetBody(),ExtractVector3(linearvel),ExtractVector3(angularvel),ExtractArray<dReal>(jointvelocity));
    }
    bool SetBodyVelocity(PyKinBodyPtr pbody, object LinearVelocities, object AngularVelocities)
    {
        CHECK_POINTER(pbody);
        vector<dReal> vLinearVelocities = ExtractArray<dReal>(LinearVelocities);
        vector<dReal> vAngularVelocities = ExtractArray<dReal>(AngularVelocities);
        vector<Vector> linearvel(vLinearVelocities.size()/3);
        for(size_t i = 0; i < vLinearVelocities.size()/3; ++i)
            linearvel[i] = Vector(vLinearVelocities[3*i],vLinearVelocities[3*i+1],vLinearVelocities[3*i+2]);
        vector<Vector> angularvel(vAngularVelocities.size()/3);
        for(size_t i = 0; i < vAngularVelocities.size()/3; ++i)
            angularvel[i] = Vector(vAngularVelocities[3*i],vAngularVelocities[3*i+1],vAngularVelocities[3*i+2]);
        return _pPhysicsEngine->SetBodyVelocity(pbody->GetBody(),linearvel,angularvel);
    }

    object GetBodyVelocityJoints(PyKinBodyPtr pbody)
    {
        CHECK_POINTER(pbody);
        Vector linearvel, angularvel;
        vector<dReal> vjointvel;
        if( !_pPhysicsEngine->GetBodyVelocity(pbody->GetBody(),linearvel,angularvel,vjointvel) ) {
            return boost::python::make_tuple(object(),object(),object());
        }

        return boost::python::make_tuple(toPyVector3(linearvel),toPyVector3(angularvel),toPyArray(vjointvel));
    }

    object GetBodyVelocityLinks(PyKinBodyPtr pbody, Vector* pLinearVelocities, Vector* pAngularVelocities)
    {
        CHECK_POINTER(pbody);
        if( pbody->GetBody()->GetDOF() == 0 )
            return boost::python::make_tuple(object(),object());
        vector<Vector> linearvel(pbody->GetBody()->GetDOF()),angularvel(pbody->GetBody()->GetDOF());
        if( !_pPhysicsEngine->GetBodyVelocity(pbody->GetBody(),linearvel,angularvel) )
            return boost::python::make_tuple(object(),object());

        npy_intp dims[] = {pbody->GetBody()->GetDOF(),3};
        PyObject *pylinear = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        PyObject *pyangular = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        dReal* pflinear = (dReal*)PyArray_DATA(pylinear);
        dReal* pfangular = (dReal*)PyArray_DATA(pyangular);
        for(size_t i = 0; i < linearvel.size(); ++i) {
            pflinear[3*i+0] = linearvel[i].x; pflinear[3*i+1] = linearvel[i].y; pflinear[3*i+2] = linearvel[i].z;
            pfangular[3*i+0] = angularvel[i].x; pfangular[3*i+1] = angularvel[i].y; pfangular[3*i+2] = angularvel[i].z;
        }
        return boost::python::make_tuple(static_cast<numeric::array>(handle<>(pylinear)),static_cast<numeric::array>(handle<>(pyangular)));
    }

    bool SetJointVelocity(PyKinBody::PyJointPtr pjoint, object jointvelocity)
    {
        CHECK_POINTER(pjoint);
        return _pPhysicsEngine->SetJointVelocity(pjoint->GetJoint(),ExtractArray<dReal>(jointvelocity));
    }

    object GetJointVelocity(PyKinBody::PyJointPtr pjoint)
    {
        CHECK_POINTER(pjoint);
        vector<dReal> vel;
        if( !_pPhysicsEngine->GetJointVelocity(pjoint->GetJoint(),vel) )
            return object();
        return toPyArray(vel);
    }

    bool SetBodyForce(PyKinBody::PyLinkPtr plink, object force, object position, bool bAdd)
    {
        CHECK_POINTER(plink);
        return _pPhysicsEngine->SetBodyForce(plink->GetLink(),ExtractVector3(force),ExtractVector3(position),bAdd);
    }

    bool SetBodyTorque(PyKinBody::PyLinkPtr plink, object torque, bool bAdd)
    {
        CHECK_POINTER(plink);
        return _pPhysicsEngine->SetBodyTorque(plink->GetLink(),ExtractVector3(torque),bAdd);
    }

    bool AddJointTorque(PyKinBody::PyJointPtr pjoint, object torques)
    {
        CHECK_POINTER(pjoint);
        return _pPhysicsEngine->AddJointTorque(pjoint->GetJoint(),ExtractArray<dReal>(torques));
    }

    void SetGravity(object gravity) { _pPhysicsEngine->SetGravity(ExtractVector3(gravity)); }
    object GetGravity() { return toPyVector3(_pPhysicsEngine->GetGravity()); }

    void SimulateStep(dReal fTimeElapsed) { _pPhysicsEngine->SimulateStep(fTimeElapsed); }
};

class PyCollisionCheckerBase : public PyInterfaceBase
{
protected:
    CollisionCheckerBasePtr _pCollisionChecker;
public:
    PyCollisionCheckerBase(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pCollisionChecker, pyenv), _pCollisionChecker(pCollisionChecker) {}
    virtual ~PyCollisionCheckerBase() {}

    CollisionCheckerBasePtr GetCollisionChecker() { return _pCollisionChecker; }

    bool SetCollisionOptions(int options) { return _pCollisionChecker->SetCollisionOptions(options); }
    int GetCollisionOptions() const { return _pCollisionChecker->GetCollisionOptions(); }
};

class PyRaveViewerBase : public PyInterfaceBase
{
protected:
    RaveViewerBasePtr _pviewer;

    static bool _ViewerCallback(object fncallback, PyEnvironmentBasePtr pyenv, KinBody::LinkPtr plink,RaveVector<float> position,RaveVector<float> direction)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            res = fncallback(PyKinBody::PyLinkPtr(new PyKinBody::PyLink(plink,pyenv)),toPyVector3(position),toPyVector3(direction));
        }
        catch(...) {
            RAVELOG_ERRORA("exception occured in python viewer callback\n");
        }
        PyGILState_Release(gstate);
        extract<bool> xb(res);
        if( xb.check() )
            return (bool)xb;
        extract<int> xi(res);
        if( xi.check() )
            return (int)xi;
        extract<double> xd(res);
        if( xd.check() )
            return (double)xd>0;
        return true;
    }
public:

    PyRaveViewerBase(RaveViewerBasePtr pviewer, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pviewer, pyenv), _pviewer(pviewer) {}
    virtual ~PyRaveViewerBase() {}

    RaveViewerBasePtr GetViewer() { return _pviewer; }

    int main(bool bShow) { return _pviewer->main(bShow); }
    void quitmainloop() { return _pviewer->quitmainloop(); }

    void ViewerSetSize(int w, int h) { _pviewer->ViewerSetSize(w,h); }
    void ViewerMove(int x, int y) { _pviewer->ViewerMove(x,y); }
    void ViewerSetTitle(const string& title) { _pviewer->ViewerSetTitle(title.c_str()); }
    bool LoadModel(const string& filename) { return _pviewer->LoadModel(filename.c_str()); }

    PyVoidHandle RegisterCallback(RaveViewerBase::ViewerEvents properties, object fncallback)
    {
        if( !fncallback )
            throw openrave_exception("callback not specified");
        boost::shared_ptr<void> p = _pviewer->RegisterCallback(properties,boost::bind(&PyRaveViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p )
            throw openrave_exception("no registration callback returned");
        return PyVoidHandle(p);
    }

    void EnvironmentSync() { return _pviewer->EnvironmentSync(); }
};

class PyEnvironmentBase : public boost::enable_shared_from_this<PyEnvironmentBase>
{
#if BOOST_VERSION < 103500
    boost::mutex _envmutex;
    std::list<boost::shared_ptr<EnvironmentMutex::scoped_lock> > _listenvlocks, _listfreelocks;
#endif
protected:
    EnvironmentBasePtr _penv;
    boost::shared_ptr<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;

    void _ViewerThread(const string& strviewer, bool bShowViewer)
    {
        RaveViewerBasePtr pviewer;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            pviewer = _penv->CreateViewer(strviewer.c_str());
            if( !!pviewer ) {
                _penv->AttachViewer(pviewer);
            }
            _conditionViewer.notify_all();
        }

        if( !pviewer )
            return;

        pviewer->main(bShowViewer); // spin until quitfrommainloop is called
        _penv->AttachViewer(RaveViewerBasePtr());
        pviewer.reset();
    }

    CollisionAction _CollisionCallback(object fncallback, CollisionReportPtr preport, bool bFromPhysics)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            PyCollisionReportPtr pyreport;
            if( !!preport ) {
                pyreport.reset(new PyCollisionReport(preport));
                pyreport->init(shared_from_this());
            }
            res = fncallback(pyreport,bFromPhysics);
        }
        catch(...) {
            RAVELOG_ERRORA("exception occured in python collision callback\n");
        }
        PyGILState_Release(gstate);
        if( res == object() || !res )
            return CA_DefaultAction;
        extract<int> xi(res);
        if( xi.check() )
            return (CollisionAction)(int)xi;
        RAVELOG_WARNA("collision callback nothing returning, so executing default action\n");
        return CA_DefaultAction;
    }

public:
    PyEnvironmentBase()
    {
        _penv = CreateEnvironment(true);
    }
    PyEnvironmentBase(EnvironmentBasePtr penv) : _penv(penv) {
    }

    PyEnvironmentBase(const PyEnvironmentBase& pyenv)
    {
        _penv = pyenv._penv;
    }

    virtual ~PyEnvironmentBase()
    {
        {
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            _penv->AttachViewer(RaveViewerBasePtr());
        }

        if( !!_threadviewer )
            _threadviewer->join();
        _threadviewer.reset();
    }

    void Reset() { _penv->Reset(); }
    void Destroy() {
        _penv->Destroy();
        if( !!_threadviewer )
            _threadviewer->join();
    }

    object GetPluginInfo()
    {
        boost::python::list plugins;
        std::list< std::pair<std::string, PLUGININFO> > listplugins;
        _penv->GetPluginInfo(listplugins);
        FOREACH(itplugin, listplugins)
            plugins.append(boost::python::make_tuple(itplugin->first,object(boost::shared_ptr<PyPluginInfo>(new PyPluginInfo(itplugin->second)))));
        return plugins;
    }

    boost::shared_ptr<PyPluginInfo> GetLoadedInterfaces()
    {
        PLUGININFO info;
        _penv->GetLoadedInterfaces(info);
        return boost::shared_ptr<PyPluginInfo>(new PyPluginInfo(info));
    }

    bool LoadPlugin(const string& name) { return _penv->LoadPlugin(name.c_str()); }

    PyInterfaceBasePtr CreateInterface(PluginType type, const string& name)
    {
        InterfaceBasePtr p = _penv->CreateInterface(type,name);
        if( !p )
            return PyInterfaceBasePtr();
        return PyInterfaceBasePtr(new PyInterfaceBase(p,shared_from_this()));
    }
    PyRobotBasePtr CreateRobot(const string& name)
    {
        RobotBasePtr p = _penv->CreateRobot(name);
        if( !p )
            return PyRobotBasePtr();
        return PyRobotBasePtr(new PyRobotBase(p, shared_from_this()));
    }
    PyPlannerBasePtr CreatePlanner(const string& name)
    {
        PlannerBasePtr p = _penv->CreatePlanner(name);
        if( !p )
            return PyPlannerBasePtr();
        return PyPlannerBasePtr(new PyPlannerBase(p, shared_from_this()));
    }
    PySensorSystemBasePtr CreateSensorSystem(const string& name)
    {
        SensorSystemBasePtr p = _penv->CreateSensorSystem(name);
        if( !p )
            return PySensorSystemBasePtr();
        return PySensorSystemBasePtr(new PySensorSystemBase(p, shared_from_this()));
    }
    PyControllerBasePtr CreateController(const string& name)
    {
        ControllerBasePtr p = _penv->CreateController(name);
        if( !p )
            return PyControllerBasePtr();
        return PyControllerBasePtr(new PyControllerBase(p, shared_from_this()));
    }
    PyProblemInstancePtr CreateProblem(const string& name)
    {
        ProblemInstancePtr p = _penv->CreateProblem(name);
        if( !p )
            return PyProblemInstancePtr();
        return PyProblemInstancePtr(new PyProblemInstance(p, shared_from_this()));
    }
    PyIkSolverBasePtr CreateIkSolver(const string& name)
    {
        IkSolverBasePtr p = _penv->CreateIkSolver(name);
        if( !p )
            return PyIkSolverBasePtr();
        return PyIkSolverBasePtr(new PyIkSolverBase(p, shared_from_this()));
    }
    PyPhysicsEngineBasePtr CreatePhysicsEngine(const string& name)
    {
        PhysicsEngineBasePtr p = _penv->CreatePhysicsEngine(name);
        if( !p )
            return PyPhysicsEngineBasePtr();
        return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(p, shared_from_this()));
    }
    PySensorBasePtr CreateSensor(const string& name)
    {
        SensorBasePtr p = _penv->CreateSensor(name);
        if( !p )
            return PySensorBasePtr();
        return PySensorBasePtr(new PySensorBase(p, shared_from_this()));
    }
    PyCollisionCheckerBasePtr CreateCollisionChecker(const string& name)
    {
        CollisionCheckerBasePtr p = _penv->CreateCollisionChecker(name);
        if( !p )
            return PyCollisionCheckerBasePtr();
        return PyCollisionCheckerBasePtr(new PyCollisionCheckerBase(p, shared_from_this()));
    }
    PyRaveViewerBasePtr CreateViewer(const string& name)
    {
        RaveViewerBasePtr p = _penv->CreateViewer(name);
        if( !p )
            return PyRaveViewerBasePtr();
        return PyRaveViewerBasePtr(new PyRaveViewerBase(p, shared_from_this()));
    }

    PyEnvironmentBasePtr CloneSelf(int options)
    {
        //RAVELOG_WARNA("cloning environment without permission!\n");
        string strviewer;
        if( options & Clone_Viewer ) {
            boost::mutex::scoped_lock lockcreate(_mutexViewer);
            if( !!_penv->GetViewer() )
                strviewer = _penv->GetViewer()->GetXMLId();
        }
        PyEnvironmentBasePtr pnewenv(new PyEnvironmentBase(_penv->CloneSelf(options)));
        if( strviewer.size() > 0 )
            pnewenv->SetViewer(strviewer);
        return pnewenv;
    }

    bool SetCollisionChecker(PyCollisionCheckerBasePtr pchecker) {
        if( !pchecker )
            return _penv->SetCollisionChecker(CollisionCheckerBasePtr());
        return _penv->SetCollisionChecker(pchecker->GetCollisionChecker());
    }
    PyCollisionCheckerBasePtr GetCollisionChecker() { return PyCollisionCheckerBasePtr(new PyCollisionCheckerBase(_penv->GetCollisionChecker(), shared_from_this())); }

    bool CheckCollision(PyKinBodyPtr pbody1)
    {
        CHECK_POINTER(pbody1);
        return _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()));
    }
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        if( !pReport )
            return _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()));

        bool bSuccess = _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        return _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()), KinBodyConstPtr(pbody2->GetBody()));
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        if( !pReport )
            return _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()), KinBodyConstPtr(pbody2->GetBody()));

        bool bSuccess = _penv->CheckCollision(KinBodyConstPtr(pbody1->GetBody()), KinBodyConstPtr(pbody2->GetBody()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }

    bool CheckCollision(PyKinBody::PyLinkPtr plink)
    {
        CHECK_POINTER(plink);
        return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()));
    }

    bool CheckCollision(PyKinBody::PyLinkPtr plink, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(plink);
        if( !pReport )
            return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()));

        bool bSuccess = _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }

    bool CheckCollision(PyKinBody::PyLinkPtr plink1, PyKinBody::PyLinkPtr plink2)
    {
        CHECK_POINTER(plink1);
        CHECK_POINTER(plink2);
        return _penv->CheckCollision(KinBody::LinkConstPtr(plink1->GetLink()), KinBody::LinkConstPtr(plink2->GetLink()));
    }
    bool CheckCollision(PyKinBody::PyLinkPtr plink1, PyKinBody::PyLinkPtr plink2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(plink1);
        CHECK_POINTER(plink2);
        if( !pReport )
            return _penv->CheckCollision(KinBody::LinkConstPtr(plink1->GetLink()), KinBody::LinkConstPtr(plink2->GetLink()));

        bool bSuccess = _penv->CheckCollision(KinBody::LinkConstPtr(plink1->GetLink()), KinBody::LinkConstPtr(plink2->GetLink()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBody::PyLinkPtr plink, PyKinBodyPtr pbody)
    {
        CHECK_POINTER(plink);
        CHECK_POINTER(pbody);
        return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()), KinBodyConstPtr(pbody->GetBody()));
    }

    bool CheckCollision(PyKinBody::PyLinkPtr plink, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(plink);
        CHECK_POINTER(pbody);
        if( !pReport )
            return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()), KinBodyConstPtr(pbody->GetBody()));

        bool bSuccess = _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()), KinBodyConstPtr(pbody->GetBody()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }
    
    bool CheckCollision(PyKinBody::PyLinkPtr plink, object bodyexcluded, object linkexcluded)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody )
                vbodyexcluded.push_back(pbody->GetBody());
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLinkPtr plink = extract<PyKinBody::PyLinkPtr>(linkexcluded[i]);
            if( !!plink )
                vlinkexcluded.push_back(plink->GetLink());
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()),vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBody::PyLinkPtr plink, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody )
                vbodyexcluded.push_back(pbody->GetBody());
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLinkPtr plink = extract<PyKinBody::PyLinkPtr>(linkexcluded[i]);
            if( !!plink )
                vlinkexcluded.push_back(plink->GetLink());
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }

        if( !pReport )
            return _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()),vbodyexcluded,vlinkexcluded);

        bool bSuccess = _penv->CheckCollision(KinBody::LinkConstPtr(plink->GetLink()), vbodyexcluded, vlinkexcluded, pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.push_back(pbody->GetBody());
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLinkPtr plink = extract<PyKinBody::PyLinkPtr>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.push_back(plink->GetLink());
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }
        return _penv->CheckCollision(KinBodyConstPtr(pbody->GetBody()),vbodyexcluded,vlinkexcluded);
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( pbody != NULL )
                vbodyexcluded.push_back(pbody->GetBody());
            else
                RAVELOG_ERRORA("failed to get excluded body\n");
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBody::PyLinkPtr plink = extract<PyKinBody::PyLinkPtr>(linkexcluded[i]);
            if( plink != NULL )
                vlinkexcluded.push_back(plink->GetLink());
            else
                RAVELOG_ERRORA("failed to get excluded link\n");
        }

        if( !pReport )
            return _penv->CheckCollision(KinBodyConstPtr(pbody->GetBody()),vbodyexcluded,vlinkexcluded);

        bool bSuccess = _penv->CheckCollision(KinBodyConstPtr(pbody->GetBody()), vbodyexcluded, vlinkexcluded, pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }

    bool CheckCollision(PyRay* pyray, PyKinBodyPtr pbody)
    {
        return _penv->CheckCollision(pyray->r,KinBodyConstPtr(pbody->GetBody()));
    }

    bool CheckCollision(PyRay* pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        bool bSuccess = _penv->CheckCollision(pyray->r, KinBodyConstPtr(pbody->GetBody()), pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }

    object CheckCollisionRays(object rays, PyKinBodyPtr pbody,bool bFrontFacingOnly=false)
    {
        object shape = rays.attr("shape");
        int num = extract<int>(shape[0]);
        if( num == 0 )
            return boost::python::make_tuple(numeric::array(boost::python::list()).astype("i4"),numeric::array(boost::python::list()));
        if( extract<int>(shape[1]) != 6 )
            throw openrave_exception("rays object needs to be a 6xN vector\n");

        COLLISIONREPORT report;
        CollisionReportPtr preport(&report,null_deleter());

        RAY r;
        npy_intp dims[] = {num,6};
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
        dReal* ppos = (dReal*)PyArray_DATA(pypos);
        PyObject* pycollision = PyArray_SimpleNew(1,&dims[0], PyArray_BOOL);
        bool* pcollision = (bool*)PyArray_DATA(pycollision);
        for(int i = 0; i < num; ++i, ppos += 6) {
            vector<dReal> ray = ExtractArray<dReal>(rays[i]);
            r.pos.x = ray[0];
            r.pos.y = ray[1];
            r.pos.z = ray[2];
            r.dir.x = ray[3];
            r.dir.y = ray[4];
            r.dir.z = ray[5];
            bool bCollision = _penv->CheckCollision(r, KinBodyConstPtr(pbody->GetBody()), preport);
            pcollision[i] = false;
            ppos[0] = 0; ppos[1] = 0; ppos[2] = 0; ppos[3] = 0; ppos[4] = 0; ppos[5] = 0;
            if( bCollision && report.contacts.size() > 0 ) {
                if( !bFrontFacingOnly || dot3(report.contacts[0].norm,r.dir)<0 ) {
                    pcollision[i] = true;
                    ppos[0] = report.contacts[0].pos.x;
                    ppos[1] = report.contacts[0].pos.y;
                    ppos[2] = report.contacts[0].pos.z;
                    ppos[3] = report.contacts[0].norm.x;
                    ppos[4] = report.contacts[0].norm.y;
                    ppos[5] = report.contacts[0].norm.z;
                }
            }
        }

        return boost::python::make_tuple(static_cast<numeric::array>(handle<>(pycollision)),static_cast<numeric::array>(handle<>(pypos)));
    }

    bool CheckCollision(PyRay* pyray)
    {
        return _penv->CheckCollision(pyray->r);
    }
    
    bool CheckCollision(PyRay* pyray, PyCollisionReportPtr pReport)
    {
        bool bSuccess = _penv->CheckCollision(pyray->r, pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }
	
    bool Load(const string& filename) { return _penv->Load(filename); }
    bool Save(const string& filename) { return _penv->Save(filename.c_str()); }

    PyRobotBasePtr ReadRobotXMLFile(const string& filename)
    {
        RobotBasePtr probot = _penv->ReadRobotXMLFile(RobotBasePtr(), filename, std::list<std::pair<std::string,std::string> >());
        if( !probot )
            return PyRobotBasePtr();
        return PyRobotBasePtr(new PyRobotBase(probot,shared_from_this()));
    }
    PyRobotBasePtr ReadRobotXMLData(const string& data)
    {
        RobotBasePtr probot = _penv->ReadRobotXMLData(RobotBasePtr(), data, std::list<std::pair<std::string,std::string> >());
        if( !probot )
            return PyRobotBasePtr();
        return PyRobotBasePtr(new PyRobotBase(probot,shared_from_this()));
    }

    PyKinBodyPtr ReadKinBodyXMLFile(const string& filename)
    {
        KinBodyPtr pbody = _penv->ReadKinBodyXMLFile(KinBodyPtr(), filename, std::list<std::pair<std::string,std::string> >());
        if( !pbody )
            return PyKinBodyPtr();
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }

    PyKinBodyPtr ReadKinBodyXMLData(const string& data)
    {
        KinBodyPtr pbody = _penv->ReadKinBodyXMLData(KinBodyPtr(), data, std::list<std::pair<std::string,std::string> >());
        if( !pbody )
            return PyKinBodyPtr();
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }

    bool AddKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _penv->AddKinBody(pbody->GetBody()); }
    bool AddRobot(PyRobotBasePtr robot) { CHECK_POINTER(robot); return _penv->AddRobot(robot->GetRobot()); }
    bool RemoveKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _penv->RemoveKinBody(pbody->GetBody()); }
    
    PyKinBodyPtr GetKinBody(const string& name)
    {
        KinBodyPtr pbody = _penv->GetKinBody(name);
        if( !pbody )
            throw openrave_exception(boost::str(boost::format("failed to get body %s")%name));
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }
    PyRobotBasePtr GetRobot(const string& name)
    {
        RobotBasePtr probot = _penv->GetRobot(name);
        if( !probot )
            throw openrave_exception(boost::str(boost::format("failed to get robot %s")%name));
        return PyRobotBasePtr(new PyRobotBase(probot,shared_from_this()));
    }

    PyKinBodyPtr GetBodyFromNetworkId(int id)
    {
        KinBodyPtr pbody = _penv->GetBodyFromNetworkId(id);
        if( !pbody )
            throw openrave_exception(boost::str(boost::format("failed to get body id %d")%id));
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }

    PyKinBodyPtr CreateKinBody()
    {
        return PyKinBodyPtr(new PyKinBody(_penv->CreateKinBody(),shared_from_this()));
    }

    PyTrajectoryBasePtr CreateTrajectory(int nDOF) { return PyTrajectoryBasePtr(new PyTrajectoryBase(_penv->CreateTrajectory(nDOF),shared_from_this())); }

    int LoadProblem(PyProblemInstancePtr prob, const string& args) { CHECK_POINTER(prob); return _penv->LoadProblem(prob->GetProblem(),args); }
    bool RemoveProblem(PyProblemInstancePtr prob) { CHECK_POINTER(prob); return _penv->RemoveProblem(prob->GetProblem()); }
    
    object GetLoadedProblems()
    {
        std::list<ProblemInstancePtr> listProblems;
        boost::shared_ptr<void> lock = _penv->GetLoadedProblems(listProblems);
        boost::python::list problems;
        FOREACHC(itprob, listProblems)
            problems.append(PyProblemInstancePtr(new PyProblemInstance(*itprob,shared_from_this())));
        return problems;
    }

    bool SetPhysicsEngine(PyPhysicsEngineBasePtr pengine)
    {
        if( !pengine )
            return _penv->SetPhysicsEngine(PhysicsEngineBasePtr());
        return _penv->SetPhysicsEngine(pengine->GetPhysicsEngine());
    }
    PyPhysicsEngineBasePtr GetPhysicsEngine() { return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(_penv->GetPhysicsEngine(),shared_from_this())); }

    PyVoidHandle RegisterCollisionCallback(object fncallback)
    {
        if( !fncallback )
            throw openrave_exception("callback not specified");
        boost::shared_ptr<void> p = _penv->RegisterCollisionCallback(boost::bind(&PyEnvironmentBase::_CollisionCallback,shared_from_this(),fncallback,_1,_2));
        if( !p )
            throw openrave_exception("registration handle is NULL");
        return PyVoidHandle(p);
    }

    void StepSimulation(dReal timeStep) { _penv->StepSimulation(timeStep); }
    void StartSimulation(dReal fDeltaTime, bool bRealTime=true) { _penv->StartSimulation(fDeltaTime,bRealTime); }
    void StopSimulation() { _penv->StopSimulation(); }
    uint64_t GetSimulationTime() { return _penv->GetSimulationTime(); }

    void LockPhysics(bool bLock)
    {
#if BOOST_VERSION < 103500
        boost::mutex::scoped_lock envlock(_envmutex);
        if( bLock ) {
            if( _listfreelocks.size() > 0 ) {
                _listfreelocks.back()->lock();
                _listenvlocks.splice(_listenvlocks.end(),_listfreelocks,--_listfreelocks.end());
            }
            else
                _listenvlocks.push_back(boost::shared_ptr<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
        }
        else {
            _listenvlocks.back()->unlock();
            _listfreelocks.splice(_listfreelocks.end(),_listenvlocks,--_listenvlocks.end());
        }
#else
        if( bLock )
            _penv->GetMutex().lock();
        else
            _penv->GetMutex().unlock();
#endif
    }
    void LockPhysics(bool bLock, float timeout)
    {
        RAVELOG_WARN("Environment.LockPhysics timeout is ignored\n");
        LockPhysics(bLock);
    }

    void __enter__()
    {
        LockPhysics(true);
    }

    void __exit__(object type, object value, object traceback)
    {
        LockPhysics(false);
    }

    bool SetViewer(const string& viewername, bool showviewer=true)
    {
        if( !!_threadviewer ) // wait for the viewer
            _threadviewer->join();
        _threadviewer.reset();
        
        _penv->AttachViewer(RaveViewerBasePtr());

        if( viewername.size() > 0 ) {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _threadviewer.reset(new boost::thread(boost::bind(&PyEnvironmentBase::_ViewerThread, shared_from_this(), viewername, showviewer)));
            _conditionViewer.wait(lock);
            
            if( !_penv->GetViewer() || _penv->GetViewer()->GetXMLId() != viewername ) {
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

    PyRaveViewerBasePtr GetViewer() { return PyRaveViewerBasePtr(new PyRaveViewerBase(_penv->GetViewer(),shared_from_this())); }

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

        vector<float> vpoints = ExtractArray<float>(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1.0);
        if( ocolors != object() ) {
            object colorshape = ocolors.attr("shape");
            if( len(colorshape) == 1 )
                vcolor = ExtractVector34(ocolors,1.0f);
            else if( len(colorshape) == 2 ) {
                int numcolors = extract<int>(colorshape[0]);
                int colordim = extract<int>(colorshape[1]);
                if( numcolors*3 != (int)vpoints.size() )
                    throw openrave_exception(boost::str(boost::format("len colors needs to be %d")%(vpoints.size()/3)));
                bool bhasalpha = false;
                if( colordim == 4 )
                    bhasalpha = true;
                else if( colordim != 3 )
                    throw openrave_exception("colors dim needs to be 3 or 4");
                
                vector<float> vcolors = ExtractArray<float>(ocolors.attr("flat"));
                return object(PyGraphHandle(_penv->plot3(&vpoints[0],vpoints.size()/3,sizeof(float)*3,pointsize,&vcolors[0],drawstyle,bhasalpha)));
            }
            else
                throw openrave_exception("color is wrong dim");
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

        vector<float> vpoints = ExtractArray<float>(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors,1.0f);
            else {
                vector<float> vcolors = ExtractArray<float>(ocolors.attr("flat"));
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

        vector<float> vpoints = ExtractArray<float>(opoints.attr("flat"));
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolors != object() ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 )
                vcolor = ExtractVector34(ocolors,1.0f);
            else {
                vector<float> vcolors = ExtractArray<float>(ocolors.attr("flat"));
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
            vcolor = ExtractVector34(ocolor,1.0f);
        return object(PyGraphHandle(_penv->drawarrow(ExtractVector3(op1),ExtractVector3(op2),linewidth,vcolor)));
    }

    object drawbox(object opos, object oextents, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( ocolor != object() )
            vcolor = ExtractVector34(ocolor,1.0f);
        return object(PyGraphHandle(_penv->drawbox(ExtractVector3(opos),ExtractVector3(oextents))));
    }

    object drawplane(object otransform, object oextents, const boost::multi_array<float,2>& _vtexture)
    {
        boost::multi_array<float,3> vtexture(boost::extents[1][_vtexture.shape()[0]][_vtexture.shape()[1]]);
        vtexture[0] = _vtexture;
        boost::array<size_t,3> dims = {{_vtexture.shape()[0],_vtexture.shape()[1],1}};
        vtexture.reshape(dims);
        return object(PyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture)));
    }
    object drawplane(object otransform, object oextents, const boost::multi_array<float,3>& vtexture)
    {
        return object(PyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture)));
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
        vector<float> vpoints = ExtractArray<float>(opoints.attr("flat"));

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
                vcolor = ExtractVector34(ocolors,1.0f);
            else {
                vector<float> vcolors = ExtractArray<float>(ocolors.attr("flat"));
                if( vcolors.size() != vpoints.size() )
                    throw openrave_exception("colors needs to be Nx3 matrix (same size as points)");
                vcolor = RaveVector<float>(vcolors[0],vcolors[1],vcolors[2],1);
            }
        }
        
        return object(PyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,vcolor)));
    }

    void SetCamera(object transform) { _penv->SetCamera(ExtractTransform(transform)); }

    void SetCameraLookAt(object lookat, object campos, object camup) {
        _penv->SetCameraLookAt(ExtractFloat3(lookat), ExtractFloat3(campos), ExtractFloat3(camup));
    }
    object GetCameraTransform() { return ReturnTransform(_penv->GetCameraTransform()); }

    object GetCameraImage(int width, int height, object extrinsic, object oKK)
    {
        vector<float> vKK = ExtractArray<float>(oKK);
        if( vKK.size() != 4 )
            throw openrave_exception("KK needs to be of size 4");
        SensorBase::CameraIntrinsics KK(vKK[0],vKK[1],vKK[2],vKK[3]);
        vector<uint8_t> memory;
        if( !_penv->GetCameraImage(memory, width,height,RaveTransform<float>(ExtractTransform(extrinsic)), KK) )
            return object();
        std::vector<npy_intp> dims(3); dims[0] = height; dims[1] = width; dims[2] = 3;
        return toPyArray(memory,dims);
    }

    object GetBodies()
    {
        std::vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        boost::python::list bodies;
        FOREACHC(itbody, vbodies)
            bodies.append(PyKinBodyPtr(new PyKinBody(*itbody,shared_from_this())));
        return bodies;
    }

    object GetRobots()
    {
        std::vector<RobotBasePtr> vrobots;
        _penv->GetRobots(vrobots);
        boost::python::list robots;
        FOREACHC(itrobot, vrobots)
            robots.append(PyRobotBasePtr(new PyRobotBase(*itrobot,shared_from_this())));
        return robots;
    }

    void UpdatePublishedBodies()
    {
        _penv->UpdatePublishedBodies();
    }

    boost::shared_ptr<PyKinBody::PyLink::PyTriMesh> Triangulate(PyKinBodyPtr pbody)
    {
        CHECK_POINTER(pbody);
        KinBody::Link::TRIMESH mesh;
        if( !_penv->Triangulate(mesh,pbody->GetBody()) )
            throw openrave_exception(boost::str(boost::format("failed to triangulate body %s")%pbody->GetBody()->GetName()));
        return boost::shared_ptr<PyKinBody::PyLink::PyTriMesh>(new PyKinBody::PyLink::PyTriMesh(mesh));
    }
    boost::shared_ptr<PyKinBody::PyLink::PyTriMesh> TriangulateScene(EnvironmentBase::TriangulateOptions opts, const string& name)
    {
        KinBody::Link::TRIMESH mesh;
        if( !_penv->TriangulateScene(mesh,opts,name) )
            throw openrave_exception(boost::str(boost::format("failed to triangulate scene: %d, %s")%opts%name));
        return boost::shared_ptr<PyKinBody::PyLink::PyTriMesh>(new PyKinBody::PyLink::PyTriMesh(mesh));
    }

    void SetDebugLevel(DebugLevel level) { _penv->SetDebugLevel(level); }
    DebugLevel GetDebugLevel() const { return _penv->GetDebugLevel(); }

    string GetHomeDirectory() { return _penv->GetHomeDirectory(); }
};

void PyKinBody::__enter__()
{
    // necessary to lock physics to prevent multiple threads from interfering
    if( _listStateSavers.size() == 0 )
        _pyenv->LockPhysics(true);
    _listStateSavers.push_back(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody)));
}

void PyKinBody::__exit__(object type, object value, object traceback)
{
    _listStateSavers.pop_back();
    if( _listStateSavers.size() == 0 )
        _pyenv->LockPhysics(false);
}

void PyRobotBase::__enter__()
{
    // necessary to lock physics to prevent multiple threads from interfering
    if( _listStateSavers.size() == 0 )
        _pyenv->LockPhysics(true);
    _listStateSavers.push_back(boost::shared_ptr<void>(new RobotBase::RobotStateSaver(_probot)));
}

object quatFromAxisAngle1(object oaxis)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen < 1e-6 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal sang = RaveSin(axislen*0.5f)/axislen;
    return toPyVector4(Vector(RaveCos(axislen*0.5f),axis[0]*sang,axis[1]*sang,axis[2]*sang));
}

object quatFromAxisAngle2(object oaxis, object oangle)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen == 0 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal angle = extract<dReal>(oangle)*0.5f;
    dReal sang = RaveSin(angle)/axislen;
    return toPyVector4(Vector(RaveCos(angle),axis[0]*sang,axis[1]*sang,axis[2]*sang));
}

object quatFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    return toPyVector4(Transform(t).rot);
}

object axisAngleFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    Vector quat = Transform(t).rot;
    if( quat.x < 0 )
        quat = -quat;
    dReal sinang = quat.y*quat.y+quat.z*quat.z+quat.w*quat.w;
    if( RaveFabs(sinang) > 0 ) {
        sinang = RaveSqrt(sinang);
        dReal f = 2.0*RaveAtan2(sinang,quat.x)/sinang;
        return toPyVector3(Vector(quat.y*f,quat.z*f,quat.w*f));
    }
    return toPyVector3(Vector(0,0,0));
}

object rotationMatrixFromQuat(object oquat)
{
    Transform t; t.rot = ExtractVector4(oquat);
    return toPyArrayRotation(TransformMatrix(t));
}

object rotationMatrixFromQArray(object qarray)
{
    boost::python::list orots;
    int N = len(qarray);
    for(int i = 0; i < N; ++i)
        orots.append(rotationMatrixFromQuat(qarray[i]));
    return orots;
}

object matrixFromQuat(object oquat)
{
    Transform t; t.rot = ExtractVector4(oquat);
    return toPyArray(TransformMatrix(t));
}

object rotationMatrixFromAxisAngle1(object oaxis)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen < 1e-6 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal sang = RaveSin(axislen*0.5f)/axislen;
    Transform t; t.rot = Vector(RaveCos(axislen*0.5f),axis[0]*sang,axis[1]*sang,axis[2]*sang);
    return toPyArrayRotation(TransformMatrix(t));
}

object rotationMatrixFromAxisAngle2(object oaxis, object oangle)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen == 0 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal angle = extract<dReal>(oangle)*0.5f;
    dReal sang = RaveSin(angle)/axislen;
    Transform t; t.rot = Vector(RaveCos(angle),axis[0]*sang,axis[1]*sang,axis[2]*sang);
    return toPyArrayRotation(TransformMatrix(t));
}

object matrixFromAxisAngle1(object oaxis)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen < 1e-6 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal sang = RaveSin(axislen*0.5f)/axislen;
    Transform t; t.rot = Vector(RaveCos(axislen*0.5f),axis[0]*sang,axis[1]*sang,axis[2]*sang);
    return toPyArray(TransformMatrix(t));
}

object matrixFromAxisAngle2(object oaxis, object oangle)
{
    Vector axis = ExtractVector3(oaxis);
    dReal axislen = RaveSqrt(axis.lengthsqr3());
    if( axislen == 0 )
        return numeric::array(boost::python::make_tuple((dReal)1,(dReal)0,(dReal)0,(dReal)0));
    dReal angle = extract<dReal>(oangle)*0.5f;
    dReal sang = RaveSin(angle)/axislen;
    Transform t; t.rot = Vector(RaveCos(angle),axis[0]*sang,axis[1]*sang,axis[2]*sang);
    return toPyArray(TransformMatrix(t));
}

object matrixFromPose(object opose)
{
    return toPyArray(TransformMatrix(ExtractTransformType<dReal>(opose)));
}

object matrixFromPoses(object oposes)
{
    boost::python::list omatrices;
    int N = len(oposes);
    for(int i = 0; i < N; ++i)
        omatrices.append(matrixFromPose(oposes[i]));
    return omatrices;
}

object poseFromMatrix(object o)
{
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        t.m[4*i+0] = extract<dReal>(o[i][0]);
        t.m[4*i+1] = extract<dReal>(o[i][1]);
        t.m[4*i+2] = extract<dReal>(o[i][2]);
        t.trans[i] = extract<dReal>(o[i][3]);
    }
    return toPyArray(Transform(t));
}

object poseFromMatrices(object otransforms)
{
    int N = len(otransforms);
    if( N == 0 )
        return static_cast<numeric::array>(handle<>());

    npy_intp dims[] = {N,7};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pvalues = (dReal*)PyArray_DATA(pyvalues);
    TransformMatrix tm;
    for(int j = 0; j < N; ++j) {
        object o = otransforms[j];
        for(int i = 0; i < 3; ++i) {
            tm.m[4*i+0] = extract<dReal>(o[i][0]);
            tm.m[4*i+1] = extract<dReal>(o[i][1]);
            tm.m[4*i+2] = extract<dReal>(o[i][2]);
            tm.trans[i] = extract<dReal>(o[i][3]);
        }
        Transform tpose(tm);
        pvalues[0] = tpose.rot.x; pvalues[1] = tpose.rot.y; pvalues[2] = tpose.rot.z; pvalues[3] = tpose.rot.w;
        pvalues[4] = tpose.trans.x; pvalues[5] = tpose.trans.y; pvalues[6] = tpose.trans.z;
        pvalues += 7;
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

object invertPoses(object o)
{
    int N = len(o);
    if( N == 0 )
        return numeric::array(boost::python::list());

    npy_intp dims[] = {N,7};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
    for(int i = 0; i < N; ++i, ptrans += 7) {
        object oinputtrans = o[i];
        Transform t = Transform(Vector(extract<dReal>(oinputtrans[0]),extract<dReal>(oinputtrans[1]),extract<dReal>(oinputtrans[2]),extract<dReal>(oinputtrans[3])),
                                Vector(extract<dReal>(oinputtrans[4]),extract<dReal>(oinputtrans[5]),extract<dReal>(oinputtrans[6]))).inverse();
        ptrans[0] = t.rot.x; ptrans[1] = t.rot.y; ptrans[2] = t.rot.z; ptrans[3] = t.rot.w;
        ptrans[4] = t.trans.x; ptrans[5] = t.trans.y; ptrans[6] = t.trans.z;
    }
    return static_cast<numeric::array>(handle<>(pytrans));
}

object quatRotateDirection(object source, object target)
{
    Vector vsource = ExtractVector3(source), vtarget = ExtractVector3(target);
    Vector rottodirection;
    cross3(rottodirection, vsource,vtarget);
    dReal fsin = RaveSqrt(rottodirection.lengthsqr3());
    dReal fcos = dot3(vsource, vtarget);
    Transform torient;
    if( fsin > 1e-6f ) {
        torient.rotfromaxisangle(rottodirection*(1/fsin), RaveAtan2(fsin, fcos));
    }
    else if( fcos < 0 ) {
        // hand is flipped 180, rotate around x axis
        rottodirection = Vector(1,0,0);
        rottodirection -= vsource * dot3(vsource, rottodirection);
        rottodirection.normalize3();
        torient.rotfromaxisangle(rottodirection, RaveAtan2(fsin, fcos));
    }
    return toPyVector4(torient.rot);
}

string matrixSerialization(object o)
{
    stringstream ss; ss << ExtractTransformMatrix(o);
    return ss.str();
}

string poseSerialization(object o)
{
    stringstream ss; ss << ExtractTransform(o);
    return ss.str();
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetController_overloads, SetController, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StartSimulation_overloads, StartSimulation, 1, 2)
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

std::string openravepyCompilerVersion()
{
    stringstream ss;
#if defined(_MSC_VER)
    ss << "msvc " << _MSC_VER;
#elif defined(__GNUC__)
    ss << "gcc " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__;
#elif defined(__MINGW32_VERSION)
    ss << "mingw " << __MINGW32_VERSION;
#endif
    return ss.str();
}

BOOST_PYTHON_MODULE(openravepy_int)
{
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< openrave_exception >( "_openrave_exception_" )
        .def( init<const std::string&>() )
        .def( init<const openrave_exception&>() )
        .def( "message", &openrave_exception::message, return_copy_const_ref() )
        .def( "__str__", &openrave_exception::message, return_copy_const_ref() )
        ;
    exception_translator<openrave_exception>();

    enum_<DebugLevel>("DebugLevel")
        .value("Fatal",Level_Fatal)
        .value("Error",Level_Error)
        .value("Warn",Level_Warn)
        .value("Info",Level_Info)
        .value("Debug",Level_Debug)
        .value("Verbose",Level_Verbose)
        ;
    enum_<SerializationOptions>("SerializationOptions")
        .value("Kinematics",SO_Kinematics)
        .value("Dynamics",SO_Dynamics)
        .value("BodyState",SO_BodyState)
        .value("NamesAndFiles",SO_NamesAndFiles)
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
        ;
    enum_<CollisionOptions>("CollisionOptions")
        .value("Distance",CO_Distance)
        .value("UseTolerance",CO_UseTolerance)
        .value("Contacts",CO_Contacts)
        .value("RayAnyHit",CO_RayAnyHit)
        ;
    enum_<CollisionAction>("CollisionAction")
        .value("DefaultAction",CA_DefaultAction)
        .value("Ignore",CA_Ignore)
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

    // several handles are returned by void
    class_< boost::shared_ptr< void > >("VoidPointer");

    class_<PyEnvironmentBase, boost::shared_ptr<PyEnvironmentBase> > classenv("Environment");
    class_<PyGraphHandle, boost::shared_ptr<PyGraphHandle> >("GraphHandle");
    class_<PyRay, boost::shared_ptr<PyRay> >("Ray")
        .def(init<object,object>())
        .def("dir",&PyRay::dir)
        .def("pos",&PyRay::pos)
        .def_pickle(Ray_pickle_suite())
        ;
    class_<PyAABB, boost::shared_ptr<PyAABB> >("AABB")
        .def(init<object,object>())
        .def("extents",&PyAABB::extents)
        .def("pos",&PyAABB::pos)
        .def_pickle(AABB_pickle_suite())
        ;
    class_<PyInterfaceBase, boost::shared_ptr<PyInterfaceBase> >("Interface", no_init)
        .def("GetInterfaceType",&PyInterfaceBase::GetInterfaceType)
        .def("GetXMLId",&PyInterfaceBase::GetXMLId)
        .def("GetPluginName",&PyInterfaceBase::GetPluginName)
        .def("GetDescription",&PyInterfaceBase::GetDescription)
        .def("GetEnv",&PyInterfaceBase::GetEnv)
        .def("Clone",&PyInterfaceBase::Clone,args("ref","cloningoptions"))
        .def("SetUserData",&PyInterfaceBase::SetUserData,args("data"))
        .def("GetUserData",&PyInterfaceBase::GetUserData)
        .def("SendCommand",&PyInterfaceBase::SendCommand,args("cmd"))
        ;

    class_<PyPluginInfo, boost::shared_ptr<PyPluginInfo> >("PluginInfo",no_init)
        .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
        ;

    {    
        bool (PyKinBody::*pkinbodyself)() = &PyKinBody::CheckSelfCollision;
        bool (PyKinBody::*pkinbodyselfr)(PyCollisionReportPtr) = &PyKinBody::CheckSelfCollision;
        void (PyKinBody::*psetjointvalues1)(object) = &PyKinBody::SetJointValues;
        void (PyKinBody::*psetjointvalues2)(object,object) = &PyKinBody::SetJointValues;
        scope kinbody = class_<PyKinBody, boost::shared_ptr<PyKinBody>, bases<PyInterfaceBase> >("KinBody", no_init)
            .def("InitFromFile",&PyKinBody::InitFromFile,args("filename"))
            .def("InitFromData",&PyKinBody::InitFromData,args("data"))
            .def("SetName", &PyKinBody::SetName,args("name"))
            .def("GetName",&PyKinBody::GetName)
            .def("GetDOF",&PyKinBody::GetDOF)
            .def("GetJointValues",&PyKinBody::GetJointValues)
            .def("GetJointLimits",&PyKinBody::GetJointLimits)
            .def("GetLinks",&PyKinBody::GetLinks)
            .def("GetLink",&PyKinBody::GetLink,args("name"))
            .def("GetJoints",&PyKinBody::GetJoints)
            .def("GetPassiveJoints",&PyKinBody::GetPassiveJoints)
            .def("GetDependencyOrderedJoints",&PyKinBody::GetDependencyOrderedJoints)
            .def("GetJoint",&PyKinBody::GetJoint,args("name"))
            .def("GetTransform",&PyKinBody::GetTransform)
            .def("GetBodyTransformations",&PyKinBody::GetBodyTransformations)
            .def("SetBodyTransformations",&PyKinBody::SetBodyTransformations,args("transforms"))
            .def("GetLinkVelocities",&PyKinBody::GetLinkVelocities)
            .def("ComputeAABB",&PyKinBody::ComputeAABB)
            .def("Enable",&PyKinBody::Enable,args("enable"))
            .def("IsEnabled",&PyKinBody::IsEnabled)
            .def("SetTransform",&PyKinBody::SetTransform,args("transform"))
            .def("SetJointValues",psetjointvalues1,args("values"))
            .def("SetJointValues",psetjointvalues2,args("values","jointindices"))
            .def("SetJointTorques",&PyKinBody::SetJointTorques,args("torques","add"))
            .def("SetTransformWithJointValues",&PyKinBody::SetTransformWithJointValues,args("transform","values"))
            .def("CalculateJacobian",&PyKinBody::CalculateJacobian,args("linkindex","offset"))
            .def("CalculateRotationJacobian",&PyKinBody::CalculateRotationJacobian,args("linkindex","quat"))
            .def("CalculateAngularVelocityJacobian",&PyKinBody::CalculateAngularVelocityJacobian,args("linkindex"))
            .def("CheckSelfCollision",pkinbodyself)
            .def("CheckSelfCollision",pkinbodyselfr,args("report"))
            .def("IsAttached",&PyKinBody::IsAttached,args("body"))
            .def("GetAttached",&PyKinBody::GetAttached)
            .def("IsRobot",&PyKinBody::IsRobot)
            .def("GetNetworkId",&PyKinBody::GetNetworkId)
            .def("DoesAffect",&PyKinBody::DoesAffect,args("jointindex","linkindex"))
            .def("GetForwardKinematics",&PyKinBody::GetForwardKinematics)
            .def("SetGuiData",&PyKinBody::SetGuiData,args("data"))
            .def("GetGuiData",&PyKinBody::GetGuiData)
            .def("GetXMLFilename",&PyKinBody::GetXMLFilename)
            .def("GetNonAdjacentLinks",&PyKinBody::GetNonAdjacentLinks)
            .def("GetAdjacentLinks",&PyKinBody::GetAdjacentLinks)
            .def("GetPhysicsData",&PyKinBody::GetPhysicsData)
            .def("GetCollisionData",&PyKinBody::GetCollisionData)
            .def("GetUpdateStamp",&PyKinBody::GetUpdateStamp)
            .def("serialize",&PyKinBody::serialize,args("options"))
            .def("GetKinematicsGeometryHash",&PyKinBody::GetKinematicsGeometryHash)
            .def("CreateKinBodyStateSaver",&PyKinBody::CreateKinBodyStateSaver)
            .def("__enter__",&PyKinBody::__enter__)
            .def("__exit__",&PyKinBody::__exit__)
            ;

        {        
            scope link = class_<PyKinBody::PyLink, boost::shared_ptr<PyKinBody::PyLink> >("Link", no_init)
                .def("GetName",&PyKinBody::PyLink::GetName)
                .def("GetIndex",&PyKinBody::PyLink::GetIndex)
                .def("IsEnabled",&PyKinBody::PyLink::IsEnabled)
                .def("IsStatic",&PyKinBody::PyLink::IsStatic)
                .def("Enable",&PyKinBody::PyLink::Enable,args("enable"))
                .def("GetParent",&PyKinBody::PyLink::GetParent)
                .def("GetCollisionData",&PyKinBody::PyLink::GetCollisionData)
                .def("ComputeAABB",&PyKinBody::PyLink::ComputeAABB)
                .def("GetTransform",&PyKinBody::PyLink::GetTransform)
                .def("GetCOMOffset",&PyKinBody::PyLink::GetCOMOffset)
                .def("GetInertia",&PyKinBody::PyLink::GetInertia)
                .def("GetMass",&PyKinBody::PyLink::GetMass)
                .def("SetTransform",&PyKinBody::PyLink::SetTransform,args("transform"))
                .def("SetForce",&PyKinBody::PyLink::SetForce,args("force","pos","add"))
                .def("SetTorque",&PyKinBody::PyLink::SetTorque,args("torque","add"))
                .def("GetGeometries",&PyKinBody::PyLink::GetGeometries)
                ;

            class_<PyKinBody::PyLink::PyTriMesh, boost::shared_ptr<PyKinBody::PyLink::PyTriMesh> >("TriMesh")
                .def(init<object,object>())
                .def_readwrite("vertices",&PyKinBody::PyLink::PyTriMesh::vertices)
                .def_readwrite("indices",&PyKinBody::PyLink::PyTriMesh::indices)
                ;

            {
                scope geomproperties = class_<PyKinBody::PyLink::PyGeomProperties, boost::shared_ptr<PyKinBody::PyLink::PyGeomProperties> >("GeomProperties",no_init)
                    .def("SetCollisionMesh",&PyKinBody::PyLink::PyGeomProperties::SetCollisionMesh,args("trimesh"))
                    .def("GetCollisionMesh",&PyKinBody::PyLink::PyGeomProperties::GetCollisionMesh)
                    .def("SetDraw",&PyKinBody::PyLink::PyGeomProperties::SetDraw,args("draw"))
                    .def("SetTransparency",&PyKinBody::PyLink::PyGeomProperties::SetTransparency,args("transparency"))
                    .def("IsDraw",&PyKinBody::PyLink::PyGeomProperties::IsDraw)
                    .def("IsModifiable",&PyKinBody::PyLink::PyGeomProperties::IsModifiable)
                    .def("GetType",&PyKinBody::PyLink::PyGeomProperties::GetType)
                    .def("GetTransform",&PyKinBody::PyLink::PyGeomProperties::GetTransform)
                    .def("GetSphereRadius",&PyKinBody::PyLink::PyGeomProperties::GetSphereRadius)
                    .def("GetCylinderRadius",&PyKinBody::PyLink::PyGeomProperties::GetCylinderRadius)
                    .def("GetCylinderHeight",&PyKinBody::PyLink::PyGeomProperties::GetCylinderHeight)
                    .def("GetBoxExtents",&PyKinBody::PyLink::PyGeomProperties::GetBoxExtents)
                    ;
                enum_<KinBody::Link::GEOMPROPERTIES::GeomType>("Type")
                    .value("None",KinBody::Link::GEOMPROPERTIES::GeomNone)
                    .value("Box",KinBody::Link::GEOMPROPERTIES::GeomBox)
                    .value("Sphere",KinBody::Link::GEOMPROPERTIES::GeomSphere)
                    .value("Cylinder",KinBody::Link::GEOMPROPERTIES::GeomCylinder)
                    .value("Trimesh",KinBody::Link::GEOMPROPERTIES::GeomTrimesh)
                    ;
            }
        }

        {
            scope joint = class_<PyKinBody::PyJoint, boost::shared_ptr<PyKinBody::PyJoint> >("Joint",no_init)
                .def("GetName", &PyKinBody::PyJoint::GetName)
                .def("GetMimicJointIndex", &PyKinBody::PyJoint::GetMimicJointIndex)
                .def("GetMimicCoeffs", &PyKinBody::PyJoint::GetMimicCoeffs)
                .def("GetMaxVel", &PyKinBody::PyJoint::GetMaxVel)
                .def("GetMaxAccel", &PyKinBody::PyJoint::GetMaxAccel)
                .def("GetMaxTorque", &PyKinBody::PyJoint::GetMaxTorque)
                .def("GetDOFIndex", &PyKinBody::PyJoint::GetDOFIndex)
                .def("GetJointIndex", &PyKinBody::PyJoint::GetJointIndex)
                .def("GetParent", &PyKinBody::PyJoint::GetParent)
                .def("GetFirstAttached", &PyKinBody::PyJoint::GetFirstAttached)
                .def("GetSecondAttached", &PyKinBody::PyJoint::GetSecondAttached)
                .def("GetType", &PyKinBody::PyJoint::GetType)
                .def("GetDOF", &PyKinBody::PyJoint::GetDOF)
                .def("GetValues", &PyKinBody::PyJoint::GetValues)
                .def("GetVelocities", &PyKinBody::PyJoint::GetVelocities)
                .def("GetAnchor", &PyKinBody::PyJoint::GetAnchor)
                .def("GetAxis", &PyKinBody::PyJoint::GetAxis,args("axis"))
                .def("GetInternalHierarchyAnchor", &PyKinBody::PyJoint::GetInternalHierarchyAnchor)
                .def("GetInternalHierarchyAxis", &PyKinBody::PyJoint::GetInternalHierarchyAxis,args("axis"))
                .def("GetInternalHierarchyLeftTransform",&PyKinBody::PyJoint::GetInternalHierarchyLeftTransform)
                .def("GetInternalHierarchyRightTransform",&PyKinBody::PyJoint::GetInternalHierarchyRightTransform)
                .def("GetLimits", &PyKinBody::PyJoint::GetLimits)
                .def("SetJointOffset",&PyKinBody::PyJoint::SetJointOffset,args("offset"))
                .def("SetJointLimits",&PyKinBody::PyJoint::SetJointLimits,args("lower","upper"))
                .def("SetResolution",&PyKinBody::PyJoint::SetResolution,args("resolution"))
                .def("SetWeights",&PyKinBody::PyJoint::SetWeights,args("weights"))
                .def("AddTorque",&PyKinBody::PyJoint::AddTorque,args("torques"))
                ;
            
            enum_<KinBody::Joint::JointType>("Type")
                .value("None",KinBody::Joint::JointNone)
                .value("Hinge",KinBody::Joint::JointHinge)
                .value("Revolute",KinBody::Joint::JointRevolute)
                .value("Slider",KinBody::Joint::JointSlider)
                .value("Prismatic",KinBody::Joint::JointPrismatic)
                .value("Universal",KinBody::Joint::JointUniversal)
                .value("Hinge2",KinBody::Joint::JointHinge2)
                .value("Spherical",KinBody::Joint::JointSpherical)
                ;
        }
    }

    class_<PyCollisionReport::PYCONTACT, boost::shared_ptr<PyCollisionReport::PYCONTACT> >("Contact")
        .def_readwrite("pos",&PyCollisionReport::PYCONTACT::pos)
        .def_readwrite("norm",&PyCollisionReport::PYCONTACT::norm)
        ;
    class_<PyCollisionReport, boost::shared_ptr<PyCollisionReport> >("CollisionReport")
        .def_readwrite("options",&PyCollisionReport::options)
        .def_readwrite("plink1",&PyCollisionReport::plink1)
        .def_readwrite("plink2",&PyCollisionReport::plink2)
        .def_readwrite("numCols",&PyCollisionReport::numCols)
        .def_readwrite("minDistance",&PyCollisionReport::minDistance)
        .def_readwrite("numWithinTol",&PyCollisionReport::numWithinTol)
        .def_readwrite("contacts",&PyCollisionReport::contacts)
        ;

    {
        void (PyRobotBase::*psetactivedofs1)(object) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(object, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(object, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBodyPtr,object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab3)(PyKinBodyPtr,PyKinBody::PyLinkPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab4)(PyKinBodyPtr,PyKinBody::PyLinkPtr,object) = &PyRobotBase::Grab;

        scope robot = class_<PyRobotBase, boost::shared_ptr<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", no_init)
            .def("GetManipulators",&PyRobotBase::GetManipulators)
            .def("SetActiveManipulator",&PyRobotBase::SetActiveManipulator,args("manipindex"))
            .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator)
            .def("GetActiveManipulatorIndex",&PyRobotBase::GetActiveManipulatorIndex)
            .def("GetSensors",&PyRobotBase::GetSensors)
            .def("GetController",&PyRobotBase::GetController)
            .def("SetController",&PyRobotBase::SetController,SetController_overloads(args("controller","args")))
            .def("SetActiveDOFs",psetactivedofs1,args("jointindices"))
            .def("SetActiveDOFs",psetactivedofs2,args("jointindices","affine"))
            .def("SetActiveDOFs",psetactivedofs3,args("jointindices","affine","rotationaxis"))
            .def("GetActiveDOF",&PyRobotBase::GetActiveDOF)
            .def("GetAffineDOF",&PyRobotBase::GetAffineDOF)
            .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex,args("index"))
            .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis)
            .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits,args("lower","upper"))
            .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits,args("lower","upper"))
            .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits,args("lower","upper"))
            .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,args("lower","upper"))
            .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels,args("lower","upper"))
            .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels,args("velocity"))
            .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels,args("velocity"))
            .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels,args("velocity"))
            .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution,args("resolution"))
            .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution,args("resolution"))
            .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution,args("resolution"))
            .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution,args("resolution"))
            .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights,args("weights"))
            .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights,args("weights"))
            .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights,args("weights"))
            .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights,args("weights"))
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
            .def("GetAffineTranslationWeights",&PyRobotBase::GetAffineTranslationWeights)
            .def("GetAffineRotationAxisWeights",&PyRobotBase::GetAffineRotationAxisWeights)
            .def("GetAffineRotation3DWeights",&PyRobotBase::GetAffineRotation3DWeights)
            .def("GetAffineRotationQuatWeights",&PyRobotBase::GetAffineRotationQuatWeights)
            .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,args("values"))
            .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues)
            .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities)
            .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities)
            .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits)
            .def("GetActiveJointIndex",&PyRobotBase::GetActiveJointIndex,args("index"))
            .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
            .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,args("linkindex","offset"))
            .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,args("linkindex","quat"))
            .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,args("linkindex"))
            .def("Grab",pgrab1,args("body"))
            .def("Grab",pgrab2,args("body"))
            .def("Grab",pgrab3,args("body"))
            .def("Grab",pgrab4,args("body"))
            .def("Release",&PyRobotBase::Release,args("body"))
            .def("ReleaseAllGrabbed",&PyRobotBase::ReleaseAllGrabbed)
            .def("RegrabAll",&PyRobotBase::RegrabAll)
            .def("IsGrabbing",&PyRobotBase::IsGrabbing,args("body"))
            .def("GetGrabbed",&PyRobotBase::GetGrabbed)
            .def("WaitForController",&PyRobotBase::WaitForController,args("timeout"))
            .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash)
            .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver)
            ;
        
        object (PyRobotBase::PyManipulator::*pmanipik)(object, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision2)(object,PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        class_<PyRobotBase::PyManipulator, boost::shared_ptr<PyRobotBase::PyManipulator> >("Manipulator", no_init)
            .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetEndEffectorTransform)
            .def("GetName",&PyRobotBase::PyManipulator::GetName)
            .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot)
            .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIKSolver)
            .def("InitIKSolver",&PyRobotBase::PyManipulator::InitIKSolver,args("iksolver"))
            .def("GetIKSolverName",&PyRobotBase::PyManipulator::GetIKSolverName)
            .def("HasIKSolver",&PyRobotBase::PyManipulator::HasIKSolver)
            .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters)
            .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters)
            .def("FindIKSolution",pmanipik,args("transform","envcheck"))
            .def("FindIKSolution",pmanipikf,args("transform","freevalues","envcheck"))
            .def("FindIKSolutions",pmanipiks,args("transform","envcheck"))
            .def("FindIKSolutions",pmanipiksf,args("transform","freevalues","envcheck"))
            .def("GetBase",&PyRobotBase::PyManipulator::GetBase)
            .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector)
            .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform)
            .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints)
            .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints)
            .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection)
            .def("GetPalmDirection",&PyRobotBase::PyManipulator::GetPalmDirection)
            .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,args("body"))
            .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints)
            .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices)
            .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks)
            .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks)
            .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,args("transform"))
            .def("CheckEndEffectorCollision",pCheckEndEffectorCollision2,args("transform","report"))
            .def("CheckIndependentCollision",pCheckIndependentCollision1)
            .def("CheckIndependentCollision",pCheckIndependentCollision2,args("report"))
            ;

        class_<PyRobotBase::PyAttachedSensor, boost::shared_ptr<PyRobotBase::PyAttachedSensor> >("AttachedSensor", no_init)
            .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor)
            .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink)
            .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform)
            .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform)
            .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot)
            .def("GetName",&PyRobotBase::PyAttachedSensor::GetName)
            .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform,args("transform"))
            ;

        class_<PyRobotBase::PyGrabbed, boost::shared_ptr<PyRobotBase::PyGrabbed> >("Grabbed",no_init)
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

    class_<PyPlannerBase, boost::shared_ptr<PyPlannerBase>, bases<PyInterfaceBase> >("Planner", no_init);
    class_<PySensorSystemBase, boost::shared_ptr<PySensorSystemBase>, bases<PyInterfaceBase> >("SensorSystem", no_init);
    class_<PyTrajectoryBase, boost::shared_ptr<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", no_init);
    class_<PyControllerBase, boost::shared_ptr<PyControllerBase>, bases<PyInterfaceBase> >("Controller", no_init)
        .def("Init",&PyControllerBase::Init)
        .def("Reset",&PyControllerBase::Reset)
        .def("SetDesired",&PyControllerBase::SetDesired)
        .def("SetPath",&PyControllerBase::SetPath)
        .def("SimulationStep",&PyControllerBase::SimulationStep)
        .def("IsDone",&PyControllerBase::IsDone)
        .def("GetTime",&PyControllerBase::GetTime)
        .def("GetVelocity",&PyControllerBase::GetVelocity)
        .def("GetTorque",&PyControllerBase::GetTorque)
        ;
    class_<PyProblemInstance, boost::shared_ptr<PyProblemInstance>, bases<PyInterfaceBase> >("Problem", no_init)
        .def("SimulationStep",&PyProblemInstance::SimulationStep)
        ;
    class_<PyIkSolverBase, boost::shared_ptr<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", no_init);

    object (PyPhysicsEngineBase::*SetPhysicsOptions1)(const string&) = &PyPhysicsEngineBase::SetPhysicsOptions;
    bool (PyPhysicsEngineBase::*SetPhysicsOptions2)(int) = &PyPhysicsEngineBase::SetPhysicsOptions;
    bool (PyPhysicsEngineBase::*SetBodyVelocity1)(PyKinBodyPtr, object, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    bool (PyPhysicsEngineBase::*SetBodyVelocity2)(PyKinBodyPtr, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    class_<PyPhysicsEngineBase, boost::shared_ptr<PyPhysicsEngineBase>, bases<PyInterfaceBase> >("PhysicsEngine", no_init)
        .def("GetPhysicsOptions",&PyPhysicsEngineBase::GetPhysicsOptions)
        .def("GetPhysicsOptions",SetPhysicsOptions1)
        .def("SetPhysicsOptions",SetPhysicsOptions2)
        .def("InitEnvironment",&PyPhysicsEngineBase::InitEnvironment)
        .def("DestroyEnvironment",&PyPhysicsEngineBase::DestroyEnvironment)
        .def("InitKinBody",&PyPhysicsEngineBase::InitKinBody)
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
        scope sensor = class_<PySensorBase, boost::shared_ptr<PySensorBase>, bases<PyInterfaceBase> >("Sensor", no_init)
            .def("GetSensorData",&PySensorBase::GetSensorData)
            .def("SetTransform",&PySensorBase::SetTransform)
            .def("GetTransform",&PySensorBase::GetTransform)
            .def("GetName",&PySensorBase::GetName)
            ;

        class_<PySensorBase::PySensorData, boost::shared_ptr<PySensorBase::PySensorData> >("SensorData",no_init)
            .def_readonly("type",&PySensorBase::PySensorData::type)
            ;
        class_<PySensorBase::PyLaserSensorData, boost::shared_ptr<PySensorBase::PyLaserSensorData>, bases<PySensorBase::PySensorData> >("LaserSensorData",no_init)
            .def_readonly("transform",&PySensorBase::PyLaserSensorData::transform)
            .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
            .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
            .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
            .def_readonly("id",&PySensorBase::PyLaserSensorData::id)
            ;
        class_<PySensorBase::PyCameraSensorData, boost::shared_ptr<PySensorBase::PyCameraSensorData>, bases<PySensorBase::PySensorData> >("CameraSensorData",no_init)
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

    class_<PyCollisionCheckerBase, boost::shared_ptr<PyCollisionCheckerBase>, bases<PyInterfaceBase> >("CollisionChecker", no_init)
        .def("SetCollisionOptions",&PyCollisionCheckerBase::SetCollisionOptions)
        .def("GetCollisionOptions",&PyCollisionCheckerBase::GetCollisionOptions)
        ;


    {
        scope viewer = class_<PyRaveViewerBase, boost::shared_ptr<PyRaveViewerBase>, bases<PyInterfaceBase> >("Viewer", no_init)
            .def("main",&PyRaveViewerBase::main)
            .def("quitmainloop",&PyRaveViewerBase::quitmainloop)
            .def("SetSize",&PyRaveViewerBase::ViewerSetSize)
            .def("Move",&PyRaveViewerBase::ViewerMove)
            .def("SetTitle",&PyRaveViewerBase::ViewerSetTitle)
            .def("LoadModel",&PyRaveViewerBase::LoadModel)
            .def("RegisterCallback",&PyRaveViewerBase::RegisterCallback)
            .def("EnvironmentSync",&PyRaveViewerBase::EnvironmentSync)
            ;

        enum_<RaveViewerBase::ViewerEvents>("ViewerEvents")
            .value("ItemSelection",RaveViewerBase::VE_ItemSelection)
            ;
    }

    bool (PyEnvironmentBase::*pcolb)(PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbr)(PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbb)(PyKinBodyPtr,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbbr)(PyKinBodyPtr, PyKinBodyPtr,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoll)(PyKinBody::PyLinkPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollr)(PyKinBody::PyLinkPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolll)(PyKinBody::PyLinkPtr,PyKinBody::PyLinkPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolllr)(PyKinBody::PyLinkPtr,PyKinBody::PyLinkPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollb)(PyKinBody::PyLinkPtr, PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcollbr)(PyKinBody::PyLinkPtr, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolle)(PyKinBody::PyLinkPtr,object,object) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoller)(PyKinBody::PyLinkPtr, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolbe)(PyKinBodyPtr,object,object) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolber)(PyKinBodyPtr, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolyb)(PyRay*,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolybr)(PyRay*, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoly)(PyRay*) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolyr)(PyRay*, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;

    void (PyEnvironmentBase::*LockPhysics1)(bool) = &PyEnvironmentBase::LockPhysics;
    void (PyEnvironmentBase::*LockPhysics2)(bool, float) = &PyEnvironmentBase::LockPhysics;

    object (PyEnvironmentBase::*drawplane1)(object, object, const boost::multi_array<float,2>&) = &PyEnvironmentBase::drawplane;
    object (PyEnvironmentBase::*drawplane2)(object, object, const boost::multi_array<float,3>&) = &PyEnvironmentBase::drawplane;

    {
        scope env = classenv
            .def("Reset",&PyEnvironmentBase::Reset)
            .def("Destroy",&PyEnvironmentBase::Destroy)
            .def("GetPluginInfo",&PyEnvironmentBase::GetPluginInfo)
            .def("GetLoadedInterfaces",&PyEnvironmentBase::GetLoadedInterfaces)
            .def("LoadPlugin",&PyEnvironmentBase::LoadPlugin,args("filename"))
            .def("CreateInterface", &PyEnvironmentBase::CreateInterface,args("type","name"))
            .def("CreateRobot", &PyEnvironmentBase::CreateRobot,args("name"))
            .def("CreatePlanner", &PyEnvironmentBase::CreatePlanner,args("name"))
            .def("CreateSensorSystem", &PyEnvironmentBase::CreateSensorSystem,args("name"))
            .def("CreateController", &PyEnvironmentBase::CreateController,args("name"))
            .def("CreateProblem", &PyEnvironmentBase::CreateProblem,args("name"))
            .def("CreateIkSolver", &PyEnvironmentBase::CreateIkSolver,args("name"))
            .def("CreatePhysicsEngine", &PyEnvironmentBase::CreatePhysicsEngine,args("name"))
            .def("CreateSensor", &PyEnvironmentBase::CreateSensor,args("name"))
            .def("CreateCollisionChecker", &PyEnvironmentBase::CreateCollisionChecker,args("name"))
            .def("CreateViewer", &PyEnvironmentBase::CreateViewer,args("name"))
            
            .def("CloneSelf",&PyEnvironmentBase::CloneSelf,args("options"))
            .def("SetCollisionChecker",&PyEnvironmentBase::SetCollisionChecker,args("collisionchecker"))
            .def("GetCollisionChecker",&PyEnvironmentBase::GetCollisionChecker)

            .def("CheckCollision",pcolb,args("body"))
            .def("CheckCollision",pcolbr,args("body","report"))
            .def("CheckCollision",pcolbb,args("body1","body2"))
            .def("CheckCollision",pcolbbr,args("body1","body2","report"))
            .def("CheckCollision",pcoll,args("link"))
            .def("CheckCollision",pcollr,args("link","report"))
            .def("CheckCollision",pcolll,args("link1","link2"))
            .def("CheckCollision",pcolllr,args("link1","link2","report"))
            .def("CheckCollision",pcollb,args("link","body"))
            .def("CheckCollision",pcollbr,args("link","body","report"))
            .def("CheckCollision",pcolle,args("link","bodyexcluded","linkexcluded"))
            .def("CheckCollision",pcoller,args("link","bodyexcluded","linkexcluded","report") )
            .def("CheckCollision",pcolbe,args("body","bodyexcluded","linkexcluded"))
            .def("CheckCollision",pcolber,args("body","bodyexcluded","linkexcluded","report"))
            .def("CheckCollision",pcolyb,args("ray","body"))
            .def("CheckCollision",pcolybr,args("ray","body","report"))
            .def("CheckCollision",pcoly,args("ray"))
            .def("CheckCollision",pcolyr,args("ray"))
            .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                 CheckCollisionRays_overloads(args("rays","body","front_facing_only"), 
                                              "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction+range."))
            .def("Load",&PyEnvironmentBase::Load,args("filename"))
            .def("Save",&PyEnvironmentBase::Save,args("filename"))
            .def("ReadRobotXMLFile",&PyEnvironmentBase::ReadRobotXMLFile,args("filename"))
            .def("ReadRobotXMLData",&PyEnvironmentBase::ReadRobotXMLData,args("data"))
            .def("ReadKinBodyXMLFile",&PyEnvironmentBase::ReadKinBodyXMLFile,args("filename"))
            .def("ReadKinBodyXMLData",&PyEnvironmentBase::ReadKinBodyXMLData,args("data"))
            .def("AddKinBody",&PyEnvironmentBase::AddKinBody,args("body"))
            .def("AddRobot",&PyEnvironmentBase::AddRobot,args("robot"))
            .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody,args("body"))
            .def("GetKinBody",&PyEnvironmentBase::GetKinBody,args("name"))
            .def("GetRobot",&PyEnvironmentBase::GetRobot,args("name"))
            .def("GetBodyFromNetworkId",&PyEnvironmentBase::GetBodyFromNetworkId )
            .def("CreateKinBody",&PyEnvironmentBase::CreateKinBody)
            .def("CreateTrajectory",&PyEnvironmentBase::CreateTrajectory,args("dof"))
            .def("LoadProblem",&PyEnvironmentBase::LoadProblem,args("problem","args"))
            .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem,args("prob"))
            .def("GetLoadedProblems",&PyEnvironmentBase::GetLoadedProblems)
            .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine,args("physics"))
            .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine)
            .def("RegisterCollisionCallback",&PyEnvironmentBase::RegisterCollisionCallback,args("callback"))
            .def("StepSimulation",&PyEnvironmentBase::StepSimulation,args("timestep"))
            .def("StartSimulation",&PyEnvironmentBase::StartSimulation,StartSimulation_overloads(args("timestep","realtime")))
            .def("StopSimulation",&PyEnvironmentBase::StopSimulation)
            .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime)
            .def("LockPhysics",LockPhysics1,args("lock"))
            .def("LockPhysics",LockPhysics2,args("lock","timeout"))
            .def("__enter__",&PyEnvironmentBase::__enter__)
            .def("__exit__",&PyEnvironmentBase::__exit__)
            .def("SetViewer",&PyEnvironmentBase::SetViewer,SetViewer_overloads(args("viewername","showviewer")))
            .def("GetViewer",&PyEnvironmentBase::GetViewer)
            .def("plot3",&PyEnvironmentBase::plot3,plot3_overloads(args("points","pointsize","colors","drawstyle")))
            .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,drawlinestrip_overloads(args("points","linewidth","colors","drawstyle")))
            .def("drawlinelist",&PyEnvironmentBase::drawlinelist,drawlinelist_overloads(args("points","linewidth","colors","drawstyle")))
            .def("drawarrow",&PyEnvironmentBase::drawarrow,drawarrow_overloads(args("p1","p2","linewidth","color")))
            .def("drawbox",&PyEnvironmentBase::drawbox,drawbox_overloads(args("pos","extents","color")))
            .def("drawplane",drawplane1,args("transform","extents","texture"))
            .def("drawplane",drawplane2,args("transform","extents","texture"))
            .def("drawtrimesh",&PyEnvironmentBase::drawtrimesh,drawtrimesh_overloads(args("points","indices","colors")))
            .def("SetCamera",&PyEnvironmentBase::SetCamera,args("transform"))
            .def("SetCameraLookAt",&PyEnvironmentBase::SetCameraLookAt,args("lookat","pos","up"))
            .def("GetCameraTransform",&PyEnvironmentBase::GetCameraTransform)
            .def("GetCameraImage",&PyEnvironmentBase::GetCameraImage,args("width","height","transform","K"))
            .def("GetRobots",&PyEnvironmentBase::GetRobots)
            .def("GetBodies",&PyEnvironmentBase::GetBodies)
            .def("UpdatePublishedBodies",&PyEnvironmentBase::UpdatePublishedBodies)
            .def("Triangulate",&PyEnvironmentBase::Triangulate,args("body"))
            .def("TriangulateScene",&PyEnvironmentBase::TriangulateScene,args("options","name"))
            .def("SetDebugLevel",&PyEnvironmentBase::SetDebugLevel,args("level"))
            .def("GetDebugLevel",&PyEnvironmentBase::GetDebugLevel)
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
            .add_property("ReturnTransformQuaternions",GetReturnTransformQuaternions,SetReturnTransformQuaternions);
    }

    scope().attr("__version__") = boost::str(boost::format("%x.%x.%x")%OPENRAVE_VERSION_MAJOR%OPENRAVE_VERSION_MINOR%OPENRAVE_VERSION_PATCH);
    scope().attr("__author__") = "Rosen Diankov";
    scope().attr("__copyright__") = "Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)";
    scope().attr("__license__") = "Lesser GPL";

    def("quatFromAxisAngle",quatFromAxisAngle1, args("axis"), "Converts an axis-angle rotation into a quaternion");
    def("quatFromAxisAngle",quatFromAxisAngle2, args("axis","angle"), "Converts an axis-angle rotation into a quaternion");
    def("quatFromRotationMatrix",quatFromRotationMatrix, args("rotation"), "Converts the rotation of a matrix into a quaternion");
    def("axisAngleFromRotationMatrix",axisAngleFromRotationMatrix, args("rotation"), "Converts the rotation of a matrix into axis-angle representation");
    def("rotationMatrixFromQuat",rotationMatrixFromQuat, args("quat"), "Converts a quaternion to a 3x3 matrix");
    def("rotationMatrixFromQArray",rotationMatrixFromQArray,args("quatarray"),"Converts an array of quaternions to a list of 3x3 rotation matrices");
    def("matrixFromQuat",matrixFromQuat, args("quat"), "Converts a quaternion to a 4x4 affine matrix");
    def("rotationMatrixFromAxisAngle",rotationMatrixFromAxisAngle1, args("axis"), "Converts an axis-angle rotation to a 3x3 matrix");
    def("rotationMatrixFromAxisAngle",rotationMatrixFromAxisAngle2, args("axis","angle"), "Converts an axis-angle rotation to a 3x3 matrix");
    def("matrixFromAxisAngle",matrixFromAxisAngle1, args("axis"), "Converts an axis-angle rotation to a 4x4 affine matrix");
    def("matrixFromAxisAngle",matrixFromAxisAngle2, args("axis","angle"), "Converts an axis-angle rotation to a 4x4 affine matrix");
    def("matrixFromPose",matrixFromPose, args("pose"), "Converts a 7 element quaterion+translation transform to a 4x4 matrix");
    def("matrixFromPoses",matrixFromPoses, args("poses"), "Converts a Nx7 element quaterion+translation array to a 4x4 matrices");
    def("poseFromMatrix",poseFromMatrix, args("transform"), "Converts a 4x4 matrix to a 7 element quaternion+translation representation");
    def("poseFromMatrices",poseFromMatrices, args("transforms"), "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation");
    def("invertPoses",invertPoses,args("poses"), "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components");
    def("quatRotateDirection",quatRotateDirection,args("sourcedir,targetdir"),"Returns the minimal quaternion rotation that rotates sourcedir into targetdir");
    def("matrixSerialization",matrixSerialization,args("matrix"),"Serializes a transformation into a string representing a 3x4 matrix");
    def("poseSerialization",poseSerialization, args("pose"), "Serializes a transformation into a string representing a quaternion with translation");
    def("openravepyCompilerVersion",openravepyCompilerVersion,"Returns the compiler version that openravepy_int was compiled with");
}
