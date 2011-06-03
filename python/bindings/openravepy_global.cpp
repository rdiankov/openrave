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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

PyRay::PyRay(object newpos, object newdir)
{
    r.pos = ExtractVector3(newpos);
    r.dir = ExtractVector3(newdir);
}

object PyRay::dir() { return toPyVector3(r.dir); }
object PyRay::pos() { return toPyVector3(r.pos); }

string PyRay::__repr__() { return boost::str(boost::format("<Ray([%f,%f,%f],[%f,%f,%f])>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z); }
string PyRay::__str__() { return boost::str(boost::format("<%f %f %f %f %f %f>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z); }

object toPyRay(const RAY& r)
{
    return object(boost::shared_ptr<PyRay>(new PyRay(r)));
}

RAY ExtractRay(object o)
{
    extract<boost::shared_ptr<PyRay> > pyray(o);
    return ((boost::shared_ptr<PyRay>)pyray)->r;
}

bool ExtractRay(object o, RAY& ray)
{
    extract<boost::shared_ptr<PyRay> > pyray(o);
    if( pyray.check() ) {
        ray = ((boost::shared_ptr<PyRay>)pyray)->r;
        return true;
    }
    return false;
}

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

    virtual string __repr__() { return boost::str(boost::format("<AABB([%f,%f,%f],[%f,%f,%f])>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z); }
    virtual string __str__() { return boost::str(boost::format("<%f %f %f %f %f %f>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z); }

    AABB ab;
};

object toPyAABB(const AABB& ab)
{
    return object(boost::shared_ptr<PyAABB>(new PyAABB(ab)));
}

class AABB_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyAABB& ab)
    {
        return boost::python::make_tuple(toPyVector3(ab.ab.pos),toPyVector3(ab.ab.extents));
    }
};

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

    string __str__() { return boost::str(boost::format("<trimesh: verts %d, tris=%d>")%len(vertices)%len(indices)); }
            
    object vertices,indices;
};

bool ExtractTriMesh(object o, KinBody::Link::TRIMESH& mesh)
{
    extract<boost::shared_ptr<PyTriMesh> > pytrimesh(o);
    if( pytrimesh.check() ) {
        ((boost::shared_ptr<PyTriMesh>)pytrimesh)->GetTriMesh(mesh);
        return true;
    }
    return false;
}

object toPyTriMesh(const KinBody::Link::TRIMESH& mesh)
{
    return object(boost::shared_ptr<PyTriMesh>(new PyTriMesh(mesh)));
}

class TriMesh_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyTriMesh& r)
    {
        return boost::python::make_tuple(r.vertices,r.indices);
    }
};

class PyIkParameterization
{
public:
    PyIkParameterization() {}
    PyIkParameterization(const string& s) {
        stringstream ss(s);
        ss >> _param;
    }
    PyIkParameterization(object o, IkParameterization::Type type)
    {
        switch(type) {
        case IkParameterization::Type_Transform6D: SetTransform6D(o); break;
        case IkParameterization::Type_Rotation3D: SetRotation3D(o); break;
        case IkParameterization::Type_Translation3D: SetTranslation3D(o); break;
        case IkParameterization::Type_Direction3D: SetDirection3D(o); break;
        case IkParameterization::Type_Ray4D: SetRay4D(extract<boost::shared_ptr<PyRay> >(o)); break;
        case IkParameterization::Type_Lookat3D: SetLookat3D(o); break;
        case IkParameterization::Type_TranslationDirection5D: SetTranslationDirection5D(extract<boost::shared_ptr<PyRay> >(o)); break;
        case IkParameterization::Type_TranslationXY2D: SetTranslationXY2D(o); break;
        case IkParameterization::Type_TranslationXYOrientation3D: SetTranslationXYOrientation3D(o); break;
        case IkParameterization::Type_TranslationLocalGlobal6D: SetTranslationLocalGlobal6D(o[0],o[1]); break;
        default: throw openrave_exception(boost::str(boost::format("incorrect ik parameterization type %d")%type));
        }
    }
    PyIkParameterization(const IkParameterization& ikparam) : _param(ikparam) {}

    IkParameterization::Type GetType() { return _param.GetType(); }

    void SetTransform6D(object o) { _param.SetTransform6D(ExtractTransform(o)); }
    void SetRotation3D(object o) { _param.SetRotation3D(ExtractVector4(o)); }
    void SetTranslation3D(object o) { _param.SetTranslation3D(ExtractVector3(o)); }
    void SetDirection3D(object o) { _param.SetDirection3D(ExtractVector3(o)); }
    void SetRay4D(boost::shared_ptr<PyRay> ray) { _param.SetRay4D(ray->r); }
    void SetLookat3D(object o) { _param.SetLookat3D(ExtractVector3(o)); }
    void SetTranslationDirection5D(boost::shared_ptr<PyRay> ray) { _param.SetTranslationDirection5D(ray->r); }
    void SetTranslationXY2D(object o) { _param.SetTranslationXY2D(ExtractVector2(o)); }
    void SetTranslationXYOrientation3D(object o) { _param.SetTranslationXYOrientation3D(ExtractVector3(o)); }
    void SetTranslationLocalGlobal6D(object olocaltrans, object otrans) { _param.SetTranslationLocalGlobal6D(ExtractVector3(olocaltrans),ExtractVector3(otrans)); }

    object GetTransform6D() { return ReturnTransform(_param.GetTransform6D()); }
    object GetRotation3D() { return toPyVector4(_param.GetRotation3D()); }
    object GetTranslation3D() { return toPyVector3(_param.GetTranslation3D()); }
    object GetDirection3D() { return toPyVector3(_param.GetDirection3D()); }
    PyRay GetRay4D() { return PyRay(_param.GetRay4D()); }
    object GetLookat3D() { return toPyVector3(_param.GetLookat3D()); }
    PyRay GetTranslationDirection5D() { return PyRay(_param.GetTranslationDirection5D()); }
    object GetTranslationXY2D() { return toPyVector2(_param.GetTranslationXY2D()); }
    object GetTranslationXYOrientation3D() { return toPyVector3(_param.GetTranslationXYOrientation3D()); }
    object GetTranslationLocalGlobal6D() { return boost::python::make_tuple(toPyVector3(_param.GetTranslationLocalGlobal6D().first),toPyVector3(_param.GetTranslationLocalGlobal6D().second)); }

    IkParameterization _param;

    string __repr__() {
        stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1); /// have to do this or otherwise precision gets lost
        ss << _param;
        return boost::str(boost::format("<IkParameterization('%s')>")%ss.str());
    }
    string __str__() {
        stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1); /// have to do this or otherwise precision gets lost
        ss << _param;
        return ss.str();
    }
};

bool ExtractIkParameterization(object o, IkParameterization& ikparam) {
    extract<boost::shared_ptr<PyIkParameterization> > pyikparam(o);
    if( pyikparam.check() ) {
        ikparam = ((boost::shared_ptr<PyIkParameterization>)pyikparam)->_param;
        return true;
    }
    return false;
}


object toPyIkParameterization(const IkParameterization& ikparam)
{
    return object(boost::shared_ptr<PyIkParameterization>(new PyIkParameterization(ikparam)));
}

class IkParameterization_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyIkParameterization& r)
    {
        return boost::python::make_tuple(r._param.GetTransform6D(),r._param.GetType());
    }
};

namespace openravepy {

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

void raveLog(const string& s, DebugLevel level)
{
    if( s.size() > 0 ) {
        RavePrintfA(s,level);
    }
}

void raveLogFatal(const string& s)
{
    raveLog(s,Level_Verbose);
}
void raveLogError(const string& s)
{
    raveLog(s,Level_Error);
}
void raveLogWarn(const string& s)
{
    raveLog(s,Level_Warn);
}
void raveLogInfo(const string& s)
{
    raveLog(s,Level_Info);
}
void raveLogDebug(const string& s)
{
    raveLog(s,Level_Debug);
}
void raveLogVerbose(const string& s)
{
    raveLog(s,Level_Verbose);
}

object RaveGetPluginInfo()
{
    boost::python::list plugins;
    std::list< std::pair<std::string, PLUGININFO> > listplugins;
    OpenRAVE::RaveGetPluginInfo(listplugins);
    FOREACH(itplugin, listplugins) {
        plugins.append(boost::python::make_tuple(itplugin->first,object(boost::shared_ptr<PyPluginInfo>(new PyPluginInfo(itplugin->second)))));
    }
    return plugins;
}

object RaveGetLoadedInterfaces()
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    boost::python::list ointerfacenames;
    FOREACHC(it, interfacenames) {
        boost::python::list names;
        FOREACHC(itname,it->second) {
            names.append(*itname);
        }
        ointerfacenames.append(boost::python::make_tuple(it->first,names));
    }
    return ointerfacenames;
}

object quatFromAxisAngle1(object oaxis)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis)));
}

object quatFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis),angle));
}

object quatFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    return toPyVector4(quatFromMatrix(t));
}

object quatSlerp(object q1, object q2, dReal t)
{
    return toPyVector4(quatSlerp(ExtractVector4(q1),ExtractVector4(q2),t));
}

object axisAngleFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    return toPyVector3(axisAngleFromMatrix(t));
}

object axisAngleFromQuat(object oquat)
{
    return toPyVector3(axisAngleFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQuat(object oquat)
{
    return toPyArrayRotation(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQArray(object qarray)
{
    boost::python::list orots;
    int N = len(qarray);
    for(int i = 0; i < N; ++i) {
        orots.append(rotationMatrixFromQuat(qarray[i]));
    }
    return orots;
}

object matrixFromQuat(object oquat)
{
    return toPyArray(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromAxisAngle1(object oaxis)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object rotationMatrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromAxisAngle1(object oaxis)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object matrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromPose(object opose)
{
    return toPyArray(TransformMatrix(ExtractTransformType<dReal>(opose)));
}

object matrixFromPoses(object oposes)
{
    boost::python::list omatrices;
    int N = len(oposes);
    for(int i = 0; i < N; ++i) {
        omatrices.append(matrixFromPose(oposes[i]));
    }
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
    if( N == 0 ) {
        return static_cast<numeric::array>(handle<>());
    }
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
    if( N == 0 ) {
        return numeric::array(boost::python::list());
    }
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
    return toPyVector4(quatRotateDirection(ExtractVector3(source), ExtractVector3(target)));
}

object normalizeAxisRotation(object axis, object quat)
{
    std::pair<dReal, Vector > res = normalizeAxisRotation(ExtractVector3(axis), ExtractVector4(quat));
    return boost::python::make_tuple(res.first,toPyVector4(res.second));
}

object quatMult(object oquat1, object oquat2)
{
    return toPyVector4(quatMultiply(ExtractVector4(oquat1),ExtractVector4(oquat2)));
}

object poseMult(object opose1, object opose2)
{
    return toPyArray(ExtractTransformType<dReal>(opose1)*ExtractTransformType<dReal>(opose2));
}

object poseTransformPoints(object opose, object opoints)
{
    Transform t = ExtractTransformType<dReal>(opose);
    int N = len(opoints);
    npy_intp dims[] = {N,3};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
    for(int i = 0; i < N; ++i, ptrans += 3) {
        Vector newpoint = t*ExtractVector3(opoints[i]);
        ptrans[0] = newpoint.x; ptrans[1] = newpoint.y; ptrans[2] = newpoint.z;
    }
    return static_cast<numeric::array>(handle<>(pytrans));
}

object transformLookat(object olookat, object ocamerapos, object ocameraup)
{
    return toPyArray(transformLookat(ExtractVector3(olookat),ExtractVector3(ocamerapos),ExtractVector3(ocameraup)));
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

BOOST_PYTHON_FUNCTION_OVERLOADS(RaveInitialize_overloads, RaveInitialize, 0, 2)

void init_openravepy_global()
{
    enum_<DebugLevel>("DebugLevel" DOXY_ENUM(DebugLevel))
        .value("Fatal",Level_Fatal)
        .value("Error",Level_Error)
        .value("Warn",Level_Warn)
        .value("Info",Level_Info)
        .value("Debug",Level_Debug)
        .value("Verbose",Level_Verbose)
        ;
    enum_<SerializationOptions>("SerializationOptions" DOXY_ENUM(SerializationOptions))
        .value("Kinematics",SO_Kinematics)
        .value("Dynamics",SO_Dynamics)
        .value("BodyState",SO_BodyState)
        .value("NamesAndFiles",SO_NamesAndFiles)
        .value("RobotManipulators",SO_RobotManipulators)
        .value("RobotSensors",SO_RobotSensors)
        .value("Geometry",SO_Geometry)
        ;
    enum_<InterfaceType>("InterfaceType" DOXY_ENUM(InterfaceType))
        .value(RaveGetInterfaceName(PT_Planner).c_str(),PT_Planner)
        .value(RaveGetInterfaceName(PT_Robot).c_str(),PT_Robot)
        .value(RaveGetInterfaceName(PT_SensorSystem).c_str(),PT_SensorSystem)
        .value(RaveGetInterfaceName(PT_Controller).c_str(),PT_Controller)
        .value(RaveGetInterfaceName(PT_ProblemInstance).c_str(),PT_ProblemInstance)
        .value(RaveGetInterfaceName(PT_IkSolver).c_str(),PT_IkSolver)
        .value(RaveGetInterfaceName(PT_KinBody).c_str(),PT_KinBody)
        .value(RaveGetInterfaceName(PT_PhysicsEngine).c_str(),PT_PhysicsEngine)
        .value(RaveGetInterfaceName(PT_Sensor).c_str(),PT_Sensor)
        .value(RaveGetInterfaceName(PT_CollisionChecker).c_str(),PT_CollisionChecker)
        .value(RaveGetInterfaceName(PT_Trajectory).c_str(),PT_Trajectory)
        .value(RaveGetInterfaceName(PT_Viewer).c_str(),PT_Viewer)
        .value(RaveGetInterfaceName(PT_SpaceSampler).c_str(),PT_SpaceSampler)
        ;
    enum_<CollisionOptions>("CollisionOptions" DOXY_ENUM(CollisionOptions))
        .value("Distance",CO_Distance)
        .value("UseTolerance",CO_UseTolerance)
        .value("Contacts",CO_Contacts)
        .value("RayAnyHit",CO_RayAnyHit)
        .value("ActiveDOFs",CO_ActiveDOFs);
        ;
    enum_<CollisionAction>("CollisionAction" DOXY_ENUM(CollisionAction))
        .value("DefaultAction",CA_DefaultAction)
        .value("Ignore",CA_Ignore)
        ;
    enum_<CloningOptions>("CloningOptions" DOXY_ENUM(CloningOptions))
        .value("Bodies",Clone_Bodies)
        .value("Viewer",Clone_Viewer)
        .value("Simulation",Clone_Simulation)
        .value("RealControllers",Clone_RealControllers)
        .value("Sensors",Clone_Sensors)
        ;
    enum_<PhysicsEngineOptions>("PhysicsEngineOptions" DOXY_ENUM(PhysicsEngineOptions))
        .value("SelfCollisions",PEO_SelfCollisions)
        ;
    enum_<IkFilterOptions>("IkFilterOptions" DOXY_ENUM(IkFilterOptions))
        .value("CheckEnvCollisions",IKFO_CheckEnvCollisions)
        .value("IgnoreSelfCollisions",IKFO_IgnoreSelfCollisions)
        .value("IgnoreJointLimits",IKFO_IgnoreJointLimits)
        .value("IgnoreCustomFilter",IKFO_IgnoreCustomFilter)
        ;
    enum_<IkFilterReturn>("IkFilterReturn" DOXY_ENUM(IkFilterReturn))
        .value("Success",IKFR_Success)
        .value("Reject",IKFR_Reject)
        .value("Quit",IKFR_Quit)
        ;
    enum_<IntervalType>("Interval" DOXY_ENUM(IntervalType))
        .value("Open",IT_Open)
        .value("OpenStart",IT_OpenStart)
        .value("OpenEnd",IT_OpenEnd)
        .value("Closed",IT_Closed)
        ;
    object iktype = enum_<IkParameterization::Type>("IkParameterizationType" DOXY_ENUM(IkParameterization::Type))
        .value("Transform6D",IkParameterization::Type_Transform6D)
        .value("Rotation3D",IkParameterization::Type_Rotation3D)
        .value("Translation3D",IkParameterization::Type_Translation3D)
        .value("Direction3D",IkParameterization::Type_Direction3D)
        .value("Ray4D",IkParameterization::Type_Ray4D)
        .value("Lookat3D",IkParameterization::Type_Lookat3D)
        .value("TranslationDirection5D",IkParameterization::Type_TranslationDirection5D)
        .value("TranslationXY2D",IkParameterization::Type_TranslationXY2D)
        .value("TranslationXYOrientation3D",IkParameterization::Type_TranslationXYOrientation3D)
        .value("TranslationLocalGlobal6D",IkParameterization::Type_TranslationLocalGlobal6D)
        ;

    class_<UserData, UserDataPtr >("UserData", DOXY_CLASS(UserData))
        ;

    class_< boost::shared_ptr< void > >("VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");

    class_<PyGraphHandle, boost::shared_ptr<PyGraphHandle> >("GraphHandle", DOXY_CLASS(GraphHandle), no_init)
        .def("SetTransform",&PyGraphHandle::SetTransform,DOXY_FN(GraphHandle,SetTransform))
        .def("SetShow",&PyGraphHandle::SetShow,DOXY_FN(GraphHandle,SetShow))
        ;
    class_<PyRay, boost::shared_ptr<PyRay> >("Ray", DOXY_CLASS(geometry::ray))
        .def(init<object,object>(args("pos","dir")))
        .def("dir",&PyRay::dir)
        .def("pos",&PyRay::pos)
        .def("__str__",&PyRay::__str__)
        .def("__repr__",&PyRay::__repr__)
        .def_pickle(Ray_pickle_suite())
        ;
    class_<PyAABB, boost::shared_ptr<PyAABB> >("AABB", DOXY_CLASS(geometry::aabb))
        .def(init<object,object>(args("pos","extents")))
        .def("extents",&PyAABB::extents)
        .def("pos",&PyAABB::pos)
        .def("__str__",&PyAABB::__str__)
        .def("__repr__",&PyAABB::__repr__)
        .def_pickle(AABB_pickle_suite())
        ;
    class_<PyTriMesh, boost::shared_ptr<PyTriMesh> >("TriMesh", DOXY_CLASS(KinBody::Link::TRIMESH))
        .def(init<object,object>(args("vertices","indices")))
        .def_readwrite("vertices",&PyTriMesh::vertices)
        .def_readwrite("indices",&PyTriMesh::indices)
        .def("__str__",&PyTriMesh::__str__)
        .def_pickle(TriMesh_pickle_suite())
        ;
    class_<InterfaceBase, InterfaceBasePtr, boost::noncopyable >("InterfaceBase", DOXY_CLASS(InterfaceBase), no_init)
        ;

    class_<PyPluginInfo, boost::shared_ptr<PyPluginInfo> >("PluginInfo", DOXY_CLASS(PLUGININFO),no_init)
        .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
        .def_readonly("version",&PyPluginInfo::version)
        ;

    {
        int (*getdof1)(IkParameterization::Type) = &IkParameterization::GetDOF;
        int (*getnumberofvalues1)(IkParameterization::Type) = &IkParameterization::GetNumberOfValues;
        scope ikparameterization = class_<PyIkParameterization, boost::shared_ptr<PyIkParameterization> >("IkParameterization", DOXY_CLASS(IkParameterization))
            .def(init<object,IkParameterization::Type>(args("primitive","type")))
            .def(init<string>(args("str")))
            .def("GetType",&PyIkParameterization::GetType, DOXY_FN(IkParameterization,GetType))
            .def("SetTransform6D",&PyIkParameterization::SetTransform6D,args("transform"), DOXY_FN(IkParameterization,SetTransform6D))
            .def("SetRotation3D",&PyIkParameterization::SetRotation3D,args("quat"), DOXY_FN(IkParameterization,SetRotation3D))
            .def("SetTranslation3D",&PyIkParameterization::SetTranslation3D,args("pos"), DOXY_FN(IkParameterization,SetTranslation3D))
            .def("SetDirection3D",&PyIkParameterization::SetDirection3D,args("dir"), DOXY_FN(IkParameterization,SetDirection3D))
            .def("SetRay4D",&PyIkParameterization::SetRay4D,args("quat"), DOXY_FN(IkParameterization,SetRay4D))
            .def("SetLookat3D",&PyIkParameterization::SetLookat3D,args("pos"), DOXY_FN(IkParameterization,SetLookat3D))
            .def("SetTranslationDirection5D",&PyIkParameterization::SetTranslationDirection5D,args("quat"), DOXY_FN(IkParameterization,SetTranslationDirection5D))
            .def("SetTranslationXY2D",&PyIkParameterization::SetTranslationXY2D,args("pos"), DOXY_FN(IkParameterization,SetTranslationXY2D))
            .def("SetTranslationXYOrientation3D",&PyIkParameterization::SetTranslationXYOrientation3D,args("posangle"), DOXY_FN(IkParameterization,SetTranslationXYOrientation3D))
            .def("SetTranslationLocalGlobal6D",&PyIkParameterization::SetTranslationLocalGlobal6D,args("localpos","pos"), DOXY_FN(IkParameterization,SetTranslationLocalGlobal6D))
            .def("GetTransform6D",&PyIkParameterization::GetTransform6D, DOXY_FN(IkParameterization,GetTransform6D))
            .def("GetRotation3D",&PyIkParameterization::GetRotation3D, DOXY_FN(IkParameterization,GetRotation3D))
            .def("GetTranslation3D",&PyIkParameterization::GetTranslation3D, DOXY_FN(IkParameterization,GetTranslation3D))
            .def("GetDirection3D",&PyIkParameterization::GetDirection3D, DOXY_FN(IkParameterization,GetDirection3D))
            .def("GetRay4D",&PyIkParameterization::GetRay4D, DOXY_FN(IkParameterization,GetRay4D))
            .def("GetLookat3D",&PyIkParameterization::GetLookat3D, DOXY_FN(IkParameterization,GetLookat3D))
            .def("GetTranslationDirection5D",&PyIkParameterization::GetTranslationDirection5D, DOXY_FN(IkParameterization,GetTranslationDirection5D))
            .def("GetTranslationXY2D",&PyIkParameterization::GetTranslationXY2D, DOXY_FN(IkParameterization,GetTranslationXY2D))
            .def("GetTranslationXYOrientation3D",&PyIkParameterization::GetTranslationXYOrientation3D, DOXY_FN(IkParameterization,GetTranslationXYOrientation3D))
            .def("GetTranslationLocalGlobal6D",&PyIkParameterization::GetTranslationLocalGlobal6D, DOXY_FN(IkParameterization,GetTranslationLocalGlobal6D))
            .def("GetDOF", getdof1,args("type"), DOXY_FN(IkParameterization,GetDOF))
            .staticmethod("GetDOF")
            .def("GetNumberOfValues",getnumberofvalues1,args("type"), DOXY_FN(IkParameterization,GetNumberOfValues))
            .staticmethod("GetNumberOfValues")
            .def_pickle(IkParameterization_pickle_suite())

            // deprecated
            .def("SetTransform",&PyIkParameterization::SetTransform6D,args("transform"), DOXY_FN(IkParameterization,SetTransform6D))
            .def("SetRotation",&PyIkParameterization::SetRotation3D,args("quat"), DOXY_FN(IkParameterization,SetRotation3D))
            .def("SetTranslation",&PyIkParameterization::SetTranslation3D,args("pos"), DOXY_FN(IkParameterization,SetTranslation3D))
            .def("SetDirection",&PyIkParameterization::SetDirection3D,args("dir"), DOXY_FN(IkParameterization,SetDirection3D))
            .def("SetRay",&PyIkParameterization::SetRay4D,args("quat"), DOXY_FN(IkParameterization,SetRay4D))
            .def("SetLookat",&PyIkParameterization::SetLookat3D,args("pos"), DOXY_FN(IkParameterization,SetLookat3D))
            .def("SetTranslationDirection",&PyIkParameterization::SetTranslationDirection5D,args("quat"), DOXY_FN(IkParameterization,SetTranslationDirection5D))
            .def("GetTransform",&PyIkParameterization::GetTransform6D, DOXY_FN(IkParameterization,GetTransform6D))
            .def("GetRotation",&PyIkParameterization::GetRotation3D, DOXY_FN(IkParameterization,GetRotation3D))
            .def("GetTranslation",&PyIkParameterization::GetTranslation3D, DOXY_FN(IkParameterization,GetTranslation3D))
            .def("GetDirection",&PyIkParameterization::GetDirection3D, DOXY_FN(IkParameterization,GetDirection3D))
            .def("GetRay",&PyIkParameterization::GetRay4D, DOXY_FN(IkParameterization,GetRay4D))
            .def("GetLookat",&PyIkParameterization::GetLookat3D, DOXY_FN(IkParameterization,GetLookat3D))
            .def("GetTranslationDirection",&PyIkParameterization::GetTranslationDirection5D, DOXY_FN(IkParameterization,GetTranslationDirection5D))
            .def("__str__",&PyIkParameterization::__str__)
            .def("__repr__",&PyIkParameterization::__repr__)
            ;
        ikparameterization.attr("Type") = iktype;
    }

    def("RaveSetDebugLevel",OpenRAVE::RaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
    def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
    def("RaveFindDatabaseFile",OpenRAVE::RaveFindDatabaseFile,DOXY_FN1(RaveFindDatabaseFile));
    def("RaveLogFatal",openravepy::raveLogFatal,args("log"),"Send a fatal log to the openrave system");
    def("RaveLogError",openravepy::raveLogError,args("log"),"Send an error log to the openrave system");
    def("RaveLogWarn",openravepy::raveLogWarn,args("log"),"Send a warn log to the openrave system");
    def("RaveLogInfo",openravepy::raveLogInfo,args("log"),"Send an info log to the openrave system");
    def("RaveLogDebug",openravepy::raveLogDebug,args("log"),"Send a debug log to the openrave system");
    def("RaveLogVerbose",openravepy::raveLogVerbose,args("log"),"Send a verbose log to the openrave system");
    def("RaveLog",openravepy::raveLog,args("log","level"),"Send a log to the openrave system with excplicit level");
    def("RaveInitialize",RaveInitialize,RaveInitialize_overloads(args("load_all_plugins","level"),DOXY_FN1(RaveInitialize)));
    def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
    def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
    def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(raveGetLoadedInterfaces));
    def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
    def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin,args("filename"),DOXY_FN1(RaveLoadPlugins));
    def("RaveHasInterface",OpenRAVE::RaveHasInterface,args("type","name"),DOXY_FN1(RaveHasInterface));
    def("RaveGlobalState",OpenRAVE::RaveGlobalState,DOXY_FN1(RaveGlobalState));

    def("raveSetDebugLevel",OpenRAVE::RaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
    def("raveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    def("raveLogFatal",openravepy::raveLogFatal,args("log"),"Send a fatal log to the openrave system");
    def("raveLogError",openravepy::raveLogError,args("log"),"Send an error log to the openrave system");
    def("raveLogWarn",openravepy::raveLogWarn,args("log"),"Send a warn log to the openrave system");
    def("raveLogInfo",openravepy::raveLogInfo,args("log"),"Send an info log to the openrave system");
    def("raveLogDebug",openravepy::raveLogDebug,args("log"),"Send a debug log to the openrave system");
    def("raveLogVerbose",openravepy::raveLogVerbose,args("log"),"Send a verbose log to the openrave system");
    def("raveLog",openravepy::raveLog,args("log","level"),"Send a log to the openrave system with excplicit level");

    def("quatFromAxisAngle",openravepy::quatFromAxisAngle1, args("axisangle"), DOXY_FN1(quatFromAxisAngle "const RaveVector"));
    def("quatFromAxisAngle",openravepy::quatFromAxisAngle2, args("axis","angle"), DOXY_FN1(quatFromAxisAngle "const RaveVector; T"));
    def("quatFromRotationMatrix",openravepy::quatFromRotationMatrix, args("rotation"), DOXY_FN1(quatFromMatrix "const RaveTransform"));
    def("quatSlerp",openravepy::quatSlerp, args("quat0","quat1","t"), DOXY_FN1(quatSlerp "const RaveVector; const RaveVector; T"));
    def("axisAngleFromRotationMatrix",openravepy::axisAngleFromRotationMatrix, args("rotation"), DOXY_FN1(axisAngleFromMatrix "const RaveTransformMatrix"));
    def("axisAngleFromQuat",openravepy::axisAngleFromQuat, args("quat"), DOXY_FN1(axisAngleFromQuat "const RaveVector"));
    def("rotationMatrixFromQuat",openravepy::rotationMatrixFromQuat, args("quat"), DOXY_FN1(matrixFromQuat "const RaveVector"));
    def("rotationMatrixFromQArray",openravepy::rotationMatrixFromQArray,args("quatarray"),"Converts an array of quaternions to a list of 3x3 rotation matrices.\n\n:param quatarray: nx4 array\n");
    def("matrixFromQuat",openravepy::matrixFromQuat, args("quat"), "Converts a quaternion to a 4x4 affine matrix.\n\n:param quat: 4 values\n");
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle1, args("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle2, args("axis","angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle1, args("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle2, args("axis","angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    def("matrixFromPose",openravepy::matrixFromPose, args("pose"), "Converts a 7 element quaterion+translation transform to a 4x4 matrix.\n\n:param pose: 7 values\n");
    def("matrixFromPoses",openravepy::matrixFromPoses, args("poses"), "Converts a Nx7 element quaterion+translation array to a 4x4 matrices.\n\n:param poses: nx7 array\n");
    def("poseFromMatrix",openravepy::poseFromMatrix, args("transform"), "Converts a 4x4 matrix to a 7 element quaternion+translation representation.\n\n:param transform: 3x4 or 4x4 affine matrix\n");
    def("poseFromMatrices",openravepy::poseFromMatrices, args("transforms"), "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation.\n\n:param transforms: list of 3x4 or 4x4 affine matrices\n");
    def("invertPoses",openravepy::invertPoses,args("poses"), "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
    def("quatRotateDirection",openravepy::quatRotateDirection,args("sourcedir,targetdir"), DOXY_FN1(quatRotateDirection));
    def("quatMult",openravepy::quatMult,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("quatMultiply",openravepy::quatMult,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("poseMult",openravepy::poseMult,args("pose1","pose2"),"multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
    def("poseTransformPoints",openravepy::poseTransformPoints,args("pose","points"),"left-transforms a set of points by a pose transformation.\n\n:param pose: 7 values\n\n:param points: Nx3 values");
    def("transformLookat",openravepy::transformLookat,args("lookat","camerapos","cameraup"),"Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
    def("matrixSerialization",openravepy::matrixSerialization,args("transform"),"Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
    def("poseSerialization",openravepy::poseSerialization, args("pose"), "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
    def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion,"Returns the compiler version that openravepy_int was compiled with");
    def("normalizeAxisRotation",openravepy::normalizeAxisRotation,args("axis","quat"),DOXY_FN1(normalizeAxisRotation));
}

} // end namespace openravepy
