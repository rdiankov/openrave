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

#include <openrave/planningutils.h>
#include <openrave/xmlreaders.h>
#include <openrave/utils.h>

namespace openravepy {

PyRay::PyRay(object newpos, object newdir)
{
    r.pos = ExtractVector3(newpos);
    r.dir = ExtractVector3(newdir);
}

object PyRay::dir() {
    return toPyVector3(r.dir);
}
object PyRay::pos() {
    return toPyVector3(r.pos);
}

string PyRay::__repr__() {
    return boost::str(boost::format("<Ray([%f,%f,%f],[%f,%f,%f])>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
string PyRay::__str__() {
    return boost::str(boost::format("<%f %f %f %f %f %f>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
object PyRay::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

class PyXMLReadable
{
public:
    PyXMLReadable(XMLReadablePtr xmlreadable) : _xmlreadable(xmlreadable) {
    }
    virtual ~PyXMLReadable() {
    }
    std::string GetXMLId() const {
        return _xmlreadable->GetXMLId();
    }
    object Serialize(int options=0)
    {
        std::string xmlid;
        OpenRAVE::xmlreaders::StreamXMLWriter writer(xmlid);
        _xmlreadable->Serialize(OpenRAVE::xmlreaders::StreamXMLWriterPtr(&writer,utils::null_deleter()),options);
        std::stringstream ss;
        writer.Serialize(ss);
        return ConvertStringToUnicode(ss.str());
    }

    XMLReadablePtr GetXMLReadable() {
        return _xmlreadable;
    }
protected:
    XMLReadablePtr _xmlreadable;
};

XMLReadablePtr ExtractXMLReadable(object o) {
    extract<PyXMLReadablePtr> pyreadable(o);
    return ((PyXMLReadablePtr)pyreadable)->GetXMLReadable();
}

object toPyXMLReadable(XMLReadablePtr p) {
    if( !p ) {
        return object();
    }
    return object(PyXMLReadablePtr(new PyXMLReadable(p)));
}

namespace xmlreaders
{

class PyStaticClass
{
public:
};

PyXMLReadablePtr pyCreateStringXMLReadable(const std::string& xmlid, const std::string& data)
{
    return PyXMLReadablePtr(new PyXMLReadable(XMLReadablePtr(new OpenRAVE::xmlreaders::StringXMLReadable(xmlid, data))));
}

} // end namespace xmlreaders

object toPyGraphHandle(const GraphHandlePtr p)
{
    if( !p ) {
        return object();
    }
    return object(PyGraphHandle(p));
}

object toPyUserData(UserDataPtr p)
{
    if( !p ) {
        return object();
    }
    return object(PyUserData(p));
}

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
    PyAABB() {
    }
    PyAABB(object newpos, object newextents) {
        ab.pos = ExtractVector3(newpos);
        ab.extents = ExtractVector3(newextents);
    }
    PyAABB(const AABB& newab) : ab(newab) {
    }

    object extents() {
        return toPyVector3(ab.extents);
    }
    object pos() {
        return toPyVector3(ab.pos);
    }

    virtual string __repr__() {
        return boost::str(boost::format("AABB([%f,%f,%f],[%f,%f,%f])")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual string __str__() {
        return boost::str(boost::format("<%f %f %f %f %f %f>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

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
    PyTriMesh() {
    }
    PyTriMesh(object vertices, object indices) : vertices(vertices), indices(indices) {
    }
    PyTriMesh(const TriMesh& mesh) {
        npy_intp dims[] = { mesh.vertices.size(),3};
        PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
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

    void GetTriMesh(TriMesh& mesh) {
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

    string __str__() {
        return boost::str(boost::format("<trimesh: verts %d, tris=%d>")%len(vertices)%len(indices));
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object vertices,indices;
};

bool ExtractTriMesh(object o, TriMesh& mesh)
{
    extract<boost::shared_ptr<PyTriMesh> > pytrimesh(o);
    if( pytrimesh.check() ) {
        ((boost::shared_ptr<PyTriMesh>)pytrimesh)->GetTriMesh(mesh);
        return true;
    }
    return false;
}

object toPyTriMesh(const TriMesh& mesh)
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

class PyConfigurationSpecification : public boost::enable_shared_from_this<PyConfigurationSpecification>
{
public:
    PyConfigurationSpecification() {
    }
    PyConfigurationSpecification(const std::string &s) {
        std::stringstream ss(s);
        ss >> _spec;
    }
    PyConfigurationSpecification(const ConfigurationSpecification& spec) {
        _spec = spec;
    }
    PyConfigurationSpecification(const ConfigurationSpecification::Group& g) {
        _spec = ConfigurationSpecification(g);
    }
    PyConfigurationSpecification(PyConfigurationSpecificationPtr pyspec) {
        _spec = pyspec->_spec;
    }
    virtual ~PyConfigurationSpecification() {
    }

    int GetDOF() const {
        return _spec.GetDOF();
    }

    bool IsValid() const {
        return _spec.IsValid();
    }

    const ConfigurationSpecification::Group& GetGroupFromName(const std::string& name) {
        return _spec.GetGroupFromName(name);
    }

    object FindCompatibleGroup(const std::string& name, bool exactmatch) const
    {
        std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindCompatibleGroup(name,exactmatch);
        if( it == _spec._vgroups.end() ) {
            return object();
        }
        return object(boost::shared_ptr<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
    }

    object FindTimeDerivativeGroup(const std::string& name, bool exactmatch) const
    {
        std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindTimeDerivativeGroup(name,exactmatch);
        if( it == _spec._vgroups.end() ) {
            return object();
        }
        return object(boost::shared_ptr<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
    }

//    ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

    void ResetGroupOffsets()
    {
        _spec.ResetGroupOffsets();
    }

    void AddVelocityGroups(bool adddeltatime)
    {
        RAVELOG_WARN("openravepy AddVelocityGroups is deprecated, use AddDerivativeGroups\n");
        _spec.AddDerivativeGroups(1,adddeltatime);
    }

    void AddDerivativeGroups(int deriv, bool adddeltatime)
    {
        _spec.AddDerivativeGroups(deriv,adddeltatime);
    }

    int AddDeltaTimeGroup() {
        return _spec.AddDeltaTimeGroup();
    }

    int AddGroup(const std::string& name, int dof, const std::string& interpolation)
    {
        return _spec.AddGroup(name,dof,interpolation);
    }

    int AddGroup(const ConfigurationSpecification::Group& g)
    {
        return _spec.AddGroup(g);
    }

    PyConfigurationSpecificationPtr ConvertToVelocitySpecification() const
    {
        return openravepy::toPyConfigurationSpecification(_spec.ConvertToVelocitySpecification());
    }

    PyConfigurationSpecificationPtr GetTimeDerivativeSpecification(int timederivative) const
    {
        return openravepy::toPyConfigurationSpecification(_spec.GetTimeDerivativeSpecification(timederivative));
    }

    object ExtractTransform(object otransform, object odata, PyKinBodyPtr pybody, int timederivative=0) const
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        Transform t;
        if( !(otransform == object()) ) {
            t = openravepy::ExtractTransform(otransform);
        }
        if( _spec.ExtractTransform(t,vdata.begin(),openravepy::GetKinBody(pybody)) ) {
            return openravepy::ReturnTransform(t);
        }
        return object();
    }

//    bool ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative=0) const;
//
//    bool ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative=0) const;
//
    object ExtractJointValues(object odata, PyKinBodyPtr pybody, object oindices, int timederivative=0) const
    {
        std::vector<int> vindices = ExtractArray<int>(oindices);
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        std::vector<dReal> values(vindices.size(),0);
        bool bfound = _spec.ExtractJointValues(values.begin(),vdata.begin(),openravepy::GetKinBody(pybody),vindices,timederivative);
        if( bfound ) {
            return toPyArray(values);
        }
        else {
            return object();
        }
    }

    object ExtractDeltaTime(object odata) const
    {
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        dReal deltatime=0;
        bool bfound = _spec.ExtractDeltaTime(deltatime,vdata.begin());
        if( bfound ) {
            return object(deltatime);
        }
        else {
            return object();
        }
    }

    bool InsertJointValues(object odata, object ovalues, PyKinBodyPtr pybody, object oindices, int timederivative=0) const
    {
        vector<int> vindices = ExtractArray<int>(oindices);
        std::vector<dReal> vdata = ExtractArray<dReal>(odata);
        std::vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
        if( !_spec.InsertJointValues(vdata.begin(), vvalues.begin(), openravepy::GetKinBody(pybody), vindices, timederivative) ) {
            return false;
        }
        // copy the value back, this is wasteful, but no other way to do it unless vdata pointer pointed directly to odata
        for(size_t i = 0; i < vdata.size(); ++i) {
            odata[i] = vdata[i];
        }
        return true;
    }

    bool InsertDeltaTime(object odata, dReal deltatime)
    {
        // it is easier to get the time index
        FOREACHC(itgroup,_spec._vgroups) {
            if( itgroup->name == "deltatime" ) {
                odata[itgroup->offset] = object(deltatime);
                return true;
            }
        }
        return false;
    }

//
//    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv);
//
//    static void ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification& targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification& sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized = true);

    bool __eq__(PyConfigurationSpecificationPtr p) {
        return !!p && _spec==p->_spec;
    }
    bool __ne__(PyConfigurationSpecificationPtr p) {
        return !p || _spec!=p->_spec;
    }

    PyConfigurationSpecificationPtr __add__(PyConfigurationSpecificationPtr r)
    {
        return PyConfigurationSpecificationPtr(new PyConfigurationSpecification(_spec + r->_spec));
    }

    PyConfigurationSpecificationPtr __iadd__(PyConfigurationSpecificationPtr r)
    {
        _spec += r->_spec;
        return shared_from_this();
    }

    string __repr__() {
        std::stringstream ss;
        ss << "ConfigurationSpecification(\"\"\"" << _spec << "\"\"\")";
        return ss.str();
    }
    string __str__() {
        std::stringstream ss;
        ss << "<configuration dof=\"" << _spec.GetDOF() << "\">";
        return ss.str();
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    ConfigurationSpecification _spec;
};

class ConfigurationSpecification_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyConfigurationSpecification& pyspec)
    {
        std::stringstream ss;
        ss << pyspec._spec;
        return boost::python::make_tuple(ss.str());
    }
};

PyConfigurationSpecificationPtr toPyConfigurationSpecification(const ConfigurationSpecification &spec)
{
    return PyConfigurationSpecificationPtr(new PyConfigurationSpecification(spec));
}

const ConfigurationSpecification& GetConfigurationSpecification(PyConfigurationSpecificationPtr p)
{
    return p->_spec;
}

struct spec_from_group
{
    spec_from_group()
    {
        boost::python::converter::registry::push_back(&convertible, &construct, boost::python::type_id<PyConfigurationSpecificationPtr>());
    }

    static void* convertible(PyObject* obj)
    {
        return obj == Py_None ||  boost::python::extract<ConfigurationSpecification::Group>(obj).check() ? obj : NULL;
    }

    static void construct(PyObject* obj, boost::python::converter::rvalue_from_python_stage1_data* data)
    {
        ConfigurationSpecification::Group g = (ConfigurationSpecification::Group)boost::python::extract<ConfigurationSpecification::Group>(obj);
        void* storage = ((boost::python::converter::rvalue_from_python_storage<PyConfigurationSpecificationPtr>*)data)->storage.bytes;
        new (storage) PyConfigurationSpecificationPtr(new PyConfigurationSpecification(g));
        data->convertible = storage;
    }
};

PyConfigurationSpecificationPtr pyRaveGetAffineConfigurationSpecification(int affinedofs,PyKinBodyPtr pybody=PyKinBodyPtr(), const std::string& interpolation="")
{
    return openravepy::toPyConfigurationSpecification(RaveGetAffineConfigurationSpecification(affinedofs,openravepy::GetKinBody(pybody), interpolation));
}

object pyRaveGetAffineDOFValuesFromTransform(object otransform, int affinedofs, object oActvAffineRotationAxis=object())
{
    Vector vActvAffineRotationAxis(0,0,1);
    if( !(oActvAffineRotationAxis == object()) ) {
        vActvAffineRotationAxis = ExtractVector3(oActvAffineRotationAxis);
    }
    std::vector<dReal> values(RaveGetAffineDOF(affinedofs));
    RaveGetAffineDOFValuesFromTransform(values.begin(),ExtractTransform(otransform), affinedofs, vActvAffineRotationAxis);
    return toPyArray(values);
}

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

void raveLog(const string &s, int level)
{
    if( s.size() > 0 ) {
        RavePrintfA(s,level);
    }
}

void raveLogFatal(const string &s)
{
    raveLog(s,Level_Fatal);
}
void raveLogError(const string &s)
{
    raveLog(s,Level_Error);
}
void raveLogWarn(const string &s)
{
    raveLog(s,Level_Warn);
}
void raveLogInfo(const string &s)
{
    raveLog(s,Level_Info);
}
void raveLogDebug(const string &s)
{
    raveLog(s,Level_Debug);
}
void raveLogVerbose(const string &s)
{
    raveLog(s,Level_Verbose);
}

int pyGetIntFromPy(object olevel, int defaultvalue)
{
    int level = defaultvalue;
    if( !(olevel == object()) ) {
        // some version of boost python return true for extract::check, even through the actual conversion will throw an OverflowError
        // therefore check for conversion compatibility starting at the longest signed integer
        extract<int64_t> levelint64(olevel);
        if( levelint64.check() ) {
            level = static_cast<int>((int64_t)levelint64);
        }
        else {
            extract<uint64_t> leveluint64(olevel);
            if( leveluint64.check() ) {
                level = static_cast<int>((uint64_t)leveluint64);
            }
            else {
                extract<uint32_t> leveluint32(olevel);
                if( leveluint32.check() ) {
                    level = static_cast<int>((uint32_t)leveluint32);
                }
                else {
                    extract<int> levelint32(olevel);
                    if( levelint32.check() ) {
                        level = (int)levelint32;
                    }
                    else {
                        RAVELOG_WARN("failed to extract int\n");
                    }
                }
            }
        }
    }
    return level;
}

void pyRaveSetDebugLevel(object olevel)
{
    OpenRAVE::RaveSetDebugLevel(pyGetIntFromPy(olevel, Level_Info));
}

int pyRaveInitialize(bool bLoadAllPlugins=true, object olevel=object())
{

    return OpenRAVE::RaveInitialize(bLoadAllPlugins,pyGetIntFromPy(olevel, Level_Info));
}

void pyRaveSetDataAccess(object oaccess)
{
    OpenRAVE::RaveSetDataAccess(pyGetIntFromPy(oaccess, Level_Info));
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
    boost::python::dict ointerfacenames;
    FOREACHC(it, interfacenames) {
        boost::python::list names;
        FOREACHC(itname,it->second) {
            names.append(*itname);
        }
        ointerfacenames[it->first] = names;
    }
    return ointerfacenames;
}

PyInterfaceBasePtr RaveClone(PyInterfaceBasePtr pyreference, int cloningoptions)
{
    InterfaceBasePtr pclone = OpenRAVE::RaveClone<InterfaceBase>(pyreference->GetInterfaceBase(), cloningoptions);
    switch(pclone->GetInterfaceType()) {
    case PT_Planner: return toPyPlanner(RaveInterfaceCast<PlannerBase>(pclone), pyreference->GetEnv());
    case PT_Robot: return toPyRobot(RaveInterfaceCast<RobotBase>(pclone), pyreference->GetEnv());
    case PT_SensorSystem: return toPySensorSystem(RaveInterfaceCast<SensorSystemBase>(pclone), pyreference->GetEnv());
    case PT_Controller: return toPyController(RaveInterfaceCast<ControllerBase>(pclone), pyreference->GetEnv());
    case PT_Module: return toPyModule(RaveInterfaceCast<ModuleBase>(pclone), pyreference->GetEnv());
    case PT_InverseKinematicsSolver: return toPyIkSolver(RaveInterfaceCast<IkSolverBase>(pclone), pyreference->GetEnv());
    case PT_KinBody: return toPyKinBody(RaveInterfaceCast<KinBody>(pclone), pyreference->GetEnv());
    case PT_PhysicsEngine: return toPyPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(pclone), pyreference->GetEnv());
    case PT_Sensor: return toPySensor(RaveInterfaceCast<SensorBase>(pclone), pyreference->GetEnv());
    case PT_CollisionChecker: return toPyCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(pclone), pyreference->GetEnv());
    case PT_Trajectory: return toPyTrajectory(RaveInterfaceCast<TrajectoryBase>(pclone), pyreference->GetEnv());
    case PT_Viewer: return toPyViewer(RaveInterfaceCast<ViewerBase>(pclone), pyreference->GetEnv());
    case PT_SpaceSampler: return toPySpaceSampler(RaveInterfaceCast<SpaceSamplerBase>(pclone), pyreference->GetEnv());
    }
    throw openrave_exception("invalid interface type",ORE_InvalidArguments);
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
    npy_intp dims[] = { N,7};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
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
    npy_intp dims[] = { N,7};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
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

object quatMultiply(object oquat1, object oquat2)
{
    return toPyVector4(OpenRAVE::geometry::quatMultiply(ExtractVector4(oquat1),ExtractVector4(oquat2)));
}

object quatInverse(object oquat)
{
    return toPyVector4(OpenRAVE::geometry::quatInverse(ExtractVector4(oquat)));
}

object poseMult(object opose1, object opose2)
{
    return toPyArray(ExtractTransformType<dReal>(opose1)*ExtractTransformType<dReal>(opose2));
}

object poseTransformPoints(object opose, object opoints)
{
    Transform t = ExtractTransformType<dReal>(opose);
    int N = len(opoints);
    npy_intp dims[] = { N,3};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
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
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransformMatrix(o);
    return ss.str();
}

string poseSerialization(object o)
{
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransform(o);
    return ss.str();
}

namespace planningutils
{

class PyStaticClass
{
public:
};

bool pyJitterTransform(PyKinBodyPtr pybody, float fJitter, int nMaxIterations=1000)
{
    return OpenRAVE::planningutils::JitterTransform(openravepy::GetKinBody(pybody), fJitter, nMaxIterations);
}

void pyConvertTrajectorySpecification(PyTrajectoryBasePtr pytraj, PyConfigurationSpecificationPtr pyspec)
{
    OpenRAVE::planningutils::ConvertTrajectorySpecification(openravepy::GetTrajectory(pytraj),openravepy::GetConfigurationSpecification(pyspec));
}

void pyComputeTrajectoryDerivatives(PyTrajectoryBasePtr pytraj, int maxderiv)
{
    OpenRAVE::planningutils::ComputeTrajectoryDerivatives(openravepy::GetTrajectory(pytraj),maxderiv);
}

object pyReverseTrajectory(PyTrajectoryBasePtr pytraj)
{
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::ReverseTrajectory(openravepy::GetTrajectory(pytraj)),openravepy::toPyEnvironment(pytraj)));
}

void pyVerifyTrajectory(object pyparameters, PyTrajectoryBasePtr pytraj, dReal samplingstep)
{
    OpenRAVE::planningutils::VerifyTrajectory(openravepy::GetPlannerParametersConst(pyparameters), openravepy::GetTrajectory(pytraj),samplingstep);
}

PlannerStatus pySmoothActiveDOFTrajectory(PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::SmoothActiveDOFTrajectory(openravepy::GetTrajectory(pytraj),openravepy::GetRobot(pyrobot),fmaxvelmult,fmaxaccelmult,plannername,plannerparameters);
}

class PyActiveDOFTrajectorySmoother
{
public:
    PyActiveDOFTrajectorySmoother(PyRobotBasePtr pyrobot, const std::string& plannername, const std::string& plannerparameters) : _smoother(openravepy::GetRobot(pyrobot), plannername, plannerparameters) {
    }
    virtual ~PyActiveDOFTrajectorySmoother() {
    }

    PlannerStatus PlanPath(PyTrajectoryBasePtr pytraj, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        return _smoother.PlanPath(ptraj);
    }

    OpenRAVE::planningutils::ActiveDOFTrajectorySmoother _smoother;
};

typedef boost::shared_ptr<PyActiveDOFTrajectorySmoother> PyActiveDOFTrajectorySmootherPtr;

PlannerStatus pySmoothAffineTrajectory(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::SmoothAffineTrajectory(openravepy::GetTrajectory(pytraj),ExtractArray<dReal>(omaxvelocities), ExtractArray<dReal>(omaxaccelerations),plannername,plannerparameters);
}

PlannerStatus pySmoothTrajectory(PyTrajectoryBasePtr pytraj, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::SmoothTrajectory(openravepy::GetTrajectory(pytraj),fmaxvelmult,fmaxaccelmult,plannername,plannerparameters);
}

PlannerStatus pyRetimeActiveDOFTrajectory(PyTrajectoryBasePtr pytraj, PyRobotBasePtr pyrobot, bool hastimestamps=false, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::RetimeActiveDOFTrajectory(openravepy::GetTrajectory(pytraj),openravepy::GetRobot(pyrobot),hastimestamps,fmaxvelmult,fmaxaccelmult,plannername,plannerparameters);
}

class PyActiveDOFTrajectoryRetimer
{
public:
    PyActiveDOFTrajectoryRetimer(PyRobotBasePtr pyrobot, const std::string& plannername, const std::string& plannerparameters) : _retimer(openravepy::GetRobot(pyrobot), plannername, plannerparameters) {
    }
    virtual ~PyActiveDOFTrajectoryRetimer() {
    }

    PlannerStatus PlanPath(PyTrajectoryBasePtr pytraj, bool hastimestamps=false, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        return _retimer.PlanPath(ptraj, hastimestamps);
    }

    OpenRAVE::planningutils::ActiveDOFTrajectoryRetimer _retimer;
};

typedef boost::shared_ptr<PyActiveDOFTrajectoryRetimer> PyActiveDOFTrajectoryRetimerPtr;

class PyAffineTrajectoryRetimer
{
public:
    PyAffineTrajectoryRetimer(const std::string& plannername, const std::string& plannerparameters) : _retimer(plannername, plannerparameters) {
    }
    virtual ~PyAffineTrajectoryRetimer() {
    }

    PlannerStatus PlanPath(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, bool hastimestamps=false, bool releasegil=true)
    {
        openravepy::PythonThreadSaverPtr statesaver;
        TrajectoryBasePtr ptraj = openravepy::GetTrajectory(pytraj);
        std::vector<dReal> vmaxvelocities = ExtractArray<dReal>(omaxvelocities);
        std::vector<dReal> vmaxaccelerations = ExtractArray<dReal>(omaxaccelerations);
        if( releasegil ) {
            statesaver.reset(new openravepy::PythonThreadSaver());
        }
        return _retimer.PlanPath(ptraj,vmaxvelocities, vmaxaccelerations, hastimestamps);
    }

    OpenRAVE::planningutils::AffineTrajectoryRetimer _retimer;
};

typedef boost::shared_ptr<PyAffineTrajectoryRetimer> PyAffineTrajectoryRetimerPtr;

PlannerStatus pyRetimeAffineTrajectory(PyTrajectoryBasePtr pytraj, object omaxvelocities, object omaxaccelerations, bool hastimestamps=false, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::RetimeAffineTrajectory(openravepy::GetTrajectory(pytraj),ExtractArray<dReal>(omaxvelocities), ExtractArray<dReal>(omaxaccelerations),hastimestamps,plannername,plannerparameters);
}

PlannerStatus pyRetimeTrajectory(PyTrajectoryBasePtr pytraj, bool hastimestamps=false, dReal fmaxvelmult=1.0, dReal fmaxaccelmult=1.0, const std::string& plannername="", const std::string& plannerparameters="")
{
    return OpenRAVE::planningutils::RetimeTrajectory(openravepy::GetTrajectory(pytraj),hastimestamps,fmaxvelmult,fmaxaccelmult,plannername,plannerparameters);
}

void pyInsertWaypointWithSmoothing(int index, object odofvalues, object odofvelocities, PyTrajectoryBasePtr pytraj, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="")
{
    OpenRAVE::planningutils::InsertWaypointWithSmoothing(index,ExtractArray<dReal>(odofvalues),ExtractArray<dReal>(odofvelocities),openravepy::GetTrajectory(pytraj),fmaxvelmult,fmaxaccelmult,plannername);
}

object pyMergeTrajectories(object pytrajectories)
{
    std::list<TrajectoryBaseConstPtr> listtrajectories;
    PyEnvironmentBasePtr pyenv;
    for(int i = 0; i < len(pytrajectories); ++i) {
        extract<PyTrajectoryBasePtr> epytrajectory(pytrajectories[i]);
        PyTrajectoryBasePtr pytrajectory = (PyTrajectoryBasePtr)epytrajectory;
        if( !pyenv ) {
            pyenv = openravepy::toPyEnvironment(pytrajectory);
        }
        else {
            BOOST_ASSERT(pyenv == openravepy::toPyEnvironment(pytrajectory));
        }
        listtrajectories.push_back(openravepy::GetTrajectory(pytrajectory));
    }
    return object(openravepy::toPyTrajectory(OpenRAVE::planningutils::MergeTrajectories(listtrajectories),pyenv));
}

class PyDHParameter
{
public:
    PyDHParameter() : parentindex(-1), transform(ReturnTransform(Transform())), d(0), a(0), theta(0), alpha(0) {
    }
    PyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv) : joint(toPyKinBodyJoint(boost::const_pointer_cast<KinBody::Joint>(p.joint), pyenv)), parentindex(p.parentindex), transform(ReturnTransform(p.transform)), d(p.d), a(p.a), theta(p.theta), alpha(p.alpha) {
    }
    PyDHParameter(object joint, int parentindex, object transform, dReal d, dReal a, dReal theta, dReal alpha) : joint(joint), parentindex(parentindex), transform(transform), d(d), a(a), theta(theta), alpha(alpha) {
    }
    virtual ~PyDHParameter() {
    }
    string __repr__() {
        return boost::str(boost::format("<DHParameter(joint=%s, parentindex=%d, d=%f, a=%f, theta=%f, alpha=%f)>")%reprPyKinBodyJoint(joint)%parentindex%d%a%theta%alpha);
    }
    string __str__() {
        TransformMatrix tm = ExtractTransformMatrix(transform);
        return boost::str(boost::format("<joint %s, transform [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f]], parentindex %d>")%strPyKinBodyJoint(joint)%tm.m[0]%tm.m[1]%tm.m[2]%tm.trans[0]%tm.m[4]%tm.m[5]%tm.m[6]%tm.trans[1]%tm.m[8]%tm.m[9]%tm.m[10]%tm.trans[2]%parentindex);
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object joint;
    int parentindex;
    object transform;
    dReal d, a, theta, alpha;
};

object toPyDHParameter(const OpenRAVE::planningutils::DHParameter& p, PyEnvironmentBasePtr pyenv)
{
    return object(boost::shared_ptr<PyDHParameter>(new PyDHParameter(p,pyenv)));
}

class DHParameter_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyDHParameter& p)
    {
        return boost::python::make_tuple(object(), p.parentindex, p.transform, p.d, p.a, p.theta, p.alpha);
    }
};

boost::python::list pyGetDHParameters(PyKinBodyPtr pybody)
{
    boost::python::list oparameters;
    std::vector<OpenRAVE::planningutils::DHParameter> vparameters;
    OpenRAVE::planningutils::GetDHParameters(vparameters,openravepy::GetKinBody(pybody));
    PyEnvironmentBasePtr pyenv = toPyEnvironment(pybody);
    FOREACH(itp,vparameters) {
        oparameters.append(toPyDHParameter(*itp,pyenv));
    }
    return oparameters;
}

class PyManipulatorIKGoalSampler
{
public:
    PyManipulatorIKGoalSampler(object pymanip, object oparameterizations, int nummaxsamples=20, int nummaxtries=10, dReal jitter=0, bool searchfreeparameters=true) {
        std::list<IkParameterization> listparameterizations;
        size_t num = len(oparameterizations);
        for(size_t i = 0; i < num; ++i) {
            IkParameterization ikparam;
            if( ExtractIkParameterization(oparameterizations[i],ikparam) ) {
                listparameterizations.push_back(ikparam);
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0("ManipulatorIKGoalSampler parameterizations need to be all IkParameterization objeccts",ORE_InvalidArguments);
            }
        }
        dReal fsampleprob=1;
        _sampler.reset(new OpenRAVE::planningutils::ManipulatorIKGoalSampler(GetRobotManipulator(pymanip), listparameterizations, nummaxsamples, nummaxtries, fsampleprob, searchfreeparameters));
        _sampler->SetJitter(jitter);
    }
    virtual ~PyManipulatorIKGoalSampler() {
    }

    object Sample(bool ikreturn = false, bool releasegil = false)
    {
        if( ikreturn ) {
            IkReturnPtr pikreturn = _sampler->Sample();
            if( !!pikreturn ) {
                return openravepy::toPyIkReturn(*pikreturn);
            }
        }
        else {
            std::vector<dReal> vgoal;
            if( _sampler->Sample(vgoal) ) {
                return toPyArray(vgoal);
            }
        }
        return object();
    }

    object SampleAll(int maxsamples=0, int maxchecksamples=0, bool releasegil = false)
    {
        boost::python::list oreturns;
        std::list<IkReturnPtr> listreturns;
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            _sampler->SampleAll(listreturns, maxsamples, maxchecksamples);
        }
        FOREACH(it,listreturns) {
            oreturns.append(openravepy::toPyIkReturn(**it));
        }
        return oreturns;
    }

    OpenRAVE::planningutils::ManipulatorIKGoalSamplerPtr _sampler;
};

typedef boost::shared_ptr<PyManipulatorIKGoalSampler> PyManipulatorIKGoalSamplerPtr;

} // end namespace planningutils

BOOST_PYTHON_FUNCTION_OVERLOADS(RaveInitialize_overloads, pyRaveInitialize, 0, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(RaveFindLocalFile_overloads, OpenRAVE::RaveFindLocalFile, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(JitterTransform_overloads, planningutils::pyJitterTransform, 2, 3);
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothActiveDOFTrajectory_overloads, planningutils::pySmoothActiveDOFTrajectory, 2, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothAffineTrajectory_overloads, planningutils::pySmoothAffineTrajectory, 3, 5)
BOOST_PYTHON_FUNCTION_OVERLOADS(SmoothTrajectory_overloads, planningutils::pySmoothTrajectory, 1, 5)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeActiveDOFTrajectory_overloads, planningutils::pyRetimeActiveDOFTrajectory, 2, 7)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeAffineTrajectory_overloads, planningutils::pyRetimeAffineTrajectory, 3, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(RetimeTrajectory_overloads, planningutils::pyRetimeTrajectory, 1, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(InsertWaypointWithSmoothing_overloads, planningutils::pyInsertWaypointWithSmoothing, 4, 7)
BOOST_PYTHON_FUNCTION_OVERLOADS(pyRaveGetAffineConfigurationSpecification_overloads, openravepy::pyRaveGetAffineConfigurationSpecification, 1, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(pyRaveGetAffineDOFValuesFromTransform_overloads, openravepy::pyRaveGetAffineDOFValuesFromTransform, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Sample_overloads, Sample, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SampleAll_overloads, SampleAll, 0, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractTransform_overloads, PyConfigurationSpecification::ExtractTransform, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractJointValues_overloads, PyConfigurationSpecification::ExtractJointValues, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads, PlanPath, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads2, PlanPath, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PlanPath_overloads3, PlanPath, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Serialize_overloads, Serialize, 0, 1)

void init_openravepy_global()
{
    enum_<OpenRAVEErrorCode>("ErrorCode" DOXY_ENUM(OpenRAVEErrorCode))
    .value("Failed",ORE_Failed)
    .value("InvalidArguments",ORE_InvalidArguments)
    .value("EnvironmentNotLocked",ORE_EnvironmentNotLocked)
    .value("CommandNotSupported",ORE_CommandNotSupported)
    .value("Assert",ORE_Assert)
    .value("InvalidPlugin",ORE_InvalidPlugin)
    .value("InvalidInterfaceHash",ORE_InvalidInterfaceHash)
    .value("NotImplemented",ORE_NotImplemented)
    .value("InconsistentConstraints",ORE_InconsistentConstraints)
    .value("NotInitialized",ORE_NotInitialized)
    .value("InvalidState",ORE_InvalidState)
    .value("Timeout",ORE_Timeout)
    ;
    enum_<DebugLevel>("DebugLevel" DOXY_ENUM(DebugLevel))
    .value("Fatal",Level_Fatal)
    .value("Error",Level_Error)
    .value("Warn",Level_Warn)
    .value("Info",Level_Info)
    .value("Debug",Level_Debug)
    .value("Verbose",Level_Verbose)
    .value("VerifyPlans",Level_VerifyPlans)
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
    .value("probleminstance",PT_Module)
    .value(RaveGetInterfaceName(PT_Module).c_str(),PT_Module)
    .value(RaveGetInterfaceName(PT_IkSolver).c_str(),PT_IkSolver)
    .value(RaveGetInterfaceName(PT_KinBody).c_str(),PT_KinBody)
    .value(RaveGetInterfaceName(PT_PhysicsEngine).c_str(),PT_PhysicsEngine)
    .value(RaveGetInterfaceName(PT_Sensor).c_str(),PT_Sensor)
    .value(RaveGetInterfaceName(PT_CollisionChecker).c_str(),PT_CollisionChecker)
    .value(RaveGetInterfaceName(PT_Trajectory).c_str(),PT_Trajectory)
    .value(RaveGetInterfaceName(PT_Viewer).c_str(),PT_Viewer)
    .value(RaveGetInterfaceName(PT_SpaceSampler).c_str(),PT_SpaceSampler)
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

    enum_<IntervalType>("Interval" DOXY_ENUM(IntervalType))
    .value("Open",IT_Open)
    .value("OpenStart",IT_OpenStart)
    .value("OpenEnd",IT_OpenEnd)
    .value("Closed",IT_Closed)
    ;
    enum_<SampleDataType>("SampleDataType" DOXY_ENUM(SampleDataType))
    .value("Real",SDT_Real)
    .value("Uint32",SDT_Uint32)
    ;

    class_<UserData, UserDataPtr >("UserData", DOXY_CLASS(UserData))
    ;

    class_< boost::shared_ptr< void > >("VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");

    class_<PyGraphHandle, boost::shared_ptr<PyGraphHandle> >("GraphHandle", DOXY_CLASS(GraphHandle), no_init)
    .def("SetTransform",&PyGraphHandle::SetTransform,DOXY_FN(GraphHandle,SetTransform))
    .def("SetShow",&PyGraphHandle::SetShow,DOXY_FN(GraphHandle,SetShow))
    .def("Close",&PyGraphHandle::Close,DOXY_FN(GraphHandle,Close))
    ;

    class_<PyUserData, boost::shared_ptr<PyUserData> >("UserData", DOXY_CLASS(UserData), no_init)
    .def("close",&PyUserData::Close,"deprecated")
    .def("Close",&PyUserData::Close,"force releasing the user handle point.")
    ;
    class_<PySerializableData, boost::shared_ptr<PySerializableData>, bases<PyUserData> >("SerializableData", DOXY_CLASS(SerializableData), no_init)
    .def("Serialize",&PySerializableData::Serialize,args("options"), DOXY_FN(SerializableData, Serialize))
    .def("Deserialize",&PySerializableData::Deserialize,args("data"), DOXY_FN(SerializableData, Deserialize))
    ;

    class_<PyRay, boost::shared_ptr<PyRay> >("Ray", DOXY_CLASS(geometry::ray))
    .def(init<object,object>(args("pos","dir")))
    .def("dir",&PyRay::dir)
    .def("pos",&PyRay::pos)
    .def("__str__",&PyRay::__str__)
    .def("__unicode__",&PyRay::__unicode__)
    .def("__repr__",&PyRay::__repr__)
    .def_pickle(Ray_pickle_suite())
    ;
    class_<PyAABB, boost::shared_ptr<PyAABB> >("AABB", DOXY_CLASS(geometry::aabb))
    .def(init<object,object>(args("pos","extents")))
    .def("extents",&PyAABB::extents)
    .def("pos",&PyAABB::pos)
    .def("__str__",&PyAABB::__str__)
    .def("__unicode__",&PyAABB::__unicode__)
    .def("__repr__",&PyAABB::__repr__)
    .def_pickle(AABB_pickle_suite())
    ;
    class_<PyTriMesh, boost::shared_ptr<PyTriMesh> >("TriMesh", DOXY_CLASS(TriMesh))
    .def(init<object,object>(args("vertices","indices")))
    .def_readwrite("vertices",&PyTriMesh::vertices)
    .def_readwrite("indices",&PyTriMesh::indices)
    .def("__str__",&PyTriMesh::__str__)
    .def("__unicode__",&PyAABB::__unicode__)
    .def_pickle(TriMesh_pickle_suite())
    ;
    class_<InterfaceBase, InterfaceBasePtr, boost::noncopyable >("InterfaceBase", DOXY_CLASS(InterfaceBase), no_init)
    ;

    class_<PyXMLReadable, PyXMLReadablePtr >("XMLReadable", DOXY_CLASS(XMLReadable), no_init)
    .def(init<XMLReadablePtr>(args("readableraw")))
    .def("GetXMLId", &PyXMLReadable::GetXMLId, DOXY_FN(XMLReadable, GetXMLId))
    .def("Serialize", &PyXMLReadable::Serialize, Serialize_overloads(args("options"), DOXY_FN(XMLReadable, Serialize)))
    ;

    class_<PyPluginInfo, boost::shared_ptr<PyPluginInfo> >("PluginInfo", DOXY_CLASS(PLUGININFO),no_init)
    .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
    .def_readonly("version",&PyPluginInfo::version)
    ;

    {
        int (PyConfigurationSpecification::*addgroup1)(const std::string&, int, const std::string&) = &PyConfigurationSpecification::AddGroup;
        int (PyConfigurationSpecification::*addgroup2)(const ConfigurationSpecification::Group&) = &PyConfigurationSpecification::AddGroup;

        scope configurationspecification = class_<PyConfigurationSpecification, PyConfigurationSpecificationPtr >("ConfigurationSpecification",DOXY_CLASS(ConfigurationSpecification))
                                           .def(init<PyConfigurationSpecificationPtr>(args("spec")) )
                                           .def(init<const ConfigurationSpecification::Group&>(args("group")) )
                                           .def(init<const std::string&>(args("xmldata")) )
                                           .def("GetGroupFromName",&PyConfigurationSpecification::GetGroupFromName, return_value_policy<copy_const_reference>(), DOXY_FN(ConfigurationSpecification,GetGroupFromName))
                                           .def("FindCompatibleGroup",&PyConfigurationSpecification::FindCompatibleGroup, DOXY_FN(ConfigurationSpecification,FindCompatibleGroup))
                                           .def("FindTimeDerivativeGroup",&PyConfigurationSpecification::FindTimeDerivativeGroup, DOXY_FN(ConfigurationSpecification,FindTimeDerivativeGroup))
                                           .def("GetDOF",&PyConfigurationSpecification::GetDOF,DOXY_FN(ConfigurationSpecification,GetDOF))
                                           .def("IsValid",&PyConfigurationSpecification::IsValid,DOXY_FN(ConfigurationSpecification,IsValid))
                                           .def("ResetGroupOffsets",&PyConfigurationSpecification::ResetGroupOffsets,DOXY_FN(ConfigurationSpecification,ResetGroupOffsets))
                                           .def("AddVelocityGroups",&PyConfigurationSpecification::AddVelocityGroups,args("adddeltatime"), DOXY_FN(ConfigurationSpecification,AddVelocityGroups))
                                           .def("AddDerivativeGroups",&PyConfigurationSpecification::AddDerivativeGroups,args("adddeltatime"), DOXY_FN(ConfigurationSpecification,AddDerivativeGroups))
                                           .def("AddDeltaTimeGroup",&PyConfigurationSpecification::AddDeltaTimeGroup,DOXY_FN(ConfigurationSpecification,AddDeltaTimeGroup))
                                           .def("AddGroup",addgroup1,args("name","dof","interpolation"), DOXY_FN(ConfigurationSpecification,AddGroup "const std::string; int; const std::string"))
                                           .def("AddGroup",addgroup2,args("group"), DOXY_FN(ConfigurationSpecification,AddGroup "const"))
                                           .def("ConvertToVelocitySpecification",&PyConfigurationSpecification::ConvertToVelocitySpecification,DOXY_FN(ConfigurationSpecification,ConvertToVelocitySpecification))
                                           .def("GetTimeDerivativeSpecification",&PyConfigurationSpecification::GetTimeDerivativeSpecification,DOXY_FN(ConfigurationSpecification,GetTimeDerivativeSpecification))

                                           .def("ExtractTransform",&PyConfigurationSpecification::ExtractTransform,ExtractTransform_overloads(args("transform","data","body","timederivative"),DOXY_FN(ConfigurationSpecification,ExtractTransform)))
                                           .def("ExtractJointValues",&PyConfigurationSpecification::ExtractJointValues,ExtractJointValues_overloads(args("data","body","indices","timederivative"),DOXY_FN(ConfigurationSpecification,ExtractJointValues)))
                                           .def("ExtractDeltaTime",&PyConfigurationSpecification::ExtractDeltaTime,args("data"),DOXY_FN(ConfigurationSpecification,ExtractDeltaTime))
                                           .def("InsertDeltaTime",&PyConfigurationSpecification::InsertDeltaTime,args("data","deltatime"),DOXY_FN(ConfigurationSpecification,InsertDeltaTime))
                                           .def("InsertJointValues",&PyConfigurationSpecification::InsertJointValues,args("data","values","body","indices","timederivative"),DOXY_FN(ConfigurationSpecification,InsertJointValues))
                                           .def("__eq__",&PyConfigurationSpecification::__eq__)
                                           .def("__ne__",&PyConfigurationSpecification::__ne__)
                                           .def("__add__",&PyConfigurationSpecification::__add__)
                                           .def("__iadd__",&PyConfigurationSpecification::__iadd__)
                                           .def_pickle(ConfigurationSpecification_pickle_suite())
                                           .def("__str__",&PyConfigurationSpecification::__str__)
                                           .def("__unicode__",&PyConfigurationSpecification::__unicode__)
                                           .def("__repr__",&PyConfigurationSpecification::__repr__)
        ;

        {
            scope group = class_<ConfigurationSpecification::Group, boost::shared_ptr<ConfigurationSpecification::Group> >("Group",DOXY_CLASS(ConfigurationSpecification::Group))
                          .def_readwrite("name",&ConfigurationSpecification::Group::name)
                          .def_readwrite("interpolation",&ConfigurationSpecification::Group::interpolation)
                          .def_readwrite("offset",&ConfigurationSpecification::Group::offset)
                          .def_readwrite("dof",&ConfigurationSpecification::Group::dof)
            ;
        }
    }

    openravepy::spec_from_group();

    {
        scope scope_xmlreaders = class_<xmlreaders::PyStaticClass>("xmlreaders")
                                 .def("CreateStringXMLReadable",xmlreaders::pyCreateStringXMLReadable, args("xmlid", "data"))
                                 .staticmethod("CreateStringXMLReadable")
        ;
    }

    {
        scope x = class_<planningutils::PyStaticClass>("planningutils")
                  .def("JitterTransform",planningutils::pyJitterTransform,JitterTransform_overloads(args("body","jitter","maxiterations"),DOXY_FN1(JitterTransform)))
                  .staticmethod("JitterTransform")
                  .def("ConvertTrajectorySpecification",planningutils::pyConvertTrajectorySpecification,args("trajectory","spec"),DOXY_FN1(ConvertTrajectorySpecification))
                  .staticmethod("ConvertTrajectorySpecification")
                  .def("ComputeTrajectoryDerivatives",planningutils::pyComputeTrajectoryDerivatives,args("trajectory","maxderiv"),DOXY_FN1(ComputeTrajectoryDerivatives))
                  .staticmethod("ComputeTrajectoryDerivatives")
                  .def("ReverseTrajectory",planningutils::pyReverseTrajectory,args("trajectory"),DOXY_FN1(ReverseTrajectory))
                  .staticmethod("ReverseTrajectory")
                  .def("VerifyTrajectory",planningutils::pyVerifyTrajectory,args("parameters","trajectory","samplingstep"),DOXY_FN1(VerifyTrajectory))
                  .staticmethod("VerifyTrajectory")
                  .def("SmoothActiveDOFTrajectory",planningutils::pySmoothActiveDOFTrajectory, SmoothActiveDOFTrajectory_overloads(args("trajectory","robot","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(SmoothActiveDOFTrajectory)))
                  .staticmethod("SmoothActiveDOFTrajectory")
                  .def("SmoothAffineTrajectory",planningutils::pySmoothAffineTrajectory, SmoothAffineTrajectory_overloads(args("trajectory","maxvelocities","maxaccelerations","plannername","plannerparameters"),DOXY_FN1(SmoothAffineTrajectory)))
                  .staticmethod("SmoothAffineTrajectory")
                  .def("SmoothTrajectory",planningutils::pySmoothTrajectory, SmoothTrajectory_overloads(args("trajectory","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(SmoothTrajectory)))
                  .staticmethod("SmoothTrajectory")
                  .def("RetimeActiveDOFTrajectory",planningutils::pyRetimeActiveDOFTrajectory, RetimeActiveDOFTrajectory_overloads(args("trajectory","robot","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(RetimeActiveDOFTrajectory)))
                  .staticmethod("RetimeActiveDOFTrajectory")
                  .def("RetimeAffineTrajectory",planningutils::pyRetimeAffineTrajectory, RetimeAffineTrajectory_overloads(args("trajectory","maxvelocities","maxaccelerations","hastimestamps","plannername","plannerparameters"),DOXY_FN1(RetimeAffineTrajectory)))
                  .staticmethod("RetimeAffineTrajectory")
                  .def("RetimeTrajectory",planningutils::pyRetimeTrajectory, RetimeTrajectory_overloads(args("trajectory","hastimestamps","maxvelmult","maxaccelmult","plannername","plannerparameters"),DOXY_FN1(RetimeTrajectory)))
                  .staticmethod("RetimeTrajectory")
                  .def("InsertWaypointWithSmoothing",planningutils::pyInsertWaypointWithSmoothing, InsertWaypointWithSmoothing_overloads(args("index","dofvalues","dofvelocities","trajectory","maxvelmult","maxaccelmult","plannername"),DOXY_FN1(InsertWaypointWithSmoothing)))
                  .staticmethod("InsertWaypointWithSmoothing")
                  .def("MergeTrajectories",planningutils::pyMergeTrajectories,args("trajectories"),DOXY_FN1(MergeTrajectories))
                  .staticmethod("MergeTrajectories")
                  .def("GetDHParameters",planningutils::pyGetDHParameters,args("body"),DOXY_FN1(GetDHParameters))
                  .staticmethod("GetDHParameters")
        ;

        class_<planningutils::PyDHParameter, boost::shared_ptr<planningutils::PyDHParameter> >("DHParameter", DOXY_CLASS(planningutils::DHParameter))
        .def(init<>())
        .def(init<object, int, object, dReal, dReal, dReal, dReal>(args("joint","parentindex","transform","d","a","theta","alpha")))
        .def_readwrite("joint",&planningutils::PyDHParameter::joint)
        .def_readwrite("transform",&planningutils::PyDHParameter::transform)
        .def_readwrite("d",&planningutils::PyDHParameter::d)
        .def_readwrite("a",&planningutils::PyDHParameter::a)
        .def_readwrite("theta",&planningutils::PyDHParameter::theta)
        .def_readwrite("alpha",&planningutils::PyDHParameter::alpha)
        .def_readwrite("parentindex",&planningutils::PyDHParameter::parentindex)
        .def("__str__",&planningutils::PyDHParameter::__str__)
        .def("__unicode__",&planningutils::PyDHParameter::__unicode__)
        .def("__repr__",&planningutils::PyDHParameter::__repr__)
        .def_pickle(planningutils::DHParameter_pickle_suite())
        ;


        class_<planningutils::PyManipulatorIKGoalSampler, planningutils::PyManipulatorIKGoalSamplerPtr >("ManipulatorIKGoalSampler", DOXY_CLASS(planningutils::ManipulatorIKGoalSampler), no_init)
        .def(init<object, object, optional<int, int, dReal, bool> >(args("manip", "parameterizations", "nummaxsamples", "nummaxtries", "jitter","searchfreeparameters")))
        .def("Sample",&planningutils::PyManipulatorIKGoalSampler::Sample, Sample_overloads(args("ikreturn","releasegil"),DOXY_FN(planningutils::ManipulatorIKGoalSampler, Sample)))
        .def("SampleAll",&planningutils::PyManipulatorIKGoalSampler::SampleAll, SampleAll_overloads(args("maxsamples", "maxchecksamples", "releasegil"),DOXY_FN(planningutils::ManipulatorIKGoalSampler, SampleAll)))
        ;

        class_<planningutils::PyActiveDOFTrajectorySmoother, planningutils::PyActiveDOFTrajectorySmootherPtr >("ActiveDOFTrajectorySmoother", DOXY_CLASS(planningutils::ActiveDOFTrajectorySmoother), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(args("robot", "plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyActiveDOFTrajectorySmoother::PlanPath,PlanPath_overloads(args("traj","releasegil"), DOXY_FN(planningutils::ActiveDOFTrajectorySmoother,PlanPath)))
        ;

        class_<planningutils::PyActiveDOFTrajectoryRetimer, planningutils::PyActiveDOFTrajectoryRetimerPtr >("ActiveDOFTrajectoryRetimer", DOXY_CLASS(planningutils::ActiveDOFTrajectoryRetimer), no_init)
        .def(init<PyRobotBasePtr, const std::string&, const std::string&>(args("robot", "hastimestamps", "plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyActiveDOFTrajectoryRetimer::PlanPath,PlanPath_overloads3(args("traj","hastimestamps", "releasegil"), DOXY_FN(planningutils::ActiveDOFTrajectoryRetimer,PlanPath)))
        ;

        class_<planningutils::PyAffineTrajectoryRetimer, planningutils::PyAffineTrajectoryRetimerPtr >("AffineTrajectoryRetimer", DOXY_CLASS(planningutils::AffineTrajectoryRetimer), no_init)
        .def(init<const std::string&, const std::string&>(args("plannername", "plannerparameters")))
        .def("PlanPath",&planningutils::PyAffineTrajectoryRetimer::PlanPath,PlanPath_overloads2(args("traj","maxvelocities", "maxaccelerations", "hastimestamps", "releasegil"), DOXY_FN(planningutils::AffineTrajectoryRetimer,PlanPath)))
        ;
    }

    def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
    def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    def("RaveSetDataAccess",openravepy::pyRaveSetDataAccess,args("accessoptions"), DOXY_FN1(RaveSetDataAccess));
    def("RaveGetDataAccess",OpenRAVE::RaveGetDataAccess,DOXY_FN1(RaveGetDataAccess));
    def("RaveFindLocalFile",OpenRAVE::RaveFindLocalFile,RaveFindLocalFile_overloads(args("filename","curdir"), DOXY_FN1(RaveFindLocalFile)));
    def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
    def("RaveFindDatabaseFile",OpenRAVE::RaveFindDatabaseFile,DOXY_FN1(RaveFindDatabaseFile));
    def("RaveLogFatal",openravepy::raveLogFatal,args("log"),"Send a fatal log to the openrave system");
    def("RaveLogError",openravepy::raveLogError,args("log"),"Send an error log to the openrave system");
    def("RaveLogWarn",openravepy::raveLogWarn,args("log"),"Send a warn log to the openrave system");
    def("RaveLogInfo",openravepy::raveLogInfo,args("log"),"Send an info log to the openrave system");
    def("RaveLogDebug",openravepy::raveLogDebug,args("log"),"Send a debug log to the openrave system");
    def("RaveLogVerbose",openravepy::raveLogVerbose,args("log"),"Send a verbose log to the openrave system");
    def("RaveLog",openravepy::raveLog,args("log","level"),"Send a log to the openrave system with excplicit level");
    def("RaveInitialize",openravepy::pyRaveInitialize,RaveInitialize_overloads(args("load_all_plugins","level"),DOXY_FN1(RaveInitialize)));
    def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
    def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
    def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(RaveGetLoadedInterfaces));
    def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
    def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin,args("filename"),DOXY_FN1(RaveLoadPlugins));
    def("RaveHasInterface",OpenRAVE::RaveHasInterface,args("type","name"),DOXY_FN1(RaveHasInterface));
    def("RaveGlobalState",OpenRAVE::RaveGlobalState,DOXY_FN1(RaveGlobalState));
    def("RaveClone",openravepy::RaveClone,args("ref","cloningoptions"), DOXY_FN1(RaveClone));
    def("RaveGetIkTypeFromUniqueId",OpenRAVE::RaveGetIkTypeFromUniqueId,args("uniqueid"), DOXY_FN1(RaveGetIkTypeFromUniqueId));
    def("RaveGetIndexFromAffineDOF",OpenRAVE::RaveGetIndexFromAffineDOF, args("affinedofs","dof"), DOXY_FN1(RaveGetIndexFromAffineDOF));
    def("RaveGetAffineDOFFromIndex",OpenRAVE::RaveGetAffineDOFFromIndex, args("affinedofs","index"), DOXY_FN1(RaveGetAffineDOFFromIndex));
    def("RaveGetAffineDOF",OpenRAVE::RaveGetAffineDOF, args("affinedofs"), DOXY_FN1(RaveGetAffineDOF));
    def("RaveGetAffineDOFValuesFromTransform",openravepy::pyRaveGetAffineDOFValuesFromTransform, pyRaveGetAffineDOFValuesFromTransform_overloads(args("transform","affinedofs","rotationaxis"), DOXY_FN1(RaveGetAffineDOFValuesFromTransform)));
    def("RaveGetAffineConfigurationSpecification",openravepy::pyRaveGetAffineConfigurationSpecification, pyRaveGetAffineConfigurationSpecification_overloads(args("affinedofs","body","interpolation"), DOXY_FN1(RaveGetAffineConfigurationSpecification)));

    def("raveSetDebugLevel",openravepy::pyRaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
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
    def("quatMult",openravepy::quatMultiply,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("quatMultiply",openravepy::quatMultiply,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("quatInverse",openravepy::quatInverse,args("quat"),DOXY_FN1(quatInverse));
    def("poseMult",openravepy::poseMult,args("pose1","pose2"),"multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
    def("poseTransformPoints",openravepy::poseTransformPoints,args("pose","points"),"left-transforms a set of points by a pose transformation.\n\n:param pose: 7 values\n\n:param points: Nx3 values");
    def("transformLookat",openravepy::transformLookat,args("lookat","camerapos","cameraup"),"Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
    def("matrixSerialization",openravepy::matrixSerialization,args("transform"),"Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
    def("poseSerialization",openravepy::poseSerialization, args("pose"), "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
    def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion,"Returns the compiler version that openravepy_int was compiled with");
    def("normalizeAxisRotation",openravepy::normalizeAxisRotation,args("axis","quat"),DOXY_FN1(normalizeAxisRotation));
}

} // end namespace openravepy
