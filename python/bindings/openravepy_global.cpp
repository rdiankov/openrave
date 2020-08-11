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
#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_kinbody.h>
#include <openravepy/openravepy_environmentbase.h>
#include <openravepy/openravepy_configurationspecification.h>
#include <openrave/xmlreaders.h>
#include <openrave/utils.h>

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

std::string PyRay::__repr__() {
    return boost::str(boost::format("<Ray([%.15e,%.15e,%.15e],[%.15e,%.15e,%.15e])>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
std::string PyRay::__str__() {
    return boost::str(boost::format("<%.15e %.15e %.15e %.15e %.15e %.15e>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z);
}
object PyRay::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

class PyReadable
{
public:
    PyReadable(ReadablePtr readable) : _readable(readable) {
    }
    virtual ~PyReadable() {
    }
    std::string GetXMLId() const {
        // some readable are not xml readable and does have a xml id
        ReadablePtr pxmlreadable = OPENRAVE_DYNAMIC_POINTER_CAST<Readable>(_readable);
        if (!pxmlreadable) {
            return "";
        }
        return pxmlreadable->GetXMLId();
    }

    object SerializeXML(int options=0) {
        // some readable are not xml readable and does not get serialized here
        ReadablePtr pxmlreadable = OPENRAVE_DYNAMIC_POINTER_CAST<Readable>(_readable);
        if (!pxmlreadable) {
            return py::none_();
        }
        std::string xmlid;
        OpenRAVE::xmlreaders::StreamXMLWriter writer(xmlid);
        if( !pxmlreadable->SerializeXML(OpenRAVE::xmlreaders::StreamXMLWriterPtr(&writer,utils::null_deleter()),options) ) {
            return py::none_();
        }

        std::stringstream ss;
        writer.Serialize(ss);
        return ConvertStringToUnicode(ss.str());
    }

    py::object SerializeJSON(dReal fUnitScale=1.0, int options=0) const
    {
        ReadablePtr pjsonreadable = OPENRAVE_DYNAMIC_POINTER_CAST<Readable>(_readable);
        if (!pjsonreadable) {
            return py::none_();
        }
        rapidjson::Document doc;
        if( !pjsonreadable->SerializeJSON(doc, doc.GetAllocator(), fUnitScale, options) ) {
            return py::none_();
        }
        return toPyObject(doc);
    }

    bool DeserializeJSON(py::object obj, dReal fUnitScale=1.0)
    {
        rapidjson::Document doc;
        toRapidJSONValue(obj, doc, doc.GetAllocator());
        ReadablePtr pjsonreadable = OPENRAVE_DYNAMIC_POINTER_CAST<Readable>(_readable);
        return pjsonreadable->DeserializeJSON(doc, fUnitScale);
    }

    ReadablePtr GetReadable() {
        return _readable;
    }
protected:
    ReadablePtr _readable;
};

ReadablePtr ExtractReadable(object o) {
    if( !IS_PYTHONOBJECT_NONE(o) ) {
        extract_<PyReadablePtr> pyreadable(o);
        return ((PyReadablePtr)pyreadable)->GetReadable();
    }
    return ReadablePtr();
}

object toPyReadable(ReadablePtr p) {
    if( !p ) {
        return py::none_();
    }
    return py::to_object(PyReadablePtr(new PyReadable(p)));
}


class PyStringReaderStaticClass
{
public:
};

PyReadablePtr pyCreateStringReadable(const std::string& id, const std::string& data)
{
    return PyReadablePtr(new PyReadable(ReadablePtr(new OpenRAVE::StringReadable(id, data))));
}

namespace xmlreaders
{
class RAVE_DEPRECATED PyXMLReaderStaticClass
{
public:
};

PyReadablePtr RAVE_DEPRECATED pyCreateStringXMLReadable(const std::string& xmlid, const std::string& data)
{
    RAVELOG_WARN("CreateStringXMLReadable is deprecated. Use CreateStringReadable instead.");
    return pyCreateStringReadable(xmlid, data);
}

} // end namespace xmlreaders (deprecated)

object toPyGraphHandle(const GraphHandlePtr p)
{
    if( !p ) {
        return py::none_();
    }
    return py::to_object(PyGraphHandle(p));
}

object toPyUserData(UserDataPtr p)
{
    if( !p ) {
        return py::none_();
    }
    return py::to_object(PyUserData(p));
}

object toPyRay(const RAY& r)
{
    return py::to_object(OPENRAVE_SHARED_PTR<PyRay>(new PyRay(r)));
}

RAY ExtractRay(object o)
{
    extract_<OPENRAVE_SHARED_PTR<PyRay> > pyray(o);
    return ((OPENRAVE_SHARED_PTR<PyRay>)pyray)->r;
}

bool ExtractRay(object o, RAY& ray)
{
    extract_<OPENRAVE_SHARED_PTR<PyRay> > pyray(o);
    if( pyray.check() ) {
        ray = ((OPENRAVE_SHARED_PTR<PyRay>)pyray)->r;
        return true;
    }
    return false;
}

class Ray_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    : public pickle_suite
#endif
{
public:
    static py::tuple getinitargs(const PyRay& r)
    {
        return py::make_tuple(toPyVector3(r.r.pos),toPyVector3(r.r.dir));
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

    dict toDict() {
        dict d;
        d["pos"] = pos();
        d["extents"] = extents();
        return d;
    }

    virtual std::string __repr__() {
        return boost::str(boost::format("AABB([%.15e,%.15e,%.15e],[%.15e,%.15e,%.15e])")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual std::string __str__() {
        return boost::str(boost::format("<%.15e %.15e %.15e %.15e %.15e %.15e>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z);
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    AABB ab;
};

AABB ExtractAABB(object o)
{
    extract_<OPENRAVE_SHARED_PTR<PyAABB> > pyaabb(o);
    return ((OPENRAVE_SHARED_PTR<PyAABB>)pyaabb)->ab;
}

object toPyAABB(const AABB& ab)
{
    return py::to_object(OPENRAVE_SHARED_PTR<PyAABB>(new PyAABB(ab)));
}

class AABB_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    : public pickle_suite
#endif
{
public:
    static py::tuple getinitargs(const PyAABB& ab)
    {
        return py::make_tuple(toPyVector3(ab.ab.pos),toPyVector3(ab.ab.extents));
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
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        // vertices
        const int nvertices = mesh.vertices.size();
        py::array_t<dReal> pyvertices({nvertices, 3});
        py::buffer_info bufvertices = pyvertices.request();
        dReal* pvdata = (dReal*) bufvertices.ptr;
        for(const Vector& vertex : mesh.vertices) {
            *pvdata++ = vertex.x;
            *pvdata++ = vertex.y;
            *pvdata++ = vertex.z;
        }
        vertices = pyvertices;

        // indices
        const int nindices = mesh.indices.size()/3;
        py::array_t<int32_t> pyindices = toPyArray(mesh.indices);
        pyindices.resize({nindices, 3});
        indices = pyindices;
        // indices = py::array_t<int32_t>({nindices, 3});
        // py::buffer_info bufindices = indices.request();
        // int32_t* pidata = (int32_t*) bufindices.ptr;
        // std::memcpy(pidata, mesh.indices.data(), mesh.indices.size() * sizeof(int32_t));
#else // USE_PYBIND11_PYTHON_BINDINGS
        npy_intp dims[] = { npy_intp(mesh.vertices.size()), npy_intp(3)};
        PyObject *pyvertices = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pvdata = (dReal*)PyArray_DATA(pyvertices);
        FOREACHC(itv, mesh.vertices) {
            *pvdata++ = itv->x;
            *pvdata++ = itv->y;
            *pvdata++ = itv->z;
        }
        vertices = py::to_array_astype<dReal>(pyvertices);

        dims[0] = mesh.indices.size()/3;
        dims[1] = 3;
        PyObject *pyindices = PyArray_SimpleNew(2,dims, PyArray_INT32);
        int32_t* pidata = reinterpret_cast<int32_t*>PyArray_DATA(pyindices);
        std::memcpy(pidata, mesh.indices.data(), mesh.indices.size() * sizeof(int32_t));
        indices = py::to_array_astype<int32_t>(pyindices);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    }

    void GetTriMesh(TriMesh& mesh) {
        if( IS_PYTHONOBJECT_NONE(vertices) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("python TriMesh 'vertices' is not initialized correctly", ORE_InvalidState);
        }
        if( IS_PYTHONOBJECT_NONE(indices) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("python TriMesh 'indices' is not initialized correctly", ORE_InvalidState);
        }
        int numverts = len(vertices);
        mesh.vertices.resize(numverts);

        PyObject *pPyVertices = vertices.ptr();
        if (PyArray_Check(pPyVertices)) {
            if (PyArray_NDIM(pPyVertices) != 2) {
                throw openrave_exception(_("vertices must be a 2D array"), ORE_InvalidArguments);
            }
            if (!PyArray_ISFLOAT(pPyVertices)) {
                throw openrave_exception(_("vertices must be in float"), ORE_InvalidArguments);
            }
            PyArrayObject* pPyVerticesContiguous = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(pPyVertices));
            AutoPyArrayObjectDereferencer pydecref(pPyVerticesContiguous);

            const size_t typeSize = PyArray_ITEMSIZE(pPyVerticesContiguous);
            const size_t n = PyArray_DIM(pPyVerticesContiguous, 0);
            const size_t nElems = PyArray_DIM(pPyVerticesContiguous, 1);

            if (typeSize == sizeof(float)) {
                const float *vdata = reinterpret_cast<float*>(PyArray_DATA(pPyVerticesContiguous));

                for (size_t i = 0, j = 0; i < n; ++i, j += nElems) {
                    mesh.vertices[i].x = static_cast<dReal>(vdata[j + 0]);
                    mesh.vertices[i].y = static_cast<dReal>(vdata[j + 1]);
                    mesh.vertices[i].z = static_cast<dReal>(vdata[j + 2]);
                }
            } else if (typeSize == sizeof(double)) {
                const double *vdata = reinterpret_cast<double*>(PyArray_DATA(pPyVerticesContiguous));

                for (size_t i = 0, j = 0; i < n; ++i, j += nElems) {
                    mesh.vertices[i].x = static_cast<dReal>(vdata[j + 0]);
                    mesh.vertices[i].y = static_cast<dReal>(vdata[j + 1]);
                    mesh.vertices[i].z = static_cast<dReal>(vdata[j + 2]);
                }
            } else {
                throw openrave_exception(_("Unsupported vertices type"), ORE_InvalidArguments);
            }

        } else {
            for(int i = 0; i < numverts; ++i) {
                object ov = vertices[i];
                mesh.vertices[i].x = extract<dReal>(ov[0]);
                mesh.vertices[i].y = extract<dReal>(ov[1]);
                mesh.vertices[i].z = extract<dReal>(ov[2]);
            }
        }

        const size_t numtris = len(indices);
        mesh.indices.resize(3*numtris);
        PyObject *pPyIndices = indices.ptr();
        if (PyArray_Check(pPyIndices)) {
            if (PyArray_NDIM(pPyIndices) != 2 || PyArray_DIM(pPyIndices, 1) != 3 || !PyArray_ISINTEGER(pPyIndices)) {
                throw openrave_exception(_("indices must be a Nx3 int array"), ORE_InvalidArguments);
            }
            PyArrayObject* pPyIndiciesContiguous = PyArray_GETCONTIGUOUS(reinterpret_cast<PyArrayObject*>(pPyIndices));
            AutoPyArrayObjectDereferencer pydecref(pPyIndiciesContiguous);

            const size_t typeSize = PyArray_ITEMSIZE(pPyIndiciesContiguous);
            const bool signedInt = PyArray_ISSIGNED(pPyIndiciesContiguous);

            if (typeSize == sizeof(int32_t)) {
                if (signedInt) {
                    const int32_t *idata = reinterpret_cast<int32_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    std::memcpy(mesh.indices.data(), idata, numtris * 3 * sizeof(int32_t));
                } else {
                    const uint32_t *idata = reinterpret_cast<uint32_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                }
            } else if (typeSize == sizeof(int64_t)) {
                if (signedInt) {
                    const int64_t *idata = reinterpret_cast<int64_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                } else {
                    const uint64_t *idata = reinterpret_cast<uint64_t*>(PyArray_DATA(pPyIndiciesContiguous));
                    for (size_t i = 0; i < 3 * numtris; ++i) {
                        mesh.indices[i] = static_cast<int32_t>(idata[i]);
                    }
                }
            } else if (typeSize == sizeof(uint16_t) && !signedInt) {
                const uint16_t *idata = reinterpret_cast<uint16_t*>(PyArray_DATA(pPyIndiciesContiguous));
                for (size_t i = 0; i < 3 * numtris; ++i) {
                    mesh.indices[i] = static_cast<int32_t>(idata[i]);
                }
            } else {
                throw openrave_exception(_("Unsupported indices type"), ORE_InvalidArguments);
            }

        } else {
            for(size_t i = 0; i < numtris; ++i) {
                object oi = indices[i];
                mesh.indices[3*i+0] = extract<int32_t>(oi[0]);
                mesh.indices[3*i+1] = extract<int32_t>(oi[1]);
                mesh.indices[3*i+2] = extract<int32_t>(oi[2]);
            }
        }
    }

    std::string __str__() {
        return boost::str(boost::format("<trimesh: verts %d, tris=%d>")%len(vertices)%len(indices));
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    object vertices = py::none_();
    object indices = py::none_();
};

bool ExtractTriMesh(object o, TriMesh& mesh)
{
    extract_<OPENRAVE_SHARED_PTR<PyTriMesh> > pytrimesh(o);
    if( pytrimesh.check() ) {
        ((OPENRAVE_SHARED_PTR<PyTriMesh>)pytrimesh)->GetTriMesh(mesh);
        return true;
    }
    return false;
}

object toPyTriMesh(const TriMesh& mesh)
{
    return py::to_object(OPENRAVE_SHARED_PTR<PyTriMesh>(new PyTriMesh(mesh)));
}

class TriMesh_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    : public pickle_suite
#endif
{
public:
    static py::tuple getinitargs(const PyTriMesh& r)
    {
        return py::make_tuple(r.vertices,r.indices);
    }
};

PyConfigurationSpecification::PyConfigurationSpecification() {
}
PyConfigurationSpecification::PyConfigurationSpecification(const std::string &s) {
    std::stringstream ss(s);
    ss >> _spec;
}
PyConfigurationSpecification::PyConfigurationSpecification(const ConfigurationSpecification& spec) {
    _Update(spec);
}

void PyConfigurationSpecification::_Update(const ConfigurationSpecification& spec) {
    _spec = spec;
}
PyConfigurationSpecification::PyConfigurationSpecification(const ConfigurationSpecification::Group& g) {
    _spec = ConfigurationSpecification(g);
}
PyConfigurationSpecification::PyConfigurationSpecification(PyConfigurationSpecificationPtr pyspec) {
    _spec = pyspec->_spec;
}
PyConfigurationSpecification::~PyConfigurationSpecification() {
}

int PyConfigurationSpecification::GetDOF() const {
    return _spec.GetDOF();
}

bool PyConfigurationSpecification::IsValid() const {
    return _spec.IsValid();
}
void PyConfigurationSpecification::DeserializeJSON(object obj) {
    rapidjson::Document doc;
    toRapidJSONValue(obj, doc, doc.GetAllocator());
    ConfigurationSpecification spec;
    spec.DeserializeJSON(doc);
    _Update(spec);
}

object PyConfigurationSpecification::SerializeJSON() {
    rapidjson::Document doc;
    _spec.SerializeJSON(doc);
    return toPyObject(doc);
}

const ConfigurationSpecification::Group& PyConfigurationSpecification::GetGroupFromName(const std::string& name) {
    return _spec.GetGroupFromName(name);
}

object PyConfigurationSpecification::FindCompatibleGroup(const std::string& name, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindCompatibleGroup(name,exactmatch);
    if( it == _spec._vgroups.end() ) {
        return py::none_();
    }
    return py::to_object(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
}

object PyConfigurationSpecification::FindTimeDerivativeGroup(const std::string& name, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator it  = _spec.FindTimeDerivativeGroup(name,exactmatch);
    if( it == _spec._vgroups.end() ) {
        return py::none_();
    }
    return py::to_object(OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group>(new ConfigurationSpecification::Group(*it)));
}

//    ConfigurationSpecification GetTimeDerivativeSpecification(int timederivative) const;

void PyConfigurationSpecification::ResetGroupOffsets()
{
    _spec.ResetGroupOffsets();
}

void PyConfigurationSpecification::AddVelocityGroups(bool adddeltatime)
{
    RAVELOG_WARN("openravepy AddVelocityGroups is deprecated, use AddDerivativeGroups\n");
    _spec.AddDerivativeGroups(1,adddeltatime);
}

void PyConfigurationSpecification::AddDerivativeGroups(int deriv, bool adddeltatime)
{
    _spec.AddDerivativeGroups(deriv,adddeltatime);
}

int PyConfigurationSpecification::AddDeltaTimeGroup() {
    return _spec.AddDeltaTimeGroup();
}

int PyConfigurationSpecification::AddGroup(const std::string& name, int dof, const std::string& interpolation)
{
    return _spec.AddGroup(name,dof,interpolation);
}

int PyConfigurationSpecification::AddGroup(const ConfigurationSpecification::Group& g)
{
    return _spec.AddGroup(g);
}

int PyConfigurationSpecification::RemoveGroups(const std::string& groupname, bool exactmatch)
{
    return _spec.RemoveGroups(groupname, exactmatch);
}

PyConfigurationSpecificationPtr PyConfigurationSpecification::ConvertToVelocitySpecification() const
{
    return openravepy::toPyConfigurationSpecification(_spec.ConvertToVelocitySpecification());
}

PyConfigurationSpecificationPtr PyConfigurationSpecification::ConvertToDerivativeSpecification(uint32_t timederivative) const
{
    return openravepy::toPyConfigurationSpecification(_spec.ConvertToDerivativeSpecification(timederivative));
}

PyConfigurationSpecificationPtr PyConfigurationSpecification::GetTimeDerivativeSpecification(int timederivative) const
{
    return openravepy::toPyConfigurationSpecification(_spec.GetTimeDerivativeSpecification(timederivative));
}

object PyConfigurationSpecification::ExtractTransform(object otransform, object odata, PyKinBodyPtr pybody, int timederivative) const
{
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    Transform t;
    if( !IS_PYTHONOBJECT_NONE(otransform) ) {
        t = openravepy::ExtractTransform(otransform);
    }
    if( _spec.ExtractTransform(t,vdata.begin(),openravepy::GetKinBody(pybody)) ) {
        return openravepy::ReturnTransform(t);
    }
    return py::none_();
}

object PyConfigurationSpecification::ExtractIkParameterization(object odata, int timederivative, const std::string& robotname, const std::string& manipulatorname) const
{
    IkParameterization ikparam;
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    bool bfound = _spec.ExtractIkParameterization(ikparam, vdata.begin(), timederivative, robotname, manipulatorname);
    if( bfound ) {
        return toPyIkParameterization(ikparam);
    }
    else {
        return py::none_();
    }
}

object PyConfigurationSpecification::ExtractAffineValues(object odata, PyKinBodyPtr pybody, int affinedofs, int timederivative) const
{
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    std::vector<dReal> values(RaveGetAffineDOF(affinedofs),0);
    bool bfound = _spec.ExtractAffineValues(values.begin(),vdata.begin(),openravepy::GetKinBody(pybody),affinedofs, timederivative);
    if( bfound ) {
        return toPyArray(values);
    }
    else {
        return py::none_();
    }
}

object PyConfigurationSpecification::ExtractJointValues(object odata, PyKinBodyPtr pybody, object oindices, int timederivative) const
{
    std::vector<int> vindices = ExtractArray<int>(oindices);
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    std::vector<dReal> values(vindices.size(),0);
    bool bfound = _spec.ExtractJointValues(values.begin(),vdata.begin(),openravepy::GetKinBody(pybody),vindices,timederivative);
    if( bfound ) {
        return toPyArray(values);
    }
    else {
        return py::none_();
    }
}

object PyConfigurationSpecification::ExtractDeltaTime(object odata) const
{
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    dReal deltatime=0;
    bool bfound = _spec.ExtractDeltaTime(deltatime,vdata.begin());
    if( bfound ) {
        return py::to_object(deltatime);
    }
    else {
        return py::none_();
    }
}

bool PyConfigurationSpecification::InsertJointValues(object odata, object ovalues, PyKinBodyPtr pybody, object oindices, int timederivative) const
{
    std::vector<int> vindices = ExtractArray<int>(oindices);
    std::vector<dReal> vdata = ExtractArray<dReal>(odata);
    std::vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
    OPENRAVE_ASSERT_OP(vvalues.size(),==,vindices.size());
    OPENRAVE_ASSERT_OP(vdata.size(),>=,vvalues.size());
    OPENRAVE_ASSERT_OP((int)vdata.size(),==,_spec.GetDOF());
    if( !_spec.InsertJointValues(vdata.begin(), vvalues.begin(), openravepy::GetKinBody(pybody), vindices, timederivative) ) {
        return false;
    }
    // copy the value back, this is wasteful, but no other way to do it unless vdata pointer pointed directly to odata
    for(size_t i = 0; i < vdata.size(); ++i) {
        odata[i] = vdata[i];
    }
    return true;
}

bool PyConfigurationSpecification::InsertDeltaTime(object odata, dReal deltatime)
{
    // it is easier to get the time index
    FOREACHC(itgroup,_spec._vgroups) {
        if( itgroup->name == "deltatime" ) {
            odata[itgroup->offset] = py::to_object(deltatime);
            return true;
        }
    }
    return false;
}

py::list PyConfigurationSpecification::ExtractUsedBodies(PyEnvironmentBasePtr pyenv)
{
    std::vector<KinBodyPtr> vusedbodies;
    _spec.ExtractUsedBodies(openravepy::GetEnvironment(pyenv), vusedbodies);
    py::list obodies;
    FOREACHC(itbody, vusedbodies) {
        if( (*itbody)->IsRobot() ) {
            obodies.append(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(*itbody), pyenv));
        }
        else {
            obodies.append(openravepy::toPyKinBody(*itbody, pyenv));
        }
    }
    return obodies;
}

object PyConfigurationSpecification::ExtractUsedIndices(PyKinBodyPtr pybody)
{
    std::vector<int> useddofindices, usedconfigindices;
    _spec.ExtractUsedIndices(openravepy::GetKinBody(pybody), useddofindices, usedconfigindices);
    return py::make_tuple(toPyArray(useddofindices), toPyArray(usedconfigindices));
}

//
//    static void ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv);
//
// source spec is the current configurationspecification spec
object PyConfigurationSpecification::ConvertData(PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv, bool filluninitialized)
{
    std::vector<dReal> vtargetdata(pytargetspec->_spec.GetDOF()*numpoints,0);
    std::vector<dReal> vsourcedata = ExtractArray<dReal>(osourcedata);
    ConfigurationSpecification::ConvertData(vtargetdata.begin(), pytargetspec->_spec, vsourcedata.begin(), _spec, numpoints, openravepy::GetEnvironment(pyenv), filluninitialized);
    return toPyArray(vtargetdata);
}

object PyConfigurationSpecification::ConvertDataFromPrevious(object otargetdata, PyConfigurationSpecificationPtr pytargetspec, object osourcedata, size_t numpoints, PyEnvironmentBasePtr pyenv)
{
    std::vector<dReal> vtargetdata = ExtractArray<dReal>(otargetdata);
    std::vector<dReal> vsourcedata = ExtractArray<dReal>(osourcedata);
    ConfigurationSpecification::ConvertData(vtargetdata.begin(), pytargetspec->_spec, vsourcedata.begin(), _spec, numpoints, openravepy::GetEnvironment(pyenv), false);
    return toPyArray(vtargetdata);
}

py::list PyConfigurationSpecification::GetGroups()
{
    py::list ogroups;
    FOREACHC(itgroup, _spec._vgroups) {
        ogroups.append(*itgroup);
    }
    return ogroups;
}

bool PyConfigurationSpecification::__eq__(PyConfigurationSpecificationPtr p) {
    return !!p && _spec==p->_spec;
}
bool PyConfigurationSpecification::__ne__(PyConfigurationSpecificationPtr p) {
    return !p || _spec!=p->_spec;
}

PyConfigurationSpecificationPtr PyConfigurationSpecification::__add__(PyConfigurationSpecificationPtr r)
{
    return PyConfigurationSpecificationPtr(new PyConfigurationSpecification(_spec + r->_spec));
}

PyConfigurationSpecificationPtr PyConfigurationSpecification::__iadd__(PyConfigurationSpecificationPtr r)
{
    _spec += r->_spec;
    return shared_from_this();
}

std::string PyConfigurationSpecification::__repr__() {
    std::stringstream ss;
    ss << "ConfigurationSpecification(\"\"\"" << _spec << "\"\"\")";
    return ss.str();
}
std::string PyConfigurationSpecification::__str__() {
    std::stringstream ss;
    ss << "<configuration dof=\"" << _spec.GetDOF() << "\">";
    return ss.str();
}
object PyConfigurationSpecification::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

class OPENRAVEPY_API ConfigurationSpecification_pickle_suite
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    : public pickle_suite
#endif
{
public:
    static py::tuple getinitargs(const PyConfigurationSpecification& pyspec)
    {
        std::stringstream ss;
        ss << pyspec._spec;
        return py::make_tuple(ss.str());
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

#ifndef USE_PYBIND11_PYTHON_BINDINGS
struct spec_from_group
{
    spec_from_group()
    {
        py::converter::registry::push_back(&convertible, &construct, py::type_id<PyConfigurationSpecificationPtr>());
    }

    static void* convertible(PyObject* obj)
    {
        return obj == Py_None ||  py::extract_<ConfigurationSpecification::Group>(obj).check() ? obj : NULL;
    }

    static void construct(PyObject* obj, py::converter::rvalue_from_python_stage1_data* data)
    {
        ConfigurationSpecification::Group g = (ConfigurationSpecification::Group)py::extract<ConfigurationSpecification::Group>(obj);
        void* storage = ((py::converter::rvalue_from_python_storage<PyConfigurationSpecificationPtr>*)data)->storage.bytes;
        new (storage) PyConfigurationSpecificationPtr(new PyConfigurationSpecification(g));
        data->convertible = storage;
    }
};
#endif // USE_PYBIND11_PYTHON_BINDINGS

PyConfigurationSpecificationPtr pyRaveGetAffineConfigurationSpecification(int affinedofs,PyKinBodyPtr pybody=PyKinBodyPtr(), const std::string& interpolation="")
{
    return openravepy::toPyConfigurationSpecification(RaveGetAffineConfigurationSpecification(affinedofs,openravepy::GetKinBody(pybody), interpolation));
}

object pyRaveGetAffineDOFValuesFromTransform(object otransform, int affinedofs, object oActvAffineRotationAxis=py::none_())
{
    Vector vActvAffineRotationAxis(0,0,1);
    if( !IS_PYTHONOBJECT_NONE(oActvAffineRotationAxis) ) {
        vActvAffineRotationAxis = ExtractVector3(oActvAffineRotationAxis);
    }
    std::vector<dReal> values(RaveGetAffineDOF(affinedofs));
    RaveGetAffineDOFValuesFromTransform(values.begin(),ExtractTransform(otransform), affinedofs, vActvAffineRotationAxis);
    return toPyArray(values);
}

std::string openravepyCompilerVersion()
{
    std::stringstream ss;
#if defined(_MSC_VER)
    ss << "msvc " << _MSC_VER;
#elif defined(__GNUC__)
    ss << "gcc " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__;
#elif defined(__MINGW32_VERSION)
    ss << "mingw " << __MINGW32_VERSION;
#endif
    return ss.str();
}

void raveLog(const std::string &s, int level)
{
    if( s.size() > 0 ) {
        RavePrintfA(s,level);
    }
}

void raveLogFatal(const std::string &s)
{
    raveLog(s,Level_Fatal);
}
void raveLogError(const std::string &s)
{
    raveLog(s,Level_Error);
}
void raveLogWarn(const std::string &s)
{
    raveLog(s,Level_Warn);
}
void raveLogInfo(const std::string &s)
{
    raveLog(s,Level_Info);
}
void raveLogDebug(const std::string &s)
{
    raveLog(s,Level_Debug);
}
void raveLogVerbose(const std::string &s)
{
    raveLog(s,Level_Verbose);
}

int pyGetIntFromPy(object olevel, int defaultvalue)
{
    int level = defaultvalue;
    if( !IS_PYTHONOBJECT_NONE(olevel) ) {
        // some version of boost python return true for extract::check, even through the actual conversion will throw an OverflowError
        // therefore check for conversion compatibility starting at the longest signed integer
        extract_<int64_t> levelint64(olevel);
        if( levelint64.check() ) {
            level = static_cast<int>((int64_t)levelint64);
        }
        else {
            extract_<uint64_t> leveluint64(olevel);
            if( leveluint64.check() ) {
                level = static_cast<int>((uint64_t)leveluint64);
            }
            else {
                extract_<uint32_t> leveluint32(olevel);
                if( leveluint32.check() ) {
                    level = static_cast<int>((uint32_t)leveluint32);
                }
                else {
                    extract_<int> levelint32(olevel);
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

int pyRaveInitialize(bool bLoadAllPlugins=true, object olevel=py::none_())
{

    return OpenRAVE::RaveInitialize(bLoadAllPlugins,pyGetIntFromPy(olevel, Level_Info));
}

void pyRaveSetDataAccess(object oaccess)
{
    OpenRAVE::RaveSetDataAccess(pyGetIntFromPy(oaccess, Level_Info));
}

// return None if nothing found
object pyRaveInvertFileLookup(const std::string& filename)
{
    std::string newfilename;
    if( OpenRAVE::RaveInvertFileLookup(newfilename, filename) ) {
        return ConvertStringToUnicode(newfilename);
    }
    return py::none_();
}

object RaveGetPluginInfo()
{
    py::list plugins;
    std::list< std::pair<std::string, PLUGININFO> > listplugins;
    OpenRAVE::RaveGetPluginInfo(listplugins);
    FOREACH(itplugin, listplugins) {
        plugins.append(py::make_tuple(itplugin->first, py::to_object(OPENRAVE_SHARED_PTR<PyPluginInfo>(new PyPluginInfo(itplugin->second)))));
    }
    return plugins;
}

object RaveGetLoadedInterfaces()
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    py::dict ointerfacenames;
    FOREACHC(it, interfacenames) {
        py::list names;
        FOREACHC(itname,it->second) {
            names.append(*itname);
        }
        ointerfacenames[it->first] = names;
    }
    return ointerfacenames;
}

PyInterfaceBasePtr pyRaveClone(PyInterfaceBasePtr pyreference, int cloningoptions, PyEnvironmentBasePtr pyenv=PyEnvironmentBasePtr())
{
    if( !pyenv ) {
        pyenv = pyreference->GetEnv();
    }
    EnvironmentBasePtr penv = openravepy::GetEnvironment(pyenv);
    InterfaceBasePtr pclone = OpenRAVE::RaveClone<InterfaceBase>(pyreference->GetInterfaceBase(), cloningoptions, penv);
    switch(pclone->GetInterfaceType()) {
    case PT_Planner: return toPyPlanner(RaveInterfaceCast<PlannerBase>(pclone), pyenv);
    case PT_Robot: return toPyRobot(RaveInterfaceCast<RobotBase>(pclone), pyenv);
    case PT_SensorSystem: return toPySensorSystem(RaveInterfaceCast<SensorSystemBase>(pclone), pyenv);
    case PT_Controller: return toPyController(RaveInterfaceCast<ControllerBase>(pclone), pyenv);
    case PT_Module: return toPyModule(RaveInterfaceCast<ModuleBase>(pclone), pyenv);
    case PT_InverseKinematicsSolver: return toPyIkSolver(RaveInterfaceCast<IkSolverBase>(pclone), pyenv);
    case PT_KinBody: return toPyKinBody(RaveInterfaceCast<KinBody>(pclone), pyenv);
    case PT_PhysicsEngine: return toPyPhysicsEngine(RaveInterfaceCast<PhysicsEngineBase>(pclone), pyenv);
    case PT_Sensor: return toPySensor(RaveInterfaceCast<SensorBase>(pclone), pyenv);
    case PT_CollisionChecker: return toPyCollisionChecker(RaveInterfaceCast<CollisionCheckerBase>(pclone), pyenv);
    case PT_Trajectory: return toPyTrajectory(RaveInterfaceCast<TrajectoryBase>(pclone), pyenv);
    case PT_Viewer: return toPyViewer(RaveInterfaceCast<ViewerBase>(pclone), pyenv);
    case PT_SpaceSampler: return toPySpaceSampler(RaveInterfaceCast<SpaceSamplerBase>(pclone), pyenv);
    }
    throw openrave_exception(_("invalid interface type"),ORE_InvalidArguments);
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

object InterpolateQuatSlerp(object q1, object q2, dReal t, bool forceshortarc=true)
{
    return toPyVector4(InterpolateQuatSlerp(ExtractVector4(q1),ExtractVector4(q2),t, forceshortarc));
}

object InterpolateQuatSquad(object q0, object q1, object q2, object q3, dReal t, bool forceshortarc=true)
{
    return toPyVector4(InterpolateQuatSquad(ExtractVector4(q0),ExtractVector4(q1),ExtractVector4(q2),ExtractVector4(q3), t, forceshortarc));
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
    py::list orots;
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
    py::list omatrices;
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
    const int N = len(otransforms);
    if( N == 0 ) {
        return py::empty_array_astype<dReal>();
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pyvalues({N, 7});
    py::buffer_info buf = pyvalues.request();
    dReal* pvalues = (dReal*) buf.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { N,7};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pvalues = (dReal*)PyArray_DATA(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    TransformMatrix tm;
    for(int j = 0; j < N; ++j) {
        object o = otransforms[j];
        for(int i = 0; i < 3; ++i) {
            tm.m[4*i+0] = extract<dReal>(o[i][0]);
            tm.m[4*i+1] = extract<dReal>(o[i][1]);
            tm.m[4*i+2] = extract<dReal>(o[i][2]);
            tm.trans[i] = extract<dReal>(o[i][3]);
        }
        // convert 4x4 transform matrix (stored as 3x4) to quat+trans (4+3) form
        Transform tpose(tm);
        pvalues[0] = tpose.rot.x; pvalues[1] = tpose.rot.y; pvalues[2] = tpose.rot.z; pvalues[3] = tpose.rot.w;
        pvalues[4] = tpose.trans.x; pvalues[5] = tpose.trans.y; pvalues[6] = tpose.trans.z;
        pvalues += 7;
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return pyvalues;
#else
    return py::to_array_astype<dReal>(pyvalues);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object InvertPoses(object o)
{
    const int N = len(o);
    if( N == 0 ) {
        return py::empty_array_astype<dReal>();
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pytrans({N, 7});
    py::buffer_info buf = pytrans.request();
    dReal* ptrans = (dReal*) buf.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { N,7};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    for(int i = 0; i < N; ++i, ptrans += 7) {
        object oinputtrans = o[i];
        Transform t = Transform(Vector(extract<dReal>(oinputtrans[0]),extract<dReal>(oinputtrans[1]),extract<dReal>(oinputtrans[2]),extract<dReal>(oinputtrans[3])),
                                Vector(extract<dReal>(oinputtrans[4]),extract<dReal>(oinputtrans[5]),extract<dReal>(oinputtrans[6]))).inverse();
        ptrans[0] = t.rot.x; ptrans[1] = t.rot.y; ptrans[2] = t.rot.z; ptrans[3] = t.rot.w;
        ptrans[4] = t.trans.x; ptrans[5] = t.trans.y; ptrans[6] = t.trans.z;
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return pytrans;
#else
    return py::to_array_astype<dReal>(pytrans);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object InvertPose(object opose)
{
    Transform t = ExtractTransformType<dReal>(opose);
    return toPyArray(t.inverse());
}

object quatRotateDirection(object source, object target)
{
    return toPyVector4(quatRotateDirection(ExtractVector3(source), ExtractVector3(target)));
}

object ExtractAxisFromQuat(object oquat, int iaxis)
{
    return toPyVector3(ExtractAxisFromQuat(ExtractVector4(oquat), iaxis));
}

object normalizeAxisRotation(object axis, object quat)
{
    std::pair<dReal, Vector > res = normalizeAxisRotation(ExtractVector3(axis), ExtractVector4(quat));
    return py::make_tuple(res.first,toPyVector4(res.second));
}

object MultiplyQuat(object oquat1, object oquat2)
{
    return toPyVector4(OpenRAVE::geometry::quatMultiply(ExtractVector4(oquat1),ExtractVector4(oquat2)));
}

object InvertQuat(object oquat)
{
    return toPyVector4(OpenRAVE::geometry::quatInverse(ExtractVector4(oquat)));
}

object MultiplyPose(object opose1, object opose2)
{
    return toPyArray(ExtractTransformType<dReal>(opose1)*ExtractTransformType<dReal>(opose2));
}

object poseTransformPoint(object opose, object opoint)
{
    Transform t = ExtractTransformType<dReal>(opose);
    Vector newpoint = t*ExtractVector3(opoint);
    return toPyVector3(newpoint);
}

object poseTransformPoints(object opose, object opoints)
{
    const Transform t = ExtractTransformType<dReal>(opose);
    const int N = len(opoints);
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pytrans({N, 3});
    py::buffer_info buf = pytrans.request();
    dReal* ptrans = (dReal*) buf.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { N,3};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    for(int i = 0; i < N; ++i, ptrans += 3) {
        Vector newpoint = t*ExtractVector3(opoints[i]);
        ptrans[0] = newpoint.x; ptrans[1] = newpoint.y; ptrans[2] = newpoint.z;
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return pytrans;
#else
    return py::to_array_astype<dReal>(pytrans);
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

object TransformLookat(object olookat, object ocamerapos, object ocameraup)
{
    return toPyArray(transformLookat(ExtractVector3(olookat),ExtractVector3(ocamerapos),ExtractVector3(ocameraup)));
}

dReal ComputePoseDistSqr(object opose0, object opose1, dReal quatweight=1.0)
{
    Transform t0 = ExtractTransformType<dReal>(opose0);
    Transform t1 = ExtractTransformType<dReal>(opose1);
    dReal e1 = (t0.rot-t1.rot).lengthsqr4();
    dReal e2 = (t0.rot+t1.rot).lengthsqr4();
    dReal e = e1 < e2 ? e1 : e2;
    return (t0.trans-t1.trans).lengthsqr3() + quatweight*e;
}

string matrixSerialization(object o)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransformMatrix(o);
    return ss.str();
}

string poseSerialization(object o)
{
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << ExtractTransform(o);
    return ss.str();
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_FUNCTION_OVERLOADS(RaveInitialize_overloads, pyRaveInitialize, 0, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(RaveFindLocalFile_overloads, OpenRAVE::RaveFindLocalFile, 1, 2)
BOOST_PYTHON_FUNCTION_OVERLOADS(InterpolateQuatSlerp_overloads, openravepy::InterpolateQuatSlerp, 3, 4)
BOOST_PYTHON_FUNCTION_OVERLOADS(InterpolateQuatSquad_overloads, openravepy::InterpolateQuatSquad, 5, 6)
BOOST_PYTHON_FUNCTION_OVERLOADS(ComputePoseDistSqr_overloads, openravepy::ComputePoseDistSqr, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(pyRaveGetAffineConfigurationSpecification_overloads, openravepy::pyRaveGetAffineConfigurationSpecification, 1, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(pyRaveGetAffineDOFValuesFromTransform_overloads, openravepy::pyRaveGetAffineDOFValuesFromTransform, 2, 3)
BOOST_PYTHON_FUNCTION_OVERLOADS(RaveClone_overloads, pyRaveClone, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractTransform_overloads, PyConfigurationSpecification::ExtractTransform, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractIkParameterization_overloads, PyConfigurationSpecification::ExtractIkParameterization, 1, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractAffineValues_overloads, PyConfigurationSpecification::ExtractAffineValues, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ExtractJointValues_overloads, PyConfigurationSpecification::ExtractJointValues, 3, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(RemoveGroups_overloads, PyConfigurationSpecification::RemoveGroups, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SerializeXML_overloads, SerializeXML, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SerializeJSON_overloads, SerializeJSON, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(DeserializeJSON_overloads, DeserializeJSON, 1, 2)
#endif // USE_PYBIND11_PYTHON_BINDINGS

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_global(py::module& m)
#else
void init_openravepy_global()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    enum_<OpenRAVEErrorCode>(m, "ErrorCode", py::arithmetic() DOXY_ENUM(OpenRAVEErrorCode))
#else
    enum_<OpenRAVEErrorCode>("ErrorCode" DOXY_ENUM(OpenRAVEErrorCode))
#endif
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
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<DebugLevel>(m, "DebugLevel", py::arithmetic() DOXY_ENUM(DebugLevel))
#else
    enum_<DebugLevel>("DebugLevel" DOXY_ENUM(DebugLevel))
#endif
    .value("Fatal",Level_Fatal)
    .value("Error",Level_Error)
    .value("Warn",Level_Warn)
    .value("Info",Level_Info)
    .value("Debug",Level_Debug)
    .value("Verbose",Level_Verbose)
    .value("VerifyPlans",Level_VerifyPlans)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<SerializationOptions>(m, "SerializationOptions", py::arithmetic() DOXY_ENUM(SerializationOptions))
#else
    enum_<SerializationOptions>("SerializationOptions" DOXY_ENUM(SerializationOptions))
#endif
    .value("Kinematics",SO_Kinematics)
    .value("Dynamics",SO_Dynamics)
    .value("BodyState",SO_BodyState)
    .value("NamesAndFiles",SO_NamesAndFiles)
    .value("RobotManipulators",SO_RobotManipulators)
    .value("RobotSensors",SO_RobotSensors)
    .value("Geometry",SO_Geometry)
    .value("InverseKinematics",SO_InverseKinematics)
    .value("JointLimits",SO_JointLimits)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<InterfaceType>(m, "InterfaceType", py::arithmetic() DOXY_ENUM(InterfaceType))
#else
    enum_<InterfaceType>("InterfaceType" DOXY_ENUM(InterfaceType))
#endif
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
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<CloningOptions>(m, "CloningOptions", py::arithmetic() DOXY_ENUM(CloningOptions))
#else
    enum_<CloningOptions>("CloningOptions" DOXY_ENUM(CloningOptions))
#endif
    .value("Bodies",Clone_Bodies)
    .value("Viewer",Clone_Viewer)
    .value("Simulation",Clone_Simulation)
    .value("RealControllers",Clone_RealControllers)
    .value("Sensors",Clone_Sensors)
    .value("Modules",Clone_Modules)
    .value("IgnoreAttachedBodies", Clone_IgnoreAttachedBodies)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // Cannot export because openravepy_viewer already has "Viewer"
    // .export_values()
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<PhysicsEngineOptions>(m, "PhysicsEngineOptions", py::arithmetic() DOXY_ENUM(PhysicsEngineOptions))
#else
    enum_<PhysicsEngineOptions>("PhysicsEngineOptions" DOXY_ENUM(PhysicsEngineOptions))
#endif
    .value("SelfCollisions",PEO_SelfCollisions)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<IntervalType>(m, "Interval", py::arithmetic() DOXY_ENUM(IntervalType))
#else
    enum_<IntervalType>("Interval" DOXY_ENUM(IntervalType))
#endif
    .value("Open",IT_Open)
    .value("OpenStart",IT_OpenStart)
    .value("OpenEnd",IT_OpenEnd)
    .value("Closed",IT_Closed)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<SampleDataType>(m, "SampleDataType", py::arithmetic() DOXY_ENUM(SampleDataType))
#else
    enum_<SampleDataType>("SampleDataType" DOXY_ENUM(SampleDataType))
#endif
    .value("Real",SDT_Real)
    .value("Uint32",SDT_Uint32)
// #ifdef USE_PYBIND11_PYTHON_BINDINGS
//     .export_values()
// #endif
    ;
// #ifdef USE_PYBIND11_PYTHON_BINDINGS
//     class_<UserData, UserDataPtr >(m, "UserData", DOXY_CLASS(UserData))
// #else
//     class_<UserData, UserDataPtr >("UserData", DOXY_CLASS(UserData))
// #endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_< OPENRAVE_SHARED_PTR< void > >(m, "VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");
#else
    class_< OPENRAVE_SHARED_PTR< void > >("VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyGraphHandle, OPENRAVE_SHARED_PTR<PyGraphHandle> >(m, "GraphHandle", DOXY_CLASS(GraphHandle))
    .def(init<>())
    .def(init<GraphHandlePtr>(), "handle"_a)
#else
    class_<PyGraphHandle, OPENRAVE_SHARED_PTR<PyGraphHandle> >("GraphHandle", DOXY_CLASS(GraphHandle), no_init)
#endif
    .def("SetTransform",&PyGraphHandle::SetTransform,DOXY_FN(GraphHandle,SetTransform))
    .def("SetShow",&PyGraphHandle::SetShow,DOXY_FN(GraphHandle,SetShow))
    .def("Close",&PyGraphHandle::Close,DOXY_FN(GraphHandle,Close))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyUserData, OPENRAVE_SHARED_PTR<PyUserData> >(m, "UserData", DOXY_CLASS(UserData))
    .def(init<>())
    .def(init<UserDataPtr>(), "handle"_a)
#else
    class_<PyUserData, OPENRAVE_SHARED_PTR<PyUserData> >("UserData", DOXY_CLASS(UserData), no_init)
#endif
    .def("close",&PyUserData::Close,"deprecated")
    .def("Close",&PyUserData::Close,"force releasing the user handle point.")
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PySerializableData, OPENRAVE_SHARED_PTR<PySerializableData>, PyUserData >(m, "SerializableData", DOXY_CLASS(SerializableData))
    .def(init<>())
    .def(init<SerializableDataPtr>(), "handle"_a)
#else
    class_<PySerializableData, OPENRAVE_SHARED_PTR<PySerializableData>, bases<PyUserData> >("SerializableData", DOXY_CLASS(SerializableData))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def(init<const std::string&>(), "data"_a)
#else
    .def(init<const std::string&>(py::args("data")))
#endif
    .def("Close",&PySerializableData::Close,DOXY_FN(SerializableData,Close))
    .def("Serialize",&PySerializableData::Serialize, PY_ARGS("options") DOXY_FN(SerializableData, Serialize))
    .def("Deserialize",&PySerializableData::Deserialize, PY_ARGS("data") DOXY_FN(SerializableData, Deserialize))
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyRay, OPENRAVE_SHARED_PTR<PyRay> >(m, "Ray", DOXY_CLASS(geometry::ray))
    .def(init<>())
    .def(init<object, object>(), "pos"_a, "dir"_a)
    .def(init<const RAY&>(), "r"_a)
#else
    class_<PyRay, OPENRAVE_SHARED_PTR<PyRay> >("Ray", DOXY_CLASS(geometry::ray))
    .def(init<object,object>(py::args("pos","dir")))
#endif
    .def("dir",&PyRay::dir)
    .def("pos",&PyRay::pos)
    .def("__str__",&PyRay::__str__)
    .def("__unicode__",&PyRay::__unicode__)
    .def("__repr__",&PyRay::__repr__)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def(py::pickle(
             [](const PyRay &pyr) {
            // __getstate__
            return Ray_pickle_suite::getinitargs(pyr);
        },
             [](py::tuple state) {
            // __setstate__
            if (state.size() != 2) {
                throw std::runtime_error("Invalid state!");
            }
            /* Create a new C++ instance */
            PyRay pyr;
            /* Assign any additional state */
            py::array_t<dReal> pos = state[0].cast<py::array_t<dReal> >();
            py::array_t<dReal> dir = state[1].cast<py::array_t<dReal> >();
            for(size_t i = 0; i < 3; ++i) {
                pyr.r.pos[i] = *pos.data(i);
                pyr.r.dir[i] = *dir.data(i);
            }
            pyr.r.pos[3] = pyr.r.dir[3] = 0;
            return pyr;
        }
             ))
#else
    .def_pickle(Ray_pickle_suite())
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyAABB, OPENRAVE_SHARED_PTR<PyAABB> >(m, "AABB", DOXY_CLASS(geometry::aabb))
    .def(init<>())
    .def(init<object, object>(), "pos"_a, "extents"_a)
    .def(init<const AABB&>(), "ab"_a)
    .def("__copy__", [](const PyAABB& self){
            return self;
        })
    .def("__deepcopy__", [](const PyAABB& self, const py::dict& memo) {
            OPENRAVE_SHARED_PTR<PyAABB> pyaabb(new PyAABB(self.ab));
            return py::to_object(pyaabb);
        })
#else
    class_<PyAABB, OPENRAVE_SHARED_PTR<PyAABB> >("AABB", DOXY_CLASS(geometry::aabb))
    .def(init<object,object>(py::args("pos","extents")))
#endif
    .def("extents",&PyAABB::extents)
    .def("pos",&PyAABB::pos)
    .def("__str__",&PyAABB::__str__)
    .def("__unicode__",&PyAABB::__unicode__)
    .def("__repr__",&PyAABB::__repr__)
    .def("toDict", &PyAABB::toDict)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def(py::pickle(
             [](const PyAABB &pyab) {
            // __getstate__
            return AABB_pickle_suite::getinitargs(pyab);
        },
             [](py::tuple state) {
            // __setstate__
            if (state.size() != 2) {
                throw std::runtime_error("Invalid state!");
            }
            /* Create a new C++ instance */
            PyAABB pyab;
            /* Assign any additional state */
            py::array_t<dReal> pos = state[0].cast<py::array_t<dReal> >();
            py::array_t<dReal> extents = state[1].cast<py::array_t<dReal> >();
            for(size_t i = 0; i < 3; ++i) {
                pyab.ab.pos[i] = *pos.data(i);
                pyab.ab.extents[i] = *extents.data(i);
            }
            pyab.ab.pos[3] = pyab.ab.extents[3] = 0;
            return pyab;
        }
             ))
#else
    .def_pickle(AABB_pickle_suite())
#endif // USE_PYBIND11_PYTHON_BINDINGS
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyTriMesh, OPENRAVE_SHARED_PTR<PyTriMesh> >(m, "TriMesh", DOXY_CLASS(TriMesh))
    .def(init<>())
    .def(init<object, object>(), "vertices"_a, "indices"_a)
    .def(init<const TriMesh&>(), "mesh"_a)
#else
    class_<PyTriMesh, OPENRAVE_SHARED_PTR<PyTriMesh> >("TriMesh", DOXY_CLASS(TriMesh))
    .def(init<object,object>(py::args("vertices","indices")))
#endif
    .def_readwrite("vertices",&PyTriMesh::vertices)
    .def_readwrite("indices",&PyTriMesh::indices)
    .def("__str__",&PyTriMesh::__str__)
    .def("__unicode__",&PyTriMesh::__unicode__)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def(py::pickle([](const PyTriMesh &pymesh) {
            // __getstate__
            return TriMesh_pickle_suite::getinitargs(pymesh);
        },
                    [](py::tuple state) {
            PyTriMesh pymesh;
            pymesh.vertices = state[0];
            pymesh.indices = state[1];
            return pymesh;
        }))
#else
    .def_pickle(TriMesh_pickle_suite())
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<InterfaceBase, InterfaceBasePtr>(m, "InterfaceBase", DOXY_CLASS(InterfaceBase))
#else
    class_<InterfaceBase, InterfaceBasePtr, boost::noncopyable >("InterfaceBase", DOXY_CLASS(InterfaceBase), no_init)
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyReadable, PyReadablePtr >(m, "Readable", DOXY_CLASS(eadable))
    .def(init<ReadablePtr>(), "readableraw"_a)
#else
    class_<PyReadable, PyReadablePtr >("Readable", DOXY_CLASS(eadable), no_init)
    .def(init<ReadablePtr>(py::args("readableraw")))
#endif
    .def("GetXMLId", &PyReadable::GetXMLId, DOXY_FN(eadable, GetXMLId))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def("SerializeXML", &PyReadable::SerializeXML,
         "options"_a = 0,
         DOXY_FN(eadable, Serialize)
         )
#else
    .def("SerializeXML", &PyReadable::SerializeXML, SerializeXML_overloads(PY_ARGS("options") DOXY_FN(Readable, Serialize)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def("SerializeJSON", &PyReadable::SerializeJSON,
        "unitScale"_a = 1.0,
        "options"_a = py::none_(),
        DOXY_FN(Readable, SerializeJSON)
    )
    .def("DeserializeJSON", &PyReadable::DeserializeJSON,
        "obj"_a,
        "unitScale"_a = 1.0,
        DOXY_FN(Readable, DeserializeJSON)
    )
#else
    .def("SerializeJSON", &PyReadable::SerializeJSON, SerializeJSON_overloads(PY_ARGS("unitScale", "options") DOXY_FN(Readable, SerializeJSON)))
    .def("DeserializeJSON", &PyReadable::DeserializeJSON, DeserializeJSON_overloads(PY_ARGS("obj", "unitScale") DOXY_FN(Readable, DeserializeJSON)))
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyPluginInfo, OPENRAVE_SHARED_PTR<PyPluginInfo> >(m, "PluginInfo", DOXY_CLASS(PLUGININFO))
    .def(init<const PLUGININFO&>(), "info"_a)
#else
    class_<PyPluginInfo, OPENRAVE_SHARED_PTR<PyPluginInfo> >("PluginInfo", DOXY_CLASS(PLUGININFO),no_init)
#endif
    .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
    .def_readonly("version",&PyPluginInfo::version)
    ;

    {
        int (PyConfigurationSpecification::*addgroup1)(const std::string&, int, const std::string&) = &PyConfigurationSpecification::AddGroup;
        int (PyConfigurationSpecification::*addgroup2)(const ConfigurationSpecification::Group&) = &PyConfigurationSpecification::AddGroup;

        scope_ configurationspecification =
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            class_<PyConfigurationSpecification, PyConfigurationSpecificationPtr >(m, "ConfigurationSpecification",DOXY_CLASS(ConfigurationSpecification))
            .def(init<>())
            .def(init<const ConfigurationSpecification&>(), "spec"_a)
            .def(init<PyConfigurationSpecificationPtr>(), "pyspec"_a)
            .def(init<const ConfigurationSpecification::Group&>(), "group"_a)
            .def(init<const std::string&>(), "xmldata"_a)
            .def("GetGroupFromName", &PyConfigurationSpecification::GetGroupFromName, DOXY_FN(ConfigurationSpecification,GetGroupFromName))
#else
            class_<PyConfigurationSpecification, PyConfigurationSpecificationPtr >("ConfigurationSpecification",DOXY_CLASS(ConfigurationSpecification))
            .def(init<PyConfigurationSpecificationPtr>(py::args("spec")) )
            .def(init<const ConfigurationSpecification::Group&>(py::args("group")) )
            .def(init<const std::string&>(py::args("xmldata")) )
            .def("GetGroupFromName",&PyConfigurationSpecification::GetGroupFromName, return_value_policy<copy_const_reference>(), DOXY_FN(ConfigurationSpecification,GetGroupFromName))
#endif
            .def("SerializeJSON", &PyConfigurationSpecification::SerializeJSON, DOXY_FN(ConfigurationSpecification, SerializeJSON))
            .def("DeserializeJSON", &PyConfigurationSpecification::DeserializeJSON, PY_ARGS("obj") DOXY_FN(ConfigurationSpecification, DeserializeJSON))
            .def("FindCompatibleGroup",&PyConfigurationSpecification::FindCompatibleGroup, DOXY_FN(ConfigurationSpecification,FindCompatibleGroup))
            .def("FindTimeDerivativeGroup",&PyConfigurationSpecification::FindTimeDerivativeGroup, DOXY_FN(ConfigurationSpecification,FindTimeDerivativeGroup))
            .def("GetDOF",&PyConfigurationSpecification::GetDOF,DOXY_FN(ConfigurationSpecification,GetDOF))
            .def("IsValid",&PyConfigurationSpecification::IsValid,DOXY_FN(ConfigurationSpecification,IsValid))
            .def("ResetGroupOffsets",&PyConfigurationSpecification::ResetGroupOffsets,DOXY_FN(ConfigurationSpecification,ResetGroupOffsets))
            .def("AddVelocityGroups",&PyConfigurationSpecification::AddVelocityGroups, PY_ARGS("adddeltatime") DOXY_FN(ConfigurationSpecification,AddVelocityGroups))
            .def("AddDerivativeGroups",&PyConfigurationSpecification::AddDerivativeGroups, PY_ARGS("deriv", "adddeltatime") DOXY_FN(ConfigurationSpecification,AddDerivativeGroups))
            .def("AddDeltaTimeGroup",&PyConfigurationSpecification::AddDeltaTimeGroup,DOXY_FN(ConfigurationSpecification,AddDeltaTimeGroup))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("RemoveGroups", &PyConfigurationSpecification::RemoveGroups,
                 "groupname"_a,
                 "exactmatch"_a = true,
                 DOXY_FN(ConfigurationSpecification, RemoveGroups)
                 )
#else
            .def("RemoveGroups", &PyConfigurationSpecification::RemoveGroups, RemoveGroups_overloads(PY_ARGS("groupname","exactmatch") DOXY_FN(ConfigurationSpecification, RemoveGroups)))
#endif
            .def("AddGroup",addgroup1, PY_ARGS("name","dof","interpolation") DOXY_FN(ConfigurationSpecification,AddGroup "const std::string; int; const std::string"))
            .def("AddGroup",addgroup2, PY_ARGS("group") DOXY_FN(ConfigurationSpecification,AddGroup "const"))
            .def("ConvertToVelocitySpecification",&PyConfigurationSpecification::ConvertToVelocitySpecification,DOXY_FN(ConfigurationSpecification,ConvertToVelocitySpecification))
            .def("ConvertToDerivativeSpecification",&PyConfigurationSpecification::ConvertToDerivativeSpecification, PY_ARGS("timederivative") DOXY_FN(ConfigurationSpecification, ConvertToDerivativeSpecification))
            .def("GetTimeDerivativeSpecification",&PyConfigurationSpecification::GetTimeDerivativeSpecification,DOXY_FN(ConfigurationSpecification,GetTimeDerivativeSpecification))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("ExtractTransform", &PyConfigurationSpecification::ExtractTransform,
                 "transform"_a,
                 "data"_a,
                 "body"_a,
                 "timederivative"_a = 0,
                 DOXY_FN(ConfigurationSpecification,ExtractTransform)
                 )
#else
            .def("ExtractTransform",&PyConfigurationSpecification::ExtractTransform,ExtractTransform_overloads(PY_ARGS("transform","data","body","timederivative") DOXY_FN(ConfigurationSpecification,ExtractTransform)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("ExtractAffineValues", &PyConfigurationSpecification::ExtractAffineValues,
                 "data"_a,
                 "body"_a,
                 "affinedofs"_a,
                 "timederivative"_a = 0,
                 DOXY_FN(ConfigurationSpecification,ExtractAffineValues)
                 )
#else
            .def("ExtractAffineValues",&PyConfigurationSpecification::ExtractAffineValues,ExtractAffineValues_overloads(PY_ARGS("data","body","affinedofs","timederivative") DOXY_FN(ConfigurationSpecification,ExtractAffineValues)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("ExtractIkParameterization", &PyConfigurationSpecification::ExtractIkParameterization,
                 "data"_a,
                 "timederivative"_a = 0,
                 "robotname"_a = "",
                 "manipulatorname"_a = "",
                 DOXY_FN(ConfigurationSpecification, ExtractIkParameterization)
                 )
#else
            .def("ExtractIkParameterization",&PyConfigurationSpecification::ExtractIkParameterization,ExtractIkParameterization_overloads(PY_ARGS("data","timederivative","robotname","manipulatorname") DOXY_FN(ConfigurationSpecification,ExtractIkParameterization)))
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("ExtractJointValues", &PyConfigurationSpecification::ExtractJointValues,
                 "data"_a,
                 "body"_a,
                 "indices"_a,
                 "timederivative"_a = 0,
                 DOXY_FN(ConfigurationSpecification,ExtractJointValues)
                 )
#else
            .def("ExtractJointValues",&PyConfigurationSpecification::ExtractJointValues,ExtractJointValues_overloads(PY_ARGS("data","body","indices","timederivative") DOXY_FN(ConfigurationSpecification,ExtractJointValues)))
#endif
            .def("ExtractDeltaTime",&PyConfigurationSpecification::ExtractDeltaTime, PY_ARGS("data") DOXY_FN(ConfigurationSpecification,ExtractDeltaTime))
            .def("InsertDeltaTime",&PyConfigurationSpecification::InsertDeltaTime, PY_ARGS("data","deltatime") DOXY_FN(ConfigurationSpecification,InsertDeltaTime))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("InsertJointValues", &PyConfigurationSpecification::InsertJointValues,
                 "data"_a,
                 "values"_a,
                 "body"_a,
                 "indices"_a,
                 "timederivative"_a = 0,
                 DOXY_FN(ConfigurationSpecification, InsertJointValues)
                 )
#else
            .def("InsertJointValues",&PyConfigurationSpecification::InsertJointValues, PY_ARGS("data","values","body","indices","timederivative") DOXY_FN(ConfigurationSpecification,InsertJointValues))
#endif
            .def("ExtractUsedBodies", &PyConfigurationSpecification::ExtractUsedBodies, PY_ARGS("env") DOXY_FN(ConfigurationSpecification, ExtractUsedBodies))
            .def("ExtractUsedIndices", &PyConfigurationSpecification::ExtractUsedIndices, PY_ARGS("env") DOXY_FN(ConfigurationSpecification, ExtractUsedIndices))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def("ConvertData", &PyConfigurationSpecification::ConvertData,
                 "targetspec"_a,
                 "sourcedata"_a,
                 "numpoints"_a,
                 "env"_a,
                 "filluninitialized"_a = true,
                 DOXY_FN(ConfigurationSpecification, ConvertData)
                 )
#else
            .def("ConvertData", &PyConfigurationSpecification::ConvertData, PY_ARGS("targetspec", "sourcedata", "numpoints", "env", "filluninitialized") DOXY_FN(ConfigurationSpecification, ConvertData))
#endif
            .def("ConvertDataFromPrevious", &PyConfigurationSpecification::ConvertDataFromPrevious, PY_ARGS("targetdata", "targetspec", "sourcedata", "numpoints", "env") DOXY_FN(ConfigurationSpecification, ConvertData))
            .def("GetGroups", &PyConfigurationSpecification::GetGroups, /*PY_ARGS("env")*/ "returns a list of the groups")
            .def("__eq__",&PyConfigurationSpecification::__eq__)
            .def("__ne__",&PyConfigurationSpecification::__ne__)
            .def("__add__",&PyConfigurationSpecification::__add__)
            .def("__iadd__",&PyConfigurationSpecification::__iadd__)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            .def(py::pickle(
                     [](const PyConfigurationSpecification& pyspec) {
                std::stringstream ss;
                ss << pyspec._spec;
                return py::make_tuple(ss.str());
            },
                     [](py::tuple state) {
                // __setstate__
                if(state.size() != 1) {
                    throw std::runtime_error("Invalid state");
                }
                return PyConfigurationSpecification(state[0].cast<std::string>());
            }
                     ))
#else
            .def_pickle(ConfigurationSpecification_pickle_suite())
#endif
            .def("__str__",&PyConfigurationSpecification::__str__)
            .def("__unicode__",&PyConfigurationSpecification::__unicode__)
            .def("__repr__",&PyConfigurationSpecification::__repr__)
        ;

        {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
            // Group belongs to ConfigurationSpecification
            scope_ group = class_<ConfigurationSpecification::Group, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> >(configurationspecification, "Group",DOXY_CLASS(ConfigurationSpecification::Group))
                           .def(init<>())
#else
            scope_ group = class_<ConfigurationSpecification::Group, OPENRAVE_SHARED_PTR<ConfigurationSpecification::Group> >("Group",DOXY_CLASS(ConfigurationSpecification::Group))
#endif
                           .def_readwrite("name",&ConfigurationSpecification::Group::name)
                           .def_readwrite("interpolation",&ConfigurationSpecification::Group::interpolation)
                           .def_readwrite("offset",&ConfigurationSpecification::Group::offset)
                           .def_readwrite("dof",&ConfigurationSpecification::Group::dof)
            ;
        }
    }
#ifndef USE_PYBIND11_PYTHON_BINDINGS
    openravepy::spec_from_group();
#endif

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ scope_stringreaders = class_<PyStringReaderStaticClass>(m, "stringreaders")
#else
        scope_ scope_stringreaders = class_<PyStringReaderStaticClass>("stringreaders")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                            .def_static("CreateStringReadable", xmlreaders::pyCreateStringXMLReadable, PY_ARGS("id", "data") DOXY_FN1(pyCreateStringReadable))
#else
                            // https://wiki.python.org/moin/boost.python/HowTo
                            .def("CreateStringReadable", pyCreateStringReadable, PY_ARGS("id", "data") DOXY_FN1(pyCreateStringReadable))
                            .staticmethod("CreateStringReadable")
#endif
        ;
    }

    {
#ifdef USE_PYBIND11_PYTHON_BINDINGS
        scope_ RAVE_DEPRECATED scope_xmlreaders = class_<xmlreaders::PyXMLReaderStaticClass>(m, "xmlreaders")
#else
        scope_ RAVE_DEPRECATED scope_xmlreaders = class_<xmlreaders::PyXMLReaderStaticClass>("xmlreaders")
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
                                  .def_static("CreateStringXMLReadable", xmlreaders::pyCreateStringXMLReadable, PY_ARGS("xmlid", "data") DOXY_FN1(pyCreateStringXMLReadable))
#else
                                  // https://wiki.python.org/moin/boost.python/HowTo
                                  .def("CreateStringXMLReadable",xmlreaders::pyCreateStringXMLReadable, PY_ARGS("xmlid", "data") DOXY_FN1(pyCreateStringXMLReadable))
                                  .staticmethod("CreateStringXMLReadable")
#endif
        ;
    }
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#else
    def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#else
    def("RaveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
#else
    def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveSetDataAccess",openravepy::pyRaveSetDataAccess, PY_ARGS("accessoptions") DOXY_FN1(RaveSetDataAccess));
#else
    def("RaveSetDataAccess",openravepy::pyRaveSetDataAccess, PY_ARGS("accessoptions") DOXY_FN1(RaveSetDataAccess));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetDataAccess",OpenRAVE::RaveGetDataAccess, DOXY_FN1(RaveGetDataAccess));
#else
    def("RaveGetDataAccess",OpenRAVE::RaveGetDataAccess, DOXY_FN1(RaveGetDataAccess));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetDefaultViewerType", OpenRAVE::RaveGetDefaultViewerType, DOXY_FN1(RaveGetDefaultViewerType));
#else
    def("RaveGetDefaultViewerType", OpenRAVE::RaveGetDefaultViewerType, DOXY_FN1(RaveGetDefaultViewerType));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveFindLocalFile", OpenRAVE::RaveFindLocalFile,
          "filename"_a,
          "curdir"_a = "",
          DOXY_FN1(RaveFindLocalFile)
          );
#else
    def("RaveFindLocalFile",OpenRAVE::RaveFindLocalFile,RaveFindLocalFile_overloads(PY_ARGS("filename","curdir") DOXY_FN1(RaveFindLocalFile)));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveInvertFileLookup",openravepy::pyRaveInvertFileLookup, PY_ARGS("filename") DOXY_FN1(RaveInvertFileLookup));
#else
    def("RaveInvertFileLookup",openravepy::pyRaveInvertFileLookup, PY_ARGS("filename") DOXY_FN1(RaveInvertFileLookup));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
#else
    def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveFindDatabaseFile",OpenRAVE::RaveFindDatabaseFile,DOXY_FN1(RaveFindDatabaseFile));
#else
    def("RaveFindDatabaseFile",OpenRAVE::RaveFindDatabaseFile,DOXY_FN1(RaveFindDatabaseFile));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogFatal",openravepy::raveLogFatal,PY_ARGS("log") "Send a fatal log to the openrave system");
#else
    def("RaveLogFatal",openravepy::raveLogFatal,PY_ARGS("log") "Send a fatal log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogError",openravepy::raveLogError,PY_ARGS("log") "Send an error log to the openrave system");
#else
    def("RaveLogError",openravepy::raveLogError,PY_ARGS("log") "Send an error log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogWarn",openravepy::raveLogWarn,PY_ARGS("log") "Send a warn log to the openrave system");
#else
    def("RaveLogWarn",openravepy::raveLogWarn,PY_ARGS("log") "Send a warn log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogInfo",openravepy::raveLogInfo,PY_ARGS("log") "Send an info log to the openrave system");
#else
    def("RaveLogInfo",openravepy::raveLogInfo,PY_ARGS("log") "Send an info log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogDebug",openravepy::raveLogDebug,PY_ARGS("log") "Send a debug log to the openrave system");
#else
    def("RaveLogDebug",openravepy::raveLogDebug,PY_ARGS("log") "Send a debug log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLogVerbose",openravepy::raveLogVerbose,PY_ARGS("log") "Send a verbose log to the openrave system");
#else
    def("RaveLogVerbose",openravepy::raveLogVerbose,PY_ARGS("log") "Send a verbose log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLog",openravepy::raveLog,PY_ARGS("log","level") "Send a log to the openrave system with excplicit level");
#else
    def("RaveLog",openravepy::raveLog,PY_ARGS("log","level") "Send a log to the openrave system with excplicit level");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveInitialize", openravepy::pyRaveInitialize,
          "load_all_plugins"_a = true,
          "level"_a = py::none_(),
          DOXY_FN1(RaveInitialize)
          );
#else
    def("RaveInitialize",openravepy::pyRaveInitialize,RaveInitialize_overloads(PY_ARGS("load_all_plugins","level") DOXY_FN1(RaveInitialize)));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
#else
    def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
#else
    def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(RaveGetLoadedInterfaces));
#else
    def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(RaveGetLoadedInterfaces));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
#else
    def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin, PY_ARGS("filename") DOXY_FN1(RaveLoadPlugins));
#else
    def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin, PY_ARGS("filename") DOXY_FN1(RaveLoadPlugins));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveHasInterface",OpenRAVE::RaveHasInterface, PY_ARGS("type","name") DOXY_FN1(RaveHasInterface));
#else
    def("RaveHasInterface",OpenRAVE::RaveHasInterface, PY_ARGS("type","name") DOXY_FN1(RaveHasInterface));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGlobalState",OpenRAVE::RaveGlobalState,DOXY_FN1(RaveGlobalState));
#else
    def("RaveGlobalState",OpenRAVE::RaveGlobalState,DOXY_FN1(RaveGlobalState));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveClone", openravepy::pyRaveClone,
          "ref"_a,
          "cloningoptions"_a,
          "cloneenv"_a = py::none_(), // PyEnvironmentBasePtr(),
          DOXY_FN1(RaveClone)
          );
#else
    def("RaveClone",openravepy::pyRaveClone,RaveClone_overloads(PY_ARGS("ref","cloningoptions", "cloneenv") DOXY_FN1(RaveClone)));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetIkTypeFromUniqueId",OpenRAVE::RaveGetIkTypeFromUniqueId, PY_ARGS("uniqueid") DOXY_FN1(RaveGetIkTypeFromUniqueId));
#else
    def("RaveGetIkTypeFromUniqueId",OpenRAVE::RaveGetIkTypeFromUniqueId, PY_ARGS("uniqueid") DOXY_FN1(RaveGetIkTypeFromUniqueId));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetIndexFromAffineDOF",OpenRAVE::RaveGetIndexFromAffineDOF, PY_ARGS("affinedofs","dof") DOXY_FN1(RaveGetIndexFromAffineDOF));
#else
    def("RaveGetIndexFromAffineDOF",OpenRAVE::RaveGetIndexFromAffineDOF, PY_ARGS("affinedofs","dof") DOXY_FN1(RaveGetIndexFromAffineDOF));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetAffineDOFFromIndex",OpenRAVE::RaveGetAffineDOFFromIndex, PY_ARGS("affinedofs","index") DOXY_FN1(RaveGetAffineDOFFromIndex));
#else
    def("RaveGetAffineDOFFromIndex",OpenRAVE::RaveGetAffineDOFFromIndex, PY_ARGS("affinedofs","index") DOXY_FN1(RaveGetAffineDOFFromIndex));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetAffineDOF",OpenRAVE::RaveGetAffineDOF, PY_ARGS("affinedofs") DOXY_FN1(RaveGetAffineDOF));
#else
    def("RaveGetAffineDOF",OpenRAVE::RaveGetAffineDOF, PY_ARGS("affinedofs") DOXY_FN1(RaveGetAffineDOF));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetAffineDOFValuesFromTransform", openravepy::pyRaveGetAffineDOFValuesFromTransform,
          "transform"_a,
          "affinedofs"_a,
          "rotationaxis"_a = py::none_(),
          DOXY_FN1(RaveGetAffineDOFValuesFromTransform)
          );
#else
    def("RaveGetAffineDOFValuesFromTransform",openravepy::pyRaveGetAffineDOFValuesFromTransform, pyRaveGetAffineDOFValuesFromTransform_overloads(PY_ARGS("transform","affinedofs","rotationaxis") DOXY_FN1(RaveGetAffineDOFValuesFromTransform)));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveGetAffineConfigurationSpecification", openravepy::pyRaveGetAffineConfigurationSpecification,
          "affinedofs"_a,
          "body"_a = py::none_(), // PyKinBodyPtr(),
          "interpolation"_a = "",
          DOXY_FN1(RaveGetAffineConfigurationSpecification)
          );
#else
    def("RaveGetAffineConfigurationSpecification",openravepy::pyRaveGetAffineConfigurationSpecification, pyRaveGetAffineConfigurationSpecification_overloads(PY_ARGS("affinedofs","body","interpolation") DOXY_FN1(RaveGetAffineConfigurationSpecification)));
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#else
    def("raveSetDebugLevel",openravepy::pyRaveSetDebugLevel, PY_ARGS("level") DOXY_FN1(RaveSetDebugLevel));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
#else
    def("raveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogFatal",openravepy::raveLogFatal,PY_ARGS("log") "Send a fatal log to the openrave system");
#else
    def("raveLogFatal",openravepy::raveLogFatal,PY_ARGS("log") "Send a fatal log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogError",openravepy::raveLogError,PY_ARGS("log") "Send an error log to the openrave system");
#else
    def("raveLogError",openravepy::raveLogError,PY_ARGS("log") "Send an error log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogWarn",openravepy::raveLogWarn,PY_ARGS("log") "Send a warn log to the openrave system");
#else
    def("raveLogWarn",openravepy::raveLogWarn,PY_ARGS("log") "Send a warn log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogInfo",openravepy::raveLogInfo,PY_ARGS("log") "Send an info log to the openrave system");
#else
    def("raveLogInfo",openravepy::raveLogInfo,PY_ARGS("log") "Send an info log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogDebug",openravepy::raveLogDebug,PY_ARGS("log") "Send a debug log to the openrave system");
#else
    def("raveLogDebug",openravepy::raveLogDebug,PY_ARGS("log") "Send a debug log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLogVerbose",openravepy::raveLogVerbose,PY_ARGS("log") "Send a verbose log to the openrave system");
#else
    def("raveLogVerbose",openravepy::raveLogVerbose,PY_ARGS("log") "Send a verbose log to the openrave system");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("raveLog",openravepy::raveLog,PY_ARGS("log","level") "Send a log to the openrave system with excplicit level");
#else
    def("raveLog",openravepy::raveLog,PY_ARGS("log","level") "Send a log to the openrave system with excplicit level");
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatFromAxisAngle",openravepy::quatFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(quatFromAxisAngle "const RaveVector"));
#else
    def("quatFromAxisAngle",openravepy::quatFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(quatFromAxisAngle "const RaveVector"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatFromAxisAngle",openravepy::quatFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(quatFromAxisAngle "const RaveVector; T"));
#else
    def("quatFromAxisAngle",openravepy::quatFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(quatFromAxisAngle "const RaveVector; T"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatFromRotationMatrix",openravepy::quatFromRotationMatrix, PY_ARGS("rotation") DOXY_FN1(quatFromMatrix "const RaveTransform"));
#else
    def("quatFromRotationMatrix",openravepy::quatFromRotationMatrix, PY_ARGS("rotation") DOXY_FN1(quatFromMatrix "const RaveTransform"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("InterpolateQuatSlerp", openravepy::InterpolateQuatSlerp,
          "quat0"_a,
          "quat1"_a,
          "t"_a,
          "forceshortarc"_a = true,
          DOXY_FN1(InterpolateQuatSlerp "const RaveVector; const RaveVector; T")
          );
#else
    def("InterpolateQuatSlerp",openravepy::InterpolateQuatSlerp, InterpolateQuatSlerp_overloads(PY_ARGS("quat0","quat1","t","forceshortarc") DOXY_FN1(InterpolateQuatSlerp "const RaveVector; const RaveVector; T")));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("InterpolateQuatSquad",openravepy::InterpolateQuatSquad,
          "quat0"_a,
          "quat1"_a,
          "quat2"_a,
          "quat3"_a,
          "t"_a,
          "forceshortarc"_a = true,
          DOXY_FN1(InterpolateQuatSquad)
          );
#else
    def("InterpolateQuatSquad",openravepy::InterpolateQuatSquad, InterpolateQuatSquad_overloads(PY_ARGS("quat0","quat1","quat2","quat3","t","forceshortarc") DOXY_FN1(InterpolateQuatSquad)));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatSlerp",openravepy::InterpolateQuatSlerp,
          "quat0"_a,
          "quat1"_a,
          "t"_a,
          "forceshortarc"_a = true,
          DOXY_FN1(quatSlerp "const RaveVector; const RaveVector; T")
          ); // deprecated
#else
    def("quatSlerp",openravepy::InterpolateQuatSlerp, PY_ARGS("quat0","quat1","t", "forceshortarc") DOXY_FN1(quatSlerp "const RaveVector; const RaveVector; T")); // deprecated
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("axisAngleFromRotationMatrix",openravepy::axisAngleFromRotationMatrix, PY_ARGS("rotation") DOXY_FN1(axisAngleFromMatrix "const RaveTransformMatrix"));
#else
    def("axisAngleFromRotationMatrix",openravepy::axisAngleFromRotationMatrix, PY_ARGS("rotation") DOXY_FN1(axisAngleFromMatrix "const RaveTransformMatrix"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("axisAngleFromQuat",openravepy::axisAngleFromQuat, PY_ARGS("quat") DOXY_FN1(axisAngleFromQuat "const RaveVector"));
#else
    def("axisAngleFromQuat",openravepy::axisAngleFromQuat, PY_ARGS("quat") DOXY_FN1(axisAngleFromQuat "const RaveVector"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("rotationMatrixFromQuat",openravepy::rotationMatrixFromQuat, PY_ARGS("quat") DOXY_FN1(matrixFromQuat "const RaveVector"));
#else
    def("rotationMatrixFromQuat",openravepy::rotationMatrixFromQuat, PY_ARGS("quat") DOXY_FN1(matrixFromQuat "const RaveVector"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("rotationMatrixFromQArray",openravepy::rotationMatrixFromQArray,PY_ARGS("quatarray") "Converts an array of quaternions to a list of 3x3 rotation matrices.\n\n:param quatarray: nx4 array\n");
#else
    def("rotationMatrixFromQArray",openravepy::rotationMatrixFromQArray,PY_ARGS("quatarray") "Converts an array of quaternions to a list of 3x3 rotation matrices.\n\n:param quatarray: nx4 array\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixFromQuat",openravepy::matrixFromQuat, PY_ARGS("quat") "Converts a quaternion to a 4x4 affine matrix.\n\n:param quat: 4 values\n");
#else
    def("matrixFromQuat",openravepy::matrixFromQuat, PY_ARGS("quat") "Converts a quaternion to a 4x4 affine matrix.\n\n:param quat: 4 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
#else
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
#else
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
#else
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle1, PY_ARGS("axisangle") DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
#else
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle2, PY_ARGS("axis","angle") DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixFromPose",openravepy::matrixFromPose, PY_ARGS("pose") "Converts a 7 element quaterion+translation transform to a 4x4 matrix.\n\n:param pose: 7 values\n");
#else
    def("matrixFromPose",openravepy::matrixFromPose, PY_ARGS("pose") "Converts a 7 element quaterion+translation transform to a 4x4 matrix.\n\n:param pose: 7 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixFromPoses",openravepy::matrixFromPoses, PY_ARGS("poses") "Converts a Nx7 element quaterion+translation array to a 4x4 matrices.\n\n:param poses: nx7 array\n");
#else
    def("matrixFromPoses",openravepy::matrixFromPoses, PY_ARGS("poses") "Converts a Nx7 element quaterion+translation array to a 4x4 matrices.\n\n:param poses: nx7 array\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseFromMatrix",openravepy::poseFromMatrix, PY_ARGS("transform") "Converts a 4x4 matrix to a 7 element quaternion+translation representation.\n\n:param transform: 3x4 or 4x4 affine matrix\n");
#else
    def("poseFromMatrix",openravepy::poseFromMatrix, PY_ARGS("transform") "Converts a 4x4 matrix to a 7 element quaternion+translation representation.\n\n:param transform: 3x4 or 4x4 affine matrix\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseFromMatrices",openravepy::poseFromMatrices, PY_ARGS("transforms") "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation.\n\n:param transforms: list of 3x4 or 4x4 affine matrices\n");
#else
    def("poseFromMatrices",openravepy::poseFromMatrices, PY_ARGS("transforms") "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation.\n\n:param transforms: list of 3x4 or 4x4 affine matrices\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("InvertPoses",openravepy::InvertPoses,PY_ARGS("poses") "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
#else
    def("InvertPoses",openravepy::InvertPoses,PY_ARGS("poses") "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("InvertPose",openravepy::InvertPose,PY_ARGS("pose") "Inverts a 7-element pose where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param pose: 7-element array");
#else
    def("InvertPose",openravepy::InvertPose,PY_ARGS("pose") "Inverts a 7-element pose where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param pose: 7-element array");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatRotateDirection",openravepy::quatRotateDirection, PY_ARGS("sourcedir", "targetdir") DOXY_FN1(quatRotateDirection));
#else
    def("quatRotateDirection",openravepy::quatRotateDirection, PY_ARGS("sourcedir", "targetdir") DOXY_FN1(quatRotateDirection));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("ExtractAxisFromQuat",openravepy::ExtractAxisFromQuat, PY_ARGS("quat","iaxis") DOXY_FN1(ExtractAxisFromQuat));
#else
    def("ExtractAxisFromQuat",openravepy::ExtractAxisFromQuat, PY_ARGS("quat","iaxis") DOXY_FN1(ExtractAxisFromQuat));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("MultiplyQuat",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#else
    def("MultiplyQuat",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatMult",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#else
    def("quatMult",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatMultiply",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#else
    def("quatMultiply",openravepy::MultiplyQuat, PY_ARGS("quat0","quat1") DOXY_FN1(quatMultiply));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("InvertQuat",openravepy::InvertQuat, PY_ARGS("quat") DOXY_FN1(quatInverse));
#else
    def("InvertQuat",openravepy::InvertQuat, PY_ARGS("quat") DOXY_FN1(quatInverse));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("quatInverse",openravepy::InvertQuat, PY_ARGS("quat") DOXY_FN1(quatInverse));
#else
    def("quatInverse",openravepy::InvertQuat, PY_ARGS("quat") DOXY_FN1(quatInverse));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("MultiplyPose",openravepy::MultiplyPose,PY_ARGS("pose1","pose2") "multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
#else
    def("MultiplyPose",openravepy::MultiplyPose,PY_ARGS("pose1","pose2") "multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseTransformPoint",openravepy::poseTransformPoint,PY_ARGS("pose","point") "left-transforms a 3D point by a pose transformation.\n\n:param pose: 7 values\n\n:param points: 3 values");
#else
    def("poseTransformPoint",openravepy::poseTransformPoint,PY_ARGS("pose","point") "left-transforms a 3D point by a pose transformation.\n\n:param pose: 7 values\n\n:param points: 3 values");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseTransformPoints",openravepy::poseTransformPoints,PY_ARGS("pose","points") "left-transforms a set of points by a pose transformation.\n\n:param pose: 7 values\n\n:param points: Nx3 values");
#else
    def("poseTransformPoints",openravepy::poseTransformPoints,PY_ARGS("pose","points") "left-transforms a set of points by a pose transformation.\n\n:param pose: 7 values\n\n:param points: Nx3 values");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("TransformLookat",openravepy::TransformLookat,PY_ARGS("lookat","camerapos","cameraup") "Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
#else
    def("TransformLookat",openravepy::TransformLookat,PY_ARGS("lookat","camerapos","cameraup") "Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("transformLookat",openravepy::TransformLookat,PY_ARGS("lookat","camerapos","cameraup") "Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
#else
    def("transformLookat",openravepy::TransformLookat,PY_ARGS("lookat","camerapos","cameraup") "Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("matrixSerialization",openravepy::matrixSerialization,PY_ARGS("transform") "Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
#else
    def("matrixSerialization",openravepy::matrixSerialization,PY_ARGS("transform") "Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseSerialization",openravepy::poseSerialization, PY_ARGS("pose") "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
#else
    def("poseSerialization",openravepy::poseSerialization, PY_ARGS("pose") "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion, "Returns the compiler version that openravepy_int was compiled with");
#else
    def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion, "Returns the compiler version that openravepy_int was compiled with");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("normalizeAxisRotation",openravepy::normalizeAxisRotation, PY_ARGS("axis","quat") DOXY_FN1(normalizeAxisRotation));
#else
    def("normalizeAxisRotation",openravepy::normalizeAxisRotation, PY_ARGS("axis","quat") DOXY_FN1(normalizeAxisRotation));
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("ComputePoseDistSqr", openravepy::ComputePoseDistSqr,
          "pose0"_a,
          "pose1"_a,
          "quatweight"_a = 1.0,
          DOXY_FN1(ComputePoseDistSqr)
          );
#else
    def("ComputePoseDistSqr", openravepy::ComputePoseDistSqr, ComputePoseDistSqr_overloads(PY_ARGS("pose0", "pose1", "quatweight") DOXY_FN1(ComputePoseDistSqr)));
#endif

    // deprecated
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("invertPoses",openravepy::InvertPoses, PY_ARGS("poses") "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
#else
    def("invertPoses",openravepy::InvertPoses, PY_ARGS("poses") "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
#endif
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("poseMult",openravepy::MultiplyPose, PY_ARGS("pose1","pose2") "multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
#else
    def("poseMult",openravepy::MultiplyPose, PY_ARGS("pose1","pose2") "multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
#endif
}

} // end namespace openravepy
