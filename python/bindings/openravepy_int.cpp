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
#include "openravepy_int.h"

/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
bool GetReturnTransformQuaternions() { return s_bReturnTransformQuaternions; }
void SetReturnTransformQuaternions(bool bset) { s_bReturnTransformQuaternions = bset; }

template <typename T>
inline object ReturnTransform(T t)
{
    if( s_bReturnTransformQuaternions) {
        return toPyArray(Transform(t));
    }
    else {
        return toPyArray(TransformMatrix(t));
    }
}

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

    virtual string __repr__() { return boost::str(boost::format("<Ray([%f,%f,%f],[%f,%f,%f])>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z); }
    virtual string __str__() { return boost::str(boost::format("<%f %f %f %f %f %f>")%r.pos.x%r.pos.y%r.pos.z%r.dir.x%r.dir.y%r.dir.z); }

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

    virtual string __repr__() { return boost::str(boost::format("<AABB([%f,%f,%f],[%f,%f,%f])>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z); }
    virtual string __str__() { return boost::str(boost::format("<%f %f %f %f %f %f>")%ab.pos.x%ab.pos.y%ab.pos.z%ab.extents.x%ab.extents.y%ab.extents.z); }

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

    InterfaceType GetInterfaceType() const { return _pbase->GetInterfaceType(); }
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

    virtual string __repr__() { return boost::str(boost::format("<env.CreateInterface(InterfaceType.%s,'%s')>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId()); }
    virtual string __str__() { return boost::str(boost::format("<%s:%s>")%RaveGetInterfaceName(_pbase->GetInterfaceType())%_pbase->GetXMLId()); }
    virtual bool __eq__(PyInterfaceBasePtr p) { return !!p && _pbase == p->GetInterfaceBase(); }
    virtual bool __ne__(PyInterfaceBasePtr p) { return !p || _pbase != p->GetInterfaceBase(); }
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

            string __str__() { return boost::str(boost::format("<trimesh: verts %d, tris=%d>")%len(vertices)%len(indices)); }
            
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
            void SetAmbientColor(object ocolor) { _plink->GetGeometry(_geomindex).SetAmbientColor(ExtractVector3(ocolor)); }
            void SetDiffuseColor(object ocolor) { _plink->GetGeometry(_geomindex).SetDiffuseColor(ExtractVector3(ocolor)); }
            bool IsDraw() { return _plink->GetGeometry(_geomindex).IsDraw(); }
            bool IsModifiable() { return _plink->GetGeometry(_geomindex).IsModifiable(); }
            KinBody::Link::GEOMPROPERTIES::GeomType GetType() { return _plink->GetGeometry(_geomindex).GetType(); }
            object GetTransform() { return ReturnTransform(_plink->GetGeometry(_geomindex).GetTransform()); }
            dReal GetSphereRadius() const { return _plink->GetGeometry(_geomindex).GetSphereRadius(); }
            dReal GetCylinderRadius() const { return _plink->GetGeometry(_geomindex).GetCylinderRadius(); }
            dReal GetCylinderHeight() const { return _plink->GetGeometry(_geomindex).GetCylinderHeight(); }
            object GetBoxExtents() const { return toPyVector3(_plink->GetGeometry(_geomindex).GetBoxExtents()); }
            object GetRenderScale() const { return toPyVector3(_plink->GetGeometry(_geomindex).GetRenderScale()); }
            string GetRenderFilename() const { return _plink->GetGeometry(_geomindex).GetRenderFilename(); }

            bool __eq__(boost::shared_ptr<PyGeomProperties> p) { return !!p && _plink == p->_plink && _geomindex == p->_geomindex; }
            bool __ne__(boost::shared_ptr<PyGeomProperties> p) { return !p || _plink != p->_plink || _geomindex != p->_geomindex; }
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
        boost::shared_ptr<PyLink> GetParentLink() const
        {
            KinBody::LinkPtr parentlink = _plink->GetParentLink();
            return !parentlink ? boost::shared_ptr<PyLink>() : boost::shared_ptr<PyLink>(new PyLink(parentlink,_pyenv));
        }
        
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

        string __repr__() { return boost::str(boost::format("<env.GetKinBody('%s').GetLink('%s')>")%_plink->GetParent()->GetName()%_plink->GetName()); }
        string __str__() { return boost::str(boost::format("<link:%s (%d), parent=%s>")%_plink->GetName()%_plink->GetIndex()%_plink->GetParent()->GetName()); }
        bool __eq__(boost::shared_ptr<PyLink> p) { return !!p && _plink == p->_plink; }
        bool __ne__(boost::shared_ptr<PyLink> p) { return !p || _plink != p->_plink; }
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
        bool IsCircular() const { return _pjoint->IsCircular(); }
        bool IsStatic() const { return _pjoint->IsStatic(); }

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
        object GetWeights() const {
            vector<dReal> weights(_pjoint->GetDOF());
            for(size_t i = 0; i < weights.size(); ++i)
                weights[i] = _pjoint->GetWeight(i);
            return toPyArray(weights);
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
        
        string __repr__() { return boost::str(boost::format("<env.GetKinBody('%s').GetJoint('%s')>")%_pjoint->GetParent()->GetName()%_pjoint->GetName()); }
        string __str__() { return boost::str(boost::format("<joint:%s (%d), dof=%d, parent=%s>")%_pjoint->GetName()%_pjoint->GetJointIndex()%_pjoint->GetDOFIndex()%_pjoint->GetParent()->GetName()); }
        bool __eq__(boost::shared_ptr<PyJoint> p) { return !!p && _pjoint==p->_pjoint; }
        bool __ne__(boost::shared_ptr<PyJoint> p) { return !p || _pjoint!=p->_pjoint; }
    };
    typedef boost::shared_ptr<PyJoint> PyJointPtr;
    typedef boost::shared_ptr<PyJoint const> PyJointConstPtr;

    class PyManageData
    {
        KinBody::ManageDataPtr _pdata;
        PyEnvironmentBasePtr _pyenv;
    public:
        PyManageData(KinBody::ManageDataPtr pdata, PyEnvironmentBasePtr pyenv) : _pdata(pdata), _pyenv(pyenv) {}
        virtual ~PyManageData() {}

        KinBody::ManageDataPtr GetManageData() { return _pdata; }
        
        PySensorSystemBasePtr GetSystem();
        PyVoidHandleConst GetData() const { return PyVoidHandleConst(_pdata->GetData()); }
        PyLinkPtr GetOffsetLink() const {
            KinBody::LinkPtr plink = _pdata->GetOffsetLink();
            return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,_pyenv));
        }
        bool IsPresent() { return _pdata->IsPresent(); }
        bool IsEnabled() { return _pdata->IsEnabled(); }
        bool IsLocked() { return _pdata->IsLocked(); }
        bool Lock(bool bDoLock) { return _pdata->Lock(bDoLock); }

        string __repr__() { return boost::str(boost::format("<env.GetKinBody('%s').GetManageData()>")%_pdata->GetOffsetLink()->GetParent()->GetName()); }
        string __str__() {
            KinBody::LinkPtr plink = _pdata->GetOffsetLink();
            SensorSystemBasePtr psystem = _pdata->GetSystem();
            string systemname = !psystem ? "(NONE)" : psystem->GetXMLId();
            return boost::str(boost::format("<managedata:%s, parent=%s:%s>")%systemname%plink->GetParent()->GetName()%plink->GetName());
        }
        bool __eq__(boost::shared_ptr<PyManageData> p) { return !!p && _pdata==p->_pdata; }
        bool __ne__(boost::shared_ptr<PyManageData> p) { return !p || _pdata!=p->_pdata; }
    };
    typedef boost::shared_ptr<PyManageData> PyManageDataPtr;
    typedef boost::shared_ptr<PyManageData const> PyManageDataConstPtr;

    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pbody,pyenv), _pbody(pbody) {}
    PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv) { _pbody = r._pbody; }
    virtual ~PyKinBody() {}
    KinBodyPtr GetBody() { return _pbody; }

    bool InitFromFile(const string& filename) { return _pbody->InitFromFile(filename,std::list<std::pair<std::string,std::string> >()); }
    bool InitFromData(const string& data) { return _pbody->InitFromData(data,std::list<std::pair<std::string,std::string> >()); }
    bool InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw)
    {
        if( vboxes.shape()[1] != 6 ) {
            throw openrave_exception("boxes needs to be a Nx6 vector\n");
        }
        std::vector<AABB> vaabbs(vboxes.shape()[0]);
        for(size_t i = 0; i < vaabbs.size(); ++i) {
            vaabbs[i].pos = Vector(vboxes[i][0],vboxes[i][1],vboxes[i][2]);
            vaabbs[i].extents = Vector(vboxes[i][3],vboxes[i][4],vboxes[i][5]);
        }

        return _pbody->InitFromBoxes(vaabbs,bDraw);
    }
    bool InitFromTrimesh(boost::shared_ptr<PyKinBody::PyLink::PyTriMesh> pytrimesh, bool bDraw)
    {
        KinBody::Link::TRIMESH mesh;
        pytrimesh->GetTriMesh(mesh);
        return _pbody->InitFromTrimesh(mesh,bDraw);
    }

    void SetName(const string& name) { _pbody->SetName(name); }
    string GetName() const { return _pbody->GetName(); }
    int GetDOF() const { return _pbody->GetDOF(); }

    object GetDOFValues() const
    {
        vector<dReal> values;
        _pbody->GetDOFValues(values);
        return toPyArray(values);
    }
    object GetDOFValues(object oindices) const
    {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values, v;
        values.reserve(vindices.size());
        FOREACHC(it, vindices) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
            pjoint->GetValues(v,false);
            values.push_back(v.at(*it-pjoint->GetDOFIndex()));
        }
        return toPyArray(values);
    }

    object GetDOFVelocities() const
    {
        vector<dReal> values;
        _pbody->GetDOFVelocities(values);
        return toPyArray(values);
    }

    object GetDOFLimits() const
    {
        vector<dReal> vlower, vupper;
        _pbody->GetDOFLimits(vlower,vupper);
        return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }

    object GetDOFLimits(object oindices) const
    {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> vlower, vupper, vtemplower, vtempupper;
        vlower.reserve(vindices.size());
        vupper.reserve(vindices.size());
        FOREACHC(it, vindices) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
            pjoint->GetLimits(vtemplower,vtempupper,false);
            vlower.push_back(vtemplower.at(*it-pjoint->GetDOFIndex()));
            vupper.push_back(vtempupper.at(*it-pjoint->GetDOFIndex()));
        }
        return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }

    object GetDOFMaxVel() const
    {
        vector<dReal> values;
        _pbody->GetDOFMaxVel(values);
        return toPyArray(values);
    }
    object GetDOFWeights() const
    {
        vector<dReal> values;
        _pbody->GetDOFWeights(values);
        return toPyArray(values);
    }
    object GetDOFResolutions() const
    {
        vector<dReal> values;
        _pbody->GetDOFResolutions(values);
        return toPyArray(values);
    }

    object GetJointValues() const
    {
        vector<dReal> values;
        _pbody->GetDOFValues(values);
        return toPyArray(values);
    }

    object GetJointVelocities() const
    {
        vector<dReal> values;
        _pbody->GetDOFVelocities(values);
        return toPyArray(values);
    }

    object GetJointLimits() const
    {
        vector<dReal> vlower, vupper;
        _pbody->GetDOFLimits(vlower,vupper);
        return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }
    object GetJointMaxVel() const
    {
        vector<dReal> values;
        _pbody->GetDOFMaxVel(values);
        return toPyArray(values);
    }
    object GetJointWeights() const
    {
        vector<dReal> values;
        _pbody->GetDOFWeights(values);
        return toPyArray(values);
    }
    
    object GetLinks() const
    {
        boost::python::list links;
        FOREACHC(itlink, _pbody->GetLinks())
            links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        return links;
    }

    object GetLinks(object oindices) const
    {
        if( oindices == object() )
            return GetLinks();
        vector<int> vindices = ExtractArray<int>(oindices);
        boost::python::list links;
        FOREACHC(it, vindices) {
            links.append(PyLinkPtr(new PyLink(_pbody->GetLinks().at(*it),GetEnv())));
        }
        return links;
    }

    PyLinkPtr GetLink(const std::string& linkname) const
    {
        KinBody::LinkPtr plink = _pbody->GetLink(linkname);
        return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,GetEnv()));
    }

    object GetJoints() const
    {
        boost::python::list joints;
        FOREACHC(itjoint, _pbody->GetJoints()) {
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        }
        return joints;
    }

    object GetJoints(object oindices) const
    {
        if( oindices == object() )
            return GetJoints();
        vector<int> vindices = ExtractArray<int>(oindices);
        boost::python::list joints;
        FOREACHC(it, vindices) {
            joints.append(PyJointPtr(new PyJoint(_pbody->GetJoints().at(*it),GetEnv())));
        }
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

    object GetRigidlyAttachedLinks(int linkindex) const
    {
        std::vector<KinBody::LinkPtr> vattachedlinks;
        _pbody->GetRigidlyAttachedLinks(linkindex,vattachedlinks);
        boost::python::list links;
        FOREACHC(itlink, vattachedlinks)
            links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        return links;
    }

    object GetChain(int linkbaseindex, int linkendindex) const
    {
        std::vector<KinBody::JointPtr> vjoints;
        _pbody->GetChain(linkbaseindex,linkendindex,vjoints);
        boost::python::list joints;
        FOREACHC(itjoint, vjoints)
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

    void SetDOFValues(object o)
    {
        if( _pbody->GetDOF() == 0 )
            return;

        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() )
            throw openrave_exception("values do not equal to body degrees of freedom");
        _pbody->SetJointValues(values,true);
    }
    void SetTransformWithDOFValues(object otrans,object ojoints)
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

    void SetDOFValues(object o, object indices)
    {
        if( _pbody->GetDOF() == 0 || len(indices) == 0 )
            return;

        vector<dReal> vsetvalues = ExtractArray<dReal>(o);
        vector<int> vindices = ExtractArray<int>(indices);
        if( vsetvalues.size() != vindices.size() )
            throw openrave_exception("sizes do not match");

        vector<dReal> values;
        _pbody->GetDOFValues(values);
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
        BOOST_ASSERT((int)vtorques.size() == GetDOF() );
        _pbody->SetJointTorques(vtorques,bAdd);
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
    int GetEnvironmentId() const { return _pbody->GetEnvironmentId(); }

    int DoesAffect(int jointindex, int linkindex ) const { return _pbody->DoesAffect(jointindex,linkindex); }

    std::string WriteForwardKinematics() const {
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
    PyManageDataPtr GetManageData() const {
        KinBody::ManageDataPtr pdata = _pbody->GetManageData();
        return !pdata ? PyManageDataPtr() : PyManageDataPtr(new PyManageData(pdata,_pyenv));
    }
    int GetUpdateStamp() const { return _pbody->GetUpdateStamp(); }

    string serialize(int options) const {
        stringstream ss;
        _pbody->serialize(ss,options);
        return ss.str();
    }

    string GetKinematicsGeometryHash() const { return _pbody->GetKinematicsGeometryHash(); }
    PyVoidHandle CreateKinBodyStateSaver() { return PyVoidHandle(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody))); }
    PyVoidHandle CreateKinBodyStateSaver(int options) { return PyVoidHandle(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody, options))); }

    virtual string __repr__() { return boost::str(boost::format("<env.GetKinBody('%s')>")%_pbody->GetName()); }
    virtual string __str__() { return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_pbody->GetInterfaceType())%_pbody->GetXMLId()%_pbody->GetName()%_pbody->GetKinematicsGeometryHash()); }
    virtual void __enter__();
    virtual void __exit__(object type, object value, object traceback);
};

class PyCollisionReport
{
public:
    PyCollisionReport() : report(new CollisionReport()) {}
    PyCollisionReport(CollisionReportPtr report) : report(report) {}
    virtual ~PyCollisionReport() {}

    struct PYCONTACT
    {
        PYCONTACT() {}
        PYCONTACT(const CollisionReport::CONTACT& c)
        {
            pos = toPyVector3(c.pos);
            norm = toPyVector3(c.norm);
            depth = c.depth;
        }

        string __str__()
        {
            Vector vpos = ExtractVector3(pos), vnorm = ExtractVector3(norm);
            stringstream ss;
            ss << "pos=["<<vpos.x<<", "<<vpos.y<<", "<<vpos.z<<"], norm=["<<vnorm.x<<", "<<vnorm.y<<", "<<vnorm.z<<"]";
            return ss.str();
        }
        object pos, norm;
        dReal depth;
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

    string __str__()
    {
        return report->__str__();
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
            stamp = pdata->__stamp;
        }
        virtual ~PySensorData() {}
        
        SensorBase::SensorType type;
        uint64_t stamp;
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
        }
        virtual ~PyLaserSensorData() {}

        object transform, positions, ranges, intensity;
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
        }
        virtual ~PyCameraSensorData() {}

        object transform, imagedata;
        object KK;
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
    
    virtual string __repr__() { return boost::str(boost::format("<env.GetSensor('%s')>")%_psensor->GetName()); }
    virtual string __str__() { return boost::str(boost::format("<%s:%s - %s>")%RaveGetInterfaceName(_psensor->GetInterfaceType())%_psensor->GetXMLId()%_psensor->GetName()); }
};

class PyIkParameterization
{
public:
    PyIkParameterization() {}
    PyIkParameterization(object o, IkParameterization::Type type)
    {
        switch(type) {
        case IkParameterization::Type_Transform6D: SetTransform(o); break;
        case IkParameterization::Type_Rotation3D: SetRotation(o); break;
        case IkParameterization::Type_Translation3D: SetTranslation(o); break;
        case IkParameterization::Type_Direction3D: SetDirection(o); break;
        case IkParameterization::Type_Ray4D: SetRay(extract<boost::shared_ptr<PyRay> >(o)); break;
        case IkParameterization::Type_Lookat3D: SetLookat(o); break;
        default: throw openrave_exception(boost::str(boost::format("incorrect ik parameterization type %d")%type));
        }
    }

    void SetTransform(object o) { _param.SetTransform(ExtractTransform(o)); }
    void SetRotation(object o) { _param.SetRotation(ExtractVector4(o)); }
    void SetTranslation(object o) { _param.SetTranslation(ExtractVector3(o)); }
    void SetDirection(object o) { _param.SetDirection(ExtractVector3(o)); }
    void SetRay(boost::shared_ptr<PyRay> ray) { _param.SetRay(ray->r); }
    void SetLookat(object o) { _param.SetLookat(ExtractVector3(o)); }

    IkParameterization::Type GetType() { return _param.GetType(); }
    object GetTransform() { return ReturnTransform(_param.GetTransform()); }
    object GetRotation() { return toPyVector4(_param.GetRotation()); }
    object GetTranslation() { return toPyVector3(_param.GetTranslation()); }
    object GetDirection() { return toPyVector3(_param.GetDirection()); }
    PyRay GetRay() { return PyRay(_param.GetRay()); }
    object GetLookat() { return toPyVector3(_param.GetLookat()); }

    IkParameterization _param;
};

class IkParameterization_pickle_suite : public pickle_suite
{
public:
    static tuple getinitargs(const PyIkParameterization& r)
    {
        return boost::python::make_tuple(r._param.GetTransform(),r._param.GetType());
    }
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

		bool SetIkSolver(PyIkSolverBasePtr iksolver) { CHECK_POINTER(iksolver); return _pmanip->SetIkSolver(iksolver->GetIkSolver()); }
        PyIkSolverBasePtr GetIkSolver() { IkSolverBasePtr iksolver = _pmanip->GetIkSolver(); return !iksolver ? PyIkSolverBasePtr() : PyIkSolverBasePtr(new PyIkSolverBase(iksolver,_pyenv)); }
        bool InitIKSolver() { RAVELOG_WARN("Manipulator::InitIKSolver has been deprecated\n"); return _pmanip->InitIKSolver(); }
        string GetIKSolverName() const { RAVELOG_WARN("Manipulator::GetIKSolverName  has been deprecated\n");  return _pmanip->GetIKSolverName(); }
        bool HasIKSolver() const { RAVELOG_WARN("Manipulator::HasIKSolver has been deprecated, use GetIkSolver\n");  return !!_pmanip->GetIkSolver(); }

        boost::shared_ptr<PyLink> GetBase() { return !_pmanip->GetBase() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetBase(),_pyenv)); }
        boost::shared_ptr<PyLink> GetEndEffector() { return !_pmanip->GetEndEffector() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetEndEffector(),_pyenv)); }
        object GetGraspTransform() { return ReturnTransform(_pmanip->GetGraspTransform()); }
        object GetGripperJoints() { RAVELOG_DEBUG("GetGripperJoints is deprecated, use GetGripperIndices\n"); return toPyArray(_pmanip->GetGripperIndices()); }
        object GetGripperIndices() { return toPyArray(_pmanip->GetGripperIndices()); }
        object GetArmJoints() { RAVELOG_DEBUG("GetArmJoints is deprecated, use GetArmIndices\n"); return toPyArray(_pmanip->GetArmIndices()); }
        object GetArmIndices() { return toPyArray(_pmanip->GetArmIndices()); }
        object GetClosingDirection() { return toPyArray(_pmanip->GetClosingDirection()); }
        object GetPalmDirection() { RAVELOG_INFO("GetPalmDirection deprecated to GetDirection\n"); return toPyVector3(_pmanip->GetDirection()); }
        object GetDirection() { return toPyVector3(_pmanip->GetDirection()); }
        bool IsGrabbing(PyKinBodyPtr pbody) { return _pmanip->IsGrabbing(pbody->GetBody()); }

        int GetNumFreeParameters() const { return _pmanip->GetNumFreeParameters(); }

        object GetFreeParameters() const {
            if( _pmanip->GetNumFreeParameters() == 0 )
                return numeric::array(boost::python::list());
            vector<dReal> values;
            _pmanip->GetFreeParameters(values);
            return toPyArray(values);
        }

        object FindIKSolution(object oparam, int filteroptions) const
        {
            vector<dReal> solution;
            extract<boost::shared_ptr<PyIkParameterization> > ikparam(oparam);
            if( ikparam.check() ) {
                if( !_pmanip->FindIKSolution(((boost::shared_ptr<PyIkParameterization>)ikparam)->_param,solution,filteroptions) )
                    return object();
            }
            // assume transformation matrix
            else if( !_pmanip->FindIKSolution(ExtractTransform(oparam),solution,filteroptions) )
                return object();
            return toPyArrayN(&solution[0],solution.size());
        }

        object FindIKSolution(object oparam, object freeparams, int filteroptions) const
        {
            vector<dReal> solution, vfreeparams = ExtractArray<dReal>(freeparams);
            extract<boost::shared_ptr<PyIkParameterization> > ikparam(oparam);
            if( ikparam.check() ) {
                if( !_pmanip->FindIKSolution(((boost::shared_ptr<PyIkParameterization>)ikparam)->_param,vfreeparams,solution,filteroptions) )
                    return object();
            }
            // assume transformation matrix
            else if( !_pmanip->FindIKSolution(ExtractTransform(oparam),vfreeparams, solution,filteroptions) )
                return object();
            return toPyArray(solution);
        }

        object FindIKSolutions(object oparam, int filteroptions) const
        {
            std::vector<std::vector<dReal> > vsolutions;
            extract<boost::shared_ptr<PyIkParameterization> > ikparam(oparam);
            boost::python::list solutions;
            if( ikparam.check() ) {
                if( !_pmanip->FindIKSolutions(((boost::shared_ptr<PyIkParameterization>)ikparam)->_param,vsolutions,filteroptions) ) {
                    return solutions;
                }
            }
            // assume transformation matrix
            else if( !_pmanip->FindIKSolutions(ExtractTransform(oparam),vsolutions,filteroptions) ) {
                return solutions;
            }
            FOREACH(itsol,vsolutions) {
                solutions.append(toPyArrayN(&(*itsol)[0],itsol->size()));
            }
            return solutions;
        }

        object FindIKSolutions(object oparam, object freeparams, int filteroptions) const
        {
            std::vector<std::vector<dReal> > vsolutions;
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            extract<boost::shared_ptr<PyIkParameterization> > ikparam(oparam);
            boost::python::list solutions;
            if( ikparam.check() ) {
                if( !_pmanip->FindIKSolutions(((boost::shared_ptr<PyIkParameterization>)ikparam)->_param,vfreeparams,vsolutions,filteroptions) ) {
                    return solutions;
                }
            }
            // assume transformation matrix
            else if( !_pmanip->FindIKSolutions(ExtractTransform(oparam),vfreeparams, vsolutions,filteroptions) ) {
                return solutions;
            }
            FOREACH(itsol,vsolutions) {
                solutions.append(toPyArray(*itsol));
            }
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

        string GetStructureHash() const { return _pmanip->GetStructureHash(); }
        string GetKinematicsStructureHash() const { return _pmanip->GetKinematicsStructureHash(); }
        string __repr__() { return boost::str(boost::format("<env.GetRobot('%s').GetManipulator('%s')>")%_pmanip->GetRobot()->GetName()%_pmanip->GetName()); }
        string __str__() { return boost::str(boost::format("<manipulator:%s, parent=%s>")%_pmanip->GetName()%_pmanip->GetRobot()->GetName()); }
        bool __eq__(boost::shared_ptr<PyManipulator> p) { return !!p && _pmanip==p->_pmanip; }
        bool __ne__(boost::shared_ptr<PyManipulator> p) { return !p || _pmanip!=p->_pmanip; }
    };
    typedef boost::shared_ptr<PyManipulator> PyManipulatorPtr;
    PyManipulatorPtr _GetManipulator(RobotBase::ManipulatorPtr pmanip) {
        return !pmanip ? PyManipulatorPtr() : PyManipulatorPtr(new PyManipulator(pmanip,_pyenv));
    }

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

        boost::shared_ptr<PySensorBase::PySensorData> GetData()
        {
            SensorBase::SensorDataPtr pdata = _pattached->GetData();
            if( !pdata ) {
                return boost::shared_ptr<PySensorBase::PySensorData>();
            }
            return boost::shared_ptr<PySensorBase::PySensorData>(new PySensorBase::PySensorData(pdata));
        }

        void SetRelativeTransform(object transform) { _pattached->SetRelativeTransform(ExtractTransform(transform)); }
        string GetStructureHash() const { return _pattached->GetStructureHash(); }
        string __repr__() { return boost::str(boost::format("<env.GetRobot('%s').GetSensor('%s')>")%_pattached->GetRobot()->GetName()%_pattached->GetName()); }
        string __str__() { return boost::str(boost::format("<attachedsensor:%s, parent=%s>")%_pattached->GetName()%_pattached->GetRobot()->GetName()); }
        bool __eq__(boost::shared_ptr<PyAttachedSensor> p) { return !!p && _pattached==p->_pattached; }
        bool __ne__(boost::shared_ptr<PyAttachedSensor> p) { return !p || _pattached!=p->_pattached; }
    };

    class PyGrabbed
    {
    public:
        PyGrabbed(const RobotBase::Grabbed& grabbed, PyEnvironmentBasePtr pyenv) {
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
            manips.append(_GetManipulator(*it));
        return manips;
    }

    object GetManipulators(const string& manipname)
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname )
                manips.append(_GetManipulator(*it));
        }
        return manips;
    }
    PyManipulatorPtr GetManipulator(const string& manipname)
    {
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname )
                return _GetManipulator(*it);
        }
        return PyManipulatorPtr();
    }

    PyManipulatorPtr SetActiveManipulator(int index) { _probot->SetActiveManipulator(index); return GetActiveManipulator(); }
    PyManipulatorPtr SetActiveManipulator(const std::string& manipname) { _probot->SetActiveManipulator(manipname); return GetActiveManipulator(); }
    PyManipulatorPtr SetActiveManipulator(PyManipulatorPtr pmanip) { _probot->SetActiveManipulator(pmanip->GetName()); return GetActiveManipulator(); }
    PyManipulatorPtr GetActiveManipulator() { return _GetManipulator(_probot->GetActiveManipulator()); }
    int GetActiveManipulatorIndex() const { return _probot->GetActiveManipulatorIndex(); }

    object GetSensors()
    {
        RAVELOG_WARN("GetSensors is deprecated, please use GetAttachedSensors\n");
        return GetAttachedSensors();
    }

    object GetAttachedSensors()
    {
        boost::python::list sensors;
        FOREACH(itsensor, _probot->GetAttachedSensors())
            sensors.append(boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
        return sensors;
    }
    boost::shared_ptr<PyAttachedSensor> GetSensor(const string& sensorname)
    {
        RAVELOG_WARN("GetSensor is deprecated, please use GetAttachedSensor\n");
        return GetAttachedSensor(sensorname);
    }

    boost::shared_ptr<PyAttachedSensor> GetAttachedSensor(const string& sensorname)
    {
        FOREACH(itsensor, _probot->GetAttachedSensors()) {
            if( (*itsensor)->GetName() == sensorname )
                return boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv));
        }
        return boost::shared_ptr<PyAttachedSensor>();
    }
    
    PyControllerBasePtr GetController() const { return !_probot->GetController() ? PyControllerBasePtr() : PyControllerBasePtr(new PyControllerBase(_probot->GetController(),_pyenv)); }
    bool SetController(PyControllerBasePtr pController, const string& args=string("")) {
        if( !pController )
            return _probot->SetController(ControllerBasePtr(),"");
        else
            return _probot->SetController(pController->GetController(),args.c_str());
    }
    
    void SetActiveDOFs(object dofindices) { _probot->SetActiveDOFs(ExtractArray<int>(dofindices)); }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask) { _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask); }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask, object rotationaxis) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
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
            return numeric::array(boost::python::list());
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
            return numeric::array(boost::python::list());
        vector<dReal> values;
        _probot->GetActiveDOFVelocities(values);
        return toPyArray(values);
    }

    object GetActiveDOFLimits() const
    {
        if( _probot->GetActiveDOF() == 0 )
            return numeric::array(boost::python::list());
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

    object GetActiveJointIndices() { RAVELOG_WARN("GetActiveJointIndices deprecated. Use GetActiveDOFIndices\n"); return toPyArray(_probot->GetActiveDOFIndices()); }
    object GetActiveDOFIndices() { return toPyArray(_probot->GetActiveDOFIndices()); }

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
    PyVoidHandle CreateRobotStateSaver(int options) { return PyVoidHandle(boost::shared_ptr<void>(new RobotBase::RobotStateSaver(_probot,options))); }

    virtual string __repr__() { return boost::str(boost::format("<env.GetRobot('%s')>")%_probot->GetName()); }
    virtual string __str__() { return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_probot->GetInterfaceType())%_probot->GetXMLId()%_probot->GetName()%_probot->GetRobotStructureHash()); }
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
    class PyPlannerParameters
    {
        PlannerBase::PlannerParametersPtr _paramswrite;
        PlannerBase::PlannerParametersConstPtr _paramsread;
      public:
        PyPlannerParameters(PlannerBase::PlannerParametersPtr params) : _paramswrite(params), _paramsread(params) {}
        PyPlannerParameters(PlannerBase::PlannerParametersConstPtr params) : _paramsread(params) {}
        virtual ~PyPlannerParameters() {}

        PlannerBase::PlannerParametersConstPtr GetParameters() const { return _paramsread; }
        
        string __repr__() { return boost::str(boost::format("<PlannerParameters(dof=%d)>")%_paramsread->GetDOF()); }
        string __str__() {
            stringstream ss;
            ss << *_paramsread << endl;
            return ss.str();
        }
        bool __eq__(boost::shared_ptr<PyPlannerParameters> p) { return !!p && _paramsread == p->_paramsread; }
        bool __ne__(boost::shared_ptr<PyPlannerParameters> p) { return !p || _paramsread != p->_paramsread; }
    };
        
    PyPlannerBase(PlannerBasePtr pplanner, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pplanner, pyenv), _pplanner(pplanner) {}
    virtual ~PyPlannerBase() {}

    bool InitPlan(PyRobotBasePtr pbase, boost::shared_ptr<PyPlannerParameters> pparams)
    {
        return _pplanner->InitPlan(pbase->GetRobot(),pparams->GetParameters());
    }
    
    bool InitPlan(PyRobotBasePtr pbase, const string& params)
    {
        stringstream ss(params);
        return _pplanner->InitPlan(pbase->GetRobot(),ss);
    }
    
    bool PlanPath(PyTrajectoryBasePtr ptraj, object output);
    
    boost::shared_ptr<PyPlannerParameters> GetParameters() const
    {
        PlannerBase::PlannerParametersConstPtr params = _pplanner->GetParameters();
        if( !params )
            return boost::shared_ptr<PyPlannerParameters>();
        return boost::shared_ptr<PyPlannerParameters>(new PyPlannerParameters(params));
    }
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

PySensorSystemBasePtr PyKinBody::PyManageData::GetSystem()
{
    SensorSystemBasePtr psystem = _pdata->GetSystem();
    return !psystem ? PySensorSystemBasePtr() : PySensorSystemBasePtr(new PySensorSystemBase(psystem,_pyenv));
}

class PyTrajectoryBase : public PyInterfaceBase
{
protected:
    TrajectoryBasePtr _ptrajectory;
public:
    PyTrajectoryBase(TrajectoryBasePtr pTrajectory, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pTrajectory, pyenv),_ptrajectory(pTrajectory) {}
    virtual ~PyTrajectoryBase() {}

    TrajectoryBasePtr GetTrajectory() { return _ptrajectory; }
};

bool PyPlannerBase::PlanPath(PyTrajectoryBasePtr ptraj, object output)
{
    return _pplanner->PlanPath(ptraj->GetTrajectory());
}

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
    bool InitEnvironment() { return _pPhysicsEngine->InitEnvironment(); }
    void DestroyEnvironment() { _pPhysicsEngine->DestroyEnvironment(); }
    bool InitKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _pPhysicsEngine->InitKinBody(pbody->GetBody()); }
    
    bool SetLinkVelocity(PyKinBody::PyLinkPtr plink, object linearvel, object angularvel)
    {
        CHECK_POINTER(plink);
        return _pPhysicsEngine->SetLinkVelocity(plink->GetLink(),ExtractVector3(linearvel),ExtractVector3(angularvel));
    }

    bool SetBodyVelocity(PyKinBodyPtr pbody, object linearvel, object angularvel, object jointvelocity)
    {
        CHECK_POINTER(pbody);
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
            return object();
        }
        return boost::python::make_tuple(toPyVector3(linearvel),toPyVector3(angularvel),toPyArray(vjointvel));
    }

    object GetLinkVelocity(PyKinBody::PyLinkPtr plink)
    {
        CHECK_POINTER(plink);
        Vector linearvel, angularvel;
        if( !_pPhysicsEngine->GetLinkVelocity(plink->GetLink(),linearvel,angularvel) ) {
            return object();
        }
        return boost::python::make_tuple(toPyVector3(linearvel),toPyVector3(angularvel));
    }
    
    object GetBodyVelocityLinks(PyKinBodyPtr pbody)
    {
        CHECK_POINTER(pbody);
        if( pbody->GetBody()->GetLinks().size() == 0 )
            return boost::python::make_tuple(numeric::array(boost::python::list()),numeric::array(boost::python::list()));
        vector<Vector> linearvel(pbody->GetBody()->GetLinks().size()),angularvel(pbody->GetBody()->GetLinks().size());
        if( !_pPhysicsEngine->GetBodyVelocity(pbody->GetBody(),linearvel,angularvel) ) {
            return object();
        }
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
            throw openrave_exception("physics engine failed");
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

class PyViewerBase : public PyInterfaceBase
{
protected:
    ViewerBasePtr _pviewer;

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

    PyViewerBase(ViewerBasePtr pviewer, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pviewer, pyenv), _pviewer(pviewer) {}
    virtual ~PyViewerBase() {}

    ViewerBasePtr GetViewer() { return _pviewer; }

    int main(bool bShow) { return _pviewer->main(bShow); }
    void quitmainloop() { return _pviewer->quitmainloop(); }

    void ViewerSetSize(int w, int h) { _pviewer->ViewerSetSize(w,h); }
    void ViewerMove(int x, int y) { _pviewer->ViewerMove(x,y); }
    void ViewerSetTitle(const string& title) { _pviewer->ViewerSetTitle(title.c_str()); }
    bool LoadModel(const string& filename) { return _pviewer->LoadModel(filename.c_str()); }

    PyVoidHandle RegisterCallback(ViewerBase::ViewerEvents properties, object fncallback)
    {
        if( !fncallback )
            throw openrave_exception("callback not specified");
        boost::shared_ptr<void> p = _pviewer->RegisterCallback(properties,boost::bind(&PyViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p )
            throw openrave_exception("no registration callback returned");
        return PyVoidHandle(p);
    }

    void EnvironmentSync() { return _pviewer->EnvironmentSync(); }

    void SetCamera(object transform) { _pviewer->SetCamera(ExtractTransform(transform)); }
    void SetCamera(object transform, float focalDistance) { _pviewer->SetCamera(ExtractTransform(transform),focalDistance); }

    object GetCameraTransform() { return ReturnTransform(_pviewer->GetCameraTransform()); }

    object GetCameraImage(int width, int height, object extrinsic, object oKK)
    {
        vector<float> vKK = ExtractArray<float>(oKK);
        if( vKK.size() != 4 )
            throw openrave_exception("KK needs to be of size 4");
        SensorBase::CameraIntrinsics KK(vKK[0],vKK[1],vKK[2],vKK[3]);
        vector<uint8_t> memory;
        if( !_pviewer->GetCameraImage(memory, width,height,RaveTransform<float>(ExtractTransform(extrinsic)), KK) )
            throw openrave_exception("failed to get camera image");
        std::vector<npy_intp> dims(3); dims[0] = height; dims[1] = width; dims[2] = 3;
        return toPyArray(memory,dims);
    }
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
        ViewerBasePtr pviewer;
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            pviewer = RaveCreateViewer(_penv, strviewer.c_str());
            if( !!pviewer ) {
                _penv->AttachViewer(pviewer);
            }
            _conditionViewer.notify_all();
        }

        if( !pviewer )
            return;

        pviewer->main(bShowViewer); // spin until quitfrommainloop is called
        _penv->AttachViewer(ViewerBasePtr());
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
        if( !RaveGlobalState() ) {
            RaveInitialize(true);
        }
        _penv = RaveCreateEnvironment();
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
            _penv->AttachViewer(ViewerBasePtr());
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
        RAVELOG_WARN("Environment.GetPluginInfo deprecated, use RaveGetPluginInfo\n");
        return openravepy::RaveGetPluginInfo();
    }

    object GetLoadedInterfaces()
    {
        RAVELOG_WARN("Environment.GetLoadedInterfaces deprecated, use RaveGetLoadedInterfaces\n");
        return openravepy::RaveGetLoadedInterfaces();
    }

    bool LoadPlugin(const string& name) { RAVELOG_WARN("Environment.LoadPlugin deprecated, use RaveLoadPlugin\n");  return RaveLoadPlugin(name.c_str()); }
    void ReloadPlugins() { RAVELOG_WARN("Environment.ReloadPlugins deprecated, use RaveReloadPlugins\n"); return RaveReloadPlugins(); }

    PyInterfaceBasePtr CreateInterface(InterfaceType type, const string& name)
    {
        RAVELOG_WARN("Environment.CreateInterface deprecated, use RaveCreateInterface\n");
        return openravepy::RaveCreateInterface(shared_from_this(),type,name);
    }
    PyRobotBasePtr CreateRobot(const string& name)
    {
        RAVELOG_WARN("Environment.CreateRobot deprecated, use RaveCreateRobot\n");
        return openravepy::RaveCreateRobot(shared_from_this(),name);
    }
    PyPlannerBasePtr CreatePlanner(const string& name)
    {
        RAVELOG_WARN("Environment.CreatePlanner deprecated, use RaveCreatePlanner\n");
        return openravepy::RaveCreatePlanner(shared_from_this(),name);
    }
    PySensorSystemBasePtr CreateSensorSystem(const string& name)
    {
        RAVELOG_WARN("Environment.CreateSensorSystem deprecated, use RaveCreateSensorSystem\n");
        return openravepy::RaveCreateSensorSystem(shared_from_this(),name);
    }
    PyControllerBasePtr CreateController(const string& name)
    {
        RAVELOG_WARN("Environment.CreateController deprecated, use RaveCreateController\n");
        return openravepy::RaveCreateController(shared_from_this(),name);
    }
    PyProblemInstancePtr CreateProblem(const string& name)
    {
        RAVELOG_WARN("Environment.CreateProblem deprecated, use RaveCreateProblem\n");
        return openravepy::RaveCreateProblem(shared_from_this(),name);
    }
    PyIkSolverBasePtr CreateIkSolver(const string& name)
    {
        RAVELOG_WARN("Environment.CreateIkSolver deprecated, use RaveCreateIkSolver\n");
        return openravepy::RaveCreateIkSolver(shared_from_this(),name);
    }
    PyPhysicsEngineBasePtr CreatePhysicsEngine(const string& name)
    {
        RAVELOG_WARN("Environment.CreatePhysicsEngine deprecated, use RaveCreatePhysicsEngine\n");
        return openravepy::RaveCreatePhysicsEngine(shared_from_this(),name);
    }
    PySensorBasePtr CreateSensor(const string& name)
    {
        RAVELOG_WARN("Environment.CreateSensor deprecated, use RaveCreateSensor\n");
        return openravepy::RaveCreateSensor(shared_from_this(),name);
    }
    PyCollisionCheckerBasePtr CreateCollisionChecker(const string& name)
    {
        RAVELOG_WARN("Environment.CreateCollisionChecker deprecated, use RaveCreateCollisionChecker\n");
        return openravepy::RaveCreateCollisionChecker(shared_from_this(),name);
    }
    PyViewerBasePtr CreateViewer(const string& name)
    {
        RAVELOG_WARN("Environment.CreateViewer deprecated, use RaveCreateViewer\n");
        return openravepy::RaveCreateViewer(shared_from_this(),name);
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

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody)
    {
        return _penv->CheckCollision(pyray->r,KinBodyConstPtr(pbody->GetBody()));
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
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
            throw openrave_exception("rays object needs to be a Nx6 vector\n");

        CollisionReport report;
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
            bool bCollision;
            if( !pbody )
                bCollision = _penv->CheckCollision(r, preport);
            else
                bCollision = _penv->CheckCollision(r, KinBodyConstPtr(pbody->GetBody()), preport);
            pcollision[i] = false;
            ppos[0] = 0; ppos[1] = 0; ppos[2] = 0; ppos[3] = 0; ppos[4] = 0; ppos[5] = 0;
            if( bCollision && report.contacts.size() > 0 ) {
                if( !bFrontFacingOnly || report.contacts[0].norm.dot3(r.dir)<0 ) {
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

    bool CheckCollision(boost::shared_ptr<PyRay> pyray)
    {
        return _penv->CheckCollision(pyray->r);
    }
    
    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyCollisionReportPtr pReport)
    {
        bool bSuccess = _penv->CheckCollision(pyray->r, pReport->report);
        pReport->init(shared_from_this());
        return bSuccess;
    }
	
    bool Load(const string& filename) { return _penv->Load(filename); }
    bool Save(const string& filename) { return _penv->Save(filename.c_str()); }

    PyRobotBasePtr ReadRobotXMLFile(const string& filename)
    {
        RobotBasePtr probot = _penv->ReadRobotXMLFile(filename);
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
        KinBodyPtr pbody = _penv->ReadKinBodyXMLFile(filename);
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

    PyInterfaceBasePtr ReadInterfaceXMLFile(const std::string& filename)
    {
        InterfaceBasePtr pbody = _penv->ReadInterfaceXMLFile(filename);
        if( !pbody )
            return PyInterfaceBasePtr();
        switch(pbody->GetInterfaceType()) {
        case PT_Planner: return PyPlannerBasePtr(new PyPlannerBase(boost::static_pointer_cast<PlannerBase>(pbody),shared_from_this()));
        case PT_Robot: return PyRobotBasePtr(new PyRobotBase(boost::static_pointer_cast<RobotBase>(pbody),shared_from_this()));
        case PT_SensorSystem: return PySensorSystemBasePtr(new PySensorSystemBase(boost::static_pointer_cast<SensorSystemBase>(pbody),shared_from_this()));
        case PT_Controller: return PyControllerBasePtr(new PyControllerBase(boost::static_pointer_cast<ControllerBase>(pbody),shared_from_this()));
        case PT_ProblemInstance: return PyProblemInstancePtr(new PyProblemInstance(boost::static_pointer_cast<ProblemInstance>(pbody),shared_from_this()));
        case PT_InverseKinematicsSolver: return PyIkSolverBasePtr(new PyIkSolverBase(boost::static_pointer_cast<IkSolverBase>(pbody),shared_from_this()));
        case PT_KinBody: return PyKinBodyPtr(new PyKinBody(boost::static_pointer_cast<KinBody>(pbody),shared_from_this()));
        case PT_PhysicsEngine: return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(boost::static_pointer_cast<PhysicsEngineBase>(pbody),shared_from_this()));
        case PT_Sensor: return PySensorBasePtr(new PySensorBase(boost::static_pointer_cast<SensorBase>(pbody),shared_from_this()));
        case PT_CollisionChecker: return PyCollisionCheckerBasePtr(new PyCollisionCheckerBase(boost::static_pointer_cast<CollisionCheckerBase>(pbody),shared_from_this()));
        case PT_Trajectory: return PyTrajectoryBasePtr(new PyTrajectoryBase(boost::static_pointer_cast<TrajectoryBase>(pbody),shared_from_this()));
        case PT_Viewer: return PyViewerBasePtr(new PyViewerBase(boost::static_pointer_cast<ViewerBase>(pbody),shared_from_this()));
        }
        return PyInterfaceBasePtr();
    }

    bool AddKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); return _penv->AddKinBody(pbody->GetBody()); }
    bool AddKinBody(PyKinBodyPtr pbody, bool bAnonymous) { CHECK_POINTER(pbody); return _penv->AddKinBody(pbody->GetBody(),bAnonymous); }
    bool AddRobot(PyRobotBasePtr robot) { CHECK_POINTER(robot); return _penv->AddRobot(robot->GetRobot()); }
    bool AddRobot(PyRobotBasePtr robot, bool bAnonymous) { CHECK_POINTER(robot); return _penv->AddRobot(robot->GetRobot(),bAnonymous); }
    bool AddSensor(PySensorBasePtr sensor, const std::string& args) { CHECK_POINTER(sensor); return _penv->AddSensor(sensor->GetSensor(),args); }
    bool AddSensor(PySensorBasePtr sensor, const std::string& args, bool bAnonymous) { CHECK_POINTER(sensor); return _penv->AddSensor(sensor->GetSensor(),args, bAnonymous); }
    bool RemoveKinBody(PyKinBodyPtr pbody) { CHECK_POINTER(pbody); RAVELOG_WARN("openravepy RemoveKinBody deprecated, use Remove\n"); return _penv->Remove(pbody->GetBody()); }
    
    PyKinBodyPtr GetKinBody(const string& name)
    {
        KinBodyPtr pbody = _penv->GetKinBody(name);
        if( !pbody ) {
            return PyKinBodyPtr();
            //throw openrave_exception(boost::str(boost::format("failed to get body %s")%name));
        }
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }
    PyRobotBasePtr GetRobot(const string& name)
    {
        RobotBasePtr probot = _penv->GetRobot(name);
        if( !probot ) {
            return PyRobotBasePtr();
            //throw openrave_exception(boost::str(boost::format("failed to get robot %s")%name));
        }
        return PyRobotBasePtr(new PyRobotBase(probot,shared_from_this()));
    }
    PySensorBasePtr GetSensor(const string& name)
    {
        SensorBasePtr psensor = _penv->GetSensor(name);
        if( !psensor ) {
            return PySensorBasePtr();
            //throw openrave_exception(boost::str(boost::format("failed to get sensor %s")%name));
        }
        return PySensorBasePtr(new PySensorBase(psensor,shared_from_this()));
    }

    PyKinBodyPtr GetBodyFromEnvironmentId(int id)
    {
        KinBodyPtr pbody = _penv->GetBodyFromEnvironmentId(id);
        if( !pbody )
            throw openrave_exception(boost::str(boost::format("failed to get body id %d")%id));
        return PyKinBodyPtr(new PyKinBody(pbody,shared_from_this()));
    }

    PyKinBodyPtr CreateKinBody()
    {
        return PyKinBodyPtr(new PyKinBody(RaveCreateKinBody(_penv),shared_from_this()));
    }

    PyTrajectoryBasePtr CreateTrajectory(int nDOF) { return PyTrajectoryBasePtr(new PyTrajectoryBase(RaveCreateTrajectory(_penv,nDOF),shared_from_this())); }

    int LoadProblem(PyProblemInstancePtr prob, const string& args) { CHECK_POINTER(prob); return _penv->LoadProblem(prob->GetProblem(),args); }
    bool RemoveProblem(PyProblemInstancePtr prob) { CHECK_POINTER(prob); RAVELOG_WARN("openravepy RemoveProblem deprecated, use Remove\n");return _penv->Remove(prob->GetProblem()); }
    bool Remove(PyInterfaceBasePtr interface) { CHECK_POINTER(interface); return _penv->Remove(interface->GetInterfaceBase()); }
    
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
            BOOST_ASSERT(_listenvlocks.size()>0);
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
        
        _penv->AttachViewer(ViewerBasePtr());

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

    PyViewerBasePtr GetViewer() { return PyViewerBasePtr(new PyViewerBase(_penv->GetViewer(),shared_from_this())); }

    /// returns the number of points
    static size_t _getGraphPoints(object opoints, vector<float>& vpoints)
    {
        if( PyObject_HasAttrString(opoints.ptr(),"shape") ) {
            object pointshape = opoints.attr("shape");
            switch(len(pointshape)) {
            case 1:
                vpoints = ExtractArray<float>(opoints);
                if( vpoints.size()%3 ) {
                    throw openrave_exception(boost::str(boost::format("points have bad size %d")%vpoints.size()),ORE_InvalidArguments);
                }
                return vpoints.size()/3;
            case 2: {
                int num = extract<int>(pointshape[0]);
                int dim = extract<int>(pointshape[1]);
                vpoints = ExtractArray<float>(opoints.attr("flat"));
                if(dim != 3) {
                    throw openrave_exception(boost::str(boost::format("points have bad size %dx%d")%num%dim),ORE_InvalidArguments);
                }
                return num;
            }
            default:
                throw openrave_exception("points have bad dimension");
            }
        }
        // assume it is a regular 1D list
        vpoints = ExtractArray<float>(opoints);
        if( vpoints.size()% 3 ) {
            throw openrave_exception(boost::str(boost::format("points have bad size %d")%vpoints.size()),ORE_InvalidArguments);
        }
        return vpoints.size()/3;
    }

    /// returns the number of colors
    static size_t _getGraphColors(object ocolors, vector<float>& vcolors)
    {
        if( ocolors != object() ) {
            if( PyObject_HasAttrString(ocolors.ptr(),"shape") ) {
                object colorshape = ocolors.attr("shape");
                switch( len(colorshape) ) {
                case 1:
                    break;
                case 2: {
                    int numcolors = extract<int>(colorshape[0]);
                    int colordim = extract<int>(colorshape[1]);
                    if( colordim != 3 && colordim != 4 ) {
                        throw openrave_exception("colors dim needs to be 3 or 4");
                    }
                    vcolors = ExtractArray<float>(ocolors.attr("flat"));
                    return numcolors;
                }
                default:
                    throw openrave_exception("colors has wrong number of dimensions",ORE_InvalidArguments);
                }
            }
            vcolors = ExtractArray<float>(ocolors);
            if( vcolors.size() == 3 ) {
                vcolors.push_back(1.0f);
            }
            else if( vcolors.size() != 4 ) {
                throw openrave_exception("colors has incorrect number of values",ORE_InvalidArguments);
            }
            return 1;
        }
        // default
        RaveVector<float> vcolor(1,0.5,0.5,1.0);
        vcolors.resize(4);
        vcolors[0] = 1; vcolors[1] = 0.5f; vcolors[2] = 0.5f; vcolors[3] = 1.0f;
        return 1;
    }

    static pair<size_t,size_t> _getGraphPointsColors(object opoints, object ocolors, vector<float>& vpoints, vector<float>& vcolors)
    {
        size_t numpoints = _getGraphPoints(opoints,vpoints);
        size_t numcolors = _getGraphColors(ocolors,vcolors);
        if( numpoints <= 0 ) {
            throw openrave_exception("points cannot be empty",ORE_InvalidArguments);
        }
        if( numcolors > 1 && numpoints != numcolors ) {
            throw openrave_exception(boost::str(boost::format("number of points (%d) need to match number of colors (%d)")%numpoints%numcolors));
        }
        return make_pair(numpoints,numcolors);
    }

    object plot3(object opoints,float pointsize,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return object(PyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,&vcolors[0],drawstyle,bhasalpha)));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return object(PyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,vcolor,drawstyle)));
    }

    object drawlinestrip(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return object(PyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0])));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return object(PyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor)));
    }

    object drawlinelist(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return object(PyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0])));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return object(PyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor)));
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
        vector<float> vpoints;
        _getGraphPoints(opoints,vpoints);
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
            if( len(shape) == 1 ) {
                return object(PyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,ExtractVector34(ocolors,1.0f))));
            }
            else {
                BOOST_ASSERT(extract<size_t>(shape[0])==vpoints.size()/3);
                return object(PyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,extract<boost::multi_array<float,2> >(ocolors))));
            }
        }
        return object(PyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,RaveVector<float>(1,0.5,0.5,1))));
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

    object GetSensors()
    {
        std::vector<SensorBasePtr> vsensors;
        _penv->GetSensors(vsensors);
        boost::python::list sensors;
        FOREACHC(itsensor, vsensors)
            sensors.append(PySensorBasePtr(new PySensorBase(*itsensor,shared_from_this())));
        return sensors;
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

    string GetHomeDirectory() { RAVELOG_WARN("Environment.GetHomeDirectory is deprecated, use RaveGetHomeDirectory\n"); return RaveGetHomeDirectory(); }
    bool __eq__(PyEnvironmentBasePtr p) { return !!p && _penv==p->_penv; }
    bool __ne__(PyEnvironmentBasePtr p) { return !p || _penv!=p->_penv; }

    EnvironmentBasePtr GetEnv() const { return _penv; }
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
    BOOST_ASSERT(_listStateSavers.size()>0);
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

namespace openravepy
{
    PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name)
    {
        InterfaceBasePtr p = OpenRAVE::RaveCreateInterface(pyenv->GetEnv(), type, name);
        if( !p ) {
            return PyInterfaceBasePtr();
        }
        return PyInterfaceBasePtr(new PyInterfaceBase(p,pyenv));
    }
    
    PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        RobotBasePtr p = OpenRAVE::RaveCreateRobot(pyenv->GetEnv(), name);
        if( !p ) {
            return PyRobotBasePtr();
        }
        return PyRobotBasePtr(new PyRobotBase(p,pyenv));
    }

    PyPlannerBasePtr RaveCreatePlanner(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        PlannerBasePtr p = OpenRAVE::RaveCreatePlanner(pyenv->GetEnv(), name);
        if( !p ) {
            return PyPlannerBasePtr();
        }
        return PyPlannerBasePtr(new PyPlannerBase(p,pyenv));
    }

    PySensorSystemBasePtr RaveCreateSensorSystem(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        SensorSystemBasePtr p = OpenRAVE::RaveCreateSensorSystem(pyenv->GetEnv(), name);
        if( !p ) {
            return PySensorSystemBasePtr();
        }
        return PySensorSystemBasePtr(new PySensorSystemBase(p,pyenv));
    }

    PyControllerBasePtr RaveCreateController(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        ControllerBasePtr p = OpenRAVE::RaveCreateController(pyenv->GetEnv(), name);
        if( !p ) {
            return PyControllerBasePtr();
        }
        return PyControllerBasePtr(new PyControllerBase(p,pyenv));
    }

    PyProblemInstancePtr RaveCreateProblem(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        ProblemInstancePtr p = OpenRAVE::RaveCreateProblem(pyenv->GetEnv(), name);
        if( !p ) {
            return PyProblemInstancePtr();
        }
        return PyProblemInstancePtr(new PyProblemInstance(p,pyenv));
    }

    PyIkSolverBasePtr RaveCreateIkSolver(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        IkSolverBasePtr p = OpenRAVE::RaveCreateIkSolver(pyenv->GetEnv(), name);
        if( !p ) {
            return PyIkSolverBasePtr();
        }
        return PyIkSolverBasePtr(new PyIkSolverBase(p,pyenv));
    }

    PyPhysicsEngineBasePtr RaveCreatePhysicsEngine(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        PhysicsEngineBasePtr p = OpenRAVE::RaveCreatePhysicsEngine(pyenv->GetEnv(), name);
        if( !p ) {
            return PyPhysicsEngineBasePtr();
        }
        return PyPhysicsEngineBasePtr(new PyPhysicsEngineBase(p,pyenv));
    }

    PySensorBasePtr RaveCreateSensor(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        SensorBasePtr p = OpenRAVE::RaveCreateSensor(pyenv->GetEnv(), name);
        if( !p ) {
            return PySensorBasePtr();
        }
        return PySensorBasePtr(new PySensorBase(p,pyenv));
    }

    PyCollisionCheckerBasePtr RaveCreateCollisionChecker(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        CollisionCheckerBasePtr p = OpenRAVE::RaveCreateCollisionChecker(pyenv->GetEnv(), name);
        if( !p ) {
            return PyCollisionCheckerBasePtr();
        }
        return PyCollisionCheckerBasePtr(new PyCollisionCheckerBase(p,pyenv));
    }

    PyViewerBasePtr RaveCreateViewer(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        ViewerBasePtr p = OpenRAVE::RaveCreateViewer(pyenv->GetEnv(), name);
        if( !p ) {
            return PyViewerBasePtr();
        }
        return PyViewerBasePtr(new PyViewerBase(p,pyenv));
    }

    PyKinBodyPtr RaveCreateKinBody(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        KinBodyPtr p = OpenRAVE::RaveCreateKinBody(pyenv->GetEnv(), name);
        if( !p ) {
            return PyKinBodyPtr();
        }
        return PyKinBodyPtr(new PyKinBody(p,pyenv));
    }

    PyTrajectoryBasePtr RaveCreateTrajectory(PyEnvironmentBasePtr pyenv, const std::string& name)
    {
        TrajectoryBasePtr p = OpenRAVE::RaveCreateTrajectory(pyenv->GetEnv(), name);
        if( !p ) {
            return PyTrajectoryBasePtr();
        }
        return PyTrajectoryBasePtr(new PyTrajectoryBase(p,pyenv));
    }
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetCamera_overloads, SetCamera, 2, 4)
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

BOOST_PYTHON_MODULE(openravepy_int)
{
#if BOOST_VERSION >= 103500
    docstring_options doc_options;
    doc_options.disable_cpp_signatures();
    doc_options.enable_py_signatures();
    doc_options.enable_user_defined();
#endif
    import_array();
    numeric::array::set_module_and_type("numpy", "ndarray");
    int_from_int();
    T_from_number<float>();
    T_from_number<double>();
    init_python_bindings();

    typedef return_value_policy< copy_const_reference > return_copy_const_ref;
    class_< openrave_exception >( "_openrave_exception_", DOXY_CLASS(openrave_exception) )
        .def( init<const std::string&>() )
        .def( init<const openrave_exception&>() )
        .def( "message", &openrave_exception::message, return_copy_const_ref() )
        .def( "__str__", &openrave_exception::message, return_copy_const_ref() )
        ;
    //exception_translator<std::exception>();
    exception_translator<openrave_exception>();

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
        ;
    enum_<InterfaceType>("InterfaceType" DOXY_ENUM(InterfaceType))
        .value(RaveGetInterfaceName(PT_Planner).c_str(),PT_Planner)
        .value(RaveGetInterfaceName(PT_Robot).c_str(),PT_Robot)
        .value(RaveGetInterfaceName(PT_SensorSystem).c_str(),PT_SensorSystem)
        .value(RaveGetInterfaceName(PT_Controller).c_str(),PT_Controller)
        .value(RaveGetInterfaceName(PT_ProblemInstance).c_str(),PT_ProblemInstance)
        .value(RaveGetInterfaceName(PT_InverseKinematicsSolver).c_str(),PT_InverseKinematicsSolver)
        .value(RaveGetInterfaceName(PT_KinBody).c_str(),PT_KinBody)
        .value(RaveGetInterfaceName(PT_PhysicsEngine).c_str(),PT_PhysicsEngine)
        .value(RaveGetInterfaceName(PT_Sensor).c_str(),PT_Sensor)
        .value(RaveGetInterfaceName(PT_CollisionChecker).c_str(),PT_CollisionChecker)
        .value(RaveGetInterfaceName(PT_Trajectory).c_str(),PT_Trajectory)
        .value(RaveGetInterfaceName(PT_Viewer).c_str(),PT_Viewer)
        ;
    enum_<CollisionOptions>("CollisionOptions" DOXY_ENUM(CollisionOptions))
        .value("Distance",CO_Distance)
        .value("UseTolerance",CO_UseTolerance)
        .value("Contacts",CO_Contacts)
        .value("RayAnyHit",CO_RayAnyHit)
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

    class_< boost::shared_ptr< void > >("VoidPointer", "Holds auto-managed resources, deleting it releases its shared data.");

    class_<PyEnvironmentBase, boost::shared_ptr<PyEnvironmentBase> > classenv("Environment", DOXY_CLASS(EnvironmentBase));
    class_<PyGraphHandle, boost::shared_ptr<PyGraphHandle> >("GraphHandle", "Holds auto-managed graphh resources, callings close() or deleting it closes the graphs.");
    class_<PyRay, boost::shared_ptr<PyRay> >("Ray", DOXY_CLASS(geometry::ray))
        .def(init<object,object>())
        .def("dir",&PyRay::dir)
        .def("pos",&PyRay::pos)
        .def("__str__",&PyRay::__str__)
        .def("__repr__",&PyRay::__repr__)
        .def_pickle(Ray_pickle_suite())
        ;
    class_<PyAABB, boost::shared_ptr<PyAABB> >("AABB", DOXY_CLASS(geometry::aabb))
        .def(init<object,object>())
        .def("extents",&PyAABB::extents)
        .def("pos",&PyAABB::pos)
        .def("__str__",&PyAABB::__str__)
        .def("__repr__",&PyAABB::__repr__)
        .def_pickle(AABB_pickle_suite())
        ;
    class_<PyInterfaceBase, boost::shared_ptr<PyInterfaceBase> >("Interface", DOXY_CLASS(InterfaceBase), no_init)
        .def("GetInterfaceType",&PyInterfaceBase::GetInterfaceType, DOXY_FN(InterfaceBase,GetInterfaceType))
        .def("GetXMLId",&PyInterfaceBase::GetXMLId, DOXY_FN(InterfaceBase,GetXMLId))
        .def("GetPluginName",&PyInterfaceBase::GetPluginName, DOXY_FN(InterfaceBase,GetPluginName))
        .def("GetDescription",&PyInterfaceBase::GetDescription, DOXY_FN(InterfaceBase,GetDescription))
        .def("GetEnv",&PyInterfaceBase::GetEnv, DOXY_FN(InterfaceBase,GetEnv))
        .def("Clone",&PyInterfaceBase::Clone,args("ref","cloningoptions"), DOXY_FN(InterfaceBase,Clone))
        .def("SetUserData",&PyInterfaceBase::SetUserData,args("data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("GetUserData",&PyInterfaceBase::GetUserData, DOXY_FN(InterfaceBase,GetUserData))
        .def("SendCommand",&PyInterfaceBase::SendCommand,args("cmd"), DOXY_FN(InterfaceBase,SendCommand))
        .def("__repr__", &PyInterfaceBase::__repr__)
        .def("__str__", &PyInterfaceBase::__str__)
        .def("__eq__",&PyInterfaceBase::__eq__)
        .def("__ne__",&PyInterfaceBase::__ne__)
        ;

    class_<PyPluginInfo, boost::shared_ptr<PyPluginInfo> >("PluginInfo", DOXY_CLASS(PLUGININFO),no_init)
        .def_readonly("interfacenames",&PyPluginInfo::interfacenames)
        .def_readonly("version",&PyPluginInfo::version)
        ;

    {    
        bool (PyKinBody::*pkinbodyself)() = &PyKinBody::CheckSelfCollision;
        bool (PyKinBody::*pkinbodyselfr)(PyCollisionReportPtr) = &PyKinBody::CheckSelfCollision;
        void (PyKinBody::*psetdofvalues1)(object) = &PyKinBody::SetDOFValues;
        void (PyKinBody::*psetdofvalues2)(object,object) = &PyKinBody::SetDOFValues;
        PyVoidHandle (PyKinBody::*statesaver1)() = &PyKinBody::CreateKinBodyStateSaver;
        PyVoidHandle (PyKinBody::*statesaver2)(int) = &PyKinBody::CreateKinBodyStateSaver;
        object (PyKinBody::*getdofvalues1)() const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdofvalues2)(object) const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdoflimits1)() const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getdoflimits2)(object) const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getlinks1)() const = &PyKinBody::GetLinks;
        object (PyKinBody::*getlinks2)(object) const = &PyKinBody::GetLinks;
        object (PyKinBody::*getjoints1)() const = &PyKinBody::GetJoints;
        object (PyKinBody::*getjoints2)(object) const = &PyKinBody::GetJoints;
        scope kinbody = class_<PyKinBody, boost::shared_ptr<PyKinBody>, bases<PyInterfaceBase> >("KinBody", DOXY_CLASS(KinBody), no_init)
            .def("InitFromFile",&PyKinBody::InitFromFile,args("filename"),DOXY_FN(KinBody,InitFromFile))
            .def("InitFromData",&PyKinBody::InitFromData,args("data"), DOXY_FN(KinBody,InitFromData))
            .def("InitFromBoxes",&PyKinBody::InitFromBoxes,args("boxes","draw"), DOXY_FN(KinBody,InitFromBoxes "const std::vector< AABB; bool"))
            .def("InitFromTrimesh",&PyKinBody::InitFromTrimesh,args("trimesh","draw"), DOXY_FN(KinBody,InitFromTrimesh))
            .def("SetName", &PyKinBody::SetName,args("name"),DOXY_FN(KinBody,SetName))
            .def("GetName",&PyKinBody::GetName,DOXY_FN(KinBody,GetName))
            .def("GetDOF",&PyKinBody::GetDOF,DOXY_FN(KinBody,GetDOF))
            .def("GetDOFValues",getdofvalues1,DOXY_FN(KinBody,GetDOFValues))
            .def("GetDOFValues",getdofvalues2,args("indices"),DOXY_FN(KinBody,GetDOFValues))
            .def("GetDOFVelocities",&PyKinBody::GetDOFVelocities, DOXY_FN(KinBody,GetDOFVelocities))
            .def("GetDOFLimits",getdoflimits1, DOXY_FN(KinBody,GetDOFLimits))
            .def("GetDOFLimits",getdoflimits2, args("indices"),DOXY_FN(KinBody,GetDOFLimits))
            .def("GetDOFMaxVel",&PyKinBody::GetDOFMaxVel, DOXY_FN(KinBody,GetDOFMaxVel))
            .def("GetDOFWeights",&PyKinBody::GetDOFWeights, DOXY_FN(KinBody,GetDOFWeights))
            .def("GetDOFResolutions",&PyKinBody::GetDOFResolutions, DOXY_FN(KinBody,GetDOFResolutions))
            .def("GetJointValues",&PyKinBody::GetJointValues, DOXY_FN(KinBody,GetJointValues))
            .def("GetJointVelocities",&PyKinBody::GetJointVelocities, DOXY_FN(KinBody,GetJointVelocities))
            .def("GetJointLimits",&PyKinBody::GetJointLimits, DOXY_FN(KinBody,GetJointLimits))
            .def("GetJointMaxVel",&PyKinBody::GetJointMaxVel, DOXY_FN(KinBody,GetJointMaxVel))
            .def("GetJointWeights",&PyKinBody::GetJointWeights, DOXY_FN(KinBody,GetJointWeights))
            .def("GetLinks",getlinks1, DOXY_FN(KinBody,GetLinks))
            .def("GetLinks",getlinks2, args("indices"), DOXY_FN(KinBody,GetLinks))
            .def("GetLink",&PyKinBody::GetLink,args("name"), DOXY_FN(KinBody,GetLink))
            .def("GetJoints",getjoints1, DOXY_FN(KinBody,GetJoints))
            .def("GetJoints",getjoints2, args("indices"), DOXY_FN(KinBody,GetJoints))
            .def("GetPassiveJoints",&PyKinBody::GetPassiveJoints, DOXY_FN(KinBody,GetPassiveJoints))
            .def("GetDependencyOrderedJoints",&PyKinBody::GetDependencyOrderedJoints, DOXY_FN(KinBody,GetDependencyOrderedJoints))
            .def("GetRigidlyAttachedLinks",&PyKinBody::GetRigidlyAttachedLinks,args("linkindex"), DOXY_FN(KinBody,GetRigidlyAttachedLinks))
            .def("GetChain",&PyKinBody::GetChain,args("linkbaseindex","linkendindex"), DOXY_FN(KinBody,GetChain))
            .def("GetJoint",&PyKinBody::GetJoint,args("name"), DOXY_FN(KinBody,GetJoint))
            .def("GetTransform",&PyKinBody::GetTransform, DOXY_FN(KinBody,GetTransform))
            .def("GetBodyTransformations",&PyKinBody::GetBodyTransformations, DOXY_FN(KinBody,GetBodyTransformations))
            .def("SetBodyTransformations",&PyKinBody::SetBodyTransformations,args("transforms"), DOXY_FN(KinBody,SetBodyTransformations))
            .def("GetLinkVelocities",&PyKinBody::GetLinkVelocities, DOXY_FN(KinBody,GetLinkVelocities))
            .def("ComputeAABB",&PyKinBody::ComputeAABB, DOXY_FN(KinBody,ComputeAABB))
            .def("Enable",&PyKinBody::Enable,args("enable"), DOXY_FN(KinBody,Enable))
            .def("IsEnabled",&PyKinBody::IsEnabled, DOXY_FN(KinBody,IsEnabled))
            .def("SetTransform",&PyKinBody::SetTransform,args("transform"), DOXY_FN(KinBody,SetTransform))
            .def("SetJointValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetJointValues "const std::vector; bool"))
            .def("SetJointValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetJointValues "const std::vector; bool"))
            .def("SetDOFValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetJointValues "const std::vector; bool"))
            .def("SetDOFValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetJointValues "const std::vector; bool"))
            .def("SetJointTorques",&PyKinBody::SetJointTorques,args("torques","add"), DOXY_FN(KinBody,SetJointTorques))
            .def("SetTransformWithJointValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetJointValues "const std::vector; const Transform; bool"))
            .def("SetTransformWithDOFValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetJointValues "const std::vector; const Transform; bool"))
            .def("CalculateJacobian",&PyKinBody::CalculateJacobian,args("linkindex","offset"), DOXY_FN(KinBody,CalculateJacobian "int; const Vector; boost::multi_array"))
            .def("CalculateRotationJacobian",&PyKinBody::CalculateRotationJacobian,args("linkindex","quat"), DOXY_FN(KinBody,CalculateRotationJacobian "int; const Vector; boost::multi_array"))
            .def("CalculateAngularVelocityJacobian",&PyKinBody::CalculateAngularVelocityJacobian,args("linkindex"), DOXY_FN(KinBody,CalculateAngularVelocityJacobian "int; boost::multi_array"))
            .def("CheckSelfCollision",pkinbodyself, DOXY_FN(KinBody,CheckSelfCollision))
            .def("CheckSelfCollision",pkinbodyselfr,args("report"), DOXY_FN(KinBody,CheckSelfCollision))
            .def("IsAttached",&PyKinBody::IsAttached,args("body"), DOXY_FN(KinBody,IsAttached))
            .def("GetAttached",&PyKinBody::GetAttached, DOXY_FN(KinBody,GetAttached))
            .def("IsRobot",&PyKinBody::IsRobot, DOXY_FN(KinBody,IsRobot))
            .def("GetEnvironmentId",&PyKinBody::GetEnvironmentId, DOXY_FN(KinBody,GetEnvironmentId))
            .def("DoesAffect",&PyKinBody::DoesAffect,args("jointindex","linkindex"), DOXY_FN(KinBody,DoesAffect))
            .def("WriteForwardKinematics",&PyKinBody::WriteForwardKinematics, DOXY_FN(KinBody,WriteForwardKinematics))
            .def("SetGuiData",&PyKinBody::SetGuiData,args("data"), DOXY_FN(KinBody,SetGuiData))
            .def("GetGuiData",&PyKinBody::GetGuiData, DOXY_FN(KinBody,GetGuiData))
            .def("GetXMLFilename",&PyKinBody::GetXMLFilename, DOXY_FN(InterfaceBase,GetXMLFilename))
            .def("GetNonAdjacentLinks",&PyKinBody::GetNonAdjacentLinks, DOXY_FN(KinBody,GetNonAdjacentLinks))
            .def("GetAdjacentLinks",&PyKinBody::GetAdjacentLinks, DOXY_FN(KinBody,GetAdjacentLinks))
            .def("GetPhysicsData",&PyKinBody::GetPhysicsData, DOXY_FN(KinBody,GetPhysicsData))
            .def("GetCollisionData",&PyKinBody::GetCollisionData, DOXY_FN(KinBody,GetCollisionData))
            .def("GetManageData",&PyKinBody::GetManageData, DOXY_FN(KinBody,GetManageData))
            .def("GetUpdateStamp",&PyKinBody::GetUpdateStamp, DOXY_FN(KinBody,GetUpdateStamp))
            .def("serialize",&PyKinBody::serialize,args("options"), DOXY_FN(KinBody,serialize))
            .def("GetKinematicsGeometryHash",&PyKinBody::GetKinematicsGeometryHash, DOXY_FN(KinBody,GetKinematicsGeometryHash))
            .def("CreateKinBodyStateSaver",statesaver1, "Creates KinBodySaveStater for this body")
            .def("CreateKinBodyStateSaver",statesaver2, args("options"), "Creates KinBodySaveStater for this body")
            .def("__enter__",&PyKinBody::__enter__)
            .def("__exit__",&PyKinBody::__exit__)
            ;

        enum_<KinBody::SaveParameters>("SaveParameters" DOXY_ENUM(SaveParameters))
            .value("LinkTransformation",KinBody::Save_LinkTransformation)
            .value("LinkEnable",KinBody::Save_LinkEnable)
            .value("ActiveDOF",KinBody::Save_ActiveDOF)
            .value("ActiveManipulator",KinBody::Save_ActiveManipulator)
            .value("GrabbedBodies",KinBody::Save_GrabbedBodies)
            ;

        {
            scope link = class_<PyKinBody::PyLink, boost::shared_ptr<PyKinBody::PyLink> >("Link", DOXY_CLASS(KinBody::Link), no_init)
                .def("GetName",&PyKinBody::PyLink::GetName, DOXY_FN(KinBody::Link,GetName))
                .def("GetIndex",&PyKinBody::PyLink::GetIndex, DOXY_FN(KinBody::Link,GetIndex))
                .def("IsEnabled",&PyKinBody::PyLink::IsEnabled, DOXY_FN(KinBody::Link,IsEnabled))
                .def("IsStatic",&PyKinBody::PyLink::IsStatic, DOXY_FN(KinBody::Link,IsStatic))
                .def("Enable",&PyKinBody::PyLink::Enable,args("enable"), DOXY_FN(KinBody::Link,Enable))
                .def("GetParent",&PyKinBody::PyLink::GetParent, DOXY_FN(KinBody::Link,GetParent))
                .def("GetParentLink",&PyKinBody::PyLink::GetParentLink, DOXY_FN(KinBody::Link,GetParentLink))
                .def("GetCollisionData",&PyKinBody::PyLink::GetCollisionData, DOXY_FN(KinBody::Link,GetCollisionData))
                .def("ComputeAABB",&PyKinBody::PyLink::ComputeAABB, DOXY_FN(KinBody::Link,ComputeAABB))
                .def("GetTransform",&PyKinBody::PyLink::GetTransform, DOXY_FN(KinBody::Link,GetTransform))
                .def("GetCOMOffset",&PyKinBody::PyLink::GetCOMOffset, DOXY_FN(KinBody::Link,GetCOMOffset))
                .def("GetInertia",&PyKinBody::PyLink::GetInertia, DOXY_FN(KinBody::Link,GetInertia))
                .def("GetMass",&PyKinBody::PyLink::GetMass, DOXY_FN(KinBody::Link,GetMass))
                .def("SetTransform",&PyKinBody::PyLink::SetTransform,args("transform"), DOXY_FN(KinBody::Link,SetTransform))
                .def("SetForce",&PyKinBody::PyLink::SetForce,args("force","pos","add"), DOXY_FN(KinBody::Link,SetForce))
                .def("SetTorque",&PyKinBody::PyLink::SetTorque,args("torque","add"), DOXY_FN(KinBody::Link,SetTorque))
                .def("GetGeometries",&PyKinBody::PyLink::GetGeometries, DOXY_FN(KinBody::Link,GetGeometries))
                .def("__repr__", &PyKinBody::PyLink::__repr__)
                .def("__str__", &PyKinBody::PyLink::__str__)
                .def("__eq__",&PyKinBody::PyLink::__eq__)
                .def("__ne__",&PyKinBody::PyLink::__ne__)
                ;

            class_<PyKinBody::PyLink::PyTriMesh, boost::shared_ptr<PyKinBody::PyLink::PyTriMesh> >("TriMesh", DOXY_CLASS(KinBody::Link::TRIMESH))
                .def(init<object,object>())
                .def_readwrite("vertices",&PyKinBody::PyLink::PyTriMesh::vertices)
                .def_readwrite("indices",&PyKinBody::PyLink::PyTriMesh::indices)
                .def("__str__",&PyKinBody::PyLink::PyTriMesh::__str__)
                ;

            {
                scope geomproperties = class_<PyKinBody::PyLink::PyGeomProperties, boost::shared_ptr<PyKinBody::PyLink::PyGeomProperties> >("GeomProperties", DOXY_CLASS(KinBody::Link::GEOMPROPERTIES),no_init)
                    .def("SetCollisionMesh",&PyKinBody::PyLink::PyGeomProperties::SetCollisionMesh,args("trimesh"), DOXY_FN(KinBody::Link::GEOMPROPERTIES,SetCollisionMesh))
                    .def("GetCollisionMesh",&PyKinBody::PyLink::PyGeomProperties::GetCollisionMesh, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetCollisionMesh))
                    .def("SetDraw",&PyKinBody::PyLink::PyGeomProperties::SetDraw,args("draw"), DOXY_FN(KinBody::Link::GEOMPROPERTIES,SetDraw))
                    .def("SetTransparency",&PyKinBody::PyLink::PyGeomProperties::SetTransparency,args("transparency"), DOXY_FN(KinBody::Link::GEOMPROPERTIES,SetTransparency))
                    .def("SetDiffuseColor",&PyKinBody::PyLink::PyGeomProperties::SetDiffuseColor,args("color"), DOXY_FN(KinBody::Link::GEOMPROPERTIES,SetDiffuseColor))
                    .def("SetAmbientColor",&PyKinBody::PyLink::PyGeomProperties::SetAmbientColor,args("color"), DOXY_FN(KinBody::Link::GEOMPROPERTIES,SetAmbientColor))
                    .def("IsDraw",&PyKinBody::PyLink::PyGeomProperties::IsDraw, DOXY_FN(KinBody::Link::GEOMPROPERTIES,IsDraw))
                    .def("IsModifiable",&PyKinBody::PyLink::PyGeomProperties::IsModifiable, DOXY_FN(KinBody::Link::GEOMPROPERTIES,IsModifiable))
                    .def("GetType",&PyKinBody::PyLink::PyGeomProperties::GetType, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetType))
                    .def("GetTransform",&PyKinBody::PyLink::PyGeomProperties::GetTransform, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetTransform))
                    .def("GetSphereRadius",&PyKinBody::PyLink::PyGeomProperties::GetSphereRadius, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetSphereRadius))
                    .def("GetCylinderRadius",&PyKinBody::PyLink::PyGeomProperties::GetCylinderRadius, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetCylinderRadius))
                    .def("GetCylinderHeight",&PyKinBody::PyLink::PyGeomProperties::GetCylinderHeight, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetCylinderHeight))
                    .def("GetBoxExtents",&PyKinBody::PyLink::PyGeomProperties::GetBoxExtents, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetBoxExtents))
                    .def("GetRenderScale",&PyKinBody::PyLink::PyGeomProperties::GetRenderScale, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetRenderScale))
                    .def("GetRenderFilename",&PyKinBody::PyLink::PyGeomProperties::GetRenderFilename, DOXY_FN(KinBody::Link::GEOMPROPERTIES,GetRenderFilename))
                    .def("__eq__",&PyKinBody::PyLink::PyGeomProperties::__eq__)
                    .def("__ne__",&PyKinBody::PyLink::PyGeomProperties::__ne__)
                    ;
                enum_<KinBody::Link::GEOMPROPERTIES::GeomType>("Type" DOXY_ENUM(GeomType))
                    .value("None",KinBody::Link::GEOMPROPERTIES::GeomNone)
                    .value("Box",KinBody::Link::GEOMPROPERTIES::GeomBox)
                    .value("Sphere",KinBody::Link::GEOMPROPERTIES::GeomSphere)
                    .value("Cylinder",KinBody::Link::GEOMPROPERTIES::GeomCylinder)
                    .value("Trimesh",KinBody::Link::GEOMPROPERTIES::GeomTrimesh)
                    ;
            }
        }

        {
            scope joint = class_<PyKinBody::PyJoint, boost::shared_ptr<PyKinBody::PyJoint> >("Joint", DOXY_CLASS(KinBody::Joint),no_init)
                .def("GetName", &PyKinBody::PyJoint::GetName, DOXY_FN(KinBody::Joint,GetName))
                .def("GetMimicJointIndex", &PyKinBody::PyJoint::GetMimicJointIndex, DOXY_FN(KinBody::Joint,GetMimicJointIndex))
                .def("GetMimicCoeffs", &PyKinBody::PyJoint::GetMimicCoeffs, DOXY_FN(KinBody::Joint,GetMimicCoeffs))
                .def("GetMaxVel", &PyKinBody::PyJoint::GetMaxVel, DOXY_FN(KinBody::Joint,GetMaxVel))
                .def("GetMaxAccel", &PyKinBody::PyJoint::GetMaxAccel, DOXY_FN(KinBody::Joint,GetMaxAccel))
                .def("GetMaxTorque", &PyKinBody::PyJoint::GetMaxTorque, DOXY_FN(KinBody::Joint,GetMaxTorque))
                .def("GetDOFIndex", &PyKinBody::PyJoint::GetDOFIndex, DOXY_FN(KinBody::Joint,GetDOFIndex))
                .def("GetJointIndex", &PyKinBody::PyJoint::GetJointIndex, DOXY_FN(KinBody::Joint,GetJointIndex))
                .def("GetParent", &PyKinBody::PyJoint::GetParent, DOXY_FN(KinBody::Joint,GetParent))
                .def("GetFirstAttached", &PyKinBody::PyJoint::GetFirstAttached, DOXY_FN(KinBody::Joint,GetFirstAttached))
                .def("GetSecondAttached", &PyKinBody::PyJoint::GetSecondAttached, DOXY_FN(KinBody::Joint,GetSecondAttached))
                .def("IsStatic",&PyKinBody::PyJoint::IsStatic, DOXY_FN(KinBody::Joint,IsStatic))
                .def("IsCircular",&PyKinBody::PyJoint::IsCircular, DOXY_FN(KinBody::Joint,IsCircular))
                .def("GetType", &PyKinBody::PyJoint::GetType, DOXY_FN(KinBody::Joint,GetType))
                .def("GetDOF", &PyKinBody::PyJoint::GetDOF, DOXY_FN(KinBody::Joint,GetDOF))
                .def("GetValues", &PyKinBody::PyJoint::GetValues, DOXY_FN(KinBody::Joint,GetValues))
                .def("GetVelocities", &PyKinBody::PyJoint::GetVelocities, DOXY_FN(KinBody::Joint,GetVelocities))
                .def("GetAnchor", &PyKinBody::PyJoint::GetAnchor, DOXY_FN(KinBody::Joint,GetAnchor))
                .def("GetAxis", &PyKinBody::PyJoint::GetAxis,args("axis"), DOXY_FN(KinBody::Joint,GetAxis))
                .def("GetInternalHierarchyAnchor", &PyKinBody::PyJoint::GetInternalHierarchyAnchor, DOXY_FN(KinBody::Joint,GetInternalHierarchyAnchor))
                .def("GetInternalHierarchyAxis", &PyKinBody::PyJoint::GetInternalHierarchyAxis,args("axis"), DOXY_FN(KinBody::Joint,GetInternalHierarchyAxis))
                .def("GetInternalHierarchyLeftTransform",&PyKinBody::PyJoint::GetInternalHierarchyLeftTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyLeftTransform))
                .def("GetInternalHierarchyRightTransform",&PyKinBody::PyJoint::GetInternalHierarchyRightTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyRightTransform))
                .def("GetLimits", &PyKinBody::PyJoint::GetLimits, DOXY_FN(KinBody::Joint,GetLimits))
                .def("GetWeights", &PyKinBody::PyJoint::GetWeights, DOXY_FN(KinBody::Joint,GetWeight))
                .def("SetJointOffset",&PyKinBody::PyJoint::SetJointOffset,args("offset"), DOXY_FN(KinBody::Joint,SetJointOffset))
                .def("SetJointLimits",&PyKinBody::PyJoint::SetJointLimits,args("lower","upper"), DOXY_FN(KinBody::Joint,SetJointLimits))
                .def("SetResolution",&PyKinBody::PyJoint::SetResolution,args("resolution"), DOXY_FN(KinBody::Joint,SetResolution))
                .def("SetWeights",&PyKinBody::PyJoint::SetWeights,args("weights"), DOXY_FN(KinBody::Joint,SetWeights))
                .def("AddTorque",&PyKinBody::PyJoint::AddTorque,args("torques"), DOXY_FN(KinBody::Joint,AddTorque))
                .def("__repr__", &PyKinBody::PyJoint::__repr__)
                .def("__str__", &PyKinBody::PyJoint::__str__)
                .def("__eq__",&PyKinBody::PyJoint::__eq__)
                .def("__ne__",&PyKinBody::PyJoint::__ne__)
                ;
            
            enum_<KinBody::Joint::JointType>("Type" DOXY_ENUM(JointType))
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

        {
            scope managedata = class_<PyKinBody::PyManageData, boost::shared_ptr<PyKinBody::PyManageData> >("ManageData", DOXY_CLASS(KinBody::ManageData),no_init)
                .def("GetSystem", &PyKinBody::PyManageData::GetSystem, DOXY_FN(KinBody::ManageData,GetSystem))
                .def("GetData", &PyKinBody::PyManageData::GetData, DOXY_FN(KinBody::ManageData,GetData))
                .def("GetOffsetLink", &PyKinBody::PyManageData::GetOffsetLink, DOXY_FN(KinBody::ManageData,GetOffsetLink))
                .def("IsPresent", &PyKinBody::PyManageData::IsPresent, DOXY_FN(KinBody::ManageData,IsPresent)) 
                .def("IsEnabled", &PyKinBody::PyManageData::IsEnabled, DOXY_FN(KinBody::ManageData,IsEnabled))
                .def("IsLocked", &PyKinBody::PyManageData::IsLocked, DOXY_FN(KinBody::ManageData,IsLocked))
                .def("Lock", &PyKinBody::PyManageData::Lock,args("dolock"), DOXY_FN(KinBody::ManageData,Lock))
                .def("__repr__", &PyKinBody::PyJoint::__repr__)
                .def("__str__", &PyKinBody::PyJoint::__str__)
                .def("__eq__",&PyKinBody::PyJoint::__eq__)
                .def("__ne__",&PyKinBody::PyJoint::__ne__)
                ;
        }
    }

    class_<PyCollisionReport::PYCONTACT, boost::shared_ptr<PyCollisionReport::PYCONTACT> >("Contact", DOXY_CLASS(CollisionReport::CONTACT))
        .def_readonly("pos",&PyCollisionReport::PYCONTACT::pos)
        .def_readonly("norm",&PyCollisionReport::PYCONTACT::norm)
        .def_readonly("depth",&PyCollisionReport::PYCONTACT::depth)
        .def("__str__",&PyCollisionReport::PYCONTACT::__str__)
        ;
    class_<PyCollisionReport, boost::shared_ptr<PyCollisionReport> >("CollisionReport", DOXY_CLASS(CollisionReport))
        .def_readonly("options",&PyCollisionReport::options)
        .def_readonly("plink1",&PyCollisionReport::plink1)
        .def_readonly("plink2",&PyCollisionReport::plink2)
        .def_readonly("numCols",&PyCollisionReport::numCols)
        .def_readonly("minDistance",&PyCollisionReport::minDistance)
        .def_readonly("numWithinTol",&PyCollisionReport::numWithinTol)
        .def_readonly("contacts",&PyCollisionReport::contacts)
        .def("__str__",&PyCollisionReport::__str__)
        ;

    {
        scope ikparameterization = class_<PyIkParameterization, boost::shared_ptr<PyIkParameterization> >("IkParameterization", DOXY_CLASS(IkParameterization))
            .def(init<object,IkParameterization::Type>())
            .def("SetTransform",&PyIkParameterization::SetTransform,args("transform"), DOXY_FN(IkParameterization,SetTransform))
            .def("SetRotation",&PyIkParameterization::SetRotation,args("quat"), DOXY_FN(IkParameterization,SetRotation))
            .def("SetTranslation",&PyIkParameterization::SetTranslation,args("pos"), DOXY_FN(IkParameterization,SetTranslation))
            .def("SetDirection",&PyIkParameterization::SetDirection,args("dir"), DOXY_FN(IkParameterization,SetDirection))
            .def("SetRay",&PyIkParameterization::SetRay,args("quat"), DOXY_FN(IkParameterization,SetRay))
            .def("GetType",&PyIkParameterization::GetType, DOXY_FN(IkParameterization,GetType))
            .def("GetTransform",&PyIkParameterization::GetTransform, DOXY_FN(IkParameterization,GetTransform))
            .def("GetRotation",&PyIkParameterization::GetRotation, DOXY_FN(IkParameterization,GetRotation))
            .def("GetTranslation",&PyIkParameterization::GetTranslation, DOXY_FN(IkParameterization,GetTranslation))
            .def("GetDirection",&PyIkParameterization::GetDirection, DOXY_FN(IkParameterization,GetDirection))
            .def("GetRay",&PyIkParameterization::GetRay, DOXY_FN(IkParameterization,GetRay))
            .def_pickle(IkParameterization_pickle_suite())
            ;

        enum_<IkParameterization::Type>("Type" DOXY_ENUM(IkParameterization::Type))
            .value("Transform6D",IkParameterization::Type_Transform6D)
            .value("Rotation3D",IkParameterization::Type_Rotation3D)
            .value("Translation3D",IkParameterization::Type_Translation3D)
            .value("Direction3D",IkParameterization::Type_Direction3D)
            .value("Ray4D",IkParameterization::Type_Ray4D)
            .value("Lookat3D",IkParameterization::Type_Lookat3D)
        ;
    }

    {
        void (PyRobotBase::*psetactivedofs1)(object) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs2)(object, int) = &PyRobotBase::SetActiveDOFs;
        void (PyRobotBase::*psetactivedofs3)(object, int, object) = &PyRobotBase::SetActiveDOFs;

        bool (PyRobotBase::*pgrab1)(PyKinBodyPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab2)(PyKinBodyPtr,object) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab3)(PyKinBodyPtr,PyKinBody::PyLinkPtr) = &PyRobotBase::Grab;
        bool (PyRobotBase::*pgrab4)(PyKinBodyPtr,PyKinBody::PyLinkPtr,object) = &PyRobotBase::Grab;

        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator1)(int) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator2)(const std::string&) = &PyRobotBase::SetActiveManipulator;
        PyRobotBase::PyManipulatorPtr (PyRobotBase::*setactivemanipulator3)(PyRobotBase::PyManipulatorPtr) = &PyRobotBase::SetActiveManipulator;

        object (PyRobotBase::*GetManipulators1)() = &PyRobotBase::GetManipulators;
        object (PyRobotBase::*GetManipulators2)(const string&) = &PyRobotBase::GetManipulators;
        PyVoidHandle (PyRobotBase::*statesaver1)() = &PyRobotBase::CreateRobotStateSaver;
        PyVoidHandle (PyRobotBase::*statesaver2)(int) = &PyRobotBase::CreateRobotStateSaver;
        scope robot = class_<PyRobotBase, boost::shared_ptr<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
            .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
            .def("GetManipulators",GetManipulators2,args("manipname"), DOXY_FN(RobotBase,GetManipulators))
            .def("GetManipulator",&PyRobotBase::GetManipulator,args("manipname"), "Return the manipulator whose name matches")
            .def("SetActiveManipulator",setactivemanipulator1,args("manipindex"), DOXY_FN(RobotBase,SetActiveManipulator "int"))
            .def("SetActiveManipulator",setactivemanipulator2,args("manipname"), DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
            .def("SetActiveManipulator",setactivemanipulator3,args("manip"), "Set the active manipulator given a pointer")
            .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
            .def("GetActiveManipulatorIndex",&PyRobotBase::GetActiveManipulatorIndex, DOXY_FN(RobotBase,GetActiveManipulatorIndex))
            .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
            .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,args("sensorname"), "Return the attached sensor whose name matches")
            .def("GetSensors",&PyRobotBase::GetSensors)
            .def("GetSensor",&PyRobotBase::GetSensor,args("sensorname"))
            .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
            .def("SetController",&PyRobotBase::SetController,SetController_overloads(args("controller","args"), DOXY_FN(RobotBase,SetController)))
            .def("SetActiveDOFs",psetactivedofs1,args("dofindices"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
            .def("SetActiveDOFs",psetactivedofs2,args("dofindices","affine"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int"))
            .def("SetActiveDOFs",psetactivedofs3,args("dofindices","affine","rotationaxis"), DOXY_FN(RobotBase,SetActiveDOFs "const std::vector; int; const Vector"))
            .def("GetActiveDOF",&PyRobotBase::GetActiveDOF, DOXY_FN(RobotBase,GetActiveDOF))
            .def("GetAffineDOF",&PyRobotBase::GetAffineDOF, DOXY_FN(RobotBase,GetAffineDOF))
            .def("GetAffineDOFIndex",&PyRobotBase::GetAffineDOFIndex,args("index"), DOXY_FN(RobotBase,GetAffineDOFIndex))
            .def("GetAffineRotationAxis",&PyRobotBase::GetAffineRotationAxis, DOXY_FN(RobotBase,GetAffineRotationAxis))
            .def("SetAffineTranslationLimits",&PyRobotBase::SetAffineTranslationLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationLimits))
            .def("SetAffineRotationAxisLimits",&PyRobotBase::SetAffineRotationAxisLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotationAxisLimits))
            .def("SetAffineRotation3DLimits",&PyRobotBase::SetAffineRotation3DLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotation3DLimits))
            .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,args("lower","upper"), DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
            .def("SetAffineTranslationMaxVels",&PyRobotBase::SetAffineTranslationMaxVels,args("lower","upper"), DOXY_FN(RobotBase,SetAffineTranslationMaxVels))
            .def("SetAffineRotationAxisMaxVels",&PyRobotBase::SetAffineRotationAxisMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationAxisMaxVels))
            .def("SetAffineRotation3DMaxVels",&PyRobotBase::SetAffineRotation3DMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotation3DMaxVels))
            .def("SetAffineRotationQuatMaxVels",&PyRobotBase::SetAffineRotationQuatMaxVels,args("velocity"), DOXY_FN(RobotBase,SetAffineRotationQuatMaxVels))
            .def("SetAffineTranslationResolution",&PyRobotBase::SetAffineTranslationResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineTranslationResolution))
            .def("SetAffineRotationAxisResolution",&PyRobotBase::SetAffineRotationAxisResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationAxisResolution))
            .def("SetAffineRotation3DResolution",&PyRobotBase::SetAffineRotation3DResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotation3DResolution))
            .def("SetAffineRotationQuatResolution",&PyRobotBase::SetAffineRotationQuatResolution,args("resolution"), DOXY_FN(RobotBase,SetAffineRotationQuatResolution))
            .def("SetAffineTranslationWeights",&PyRobotBase::SetAffineTranslationWeights,args("weights"), DOXY_FN(RobotBase,SetAffineTranslationWeights))
            .def("SetAffineRotationAxisWeights",&PyRobotBase::SetAffineRotationAxisWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationAxisWeights))
            .def("SetAffineRotation3DWeights",&PyRobotBase::SetAffineRotation3DWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotation3DWeights))
            .def("SetAffineRotationQuatWeights",&PyRobotBase::SetAffineRotationQuatWeights,args("weights"), DOXY_FN(RobotBase,SetAffineRotationQuatWeights))
            .def("GetAffineTranslationLimits",&PyRobotBase::GetAffineTranslationLimits, DOXY_FN(RobotBase,GetAffineTranslationLimits))
            .def("GetAffineRotationAxisLimits",&PyRobotBase::GetAffineRotationAxisLimits, DOXY_FN(RobotBase,GetAffineRotationAxisLimits))
            .def("GetAffineRotation3DLimits",&PyRobotBase::GetAffineRotation3DLimits, DOXY_FN(RobotBase,GetAffineRotation3DLimits))
            .def("GetAffineRotationQuatLimits",&PyRobotBase::GetAffineRotationQuatLimits, DOXY_FN(RobotBase,GetAffineRotationQuatLimits))
            .def("GetAffineTranslationMaxVels",&PyRobotBase::GetAffineTranslationMaxVels, DOXY_FN(RobotBase,GetAffineTranslationMaxVels))
            .def("GetAffineRotationAxisMaxVels",&PyRobotBase::GetAffineRotationAxisMaxVels, DOXY_FN(RobotBase,GetAffineRotationAxisMaxVels))
            .def("GetAffineRotation3DMaxVels",&PyRobotBase::GetAffineRotation3DMaxVels, DOXY_FN(RobotBase,GetAffineRotation3DMaxVels))
            .def("GetAffineRotationQuatMaxVels",&PyRobotBase::GetAffineRotationQuatMaxVels, DOXY_FN(RobotBase,GetAffineRotationQuatMaxVels))
            .def("GetAffineTranslationResolution",&PyRobotBase::GetAffineTranslationResolution, DOXY_FN(RobotBase,GetAffineTranslationResolution))
            .def("GetAffineRotationAxisResolution",&PyRobotBase::GetAffineRotationAxisResolution, DOXY_FN(RobotBase,GetAffineRotationAxisResolution))
            .def("GetAffineRotation3DResolution",&PyRobotBase::GetAffineRotation3DResolution, DOXY_FN(RobotBase,GetAffineRotation3DResolution))
            .def("GetAffineRotationQuatResolution",&PyRobotBase::GetAffineRotationQuatResolution, DOXY_FN(RobotBase,GetAffineRotationQuatResolution))
            .def("GetAffineTranslationWeights",&PyRobotBase::GetAffineTranslationWeights, DOXY_FN(RobotBase,GetAffineTranslationWeights))
            .def("GetAffineRotationAxisWeights",&PyRobotBase::GetAffineRotationAxisWeights, DOXY_FN(RobotBase,GetAffineRotationAxisWeights))
            .def("GetAffineRotation3DWeights",&PyRobotBase::GetAffineRotation3DWeights, DOXY_FN(RobotBase,GetAffineRotation3DWeights))
            .def("GetAffineRotationQuatWeights",&PyRobotBase::GetAffineRotationQuatWeights, DOXY_FN(RobotBase,GetAffineRotationQuatWeights))
            .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,args("values"), DOXY_FN(RobotBase,SetActiveDOFValues))
            .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
            .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, DOXY_FN(RobotBase,SetActiveDOFVelocities))
            .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
            .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
            .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
            .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
            .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,args("linkindex","offset"), DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; boost::multi_array"))
            .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,args("linkindex","quat"), DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; boost::multi_array"))
            .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,args("linkindex"), DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; boost::multi_array"))
            .def("Grab",pgrab1,args("body"), DOXY_FN(RobotBase,Grab "KinBodyPtr"))
            .def("Grab",pgrab2,args("body","linkstoignre"), DOXY_FN(RobotBase,Grab "KinBodyPtr; const std::set"))
            .def("Grab",pgrab3,args("body","grablink"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr"))
            .def("Grab",pgrab4,args("body","grablink","linkstoignore"), DOXY_FN(RobotBase,Grab "KinBodyPtr; LinkPtr; const std::set"))
            .def("Release",&PyRobotBase::Release,args("body"), DOXY_FN(RobotBase,Release))
            .def("ReleaseAllGrabbed",&PyRobotBase::ReleaseAllGrabbed, DOXY_FN(RobotBase,ReleaseAllGrabbed))
            .def("RegrabAll",&PyRobotBase::RegrabAll, DOXY_FN(RobotBase,RegrabAll))
            .def("IsGrabbing",&PyRobotBase::IsGrabbing,args("body"), DOXY_FN(RobotBase,IsGrabbing))
            .def("GetGrabbed",&PyRobotBase::GetGrabbed, DOXY_FN(RobotBase,GetGrabbed))
            .def("WaitForController",&PyRobotBase::WaitForController,args("timeout"), "Wait until the robot controller is done")
            .def("GetRobotStructureHash",&PyRobotBase::GetRobotStructureHash, DOXY_FN(RobotBase,GetRobotStructureHash))
            .def("CreateRobotStateSaver",statesaver1,"Creates RobotSaveStater for this robot.")
            .def("CreateRobotStateSaver",statesaver2,args("options"),"Creates RobotSaveStater for this robot.")
            .def("__repr__", &PyRobotBase::__repr__)
            .def("__str__", &PyRobotBase::__str__)
            ;
        
        object (PyRobotBase::PyManipulator::*pmanipik)(object, int) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision2)(object,PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        class_<PyRobotBase::PyManipulator, boost::shared_ptr<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
            .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetEndEffectorTransform, DOXY_FN(RobotBase::Manipulator,GetEndEffectorTransform))
            .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
            .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
            .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
            .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
            .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
            .def("InitIKSolver",&PyRobotBase::PyManipulator::InitIKSolver,args("iksolver"), DOXY_FN(RobotBase::Manipulator,InitIKSolver))
            .def("GetIKSolverName",&PyRobotBase::PyManipulator::GetIKSolverName, DOXY_FN(RobotBase::Manipulator,GetIKSolverName))
            .def("HasIKSolver",&PyRobotBase::PyManipulator::HasIKSolver, DOXY_FN(RobotBase::Manipulator,HasIKSolver))
            .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
            .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
            .def("FindIKSolution",pmanipik,args("param","filteroptions"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int"))
            .def("FindIKSolution",pmanipikf,args("param","freevalues","filteroptions"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int"))
            .def("FindIKSolutions",pmanipiks,args("param","filteroptions"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int"))
            .def("FindIKSolutions",pmanipiksf,args("param","freevalues","filteroptions"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int"))
            .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
            .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
            .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetGraspTransform))
            .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
            .def("GetGripperIndices",&PyRobotBase::PyManipulator::GetGripperIndices, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
            .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
            .def("GetArmIndices",&PyRobotBase::PyManipulator::GetArmIndices, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
            .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection, DOXY_FN(RobotBase::Manipulator,GetClosingDirection))
            .def("GetPalmDirection",&PyRobotBase::PyManipulator::GetPalmDirection, DOXY_FN(RobotBase::Manipulator,GetDirection))
            .def("GetDirection",&PyRobotBase::PyManipulator::GetDirection, DOXY_FN(RobotBase::Manipulator,GetDirection))
            .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,args("body"), DOXY_FN(RobotBase::Manipulator,IsGrabbing))
            .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
            .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
            .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
            .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
            .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,args("transform"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
            .def("CheckEndEffectorCollision",pCheckEndEffectorCollision2,args("transform","report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
            .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
            .def("CheckIndependentCollision",pCheckIndependentCollision2,args("report"), DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
            .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
            .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
            .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
            .def("__str__",&PyRobotBase::PyManipulator::__str__)
            .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
            .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
            ;

        class_<PyRobotBase::PyAttachedSensor, boost::shared_ptr<PyRobotBase::PyAttachedSensor> >("AttachedSensor", DOXY_CLASS(RobotBase::AttachedSensor), no_init)
            .def("GetSensor",&PyRobotBase::PyAttachedSensor::GetSensor, DOXY_FN(RobotBase::AttachedSensor,GetSensor))
            .def("GetAttachingLink",&PyRobotBase::PyAttachedSensor::GetAttachingLink, DOXY_FN(RobotBase::AttachedSensor,GetAttachingLink))
            .def("GetRelativeTransform",&PyRobotBase::PyAttachedSensor::GetRelativeTransform, DOXY_FN(RobotBase::AttachedSensor,GetRelativeTransform))
            .def("GetTransform",&PyRobotBase::PyAttachedSensor::GetTransform, DOXY_FN(RobotBase::AttachedSensor,GetTransform))
            .def("GetRobot",&PyRobotBase::PyAttachedSensor::GetRobot, DOXY_FN(RobotBase::AttachedSensor,GetRobot))
            .def("GetName",&PyRobotBase::PyAttachedSensor::GetName, DOXY_FN(RobotBase::AttachedSensor,GetName))
            .def("GetData",&PyRobotBase::PyAttachedSensor::GetData, DOXY_FN(RobotBase::AttachedSensor,GetData))
            .def("SetRelativeTransform",&PyRobotBase::PyAttachedSensor::SetRelativeTransform,args("transform"), DOXY_FN(RobotBase::AttachedSensor,SetRelativeTransform))
            .def("GetStructureHash",&PyRobotBase::PyAttachedSensor::GetStructureHash, DOXY_FN(RobotBase::AttachedSensor,GetStructureHash))
            .def("__str__",&PyRobotBase::PyAttachedSensor::__str__)
            .def("__repr__",&PyRobotBase::PyAttachedSensor::__repr__)
            .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
            .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
            ;

        class_<PyRobotBase::PyGrabbed, boost::shared_ptr<PyRobotBase::PyGrabbed> >("Grabbed", DOXY_CLASS(RobotBase::Grabbed),no_init)
            .def_readwrite("grabbedbody",&PyRobotBase::PyGrabbed::grabbedbody)
            .def_readwrite("linkrobot",&PyRobotBase::PyGrabbed::linkrobot)
            .def_readwrite("validColLinks",&PyRobotBase::PyGrabbed::validColLinks)
            .def_readwrite("troot",&PyRobotBase::PyGrabbed::troot)
            ;

        enum_<RobotBase::DOFAffine>("DOFAffine" DOXY_ENUM(DOFAffine))
            .value("NoTransform",RobotBase::DOF_NoTransform)
            .value("X",RobotBase::DOF_X)
            .value("Y",RobotBase::DOF_Y)
            .value("Z",RobotBase::DOF_Z)
            .value("RotationAxis",RobotBase::DOF_RotationAxis)
            .value("Rotation3D",RobotBase::DOF_Rotation3D)
            .value("RotationQuat",RobotBase::DOF_RotationQuat)
            ;
    }

    {
        bool (PyPlannerBase::*InitPlan1)(PyRobotBasePtr, boost::shared_ptr<PyPlannerBase::PyPlannerParameters>) = &PyPlannerBase::InitPlan;
        bool (PyPlannerBase::*InitPlan2)(PyRobotBasePtr, const string&) = &PyPlannerBase::InitPlan;
        scope planner = class_<PyPlannerBase, boost::shared_ptr<PyPlannerBase>, bases<PyInterfaceBase> >("Planner", DOXY_CLASS(PlannerBase), no_init)
                .def("InitPlan",InitPlan1,args("robot","params"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; PlannerParametersConstPtr"))
                .def("InitPlan",InitPlan2,args("robot","xmlparams"), DOXY_FN(PlannerBase,InitPlan "RobotBasePtr; std::istream"))
                .def("PlanPath",&PyPlannerBase::PlanPath,args("traj","output"), DOXY_FN(PlannerBase,PlanPath))
                .def("GetParameters",&PyPlannerBase::GetParameters, DOXY_FN(PlannerBase,GetParameters))
                ;
        
        class_<PyPlannerBase::PyPlannerParameters, boost::shared_ptr<PyPlannerBase::PyPlannerParameters> >("PlannerParameters", DOXY_CLASS(PlannerBase::PlannerParameters),no_init)
                .def("__str__",&PyPlannerBase::PyPlannerParameters::__str__)
                .def("__repr__",&PyPlannerBase::PyPlannerParameters::__repr__)
                .def("__eq__",&PyPlannerBase::PyPlannerParameters::__eq__)
                .def("__ne__",&PyPlannerBase::PyPlannerParameters::__ne__)
                ;
    }
    class_<PySensorSystemBase, boost::shared_ptr<PySensorSystemBase>, bases<PyInterfaceBase> >("SensorSystem", DOXY_CLASS(SensorSystemBase), no_init);
    class_<PyTrajectoryBase, boost::shared_ptr<PyTrajectoryBase>, bases<PyInterfaceBase> >("Trajectory", DOXY_CLASS(TrajectoryBase), no_init);
    class_<PyControllerBase, boost::shared_ptr<PyControllerBase>, bases<PyInterfaceBase> >("Controller", DOXY_CLASS(ControllerBase), no_init)
        .def("Init",&PyControllerBase::Init, DOXY_FN(ControllerBase,Init))
        .def("Reset",&PyControllerBase::Reset, DOXY_FN(ControllerBase,Reset))
        .def("SetDesired",&PyControllerBase::SetDesired, DOXY_FN(ControllerBase,SetDesired))
        .def("SetPath",&PyControllerBase::SetPath, DOXY_FN(ControllerBase,SetPath))
        .def("SimulationStep",&PyControllerBase::SimulationStep, DOXY_FN(ControllerBase,SimulationStep "dReal"))
        .def("IsDone",&PyControllerBase::IsDone, DOXY_FN(ControllerBase,IsDone))
        .def("GetTime",&PyControllerBase::GetTime, DOXY_FN(ControllerBase,GetTime))
        .def("GetVelocity",&PyControllerBase::GetVelocity, DOXY_FN(ControllerBase,GetVelocity))
        .def("GetTorque",&PyControllerBase::GetTorque, DOXY_FN(ControllerBase,GetTorque))
        ;
    class_<PyProblemInstance, boost::shared_ptr<PyProblemInstance>, bases<PyInterfaceBase> >("Problem", DOXY_CLASS(ProblemInstance), no_init)
        .def("SimulationStep",&PyProblemInstance::SimulationStep, DOXY_FN(ProblemInstance,"SimulationStep"))
        ;
    class_<PyIkSolverBase, boost::shared_ptr<PyIkSolverBase>, bases<PyInterfaceBase> >("IkSolver", DOXY_CLASS(IkSolverBase), no_init);

    bool (PyPhysicsEngineBase::*SetBodyVelocity1)(PyKinBodyPtr, object, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    bool (PyPhysicsEngineBase::*SetBodyVelocity2)(PyKinBodyPtr, object, object) = &PyPhysicsEngineBase::SetBodyVelocity;
    class_<PyPhysicsEngineBase, boost::shared_ptr<PyPhysicsEngineBase>, bases<PyInterfaceBase> >("PhysicsEngine", DOXY_CLASS(PhysicsEngineBase), no_init)
        .def("GetPhysicsOptions",&PyPhysicsEngineBase::GetPhysicsOptions, DOXY_FN(PhysicsEngineBase,GetPhysicsOptions))
        .def("SetPhysicsOptions",&PyPhysicsEngineBase::SetPhysicsOptions, DOXY_FN(PhysicsEngineBase,SetPhysicsOptions "int"))
        .def("InitEnvironment",&PyPhysicsEngineBase::InitEnvironment, DOXY_FN(PhysicsEngineBase,InitEnvironment))
        .def("DestroyEnvironment",&PyPhysicsEngineBase::DestroyEnvironment, DOXY_FN(PhysicsEngineBase,DestroyEnvironment))
        .def("InitKinBody",&PyPhysicsEngineBase::InitKinBody, DOXY_FN(PhysicsEngineBase,InitKinBody))
        .def("SetBodyVelocity",SetBodyVelocity1, args("body","linearvel","angularvel","jointvel"), DOXY_FN(PhysicsEngineBase,SetBodyVelocity "KinBodyPtr; const Vector; const Vector; const std::vector"))
        .def("SetBodyVelocity",SetBodyVelocity2, args("body","linklinearvels","linkangularvels"), DOXY_FN(PhysicsEngineBase,SetBodyVelocity "KinBodyPtr; const std::vector; const std::vector"))
        .def("SetLinkVelocity",&PyPhysicsEngineBase::SetLinkVelocity, DOXY_FN(PhysicsEngineBase,SetLinkVelocity))
        .def("GetLinkVelocity",&PyPhysicsEngineBase::GetLinkVelocity, DOXY_FN(PhysicsEngineBase,GetLinkVelocity))
        .def("GetBodyVelocityJoints",&PyPhysicsEngineBase::GetBodyVelocityJoints, DOXY_FN(PhysicsEngineBase,GetBodyVelocityJoints "KinBodyConstPtr; Vector, Vector, std::vector"))
        .def("GetBodyVelocityLinks",&PyPhysicsEngineBase::GetBodyVelocityLinks, DOXY_FN(PhysicsEngineBase,GetBodyVelocityLinks "KinBodyConstPtr; std::vector, std::vector"))
        .def("SetJointVelocity",&PyPhysicsEngineBase::SetJointVelocity, DOXY_FN(PhysicsEngineBase,SetJointVelocity))
        .def("GetJointVelocity",&PyPhysicsEngineBase::GetJointVelocity, DOXY_FN(PhysicsEngineBase,GetJointVelocity))
        .def("SetBodyForce",&PyPhysicsEngineBase::SetBodyForce, DOXY_FN(PhysicsEngineBase,SetBodyForce))
        .def("SetBodyTorque",&PyPhysicsEngineBase::SetBodyTorque, DOXY_FN(PhysicsEngineBase,SetBodyTorque))
        .def("AddJointTorque",&PyPhysicsEngineBase::AddJointTorque, DOXY_FN(PhysicsEngineBase,AddJointTorque))
        .def("SetGravity",&PyPhysicsEngineBase::SetGravity, DOXY_FN(PhysicsEngineBase,SetGravity))
        .def("GetGravity",&PyPhysicsEngineBase::GetGravity, DOXY_FN(PhysicsEngineBase,GetGravity))
        .def("SimulateStep",&PyPhysicsEngineBase::SimulateStep, DOXY_FN(PhysicsEngineBase,SimulateStep))
        ;
    {
        scope sensor = class_<PySensorBase, boost::shared_ptr<PySensorBase>, bases<PyInterfaceBase> >("Sensor", DOXY_CLASS(SensorBase), no_init)
            .def("GetSensorData",&PySensorBase::GetSensorData, DOXY_FN(SensorBase,GetSensorData))
            .def("SetTransform",&PySensorBase::SetTransform, DOXY_FN(SensorBase,SetTransform))
            .def("GetTransform",&PySensorBase::GetTransform, DOXY_FN(SensorBase,GetTransform))
            .def("GetName",&PySensorBase::GetName, DOXY_FN(SensorBase,GetName))
            .def("__str__",&PySensorBase::__str__)
            .def("__repr__",&PySensorBase::__repr__)
            ;

        class_<PySensorBase::PySensorData, boost::shared_ptr<PySensorBase::PySensorData> >("SensorData", DOXY_CLASS(SensorBase::SensorData),no_init)
            .def_readonly("type",&PySensorBase::PySensorData::type)
            .def_readonly("stamp",&PySensorBase::PySensorData::stamp)
            ;
        class_<PySensorBase::PyLaserSensorData, boost::shared_ptr<PySensorBase::PyLaserSensorData>, bases<PySensorBase::PySensorData> >("LaserSensorData", DOXY_CLASS(SensorBase::LaserSensorData),no_init)
            .def_readonly("transform",&PySensorBase::PyLaserSensorData::transform)
            .def_readonly("positions",&PySensorBase::PyLaserSensorData::positions)
            .def_readonly("ranges",&PySensorBase::PyLaserSensorData::ranges)
            .def_readonly("intensity",&PySensorBase::PyLaserSensorData::intensity)
            ;
        class_<PySensorBase::PyCameraSensorData, boost::shared_ptr<PySensorBase::PyCameraSensorData>, bases<PySensorBase::PySensorData> >("CameraSensorData", DOXY_CLASS(SensorBase::CameraSensorData),no_init)
            .def_readonly("transform",&PySensorBase::PyCameraSensorData::transform)
            .def_readonly("imagedata",&PySensorBase::PyCameraSensorData::imagedata)
            .def_readonly("KK",&PySensorBase::PyCameraSensorData::KK)
            ;

        enum_<SensorBase::SensorType>("Type" DOXY_ENUM(SensorType))
            .value("Invalid",SensorBase::ST_Invalid)
            .value("Laser",SensorBase::ST_Laser)
            .value("Camera",SensorBase::ST_Camera)
            .value("JointEncoder",SensorBase::ST_JointEncoder)
            .value("Force6D",SensorBase::ST_Force6D)
            ;
    }

    class_<PyCollisionCheckerBase, boost::shared_ptr<PyCollisionCheckerBase>, bases<PyInterfaceBase> >("CollisionChecker", DOXY_CLASS(CollisionCheckerBase), no_init)
        .def("SetCollisionOptions",&PyCollisionCheckerBase::SetCollisionOptions, DOXY_FN(CollisionCheckerBase,SetCollisionOptions "int"))
        .def("GetCollisionOptions",&PyCollisionCheckerBase::GetCollisionOptions, DOXY_FN(CollisionCheckerBase,GetCollisionOptions))
        ;


    {
        void (PyViewerBase::*setcamera1)(object) = &PyViewerBase::SetCamera;
        void (PyViewerBase::*setcamera2)(object,float) = &PyViewerBase::SetCamera;
        scope viewer = class_<PyViewerBase, boost::shared_ptr<PyViewerBase>, bases<PyInterfaceBase> >("Viewer", DOXY_CLASS(ViewerBase), no_init)
            .def("main",&PyViewerBase::main, DOXY_FN(ViewerBase,main))
            .def("quitmainloop",&PyViewerBase::quitmainloop, DOXY_FN(ViewerBase,quitmainloop))
            .def("SetSize",&PyViewerBase::ViewerSetSize, DOXY_FN(ViewerBase,ViewerSetSize))
            .def("Move",&PyViewerBase::ViewerMove, DOXY_FN(ViewerBase,ViewerMove))
            .def("SetTitle",&PyViewerBase::ViewerSetTitle, DOXY_FN(ViewerBase,ViewerSetTitle))
            .def("LoadModel",&PyViewerBase::LoadModel, DOXY_FN(ViewerBase,LoadModel))
            .def("RegisterCallback",&PyViewerBase::RegisterCallback, DOXY_FN(ViewerBase,RegisterCallback))
            .def("EnvironmentSync",&PyViewerBase::EnvironmentSync, DOXY_FN(ViewerBase,EnvironmentSync))
            .def("SetCamera",setcamera1,args("transform"), DOXY_FN(ViewerBase,SetCamera))
            .def("SetCamera",setcamera2,args("transform","focalDistance"), DOXY_FN(ViewerBase,SetCamera))
            .def("GetCameraTransform",&PyViewerBase::GetCameraTransform, DOXY_FN(ViewerBase,GetCameraTransform))
            .def("GetCameraImage",&PyViewerBase::GetCameraImage,args("width","height","transform","K"), DOXY_FN(ViewerBase,GetCameraImage))
            ;

        enum_<ViewerBase::ViewerEvents>("Events" DOXY_ENUM(ViewerEvents))
            .value("ItemSelection",ViewerBase::VE_ItemSelection)
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
    bool (PyEnvironmentBase::*pcolyb)(boost::shared_ptr<PyRay>,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolybr)(boost::shared_ptr<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcoly)(boost::shared_ptr<PyRay>) = &PyEnvironmentBase::CheckCollision;
    bool (PyEnvironmentBase::*pcolyr)(boost::shared_ptr<PyRay>, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;

    void (PyEnvironmentBase::*LockPhysics1)(bool) = &PyEnvironmentBase::LockPhysics;
    void (PyEnvironmentBase::*LockPhysics2)(bool, float) = &PyEnvironmentBase::LockPhysics;

    object (PyEnvironmentBase::*drawplane1)(object, object, const boost::multi_array<float,2>&) = &PyEnvironmentBase::drawplane;
    object (PyEnvironmentBase::*drawplane2)(object, object, const boost::multi_array<float,3>&) = &PyEnvironmentBase::drawplane;

    bool (PyEnvironmentBase::*addkinbody1)(PyKinBodyPtr) = &PyEnvironmentBase::AddKinBody;
    bool (PyEnvironmentBase::*addkinbody2)(PyKinBodyPtr,bool) = &PyEnvironmentBase::AddKinBody;
    bool (PyEnvironmentBase::*addrobot1)(PyRobotBasePtr) = &PyEnvironmentBase::AddRobot;
    bool (PyEnvironmentBase::*addrobot2)(PyRobotBasePtr,bool) = &PyEnvironmentBase::AddRobot;
    bool (PyEnvironmentBase::*addsensor1)(PySensorBasePtr, const std::string&) = &PyEnvironmentBase::AddSensor;
    bool (PyEnvironmentBase::*addsensor2)(PySensorBasePtr,const std::string&,bool) = &PyEnvironmentBase::AddSensor;
    {
        scope env = classenv
            .def(init<>())
            .def("Reset",&PyEnvironmentBase::Reset, DOXY_FN(EnvironmentBase,Reset))
            .def("Destroy",&PyEnvironmentBase::Destroy, DOXY_FN(EnvironmentBase,Destroy))
            .def("GetPluginInfo",&PyEnvironmentBase::GetPluginInfo, DOXY_FN(EnvironmentBase,GetPluginInfo))
            .def("GetLoadedInterfaces",&PyEnvironmentBase::GetLoadedInterfaces, DOXY_FN(EnvironmentBase,GetLoadedInterfaces))
            .def("LoadPlugin",&PyEnvironmentBase::LoadPlugin,args("filename"), DOXY_FN(EnvironmentBase,LoadPlugin))
            .def("ReloadPlugins",&PyEnvironmentBase::ReloadPlugins, DOXY_FN(EnvironmentBase,ReloadPlugins))
            .def("CreateInterface", &PyEnvironmentBase::CreateInterface,args("type","name"), DOXY_FN(EnvironmentBase,ReloadPlugins))
            .def("CreateRobot", &PyEnvironmentBase::CreateRobot,args("name"), DOXY_FN(EnvironmentBase,CreateRobot))
            .def("CreatePlanner", &PyEnvironmentBase::CreatePlanner,args("name"), DOXY_FN(EnvironmentBase,CreatePlanner))
            .def("CreateSensorSystem", &PyEnvironmentBase::CreateSensorSystem,args("name"), DOXY_FN(EnvironmentBase,CreateSensorSystem))
            .def("CreateController", &PyEnvironmentBase::CreateController,args("name"), DOXY_FN(EnvironmentBase,CreateController))
            .def("CreateProblem", &PyEnvironmentBase::CreateProblem,args("name"), DOXY_FN(EnvironmentBase,CreateProblem))
            .def("CreateIkSolver", &PyEnvironmentBase::CreateIkSolver,args("name"), DOXY_FN(EnvironmentBase,CreateIkSolver))
            .def("CreatePhysicsEngine", &PyEnvironmentBase::CreatePhysicsEngine,args("name"), DOXY_FN(EnvironmentBase,CreateIkSolver))
            .def("CreateSensor", &PyEnvironmentBase::CreateSensor,args("name"), DOXY_FN(EnvironmentBase,CreateIkSolver))
            .def("CreateCollisionChecker", &PyEnvironmentBase::CreateCollisionChecker,args("name"), DOXY_FN(EnvironmentBase,CreateIkSolver))
            .def("CreateViewer", &PyEnvironmentBase::CreateViewer,args("name"), DOXY_FN(EnvironmentBase,CreateViewer))
            
            .def("CloneSelf",&PyEnvironmentBase::CloneSelf,args("options"), DOXY_FN(EnvironmentBase,CloneSelf))
            .def("SetCollisionChecker",&PyEnvironmentBase::SetCollisionChecker,args("collisionchecker"), DOXY_FN(EnvironmentBase,SetCollisionChecker))
            .def("GetCollisionChecker",&PyEnvironmentBase::GetCollisionChecker, DOXY_FN(EnvironmentBase,GetCollisionChecker))

            .def("CheckCollision",pcolb,args("body"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolbr,args("body","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolbb,args("body1","body2"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolbbr,args("body1","body2","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcoll,args("link"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcollr,args("link","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolll,args("link1","link2"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolllr,args("link1","link2","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcollb,args("link","body"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcollbr,args("link","body","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolle,args("link","bodyexcluded","linkexcluded"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
            .def("CheckCollision",pcoller,args("link","bodyexcluded","linkexcluded","report") , DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
            .def("CheckCollision",pcolbe,args("body","bodyexcluded","linkexcluded"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
            .def("CheckCollision",pcolber,args("body","bodyexcluded","linkexcluded","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
            .def("CheckCollision",pcolyb,args("ray","body"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcolybr,args("ray","body","report"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
            .def("CheckCollision",pcoly,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
            .def("CheckCollision",pcolyr,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
            .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                 CheckCollisionRays_overloads(args("rays","body","front_facing_only"), 
                                              "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction+range."))
            .def("Load",&PyEnvironmentBase::Load,args("filename"), DOXY_FN(EnvironmentBase,Load))
            .def("Save",&PyEnvironmentBase::Save,args("filename"), DOXY_FN(EnvironmentBase,Save))
            .def("ReadRobotXMLFile",&PyEnvironmentBase::ReadRobotXMLFile,args("filename"), DOXY_FN(EnvironmentBase,ReadRobotXMLFile "const std::string"))
            .def("ReadRobotXMLData",&PyEnvironmentBase::ReadRobotXMLData,args("data"), DOXY_FN(EnvironmentBase,ReadRobotXMLData "RobotBasePtr; const std::string; const std::list"))
            .def("ReadKinBodyXMLFile",&PyEnvironmentBase::ReadKinBodyXMLFile,args("filename"), DOXY_FN(EnvironmentBase,ReadKinBodyXMLFile "const std::string"))
            .def("ReadKinBodyXMLData",&PyEnvironmentBase::ReadKinBodyXMLData,args("data"), DOXY_FN(EnvironmentBase,ReadKinBodyXMLData "KinBodyPtr; const std::string; const std::list"))
            .def("ReadInterfaceXMLFile",&PyEnvironmentBase::ReadInterfaceXMLFile,args("filename"), DOXY_FN(EnvironmentBase,ReadInterfaceXMLFile "const std::string"))
            .def("AddKinBody",addkinbody1,args("body"), DOXY_FN(EnvironmentBase,AddKinBody))
            .def("AddKinBody",addkinbody2,args("body","anonymous"), DOXY_FN(EnvironmentBase,AddKinBody))
            .def("AddRobot",addrobot1,args("robot"), DOXY_FN(EnvironmentBase,AddRobot))
            .def("AddRobot",addrobot2,args("robot","anonymous"), DOXY_FN(EnvironmentBase,AddRobot))
            .def("AddSensor",addsensor1,args("sensor","args"), DOXY_FN(EnvironmentBase,AddSensor))
            .def("AddSensor",addsensor2,args("sensor","args","anonymous"), DOXY_FN(EnvironmentBase,AddSensor))
            .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody,args("body"), DOXY_FN(EnvironmentBase,RemoveKinBody))
            .def("Remove",&PyEnvironmentBase::Remove,args("interface"), DOXY_FN(EnvironmentBase,Remove))
            .def("GetKinBody",&PyEnvironmentBase::GetKinBody,args("name"), DOXY_FN(EnvironmentBase,GetKinBody))
            .def("GetRobot",&PyEnvironmentBase::GetRobot,args("name"), DOXY_FN(EnvironmentBase,GetRobot))
            .def("GetSensor",&PyEnvironmentBase::GetSensor,args("name"), DOXY_FN(EnvironmentBase,GetSensor))
            .def("GetBodyFromEnvironmentId",&PyEnvironmentBase::GetBodyFromEnvironmentId , DOXY_FN(EnvironmentBase,GetBodyFromEnvironmentId))
            .def("CreateKinBody",&PyEnvironmentBase::CreateKinBody, DOXY_FN(EnvironmentBase,CreateKinBody))
            .def("CreateTrajectory",&PyEnvironmentBase::CreateTrajectory,args("dof"), DOXY_FN(EnvironmentBase,CreateTrajectory))
            .def("LoadProblem",&PyEnvironmentBase::LoadProblem,args("problem","args"), DOXY_FN(EnvironmentBase,LoadProblem))
            .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem,args("prob"), DOXY_FN(EnvironmentBase,RemoveProblem))
            .def("GetLoadedProblems",&PyEnvironmentBase::GetLoadedProblems, DOXY_FN(EnvironmentBase,GetLoadedProblems))
            .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine,args("physics"), DOXY_FN(EnvironmentBase,SetPhysicsEngine))
            .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine, DOXY_FN(EnvironmentBase,GetPhysicsEngine))
            .def("RegisterCollisionCallback",&PyEnvironmentBase::RegisterCollisionCallback,args("callback"), DOXY_FN(EnvironmentBase,RegisterCollisionCallback))
            .def("StepSimulation",&PyEnvironmentBase::StepSimulation,args("timestep"), DOXY_FN(EnvironmentBase,StepSimulation))
            .def("StartSimulation",&PyEnvironmentBase::StartSimulation,StartSimulation_overloads(args("timestep","realtime"), DOXY_FN(EnvironmentBase,StartSimulation)))
            .def("StopSimulation",&PyEnvironmentBase::StopSimulation, DOXY_FN(EnvironmentBase,StopSimulation))
            .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime, DOXY_FN(EnvironmentBase,GetSimulationTime))
            .def("LockPhysics",LockPhysics1,args("lock"), "Locks the environment mutex.")
            .def("LockPhysics",LockPhysics2,args("lock","timeout"), "Locks the environment mutex with a timeout.")
            .def("SetViewer",&PyEnvironmentBase::SetViewer,SetViewer_overloads(args("viewername","showviewer"), "Attaches the viewer and starts its thread"))
            .def("GetViewer",&PyEnvironmentBase::GetViewer, DOXY_FN(EnvironmentBase,GetViewer))
            .def("plot3",&PyEnvironmentBase::plot3,plot3_overloads(args("points","pointsize","colors","drawstyle"), DOXY_FN(EnvironmentBase,plot3 "const float; int; int; float; const float; int, bool")))
            .def("drawlinestrip",&PyEnvironmentBase::drawlinestrip,drawlinestrip_overloads(args("points","linewidth","colors","drawstyle"), DOXY_FN(EnvironmentBase,drawlinestrip "const float; int; int; float; const float")))
            .def("drawlinelist",&PyEnvironmentBase::drawlinelist,drawlinelist_overloads(args("points","linewidth","colors","drawstyle"), DOXY_FN(EnvironmentBase,drawlinelist "const float; int; int; float; const float")))
            .def("drawarrow",&PyEnvironmentBase::drawarrow,drawarrow_overloads(args("p1","p2","linewidth","color"), DOXY_FN(EnvironmentBase,drawarrow)))
            .def("drawbox",&PyEnvironmentBase::drawbox,drawbox_overloads(args("pos","extents","color"), DOXY_FN(EnvironmentBase,drawbox)))
            .def("drawplane",drawplane1,args("transform","extents","texture"), DOXY_FN(EnvironmentBase,drawplane))
            .def("drawplane",drawplane2,args("transform","extents","texture"), DOXY_FN(EnvironmentBase,drawplane))
            .def("drawtrimesh",&PyEnvironmentBase::drawtrimesh,drawtrimesh_overloads(args("points","indices","colors"), DOXY_FN(EnvironmentBase,drawtrimesh "const float; int; const int; int; const boost::multi_array")))
            .def("GetRobots",&PyEnvironmentBase::GetRobots, DOXY_FN(EnvironmentBase,GetRobots))
            .def("GetBodies",&PyEnvironmentBase::GetBodies, DOXY_FN(EnvironmentBase,GetBodies))
            .def("GetSensors",&PyEnvironmentBase::GetSensors, DOXY_FN(EnvironmentBase,GetSensors))
            .def("UpdatePublishedBodies",&PyEnvironmentBase::UpdatePublishedBodies, DOXY_FN(EnvironmentBase,UpdatePublishedBodies))
            .def("Triangulate",&PyEnvironmentBase::Triangulate,args("body"), DOXY_FN(EnvironmentBase,Triangulate))
            .def("TriangulateScene",&PyEnvironmentBase::TriangulateScene,args("options","name"), DOXY_FN(EnvironmentBase,TriangulateScene))
            .def("SetDebugLevel",&PyEnvironmentBase::SetDebugLevel,args("level"), DOXY_FN(EnvironmentBase,SetDebugLevel))
            .def("GetDebugLevel",&PyEnvironmentBase::GetDebugLevel, DOXY_FN(EnvironmentBase,GetDebugLevel))
            .def("GetHomeDirectory",&PyEnvironmentBase::GetHomeDirectory, DOXY_FN(EnvironmentBase,GetHomeDirectory))
            .def("__enter__",&PyEnvironmentBase::__enter__)
            .def("__exit__",&PyEnvironmentBase::__exit__)
            .def("__eq__",&PyEnvironmentBase::__eq__)
            .def("__ne__",&PyEnvironmentBase::__ne__)
            ;

        enum_<EnvironmentBase::TriangulateOptions>("TriangulateOptions" DOXY_ENUM(TriangulateOptions))
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

    scope().attr("__version__") = OPENRAVE_VERSION_STRING;
    scope().attr("__author__") = "Rosen Diankov";
    scope().attr("__copyright__") = "2009-2010 Rosen Diankov (rosen.diankov@gmail.com)";
    scope().attr("__license__") = "Lesser GPL";
    scope().attr("__docformat__") = "restructuredtext";

    openravepy::init_openravepy_global();

    def("RaveCreateInterface",openravepy::RaveCreateInterface,args("env","type","name"),DOXY_FN1(RaveCreateInterface));
    def("RaveCreateRobot",openravepy::RaveCreateRobot,args("env","name"),DOXY_FN1(RaveCreateRobot));
    def("RaveCreatePlanner",openravepy::RaveCreatePlanner,args("env","name"),DOXY_FN1(RaveCreatePlanner));
    def("RaveCreateSensorSystem",openravepy::RaveCreateSensorSystem,args("env","name"),DOXY_FN1(RaveCreateSensorSystem));
    def("RaveCreateController",openravepy::RaveCreateController,args("env","name"),DOXY_FN1(RaveCreateController));
    def("RaveCreateProblem",openravepy::RaveCreateProblem,args("env","name"),DOXY_FN1(RaveCreateProblem));
    def("RaveCreateIkSolver",openravepy::RaveCreateIkSolver,args("env","name"),DOXY_FN1(RaveCreateIkSolver));
    def("RaveCreatePhysicsEngine",openravepy::RaveCreatePhysicsEngine,args("env","name"),DOXY_FN1(RaveCreatePhysicsEngine));
    def("RaveCreateSensor",openravepy::RaveCreateSensor,args("env","name"),DOXY_FN1(RaveCreateSensor));
    def("RaveCreateCollisionChecker",openravepy::RaveCreateCollisionChecker,args("env","name"),DOXY_FN1(RaveCreateCollisionChecker));
    def("RaveCreateViewer",openravepy::RaveCreateViewer,args("env","name"),DOXY_FN1(RaveCreateViewer));
    def("RaveCreateKinBody",openravepy::RaveCreateKinBody,args("env","name"),DOXY_FN1(RaveCreateKinBody));
    def("RaveCreateTrajectory",openravepy::RaveCreateTrajectory,args("env","name"),DOXY_FN1(RaveCreateTrajectory));

}
