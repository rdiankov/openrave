// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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

#include <openrave/utils.h>

namespace openravepy {

class PyStateRestoreContextBase
{
public:
    virtual ~PyStateRestoreContextBase() {
    }
    virtual object __enter__() = 0;
    virtual void __exit__(object type, object value, object traceback) = 0;
    virtual object GetBody() const = 0;
    virtual void Restore(object p=object())  = 0;
    virtual void Release() = 0;
    virtual void Close() = 0;
    virtual std::string __str__() = 0;
    virtual object __unicode__() = 0;
};

/// \brief simple wrapper around a save state that manages  enter/exit scope
template <typename T, typename U>
class PyStateRestoreContext : public PyStateRestoreContextBase
{
    T _state;
public:
    PyStateRestoreContext(T state) : _state(state) {
    }
    virtual ~PyStateRestoreContext() {
    }
    object __enter__() {
        return object(_state);
    }
    void __exit__(object type, object value, object traceback) {
        _state->Restore();
    }

    object GetBody() const {
        return _state->GetBody();
    }

    void Restore(object p=object()) {
        if( p == object() ) {
            _state->Restore();
        }
        else {
            U pytarget = boost::python::extract<U>(p);
            _state->Restore(pytarget);
        }
    }

    void Release() {
        _state->Release();
    }

    void Close() {
        _state.reset();
    }

    std::string __str__() {
        return _state->__str__();
    }
    object __unicode__() {
        return _state->__unicode__();
    }
};


class PyKinBody : public PyInterfaceBase
{
protected:
    KinBodyPtr _pbody;
    std::list<boost::shared_ptr<void> > _listStateSavers;

public:
    class PyGeometryInfo
    {
public:
        PyGeometryInfo() {
            _t = ReturnTransform(Transform());
            _vGeomData = toPyVector4(Vector());
            _vDiffuseColor = toPyVector3(Vector(1,1,1));
            _vAmbientColor = toPyVector3(Vector(0,0,0));
            _type = GT_None;
            _fTransparency = 0;
            _vRenderScale = toPyVector3(Vector(1,1,1));
            _vCollisionScale = toPyVector3(Vector(1,1,1));
            _bVisible = true;
            _bModifiable = true;
        }
        KinBody::GeometryInfoPtr GetGeometryInfo() {
            KinBody::GeometryInfoPtr pinfo(new KinBody::GeometryInfo());
            KinBody::GeometryInfo& info = *pinfo;
            info._t = ExtractTransform(_t);
            info._vGeomData = ExtractVector<dReal>(_vGeomData);
            info._vDiffuseColor = ExtractVector34<dReal>(_vDiffuseColor,0);
            info._vAmbientColor = ExtractVector34<dReal>(_vAmbientColor,0);
            if( _meshcollision != object() ) {
                ExtractTriMesh(_meshcollision,info._meshcollision);
            }
            info._type = _type;
            info._filenamerender = _filenamerender;
            info._filenamecollision = _filenamecollision;
            info._vRenderScale = ExtractVector3(_vRenderScale);
            info._vCollisionScale = ExtractVector3(_vCollisionScale);
            info._fTransparency = _fTransparency;
            info._bVisible = _bVisible;
            info._bModifiable = _bModifiable;
            return pinfo;
        }

        object _t, _vGeomData, _vDiffuseColor, _vAmbientColor, _meshcollision;
        GeometryType _type;
        std::string _filenamerender, _filenamecollision;
        object _vRenderScale, _vCollisionScale;
        float _fTransparency;
        bool _bVisible, _bModifiable;
    };
    typedef boost::shared_ptr<PyGeometryInfo> PyGeometryInfoPtr;

    class PyLinkInfo
    {
public:
        PyLinkInfo() {
            _t = ReturnTransform(Transform());
            _tMassFrame = ReturnTransform(Transform());
            _mass = 0;
            _vinertiamoments = toPyVector3(Vector(1,1,1));
            _bStatic = false;
            _bIsEnabled = true;
            _vForcedAdjacentLinks = boost::python::list();
        }
        KinBody::LinkInfoPtr GetLinkInfo() {
            KinBody::LinkInfoPtr pinfo(new KinBody::LinkInfo());
            KinBody::LinkInfo& info = *pinfo;
            info._vgeometryinfos.resize(len(_vgeometryinfos));
            for(size_t i = 0; i < info._vgeometryinfos.size(); ++i) {
                PyGeometryInfoPtr pygeom = boost::python::extract<PyGeometryInfoPtr>(_vgeometryinfos[i]);
                info._vgeometryinfos[i] = pygeom->GetGeometryInfo();
            }
            info._name = _name;
            info._t = ExtractTransform(_t);
            info._tMassFrame = ExtractTransform(_tMassFrame);
            info._mass = _mass;
            info._vinertiamoments = ExtractVector3(_vinertiamoments);
            size_t num = len(_mapFloatParameters);
            object okeyvalueiter = _mapFloatParameters.iteritems();
            info._mapFloatParameters.clear();
            for(size_t i = 0; i < num; ++i) {
                object okeyvalue = okeyvalueiter.attr("next") ();
                std::string name = extract<std::string>(okeyvalue[0]);
                info._mapFloatParameters[name] = ExtractArray<dReal>(okeyvalue[1]);
            }
            okeyvalueiter = _mapIntParameters.iteritems();
            num = len(_mapIntParameters);
            info._mapIntParameters.clear();
            for(size_t i = 0; i < num; ++i) {
                object okeyvalue = okeyvalueiter.attr("next") ();
                std::string name = extract<std::string>(okeyvalue[0]);
                info._mapIntParameters[name] = ExtractArray<int>(okeyvalue[1]);
            }
            info._vForcedAdjacentLinks = ExtractArray<std::string>(_vForcedAdjacentLinks);
            info._bStatic = _bStatic;
            info._bIsEnabled = _bIsEnabled;
            return pinfo;
        }

        boost::python::list _vgeometryinfos;
        std::string _name;
        object _t, _tMassFrame;
        dReal _mass;
        object _vinertiamoments;
        boost::python::dict _mapFloatParameters, _mapIntParameters;
        object _vForcedAdjacentLinks;
        bool _bStatic;
        bool _bIsEnabled;
    };
    typedef boost::shared_ptr<PyLinkInfo> PyLinkInfoPtr;


    class PyJointInfo
    {
public:
        PyJointInfo() {
            _type = KinBody::JointNone;
            _vanchor = toPyVector3(Vector());
            _vresolution = toPyVector3(Vector(0.02,0.02,0.02));
            _vmaxvel = toPyVector3(Vector(10,10,10));
            _vhardmaxvel = toPyVector3(Vector(10,10,10));
            _vmaxaccel = toPyVector3(Vector(50,50,50));
            _vmaxtorque = toPyVector3(Vector(1e5,1e5,1e5));
            _vweights = toPyVector3(Vector(1,1,1));
            _voffsets = toPyVector3(Vector(0,0,0));
            _vlowerlimit = toPyVector3(Vector(0,0,0));
            _vupperlimit = toPyVector3(Vector(0,0,0));
            _bIsCircular = toPyVector3(Vector(0,0,0));
        }
        KinBody::JointInfoPtr GetJointInfo() {
            KinBody::JointInfoPtr pinfo(new KinBody::JointInfo());
            KinBody::JointInfo& info = *pinfo;
            info._type = _type;
            info._name = _name;
            info._linkname0 = _linkname0;
            info._linkname1 = _linkname1;
            info._vanchor = ExtractVector3(_vanchor);
            size_t num = len(_vaxes);
            for(size_t i = 0; i < num; ++i) {
                info._vaxes.at(i) = ExtractVector3(_vaxes[i]);
            }
            if( _vcurrentvalues != object() ) {
                info._vcurrentvalues = ExtractArray<dReal>(_vcurrentvalues);
            }
            num = len(_vresolution);
            for(size_t i = 0; i < num; ++i) {
                info._vresolution.at(i) = boost::python::extract<dReal>(_vresolution[i]);
            }
            num = len(_vmaxvel);
            for(size_t i = 0; i < num; ++i) {
                info._vmaxvel.at(i) = boost::python::extract<dReal>(_vmaxvel[i]);
            }
            num = len(_vhardmaxvel);
            for(size_t i = 0; i < num; ++i) {
                info._vhardmaxvel.at(i) = boost::python::extract<dReal>(_vhardmaxvel[i]);
            }
            num = len(_vmaxaccel);
            for(size_t i = 0; i < num; ++i) {
                info._vmaxaccel.at(i) = boost::python::extract<dReal>(_vmaxaccel[i]);
            }
            num = len(_vmaxtorque);
            for(size_t i = 0; i < num; ++i) {
                info._vmaxtorque.at(i) = boost::python::extract<dReal>(_vmaxtorque[i]);
            }
            num = len(_vweights);
            for(size_t i = 0; i < num; ++i) {
                info._vweights.at(i) = boost::python::extract<dReal>(_vweights[i]);
            }
            num = len(_voffsets);
            for(size_t i = 0; i < num; ++i) {
                info._voffsets.at(i) = boost::python::extract<dReal>(_voffsets[i]);
            }
            num = len(_vlowerlimit);
            for(size_t i = 0; i < num; ++i) {
                info._vlowerlimit.at(i) = boost::python::extract<dReal>(_vlowerlimit[i]);
            }
            num = len(_vupperlimit);
            for(size_t i = 0; i < num; ++i) {
                info._vupperlimit.at(i) = boost::python::extract<dReal>(_vupperlimit[i]);
            }
            if( _trajfollow != object() ) {
                info._trajfollow = GetTrajectory(_trajfollow);
            }
            if( _vmimic != object() ) {
                num = len(_vmimic);
                for(size_t i = 0; i < num; ++i) {
                    if( _vmimic[i] != object() ) {
                        OPENRAVE_ASSERT_OP(len(_vmimic[i]),==,3);
                        info._vmimic[i].reset(new KinBody::MimicInfo());
                        for(size_t j = 0; j < 3; ++j) {
                            info._vmimic[i]->_equations.at(j) = boost::python::extract<std::string>(_vmimic[i][j]);
                        }
                    }
                }
            }
            num = len(_mapFloatParameters);
            object okeyvalueiter = _mapFloatParameters.iteritems();
            info._mapFloatParameters.clear();
            for(size_t i = 0; i < num; ++i) {
                object okeyvalue = okeyvalueiter.attr("next") ();
                std::string name = extract<std::string>(okeyvalue[0]);
                info._mapFloatParameters[name] = ExtractArray<dReal>(okeyvalue[1]);
            }
            okeyvalueiter = _mapIntParameters.iteritems();
            num = len(_mapIntParameters);
            info._mapIntParameters.clear();
            for(size_t i = 0; i < num; ++i) {
                object okeyvalue = okeyvalueiter.attr("next") ();
                std::string name = extract<std::string>(okeyvalue[0]);
                info._mapIntParameters[name] = ExtractArray<int>(okeyvalue[1]);
            }
            num = len(_bIsCircular);
            for(size_t i = 0; i < num; ++i) {
                info._bIsCircular.at(i) = boost::python::extract<int>(_bIsCircular[i])!=0;
            }
            return pinfo;
        }
        KinBody::JointType _type;
        std::string _name;
        std::string _linkname0, _linkname1;
        object _vanchor, _vaxes, _vcurrentvalues, _vresolution, _vmaxvel, _vhardmaxvel, _vmaxaccel, _vmaxtorque, _vweights, _voffsets, _vlowerlimit, _vupperlimit, _trajfollow;
        boost::python::list _vmimic;
        boost::python::dict _mapFloatParameters, _mapIntParameters;
        object _bIsCircular;
    };
    typedef boost::shared_ptr<PyJointInfo> PyJointInfoPtr;

    class PyLink
    {
        KinBody::LinkPtr _plink;
        PyEnvironmentBasePtr _pyenv;
public:
        class PyGeometry
        {
            KinBody::Link::GeometryPtr _pgeometry;
public:
            PyGeometry(KinBody::Link::GeometryPtr pgeometry) : _pgeometry(pgeometry) {
            }

            virtual void SetCollisionMesh(object pytrimesh) {
                TriMesh mesh;
                if( ExtractTriMesh(pytrimesh,mesh) ) {
                    _pgeometry->SetCollisionMesh(mesh);
                }
                else {
                    throw openrave_exception("bad trimesh");
                }
            }

            object GetCollisionMesh() {
                return toPyTriMesh(_pgeometry->GetCollisionMesh());
            }
            object ComputeAABB(object otransform) const {
                return toPyAABB(_pgeometry->ComputeAABB(ExtractTransform(otransform)));
            }
            void SetDraw(bool bDraw) {
                _pgeometry->SetVisible(bDraw);
            }
            bool SetVisible(bool visible) {
                return _pgeometry->SetVisible(visible);
            }
            void SetTransparency(float f) {
                _pgeometry->SetTransparency(f);
            }
            void SetAmbientColor(object ocolor) {
                _pgeometry->SetAmbientColor(ExtractVector3(ocolor));
            }
            void SetDiffuseColor(object ocolor) {
                _pgeometry->SetDiffuseColor(ExtractVector3(ocolor));
            }
            void SetRenderFilename(const string& filename) {
                _pgeometry->SetRenderFilename(filename);
            }
            bool IsDraw() {
                RAVELOG_WARN("IsDraw deprecated, use Geometry.IsVisible\n");
                return _pgeometry->IsVisible();
            }
            bool IsVisible() {
                return _pgeometry->IsVisible();
            }
            bool IsModifiable() {
                return _pgeometry->IsModifiable();
            }
            GeometryType GetType() {
                return _pgeometry->GetType();
            }
            object GetTransform() {
                return ReturnTransform(_pgeometry->GetTransform());
            }
            dReal GetSphereRadius() const {
                return _pgeometry->GetSphereRadius();
            }
            dReal GetCylinderRadius() const {
                return _pgeometry->GetCylinderRadius();
            }
            dReal GetCylinderHeight() const {
                return _pgeometry->GetCylinderHeight();
            }
            object GetBoxExtents() const {
                return toPyVector3(_pgeometry->GetBoxExtents());
            }
            object GetRenderScale() const {
                return toPyVector3(_pgeometry->GetRenderScale());
            }
            object GetRenderFilename() const {
                return ConvertStringToUnicode(_pgeometry->GetRenderFilename());
            }
            float GetTransparency() const {
                return _pgeometry->GetTransparency();
            }
            object GetDiffuseColor() const {
                return toPyVector3(_pgeometry->GetDiffuseColor());
            }
            object GetAmbientColor() const {
                return toPyVector3(_pgeometry->GetAmbientColor());
            }
            bool __eq__(boost::shared_ptr<PyGeometry> p) {
                return !!p && _pgeometry == p->_pgeometry;
            }
            bool __ne__(boost::shared_ptr<PyGeometry> p) {
                return !p || _pgeometry != p->_pgeometry;
            }
            int __hash__() {
                return static_cast<int>(uintptr_t(_pgeometry.get()));
            }
        };

        PyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv) : _plink(plink), _pyenv(pyenv) {
        }
        virtual ~PyLink() {
        }

        KinBody::LinkPtr GetLink() {
            return _plink;
        }

        object GetName() {
            return ConvertStringToUnicode(_plink->GetName());
        }
        int GetIndex() {
            return _plink->GetIndex();
        }
        bool IsEnabled() const {
            return _plink->IsEnabled();
        }
        bool SetVisible(bool visible) {
            return _plink->SetVisible(visible);
        }
        bool IsVisible() const {
            return _plink->IsVisible();
        }
        bool IsStatic() const {
            return _plink->IsStatic();
        }
        void Enable(bool bEnable) {
            _plink->Enable(bEnable);
        }

        object GetParent() const;

        object GetParentLinks() const {
            std::vector<KinBody::LinkPtr> vParentLinks;
            _plink->GetParentLinks(vParentLinks);
            boost::python::list links;
            FOREACHC(itlink, vParentLinks) {
                links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
            }
            return links;
        }

        bool IsParentLink(boost::shared_ptr<PyLink> pylink) const {
            return _plink->IsParentLink(pylink->GetLink());
        }

        object GetCollisionData() {
            return toPyTriMesh(_plink->GetCollisionData());
        }
        object ComputeLocalAABB(object otransform) const {
            return toPyAABB(_plink->ComputeLocalAABB());
        }

        object ComputeAABB() const {
            return toPyAABB(_plink->ComputeAABB());
        }
        object GetTransform() const {
            return ReturnTransform(_plink->GetTransform());
        }

        object GetCOMOffset() const {
            return toPyVector3(_plink->GetCOMOffset());
        }
        object GetLocalCOM() const {
            return toPyVector3(_plink->GetLocalCOM());
        }
        object GetGlobalCOM() const {
            return toPyVector3(_plink->GetGlobalCOM());
        }

        object GetLocalInertia() const {
            TransformMatrix t = _plink->GetLocalInertia();
            npy_intp dims[] = { 3, 3};
            PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
            dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
            pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2];
            pdata[3] = t.m[4]; pdata[4] = t.m[5]; pdata[5] = t.m[6];
            pdata[6] = t.m[8]; pdata[7] = t.m[9]; pdata[8] = t.m[10];
            return static_cast<numeric::array>(handle<>(pyvalues));
        }
        dReal GetMass() const {
            return _plink->GetMass();
        }
        object GetPrincipalMomentsOfInertia() const {
            return toPyVector3(_plink->GetPrincipalMomentsOfInertia());
        }
        object GetLocalMassFrame() const {
            return ReturnTransform(_plink->GetLocalMassFrame());
        }
        object GetGlobalMassFrame() const {
            return ReturnTransform(_plink->GetGlobalMassFrame());
        }
        void SetLocalMassFrame(object omassframe) {
            _plink->SetLocalMassFrame(ExtractTransform(omassframe));
        }
        void SetPrincipalMomentsOfInertia(object oinertiamoments) {
            _plink->SetPrincipalMomentsOfInertia(ExtractVector3(oinertiamoments));
        }
        void SetMass(dReal mass) {
            _plink->SetMass(mass);
        }

        void SetStatic(bool bStatic) {
            _plink->SetStatic(bStatic);
        }
        void SetTransform(object otrans) {
            _plink->SetTransform(ExtractTransform(otrans));
        }
        void SetForce(object oforce, object opos, bool bAdd) {
            return _plink->SetForce(ExtractVector3(oforce),ExtractVector3(opos),bAdd);
        }
        void SetTorque(object otorque, bool bAdd) {
            return _plink->SetTorque(ExtractVector3(otorque),bAdd);
        }

        object GetGeometries() {
            boost::python::list geoms;
            size_t N = _plink->GetGeometries().size();
            for(size_t i = 0; i < N; ++i)
                geoms.append(boost::shared_ptr<PyGeometry>(new PyGeometry(_plink->GetGeometry(i))));
            return geoms;
        }

        object GetRigidlyAttachedLinks() const {
            std::vector<KinBody::LinkPtr> vattachedlinks;
            _plink->GetRigidlyAttachedLinks(vattachedlinks);
            boost::python::list links;
            FOREACHC(itlink, vattachedlinks) {
                links.append(PyLinkPtr(new PyLink(*itlink, _pyenv)));
            }
            return links;
        }

        bool IsRigidlyAttached(boost::shared_ptr<PyLink>  plink) {
            CHECK_POINTER(plink);
            return _plink->IsRigidlyAttached(plink->GetLink());
        }

        void SetVelocity(object olinear, object oangular) {
            _plink->SetVelocity(ExtractVector3(olinear),ExtractVector3(oangular));
        }

        object GetVelocity() const {
            std::pair<Vector,Vector> velocity;
            velocity = _plink->GetVelocity();
            boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
            return toPyArray<dReal,6>(v);
        }

        boost::python::dict GetFloatParameters() const {
            boost::python::dict parameters;
            FOREACHC(it, _plink->GetFloatParameters()) {
                parameters[it->first] = toPyArray(it->second);
            }
            return parameters;
        }
        boost::python::dict GetIntParameters() const {
            boost::python::dict parameters;
            FOREACHC(it, _plink->GetIntParameters()) {
                parameters[it->first] = toPyArray(it->second);
            }
            return parameters;
        }

        std::string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetLink('%s')")%RaveGetEnvironmentId(_plink->GetParent()->GetEnv())%_plink->GetParent()->GetName()%_plink->GetName());
        }
        std::string __str__() {
            return boost::str(boost::format("<link:%s (%d), parent=%s>")%_plink->GetName()%_plink->GetIndex()%_plink->GetParent()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyLink> p) {
            return !!p && _plink == p->_plink;
        }
        bool __ne__(boost::shared_ptr<PyLink> p) {
            return !p || _plink != p->_plink;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_plink.get()));
        }
    };

    typedef boost::shared_ptr<PyLink> PyLinkPtr;
    typedef boost::shared_ptr<PyLink const> PyLinkConstPtr;

    class PyJoint
    {
        KinBody::JointPtr _pjoint;
        PyEnvironmentBasePtr _pyenv;
public:
        PyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv) : _pjoint(pjoint), _pyenv(pyenv) {
        }
        virtual ~PyJoint() {
        }

        KinBody::JointPtr GetJoint() {
            return _pjoint;
        }

        object GetName() {
            return ConvertStringToUnicode(_pjoint->GetName());
        }
        bool IsMimic(int iaxis=-1) {
            return _pjoint->IsMimic(iaxis);
        }
        string GetMimicEquation(int iaxis=0, int itype=0, const std::string& format="") {
            return _pjoint->GetMimicEquation(iaxis,itype,format);
        }
        object GetMimicDOFIndices(int iaxis=0) {
            std::vector<int> vmimicdofs;
            _pjoint->GetMimicDOFIndices(vmimicdofs,iaxis);
            return toPyArray(vmimicdofs);
        }
        void SetMimicEquations(int iaxis, const std::string& poseq, const std::string& veleq, const std::string& acceleq) {
            _pjoint->SetMimicEquations(iaxis,poseq,veleq,acceleq);
        }

        dReal GetMaxVel(int iaxis=0) const {
            return _pjoint->GetMaxVel(iaxis);
        }
        dReal GetMaxAccel(int iaxis=0) const {
            return _pjoint->GetMaxAccel(iaxis);
        }
        dReal GetMaxTorque(int iaxis=0) const {
            return _pjoint->GetMaxTorque(iaxis);
        }

        int GetDOFIndex() const {
            return _pjoint->GetDOFIndex();
        }
        int GetJointIndex() const {
            return _pjoint->GetJointIndex();
        }

        PyKinBodyPtr GetParent() const {
            return PyKinBodyPtr(new PyKinBody(_pjoint->GetParent(),_pyenv));
        }

        PyLinkPtr GetFirstAttached() const {
            return !_pjoint->GetFirstAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetFirstAttached(), _pyenv));
        }
        PyLinkPtr GetSecondAttached() const {
            return !_pjoint->GetSecondAttached() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetSecondAttached(), _pyenv));
        }

        KinBody::JointType GetType() const {
            return _pjoint->GetType();
        }
        bool IsCircular(int iaxis) const {
            return _pjoint->IsCircular(iaxis);
        }
        bool IsRevolute(int iaxis) const {
            return _pjoint->IsRevolute(iaxis);
        }
        bool IsPrismatic(int iaxis) const {
            return _pjoint->IsPrismatic(iaxis);
        }
        bool IsStatic() const {
            return _pjoint->IsStatic();
        }

        int GetDOF() const {
            return _pjoint->GetDOF();
        }
        object GetValues() const {
            vector<dReal> values;
            _pjoint->GetValues(values);
            return toPyArray(values);
        }
        object GetVelocities() const {
            vector<dReal> values;
            _pjoint->GetVelocities(values);
            return toPyArray(values);
        }

        object GetAnchor() const {
            return toPyVector3(_pjoint->GetAnchor());
        }
        object GetAxis(int iaxis=0) {
            return toPyVector3(_pjoint->GetAxis(iaxis));
        }
        PyLinkPtr GetHierarchyParentLink() const {
            return !_pjoint->GetHierarchyParentLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyParentLink(),_pyenv));
        }
        PyLinkPtr GetHierarchyChildLink() const {
            return !_pjoint->GetHierarchyChildLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pjoint->GetHierarchyChildLink(),_pyenv));
        }
        object GetInternalHierarchyAxis(int iaxis) {
            return toPyVector3(_pjoint->GetInternalHierarchyAxis(iaxis));
        }
        object GetInternalHierarchyLeftTransform() {
            return ReturnTransform(_pjoint->GetInternalHierarchyLeftTransform());
        }
        object GetInternalHierarchyRightTransform() {
            return ReturnTransform(_pjoint->GetInternalHierarchyRightTransform());
        }

        object GetLimits() const {
            vector<dReal> lower, upper;
            _pjoint->GetLimits(lower,upper);
            return boost::python::make_tuple(toPyArray(lower),toPyArray(upper));
        }
        object GetVelocityLimits() const {
            vector<dReal> vlower,vupper;
            _pjoint->GetVelocityLimits(vlower,vupper);
            return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
        }
        object GetAccelerationLimits() const {
            vector<dReal> v;
            _pjoint->GetAccelerationLimits(v);
            return toPyArray(v);
        }
        object GetTorqueLimits() const {
            vector<dReal> v;
            _pjoint->GetTorqueLimits(v);
            return toPyArray(v);
        }

        dReal GetWrapOffset(int iaxis=0) {
            return _pjoint->GetWrapOffset(iaxis);
        }
        void SetWrapOffset(dReal offset, int iaxis=0) {
            _pjoint->SetWrapOffset(offset,iaxis);
        }
        void SetLimits(object olower, object oupper) {
            vector<dReal> vlower = ExtractArray<dReal>(olower);
            vector<dReal> vupper = ExtractArray<dReal>(oupper);
            if(( vlower.size() != vupper.size()) ||( (int)vlower.size() != _pjoint->GetDOF()) ) {
                throw openrave_exception("limits are wrong dimensions");
            }
            _pjoint->SetLimits(vlower,vupper);
        }
        void SetVelocityLimits(object omaxlimits) {
            vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
            if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
                throw openrave_exception("limits are wrong dimensions");
            }
            _pjoint->SetVelocityLimits(vmaxlimits);
        }
        void SetAccelerationLimits(object omaxlimits) {
            vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
            if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
                throw openrave_exception("limits are wrong dimensions");
            }
            _pjoint->SetAccelerationLimits(vmaxlimits);
        }
        void SetTorqueLimits(object omaxlimits) {
            vector<dReal> vmaxlimits = ExtractArray<dReal>(omaxlimits);
            if( (int)vmaxlimits.size() != _pjoint->GetDOF() ) {
                throw openrave_exception("limits are wrong dimensions");
            }
            _pjoint->SetTorqueLimits(vmaxlimits);
        }

        object GetResolutions() const {
            vector<dReal> resolutions;
            _pjoint->GetResolutions(resolutions);
            return toPyArray(resolutions);
        }
        dReal GetResolution(int iaxis) {
            return _pjoint->GetResolution(iaxis);
        }
        void SetResolution(dReal resolution) {
            _pjoint->SetResolution(resolution);
        }

        object GetWeights() const {
            vector<dReal> weights;
            _pjoint->GetWeights(weights);
            return toPyArray(weights);
        }
        dReal GetWeight(int iaxis) {
            return _pjoint->GetWeight(iaxis);
        }
        void SetWeights(object o) {
            _pjoint->SetWeights(ExtractArray<dReal>(o));
        }

        object SubtractValues(object ovalues0, object ovalues1) {
            vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
            vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
            BOOST_ASSERT((int)values0.size() == GetDOF() );
            BOOST_ASSERT((int)values1.size() == GetDOF() );
            _pjoint->SubtractValues(values0,values1);
            return toPyArray(values0);
        }

        dReal SubtractValue(dReal value0, dReal value1, int iaxis) {
            return _pjoint->SubtractValue(value0,value1,iaxis);
        }

        void AddTorque(object otorques) {
            vector<dReal> vtorques = ExtractArray<dReal>(otorques);
            return _pjoint->AddTorque(vtorques);
        }

        boost::python::dict GetFloatParameters() const {
            boost::python::dict parameters;
            FOREACHC(it, _pjoint->GetFloatParameters()) {
                parameters[it->first] = toPyArray(it->second);
            }
            return parameters;
        }
        boost::python::dict GetIntParameters() const {
            boost::python::dict parameters;
            FOREACHC(it, _pjoint->GetIntParameters()) {
                parameters[it->first] = toPyArray(it->second);
            }
            return parameters;
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetJoint('%s')")%RaveGetEnvironmentId(_pjoint->GetParent()->GetEnv())%_pjoint->GetParent()->GetName()%_pjoint->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<joint:%s (%d), dof=%d, parent=%s>")%_pjoint->GetName()%_pjoint->GetJointIndex()%_pjoint->GetDOFIndex()%_pjoint->GetParent()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyJoint> p) {
            return !!p && _pjoint==p->_pjoint;
        }
        bool __ne__(boost::shared_ptr<PyJoint> p) {
            return !p || _pjoint!=p->_pjoint;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_pjoint.get()));
        }
    };
    typedef boost::shared_ptr<PyJoint> PyJointPtr;
    typedef boost::shared_ptr<PyJoint const> PyJointConstPtr;

    class PyKinBodyStateSaver
    {
        PyEnvironmentBasePtr _pyenv;
        KinBody::KinBodyStateSaver _state;
public:
        PyKinBodyStateSaver(PyKinBodyPtr pybody) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody()) {
        }
        PyKinBodyStateSaver(PyKinBodyPtr pybody, object options) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody(),pyGetIntFromPy(options)) {
        }
        PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(pbody) {
        }
        PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(pbody,pyGetIntFromPy(options)) {
        }
        virtual ~PyKinBodyStateSaver() {
            _state.Release();
        }

        object GetBody() const {
            return object(toPyKinBody(_state.GetBody(),_pyenv));
        }

        void Restore(PyKinBodyPtr pybody=PyKinBodyPtr()) {
            _state.Restore(!pybody ? KinBodyPtr() : pybody->GetBody());
        }

        void Release() {
            _state.Release();
        }

        std::string __str__() {
            KinBodyPtr pbody = _state.GetBody();
            if( !pbody ) {
                return "state empty";
            }
            return boost::str(boost::format("state for %s")%pbody->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
    };
    typedef boost::shared_ptr<PyKinBodyStateSaver> PyKinBodyStateSaverPtr;

    class PyManageData
    {
        KinBody::ManageDataPtr _pdata;
        PyEnvironmentBasePtr _pyenv;
public:
        PyManageData(KinBody::ManageDataPtr pdata, PyEnvironmentBasePtr pyenv) : _pdata(pdata), _pyenv(pyenv) {
        }
        virtual ~PyManageData() {
        }

        KinBody::ManageDataPtr GetManageData() {
            return _pdata;
        }

        object GetSystem() {
            return object(openravepy::toPySensorSystem(_pdata->GetSystem(),_pyenv));
        }

        PyVoidHandleConst GetData() const {
            return PyVoidHandleConst(_pdata->GetData());
        }
        PyLinkPtr GetOffsetLink() const {
            KinBody::LinkPtr plink = _pdata->GetOffsetLink();
            return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,_pyenv));
        }
        bool IsPresent() {
            return _pdata->IsPresent();
        }
        bool IsEnabled() {
            return _pdata->IsEnabled();
        }
        bool IsLocked() {
            return _pdata->IsLocked();
        }
        bool Lock(bool bDoLock) {
            return _pdata->Lock(bDoLock);
        }

        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s').GetManageData()")%RaveGetEnvironmentId(_pdata->GetOffsetLink()->GetParent()->GetEnv())%_pdata->GetOffsetLink()->GetParent()->GetName());
        }
        string __str__() {
            KinBody::LinkPtr plink = _pdata->GetOffsetLink();
            SensorSystemBasePtr psystem = _pdata->GetSystem();
            string systemname = !psystem ? "(NONE)" : psystem->GetXMLId();
            return boost::str(boost::format("<managedata:%s, parent=%s:%s>")%systemname%plink->GetParent()->GetName()%plink->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyManageData> p) {
            return !!p && _pdata==p->_pdata;
        }
        bool __ne__(boost::shared_ptr<PyManageData> p) {
            return !p || _pdata!=p->_pdata;
        }
    };
    typedef boost::shared_ptr<PyManageData> PyManageDataPtr;
    typedef boost::shared_ptr<PyManageData const> PyManageDataConstPtr;

    PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pbody,pyenv), _pbody(pbody) {
    }
    PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv) {
        _pbody = r._pbody;
    }
    virtual ~PyKinBody() {
    }
    KinBodyPtr GetBody() {
        return _pbody;
    }

    bool InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw) {
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
    bool InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw) {
        if( vspheres.shape()[1] != 4 ) {
            throw openrave_exception("spheres needs to be a Nx4 vector\n");
        }
        std::vector<Vector> vvspheres(vspheres.shape()[0]);
        for(size_t i = 0; i < vvspheres.size(); ++i) {
            vvspheres[i] = Vector(vspheres[i][0],vspheres[i][1],vspheres[i][2],vspheres[i][3]);
        }
        return _pbody->InitFromSpheres(vvspheres,bDraw);
    }

    bool InitFromTrimesh(object pytrimesh, bool bDraw) {
        TriMesh mesh;
        if( ExtractTriMesh(pytrimesh,mesh) ) {
            return _pbody->InitFromTrimesh(mesh,bDraw);
        }
        else {
            throw openrave_exception("bad trimesh");
        }
    }

    bool InitFromGeometries(object ogeometries) {
        std::vector<KinBody::GeometryInfoConstPtr> geometries(len(ogeometries));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = boost::python::extract<PyGeometryInfoPtr>(ogeometries[i]);
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.GeometryInfo",ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        return _pbody->InitFromGeometries(geometries);
    }

    bool Init(object olinkinfos, object ojointinfos) {
        std::vector<KinBody::LinkInfoConstPtr> vlinkinfos(len(olinkinfos));
        for(size_t i = 0; i < vlinkinfos.size(); ++i) {
            PyLinkInfoPtr pylink = boost::python::extract<PyLinkInfoPtr>(olinkinfos[i]);
            if( !pylink ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.LinkInfo",ORE_InvalidArguments);
            }
            vlinkinfos[i] = pylink->GetLinkInfo();
        }
        std::vector<KinBody::JointInfoConstPtr> vjointinfos(len(ojointinfos));
        for(size_t i = 0; i < vjointinfos.size(); ++i) {
            PyJointInfoPtr pyjoint = boost::python::extract<PyJointInfoPtr>(ojointinfos[i]);
            if( !pyjoint ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.JointInfo",ORE_InvalidArguments);
            }
            vjointinfos[i] = pyjoint->GetJointInfo();
        }
        return _pbody->Init(vlinkinfos, vjointinfos);
    }

    void SetName(const std::string& name) {
        _pbody->SetName(name);
    }
    object GetName() const {
        return ConvertStringToUnicode(_pbody->GetName());
    }
    int GetDOF() const {
        return _pbody->GetDOF();
    }

    object GetDOFValues() const {
        vector<dReal> values;
        _pbody->GetDOFValues(values);
        return toPyArray(values);
    }
    object GetDOFValues(object oindices) const {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _pbody->GetDOFValues(values,vindices);
        return toPyArray(values);
    }

    object GetDOFVelocities() const {
        vector<dReal> values;
        _pbody->GetDOFVelocities(values);
        return toPyArray(values);
    }

    object GetDOFVelocities(object oindices) const {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _pbody->GetDOFVelocities(values,vindices);
        return toPyArray(values);
    }

    object GetDOFLimits() const {
        vector<dReal> vlower, vupper;
        _pbody->GetDOFLimits(vlower,vupper);
        return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
    }

    object GetDOFVelocityLimits() const {
        vector<dReal> vmax;
        _pbody->GetDOFVelocityLimits(vmax);
        return toPyArray(vmax);
    }

    object GetDOFAccelerationLimits() const {
        vector<dReal> vmax;
        _pbody->GetDOFAccelerationLimits(vmax);
        return toPyArray(vmax);
    }

    object GetDOFTorqueLimits() const {
        vector<dReal> vmax;
        _pbody->GetDOFTorqueLimits(vmax);
        return toPyArray(vmax);
    }

    object GetDOFLimits(object oindices) const {
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

    object GetDOFVelocityLimits(object oindices) const {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> vmax, vtempmax;
        vmax.reserve(vindices.size());
        FOREACHC(it, vindices) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
            pjoint->GetVelocityLimits(vtempmax,false);
            vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
        }
        return toPyArray(vmax);
    }

    object GetDOFAccelerationLimits(object oindices) const {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> vmax, vtempmax;
        vmax.reserve(vindices.size());
        FOREACHC(it, vindices) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
            pjoint->GetAccelerationLimits(vtempmax,false);
            vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
        }
        return toPyArray(vmax);
    }

    object GetDOFTorqueLimits(object oindices) const {
        if( oindices == object() ) {
            return numeric::array(boost::python::list());
        }
        vector<int> vindices = ExtractArray<int>(oindices);
        if( vindices.size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> vmax, vtempmax;
        vmax.reserve(vindices.size());
        FOREACHC(it, vindices) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(*it);
            pjoint->GetTorqueLimits(vtempmax,false);
            vmax.push_back(vtempmax.at(*it-pjoint->GetDOFIndex()));
        }
        return toPyArray(vmax);
    }

    object GetDOFMaxVel() const {
        RAVELOG_WARN("KinBody.GetDOFMaxVel() is deprecated, use GetDOFVelocityLimits\n");
        vector<dReal> values;
        _pbody->GetDOFVelocityLimits(values);
        return toPyArray(values);
    }
    object GetDOFMaxTorque() const {
        vector<dReal> values;
        _pbody->GetDOFMaxTorque(values);
        return toPyArray(values);
    }
    object GetDOFMaxAccel() const {
        RAVELOG_WARN("KinBody.GetDOFMaxAccel() is deprecated, use GetDOFAccelerationLimits\n");
        vector<dReal> values;
        _pbody->GetDOFAccelerationLimits(values);
        return toPyArray(values);
    }

    object GetDOFWeights() const {
        vector<dReal> values;
        _pbody->GetDOFWeights(values);
        return toPyArray(values);
    }

    object GetDOFWeights(object oindices) const {
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
            values.push_back(pjoint->GetWeight(*it-pjoint->GetDOFIndex()));
        }
        return toPyArray(values);
    }

    object GetDOFResolutions() const {
        vector<dReal> values;
        _pbody->GetDOFResolutions(values);
        return toPyArray(values);
    }

    object GetDOFResolutions(object oindices) const {
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
            values.push_back(pjoint->GetResolution());
        }
        return toPyArray(values);
    }

    object GetLinks() const {
        boost::python::list links;
        FOREACHC(itlink, _pbody->GetLinks()) {
            links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        }
        return links;
    }

    object GetLinks(object oindices) const {
        if( oindices == object() ) {
            return GetLinks();
        }
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
        if( oindices == object() ) {
            return GetJoints();
        }
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
        FOREACHC(itjoint, _pbody->GetPassiveJoints()) {
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        }
        return joints;
    }

    object GetDependencyOrderedJoints()
    {
        boost::python::list joints;
        FOREACHC(itjoint, _pbody->GetDependencyOrderedJoints()) {
            joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
        }
        return joints;
    }

    object GetClosedLoops()
    {
        boost::python::list loops;
        FOREACHC(itloop, _pbody->GetClosedLoops()) {
            boost::python::list loop;
            FOREACHC(itpair,*itloop) {
                loop.append(boost::python::make_tuple(PyLinkPtr(new PyLink(itpair->first,GetEnv())),PyJointPtr(new PyJoint(itpair->second,GetEnv()))));
            }
            loops.append(loop);
        }
        return loops;
    }

    object GetRigidlyAttachedLinks(int linkindex) const
    {
        RAVELOG_WARN("KinBody.GetRigidlyAttachedLinks is deprecated, use KinBody.Link.GetRigidlyAttachedLinks\n");
        std::vector<KinBody::LinkPtr> vattachedlinks;
        _pbody->GetLinks().at(linkindex)->GetRigidlyAttachedLinks(vattachedlinks);
        boost::python::list links;
        FOREACHC(itlink, vattachedlinks) {
            links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
        }
        return links;
    }

    object GetChain(int linkindex1, int linkindex2,bool returnjoints = true) const
    {
        boost::python::list chain;
        if( returnjoints ) {
            std::vector<KinBody::JointPtr> vjoints;
            _pbody->GetChain(linkindex1,linkindex2,vjoints);
            FOREACHC(itjoint, vjoints) {
                chain.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
            }
        }
        else {
            std::vector<KinBody::LinkPtr> vlinks;
            _pbody->GetChain(linkindex1,linkindex2,vlinks);
            FOREACHC(itlink, vlinks) {
                chain.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
            }
        }
        return chain;
    }

    bool IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const
    {
        return _pbody->IsDOFInChain(linkindex1,linkindex2,dofindex);
    }

    int GetJointIndex(const std::string& jointname) const
    {
        return _pbody->GetJointIndex(jointname);
    }

    PyJointPtr GetJoint(const std::string& jointname) const
    {
        KinBody::JointPtr pjoint = _pbody->GetJoint(jointname);
        return !pjoint ? PyJointPtr() : PyJointPtr(new PyJoint(pjoint,GetEnv()));
    }

    PyJointPtr GetJointFromDOFIndex(int dofindex) const
    {
        KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(dofindex);
        return !pjoint ? PyJointPtr() : PyJointPtr(new PyJoint(pjoint,GetEnv()));
    }

    object GetTransform() const {
        return ReturnTransform(_pbody->GetTransform());
    }

    object GetLinkTransformations(bool returndofbranches=false) const
    {
        boost::python::list otransforms;
        vector<Transform> vtransforms;
        std::vector<int> vdofbranches;
        _pbody->GetLinkTransformations(vtransforms,vdofbranches);
        FOREACHC(it, vtransforms) {
            otransforms.append(ReturnTransform(*it));
        }
        if( returndofbranches ) {
            return boost::python::make_tuple(otransforms, toPyArray(vdofbranches));
        }
        return otransforms;
    }

    void SetLinkTransformations(object transforms, object odofbranches=object())
    {
        size_t numtransforms = len(transforms);
        if( numtransforms != _pbody->GetLinks().size() ) {
            throw openrave_exception("number of input transforms not equal to links");
        }
        std::vector<Transform> vtransforms(numtransforms);
        for(size_t i = 0; i < numtransforms; ++i) {
            vtransforms[i] = ExtractTransform(transforms[i]);
        }
        if( odofbranches == object() ) {
            _pbody->SetLinkTransformations(vtransforms);
        }
        else {
            _pbody->SetLinkTransformations(vtransforms, ExtractArray<int>(odofbranches));
        }
    }

    void SetLinkVelocities(object ovelocities)
    {
        std::vector<std::pair<Vector,Vector> > velocities;
        velocities.resize(len(ovelocities));
        for(size_t i = 0; i < velocities.size(); ++i) {
            vector<dReal> v = ExtractArray<dReal>(ovelocities[i]);
            BOOST_ASSERT(v.size()==6);
            velocities[i].first.x = v[0];
            velocities[i].first.y = v[1];
            velocities[i].first.z = v[2];
            velocities[i].second.x = v[3];
            velocities[i].second.y = v[4];
            velocities[i].second.z = v[5];
        }
        return _pbody->SetLinkVelocities(velocities);
    }

    bool SetVelocity(object olinearvel, object oangularvel)
    {
        return _pbody->SetVelocity(ExtractVector3(olinearvel),ExtractVector3(oangularvel));
    }

    void SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel, uint32_t checklimits)
    {
        _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel),checklimits);
    }

    void SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel)
    {
        _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel));
    }

    void SetDOFVelocities(object odofvelocities)
    {
        _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities));
    }

    void SetDOFVelocities(object odofvelocities, uint32_t checklimits=KinBody::CLA_CheckLimits, object oindices = object())
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> vsetvalues = ExtractArray<dReal>(odofvelocities);
        if( oindices == object() ) {
            _pbody->SetDOFVelocities(vsetvalues,checklimits);
        }
        else {
            if( len(oindices) == 0 ) {
                return;
            }
            vector<int> vindices = ExtractArray<int>(oindices);
            _pbody->SetDOFVelocities(vsetvalues,checklimits, vindices);
        }
    }

    object GetLinkVelocities() const
    {
        if( _pbody->GetLinks().size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        std::vector<std::pair<Vector,Vector> > velocities;
        _pbody->GetLinkVelocities(velocities);

        npy_intp dims[] = { velocities.size(),6};
        PyObject *pyvel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pfvel = (dReal*)PyArray_DATA(pyvel);
        for(size_t i = 0; i < velocities.size(); ++i) {
            pfvel[6*i+0] = velocities[i].first.x;
            pfvel[6*i+1] = velocities[i].first.y;
            pfvel[6*i+2] = velocities[i].first.z;
            pfvel[6*i+3] = velocities[i].second.x;
            pfvel[6*i+4] = velocities[i].second.y;
            pfvel[6*i+5] = velocities[i].second.z;
        }
        return static_cast<numeric::array>(handle<>(pyvel));
    }

    object GetLinkAccelerations(object odofaccelerations) const
    {
        if( _pbody->GetLinks().size() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> vDOFAccelerations = ExtractArray<dReal>(odofaccelerations);
        std::vector<std::pair<Vector,Vector> > vLinkAccelerations;
        _pbody->GetLinkAccelerations(vDOFAccelerations,vLinkAccelerations);

        npy_intp dims[] = { vLinkAccelerations.size(),6};
        PyObject *pyaccel = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
        dReal* pf = (dReal*)PyArray_DATA(pyaccel);
        for(size_t i = 0; i < vLinkAccelerations.size(); ++i) {
            pf[6*i+0] = vLinkAccelerations[i].first.x;
            pf[6*i+1] = vLinkAccelerations[i].first.y;
            pf[6*i+2] = vLinkAccelerations[i].first.z;
            pf[6*i+3] = vLinkAccelerations[i].second.x;
            pf[6*i+4] = vLinkAccelerations[i].second.y;
            pf[6*i+5] = vLinkAccelerations[i].second.z;
        }
        return static_cast<numeric::array>(handle<>(pyaccel));
    }

    object ComputeAABB() {
        return toPyAABB(_pbody->ComputeAABB());
    }
    void Enable(bool bEnable) {
        _pbody->Enable(bEnable);
    }
    bool IsEnabled() const {
        return _pbody->IsEnabled();
    }
    bool SetVisible(bool visible) {
        return _pbody->SetVisible(visible);
    }
    bool IsVisible() const {
        return _pbody->IsVisible();
    }
    void SetTransform(object transform) {
        _pbody->SetTransform(ExtractTransform(transform));
    }

    void SetDOFWeights(object o)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFWeights(values);
    }

    void SetDOFLimits(object olower, object oupper)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> vlower = ExtractArray<dReal>(olower), vupper = ExtractArray<dReal>(oupper);
        if( (int)vlower.size() != GetDOF() || (int)vupper.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFLimits(vlower,vupper);
    }

    void SetDOFVelocityLimits(object o)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFVelocityLimits(values);
    }

    void SetDOFAccelerationLimits(object o)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFAccelerationLimits(values);
    }

    void SetDOFTorqueLimits(object o)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFTorqueLimits(values);
    }

    void SetDOFValues(object o)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(o);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFValues(values,true);
    }
    void SetTransformWithDOFValues(object otrans,object ojoints)
    {
        if( _pbody->GetDOF() == 0 ) {
            _pbody->SetTransform(ExtractTransform(otrans));
            return;
        }
        vector<dReal> values = ExtractArray<dReal>(ojoints);
        if( (int)values.size() != GetDOF() ) {
            throw openrave_exception("values do not equal to body degrees of freedom");
        }
        _pbody->SetDOFValues(values,ExtractTransform(otrans),true);
    }

    void SetDOFValues(object o, object indices, uint32_t checklimits)
    {
        if( _pbody->GetDOF() == 0 ) {
            return;
        }
        vector<dReal> vsetvalues = ExtractArray<dReal>(o);
        if( indices == object() ) {
            _pbody->SetDOFValues(vsetvalues,checklimits);
        }
        else {
            if( len(indices) == 0 ) {
                return;
            }
            vector<int> vindices = ExtractArray<int>(indices);
            _pbody->SetDOFValues(vsetvalues,checklimits, vindices);
        }
    }

    void SetDOFValues(object o, object indices)
    {
        SetDOFValues(o,indices,true);
    }

    object SubtractDOFValues(object ovalues0, object ovalues1)
    {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetDOF() );
        BOOST_ASSERT((int)values1.size() == GetDOF() );
        _pbody->SubtractDOFValues(values0,values1);
        return toPyArray(values0);
    }

    void SetDOFTorques(object otorques, bool bAdd)
    {
        vector<dReal> vtorques = ExtractArray<dReal>(otorques);
        BOOST_ASSERT((int)vtorques.size() == GetDOF() );
        _pbody->SetDOFTorques(vtorques,bAdd);
    }

    object ComputeJacobianTranslation(int index, object oposition, object oindices=object())
    {
        vector<int> vindices;
        if( oindices != object() ) {
            vindices = ExtractArray<int>(oindices);
        }
        std::vector<dReal> vjacobian;
        _pbody->ComputeJacobianTranslation(index,ExtractVector3(oposition),vjacobian,vindices);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
        return toPyArray(vjacobian,dims);
    }

    object ComputeJacobianAxisAngle(int index, object oindices=object())
    {
        vector<int> vindices;
        if( oindices != object() ) {
            vindices = ExtractArray<int>(oindices);
        }
        std::vector<dReal> vjacobian;
        _pbody->ComputeJacobianAxisAngle(index,vjacobian,vindices);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
        return toPyArray(vjacobian,dims);
    }

    object CalculateJacobian(int index, object oposition)
    {
        std::vector<dReal> vjacobian;
        _pbody->CalculateJacobian(index,ExtractVector3(oposition),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
        return toPyArray(vjacobian,dims);
    }

    object CalculateRotationJacobian(int index, object q) const
    {
        std::vector<dReal> vjacobian;
        _pbody->CalculateRotationJacobian(index,ExtractVector4(q),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pbody->GetDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateAngularVelocityJacobian(int index) const
    {
        std::vector<dReal> vjacobian;
        _pbody->ComputeJacobianAxisAngle(index,vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pbody->GetDOF();
        return toPyArray(vjacobian,dims);
    }

    object ComputeHessianTranslation(int index, object oposition, object oindices=object())
    {
        vector<int> vindices;
        if( oindices != object() ) {
            vindices = ExtractArray<int>(oindices);
        }
        size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
        std::vector<dReal> vhessian;
        _pbody->ComputeHessianTranslation(index,ExtractVector3(oposition),vhessian,vindices);
        std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
        return toPyArray(vhessian,dims);
    }

    object ComputeHessianAxisAngle(int index, object oindices=object())
    {
        vector<int> vindices;
        if( oindices != object() ) {
            vindices = ExtractArray<int>(oindices);
        }
        size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
        std::vector<dReal> vhessian;
        _pbody->ComputeHessianAxisAngle(index,vhessian,vindices);
        std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
        return toPyArray(vhessian,dims);
    }

    object ComputeInverseDynamics(object odofaccelerations, object oexternalforcetorque=object(), bool returncomponents=false)
    {
        vector<dReal> vDOFAccelerations;
        if( odofaccelerations != object() ) {
            vDOFAccelerations = ExtractArray<dReal>(odofaccelerations);
        }
        KinBody::ForceTorqueMap mapExternalForceTorque;
        if( oexternalforcetorque != object() ) {
            boost::python::dict odict = (boost::python::dict)oexternalforcetorque;
            boost::python::list iterkeys = (boost::python::list)odict.iterkeys();
            vector<dReal> v;
            for (int i = 0; i < boost::python::len(iterkeys); i++) {
                int linkindex = boost::python::extract<int>(iterkeys[i]);
                object oforcetorque = odict[iterkeys[i]];
                OPENRAVE_ASSERT_OP(len(oforcetorque),==,6);
                mapExternalForceTorque[linkindex] = make_pair(Vector(boost::python::extract<dReal>(oforcetorque[0]),boost::python::extract<dReal>(oforcetorque[1]),boost::python::extract<dReal>(oforcetorque[2])),Vector(boost::python::extract<dReal>(oforcetorque[3]),boost::python::extract<dReal>(oforcetorque[4]),boost::python::extract<dReal>(oforcetorque[5])));
            }
        }
        if( returncomponents ) {
            boost::array< vector<dReal>, 3> vDOFTorqueComponents;
            _pbody->ComputeInverseDynamics(vDOFTorqueComponents,vDOFAccelerations,mapExternalForceTorque);
            return boost::python::make_tuple(toPyArray(vDOFTorqueComponents[0]), toPyArray(vDOFTorqueComponents[1]), toPyArray(vDOFTorqueComponents[2]));
        }
        else {
            vector<dReal> vDOFTorques;
            _pbody->ComputeInverseDynamics(vDOFTorques,vDOFAccelerations,mapExternalForceTorque);
            return toPyArray(vDOFTorques);
        }
    }

    bool CheckSelfCollision() {
        return _pbody->CheckSelfCollision();
    }
    bool CheckSelfCollision(PyCollisionReportPtr pReport)
    {
        bool bCollision = _pbody->CheckSelfCollision(openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,GetEnv());
        return bCollision;
    }

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

    void SetZeroConfiguration() {
        _pbody->SetZeroConfiguration();
    }
    void SetNonCollidingConfiguration() {
        _pbody->SetNonCollidingConfiguration();
    }

    object GetConfigurationSpecification(const std::string& interpolation="") const {
        return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecification(interpolation)));
    }

    object GetConfigurationSpecificationIndices(object oindices,const std::string& interpolation="") const {
        vector<int> vindices = ExtractArray<int>(oindices);
        return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecificationIndices(vindices,interpolation)));
    }

    void SetConfigurationValues(object ovalues, uint32_t checklimits=KinBody::CLA_CheckLimits) {
        vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
        BOOST_ASSERT((int)vvalues.size()==_pbody->GetDOF()+7);
        _pbody->SetConfigurationValues(vvalues.begin(),checklimits);
    }

    object GetConfigurationValues() const {
        vector<dReal> values;
        _pbody->GetConfigurationValues(values);
        return toPyArray(values);
    }

    bool IsRobot() const {
        return _pbody->IsRobot();
    }
    int GetEnvironmentId() const {
        return _pbody->GetEnvironmentId();
    }

    int DoesAffect(int jointindex, int linkindex ) const {
        return _pbody->DoesAffect(jointindex,linkindex);
    }

    object GetViewerData() const {
        return toPyUserData(_pbody->GetViewerData());
    }

    object GetURI() const {
        return ConvertStringToUnicode(_pbody->GetURI());
    }

    object GetNonAdjacentLinks() const {
        boost::python::list ononadjacent;
        const std::set<int>& nonadjacent = _pbody->GetNonAdjacentLinks();
        FOREACHC(it,nonadjacent) {
            ononadjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        }
        return ononadjacent;
    }
    object GetNonAdjacentLinks(int adjacentoptions) const {
        boost::python::list ononadjacent;
        const std::set<int>& nonadjacent = _pbody->GetNonAdjacentLinks(adjacentoptions);
        FOREACHC(it,nonadjacent) {
            ononadjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        }
        return ononadjacent;
    }

    object GetAdjacentLinks() const {
        boost::python::list adjacent;
        FOREACHC(it,_pbody->GetAdjacentLinks())
        adjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
        return adjacent;
    }

    object GetPhysicsData() const {
        return toPyUserData(_pbody->GetPhysicsData());
    }
    object GetCollisionData() const {
        return toPyUserData(_pbody->GetCollisionData());
    }
    PyManageDataPtr GetManageData() const {
        KinBody::ManageDataPtr pdata = _pbody->GetManageData();
        return !pdata ? PyManageDataPtr() : PyManageDataPtr(new PyManageData(pdata,_pyenv));
    }
    int GetUpdateStamp() const {
        return _pbody->GetUpdateStamp();
    }

    string serialize(int options) const {
        stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        _pbody->serialize(ss,options);
        return ss.str();
    }

    string GetKinematicsGeometryHash() const {
        return _pbody->GetKinematicsGeometryHash();
    }
    PyStateRestoreContextBase* CreateKinBodyStateSaver(object options=object()) {
        PyKinBodyStateSaverPtr saver;
        if( options == object() ) {
            saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv));
        }
        else {
            saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv,options));
        }
        return new PyStateRestoreContext<PyKinBodyStateSaverPtr, PyKinBodyPtr>(saver);
    }

    virtual string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s')")%RaveGetEnvironmentId(_pbody->GetEnv())%_pbody->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_pbody->GetInterfaceType())%_pbody->GetXMLId()%_pbody->GetName()%_pbody->GetKinematicsGeometryHash());
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    virtual void __enter__()
    {
        // necessary to lock physics to prevent multiple threads from interfering
        if( _listStateSavers.size() == 0 ) {
            openravepy::LockEnvironment(_pyenv);
        }
        _listStateSavers.push_back(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody)));
    }

    virtual void __exit__(object type, object value, object traceback)
    {
        BOOST_ASSERT(_listStateSavers.size()>0);
        _listStateSavers.pop_back();
        if( _listStateSavers.size() == 0 ) {
            openravepy::UnlockEnvironment(_pyenv);
        }
    }

};

class PyRobotBase : public PyKinBody
{
protected:
    RobotBasePtr _probot;
public:
    RobotBasePtr GetRobot() {
        return _probot;
    }

    class PyManipulatorInfo
    {
public:
        PyManipulatorInfo() {
            _tLocalTool = ReturnTransform(Transform());
            _vClosingDirection = numeric::array(boost::python::list());
            _vdirection = toPyVector3(Vector(0,0,1));
            _vGripperJointNames = boost::python::list();
        }
        RobotBase::ManipulatorInfo GetManipulatorInfo() {
            RobotBase::ManipulatorInfo info;
            info._name = _name;
            info._sBaseLinkName = _sBaseLinkName;
            info._sEffectorLinkName = _sEffectorLinkName;
            info._tLocalTool = ExtractTransform(_tLocalTool);
            info._vClosingDirection = ExtractArray<dReal>(_vClosingDirection);
            info._vdirection = ExtractVector3(_vdirection);
            info._sIkSolverXMLId = _sIkSolverXMLId;
            info._vGripperJointNames = ExtractArray<std::string>(_vGripperJointNames);
            return info;
        }

        std::string _name, _sBaseLinkName, _sEffectorLinkName;
        object _tLocalTool;
        object _vClosingDirection;
        object _vdirection;
        std::string _sIkSolverXMLId;
        object _vGripperJointNames;
    };
    typedef boost::shared_ptr<PyManipulatorInfo> PyManipulatorInfoPtr;

    class PyManipulator
    {
        RobotBase::ManipulatorPtr _pmanip;
        PyEnvironmentBasePtr _pyenv;
public:
        PyManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv) : _pmanip(pmanip),_pyenv(pyenv) {
        }
        virtual ~PyManipulator() {
        }

        RobotBase::ManipulatorPtr GetManipulator() const {
            return _pmanip;
        }

        object GetTransform() const {
            return ReturnTransform(_pmanip->GetTransform());
        }

        object GetVelocity() const {
            std::pair<Vector, Vector> velocity;
            velocity = _pmanip->GetVelocity();
            boost::array<dReal,6> v = {{ velocity.first.x, velocity.first.y, velocity.first.z, velocity.second.x, velocity.second.y, velocity.second.z}};
            return toPyArray<dReal,6>(v);
        }

        object GetName() const {
            return ConvertStringToUnicode(_pmanip->GetName());
        }

        void SetName(const std::string& s) {
            _pmanip->SetName(s);
        }

        PyRobotBasePtr GetRobot() {
            return PyRobotBasePtr(new PyRobotBase(_pmanip->GetRobot(),_pyenv));
        }

        bool SetIkSolver(PyIkSolverBasePtr iksolver) {
            return _pmanip->SetIkSolver(openravepy::GetIkSolver(iksolver));
        }
        object GetIkSolver() {
            return object(openravepy::toPyIkSolver(_pmanip->GetIkSolver(),_pyenv));
        }

        boost::shared_ptr<PyLink> GetBase() {
            return !_pmanip->GetBase() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetBase(),_pyenv));
        }
        boost::shared_ptr<PyLink> GetEndEffector() {
            return !_pmanip->GetEndEffector() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pmanip->GetEndEffector(),_pyenv));
        }
        object GetGraspTransform() {
            RAVELOG_WARN("Robot.Manipulator.GetGraspTransform deprecated, use GetLocalToolTransform\n");
            return ReturnTransform(_pmanip->GetLocalToolTransform());
        }
        object GetLocalToolTransform() {
            return ReturnTransform(_pmanip->GetLocalToolTransform());
        }
        void SetLocalToolTransform(object otrans) {
            _pmanip->SetLocalToolTransform(ExtractTransform(otrans));
        }
        object GetGripperJoints() {
            RAVELOG_DEBUG("GetGripperJoints is deprecated, use GetGripperIndices\n");
            return toPyArray(_pmanip->GetGripperIndices());
        }
        object GetGripperIndices() {
            return toPyArray(_pmanip->GetGripperIndices());
        }
        object GetArmJoints() {
            RAVELOG_DEBUG("GetArmJoints is deprecated, use GetArmIndices\n");
            return toPyArray(_pmanip->GetArmIndices());
        }
        object GetArmIndices() {
            return toPyArray(_pmanip->GetArmIndices());
        }
        object GetClosingDirection() {
            return toPyArray(_pmanip->GetClosingDirection());
        }
        object GetPalmDirection() {
            RAVELOG_INFO("GetPalmDirection deprecated to GetDirection\n");
            return toPyVector3(_pmanip->GetDirection());
        }
        object GetDirection() {
            return toPyVector3(_pmanip->GetLocalToolDirection());
        }
        object GetLocalToolDirection() {
            return toPyVector3(_pmanip->GetLocalToolDirection());
        }
        bool IsGrabbing(PyKinBodyPtr pbody) {
            return _pmanip->IsGrabbing(pbody->GetBody());
        }

        int GetNumFreeParameters() const {
            RAVELOG_WARN("Manipulator::GetNumFreeParameters() is deprecated\n");
            return _pmanip->GetIkSolver()->GetNumFreeParameters();
        }

        object GetFreeParameters() const {
            RAVELOG_WARN("Manipulator::GetFreeParameters() is deprecated\n");
            if( _pmanip->GetIkSolver()->GetNumFreeParameters() == 0 ) {
                return numeric::array(boost::python::list());
            }
            vector<dReal> values;
            _pmanip->GetIkSolver()->GetFreeParameters(values);
            return toPyArray(values);
        }

        bool _FindIKSolution(const IkParameterization& ikparam, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,solution,filteroptions);

        }
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,vFreeParameters, solution,filteroptions);
        }
        bool _FindIKSolution(const IkParameterization& ikparam, int filteroptions, IkReturn& ikreturn, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
        }
        bool _FindIKSolution(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturn& ikreturn, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolution(ikparam,vFreeParameters, filteroptions,IkReturnPtr(&ikreturn,utils::null_deleter()));
        }

        bool _FindIKSolutions(const IkParameterization& ikparam, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,solutions,filteroptions);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,vFreeParameters,solutions,filteroptions);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,filteroptions,vikreturns);
        }
        bool _FindIKSolutions(const IkParameterization& ikparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns, bool releasegil) const
        {
            openravepy::PythonThreadSaverPtr statesaver;
            if( releasegil ) {
                statesaver.reset(new openravepy::PythonThreadSaver());
            }
            return _pmanip->FindIKSolutions(ikparam,vFreeParameters,filteroptions,vikreturns);
        }

        object FindIKSolution(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ExtractIkParameterization(oparam,ikparam) ) {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ikparam,solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
            // assume transformation matrix
            else {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ExtractTransform(oparam),filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ExtractTransform(oparam),solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
        }

        object FindIKSolution(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ExtractIkParameterization(oparam,ikparam) ) {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ikparam,vfreeparams,solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
            // assume transformation matrix
            else {
                if( ikreturn ) {
                    IkReturn ikreturn(IKRA_Reject);
                    _FindIKSolution(ikparam,vfreeparams,filteroptions,ikreturn,releasegil);
                    return openravepy::toPyIkReturn(ikreturn);
                }
                else {
                    vector<dReal> solution;
                    if( !_FindIKSolution(ExtractTransform(oparam),vfreeparams, solution,filteroptions,releasegil) ) {
                        return object();
                    }
                    return toPyArray(solution);
                }
            }
        }

        object FindIKSolutions(object oparam, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ikreturn ) {
                std::vector<IkReturnPtr> vikreturns;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,filteroptions,vikreturns,releasegil) ) {
                        return boost::python::list();
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),filteroptions,vikreturns,releasegil) ) {
                    return boost::python::list();
                }

                boost::python::list oikreturns;
                FOREACH(it,vikreturns) {
                    oikreturns.append(openravepy::toPyIkReturn(**it));
                }
                return oikreturns;
            }
            else {
                std::vector<std::vector<dReal> > vsolutions;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vsolutions,filteroptions,releasegil) ) {
                        return numeric::array(boost::python::list());
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vsolutions,filteroptions,releasegil) ) {
                    return numeric::array(boost::python::list());
                }

                npy_intp dims[] = { vsolutions.size(),_pmanip->GetArmIndices().size() };
                PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
                dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
                FOREACH(itsol,vsolutions) {
                    BOOST_ASSERT(itsol->size()==size_t(dims[1]));
                    std::copy(itsol->begin(),itsol->end(),ppos);
                    ppos += itsol->size();
                }
                return static_cast<numeric::array>(handle<>(pysolutions));
            }
        }

        object FindIKSolutions(object oparam, object freeparams, int filteroptions, bool ikreturn=false, bool releasegil=false) const
        {
            vector<dReal> vfreeparams = ExtractArray<dReal>(freeparams);
            IkParameterization ikparam;
            EnvironmentMutex::scoped_lock lock(openravepy::GetEnvironment(_pyenv)->GetMutex()); // lock just in case since many users call this without locking...
            if( ikreturn ) {
                std::vector<IkReturnPtr> vikreturns;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vfreeparams,filteroptions,vikreturns,releasegil) ) {
                        return boost::python::list();
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams,filteroptions,vikreturns,releasegil) ) {
                    return boost::python::list();
                }

                boost::python::list oikreturns;
                FOREACH(it,vikreturns) {
                    oikreturns.append(openravepy::toPyIkReturn(**it));
                }
                return oikreturns;
            }
            else {
                std::vector<std::vector<dReal> > vsolutions;
                if( ExtractIkParameterization(oparam,ikparam) ) {
                    if( !_FindIKSolutions(ikparam,vfreeparams,vsolutions,filteroptions,releasegil) ) {
                        return numeric::array(boost::python::list());
                    }
                }
                // assume transformation matrix
                else if( !_FindIKSolutions(ExtractTransform(oparam),vfreeparams, vsolutions,filteroptions,releasegil) ) {
                    return numeric::array(boost::python::list());
                }

                npy_intp dims[] = { vsolutions.size(),_pmanip->GetArmIndices().size() };
                PyObject *pysolutions = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
                dReal* ppos = (dReal*)PyArray_DATA(pysolutions);
                FOREACH(itsol,vsolutions) {
                    BOOST_ASSERT(itsol->size()==size_t(dims[1]));
                    std::copy(itsol->begin(),itsol->end(),ppos);
                    ppos += itsol->size();
                }
                return static_cast<numeric::array>(handle<>(pysolutions));
            }
        }

        object GetIkParameterization(object oparam, bool inworld=true)
        {
            IkParameterization ikparam;
            if( ExtractIkParameterization(oparam,ikparam) ) {
                return toPyIkParameterization(_pmanip->GetIkParameterization(ikparam,inworld));
            }
            // must be IkParameterizationType
            return toPyIkParameterization(_pmanip->GetIkParameterization((IkParameterizationType)extract<IkParameterizationType>(oparam),inworld));
        }

        object GetChildJoints() {
            std::vector<KinBody::JointPtr> vjoints;
            _pmanip->GetChildJoints(vjoints);
            boost::python::list joints;
            FOREACH(itjoint,vjoints) {
                joints.append(PyJointPtr(new PyJoint(*itjoint, _pyenv)));
            }
            return joints;
        }
        object GetChildDOFIndices() {
            std::vector<int> vdofindices;
            _pmanip->GetChildDOFIndices(vdofindices);
            boost::python::list dofindices;
            FOREACH(itindex,vdofindices) {
                dofindices.append(*itindex);
            }
            return dofindices;
        }

        object GetChildLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetChildLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks) {
                links.append(PyLinkPtr(new PyLink(*itlink,_pyenv)));
            }
            return links;
        }

        bool IsChildLink(PyLinkPtr plink)
        {
            CHECK_POINTER(plink);
            return _pmanip->IsChildLink(plink->GetLink());
        }

        object GetIndependentLinks() {
            std::vector<KinBody::LinkPtr> vlinks;
            _pmanip->GetIndependentLinks(vlinks);
            boost::python::list links;
            FOREACH(itlink,vlinks) {
                links.append(PyLinkPtr(new PyLink(*itlink,_pyenv)));
            }
            return links;
        }

        object GetArmConfigurationSpecification(const std::string& interpolation="") const {
            return object(openravepy::toPyConfigurationSpecification(_pmanip->GetArmConfigurationSpecification(interpolation)));
        }

        bool CheckEndEffectorCollision(object otrans) const
        {
            IkParameterization ikparam;
            if( ExtractIkParameterization(otrans,ikparam) ) {
                return _pmanip->CheckEndEffectorCollision(ikparam);
            }
            return _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans));
        }
        bool CheckEndEffectorCollision(object otrans, PyCollisionReportPtr pReport) const
        {
            bool bCollision;
            IkParameterization ikparam;
            if( ExtractIkParameterization(otrans,ikparam) ) {
                bCollision = _pmanip->CheckEndEffectorCollision(ikparam,openravepy::GetCollisionReport(pReport));
            }
            else {
                bCollision = _pmanip->CheckEndEffectorCollision(ExtractTransform(otrans),openravepy::GetCollisionReport(pReport));
            }
            openravepy::UpdateCollisionReport(pReport,_pyenv);
            return bCollision;
        }
        bool CheckIndependentCollision() const
        {
            return _pmanip->CheckIndependentCollision();
        }
        bool CheckIndependentCollision(PyCollisionReportPtr pReport) const
        {
            bool bCollision = _pmanip->CheckIndependentCollision(openravepy::GetCollisionReport(pReport));
            openravepy::UpdateCollisionReport(pReport,_pyenv);
            return bCollision;
        }

        object CalculateJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        object CalculateRotationJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateRotationJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        object CalculateAngularVelocityJacobian()
        {
            std::vector<dReal> vjacobian;
            _pmanip->CalculateAngularVelocityJacobian(vjacobian);
            std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pmanip->GetArmIndices().size();
            return toPyArray(vjacobian,dims);
        }

        string GetStructureHash() const {
            return _pmanip->GetStructureHash();
        }
        string GetKinematicsStructureHash() const {
            return _pmanip->GetKinematicsStructureHash();
        }
        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetManipulator('%s')")%RaveGetEnvironmentId(_pmanip->GetRobot()->GetEnv())%_pmanip->GetRobot()->GetName()%_pmanip->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<manipulator:%s, parent=%s>")%_pmanip->GetName()%_pmanip->GetRobot()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyManipulator> p) {
            return !!p && _pmanip==p->_pmanip;
        }
        bool __ne__(boost::shared_ptr<PyManipulator> p) {
            return !p || _pmanip!=p->_pmanip;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_pmanip.get()));
        }
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
        PyAttachedSensor(RobotBase::AttachedSensorPtr pattached, PyEnvironmentBasePtr pyenv) : _pattached(pattached),_pyenv(pyenv) {
        }
        virtual ~PyAttachedSensor() {
        }

        object GetSensor() {
            return object(openravepy::toPySensor(_pattached->GetSensor(),_pyenv));
        }
        PyLinkPtr GetAttachingLink() const {
            return !_pattached->GetAttachingLink() ? PyLinkPtr() : PyLinkPtr(new PyLink(_pattached->GetAttachingLink(), _pyenv));
        }
        object GetRelativeTransform() const {
            return ReturnTransform(_pattached->GetRelativeTransform());
        }
        object GetTransform() const {
            return ReturnTransform(_pattached->GetTransform());
        }
        PyRobotBasePtr GetRobot() const {
            return _pattached->GetRobot() ? PyRobotBasePtr() : PyRobotBasePtr(new PyRobotBase(_pattached->GetRobot(), _pyenv));
        }
        object GetName() const {
            return ConvertStringToUnicode(_pattached->GetName());
        }

        object GetData()
        {
            return openravepy::toPySensorData(_pattached->GetSensor(),_pyenv);
        }

        void SetRelativeTransform(object transform) {
            _pattached->SetRelativeTransform(ExtractTransform(transform));
        }
        string GetStructureHash() const {
            return _pattached->GetStructureHash();
        }
        string __repr__() {
            return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s').GetSensor('%s')")%RaveGetEnvironmentId(_pattached->GetRobot()->GetEnv())%_pattached->GetRobot()->GetName()%_pattached->GetName());
        }
        string __str__() {
            return boost::str(boost::format("<attachedsensor:%s, parent=%s>")%_pattached->GetName()%_pattached->GetRobot()->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
        bool __eq__(boost::shared_ptr<PyAttachedSensor> p) {
            return !!p && _pattached==p->_pattached;
        }
        bool __ne__(boost::shared_ptr<PyAttachedSensor> p) {
            return !p || _pattached!=p->_pattached;
        }
        int __hash__() {
            return static_cast<int>(uintptr_t(_pattached.get()));
        }
    };

    class PyRobotStateSaver
    {
        PyEnvironmentBasePtr _pyenv;
        RobotBase::RobotStateSaver _state;
public:
        PyRobotStateSaver(PyRobotBasePtr pyrobot) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot()) {
        }
        PyRobotStateSaver(PyRobotBasePtr pyrobot, object options) : _pyenv(pyrobot->GetEnv()), _state(pyrobot->GetRobot(),pyGetIntFromPy(options)) {
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(probot) {
        }
        PyRobotStateSaver(RobotBasePtr probot, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(probot,pyGetIntFromPy(options)) {
        }
        virtual ~PyRobotStateSaver() {
        }

        object GetBody() const {
            return object(toPyRobot(RaveInterfaceCast<RobotBase>(_state.GetBody()),_pyenv));
        }

        void Restore(PyRobotBasePtr pyrobot=PyRobotBasePtr()) {
            _state.Restore(!pyrobot ? RobotBasePtr() : pyrobot->GetRobot());
        }

        void Release() {
            _state.Release();
        }

        std::string __str__() {
            KinBodyPtr pbody = _state.GetBody();
            if( !pbody ) {
                return "robot state empty";
            }
            return boost::str(boost::format("robot state for %s")%pbody->GetName());
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
        }
    };
    typedef boost::shared_ptr<PyRobotStateSaver> PyRobotStateSaverPtr;

    PyRobotBase(RobotBasePtr probot, PyEnvironmentBasePtr pyenv) : PyKinBody(probot,pyenv), _probot(probot) {
    }
    PyRobotBase(const PyRobotBase &r) : PyKinBody(r._probot,r._pyenv) {
        _probot = r._probot;
    }
    virtual ~PyRobotBase() {
    }

    object GetManipulators()
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            manips.append(_GetManipulator(*it));
        }
        return manips;
    }

    object GetManipulators(const string& manipname)
    {
        boost::python::list manips;
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                manips.append(_GetManipulator(*it));
            }
        }
        return manips;
    }
    PyManipulatorPtr GetManipulator(const string& manipname)
    {
        FOREACH(it, _probot->GetManipulators()) {
            if( (*it)->GetName() == manipname ) {
                return _GetManipulator(*it);
            }
        }
        return PyManipulatorPtr();
    }

    PyManipulatorPtr SetActiveManipulator(int index) {
        RAVELOG_WARN("SetActiveManipulator(int) is deprecated\n");
        _probot->SetActiveManipulator(index);
        return GetActiveManipulator();
    }
    PyManipulatorPtr SetActiveManipulator(const std::string& manipname) {
        _probot->SetActiveManipulator(manipname);
        return GetActiveManipulator();
    }
    PyManipulatorPtr SetActiveManipulator(PyManipulatorPtr pmanip) {
        _probot->SetActiveManipulator(pmanip->GetManipulator());
        return GetActiveManipulator();
    }
    PyManipulatorPtr GetActiveManipulator() {
        return _GetManipulator(_probot->GetActiveManipulator());
    }
    int GetActiveManipulatorIndex() const {
        RAVELOG_WARN("GetActiveManipulatorIndex is deprecated\n");
        return _probot->GetActiveManipulatorIndex();
    }
    PyManipulatorPtr AddManipulator(PyManipulatorInfoPtr pmanipinfo) {
        return _GetManipulator(_probot->AddManipulator(pmanipinfo->GetManipulatorInfo()));
    }
    void RemoveManipulator(PyManipulatorPtr pmanip) {
        _probot->RemoveManipulator(pmanip->GetManipulator());
    }

    object GetSensors()
    {
        RAVELOG_WARN("GetSensors is deprecated, please use GetAttachedSensors\n");
        return GetAttachedSensors();
    }

    object GetAttachedSensors()
    {
        boost::python::list sensors;
        FOREACH(itsensor, _probot->GetAttachedSensors()) {
            sensors.append(boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv)));
        }
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
            if( (*itsensor)->GetName() == sensorname ) {
                return boost::shared_ptr<PyAttachedSensor>(new PyAttachedSensor(*itsensor,_pyenv));
            }
        }
        return boost::shared_ptr<PyAttachedSensor>();
    }

    object GetController() const {
        CHECK_POINTER(_probot);
        return object(openravepy::toPyController(_probot->GetController(),_pyenv));
    }

    bool SetController(PyControllerBasePtr pController, const string& args) {
        RAVELOG_WARN("RobotBase::SetController(PyControllerBasePtr,args) is deprecated\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    bool SetController(PyControllerBasePtr pController, object odofindices, int nControlTransformation) {
        CHECK_POINTER(pController);
        vector<int> dofindices = ExtractArray<int>(odofindices);
        return _probot->SetController(openravepy::GetController(pController),dofindices,nControlTransformation);
    }

    bool SetController(PyControllerBasePtr pController) {
        RAVELOG_VERBOSE("RobotBase::SetController(PyControllerBasePtr) will control all DOFs and transformation\n");
        std::vector<int> dofindices;
        for(int i = 0; i < _probot->GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return _probot->SetController(openravepy::GetController(pController),dofindices,1);
    }

    void SetActiveDOFs(object dofindices) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices));
    }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask);
    }
    void SetActiveDOFs(object dofindices, int nAffineDOsBitmask, object rotationaxis) {
        _probot->SetActiveDOFs(ExtractArray<int>(dofindices), nAffineDOsBitmask, ExtractVector3(rotationaxis));
    }

    int GetActiveDOF() const {
        return _probot->GetActiveDOF();
    }
    int GetAffineDOF() const {
        return _probot->GetAffineDOF();
    }
    int GetAffineDOFIndex(DOFAffine dof) const {
        return _probot->GetAffineDOFIndex(dof);
    }

    object GetAffineRotationAxis() const {
        return toPyVector3(_probot->GetAffineRotationAxis());
    }
    void SetAffineTranslationLimits(object lower, object upper) {
        return _probot->SetAffineTranslationLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationAxisLimits(object lower, object upper) {
        return _probot->SetAffineRotationAxisLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotation3DLimits(object lower, object upper) {
        return _probot->SetAffineRotation3DLimits(ExtractVector3(lower),ExtractVector3(upper));
    }
    void SetAffineRotationQuatLimits(object quatangle) {
        return _probot->SetAffineRotationQuatLimits(ExtractVector4(quatangle));
    }
    void SetAffineTranslationMaxVels(object vels) {
        _probot->SetAffineTranslationMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationAxisMaxVels(object vels) {
        _probot->SetAffineRotationAxisMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotation3DMaxVels(object vels) {
        _probot->SetAffineRotation3DMaxVels(ExtractVector3(vels));
    }
    void SetAffineRotationQuatMaxVels(dReal vels) {
        _probot->SetAffineRotationQuatMaxVels(vels);
    }
    void SetAffineTranslationResolution(object resolution) {
        _probot->SetAffineTranslationResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationAxisResolution(object resolution) {
        _probot->SetAffineRotationAxisResolution(ExtractVector3(resolution));
    }
    void SetAffineRotation3DResolution(object resolution) {
        _probot->SetAffineRotation3DResolution(ExtractVector3(resolution));
    }
    void SetAffineRotationQuatResolution(dReal resolution) {
        _probot->SetAffineRotationQuatResolution(resolution);
    }
    void SetAffineTranslationWeights(object weights) {
        _probot->SetAffineTranslationWeights(ExtractVector3(weights));
    }
    void SetAffineRotationAxisWeights(object weights) {
        _probot->SetAffineRotationAxisWeights(ExtractVector4(weights));
    }
    void SetAffineRotation3DWeights(object weights) {
        _probot->SetAffineRotation3DWeights(ExtractVector3(weights));
    }
    void SetAffineRotationQuatWeights(dReal weights) {
        _probot->SetAffineRotationQuatWeights(weights);
    }

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
        return toPyVector4(_probot->GetAffineRotationQuatLimits());
    }
    object GetAffineTranslationMaxVels() const {
        return toPyVector3(_probot->GetAffineTranslationMaxVels());
    }
    object GetAffineRotationAxisMaxVels() const {
        return toPyVector3(_probot->GetAffineRotationAxisMaxVels());
    }
    object GetAffineRotation3DMaxVels() const {
        return toPyVector3(_probot->GetAffineRotation3DMaxVels());
    }
    dReal GetAffineRotationQuatMaxVels() const {
        return _probot->GetAffineRotationQuatMaxVels();
    }
    object GetAffineTranslationResolution() const {
        return toPyVector3(_probot->GetAffineTranslationResolution());
    }
    object GetAffineRotationAxisResolution() const {
        return toPyVector4(_probot->GetAffineRotationAxisResolution());
    }
    object GetAffineRotation3DResolution() const {
        return toPyVector3(_probot->GetAffineRotation3DResolution());
    }
    dReal GetAffineRotationQuatResolution() const {
        return _probot->GetAffineRotationQuatResolution();
    }
    object GetAffineTranslationWeights() const {
        return toPyVector3(_probot->GetAffineTranslationWeights());
    }
    object GetAffineRotationAxisWeights() const {
        return toPyVector4(_probot->GetAffineRotationAxisWeights());
    }
    object GetAffineRotation3DWeights() const {
        return toPyVector3(_probot->GetAffineRotation3DWeights());
    }
    dReal GetAffineRotationQuatWeights() const {
        return _probot->GetAffineRotationQuatWeights();
    }

    void SetActiveDOFValues(object values, uint32_t checklimits=1) const
    {
        vector<dReal> vvalues = ExtractArray<dReal>(values);
        if( vvalues.size() > 0 ) {
            _probot->SetActiveDOFValues(vvalues,checklimits);
        }
    }
    object GetActiveDOFValues() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFValues(values);
        return toPyArray(values);
    }

    object GetActiveDOFWeights() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> weights;
        _probot->GetActiveDOFWeights(weights);
        return toPyArray(weights);
    }

    void SetActiveDOFVelocities(object velocities)
    {
        _probot->SetActiveDOFVelocities(ExtractArray<dReal>(velocities));
    }
    object GetActiveDOFVelocities() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFVelocities(values);
        return toPyArray(values);
    }

    object GetActiveDOFLimits() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> lower, upper;
        _probot->GetActiveDOFLimits(lower,upper);
        return boost::python::make_tuple(toPyArray(lower),toPyArray(upper));
    }

    object GetActiveDOFMaxVel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxVel(values);
        return toPyArray(values);
    }

    object GetActiveDOFMaxAccel() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFMaxAccel(values);
        return toPyArray(values);
    }

    object GetActiveDOFResolutions() const
    {
        if( _probot->GetActiveDOF() == 0 ) {
            return numeric::array(boost::python::list());
        }
        vector<dReal> values;
        _probot->GetActiveDOFResolutions(values);
        return toPyArray(values);
    }

    object GetActiveConfigurationSpecification(const std::string& interpolation="") const {
        return object(openravepy::toPyConfigurationSpecification(_probot->GetActiveConfigurationSpecification(interpolation)));
    }

    object GetActiveJointIndices() {
        RAVELOG_WARN("GetActiveJointIndices deprecated. Use GetActiveDOFIndices\n"); return toPyArray(_probot->GetActiveDOFIndices());
    }
    object GetActiveDOFIndices() {
        return toPyArray(_probot->GetActiveDOFIndices());
    }

    object SubtractActiveDOFValues(object ovalues0, object ovalues1)
    {
        vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
        vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
        BOOST_ASSERT((int)values0.size() == GetActiveDOF() );
        BOOST_ASSERT((int)values1.size() == GetActiveDOF() );
        _probot->SubtractActiveDOFValues(values0,values1);
        return toPyArray(values0);
    }

    object CalculateActiveJacobian(int index, object offset) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveJacobian(index,ExtractVector3(offset),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveRotationJacobian(int index, object q) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveRotationJacobian(index,ExtractVector4(q),vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    object CalculateActiveAngularVelocityJacobian(int index) const
    {
        std::vector<dReal> vjacobian;
        _probot->CalculateActiveAngularVelocityJacobian(index,vjacobian);
        std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _probot->GetActiveDOF();
        return toPyArray(vjacobian,dims);
    }

    bool Grab(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); return _probot->Grab(pbody->GetBody());
    }
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
    void Release(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); _probot->Release(pbody->GetBody());
    }
    void ReleaseAllGrabbed() {
        _probot->ReleaseAllGrabbed();
    }
    void RegrabAll() {
        _probot->RegrabAll();
    }
    PyLinkPtr IsGrabbing(PyKinBodyPtr pbody) const {
        CHECK_POINTER(pbody);
        KinBody::LinkPtr plink = _probot->IsGrabbing(pbody->GetBody());
        return !plink ? PyLinkPtr() : PyLinkPtr(new PyLink(plink,_pyenv));
    }

    object GetGrabbed() const
    {
        boost::python::list bodies;
        std::vector<KinBodyPtr> vbodies;
        _probot->GetGrabbed(vbodies);
        FOREACH(itbody, vbodies) {
            bodies.append(PyKinBodyPtr(new PyKinBody(*itbody,_pyenv)));
        }
        return bodies;
    }

    bool WaitForController(float ftimeout)
    {
        ControllerBasePtr pcontroller = _probot->GetController();
        if( !pcontroller ) {
            return false;
        }
        if( pcontroller->IsDone() ) {
            return true;
        }
        bool bSuccess = true;
        Py_BEGIN_ALLOW_THREADS;

        try {
            uint64_t starttime = GetMicroTime();
            uint64_t deltatime = (uint64_t)(ftimeout*1000000.0);
            while( !pcontroller->IsDone() ) {
                Sleep(1);
                if(( deltatime > 0) &&( (GetMicroTime()-starttime)>deltatime) ) {
                    bSuccess = false;
                    break;
                }
            }
        }
        catch(...) {
            RAVELOG_ERROR("exception raised inside WaitForController:\n");
            PyErr_Print();
            bSuccess = false;
        }

        Py_END_ALLOW_THREADS;
        return bSuccess;
    }

    string GetRobotStructureHash() const {
        return _probot->GetRobotStructureHash();
    }

    PyStateRestoreContextBase* CreateRobotStateSaver(object options=object()) {
        PyRobotStateSaverPtr saver;
        if( options == object() ) {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv));
        }
        else {
            saver.reset(new PyRobotStateSaver(_probot,_pyenv,options));
        }
        return new PyStateRestoreContext<PyRobotStateSaverPtr, PyRobotBasePtr>(saver);
    }

    virtual string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d).GetRobot('%s')")%RaveGetEnvironmentId(_probot->GetEnv())%_probot->GetName());
    }
    virtual string __str__() {
        return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_probot->GetInterfaceType())%_probot->GetXMLId()%_probot->GetName()%_probot->GetRobotStructureHash());
    }
    virtual object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }
    virtual void __enter__()
    {
        // necessary to lock physics to prevent multiple threads from interfering
        if( _listStateSavers.size() == 0 ) {
            openravepy::LockEnvironment(_pyenv);
        }
        _listStateSavers.push_back(boost::shared_ptr<void>(new RobotBase::RobotStateSaver(_probot)));
    }
};

object PyKinBody::PyLink::GetParent() const
{
    KinBodyPtr parent = _plink->GetParent();
    if( parent->IsRobot() ) {
        return object(PyRobotBasePtr(new PyRobotBase(RaveInterfaceCast<RobotBase>(_plink->GetParent()),_pyenv)));
    }
    else {
        return object(PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)));
    }
}

object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv)
{
    return object(PyKinBody::PyLinkPtr(new PyKinBody::PyLink(plink,pyenv)));
}

object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv)
{
    return object(PyKinBody::PyJointPtr(new PyKinBody::PyJoint(pjoint,pyenv)));
}

KinBody::LinkPtr GetKinBodyLink(object o)
{
    extract<PyKinBody::PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyKinBody::PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkPtr();
}

KinBody::LinkConstPtr GetKinBodyLinkConst(object o)
{
    extract<PyKinBody::PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyKinBody::PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkConstPtr();
}

KinBody::JointPtr GetKinBodyJoint(object o)
{
    extract<PyKinBody::PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyKinBody::PyJointPtr)pyjoint)->GetJoint();
    }
    return KinBody::JointPtr();
}

std::string reprPyKinBodyJoint(object o)
{
    extract<PyKinBody::PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyKinBody::PyJointPtr)pyjoint)->__repr__();
    }
    return std::string();
}

std::string strPyKinBodyJoint(object o)
{
    extract<PyKinBody::PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyKinBody::PyJointPtr)pyjoint)->__str__();
    }
    return std::string();
}

KinBodyPtr GetKinBody(object o)
{
    extract<PyKinBodyPtr> pykinbody(o);
    if( pykinbody.check() ) {
        return ((PyKinBodyPtr)pykinbody)->GetBody();
    }
    return KinBodyPtr();
}

KinBodyPtr GetKinBody(PyKinBodyPtr pykinbody)
{
    return !pykinbody ? KinBodyPtr() : pykinbody->GetBody();
}

PyEnvironmentBasePtr toPyEnvironment(PyKinBodyPtr pykinbody)
{
    return pykinbody->GetEnv();
}

PyInterfaceBasePtr toPyKinBody(KinBodyPtr pkinbody, PyEnvironmentBasePtr pyenv)
{
    return !pkinbody ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyKinBody(pkinbody,pyenv));
}

RobotBasePtr GetRobot(PyRobotBasePtr pyrobot)
{
    return !pyrobot ? RobotBasePtr() : pyrobot->GetRobot();
}

PyInterfaceBasePtr toPyRobot(RobotBasePtr probot, PyEnvironmentBasePtr pyenv)
{
    return !probot ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyRobotBase(probot,pyenv));
}

RobotBase::ManipulatorPtr GetRobotManipulator(object o)
{
    extract<PyRobotBase::PyManipulatorPtr> pymanipulator(o);
    if( pymanipulator.check() ) {
        return ((PyRobotBase::PyManipulatorPtr)pymanipulator)->GetManipulator();
    }
    return RobotBase::ManipulatorPtr();
}

object toPyRobotManipulator(RobotBase::ManipulatorPtr pmanip, PyEnvironmentBasePtr pyenv)
{
    return !pmanip ? object() : object(PyRobotBase::PyManipulatorPtr(new PyRobotBase::PyManipulator(pmanip,pyenv)));
}

PyRobotBasePtr RaveCreateRobot(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    RobotBasePtr p = OpenRAVE::RaveCreateRobot(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyRobotBasePtr();
    }
    return PyRobotBasePtr(new PyRobotBase(p,pyenv));
}

PyKinBodyPtr RaveCreateKinBody(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    KinBodyPtr p = OpenRAVE::RaveCreateKinBody(openravepy::GetEnvironment(pyenv), name);
    if( !p ) {
        return PyKinBodyPtr();
    }
    return PyKinBodyPtr(new PyKinBody(p,pyenv));
}

class GeometryInfo_pickle_suite : public pickle_suite
{
public:
    static tuple getstate(const PyKinBody::PyGeometryInfo& r)
    {
        return boost::python::make_tuple(r._t, r._vGeomData, r._vDiffuseColor, r._vAmbientColor, r._meshcollision, r._type, r._filenamerender, r._filenamecollision, r._vRenderScale, r._vCollisionScale, r._fTransparency, r._bVisible, r._bModifiable);
    }
    static void setstate(PyKinBody::PyGeometryInfo& r, boost::python::tuple state) {
        r._t = state[0];
        r._vGeomData = state[1];
        r._vDiffuseColor = state[2];
        r._vAmbientColor = state[3];
        r._meshcollision = state[4];
        r._type = boost::python::extract<GeometryType>(state[5]);
        r._filenamerender = boost::python::extract<std::string>(state[6]);
        r._filenamecollision = boost::python::extract<std::string>(state[7]);
        r._vRenderScale = state[8];
        r._vCollisionScale = state[9];
        r._fTransparency = boost::python::extract<float>(state[10]);
        r._bVisible = boost::python::extract<bool>(state[11]);
        r._bModifiable = boost::python::extract<bool>(state[12]);
    }
};

class LinkInfo_pickle_suite : public pickle_suite
{
public:
    static tuple getstate(const PyKinBody::PyLinkInfo& r)
    {
        return boost::python::make_tuple(r._vgeometryinfos, r._name, r._t, r._tMassFrame, r._mass, r._vinertiamoments, r._mapFloatParameters, r._mapIntParameters, r._vForcedAdjacentLinks, r._bStatic, r._bIsEnabled);
    }
    static void setstate(PyKinBody::PyLinkInfo& r, boost::python::tuple state) {
        r._vgeometryinfos = boost::python::list(state[0]);
        r._name = boost::python::extract<std::string>(state[1]);
        r._t = state[2];
        r._tMassFrame = state[3];
        r._mass = boost::python::extract<dReal>(state[4]);
        r._mapFloatParameters = dict(state[5]);
        r._mapIntParameters = dict(state[6]);
        r._vForcedAdjacentLinks = dict(state[7]);
        r._bStatic = boost::python::extract<bool>(state[8]);
        r._bIsEnabled = boost::python::extract<bool>(state[9]);
    }
};

class JointInfo_pickle_suite : public pickle_suite
{
public:
    static tuple getstate(const PyKinBody::PyJointInfo& r)
    {
        return boost::python::make_tuple(boost::python::make_tuple(r._type, r._name, r._linkname0, r._linkname1, r._vanchor, r._vaxes, r._vcurrentvalues), boost::python::make_tuple(r._vresolution, r._vmaxvel, r._vhardmaxvel, r._vmaxaccel, r._vmaxtorque, r._vweights, r._voffsets, r._vlowerlimit, r._vupperlimit), boost::python::make_tuple(r._trajfollow, r._vmimic, r._mapFloatParameters, r._mapIntParameters, r._bIsCircular));
    }
    static void setstate(PyKinBody::PyJointInfo& r, boost::python::tuple state) {
        r._type = boost::python::extract<KinBody::JointType>(state[0][0]);
        r._name = boost::python::extract<std::string>(state[0][1]);
        r._linkname0 = boost::python::extract<std::string>(state[0][2]);
        r._linkname1 = boost::python::extract<std::string>(state[0][3]);
        r._vanchor = state[0][4];
        r._vaxes = state[0][5];
        r._vcurrentvalues = state[0][6];
        r._vresolution = state[1][0];
        r._vmaxvel = state[1][1];
        r._vhardmaxvel = state[1][2];
        r._vmaxaccel = state[1][3];
        r._vmaxtorque = state[1][4];
        r._vweights = state[1][5];
        r._voffsets = state[1][6];
        r._vlowerlimit = state[1][7];
        r._vupperlimit = state[1][8];
        r._trajfollow = state[2][0];
        r._vmimic = boost::python::list(state[2][1]);
        r._mapFloatParameters = boost::python::dict(state[2][2]);
        r._mapIntParameters = boost::python::dict(state[2][3]);
        r._bIsCircular = state[2][4];
    }
};

class ManipulatorInfo_pickle_suite : public pickle_suite
{
public:
    static tuple getstate(const PyRobotBase::PyManipulatorInfo& r)
    {
        return boost::python::make_tuple(r._name, r._sBaseLinkName, r._sEffectorLinkName, r._tLocalTool, r._vClosingDirection, r._vdirection, r._sIkSolverXMLId, r._vGripperJointNames);
    }
    static void setstate(PyRobotBase::PyManipulatorInfo& r, boost::python::tuple state) {
        r._name = boost::python::extract<std::string>(state[0]);
        r._sBaseLinkName = boost::python::extract<std::string>(state[1]);
        r._sEffectorLinkName = boost::python::extract<std::string>(state[2]);
        r._tLocalTool = state[4];
        r._vClosingDirection = state[5];
        r._vdirection = state[6];
        r._sIkSolverXMLId = boost::python::extract<std::string>(state[7]);
        r._vGripperJointNames = state[8];
    }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(IsMimic_overloads, IsMimic, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicEquation_overloads, GetMimicEquation, 0, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicDOFIndices_overloads, GetMimicDOFIndices, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetChain_overloads, GetChain, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetActiveConfigurationSpecification_overloads, GetActiveConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetConfigurationSpecification_overloads, GetConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetConfigurationSpecificationIndices_overloads, GetConfigurationSpecificationIndices, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetAxis_overloads, GetAxis, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetWrapOffset_overloads, GetWrapOffset, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetWrapOffset_overloads, SetWrapOffset, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxVel_overloads, GetMaxVel, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxAccel_overloads, GetMaxAccel, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMaxTorque_overloads, GetMaxTorque, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetLinkTransformations_overloads, GetLinkTransformations, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetLinkTransformations_overloads, SetLinkTransformations, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIkParameterization_overloads, GetIkParameterization, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolution_overloads, FindIKSolution, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionFree_overloads, FindIKSolution, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutions_overloads, FindIKSolutions, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(FindIKSolutionsFree_overloads, FindIKSolutions, 3, 5)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetArmConfigurationSpecification_overloads, GetArmConfigurationSpecification, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianTranslation_overloads, ComputeJacobianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianAxisAngle_overloads, ComputeJacobianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianTranslation_overloads, ComputeHessianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianAxisAngle_overloads, ComputeHessianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeInverseDynamics_overloads, ComputeInverseDynamics, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateKinBodyStateSaver_overloads, CreateKinBodyStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateRobotStateSaver_overloads, CreateRobotStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetConfigurationValues_overloads, SetConfigurationValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetActiveDOFValues_overloads, SetActiveDOFValues, 1,2)

void init_openravepy_kinbody()
{
    object dofaffine = enum_<DOFAffine>("DOFAffine" DOXY_ENUM(DOFAffine))
                       .value("NoTransform",DOF_NoTransform)
                       .value("X",DOF_X)
                       .value("Y",DOF_Y)
                       .value("Z",DOF_Z)
                       .value("RotationAxis",DOF_RotationAxis)
                       .value("Rotation3D",DOF_Rotation3D)
                       .value("RotationQuat",DOF_RotationQuat)
                       .value("RotationMask",DOF_RotationMask)
                       .value("Transform",DOF_Transform)
    ;

    class_<PyStateRestoreContextBase, boost::noncopyable>("StateRestoreContext",no_init)
    .def("__enter__",&PyStateRestoreContextBase::__enter__,"returns the object storing the state")
    .def("__exit__",&PyStateRestoreContextBase::__exit__,"restores the state held in the object")
    .def("GetBody",&PyStateRestoreContextBase::GetBody,DOXY_FN(KinBody::KinBodyStateSaver, GetBody))
    .def("Restore",&PyStateRestoreContextBase::Restore,Restore_overloads(args("body"), DOXY_FN(KinBody::KinBodyStateSaver, Restore)))
    .def("Release",&PyStateRestoreContextBase::Release,DOXY_FN(KinBody::KinBodyStateSaver, Release))
    .def("Close",&PyStateRestoreContextBase::Close,DOXY_FN(KinBody::KinBodyStateSaver, Close))
    .def("__str__",&PyStateRestoreContextBase::__str__)
    .def("__unicode__",&PyStateRestoreContextBase::__unicode__)
    ;
    object geometrytype = enum_<GeometryType>("GeometryType" DOXY_ENUM(GeometryType))
                          .value("None",GT_None)
                          .value("Box",GT_Box)
                          .value("Sphere",GT_Sphere)
                          .value("Cylinder",GT_Cylinder)
                          .value("Trimesh",GT_TriMesh)
    ;
    {
        bool (PyKinBody::*pkinbodyself)() = &PyKinBody::CheckSelfCollision;
        bool (PyKinBody::*pkinbodyselfr)(PyCollisionReportPtr) = &PyKinBody::CheckSelfCollision;
        void (PyKinBody::*psetdofvalues1)(object) = &PyKinBody::SetDOFValues;
        void (PyKinBody::*psetdofvalues2)(object,object) = &PyKinBody::SetDOFValues;
        void (PyKinBody::*psetdofvalues3)(object,object,uint32_t) = &PyKinBody::SetDOFValues;
        object (PyKinBody::*getdofvalues1)() const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdofvalues2)(object) const = &PyKinBody::GetDOFValues;
        object (PyKinBody::*getdofvelocities1)() const = &PyKinBody::GetDOFVelocities;
        object (PyKinBody::*getdofvelocities2)(object) const = &PyKinBody::GetDOFVelocities;
        object (PyKinBody::*getdoflimits1)() const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getdoflimits2)(object) const = &PyKinBody::GetDOFLimits;
        object (PyKinBody::*getdofweights1)() const = &PyKinBody::GetDOFWeights;
        object (PyKinBody::*getdofweights2)(object) const = &PyKinBody::GetDOFWeights;
        object (PyKinBody::*getdofresolutions1)() const = &PyKinBody::GetDOFResolutions;
        object (PyKinBody::*getdofresolutions2)(object) const = &PyKinBody::GetDOFResolutions;
        object (PyKinBody::*getdofvelocitylimits1)() const = &PyKinBody::GetDOFVelocityLimits;
        object (PyKinBody::*getdofvelocitylimits2)(object) const = &PyKinBody::GetDOFVelocityLimits;
        object (PyKinBody::*getdofaccelerationlimits1)() const = &PyKinBody::GetDOFAccelerationLimits;
        object (PyKinBody::*getdofaccelerationlimits2)(object) const = &PyKinBody::GetDOFAccelerationLimits;
        object (PyKinBody::*getdoftorquelimits1)() const = &PyKinBody::GetDOFTorqueLimits;
        object (PyKinBody::*getdoftorquelimits2)(object) const = &PyKinBody::GetDOFTorqueLimits;
        object (PyKinBody::*getlinks1)() const = &PyKinBody::GetLinks;
        object (PyKinBody::*getlinks2)(object) const = &PyKinBody::GetLinks;
        object (PyKinBody::*getjoints1)() const = &PyKinBody::GetJoints;
        object (PyKinBody::*getjoints2)(object) const = &PyKinBody::GetJoints;
        void (PyKinBody::*setdofvelocities1)(object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities2)(object,object,object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities3)(object,uint32_t,object) = &PyKinBody::SetDOFVelocities;
        void (PyKinBody::*setdofvelocities4)(object,object,object,uint32_t) = &PyKinBody::SetDOFVelocities;
        object (PyKinBody::*GetNonAdjacentLinks1)() const = &PyKinBody::GetNonAdjacentLinks;
        object (PyKinBody::*GetNonAdjacentLinks2)(int) const = &PyKinBody::GetNonAdjacentLinks;
        std::string sInitFromBoxesDoc = std::string(DOXY_FN(KinBody,InitFromBoxes "const std::vector< AABB; bool")) + std::string("\nboxes is a Nx6 array, first 3 columsn are position, last 3 are extents");
        std::string sGetChainDoc = std::string(DOXY_FN(KinBody,GetChain)) + std::string("If returnjoints is false will return a list of links, otherwise will return a list of links (default is true)");
        std::string sComputeInverseDynamicsDoc = std::string(":param returncomponents: If True will return three N-element arrays that represents the torque contributions to M, C, and G.\n\n:param externalforcetorque: A dictionary of link indices and a 6-element array of forces/torques in that order.\n\n") + std::string(DOXY_FN(KinBody, ComputeInverseDynamics));
        scope kinbody = class_<PyKinBody, boost::shared_ptr<PyKinBody>, bases<PyInterfaceBase> >("KinBody", DOXY_CLASS(KinBody), no_init)
                        .def("InitFromBoxes",&PyKinBody::InitFromBoxes,args("boxes","draw"), sInitFromBoxesDoc.c_str())
                        .def("InitFromSpheres",&PyKinBody::InitFromSpheres,args("spherex","draw"), DOXY_FN(KinBody,InitFromSpheres))
                        .def("InitFromTrimesh",&PyKinBody::InitFromTrimesh,args("trimesh","draw"), DOXY_FN(KinBody,InitFromTrimesh))
                        .def("InitFromGeometries",&PyKinBody::InitFromGeometries,args("geometries"), DOXY_FN(KinBody,InitFromGeometries))
                        .def("Init",&PyKinBody::Init,args("linkinfos","jointinfos"), DOXY_FN(KinBody,Init))
                        .def("SetName", &PyKinBody::SetName,args("name"),DOXY_FN(KinBody,SetName))
                        .def("GetName",&PyKinBody::GetName,DOXY_FN(KinBody,GetName))
                        .def("GetDOF",&PyKinBody::GetDOF,DOXY_FN(KinBody,GetDOF))
                        .def("GetDOFValues",getdofvalues1,DOXY_FN(KinBody,GetDOFValues))
                        .def("GetDOFValues",getdofvalues2,args("indices"),DOXY_FN(KinBody,GetDOFValues))
                        .def("GetDOFVelocities",getdofvelocities1, DOXY_FN(KinBody,GetDOFVelocities))
                        .def("GetDOFVelocities",getdofvelocities2, args("indices"), DOXY_FN(KinBody,GetDOFVelocities))
                        .def("GetDOFLimits",getdoflimits1, DOXY_FN(KinBody,GetDOFLimits))
                        .def("GetDOFLimits",getdoflimits2, args("indices"),DOXY_FN(KinBody,GetDOFLimits))
                        .def("GetDOFVelocityLimits",getdofvelocitylimits1, DOXY_FN(KinBody,GetDOFVelocityLimits))
                        .def("GetDOFVelocityLimits",getdofvelocitylimits2, args("indices"),DOXY_FN(KinBody,GetDOFVelocityLimits))
                        .def("GetDOFAccelerationLimits",getdofaccelerationlimits1, DOXY_FN(KinBody,GetDOFAccelerationLimits))
                        .def("GetDOFAccelerationLimits",getdofaccelerationlimits2, args("indices"),DOXY_FN(KinBody,GetDOFAccelerationLimits))
                        .def("GetDOFTorqueLimits",getdoftorquelimits1, DOXY_FN(KinBody,GetDOFTorqueLimits))
                        .def("GetDOFTorqueLimits",getdoftorquelimits2, args("indices"),DOXY_FN(KinBody,GetDOFTorqueLimits))
                        .def("GetDOFMaxVel",&PyKinBody::GetDOFMaxVel, DOXY_FN(KinBody,GetDOFMaxVel))
                        .def("GetDOFMaxTorque",&PyKinBody::GetDOFMaxTorque, DOXY_FN(KinBody,GetDOFMaxTorque))
                        .def("GetDOFMaxAccel",&PyKinBody::GetDOFMaxAccel, DOXY_FN(KinBody,GetDOFMaxAccel))
                        .def("GetDOFWeights",getdofweights1, DOXY_FN(KinBody,GetDOFWeights))
                        .def("GetDOFWeights",getdofweights2, DOXY_FN(KinBody,GetDOFWeights))
                        .def("SetDOFWeights",&PyKinBody::SetDOFWeights, args("weights"), DOXY_FN(KinBody,SetDOFWeights))
                        .def("SetDOFLimits",&PyKinBody::SetDOFLimits, args("lower","upper"), DOXY_FN(KinBody,SetDOFLimits))
                        .def("SetDOFVelocityLimits",&PyKinBody::SetDOFVelocityLimits, args("limits"), DOXY_FN(KinBody,SetDOFVelocityLimits))
                        .def("SetDOFAccelerationLimits",&PyKinBody::SetDOFAccelerationLimits, args("limits"), DOXY_FN(KinBody,SetDOFAccelerationLimits))
                        .def("SetDOFTorqueLimits",&PyKinBody::SetDOFTorqueLimits, args("limits"), DOXY_FN(KinBody,SetDOFTorqueLimits))
                        .def("GetDOFResolutions",getdofresolutions1, DOXY_FN(KinBody,GetDOFResolutions))
                        .def("GetDOFResolutions",getdofresolutions2, DOXY_FN(KinBody,GetDOFResolutions))
                        .def("GetLinks",getlinks1, DOXY_FN(KinBody,GetLinks))
                        .def("GetLinks",getlinks2, args("indices"), DOXY_FN(KinBody,GetLinks))
                        .def("GetLink",&PyKinBody::GetLink,args("name"), DOXY_FN(KinBody,GetLink))
                        .def("GetJoints",getjoints1, DOXY_FN(KinBody,GetJoints))
                        .def("GetJoints",getjoints2, args("indices"), DOXY_FN(KinBody,GetJoints))
                        .def("GetPassiveJoints",&PyKinBody::GetPassiveJoints, DOXY_FN(KinBody,GetPassiveJoints))
                        .def("GetDependencyOrderedJoints",&PyKinBody::GetDependencyOrderedJoints, DOXY_FN(KinBody,GetDependencyOrderedJoints))
                        .def("GetClosedLoops",&PyKinBody::GetClosedLoops,DOXY_FN(KinBody,GetClosedLoops))
                        .def("GetRigidlyAttachedLinks",&PyKinBody::GetRigidlyAttachedLinks,args("linkindex"), DOXY_FN(KinBody,GetRigidlyAttachedLinks))
                        .def("GetChain",&PyKinBody::GetChain,GetChain_overloads(args("linkindex1","linkindex2","returnjoints"), sGetChainDoc.c_str()))
                        .def("IsDOFInChain",&PyKinBody::IsDOFInChain,args("linkindex1","linkindex2","dofindex"), DOXY_FN(KinBody,IsDOFInChain))
                        .def("GetJointIndex",&PyKinBody::GetJointIndex,args("name"), DOXY_FN(KinBody,GetJointIndex))
                        .def("GetJoint",&PyKinBody::GetJoint,args("name"), DOXY_FN(KinBody,GetJoint))
                        .def("GetJointFromDOFIndex",&PyKinBody::GetJointFromDOFIndex,args("dofindex"), DOXY_FN(KinBody,GetJointFromDOFIndex))
                        .def("GetTransform",&PyKinBody::GetTransform, DOXY_FN(KinBody,GetTransform))
                        .def("GetLinkTransformations",&PyKinBody::GetLinkTransformations, GetLinkTransformations_overloads(args("returndofbranches"), DOXY_FN(KinBody,GetLinkTransformations)))
                        .def("GetBodyTransformations",&PyKinBody::GetLinkTransformations, DOXY_FN(KinBody,GetLinkTransformations))
                        .def("SetLinkTransformations",&PyKinBody::SetLinkTransformations,SetLinkTransformations_overloads(args("transforms","dofbranches"), DOXY_FN(KinBody,SetLinkTransformations)))
                        .def("SetBodyTransformations",&PyKinBody::SetLinkTransformations,args("transforms"), DOXY_FN(KinBody,SetLinkTransformations))
                        .def("SetLinkVelocities",&PyKinBody::SetLinkVelocities,args("velocities"), DOXY_FN(KinBody,SetLinkVelocities))
                        .def("SetVelocity",&PyKinBody::SetVelocity, args("linear","angular"), DOXY_FN(KinBody,SetVelocity "const Vector; const Vector"))
                        .def("SetDOFVelocities",setdofvelocities1, args("dofvelocities"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; uint32_t"))
                        .def("SetDOFVelocities",setdofvelocities2, args("dofvelocities","linear","angular"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; const Vector; const Vector; uint32_t"))
                        .def("SetDOFVelocities",setdofvelocities3, args("dofvelocities","checklimits","indices"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFVelocities",setdofvelocities4, args("dofvelocities","linear","angular","checklimits"), DOXY_FN(KinBody,SetDOFVelocities "const std::vector; const Vector; const Vector; uint32_t"))
                        .def("GetLinkVelocities",&PyKinBody::GetLinkVelocities, DOXY_FN(KinBody,GetLinkVelocities))
                        .def("GetLinkAccelerations",&PyKinBody::GetLinkAccelerations, DOXY_FN(KinBody,GetLinkAccelerations))
                        .def("ComputeAABB",&PyKinBody::ComputeAABB, DOXY_FN(KinBody,ComputeAABB))
                        .def("Enable",&PyKinBody::Enable,args("enable"), DOXY_FN(KinBody,Enable))
                        .def("IsEnabled",&PyKinBody::IsEnabled, DOXY_FN(KinBody,IsEnabled))
                        .def("SetVisible",&PyKinBody::SetVisible,args("visible"), DOXY_FN(KinBody,SetVisible))
                        .def("IsVisible",&PyKinBody::IsVisible, DOXY_FN(KinBody,IsVisible))
                        .def("SetTransform",&PyKinBody::SetTransform,args("transform"), DOXY_FN(KinBody,SetTransform))
                        .def("SetJointValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetJointValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues1,args("values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues2,args("values","dofindices"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SetDOFValues",psetdofvalues3,args("values","dofindices","checklimits"), DOXY_FN(KinBody,SetDOFValues "const std::vector; uint32_t; const std::vector"))
                        .def("SubtractDOFValues",&PyKinBody::SubtractDOFValues,args("values0","values1"), DOXY_FN(KinBody,SubtractDOFValues))
                        .def("SetDOFTorques",&PyKinBody::SetDOFTorques,args("torques","add"), DOXY_FN(KinBody,SetDOFTorques))
                        .def("SetJointTorques",&PyKinBody::SetDOFTorques,args("torques","add"), DOXY_FN(KinBody,SetDOFTorques))
                        .def("SetTransformWithJointValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; const Transform; uint32_t"))
                        .def("SetTransformWithDOFValues",&PyKinBody::SetTransformWithDOFValues,args("transform","values"), DOXY_FN(KinBody,SetDOFValues "const std::vector; const Transform; uint32_t"))
                        .def("ComputeJacobianTranslation",&PyKinBody::ComputeJacobianTranslation,ComputeJacobianTranslation_overloads(args("linkindex","position","indices"), DOXY_FN(KinBody,ComputeJacobianTranslation)))
                        .def("ComputeJacobianAxisAngle",&PyKinBody::ComputeJacobianAxisAngle,ComputeJacobianAxisAngle_overloads(args("linkindex","indices"), DOXY_FN(KinBody,ComputeJacobianAxisAngle)))
                        .def("CalculateJacobian",&PyKinBody::CalculateJacobian,args("linkindex","position"), DOXY_FN(KinBody,CalculateJacobian "int; const Vector; std::vector"))
                        .def("CalculateRotationJacobian",&PyKinBody::CalculateRotationJacobian,args("linkindex","quat"), DOXY_FN(KinBody,CalculateRotationJacobian "int; const Vector; std::vector"))
                        .def("CalculateAngularVelocityJacobian",&PyKinBody::CalculateAngularVelocityJacobian,args("linkindex"), DOXY_FN(KinBody,CalculateAngularVelocityJacobian "int; std::vector"))
                        .def("ComputeHessianTranslation",&PyKinBody::ComputeHessianTranslation,ComputeHessianTranslation_overloads(args("linkindex","position","indices"), DOXY_FN(KinBody,ComputeHessianTranslation)))
                        .def("ComputeHessianAxisAngle",&PyKinBody::ComputeHessianAxisAngle,ComputeHessianAxisAngle_overloads(args("linkindex","indices"), DOXY_FN(KinBody,ComputeHessianAxisAngle)))
                        .def("ComputeInverseDynamics",&PyKinBody::ComputeInverseDynamics, ComputeInverseDynamics_overloads(args("dofaccelerations","externalforcetorque","returncomponents"), sComputeInverseDynamicsDoc.c_str()))
                        .def("CheckSelfCollision",pkinbodyself, DOXY_FN(KinBody,CheckSelfCollision))
                        .def("CheckSelfCollision",pkinbodyselfr,args("report"), DOXY_FN(KinBody,CheckSelfCollision))
                        .def("IsAttached",&PyKinBody::IsAttached,args("body"), DOXY_FN(KinBody,IsAttached))
                        .def("GetAttached",&PyKinBody::GetAttached, DOXY_FN(KinBody,GetAttached))
                        .def("SetZeroConfiguration",&PyKinBody::SetZeroConfiguration, DOXY_FN(KinBody,SetZeroConfiguration))
                        .def("SetNonCollidingConfiguration",&PyKinBody::SetNonCollidingConfiguration, DOXY_FN(KinBody,SetNonCollidingConfiguration))
                        .def("GetConfigurationSpecification",&PyKinBody::GetConfigurationSpecification, GetConfigurationSpecification_overloads(args("interpolation"), DOXY_FN(KinBody,GetConfigurationSpecification)))
                        .def("GetConfigurationSpecificationIndices",&PyKinBody::GetConfigurationSpecificationIndices, GetConfigurationSpecificationIndices_overloads(args("indices","interpolation"), DOXY_FN(KinBody,GetConfigurationSpecificationIndices)))
                        .def("SetConfigurationValues",&PyKinBody::SetConfigurationValues, SetConfigurationValues_overloads(args("values","checklimits"), DOXY_FN(KinBody,SetConfigurationValues)))
                        .def("GetConfigurationValues",&PyKinBody::GetConfigurationValues, DOXY_FN(KinBody,GetConfigurationValues))
                        .def("IsRobot",&PyKinBody::IsRobot, DOXY_FN(KinBody,IsRobot))
                        .def("GetEnvironmentId",&PyKinBody::GetEnvironmentId, DOXY_FN(KinBody,GetEnvironmentId))
                        .def("DoesAffect",&PyKinBody::DoesAffect,args("jointindex","linkindex"), DOXY_FN(KinBody,DoesAffect))
                        .def("GetViewerData",&PyKinBody::GetViewerData, DOXY_FN(KinBody,GetViewerData))
                        .def("GetGuiData",&PyKinBody::GetViewerData, DOXY_FN(KinBody,GetViewerData))
                        .def("GetURI",&PyKinBody::GetURI, DOXY_FN(InterfaceBase,GetURI))
                        .def("GetXMLFilename",&PyKinBody::GetURI, DOXY_FN(InterfaceBase,GetURI))
                        .def("GetNonAdjacentLinks",GetNonAdjacentLinks1, DOXY_FN(KinBody,GetNonAdjacentLinks))
                        .def("GetNonAdjacentLinks",GetNonAdjacentLinks2, args("adjacentoptions"), DOXY_FN(KinBody,GetNonAdjacentLinks))
                        .def("GetAdjacentLinks",&PyKinBody::GetAdjacentLinks, DOXY_FN(KinBody,GetAdjacentLinks))
                        .def("GetPhysicsData",&PyKinBody::GetPhysicsData, DOXY_FN(KinBody,GetPhysicsData))
                        .def("GetCollisionData",&PyKinBody::GetCollisionData, DOXY_FN(KinBody,GetCollisionData))
                        .def("GetManageData",&PyKinBody::GetManageData, DOXY_FN(KinBody,GetManageData))
                        .def("GetUpdateStamp",&PyKinBody::GetUpdateStamp, DOXY_FN(KinBody,GetUpdateStamp))
                        .def("serialize",&PyKinBody::serialize,args("options"), DOXY_FN(KinBody,serialize))
                        .def("GetKinematicsGeometryHash",&PyKinBody::GetKinematicsGeometryHash, DOXY_FN(KinBody,GetKinematicsGeometryHash))
                        .def("CreateKinBodyStateSaver",&PyKinBody::CreateKinBodyStateSaver, CreateKinBodyStateSaver_overloads(args("options"), "Creates an object that can be entered using 'with' and returns a KinBodyStateSaver")[return_value_policy<manage_new_object>()])
                        .def("__enter__",&PyKinBody::__enter__)
                        .def("__exit__",&PyKinBody::__exit__)
                        .def("__repr__",&PyKinBody::__repr__)
                        .def("__str__",&PyKinBody::__str__)
                        .def("__unicode__",&PyKinBody::__unicode__)
        ;

        enum_<KinBody::SaveParameters>("SaveParameters" DOXY_ENUM(SaveParameters))
        .value("LinkTransformation",KinBody::Save_LinkTransformation)
        .value("LinkEnable",KinBody::Save_LinkEnable)
        .value("JointMaxVelocityAndAcceleration",KinBody::Save_JointMaxVelocityAndAcceleration)
        .value("ActiveDOF",KinBody::Save_ActiveDOF)
        .value("ActiveManipulator",KinBody::Save_ActiveManipulator)
        .value("GrabbedBodies",KinBody::Save_GrabbedBodies)
        ;
        enum_<KinBody::CheckLimitsAction>("CheckLimitsAction" DOXY_ENUM(CheckLimitsAction))
        .value("Nothing",KinBody::CLA_Nothing)
        .value("CheckLimits",KinBody::CLA_CheckLimits)
        .value("CheckLimitsSilent",KinBody::CLA_CheckLimitsSilent)
        .value("CheckLimitsThrow",KinBody::CLA_CheckLimitsThrow)
        ;
        enum_<KinBody::AdjacentOptions>("AdjacentOptions" DOXY_ENUM(AdjacentOptions))
        .value("Enabled",KinBody::AO_Enabled)
        .value("ActiveDOFs",KinBody::AO_ActiveDOFs)
        ;
        object jointtype = enum_<KinBody::JointType>("JointType" DOXY_ENUM(JointType))
                           .value("None",KinBody::JointNone)
                           .value("Hinge",KinBody::JointHinge)
                           .value("Revolute",KinBody::JointRevolute)
                           .value("Slider",KinBody::JointSlider)
                           .value("Prismatic",KinBody::JointPrismatic)
                           .value("RR",KinBody::JointRR)
                           .value("RP",KinBody::JointRP)
                           .value("PR",KinBody::JointPR)
                           .value("PP",KinBody::JointPP)
                           .value("Universal",KinBody::JointUniversal)
                           .value("Hinge2",KinBody::JointHinge2)
                           .value("Spherical",KinBody::JointSpherical)
                           .value("Trajectory",KinBody::JointTrajectory)
        ;
        object geometryinfo = class_<PyKinBody::PyGeometryInfo, boost::shared_ptr<PyKinBody::PyGeometryInfo> >("GeometryInfo", DOXY_CLASS(KinBody::GeometryInfo))
                              .def_readwrite("_t",&PyKinBody::PyGeometryInfo::_t)
                              .def_readwrite("_vGeomData",&PyKinBody::PyGeometryInfo::_vGeomData)
                              .def_readwrite("_vDiffuseColor",&PyKinBody::PyGeometryInfo::_vDiffuseColor)
                              .def_readwrite("_vAmbientColor",&PyKinBody::PyGeometryInfo::_vAmbientColor)
                              .def_readwrite("_meshcollision",&PyKinBody::PyGeometryInfo::_meshcollision)
                              .def_readwrite("_type",&PyKinBody::PyGeometryInfo::_type)
                              .def_readwrite("_filenamerender",&PyKinBody::PyGeometryInfo::_filenamerender)
                              .def_readwrite("_filenamecollision",&PyKinBody::PyGeometryInfo::_filenamecollision)
                              .def_readwrite("_vRenderScale",&PyKinBody::PyGeometryInfo::_vRenderScale)
                              .def_readwrite("_vCollisionScale",&PyKinBody::PyGeometryInfo::_vCollisionScale)
                              .def_readwrite("_fTransparency",&PyKinBody::PyGeometryInfo::_fTransparency)
                              .def_readwrite("_bVisible",&PyKinBody::PyGeometryInfo::_bVisible)
                              .def_readwrite("_bModifiable",&PyKinBody::PyGeometryInfo::_bModifiable)
                              .def_pickle(GeometryInfo_pickle_suite())
        ;
        object linkinfo = class_<PyKinBody::PyLinkInfo, boost::shared_ptr<PyKinBody::PyLinkInfo> >("LinkInfo", DOXY_CLASS(KinBody::LinkInfo))
                          .def_readwrite("_vgeometryinfos",&PyKinBody::PyLinkInfo::_vgeometryinfos)
                          .def_readwrite("_name",&PyKinBody::PyLinkInfo::_name)
                          .def_readwrite("_t",&PyKinBody::PyLinkInfo::_t)
                          .def_readwrite("_tMassFrame",&PyKinBody::PyLinkInfo::_tMassFrame)
                          .def_readwrite("_mass",&PyKinBody::PyLinkInfo::_mass)
                          .def_readwrite("_vinertiamoments",&PyKinBody::PyLinkInfo::_vinertiamoments)
                          .def_readwrite("_mapFloatParameters",&PyKinBody::PyLinkInfo::_mapFloatParameters)
                          .def_readwrite("_mapIntParameters",&PyKinBody::PyLinkInfo::_mapIntParameters)
                          .def_readwrite("_vForcedAdjacentLinks",&PyKinBody::PyLinkInfo::_vForcedAdjacentLinks)
                          .def_readwrite("_bStatic",&PyKinBody::PyLinkInfo::_bStatic)
                          .def_readwrite("_bIsEnabled",&PyKinBody::PyLinkInfo::_bIsEnabled)
                          .def_pickle(LinkInfo_pickle_suite())
        ;
        object jointinfo = class_<PyKinBody::PyJointInfo, boost::shared_ptr<PyKinBody::PyJointInfo> >("JointInfo", DOXY_CLASS(KinBody::JointInfo))
                           .def_readwrite("_type",&PyKinBody::PyJointInfo::_type)
                           .def_readwrite("_name",&PyKinBody::PyJointInfo::_name)
                           .def_readwrite("_linkname0",&PyKinBody::PyJointInfo::_linkname0)
                           .def_readwrite("_linkname1",&PyKinBody::PyJointInfo::_linkname1)
                           .def_readwrite("_vanchor",&PyKinBody::PyJointInfo::_vanchor)
                           .def_readwrite("_vaxes",&PyKinBody::PyJointInfo::_vaxes)
                           .def_readwrite("_vcurrentvalues",&PyKinBody::PyJointInfo::_vcurrentvalues)
                           .def_readwrite("_vresolution",&PyKinBody::PyJointInfo::_vresolution)
                           .def_readwrite("_vmaxvel",&PyKinBody::PyJointInfo::_vmaxvel)
                           .def_readwrite("_vhardmaxvel",&PyKinBody::PyJointInfo::_vhardmaxvel)
                           .def_readwrite("_vmaxaccel",&PyKinBody::PyJointInfo::_vmaxaccel)
                           .def_readwrite("_vmaxtorque",&PyKinBody::PyJointInfo::_vmaxtorque)
                           .def_readwrite("_vweights",&PyKinBody::PyJointInfo::_vweights)
                           .def_readwrite("_voffsets",&PyKinBody::PyJointInfo::_voffsets)
                           .def_readwrite("_vlowerlimit",&PyKinBody::PyJointInfo::_vlowerlimit)
                           .def_readwrite("_vupperlimit",&PyKinBody::PyJointInfo::_vupperlimit)
                           .def_readwrite("_trajfollow",&PyKinBody::PyJointInfo::_trajfollow)
                           .def_readwrite("_vmimic",&PyKinBody::PyJointInfo::_vmimic)
                           .def_readwrite("_mapFloatParameters",&PyKinBody::PyJointInfo::_mapFloatParameters)
                           .def_readwrite("_mapIntParameters",&PyKinBody::PyJointInfo::_mapIntParameters)
                           .def_readwrite("_bIsCircular",&PyKinBody::PyJointInfo::_bIsCircular)
                           .def_pickle(JointInfo_pickle_suite())
        ;
        {
            scope link = class_<PyKinBody::PyLink, boost::shared_ptr<PyKinBody::PyLink> >("Link", DOXY_CLASS(KinBody::Link), no_init)
                         .def("GetName",&PyKinBody::PyLink::GetName, DOXY_FN(KinBody::Link,GetName))
                         .def("GetIndex",&PyKinBody::PyLink::GetIndex, DOXY_FN(KinBody::Link,GetIndex))
                         .def("Enable",&PyKinBody::PyLink::Enable,args("enable"), DOXY_FN(KinBody::Link,Enable))
                         .def("IsEnabled",&PyKinBody::PyLink::IsEnabled, DOXY_FN(KinBody::Link,IsEnabled))
                         .def("IsStatic",&PyKinBody::PyLink::IsStatic, DOXY_FN(KinBody::Link,IsStatic))
                         .def("SetVisible",&PyKinBody::PyLink::SetVisible,args("visible"), DOXY_FN(KinBody::Link,SetVisible))
                         .def("IsVisible",&PyKinBody::PyLink::IsVisible, DOXY_FN(KinBody::Link,IsVisible))
                         .def("GetParent",&PyKinBody::PyLink::GetParent, DOXY_FN(KinBody::Link,GetParent))
                         .def("GetParentLinks",&PyKinBody::PyLink::GetParentLinks, DOXY_FN(KinBody::Link,GetParentLinks))
                         .def("IsParentLink",&PyKinBody::PyLink::IsParentLink, DOXY_FN(KinBody::Link,IsParentLink))
                         .def("GetCollisionData",&PyKinBody::PyLink::GetCollisionData, DOXY_FN(KinBody::Link,GetCollisionData))
                         .def("ComputeAABB",&PyKinBody::PyLink::ComputeAABB, DOXY_FN(KinBody::Link,ComputeAABB))
                         .def("ComputeLocalAABB",&PyKinBody::PyLink::ComputeLocalAABB, DOXY_FN(KinBody::Link,ComputeLocalAABB))
                         .def("GetTransform",&PyKinBody::PyLink::GetTransform, DOXY_FN(KinBody::Link,GetTransform))
                         .def("GetCOMOffset",&PyKinBody::PyLink::GetCOMOffset, DOXY_FN(KinBody::Link,GetCOMOffset))
                         .def("GetLocalCOM",&PyKinBody::PyLink::GetLocalCOM, DOXY_FN(KinBody::Link,GetLocalCOM))
                         .def("GetGlobalCOM",&PyKinBody::PyLink::GetGlobalCOM, DOXY_FN(KinBody::Link,GetGlobalCOM))
                         .def("GetLocalInertia",&PyKinBody::PyLink::GetLocalInertia, DOXY_FN(KinBody::Link,GetLocalInertia))
                         .def("GetPrincipalMomentsOfInertia",&PyKinBody::PyLink::GetPrincipalMomentsOfInertia, DOXY_FN(KinBody::Link,GetPrincipalMomentsOfInertia))
                         .def("GetLocalMassFrame",&PyKinBody::PyLink::GetLocalMassFrame, DOXY_FN(KinBody::Link,GetLocalMassFrame))
                         .def("GetGlobalMassFrame",&PyKinBody::PyLink::GetGlobalMassFrame, DOXY_FN(KinBody::Link,GetGlobalMassFrame))
                         .def("GetMass",&PyKinBody::PyLink::GetMass, DOXY_FN(KinBody::Link,GetMass))
                         .def("SetLocalMassFrame",&PyKinBody::PyLink::SetLocalMassFrame, args("massframe"), DOXY_FN(KinBody::Link,SetLocalMassFrame))
                         .def("SetPrincipalMomentsOfInertia",&PyKinBody::PyLink::SetPrincipalMomentsOfInertia, args("inertiamoments"), DOXY_FN(KinBody::Link,SetPrincipalMomentsOfInertia))
                         .def("SetMass",&PyKinBody::PyLink::SetMass, args("mass"), DOXY_FN(KinBody::Link,SetMass))
                         .def("SetStatic",&PyKinBody::PyLink::SetStatic,args("static"), DOXY_FN(KinBody::Link,SetStatic))
                         .def("SetTransform",&PyKinBody::PyLink::SetTransform,args("transform"), DOXY_FN(KinBody::Link,SetTransform))
                         .def("SetForce",&PyKinBody::PyLink::SetForce,args("force","pos","add"), DOXY_FN(KinBody::Link,SetForce))
                         .def("SetTorque",&PyKinBody::PyLink::SetTorque,args("torque","add"), DOXY_FN(KinBody::Link,SetTorque))
                         .def("GetGeometries",&PyKinBody::PyLink::GetGeometries, DOXY_FN(KinBody::Link,GetGeometries))
                         .def("GetRigidlyAttachedLinks",&PyKinBody::PyLink::GetRigidlyAttachedLinks, DOXY_FN(KinBody::Link,GetRigidlyAttachedLinks))
                         .def("IsRigidlyAttached",&PyKinBody::PyLink::IsRigidlyAttached, DOXY_FN(KinBody::Link,IsRigidlyAttached))
                         .def("GetVelocity",&PyKinBody::PyLink::GetVelocity,DOXY_FN(KinBody::Link,GetVelocity))
                         .def("SetVelocity",&PyKinBody::PyLink::SetVelocity,DOXY_FN(KinBody::Link,SetVelocity))
                         .def("GetFloatParameters",&PyKinBody::PyLink::GetFloatParameters,DOXY_FN(KinBody::Link,GetFloatParameters))
                         .def("GetIntParameters",&PyKinBody::PyLink::GetIntParameters,DOXY_FN(KinBody::Link,GetIntParameters))
                         .def("__repr__", &PyKinBody::PyLink::__repr__)
                         .def("__str__", &PyKinBody::PyLink::__str__)
                         .def("__unicode__", &PyKinBody::PyLink::__unicode__)
                         .def("__eq__",&PyKinBody::PyLink::__eq__)
                         .def("__ne__",&PyKinBody::PyLink::__ne__)
                         .def("__hash__",&PyKinBody::PyLink::__hash__)
            ;
            // \deprecated (12/10/18)
            link.attr("GeomType") = geometrytype;
            link.attr("GeometryInfo") = geometryinfo;
            {
                scope geometry = class_<PyKinBody::PyLink::PyGeometry, boost::shared_ptr<PyKinBody::PyLink::PyGeometry> >("Geometry", DOXY_CLASS(KinBody::Link::Geometry),no_init)
                                 .def("SetCollisionMesh",&PyKinBody::PyLink::PyGeometry::SetCollisionMesh,args("trimesh"), DOXY_FN(KinBody::Link::Geometry,SetCollisionMesh))
                                 .def("GetCollisionMesh",&PyKinBody::PyLink::PyGeometry::GetCollisionMesh, DOXY_FN(KinBody::Link::Geometry,GetCollisionMesh))
                                 .def("ComputeAABB",&PyKinBody::PyLink::PyGeometry::ComputeAABB, args("transform"), DOXY_FN(KinBody::Link::Geometry,ComputeAABB))
                                 .def("SetDraw",&PyKinBody::PyLink::PyGeometry::SetDraw,args("draw"), DOXY_FN(KinBody::Link::Geometry,SetDraw))
                                 .def("SetTransparency",&PyKinBody::PyLink::PyGeometry::SetTransparency,args("transparency"), DOXY_FN(KinBody::Link::Geometry,SetTransparency))
                                 .def("SetDiffuseColor",&PyKinBody::PyLink::PyGeometry::SetDiffuseColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetDiffuseColor))
                                 .def("SetAmbientColor",&PyKinBody::PyLink::PyGeometry::SetAmbientColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetAmbientColor))
                                 .def("SetRenderFilename",&PyKinBody::PyLink::PyGeometry::SetRenderFilename,args("color"), DOXY_FN(KinBody::Link::Geometry,SetRenderFilename))
                                 .def("IsDraw",&PyKinBody::PyLink::PyGeometry::IsDraw, DOXY_FN(KinBody::Link::Geometry,IsDraw))
                                 .def("IsVisible",&PyKinBody::PyLink::PyGeometry::IsVisible, DOXY_FN(KinBody::Link::Geometry,IsVisible))
                                 .def("IsModifiable",&PyKinBody::PyLink::PyGeometry::IsModifiable, DOXY_FN(KinBody::Link::Geometry,IsModifiable))
                                 .def("GetType",&PyKinBody::PyLink::PyGeometry::GetType, DOXY_FN(KinBody::Link::Geometry,GetType))
                                 .def("GetTransform",&PyKinBody::PyLink::PyGeometry::GetTransform, DOXY_FN(KinBody::Link::Geometry,GetTransform))
                                 .def("GetSphereRadius",&PyKinBody::PyLink::PyGeometry::GetSphereRadius, DOXY_FN(KinBody::Link::Geometry,GetSphereRadius))
                                 .def("GetCylinderRadius",&PyKinBody::PyLink::PyGeometry::GetCylinderRadius, DOXY_FN(KinBody::Link::Geometry,GetCylinderRadius))
                                 .def("GetCylinderHeight",&PyKinBody::PyLink::PyGeometry::GetCylinderHeight, DOXY_FN(KinBody::Link::Geometry,GetCylinderHeight))
                                 .def("GetBoxExtents",&PyKinBody::PyLink::PyGeometry::GetBoxExtents, DOXY_FN(KinBody::Link::Geometry,GetBoxExtents))
                                 .def("GetRenderScale",&PyKinBody::PyLink::PyGeometry::GetRenderScale, DOXY_FN(KinBody::Link::Geometry,GetRenderScale))
                                 .def("GetRenderFilename",&PyKinBody::PyLink::PyGeometry::GetRenderFilename, DOXY_FN(KinBody::Link::Geometry,GetRenderFilename))
                                 .def("GetTransparency",&PyKinBody::PyLink::PyGeometry::GetTransparency,DOXY_FN(KinBody::Link::Geometry,GetTransparency))
                                 .def("GetDiffuseColor",&PyKinBody::PyLink::PyGeometry::GetDiffuseColor,DOXY_FN(KinBody::Link::Geometry,GetDiffuseColor))
                                 .def("GetAmbientColor",&PyKinBody::PyLink::PyGeometry::GetAmbientColor,DOXY_FN(KinBody::Link::Geometry,GetAmbientColor))
                                 .def("__eq__",&PyKinBody::PyLink::PyGeometry::__eq__)
                                 .def("__ne__",&PyKinBody::PyLink::PyGeometry::__ne__)
                                 .def("__hash__",&PyKinBody::PyLink::PyGeometry::__hash__)
                ;
                // \deprecated (12/07/16)
                geometry.attr("Type") = geometrytype;
            }
            // \deprecated (12/07/16)
            link.attr("GeomProperties") = link.attr("Geometry");
        }
        {
            scope joint = class_<PyKinBody::PyJoint, boost::shared_ptr<PyKinBody::PyJoint> >("Joint", DOXY_CLASS(KinBody::Joint),no_init)
                          .def("GetName", &PyKinBody::PyJoint::GetName, DOXY_FN(KinBody::Joint,GetName))
                          .def("IsMimic",&PyKinBody::PyJoint::IsMimic,IsMimic_overloads(args("axis"), DOXY_FN(KinBody::Joint,IsMimic)))
                          .def("GetMimicEquation",&PyKinBody::PyJoint::GetMimicEquation,GetMimicEquation_overloads(args("axis","type","format"), DOXY_FN(KinBody::Joint,GetMimicEquation)))
                          .def("GetMimicDOFIndices",&PyKinBody::PyJoint::GetMimicDOFIndices,GetMimicDOFIndices_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetMimicDOFIndices)))
                          .def("SetMimicEquations", &PyKinBody::PyJoint::SetMimicEquations, args("axis","poseq","veleq","acceleq"), DOXY_FN(KinBody::Joint,SetMimicEquations))
                          .def("GetMaxVel", &PyKinBody::PyJoint::GetMaxVel, GetMaxVel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxVel)))
                          .def("GetMaxAccel", &PyKinBody::PyJoint::GetMaxAccel, GetMaxAccel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxAccel)))
                          .def("GetMaxTorque", &PyKinBody::PyJoint::GetMaxTorque, GetMaxTorque_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxTorque)))
                          .def("GetDOFIndex", &PyKinBody::PyJoint::GetDOFIndex, DOXY_FN(KinBody::Joint,GetDOFIndex))
                          .def("GetJointIndex", &PyKinBody::PyJoint::GetJointIndex, DOXY_FN(KinBody::Joint,GetJointIndex))
                          .def("GetParent", &PyKinBody::PyJoint::GetParent, DOXY_FN(KinBody::Joint,GetParent))
                          .def("GetFirstAttached", &PyKinBody::PyJoint::GetFirstAttached, DOXY_FN(KinBody::Joint,GetFirstAttached))
                          .def("GetSecondAttached", &PyKinBody::PyJoint::GetSecondAttached, DOXY_FN(KinBody::Joint,GetSecondAttached))
                          .def("IsStatic",&PyKinBody::PyJoint::IsStatic, DOXY_FN(KinBody::Joint,IsStatic))
                          .def("IsCircular",&PyKinBody::PyJoint::IsCircular, DOXY_FN(KinBody::Joint,IsCircular))
                          .def("IsRevolute",&PyKinBody::PyJoint::IsRevolute, DOXY_FN(KinBody::Joint,IsRevolute))
                          .def("IsPrismatic",&PyKinBody::PyJoint::IsPrismatic, DOXY_FN(KinBody::Joint,IsPrismatic))
                          .def("GetType", &PyKinBody::PyJoint::GetType, DOXY_FN(KinBody::Joint,GetType))
                          .def("GetDOF", &PyKinBody::PyJoint::GetDOF, DOXY_FN(KinBody::Joint,GetDOF))
                          .def("GetValues", &PyKinBody::PyJoint::GetValues, DOXY_FN(KinBody::Joint,GetValues))
                          .def("GetVelocities", &PyKinBody::PyJoint::GetVelocities, DOXY_FN(KinBody::Joint,GetVelocities))
                          .def("GetAnchor", &PyKinBody::PyJoint::GetAnchor, DOXY_FN(KinBody::Joint,GetAnchor))
                          .def("GetAxis", &PyKinBody::PyJoint::GetAxis,GetAxis_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetAxis)))
                          .def("GetHierarchyParentLink", &PyKinBody::PyJoint::GetHierarchyParentLink, DOXY_FN(KinBody::Joint,GetHierarchyParentLink))
                          .def("GetHierarchyChildLink", &PyKinBody::PyJoint::GetHierarchyChildLink, DOXY_FN(KinBody::Joint,GetHierarchyChildLink))
                          .def("GetInternalHierarchyAxis", &PyKinBody::PyJoint::GetInternalHierarchyAxis,args("axis"), DOXY_FN(KinBody::Joint,GetInternalHierarchyAxis))
                          .def("GetInternalHierarchyLeftTransform",&PyKinBody::PyJoint::GetInternalHierarchyLeftTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyLeftTransform))
                          .def("GetInternalHierarchyRightTransform",&PyKinBody::PyJoint::GetInternalHierarchyRightTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyRightTransform))
                          .def("GetLimits", &PyKinBody::PyJoint::GetLimits, DOXY_FN(KinBody::Joint,GetLimits))
                          .def("GetVelocityLimits", &PyKinBody::PyJoint::GetVelocityLimits, DOXY_FN(KinBody::Joint,GetVelocityLimits))
                          .def("GetAccelerationLimits", &PyKinBody::PyJoint::GetAccelerationLimits, DOXY_FN(KinBody::Joint,GetAccelerationLimits))
                          .def("GetTorqueLimits", &PyKinBody::PyJoint::GetTorqueLimits, DOXY_FN(KinBody::Joint,GetTorqueLimits))
                          .def("SetWrapOffset",&PyKinBody::PyJoint::SetWrapOffset,SetWrapOffset_overloads(args("offset","axis"), DOXY_FN(KinBody::Joint,SetWrapOffset)))
                          .def("GetWrapOffset",&PyKinBody::PyJoint::GetWrapOffset,GetWrapOffset_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetWrapOffset)))
                          .def("SetLimits",&PyKinBody::PyJoint::SetLimits,args("lower","upper"), DOXY_FN(KinBody::Joint,SetLimits))
                          .def("SetVelocityLimits",&PyKinBody::PyJoint::SetVelocityLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetVelocityLimits))
                          .def("SetAccelerationLimits",&PyKinBody::PyJoint::SetAccelerationLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetAccelerationLimits))
                          .def("SetTorqueLimits",&PyKinBody::PyJoint::SetTorqueLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetTorqueLimits))
                          .def("GetResolution",&PyKinBody::PyJoint::GetResolution,args("axis"), DOXY_FN(KinBody::Joint,GetResolution))
                          .def("GetResolutions",&PyKinBody::PyJoint::GetResolutions,DOXY_FN(KinBody::Joint,GetResolutions))
                          .def("SetResolution",&PyKinBody::PyJoint::SetResolution,args("resolution"), DOXY_FN(KinBody::Joint,SetResolution))
                          .def("GetWeight",&PyKinBody::PyJoint::GetWeight,args("axis"), DOXY_FN(KinBody::Joint,GetWeight))
                          .def("GetWeights",&PyKinBody::PyJoint::GetWeights,DOXY_FN(KinBody::Joint,GetWeights))
                          .def("SetWeights",&PyKinBody::PyJoint::SetWeights,args("weights"), DOXY_FN(KinBody::Joint,SetWeights))
                          .def("SubtractValues",&PyKinBody::PyJoint::SubtractValues,args("values0","values1"), DOXY_FN(KinBody::Joint,SubtractValues))
                          .def("SubtractValue",&PyKinBody::PyJoint::SubtractValue,args("value0","value1","axis"), DOXY_FN(KinBody::Joint,SubtractValue))

                          .def("AddTorque",&PyKinBody::PyJoint::AddTorque,args("torques"), DOXY_FN(KinBody::Joint,AddTorque))
                          .def("GetFloatParameters",&PyKinBody::PyJoint::GetFloatParameters,DOXY_FN(KinBody::Joint,GetFloatParameters))
                          .def("GetIntParameters",&PyKinBody::PyJoint::GetIntParameters,DOXY_FN(KinBody::Joint,GetIntParameters))
                          .def("__repr__", &PyKinBody::PyJoint::__repr__)
                          .def("__str__", &PyKinBody::PyJoint::__str__)
                          .def("__unicode__", &PyKinBody::PyJoint::__unicode__)
                          .def("__eq__",&PyKinBody::PyJoint::__eq__)
                          .def("__ne__",&PyKinBody::PyJoint::__ne__)
                          .def("__hash__",&PyKinBody::PyJoint::__hash__)
            ;
            joint.attr("Type") = jointtype;
        }

        {
            scope statesaver = class_<PyKinBody::PyKinBodyStateSaver, boost::shared_ptr<PyKinBody::PyKinBodyStateSaver> >("KinBodyStateSaver", DOXY_CLASS(KinBody::KinBodyStateSaver), no_init)
                               .def(init<PyKinBodyPtr>(args("body")))
                               .def(init<PyKinBodyPtr,object>(args("body","options")))
                               .def("GetBody",&PyKinBody::PyKinBodyStateSaver::GetBody,DOXY_FN(KinBody::KinBodyStateSaver, GetBody))
                               .def("Restore",&PyKinBody::PyKinBodyStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(KinBody::KinBodyStateSaver, Restore)))
                               .def("Release",&PyKinBody::PyKinBodyStateSaver::Release,DOXY_FN(KinBody::KinBodyStateSaver, Release))
                               .def("__str__",&PyKinBody::PyKinBodyStateSaver::__str__)
                               .def("__unicode__",&PyKinBody::PyKinBodyStateSaver::__unicode__)
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
                               .def("__repr__", &PyKinBody::PyManageData::__repr__)
                               .def("__str__", &PyKinBody::PyManageData::__str__)
                               .def("__unicode__", &PyKinBody::PyManageData::__unicode__)
                               .def("__eq__",&PyKinBody::PyManageData::__eq__)
                               .def("__ne__",&PyKinBody::PyManageData::__ne__)
            ;
        }
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
        object (PyRobotBase::*GetManipulators2)(const string &) = &PyRobotBase::GetManipulators;
        bool (PyRobotBase::*setcontroller1)(PyControllerBasePtr,const string &) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller2)(PyControllerBasePtr,object,int) = &PyRobotBase::SetController;
        bool (PyRobotBase::*setcontroller3)(PyControllerBasePtr) = &PyRobotBase::SetController;
        scope robot = class_<PyRobotBase, boost::shared_ptr<PyRobotBase>, bases<PyKinBody, PyInterfaceBase> >("Robot", DOXY_CLASS(RobotBase), no_init)
                      .def("GetManipulators",GetManipulators1, DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulators",GetManipulators2,args("manipname"), DOXY_FN(RobotBase,GetManipulators))
                      .def("GetManipulator",&PyRobotBase::GetManipulator,args("manipname"), "Return the manipulator whose name matches")
                      .def("SetActiveManipulator",setactivemanipulator1,args("manipindex"), DOXY_FN(RobotBase,SetActiveManipulator "int"))
                      .def("SetActiveManipulator",setactivemanipulator2,args("manipname"), DOXY_FN(RobotBase,SetActiveManipulator "const std::string"))
                      .def("SetActiveManipulator",setactivemanipulator3,args("manip"), "Set the active manipulator given a pointer")
                      .def("GetActiveManipulator",&PyRobotBase::GetActiveManipulator, DOXY_FN(RobotBase,GetActiveManipulator))
                      .def("AddManipulator",&PyRobotBase::AddManipulator, args("manip"), DOXY_FN(RobotBase,AddManipulator))
                      .def("RemoveManipulator",&PyRobotBase::RemoveManipulator, args("manip"), DOXY_FN(RobotBase,RemoveManipulator))
                      .def("GetActiveManipulatorIndex",&PyRobotBase::GetActiveManipulatorIndex, DOXY_FN(RobotBase,GetActiveManipulatorIndex))
                      .def("GetAttachedSensors",&PyRobotBase::GetAttachedSensors, DOXY_FN(RobotBase,GetAttachedSensors))
                      .def("GetAttachedSensor",&PyRobotBase::GetAttachedSensor,args("sensorname"), "Return the attached sensor whose name matches")
                      .def("GetSensors",&PyRobotBase::GetSensors)
                      .def("GetSensor",&PyRobotBase::GetSensor,args("sensorname"))
                      .def("GetController",&PyRobotBase::GetController, DOXY_FN(RobotBase,GetController))
                      .def("SetController",setcontroller1,DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller2,args("robot","dofindices","controltransform"), DOXY_FN(RobotBase,SetController))
                      .def("SetController",setcontroller3,DOXY_FN(RobotBase,SetController))
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
                      .def("SetAffineRotationQuatLimits",&PyRobotBase::SetAffineRotationQuatLimits,args("quatangle"), DOXY_FN(RobotBase,SetAffineRotationQuatLimits))
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
                      .def("SetActiveDOFValues",&PyRobotBase::SetActiveDOFValues,SetActiveDOFValues_overloads(args("values","checklimits"), DOXY_FN(RobotBase,SetActiveDOFValues)))
                      .def("GetActiveDOFValues",&PyRobotBase::GetActiveDOFValues, DOXY_FN(RobotBase,GetActiveDOFValues))
                      .def("GetActiveDOFWeights",&PyRobotBase::GetActiveDOFWeights, DOXY_FN(RobotBase,GetActiveDOFWeights))
                      .def("SetActiveDOFVelocities",&PyRobotBase::SetActiveDOFVelocities, DOXY_FN(RobotBase,SetActiveDOFVelocities))
                      .def("GetActiveDOFVelocities",&PyRobotBase::GetActiveDOFVelocities, DOXY_FN(RobotBase,GetActiveDOFVelocities))
                      .def("GetActiveDOFLimits",&PyRobotBase::GetActiveDOFLimits, DOXY_FN(RobotBase,GetActiveDOFLimits))
                      .def("GetActiveDOFMaxVel",&PyRobotBase::GetActiveDOFMaxVel, DOXY_FN(RobotBase,GetActiveDOFMaxVel))
                      .def("GetActiveDOFMaxAccel",&PyRobotBase::GetActiveDOFMaxAccel, DOXY_FN(RobotBase,GetActiveDOFMaxAccel))
                      .def("GetActiveDOFResolutions",&PyRobotBase::GetActiveDOFResolutions, DOXY_FN(RobotBase,GetActiveDOFResolutions))
                      .def("GetActiveConfigurationSpecification",&PyRobotBase::GetActiveConfigurationSpecification, GetActiveConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase,GetActiveConfigurationSpecification)))
                      .def("GetActiveJointIndices",&PyRobotBase::GetActiveJointIndices)
                      .def("GetActiveDOFIndices",&PyRobotBase::GetActiveDOFIndices, DOXY_FN(RobotBase,GetActiveDOFIndices))
                      .def("SubtractActiveDOFValues",&PyRobotBase::SubtractActiveDOFValues, args("values0","values1"), DOXY_FN(RobotBase,SubtractActiveDOFValues))
                      .def("CalculateActiveJacobian",&PyRobotBase::CalculateActiveJacobian,args("linkindex","offset"), DOXY_FN(RobotBase,CalculateActiveJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveRotationJacobian",&PyRobotBase::CalculateActiveRotationJacobian,args("linkindex","quat"), DOXY_FN(RobotBase,CalculateActiveRotationJacobian "int; const Vector; std::vector"))
                      .def("CalculateActiveAngularVelocityJacobian",&PyRobotBase::CalculateActiveAngularVelocityJacobian,args("linkindex"), DOXY_FN(RobotBase,CalculateActiveAngularVelocityJacobian "int; std::vector"))
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
                      .def("CreateRobotStateSaver",&PyRobotBase::CreateRobotStateSaver, CreateRobotStateSaver_overloads(args("options"), "Creates an object that can be entered using 'with' and returns a RobotStateSaver")[return_value_policy<manage_new_object>()])
                      .def("__repr__", &PyRobotBase::__repr__)
                      .def("__str__", &PyRobotBase::__str__)
                      .def("__unicode__", &PyRobotBase::__unicode__)
        ;
        robot.attr("DOFAffine") = dofaffine; // deprecated (11/10/04)

        {
            scope manipulatorinfo = class_<PyRobotBase::PyManipulatorInfo, boost::shared_ptr<PyRobotBase::PyManipulatorInfo> >("ManipulatorInfo", DOXY_CLASS(RobotBase::ManipulatorInfo))
                                    .def_readwrite("_name",&PyRobotBase::PyManipulatorInfo::_name)
                                    .def_readwrite("_sBaseLinkName",&PyRobotBase::PyManipulatorInfo::_sBaseLinkName)
                                    .def_readwrite("_sEffectorLinkName",&PyRobotBase::PyManipulatorInfo::_sEffectorLinkName)
                                    .def_readwrite("_tLocalTool",&PyRobotBase::PyManipulatorInfo::_tLocalTool)
                                    .def_readwrite("_vClosingDirection",&PyRobotBase::PyManipulatorInfo::_vClosingDirection)
                                    .def_readwrite("_vdirection",&PyRobotBase::PyManipulatorInfo::_vdirection)
                                    .def_readwrite("_sIkSolverXMLId",&PyRobotBase::PyManipulatorInfo::_sIkSolverXMLId)
                                    .def_readwrite("_vGripperJointNames",&PyRobotBase::PyManipulatorInfo::_vGripperJointNames)
                                    .def_pickle(ManipulatorInfo_pickle_suite())
            ;
        }

        object (PyRobotBase::PyManipulator::*pmanipik)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipikf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolution;
        object (PyRobotBase::PyManipulator::*pmanipiks)(object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;
        object (PyRobotBase::PyManipulator::*pmanipiksf)(object, object, int, bool, bool) const = &PyRobotBase::PyManipulator::FindIKSolutions;

        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision1)(object) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckEndEffectorCollision2)(object,PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckEndEffectorCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision1)() const = &PyRobotBase::PyManipulator::CheckIndependentCollision;
        bool (PyRobotBase::PyManipulator::*pCheckIndependentCollision2)(PyCollisionReportPtr) const = &PyRobotBase::PyManipulator::CheckIndependentCollision;

        std::string GetIkParameterization_doc = std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "const IkParameterization; bool")) + std::string(DOXY_FN(RobotBase::Manipulator,GetIkParameterization "IkParameterizationType; bool"));
        class_<PyRobotBase::PyManipulator, boost::shared_ptr<PyRobotBase::PyManipulator> >("Manipulator", DOXY_CLASS(RobotBase::Manipulator), no_init)
        .def("GetEndEffectorTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetTransform", &PyRobotBase::PyManipulator::GetTransform, DOXY_FN(RobotBase::Manipulator,GetTransform))
        .def("GetVelocity", &PyRobotBase::PyManipulator::GetVelocity, DOXY_FN(RobotBase::Manipulator,GetVelocity))
        .def("GetName",&PyRobotBase::PyManipulator::GetName, DOXY_FN(RobotBase::Manipulator,GetName))
        .def("SetName",&PyRobotBase::PyManipulator::SetName, args("name"), DOXY_FN(RobotBase::Manipulator,SetName))
        .def("GetRobot",&PyRobotBase::PyManipulator::GetRobot, DOXY_FN(RobotBase::Manipulator,GetRobot))
        .def("SetIkSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetIkSolver",&PyRobotBase::PyManipulator::GetIkSolver, DOXY_FN(RobotBase::Manipulator,GetIkSolver))
        .def("SetIKSolver",&PyRobotBase::PyManipulator::SetIkSolver, DOXY_FN(RobotBase::Manipulator,SetIkSolver))
        .def("GetNumFreeParameters",&PyRobotBase::PyManipulator::GetNumFreeParameters, DOXY_FN(RobotBase::Manipulator,GetNumFreeParameters))
        .def("GetFreeParameters",&PyRobotBase::PyManipulator::GetFreeParameters, DOXY_FN(RobotBase::Manipulator,GetFreeParameters))
        .def("FindIKSolution",pmanipik,FindIKSolution_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; std::vector; int")))
        .def("FindIKSolution",pmanipikf,FindIKSolutionFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolution "const IkParameterization; const std::vector; std::vector; int")))
        .def("FindIKSolutions",pmanipiks,FindIKSolutions_overloads(args("param","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; std::vector; int")))
        .def("FindIKSolutions",pmanipiksf,FindIKSolutionsFree_overloads(args("param","freevalues","filteroptions","ikreturn","releasegil"), DOXY_FN(RobotBase::Manipulator,FindIKSolutions "const IkParameterization; const std::vector; std::vector; int")))
        .def("GetIkParameterization",&PyRobotBase::PyManipulator::GetIkParameterization, GetIkParameterization_overloads(args("iktype","inworld"), GetIkParameterization_doc.c_str()))
        .def("GetBase",&PyRobotBase::PyManipulator::GetBase, DOXY_FN(RobotBase::Manipulator,GetBase))
        .def("GetEndEffector",&PyRobotBase::PyManipulator::GetEndEffector, DOXY_FN(RobotBase::Manipulator,GetEndEffector))
        .def("GetGraspTransform",&PyRobotBase::PyManipulator::GetGraspTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("GetLocalToolTransform",&PyRobotBase::PyManipulator::GetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,GetLocalToolTransform))
        .def("SetLocalToolTransform",&PyRobotBase::PyManipulator::SetLocalToolTransform, DOXY_FN(RobotBase::Manipulator,SetLocalToolTransform))
        .def("GetGripperJoints",&PyRobotBase::PyManipulator::GetGripperJoints, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetGripperIndices",&PyRobotBase::PyManipulator::GetGripperIndices, DOXY_FN(RobotBase::Manipulator,GetGripperIndices))
        .def("GetArmJoints",&PyRobotBase::PyManipulator::GetArmJoints, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetArmIndices",&PyRobotBase::PyManipulator::GetArmIndices, DOXY_FN(RobotBase::Manipulator,GetArmIndices))
        .def("GetClosingDirection",&PyRobotBase::PyManipulator::GetClosingDirection, DOXY_FN(RobotBase::Manipulator,GetClosingDirection))
        .def("GetPalmDirection",&PyRobotBase::PyManipulator::GetPalmDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetDirection",&PyRobotBase::PyManipulator::GetDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("GetLocalToolDirection",&PyRobotBase::PyManipulator::GetLocalToolDirection, DOXY_FN(RobotBase::Manipulator,GetLocalToolDirection))
        .def("IsGrabbing",&PyRobotBase::PyManipulator::IsGrabbing,args("body"), DOXY_FN(RobotBase::Manipulator,IsGrabbing))
        .def("GetChildJoints",&PyRobotBase::PyManipulator::GetChildJoints, DOXY_FN(RobotBase::Manipulator,GetChildJoints))
        .def("GetChildDOFIndices",&PyRobotBase::PyManipulator::GetChildDOFIndices, DOXY_FN(RobotBase::Manipulator,GetChildDOFIndices))
        .def("GetChildLinks",&PyRobotBase::PyManipulator::GetChildLinks, DOXY_FN(RobotBase::Manipulator,GetChildLinks))
        .def("IsChildLink",&PyRobotBase::PyManipulator::IsChildLink, DOXY_FN(RobotBase::Manipulator,IsChildLink))
        .def("GetIndependentLinks",&PyRobotBase::PyManipulator::GetIndependentLinks, DOXY_FN(RobotBase::Manipulator,GetIndependentLinks))
        .def("GetArmConfigurationSpecification",&PyRobotBase::PyManipulator::GetArmConfigurationSpecification, GetArmConfigurationSpecification_overloads(args("interpolation"),DOXY_FN(RobotBase::Manipulator,GetArmConfigurationSpecification)))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision1,args("transform"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckEndEffectorCollision",pCheckEndEffectorCollision2,args("transform","report"), DOXY_FN(RobotBase::Manipulator,CheckEndEffectorCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision1, DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CheckIndependentCollision",pCheckIndependentCollision2,args("report"), DOXY_FN(RobotBase::Manipulator,CheckIndependentCollision))
        .def("CalculateJacobian",&PyRobotBase::PyManipulator::CalculateJacobian,DOXY_FN(RobotBase::Manipulator,CalculateJacobian))
        .def("CalculateRotationJacobian",&PyRobotBase::PyManipulator::CalculateRotationJacobian,DOXY_FN(RobotBase::Manipulator,CalculateRotationJacobian))
        .def("CalculateAngularVelocityJacobian",&PyRobotBase::PyManipulator::CalculateAngularVelocityJacobian,DOXY_FN(RobotBase::Manipulator,CalculateAngularVelocityJacobian))
        .def("GetStructureHash",&PyRobotBase::PyManipulator::GetStructureHash, DOXY_FN(RobotBase::Manipulator,GetStructureHash))
        .def("GetKinematicsStructureHash",&PyRobotBase::PyManipulator::GetKinematicsStructureHash, DOXY_FN(RobotBase::Manipulator,GetKinematicsStructureHash))
        .def("__repr__",&PyRobotBase::PyManipulator::__repr__)
        .def("__str__",&PyRobotBase::PyManipulator::__str__)
        .def("__unicode__",&PyRobotBase::PyManipulator::__unicode__)
        .def("__eq__",&PyRobotBase::PyManipulator::__eq__)
        .def("__ne__",&PyRobotBase::PyManipulator::__ne__)
        .def("__hash__",&PyRobotBase::PyManipulator::__hash__)
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
        .def("__unicode__",&PyRobotBase::PyAttachedSensor::__unicode__)
        .def("__eq__",&PyRobotBase::PyAttachedSensor::__eq__)
        .def("__ne__",&PyRobotBase::PyAttachedSensor::__ne__)
        .def("__hash__",&PyRobotBase::PyAttachedSensor::__hash__)
        ;

        class_<PyRobotBase::PyRobotStateSaver, boost::shared_ptr<PyRobotBase::PyRobotStateSaver> >("RobotStateSaver", DOXY_CLASS(Robot::RobotStateSaver), no_init)
        .def(init<PyRobotBasePtr>(args("robot")))
        .def(init<PyRobotBasePtr,object>(args("robot","options")))
        .def("GetBody",&PyRobotBase::PyRobotStateSaver::GetBody,DOXY_FN(Robot::RobotStateSaver, GetBody))
        .def("Restore",&PyRobotBase::PyRobotStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(Robot::RobotStateSaver, Restore)))
        .def("Release",&PyRobotBase::PyRobotStateSaver::Release,DOXY_FN(Robot::RobotStateSaver, Release))
        .def("__str__",&PyRobotBase::PyRobotStateSaver::__str__)
        .def("__unicode__",&PyRobotBase::PyRobotStateSaver::__unicode__)
        ;
    }

    def("RaveCreateRobot",openravepy::RaveCreateRobot,args("env","name"),DOXY_FN1(RaveCreateRobot));
    def("RaveCreateKinBody",openravepy::RaveCreateKinBody,args("env","name"),DOXY_FN1(RaveCreateKinBody));
}

}
