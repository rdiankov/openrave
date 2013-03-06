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
#include "openravepy_kinbody.h"

namespace openravepy {

class PyLink;
typedef boost::shared_ptr<PyLink> PyLinkPtr;
typedef boost::shared_ptr<PyLink const> PyLinkConstPtr;
class PyJoint;
typedef boost::shared_ptr<PyJoint> PyJointPtr;
typedef boost::shared_ptr<PyJoint const> PyJointConstPtr;

template <typename T>
boost::python::object GetCustomParameters(const std::map<std::string, std::vector<T> >& parameters, boost::python::object oname=boost::python::object(), int index=-1)
{
    if( oname == object() ) {
        boost::python::dict oparameters;
        FOREACHC(it, parameters) {
            oparameters[it->first] = toPyArray(it->second);
        }
        return oparameters;
    }
    std::string name = boost::python::extract<std::string>(oname);
    typename std::map<std::string, std::vector<T> >::const_iterator it = parameters.find(name);
    if( it != parameters.end() ) {
        if( index >= 0 ) {
            if( (size_t)index < it->second.size() ) {
                return object(it->second.at(index));
            }
            else {
                return boost::python::object();
            }
        }
        return toPyArray(it->second);
    }
    return boost::python::object();
}

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
    PyGeometryInfo(const KinBody::GeometryInfo info) {
        _t = ReturnTransform(info._t);
        _vGeomData = toPyVector4(info._vGeomData);
        _vDiffuseColor = toPyVector3(info._vDiffuseColor);
        _vAmbientColor = toPyVector3(info._vAmbientColor);
        _meshcollision = toPyTriMesh(info._meshcollision);
        _type = info._type;
        _filenamerender = ConvertStringToUnicode(info._filenamerender);
        _filenamecollision = ConvertStringToUnicode(info._filenamecollision);
        _vRenderScale = toPyVector3(info._vRenderScale);
        _vCollisionScale = toPyVector3(info._vCollisionScale);
        _fTransparency = info._fTransparency;
        _bVisible = info._bVisible;
        _bModifiable = info._bModifiable;
    }

    KinBody::GeometryInfoPtr GetGeometryInfo() {
        KinBody::GeometryInfoPtr pinfo(new KinBody::GeometryInfo());
        KinBody::GeometryInfo& info = *pinfo;
        info._t = ExtractTransform(_t);
        info._vGeomData = ExtractVector<dReal>(_vGeomData);
        info._vDiffuseColor = ExtractVector34<dReal>(_vDiffuseColor,0);
        info._vAmbientColor = ExtractVector34<dReal>(_vAmbientColor,0);
        if( !(_meshcollision == object()) ) {
            ExtractTriMesh(_meshcollision,info._meshcollision);
        }
        info._type = _type;
        if( !(_filenamerender == boost::python::object()) ) {
            info._filenamerender = boost::python::extract<std::string>(_filenamerender);
        }
        if( !(_filenamecollision == boost::python::object()) ) {
            info._filenamecollision = boost::python::extract<std::string>(_filenamecollision);
        }
        info._vRenderScale = ExtractVector3(_vRenderScale);
        info._vCollisionScale = ExtractVector3(_vCollisionScale);
        info._fTransparency = _fTransparency;
        info._bVisible = _bVisible;
        info._bModifiable = _bModifiable;
        return pinfo;
    }

    object _t, _vGeomData, _vDiffuseColor, _vAmbientColor, _meshcollision;
    GeometryType _type;
    object _filenamerender, _filenamecollision;
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
    PyLinkInfo(const KinBody::LinkInfo& info) {
        FOREACHC(itgeominfo, info._vgeometryinfos) {
            _vgeometryinfos.append(PyGeometryInfoPtr(new PyGeometryInfo(**itgeominfo)));
        }
        _name = ConvertStringToUnicode(info._name);
        _t = ReturnTransform(info._t);
        _tMassFrame = ReturnTransform(info._tMassFrame);
        _mass = info._mass;
        _vinertiamoments = toPyVector3(info._vinertiamoments);
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        boost::python::list vForcedAdjacentLinks;
        FOREACHC(it, info._vForcedAdjacentLinks) {
            vForcedAdjacentLinks.append(ConvertStringToUnicode(*it));
        }
        _vForcedAdjacentLinks = vForcedAdjacentLinks;
        _bStatic = info._bStatic;
        _bIsEnabled = info._bIsEnabled;
    }

    KinBody::LinkInfoPtr GetLinkInfo() {
        KinBody::LinkInfoPtr pinfo(new KinBody::LinkInfo());
        KinBody::LinkInfo& info = *pinfo;
        info._vgeometryinfos.resize(len(_vgeometryinfos));
        for(size_t i = 0; i < info._vgeometryinfos.size(); ++i) {
            PyGeometryInfoPtr pygeom = boost::python::extract<PyGeometryInfoPtr>(_vgeometryinfos[i]);
            info._vgeometryinfos[i] = pygeom->GetGeometryInfo();
        }
        if( !(_name == boost::python::object()) ) {
            info._name = boost::python::extract<std::string>(_name);
        }
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
    object _name;
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
        _vaxes = boost::python::list();
        _vresolution = toPyVector3(Vector(0.02,0.02,0.02));
        _vmaxvel = toPyVector3(Vector(10,10,10));
        _vhardmaxvel = toPyVector3(Vector(10,10,10));
        _vmaxaccel = toPyVector3(Vector(50,50,50));
        _vmaxtorque = toPyVector3(Vector(1e5,1e5,1e5));
        _vweights = toPyVector3(Vector(1,1,1));
        _voffsets = toPyVector3(Vector(0,0,0));
        _vlowerlimit = toPyVector3(Vector(0,0,0));
        _vupperlimit = toPyVector3(Vector(0,0,0));
        _bIsCircular = boost::python::list();
        _bIsActive = true;
    }

    PyJointInfo(const KinBody::JointInfo& info, PyEnvironmentBasePtr pyenv) {
        _type = info._type;
        _name = ConvertStringToUnicode(info._name);
        _linkname0 = ConvertStringToUnicode(info._linkname0);
        _linkname1 = ConvertStringToUnicode(info._linkname1);
        _vanchor = toPyVector3(info._vanchor);
        boost::python::list vaxes;
        for(size_t i = 0; i < info._vaxes.size(); ++i) {
            vaxes.append(toPyVector3(info._vaxes[i]));
        }
        _vaxes = vaxes;
        _vcurrentvalues = toPyArray(info._vcurrentvalues);
        _vresolution = toPyArray<dReal,3>(info._vresolution);
        _vmaxvel = toPyArray<dReal,3>(info._vmaxvel);
        _vhardmaxvel = toPyArray<dReal,3>(info._vhardmaxvel);
        _vmaxaccel = toPyArray<dReal,3>(info._vmaxaccel);
        _vmaxtorque = toPyArray<dReal,3>(info._vmaxtorque);
        _vweights = toPyArray<dReal,3>(info._vweights);
        _voffsets = toPyArray<dReal,3>(info._voffsets);
        _vlowerlimit = toPyArray<dReal,3>(info._vlowerlimit);
        _vupperlimit = toPyArray<dReal,3>(info._vupperlimit);
        _trajfollow = object(toPyTrajectory(info._trajfollow, pyenv));
        FOREACHC(itmimic, info._vmimic) {
            if( !*itmimic ) {
                _vmimic.append(boost::python::object());
            }
            else {
                boost::python::list oequations;
                FOREACHC(itequation, (*itmimic)->_equations) {
                    oequations.append(*itequation);
                }
                _vmimic.append(oequations);
            }
        }
        FOREACHC(it, info._mapFloatParameters) {
            _mapFloatParameters[it->first] = toPyArray(it->second);
        }
        FOREACHC(it, info._mapIntParameters) {
            _mapIntParameters[it->first] = toPyArray(it->second);
        }
        boost::python::list bIsCircular;
        FOREACHC(it, info._bIsCircular) {
            bIsCircular.append(*it);
        }
        _bIsCircular = bIsCircular;
        _bIsActive = info._bIsActive;
    }

    KinBody::JointInfoPtr GetJointInfo() {
        KinBody::JointInfoPtr pinfo(new KinBody::JointInfo());
        KinBody::JointInfo& info = *pinfo;
        info._type = _type;
        if( !(_name == boost::python::object()) ) {
            info._name = boost::python::extract<std::string>(_name);
        }
        if( !(_linkname0 == boost::python::object()) ) {
            info._linkname0 = boost::python::extract<std::string>(_linkname0);
        }
        if( !(_linkname1 == boost::python::object()) ) {
            info._linkname1 = boost::python::extract<std::string>(_linkname1);
        }
        info._vanchor = ExtractVector3(_vanchor);
        size_t num = len(_vaxes);
        for(size_t i = 0; i < num; ++i) {
            info._vaxes.at(i) = ExtractVector3(_vaxes[i]);
        }
        if( !(_vcurrentvalues == object()) ) {
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
        if( !(_trajfollow == boost::python::object()) ) {
            info._trajfollow = GetTrajectory(_trajfollow);
        }
        if( !(_vmimic == object()) ) {
            num = len(_vmimic);
            for(size_t i = 0; i < num; ++i) {
                if( !(_vmimic[i] == object()) ) {
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
        info._bIsActive = _bIsActive;
        return pinfo;
    }
    KinBody::JointType _type;
    object _name;
    object _linkname0, _linkname1;
    object _vanchor, _vaxes, _vcurrentvalues, _vresolution, _vmaxvel, _vhardmaxvel, _vmaxaccel, _vmaxtorque, _vweights, _voffsets, _vlowerlimit, _vupperlimit;
    object _trajfollow;
    boost::python::list _vmimic;
    boost::python::dict _mapFloatParameters, _mapIntParameters;
    object _bIsCircular;
    bool _bIsActive;
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
        object GetInfo() {
            return object(PyGeometryInfoPtr(new PyGeometryInfo(_pgeometry->GetInfo())));
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

    object GetParent() const
    {
        KinBodyPtr parent = _plink->GetParent();
        if( parent->IsRobot() ) {
            return object(toPyRobot(RaveInterfaceCast<RobotBase>(_plink->GetParent()),_pyenv));
        }
        else {
            return object(PyKinBodyPtr(new PyKinBody(_plink->GetParent(),_pyenv)));
        }
    }

    object GetParentLinks() const
    {
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
        for(size_t i = 0; i < N; ++i) {
            geoms.append(boost::shared_ptr<PyGeometry>(new PyGeometry(_plink->GetGeometry(i))));
        }
        return geoms;
    }

    void InitGeometries(object ogeometryinfos)
    {
        std::vector<KinBody::GeometryInfoConstPtr> geometries(len(ogeometryinfos));
        for(size_t i = 0; i < geometries.size(); ++i) {
            PyGeometryInfoPtr pygeom = boost::python::extract<PyGeometryInfoPtr>(ogeometryinfos[i]);
            if( !pygeom ) {
                throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.GeometryInfo",ORE_InvalidArguments);
            }
            geometries[i] = pygeom->GetGeometryInfo();
        }
        return _plink->InitGeometries(geometries);
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

    boost::python::object GetFloatParameters(object oname=boost::python::object(), int index=-1) const {
        return GetCustomParameters(_plink->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, object oparameters)
    {
        _plink->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    boost::python::object GetIntParameters(object oname=boost::python::object(), int index=-1) const {
        return GetCustomParameters(_plink->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, object oparameters)
    {
        _plink->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    void UpdateInfo() {
        _plink->UpdateInfo();
    }
    object GetInfo() {
        return object(PyLinkInfoPtr(new PyLinkInfo(_plink->GetInfo())));
    }
    object UpdateAndGetInfo() {
        return object(PyLinkInfoPtr(new PyLinkInfo(_plink->UpdateAndGetInfo())));
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
    dReal GetValue(int iaxis) const {
        return _pjoint->GetValue(iaxis);
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

    boost::python::object GetFloatParameters(object oname=boost::python::object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetFloatParameters(), oname, index);
    }

    void SetFloatParameters(const std::string& key, object oparameters)
    {
        _pjoint->SetFloatParameters(key,ExtractArray<dReal>(oparameters));
    }

    boost::python::object GetIntParameters(object oname=boost::python::object(), int index=-1) const {
        return GetCustomParameters(_pjoint->GetIntParameters(), oname, index);
    }

    void SetIntParameters(const std::string& key, object oparameters)
    {
        _pjoint->SetIntParameters(key,ExtractArray<int>(oparameters));
    }

    void UpdateInfo() {
        _pjoint->UpdateInfo();
    }
    object GetInfo() {
        return object(PyJointInfoPtr(new PyJointInfo(_pjoint->GetInfo(), _pyenv)));
    }
    object UpdateAndGetInfo() {
        return object(PyJointInfoPtr(new PyJointInfo(_pjoint->UpdateAndGetInfo(), _pyenv)));
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

class PyKinBodyStateSaver
{
    PyEnvironmentBasePtr _pyenv;
    KinBody::KinBodyStateSaver _state;
public:
    PyKinBodyStateSaver(PyKinBodyPtr pybody) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody()) {
    }
    PyKinBodyStateSaver(PyKinBodyPtr pybody, object options) : _pyenv(pybody->GetEnv()), _state(pybody->GetBody(),pyGetIntFromPy(options,0)) {
    }
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : _pyenv(pyenv), _state(pbody) {
    }
    PyKinBodyStateSaver(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv, object options) : _pyenv(pyenv), _state(pbody,pyGetIntFromPy(options, 0)) {
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

PyKinBody::PyKinBody(KinBodyPtr pbody, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pbody,pyenv), _pbody(pbody)
{
}

PyKinBody::PyKinBody(const PyKinBody& r) : PyInterfaceBase(r._pbody,r._pyenv)
{
    _pbody = r._pbody;
}
PyKinBody::~PyKinBody()
{
}
KinBodyPtr PyKinBody::GetBody()
{
    return _pbody;
}

bool PyKinBody::InitFromBoxes(const boost::multi_array<dReal,2>& vboxes, bool bDraw)
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
bool PyKinBody::InitFromSpheres(const boost::multi_array<dReal,2>& vspheres, bool bDraw)
{
    if( vspheres.shape()[1] != 4 ) {
        throw openrave_exception("spheres needs to be a Nx4 vector\n");
    }
    std::vector<Vector> vvspheres(vspheres.shape()[0]);
    for(size_t i = 0; i < vvspheres.size(); ++i) {
        vvspheres[i] = Vector(vspheres[i][0],vspheres[i][1],vspheres[i][2],vspheres[i][3]);
    }
    return _pbody->InitFromSpheres(vvspheres,bDraw);
}

bool PyKinBody::InitFromTrimesh(object pytrimesh, bool bDraw)
{
    TriMesh mesh;
    if( ExtractTriMesh(pytrimesh,mesh) ) {
        return _pbody->InitFromTrimesh(mesh,bDraw);
    }
    else {
        throw openrave_exception("bad trimesh");
    }
}

bool PyKinBody::InitFromGeometries(object ogeometries)
{
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

void PyKinBody::_ParseLinkInfos(object olinkinfos, std::vector<KinBody::LinkInfoConstPtr>& vlinkinfos)
{
    vlinkinfos.resize(len(olinkinfos));
    for(size_t i = 0; i < vlinkinfos.size(); ++i) {
        PyLinkInfoPtr pylink = boost::python::extract<PyLinkInfoPtr>(olinkinfos[i]);
        if( !pylink ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.LinkInfo",ORE_InvalidArguments);
        }
        vlinkinfos[i] = pylink->GetLinkInfo();
    }
}

void PyKinBody::_ParseJointInfos(object ojointinfos, std::vector<KinBody::JointInfoConstPtr>& vjointinfos)
{
    vjointinfos.resize(len(ojointinfos));
    for(size_t i = 0; i < vjointinfos.size(); ++i) {
        PyJointInfoPtr pyjoint = boost::python::extract<PyJointInfoPtr>(ojointinfos[i]);
        if( !pyjoint ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("cannot cast to KinBody.JointInfo",ORE_InvalidArguments);
        }
        vjointinfos[i] = pyjoint->GetJointInfo();
    }
}

bool PyKinBody::Init(object olinkinfos, object ojointinfos)
{
    std::vector<KinBody::LinkInfoConstPtr> vlinkinfos;
    _ParseLinkInfos(olinkinfos, vlinkinfos);
    std::vector<KinBody::JointInfoConstPtr> vjointinfos;
    _ParseJointInfos(ojointinfos, vjointinfos);
    return _pbody->Init(vlinkinfos, vjointinfos);
}

void PyKinBody::SetName(const std::string& name)
{
    _pbody->SetName(name);
}
object PyKinBody::GetName() const
{
    return ConvertStringToUnicode(_pbody->GetName());
}
int PyKinBody::GetDOF() const
{
    return _pbody->GetDOF();
}

object PyKinBody::GetDOFValues() const
{
    vector<dReal> values;
    _pbody->GetDOFValues(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFValues(object oindices) const
{
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

object PyKinBody::GetDOFVelocities() const
{
    vector<dReal> values;
    _pbody->GetDOFVelocities(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFVelocities(object oindices) const
{
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

object PyKinBody::GetDOFLimits() const
{
    vector<dReal> vlower, vupper;
    _pbody->GetDOFLimits(vlower,vupper);
    return boost::python::make_tuple(toPyArray(vlower),toPyArray(vupper));
}

object PyKinBody::GetDOFVelocityLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFVelocityLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFAccelerationLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFAccelerationLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFTorqueLimits() const
{
    vector<dReal> vmax;
    _pbody->GetDOFTorqueLimits(vmax);
    return toPyArray(vmax);
}

object PyKinBody::GetDOFLimits(object oindices) const
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

object PyKinBody::GetDOFVelocityLimits(object oindices) const
{
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

object PyKinBody::GetDOFAccelerationLimits(object oindices) const
{
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

object PyKinBody::GetDOFTorqueLimits(object oindices) const
{
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

object PyKinBody::GetDOFMaxVel() const
{
    RAVELOG_WARN("KinBody.GetDOFMaxVel() is deprecated, use GetDOFVelocityLimits\n");
    vector<dReal> values;
    _pbody->GetDOFVelocityLimits(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFMaxTorque() const
{
    vector<dReal> values;
    _pbody->GetDOFMaxTorque(values);
    return toPyArray(values);
}
object PyKinBody::GetDOFMaxAccel() const
{
    RAVELOG_WARN("KinBody.GetDOFMaxAccel() is deprecated, use GetDOFAccelerationLimits\n");
    vector<dReal> values;
    _pbody->GetDOFAccelerationLimits(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFWeights() const
{
    vector<dReal> values;
    _pbody->GetDOFWeights(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFWeights(object oindices) const
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
        values.push_back(pjoint->GetWeight(*it-pjoint->GetDOFIndex()));
    }
    return toPyArray(values);
}

object PyKinBody::GetDOFResolutions() const
{
    vector<dReal> values;
    _pbody->GetDOFResolutions(values);
    return toPyArray(values);
}

object PyKinBody::GetDOFResolutions(object oindices) const
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
        values.push_back(pjoint->GetResolution());
    }
    return toPyArray(values);
}

object PyKinBody::GetLinks() const
{
    boost::python::list links;
    FOREACHC(itlink, _pbody->GetLinks()) {
        links.append(PyLinkPtr(new PyLink(*itlink, GetEnv())));
    }
    return links;
}

object PyKinBody::GetLinks(object oindices) const
{
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

object PyKinBody::GetLink(const std::string& linkname) const
{
    KinBody::LinkPtr plink = _pbody->GetLink(linkname);
    return !plink ? object() : object(PyLinkPtr(new PyLink(plink,GetEnv())));
}

object PyKinBody::GetJoints() const
{
    boost::python::list joints;
    FOREACHC(itjoint, _pbody->GetJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetJoints(object oindices) const
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

object PyKinBody::GetPassiveJoints()
{
    boost::python::list joints;
    FOREACHC(itjoint, _pbody->GetPassiveJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetDependencyOrderedJoints()
{
    boost::python::list joints;
    FOREACHC(itjoint, _pbody->GetDependencyOrderedJoints()) {
        joints.append(PyJointPtr(new PyJoint(*itjoint, GetEnv())));
    }
    return joints;
}

object PyKinBody::GetClosedLoops()
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

object PyKinBody::GetRigidlyAttachedLinks(int linkindex) const
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

object PyKinBody::GetChain(int linkindex1, int linkindex2,bool returnjoints) const
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

bool PyKinBody::IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const
{
    return _pbody->IsDOFInChain(linkindex1,linkindex2,dofindex);
}

int PyKinBody::GetJointIndex(const std::string& jointname) const
{
    return _pbody->GetJointIndex(jointname);
}

object PyKinBody::GetJoint(const std::string& jointname) const
{
    KinBody::JointPtr pjoint = _pbody->GetJoint(jointname);
    return !pjoint ? object() : object(PyJointPtr(new PyJoint(pjoint,GetEnv())));
}

object PyKinBody::GetJointFromDOFIndex(int dofindex) const
{
    KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(dofindex);
    return !pjoint ? object() : object(PyJointPtr(new PyJoint(pjoint,GetEnv())));
}

object PyKinBody::GetTransform() const {
    return ReturnTransform(_pbody->GetTransform());
}

object PyKinBody::GetLinkTransformations(bool returndofbranches) const
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

void PyKinBody::SetLinkTransformations(object transforms, object odofbranches)
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

void PyKinBody::SetLinkVelocities(object ovelocities)
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

bool PyKinBody::SetVelocity(object olinearvel, object oangularvel)
{
    return _pbody->SetVelocity(ExtractVector3(olinearvel),ExtractVector3(oangularvel));
}

void PyKinBody::SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel, uint32_t checklimits)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel),checklimits);
}

void PyKinBody::SetDOFVelocities(object odofvelocities, object olinearvel, object oangularvel)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities),ExtractVector3(olinearvel),ExtractVector3(oangularvel));
}

void PyKinBody::SetDOFVelocities(object odofvelocities)
{
    _pbody->SetDOFVelocities(ExtractArray<dReal>(odofvelocities));
}

void PyKinBody::SetDOFVelocities(object odofvelocities, uint32_t checklimits, object oindices)
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

object PyKinBody::GetLinkVelocities() const
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

object PyKinBody::GetLinkAccelerations(object odofaccelerations) const
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

object PyKinBody::ComputeAABB()
{
    return toPyAABB(_pbody->ComputeAABB());
}
void PyKinBody::Enable(bool bEnable)
{
    _pbody->Enable(bEnable);
}
bool PyKinBody::IsEnabled() const
{
    return _pbody->IsEnabled();
}
bool PyKinBody::SetVisible(bool visible)
{
    return _pbody->SetVisible(visible);
}
bool PyKinBody::IsVisible() const
{
    return _pbody->IsVisible();
}
void PyKinBody::SetTransform(object transform)
{
    _pbody->SetTransform(ExtractTransform(transform));
}

void PyKinBody::SetDOFWeights(object o)
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

void PyKinBody::SetDOFLimits(object olower, object oupper)
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

void PyKinBody::SetDOFVelocityLimits(object o)
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

void PyKinBody::SetDOFAccelerationLimits(object o)
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

void PyKinBody::SetDOFTorqueLimits(object o)
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

void PyKinBody::SetDOFValues(object o)
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
void PyKinBody::SetTransformWithDOFValues(object otrans,object ojoints)
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

void PyKinBody::SetDOFValues(object o, object indices, uint32_t checklimits)
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

void PyKinBody::SetDOFValues(object o, object indices)
{
    SetDOFValues(o,indices,true);
}

object PyKinBody::SubtractDOFValues(object ovalues0, object ovalues1)
{
    vector<dReal> values0 = ExtractArray<dReal>(ovalues0);
    vector<dReal> values1 = ExtractArray<dReal>(ovalues1);
    BOOST_ASSERT((int)values0.size() == GetDOF() );
    BOOST_ASSERT((int)values1.size() == GetDOF() );
    _pbody->SubtractDOFValues(values0,values1);
    return toPyArray(values0);
}

void PyKinBody::SetDOFTorques(object otorques, bool bAdd)
{
    vector<dReal> vtorques = ExtractArray<dReal>(otorques);
    BOOST_ASSERT((int)vtorques.size() == GetDOF() );
    _pbody->SetDOFTorques(vtorques,bAdd);
}

object PyKinBody::ComputeJacobianTranslation(int index, object oposition, object oindices)
{
    vector<int> vindices;
    if( !(oindices == object()) ) {
        vindices = ExtractArray<int>(oindices);
    }
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianTranslation(index,ExtractVector3(oposition),vjacobian,vindices);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::ComputeJacobianAxisAngle(int index, object oindices)
{
    vector<int> vindices;
    if( !(oindices == object()) ) {
        vindices = ExtractArray<int>(oindices);
    }
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianAxisAngle(index,vjacobian,vindices);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateJacobian(int index, object oposition)
{
    std::vector<dReal> vjacobian;
    _pbody->CalculateJacobian(index,ExtractVector3(oposition),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = vjacobian.size()/3;
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateRotationJacobian(int index, object q) const
{
    std::vector<dReal> vjacobian;
    _pbody->CalculateRotationJacobian(index,ExtractVector4(q),vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 4; dims[1] = _pbody->GetDOF();
    return toPyArray(vjacobian,dims);
}

object PyKinBody::CalculateAngularVelocityJacobian(int index) const
{
    std::vector<dReal> vjacobian;
    _pbody->ComputeJacobianAxisAngle(index,vjacobian);
    std::vector<npy_intp> dims(2); dims[0] = 3; dims[1] = _pbody->GetDOF();
    return toPyArray(vjacobian,dims);
}

object PyKinBody::ComputeHessianTranslation(int index, object oposition, object oindices)
{
    vector<int> vindices;
    if( !(oindices == object()) ) {
        vindices = ExtractArray<int>(oindices);
    }
    size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
    std::vector<dReal> vhessian;
    _pbody->ComputeHessianTranslation(index,ExtractVector3(oposition),vhessian,vindices);
    std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
    return toPyArray(vhessian,dims);
}

object PyKinBody::ComputeHessianAxisAngle(int index, object oindices)
{
    vector<int> vindices;
    if( !(oindices == object()) ) {
        vindices = ExtractArray<int>(oindices);
    }
    size_t dof = vindices.size() == 0 ? (size_t)_pbody->GetDOF() : vindices.size();
    std::vector<dReal> vhessian;
    _pbody->ComputeHessianAxisAngle(index,vhessian,vindices);
    std::vector<npy_intp> dims(3); dims[0] = dof; dims[1] = 3; dims[2] = dof;
    return toPyArray(vhessian,dims);
}

object PyKinBody::ComputeInverseDynamics(object odofaccelerations, object oexternalforcetorque, bool returncomponents)
{
    vector<dReal> vDOFAccelerations;
    if( !(odofaccelerations == object()) ) {
        vDOFAccelerations = ExtractArray<dReal>(odofaccelerations);
    }
    KinBody::ForceTorqueMap mapExternalForceTorque;
    if( !(oexternalforcetorque == object()) ) {
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

bool PyKinBody::CheckSelfCollision() {
    return _pbody->CheckSelfCollision();
}
bool PyKinBody::CheckSelfCollision(PyCollisionReportPtr pReport)
{
    bool bCollision = _pbody->CheckSelfCollision(openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,GetEnv());
    return bCollision;
}

bool PyKinBody::IsAttached(PyKinBodyPtr pattachbody)
{
    CHECK_POINTER(pattachbody);
    return _pbody->IsAttached(pattachbody->GetBody());
}
object PyKinBody::GetAttached() const
{
    boost::python::list attached;
    std::set<KinBodyPtr> vattached;
    _pbody->GetAttached(vattached);
    FOREACHC(it,vattached)
    attached.append(PyKinBodyPtr(new PyKinBody(*it,_pyenv)));
    return attached;
}

void PyKinBody::SetZeroConfiguration()
{
    _pbody->SetZeroConfiguration();
}
void PyKinBody::SetNonCollidingConfiguration()
{
    _pbody->SetNonCollidingConfiguration();
}

object PyKinBody::GetConfigurationSpecification(const std::string& interpolation) const
{
    return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecification(interpolation)));
}

object PyKinBody::GetConfigurationSpecificationIndices(object oindices,const std::string& interpolation) const
{
    vector<int> vindices = ExtractArray<int>(oindices);
    return object(openravepy::toPyConfigurationSpecification(_pbody->GetConfigurationSpecificationIndices(vindices,interpolation)));
}

void PyKinBody::SetConfigurationValues(object ovalues, uint32_t checklimits)
{
    vector<dReal> vvalues = ExtractArray<dReal>(ovalues);
    BOOST_ASSERT((int)vvalues.size()==_pbody->GetDOF()+7);
    _pbody->SetConfigurationValues(vvalues.begin(),checklimits);
}

object PyKinBody::GetConfigurationValues() const
{
    vector<dReal> values;
    _pbody->GetConfigurationValues(values);
    return toPyArray(values);
}

bool PyKinBody::IsRobot() const
{
    return _pbody->IsRobot();
}
int PyKinBody::GetEnvironmentId() const
{
    return _pbody->GetEnvironmentId();
}

int PyKinBody::DoesAffect(int jointindex, int linkindex ) const
{
    return _pbody->DoesAffect(jointindex,linkindex);
}

object PyKinBody::GetViewerData() const
{
    return toPyUserData(_pbody->GetViewerData());
}

object PyKinBody::GetURI() const
{
    return ConvertStringToUnicode(_pbody->GetURI());
}

object PyKinBody::GetNonAdjacentLinks() const
{
    boost::python::list ononadjacent;
    const std::set<int>& nonadjacent = _pbody->GetNonAdjacentLinks();
    FOREACHC(it,nonadjacent) {
        ononadjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    }
    return ononadjacent;
}
object PyKinBody::GetNonAdjacentLinks(int adjacentoptions) const
{
    boost::python::list ononadjacent;
    const std::set<int>& nonadjacent = _pbody->GetNonAdjacentLinks(adjacentoptions);
    FOREACHC(it,nonadjacent) {
        ononadjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    }
    return ononadjacent;
}

object PyKinBody::GetAdjacentLinks() const
{
    boost::python::list adjacent;
    FOREACHC(it,_pbody->GetAdjacentLinks())
    adjacent.append(boost::python::make_tuple((int)(*it)&0xffff,(int)(*it)>>16));
    return adjacent;
}

object PyKinBody::GetPhysicsData() const
{
    return toPyUserData(_pbody->GetPhysicsData());
}
object PyKinBody::GetCollisionData() const
{
    return toPyUserData(_pbody->GetCollisionData());
}
object PyKinBody::GetManageData() const
{
    KinBody::ManageDataPtr pdata = _pbody->GetManageData();
    return !pdata ? object() : object(PyManageDataPtr(new PyManageData(pdata,_pyenv)));
}
int PyKinBody::GetUpdateStamp() const
{
    return _pbody->GetUpdateStamp();
}

string PyKinBody::serialize(int options) const
{
    stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    _pbody->serialize(ss,options);
    return ss.str();
}

string PyKinBody::GetKinematicsGeometryHash() const
{
    return _pbody->GetKinematicsGeometryHash();
}

PyStateRestoreContextBase* PyKinBody::CreateKinBodyStateSaver(object options)
{
    PyKinBodyStateSaverPtr saver;
    if( options == object() ) {
        saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv));
    }
    else {
        saver.reset(new PyKinBodyStateSaver(_pbody,_pyenv,options));
    }
    return new PyStateRestoreContext<PyKinBodyStateSaverPtr, PyKinBodyPtr>(saver);
}

string PyKinBody::__repr__()
{
    return boost::str(boost::format("RaveGetEnvironment(%d).GetKinBody('%s')")%RaveGetEnvironmentId(_pbody->GetEnv())%_pbody->GetName());
}
string PyKinBody::__str__()
{
    return boost::str(boost::format("<%s:%s - %s (%s)>")%RaveGetInterfaceName(_pbody->GetInterfaceType())%_pbody->GetXMLId()%_pbody->GetName()%_pbody->GetKinematicsGeometryHash());
}

object PyKinBody::__unicode__()
{
    return ConvertStringToUnicode(__str__());
}

void PyKinBody::__enter__()
{
    // necessary to lock physics to prevent multiple threads from interfering
    if( _listStateSavers.size() == 0 ) {
        openravepy::LockEnvironment(_pyenv);
    }
    _listStateSavers.push_back(boost::shared_ptr<void>(new KinBody::KinBodyStateSaver(_pbody)));
}

void PyKinBody::__exit__(object type, object value, object traceback)
{
    BOOST_ASSERT(_listStateSavers.size()>0);
    _listStateSavers.pop_back();
    if( _listStateSavers.size() == 0 ) {
        openravepy::UnlockEnvironment(_pyenv);
    }
}

object toPyKinBodyLink(KinBody::LinkPtr plink, PyEnvironmentBasePtr pyenv)
{
    if( !plink ) {
        return object();
    }
    return object(PyLinkPtr(new PyLink(plink,pyenv)));
}

object toPyKinBodyLink(KinBody::LinkPtr plink, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyKinBodyLink(plink,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
}

object toPyKinBodyJoint(KinBody::JointPtr pjoint, PyEnvironmentBasePtr pyenv)
{
    if( !pjoint ) {
        return object();
    }
    return object(PyJointPtr(new PyJoint(pjoint,pyenv)));
}

KinBody::LinkPtr GetKinBodyLink(object o)
{
    extract<PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkPtr();
}

KinBody::LinkConstPtr GetKinBodyLinkConst(object o)
{
    extract<PyLinkPtr> pylink(o);
    if( pylink.check() ) {
        return ((PyLinkPtr)pylink)->GetLink();
    }
    return KinBody::LinkConstPtr();
}

KinBody::JointPtr GetKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->GetJoint();
    }
    return KinBody::JointPtr();
}

std::string reprPyKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->__repr__();
    }
    return std::string();
}

std::string strPyKinBodyJoint(object o)
{
    extract<PyJointPtr> pyjoint(o);
    if( pyjoint.check() ) {
        return ((PyJointPtr)pyjoint)->__str__();
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

object toPyKinBody(KinBodyPtr pkinbody, object opyenv)
{
    extract<PyEnvironmentBasePtr> pyenv(opyenv);
    if( pyenv.check() ) {
        return object(toPyKinBody(pkinbody,(PyEnvironmentBasePtr)pyenv));
    }
    return object();
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
    static tuple getstate(const PyGeometryInfo& r)
    {
        return boost::python::make_tuple(r._t, r._vGeomData, r._vDiffuseColor, r._vAmbientColor, r._meshcollision, r._type, r._filenamerender, r._filenamecollision, r._vRenderScale, r._vCollisionScale, r._fTransparency, r._bVisible, r._bModifiable);
    }
    static void setstate(PyGeometryInfo& r, boost::python::tuple state) {
        r._t = state[0];
        r._vGeomData = state[1];
        r._vDiffuseColor = state[2];
        r._vAmbientColor = state[3];
        r._meshcollision = state[4];
        r._type = boost::python::extract<GeometryType>(state[5]);
        r._filenamerender = state[6];
        r._filenamecollision = state[7];
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
    static tuple getstate(const PyLinkInfo& r)
    {
        return boost::python::make_tuple(r._vgeometryinfos, r._name, r._t, r._tMassFrame, r._mass, r._vinertiamoments, r._mapFloatParameters, r._mapIntParameters, r._vForcedAdjacentLinks, r._bStatic, r._bIsEnabled);
    }
    static void setstate(PyLinkInfo& r, boost::python::tuple state) {
        r._vgeometryinfos = boost::python::list(state[0]);
        r._name = state[1];
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
    static tuple getstate(const PyJointInfo& r)
    {
        return boost::python::make_tuple(boost::python::make_tuple(r._type, r._name, r._linkname0, r._linkname1, r._vanchor, r._vaxes, r._vcurrentvalues), boost::python::make_tuple(r._vresolution, r._vmaxvel, r._vhardmaxvel, r._vmaxaccel, r._vmaxtorque, r._vweights, r._voffsets, r._vlowerlimit, r._vupperlimit), boost::python::make_tuple(r._trajfollow, r._vmimic, r._mapFloatParameters, r._mapIntParameters, r._bIsCircular, r._bIsActive));
    }
    static void setstate(PyJointInfo& r, boost::python::tuple state) {
        r._type = boost::python::extract<KinBody::JointType>(state[0][0]);
        r._name = state[0][1];
        r._linkname0 = state[0][2];
        r._linkname1 = state[0][3];
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
        r._bIsActive = boost::python::extract<bool>(state[2][5]);
    }
};

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(IsMimic_overloads, IsMimic, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicEquation_overloads, GetMimicEquation, 0, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetMimicDOFIndices_overloads, GetMimicDOFIndices, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetChain_overloads, GetChain, 2, 3)
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
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianTranslation_overloads, ComputeJacobianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeJacobianAxisAngle_overloads, ComputeJacobianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianTranslation_overloads, ComputeHessianTranslation, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeHessianAxisAngle_overloads, ComputeHessianAxisAngle, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ComputeInverseDynamics_overloads, ComputeInverseDynamics, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Restore_overloads, Restore, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CreateKinBodyStateSaver_overloads, CreateKinBodyStateSaver, 0,1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetConfigurationValues_overloads, SetConfigurationValues, 1,2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetFloatParameters_overloads, GetFloatParameters, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetIntParameters_overloads, GetIntParameters, 0, 2)

void init_openravepy_kinbody()
{
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
        object geometryinfo = class_<PyGeometryInfo, boost::shared_ptr<PyGeometryInfo> >("GeometryInfo", DOXY_CLASS(KinBody::GeometryInfo))
                              .def_readwrite("_t",&PyGeometryInfo::_t)
                              .def_readwrite("_vGeomData",&PyGeometryInfo::_vGeomData)
                              .def_readwrite("_vDiffuseColor",&PyGeometryInfo::_vDiffuseColor)
                              .def_readwrite("_vAmbientColor",&PyGeometryInfo::_vAmbientColor)
                              .def_readwrite("_meshcollision",&PyGeometryInfo::_meshcollision)
                              .def_readwrite("_type",&PyGeometryInfo::_type)
                              .def_readwrite("_filenamerender",&PyGeometryInfo::_filenamerender)
                              .def_readwrite("_filenamecollision",&PyGeometryInfo::_filenamecollision)
                              .def_readwrite("_vRenderScale",&PyGeometryInfo::_vRenderScale)
                              .def_readwrite("_vCollisionScale",&PyGeometryInfo::_vCollisionScale)
                              .def_readwrite("_fTransparency",&PyGeometryInfo::_fTransparency)
                              .def_readwrite("_bVisible",&PyGeometryInfo::_bVisible)
                              .def_readwrite("_bModifiable",&PyGeometryInfo::_bModifiable)
                              .def_pickle(GeometryInfo_pickle_suite())
        ;
        object linkinfo = class_<PyLinkInfo, boost::shared_ptr<PyLinkInfo> >("LinkInfo", DOXY_CLASS(KinBody::LinkInfo))
                          .def_readwrite("_vgeometryinfos",&PyLinkInfo::_vgeometryinfos)
                          .def_readwrite("_name",&PyLinkInfo::_name)
                          .def_readwrite("_t",&PyLinkInfo::_t)
                          .def_readwrite("_tMassFrame",&PyLinkInfo::_tMassFrame)
                          .def_readwrite("_mass",&PyLinkInfo::_mass)
                          .def_readwrite("_vinertiamoments",&PyLinkInfo::_vinertiamoments)
                          .def_readwrite("_mapFloatParameters",&PyLinkInfo::_mapFloatParameters)
                          .def_readwrite("_mapIntParameters",&PyLinkInfo::_mapIntParameters)
                          .def_readwrite("_vForcedAdjacentLinks",&PyLinkInfo::_vForcedAdjacentLinks)
                          .def_readwrite("_bStatic",&PyLinkInfo::_bStatic)
                          .def_readwrite("_bIsEnabled",&PyLinkInfo::_bIsEnabled)
                          .def_pickle(LinkInfo_pickle_suite())
        ;
        object jointinfo = class_<PyJointInfo, boost::shared_ptr<PyJointInfo> >("JointInfo", DOXY_CLASS(KinBody::JointInfo))
                           .def_readwrite("_type",&PyJointInfo::_type)
                           .def_readwrite("_name",&PyJointInfo::_name)
                           .def_readwrite("_linkname0",&PyJointInfo::_linkname0)
                           .def_readwrite("_linkname1",&PyJointInfo::_linkname1)
                           .def_readwrite("_vanchor",&PyJointInfo::_vanchor)
                           .def_readwrite("_vaxes",&PyJointInfo::_vaxes)
                           .def_readwrite("_vcurrentvalues",&PyJointInfo::_vcurrentvalues)
                           .def_readwrite("_vresolution",&PyJointInfo::_vresolution)
                           .def_readwrite("_vmaxvel",&PyJointInfo::_vmaxvel)
                           .def_readwrite("_vhardmaxvel",&PyJointInfo::_vhardmaxvel)
                           .def_readwrite("_vmaxaccel",&PyJointInfo::_vmaxaccel)
                           .def_readwrite("_vmaxtorque",&PyJointInfo::_vmaxtorque)
                           .def_readwrite("_vweights",&PyJointInfo::_vweights)
                           .def_readwrite("_voffsets",&PyJointInfo::_voffsets)
                           .def_readwrite("_vlowerlimit",&PyJointInfo::_vlowerlimit)
                           .def_readwrite("_vupperlimit",&PyJointInfo::_vupperlimit)
                           .def_readwrite("_trajfollow",&PyJointInfo::_trajfollow)
                           .def_readwrite("_vmimic",&PyJointInfo::_vmimic)
                           .def_readwrite("_mapFloatParameters",&PyJointInfo::_mapFloatParameters)
                           .def_readwrite("_mapIntParameters",&PyJointInfo::_mapIntParameters)
                           .def_readwrite("_bIsCircular",&PyJointInfo::_bIsCircular)
                           .def_readwrite("_bIsActive",&PyJointInfo::_bIsActive)
                           .def_pickle(JointInfo_pickle_suite())
        ;
        {
            scope link = class_<PyLink, boost::shared_ptr<PyLink> >("Link", DOXY_CLASS(KinBody::Link), no_init)
                         .def("GetName",&PyLink::GetName, DOXY_FN(KinBody::Link,GetName))
                         .def("GetIndex",&PyLink::GetIndex, DOXY_FN(KinBody::Link,GetIndex))
                         .def("Enable",&PyLink::Enable,args("enable"), DOXY_FN(KinBody::Link,Enable))
                         .def("IsEnabled",&PyLink::IsEnabled, DOXY_FN(KinBody::Link,IsEnabled))
                         .def("IsStatic",&PyLink::IsStatic, DOXY_FN(KinBody::Link,IsStatic))
                         .def("SetVisible",&PyLink::SetVisible,args("visible"), DOXY_FN(KinBody::Link,SetVisible))
                         .def("IsVisible",&PyLink::IsVisible, DOXY_FN(KinBody::Link,IsVisible))
                         .def("GetParent",&PyLink::GetParent, DOXY_FN(KinBody::Link,GetParent))
                         .def("GetParentLinks",&PyLink::GetParentLinks, DOXY_FN(KinBody::Link,GetParentLinks))
                         .def("IsParentLink",&PyLink::IsParentLink, DOXY_FN(KinBody::Link,IsParentLink))
                         .def("GetCollisionData",&PyLink::GetCollisionData, DOXY_FN(KinBody::Link,GetCollisionData))
                         .def("ComputeAABB",&PyLink::ComputeAABB, DOXY_FN(KinBody::Link,ComputeAABB))
                         .def("ComputeLocalAABB",&PyLink::ComputeLocalAABB, DOXY_FN(KinBody::Link,ComputeLocalAABB))
                         .def("GetTransform",&PyLink::GetTransform, DOXY_FN(KinBody::Link,GetTransform))
                         .def("GetCOMOffset",&PyLink::GetCOMOffset, DOXY_FN(KinBody::Link,GetCOMOffset))
                         .def("GetLocalCOM",&PyLink::GetLocalCOM, DOXY_FN(KinBody::Link,GetLocalCOM))
                         .def("GetGlobalCOM",&PyLink::GetGlobalCOM, DOXY_FN(KinBody::Link,GetGlobalCOM))
                         .def("GetLocalInertia",&PyLink::GetLocalInertia, DOXY_FN(KinBody::Link,GetLocalInertia))
                         .def("GetPrincipalMomentsOfInertia",&PyLink::GetPrincipalMomentsOfInertia, DOXY_FN(KinBody::Link,GetPrincipalMomentsOfInertia))
                         .def("GetLocalMassFrame",&PyLink::GetLocalMassFrame, DOXY_FN(KinBody::Link,GetLocalMassFrame))
                         .def("GetGlobalMassFrame",&PyLink::GetGlobalMassFrame, DOXY_FN(KinBody::Link,GetGlobalMassFrame))
                         .def("GetMass",&PyLink::GetMass, DOXY_FN(KinBody::Link,GetMass))
                         .def("SetLocalMassFrame",&PyLink::SetLocalMassFrame, args("massframe"), DOXY_FN(KinBody::Link,SetLocalMassFrame))
                         .def("SetPrincipalMomentsOfInertia",&PyLink::SetPrincipalMomentsOfInertia, args("inertiamoments"), DOXY_FN(KinBody::Link,SetPrincipalMomentsOfInertia))
                         .def("SetMass",&PyLink::SetMass, args("mass"), DOXY_FN(KinBody::Link,SetMass))
                         .def("SetStatic",&PyLink::SetStatic,args("static"), DOXY_FN(KinBody::Link,SetStatic))
                         .def("SetTransform",&PyLink::SetTransform,args("transform"), DOXY_FN(KinBody::Link,SetTransform))
                         .def("SetForce",&PyLink::SetForce,args("force","pos","add"), DOXY_FN(KinBody::Link,SetForce))
                         .def("SetTorque",&PyLink::SetTorque,args("torque","add"), DOXY_FN(KinBody::Link,SetTorque))
                         .def("GetGeometries",&PyLink::GetGeometries, DOXY_FN(KinBody::Link,GetGeometries))
                         .def("InitGeometries",&PyLink::InitGeometries, args("geometries"), DOXY_FN(KinBody::Link,InitGeometries))
                         .def("GetRigidlyAttachedLinks",&PyLink::GetRigidlyAttachedLinks, DOXY_FN(KinBody::Link,GetRigidlyAttachedLinks))
                         .def("IsRigidlyAttached",&PyLink::IsRigidlyAttached, DOXY_FN(KinBody::Link,IsRigidlyAttached))
                         .def("GetVelocity",&PyLink::GetVelocity,DOXY_FN(KinBody::Link,GetVelocity))
                         .def("SetVelocity",&PyLink::SetVelocity,DOXY_FN(KinBody::Link,SetVelocity))
                         .def("GetFloatParameters",&PyLink::GetFloatParameters,GetFloatParameters_overloads(args("name","index"), DOXY_FN(KinBody::Link,GetFloatParameters)))
                         .def("SetFloatParameters",&PyLink::SetFloatParameters,DOXY_FN(KinBody::Link,SetFloatParameters))
                         .def("GetIntParameters",&PyLink::GetIntParameters,GetIntParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Link,GetIntParameters)))
                         .def("UpdateInfo",&PyLink::UpdateInfo,DOXY_FN(KinBody::Link,UpdateInfo))
                         .def("GetInfo",&PyLink::GetInfo,DOXY_FN(KinBody::Link,GetInfo))
                         .def("UpdateAndGetInfo",&PyLink::UpdateAndGetInfo,DOXY_FN(KinBody::Link,UpdateAndGetInfo))
                         .def("SetIntParameters",&PyLink::SetIntParameters,DOXY_FN(KinBody::Link,SetIntParameters))
                         .def("__repr__", &PyLink::__repr__)
                         .def("__str__", &PyLink::__str__)
                         .def("__unicode__", &PyLink::__unicode__)
                         .def("__eq__",&PyLink::__eq__)
                         .def("__ne__",&PyLink::__ne__)
                         .def("__hash__",&PyLink::__hash__)
            ;
            // \deprecated (12/10/18)
            link.attr("GeomType") = geometrytype;
            link.attr("GeometryInfo") = geometryinfo;
            {
                scope geometry = class_<PyLink::PyGeometry, boost::shared_ptr<PyLink::PyGeometry> >("Geometry", DOXY_CLASS(KinBody::Link::Geometry),no_init)
                                 .def("SetCollisionMesh",&PyLink::PyGeometry::SetCollisionMesh,args("trimesh"), DOXY_FN(KinBody::Link::Geometry,SetCollisionMesh))
                                 .def("GetCollisionMesh",&PyLink::PyGeometry::GetCollisionMesh, DOXY_FN(KinBody::Link::Geometry,GetCollisionMesh))
                                 .def("ComputeAABB",&PyLink::PyGeometry::ComputeAABB, args("transform"), DOXY_FN(KinBody::Link::Geometry,ComputeAABB))
                                 .def("SetDraw",&PyLink::PyGeometry::SetDraw,args("draw"), DOXY_FN(KinBody::Link::Geometry,SetDraw))
                                 .def("SetTransparency",&PyLink::PyGeometry::SetTransparency,args("transparency"), DOXY_FN(KinBody::Link::Geometry,SetTransparency))
                                 .def("SetDiffuseColor",&PyLink::PyGeometry::SetDiffuseColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetDiffuseColor))
                                 .def("SetAmbientColor",&PyLink::PyGeometry::SetAmbientColor,args("color"), DOXY_FN(KinBody::Link::Geometry,SetAmbientColor))
                                 .def("SetRenderFilename",&PyLink::PyGeometry::SetRenderFilename,args("color"), DOXY_FN(KinBody::Link::Geometry,SetRenderFilename))
                                 .def("IsDraw",&PyLink::PyGeometry::IsDraw, DOXY_FN(KinBody::Link::Geometry,IsDraw))
                                 .def("IsVisible",&PyLink::PyGeometry::IsVisible, DOXY_FN(KinBody::Link::Geometry,IsVisible))
                                 .def("IsModifiable",&PyLink::PyGeometry::IsModifiable, DOXY_FN(KinBody::Link::Geometry,IsModifiable))
                                 .def("GetType",&PyLink::PyGeometry::GetType, DOXY_FN(KinBody::Link::Geometry,GetType))
                                 .def("GetTransform",&PyLink::PyGeometry::GetTransform, DOXY_FN(KinBody::Link::Geometry,GetTransform))
                                 .def("GetSphereRadius",&PyLink::PyGeometry::GetSphereRadius, DOXY_FN(KinBody::Link::Geometry,GetSphereRadius))
                                 .def("GetCylinderRadius",&PyLink::PyGeometry::GetCylinderRadius, DOXY_FN(KinBody::Link::Geometry,GetCylinderRadius))
                                 .def("GetCylinderHeight",&PyLink::PyGeometry::GetCylinderHeight, DOXY_FN(KinBody::Link::Geometry,GetCylinderHeight))
                                 .def("GetBoxExtents",&PyLink::PyGeometry::GetBoxExtents, DOXY_FN(KinBody::Link::Geometry,GetBoxExtents))
                                 .def("GetRenderScale",&PyLink::PyGeometry::GetRenderScale, DOXY_FN(KinBody::Link::Geometry,GetRenderScale))
                                 .def("GetRenderFilename",&PyLink::PyGeometry::GetRenderFilename, DOXY_FN(KinBody::Link::Geometry,GetRenderFilename))
                                 .def("GetTransparency",&PyLink::PyGeometry::GetTransparency,DOXY_FN(KinBody::Link::Geometry,GetTransparency))
                                 .def("GetDiffuseColor",&PyLink::PyGeometry::GetDiffuseColor,DOXY_FN(KinBody::Link::Geometry,GetDiffuseColor))
                                 .def("GetAmbientColor",&PyLink::PyGeometry::GetAmbientColor,DOXY_FN(KinBody::Link::Geometry,GetAmbientColor))
                                 .def("GetInfo",&PyLink::PyGeometry::GetInfo,DOXY_FN(KinBody::Link::Geometry,GetInfo))
                                 .def("__eq__",&PyLink::PyGeometry::__eq__)
                                 .def("__ne__",&PyLink::PyGeometry::__ne__)
                                 .def("__hash__",&PyLink::PyGeometry::__hash__)
                ;
                // \deprecated (12/07/16)
                geometry.attr("Type") = geometrytype;
            }
            // \deprecated (12/07/16)
            link.attr("GeomProperties") = link.attr("Geometry");
        }
        {
            scope joint = class_<PyJoint, boost::shared_ptr<PyJoint> >("Joint", DOXY_CLASS(KinBody::Joint),no_init)
                          .def("GetName", &PyJoint::GetName, DOXY_FN(KinBody::Joint,GetName))
                          .def("IsMimic",&PyJoint::IsMimic,IsMimic_overloads(args("axis"), DOXY_FN(KinBody::Joint,IsMimic)))
                          .def("GetMimicEquation",&PyJoint::GetMimicEquation,GetMimicEquation_overloads(args("axis","type","format"), DOXY_FN(KinBody::Joint,GetMimicEquation)))
                          .def("GetMimicDOFIndices",&PyJoint::GetMimicDOFIndices,GetMimicDOFIndices_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetMimicDOFIndices)))
                          .def("SetMimicEquations", &PyJoint::SetMimicEquations, args("axis","poseq","veleq","acceleq"), DOXY_FN(KinBody::Joint,SetMimicEquations))
                          .def("GetMaxVel", &PyJoint::GetMaxVel, GetMaxVel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxVel)))
                          .def("GetMaxAccel", &PyJoint::GetMaxAccel, GetMaxAccel_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxAccel)))
                          .def("GetMaxTorque", &PyJoint::GetMaxTorque, GetMaxTorque_overloads(args("axis"),DOXY_FN(KinBody::Joint,GetMaxTorque)))
                          .def("GetDOFIndex", &PyJoint::GetDOFIndex, DOXY_FN(KinBody::Joint,GetDOFIndex))
                          .def("GetJointIndex", &PyJoint::GetJointIndex, DOXY_FN(KinBody::Joint,GetJointIndex))
                          .def("GetParent", &PyJoint::GetParent, DOXY_FN(KinBody::Joint,GetParent))
                          .def("GetFirstAttached", &PyJoint::GetFirstAttached, DOXY_FN(KinBody::Joint,GetFirstAttached))
                          .def("GetSecondAttached", &PyJoint::GetSecondAttached, DOXY_FN(KinBody::Joint,GetSecondAttached))
                          .def("IsStatic",&PyJoint::IsStatic, DOXY_FN(KinBody::Joint,IsStatic))
                          .def("IsCircular",&PyJoint::IsCircular, DOXY_FN(KinBody::Joint,IsCircular))
                          .def("IsRevolute",&PyJoint::IsRevolute, DOXY_FN(KinBody::Joint,IsRevolute))
                          .def("IsPrismatic",&PyJoint::IsPrismatic, DOXY_FN(KinBody::Joint,IsPrismatic))
                          .def("GetType", &PyJoint::GetType, DOXY_FN(KinBody::Joint,GetType))
                          .def("GetDOF", &PyJoint::GetDOF, DOXY_FN(KinBody::Joint,GetDOF))
                          .def("GetValues", &PyJoint::GetValues, DOXY_FN(KinBody::Joint,GetValues))
                          .def("GetValue", &PyJoint::GetValue, DOXY_FN(KinBody::Joint,GetValue))
                          .def("GetVelocities", &PyJoint::GetVelocities, DOXY_FN(KinBody::Joint,GetVelocities))
                          .def("GetAnchor", &PyJoint::GetAnchor, DOXY_FN(KinBody::Joint,GetAnchor))
                          .def("GetAxis", &PyJoint::GetAxis,GetAxis_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetAxis)))
                          .def("GetHierarchyParentLink", &PyJoint::GetHierarchyParentLink, DOXY_FN(KinBody::Joint,GetHierarchyParentLink))
                          .def("GetHierarchyChildLink", &PyJoint::GetHierarchyChildLink, DOXY_FN(KinBody::Joint,GetHierarchyChildLink))
                          .def("GetInternalHierarchyAxis", &PyJoint::GetInternalHierarchyAxis,args("axis"), DOXY_FN(KinBody::Joint,GetInternalHierarchyAxis))
                          .def("GetInternalHierarchyLeftTransform",&PyJoint::GetInternalHierarchyLeftTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyLeftTransform))
                          .def("GetInternalHierarchyRightTransform",&PyJoint::GetInternalHierarchyRightTransform, DOXY_FN(KinBody::Joint,GetInternalHierarchyRightTransform))
                          .def("GetLimits", &PyJoint::GetLimits, DOXY_FN(KinBody::Joint,GetLimits))
                          .def("GetVelocityLimits", &PyJoint::GetVelocityLimits, DOXY_FN(KinBody::Joint,GetVelocityLimits))
                          .def("GetAccelerationLimits", &PyJoint::GetAccelerationLimits, DOXY_FN(KinBody::Joint,GetAccelerationLimits))
                          .def("GetTorqueLimits", &PyJoint::GetTorqueLimits, DOXY_FN(KinBody::Joint,GetTorqueLimits))
                          .def("SetWrapOffset",&PyJoint::SetWrapOffset,SetWrapOffset_overloads(args("offset","axis"), DOXY_FN(KinBody::Joint,SetWrapOffset)))
                          .def("GetWrapOffset",&PyJoint::GetWrapOffset,GetWrapOffset_overloads(args("axis"), DOXY_FN(KinBody::Joint,GetWrapOffset)))
                          .def("SetLimits",&PyJoint::SetLimits,args("lower","upper"), DOXY_FN(KinBody::Joint,SetLimits))
                          .def("SetVelocityLimits",&PyJoint::SetVelocityLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetVelocityLimits))
                          .def("SetAccelerationLimits",&PyJoint::SetAccelerationLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetAccelerationLimits))
                          .def("SetTorqueLimits",&PyJoint::SetTorqueLimits,args("maxlimits"), DOXY_FN(KinBody::Joint,SetTorqueLimits))
                          .def("GetResolution",&PyJoint::GetResolution,args("axis"), DOXY_FN(KinBody::Joint,GetResolution))
                          .def("GetResolutions",&PyJoint::GetResolutions,DOXY_FN(KinBody::Joint,GetResolutions))
                          .def("SetResolution",&PyJoint::SetResolution,args("resolution"), DOXY_FN(KinBody::Joint,SetResolution))
                          .def("GetWeight",&PyJoint::GetWeight,args("axis"), DOXY_FN(KinBody::Joint,GetWeight))
                          .def("GetWeights",&PyJoint::GetWeights,DOXY_FN(KinBody::Joint,GetWeights))
                          .def("SetWeights",&PyJoint::SetWeights,args("weights"), DOXY_FN(KinBody::Joint,SetWeights))
                          .def("SubtractValues",&PyJoint::SubtractValues,args("values0","values1"), DOXY_FN(KinBody::Joint,SubtractValues))
                          .def("SubtractValue",&PyJoint::SubtractValue,args("value0","value1","axis"), DOXY_FN(KinBody::Joint,SubtractValue))

                          .def("AddTorque",&PyJoint::AddTorque,args("torques"), DOXY_FN(KinBody::Joint,AddTorque))
                          .def("GetFloatParameters",&PyJoint::GetFloatParameters,GetFloatParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Joint,GetFloatParameters)))
                          .def("SetFloatParameters",&PyJoint::SetFloatParameters,DOXY_FN(KinBody::Joint,SetFloatParameters))
                          .def("GetIntParameters",&PyJoint::GetIntParameters,GetIntParameters_overloads(args("name", "index"), DOXY_FN(KinBody::Joint,GetIntParameters)))
                          .def("SetIntParameters",&PyJoint::SetIntParameters,DOXY_FN(KinBody::Joint,SetIntParameters))
                          .def("UpdateInfo",&PyJoint::UpdateInfo,DOXY_FN(KinBody::Joint,UpdateInfo))
                          .def("GetInfo",&PyJoint::GetInfo,DOXY_FN(KinBody::Joint,GetInfo))
                          .def("UpdateAndGetInfo",&PyJoint::UpdateAndGetInfo,DOXY_FN(KinBody::Joint,UpdateAndGetInfo))
                          .def("__repr__", &PyJoint::__repr__)
                          .def("__str__", &PyJoint::__str__)
                          .def("__unicode__", &PyJoint::__unicode__)
                          .def("__eq__",&PyJoint::__eq__)
                          .def("__ne__",&PyJoint::__ne__)
                          .def("__hash__",&PyJoint::__hash__)
            ;
            joint.attr("Type") = jointtype;
        }

        {
            scope statesaver = class_<PyKinBodyStateSaver, boost::shared_ptr<PyKinBodyStateSaver> >("KinBodyStateSaver", DOXY_CLASS(KinBody::KinBodyStateSaver), no_init)
                               .def(init<PyKinBodyPtr>(args("body")))
                               .def(init<PyKinBodyPtr,object>(args("body","options")))
                               .def("GetBody",&PyKinBodyStateSaver::GetBody,DOXY_FN(KinBody::KinBodyStateSaver, GetBody))
                               .def("Restore",&PyKinBodyStateSaver::Restore,Restore_overloads(args("body"), DOXY_FN(KinBody::KinBodyStateSaver, Restore)))
                               .def("Release",&PyKinBodyStateSaver::Release,DOXY_FN(KinBody::KinBodyStateSaver, Release))
                               .def("__str__",&PyKinBodyStateSaver::__str__)
                               .def("__unicode__",&PyKinBodyStateSaver::__unicode__)
            ;
        }

        {
            scope managedata = class_<PyManageData, boost::shared_ptr<PyManageData> >("ManageData", DOXY_CLASS(KinBody::ManageData),no_init)
                               .def("GetSystem", &PyManageData::GetSystem, DOXY_FN(KinBody::ManageData,GetSystem))
                               .def("GetData", &PyManageData::GetData, DOXY_FN(KinBody::ManageData,GetData))
                               .def("GetOffsetLink", &PyManageData::GetOffsetLink, DOXY_FN(KinBody::ManageData,GetOffsetLink))
                               .def("IsPresent", &PyManageData::IsPresent, DOXY_FN(KinBody::ManageData,IsPresent))
                               .def("IsEnabled", &PyManageData::IsEnabled, DOXY_FN(KinBody::ManageData,IsEnabled))
                               .def("IsLocked", &PyManageData::IsLocked, DOXY_FN(KinBody::ManageData,IsLocked))
                               .def("Lock", &PyManageData::Lock,args("dolock"), DOXY_FN(KinBody::ManageData,Lock))
                               .def("__repr__", &PyManageData::__repr__)
                               .def("__str__", &PyManageData::__str__)
                               .def("__unicode__", &PyManageData::__unicode__)
                               .def("__eq__",&PyManageData::__eq__)
                               .def("__ne__",&PyManageData::__ne__)
            ;
        }
    }


    def("RaveCreateKinBody",openravepy::RaveCreateKinBody,args("env","name"),DOXY_FN1(RaveCreateKinBody));
}

}
