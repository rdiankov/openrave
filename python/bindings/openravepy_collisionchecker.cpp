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
#include <openravepy/openravepy_collisionreport.h>
#include <openravepy/openravepy_collisioncheckerbase.h>

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

PyCollisionReport::PyCollisionReport() : report(new CollisionReport()) {
}
PyCollisionReport::PyCollisionReport(CollisionReportPtr report) : report(report) {
}
PyCollisionReport::~PyCollisionReport() {
}

PyCollisionReport::PYCONTACT::PYCONTACT() {
}
PyCollisionReport::PYCONTACT::PYCONTACT(const CollisionReport::CONTACT& c)
{
    pos = toPyVector3(c.pos);
    norm = toPyVector3(c.norm);
    depth = c.depth;
}

std::string PyCollisionReport::PYCONTACT::__str__()
{
    Vector vpos = ExtractVector3(pos), vnorm = ExtractVector3(norm);
    std::stringstream ss;
    ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
    ss << "pos=["<<vpos.x<<", "<<vpos.y<<", "<<vpos.z<<"], norm=["<<vnorm.x<<", "<<vnorm.y<<", "<<vnorm.z<<"]";
    return ss.str();
}
object PyCollisionReport::PYCONTACT::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

void PyCollisionReport::init(PyEnvironmentBasePtr pyenv)
{
    options = report->options;
    minDistance = report->minDistance;
    numWithinTol = report->numWithinTol;
    nKeepPrevious = report->nKeepPrevious;
    if( !!report->plink1 ) {
        plink1 = openravepy::toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(report->plink1), pyenv);
    }
    else {
        plink1 = py::none_();
    }
    if( !!report->plink2 ) {
        plink2 = openravepy::toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(report->plink2), pyenv);
    }
    else {
        plink2 = py::none_();
    }
    py::list newcontacts;
    FOREACH(itc, report->contacts) {
        newcontacts.append(PYCONTACT(*itc));
    }
    contacts = newcontacts;

    py::list newLinkColliding;
    FOREACHC(itlinks, report->vLinkColliding) {
        object pylink1, pylink2;
        if( !!itlinks->first ) {
            pylink1 = openravepy::toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(itlinks->first), pyenv);
        }
        if( !!itlinks->second ) {
            pylink2 = openravepy::toPyKinBodyLink(OPENRAVE_CONST_POINTER_CAST<KinBody::Link>(itlinks->second), pyenv);
        }
        newLinkColliding.append(py::make_tuple(pylink1, pylink2));
    }
    vLinkColliding = newLinkColliding;
}

std::string PyCollisionReport::__str__()
{
    return report->__str__();
}
object PyCollisionReport::__unicode__() {
    return ConvertStringToUnicode(__str__());
}

PyCollisionCheckerBase::PyCollisionCheckerBase(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pCollisionChecker, pyenv), _pCollisionChecker(pCollisionChecker) {
}
PyCollisionCheckerBase::~PyCollisionCheckerBase() {
}

CollisionCheckerBasePtr PyCollisionCheckerBase::GetCollisionChecker() {
    return _pCollisionChecker;
}

bool PyCollisionCheckerBase::SetCollisionOptions(int options) {
    return _pCollisionChecker->SetCollisionOptions(options);
}
int PyCollisionCheckerBase::GetCollisionOptions() const {
    return _pCollisionChecker->GetCollisionOptions();
}

bool PyCollisionCheckerBase::InitEnvironment()
{
    return _pCollisionChecker->InitEnvironment();
}

void PyCollisionCheckerBase::DestroyEnvironment()
{
    return _pCollisionChecker->DestroyEnvironment();
}

bool PyCollisionCheckerBase::InitKinBody(PyKinBodyPtr pbody)
{
    return _pCollisionChecker->InitKinBody(openravepy::GetKinBody(pbody));
}

void PyCollisionCheckerBase::SetGeometryGroup(const std::string& groupname)
{
    _pCollisionChecker->SetGeometryGroup(groupname);
}

bool PyCollisionCheckerBase::SetBodyGeometryGroup(PyKinBodyPtr pybody, const std::string& groupname)
{
    return _pCollisionChecker->SetBodyGeometryGroup(openravepy::GetKinBody(pybody), groupname);
}

object PyCollisionCheckerBase::GetGeometryGroup()
{
    return ConvertStringToUnicode(_pCollisionChecker->GetGeometryGroup());
}

void PyCollisionCheckerBase::RemoveKinBody(PyKinBodyPtr pbody)
{
    _pCollisionChecker->RemoveKinBody(openravepy::GetKinBody(pbody));
}

bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody1)
{
    CHECK_POINTER(pbody1);
    return _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)));
}
bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(pbody1);
    bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
{
    CHECK_POINTER(pbody1);
    CHECK_POINTER(pbody2);
    return _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)));
}

bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(pbody1);
    CHECK_POINTER(pbody2);
    bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(object o1)
{
    CHECK_POINTER(o1);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        return _pCollisionChecker->CheckCollision(plink);
    }
    KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
    if( !!pbody ) {
        return _pCollisionChecker->CheckCollision(pbody);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
}

bool PyCollisionCheckerBase::CheckCollision(object o1, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    bool bCollision;
    if( !!plink ) {
        bCollision = _pCollisionChecker->CheckCollision(plink,openravepy::GetCollisionReport(pReport));
    }
    else {
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            bCollision = _pCollisionChecker->CheckCollision(pbody,openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(object o1, object o2)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(o2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            return _pCollisionChecker->CheckCollision(plink,plink2);
        }
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
        if( !!pbody2 ) {
            return _pCollisionChecker->CheckCollision(plink,pbody2);
        }
        CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
        if( !!preport2 ) {
            bool bCollision = _pCollisionChecker->CheckCollision(plink,preport2);
            openravepy::UpdateCollisionReport(o2,_pyenv);
            return bCollision;
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
    }
    KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
    if( !!pbody ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            return _pCollisionChecker->CheckCollision(plink2,pbody);
        }
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
        if( !!pbody2 ) {
            return _pCollisionChecker->CheckCollision(pbody,pbody2);
        }
        CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
        if( !!preport2 ) {
            bool bCollision = _pCollisionChecker->CheckCollision(pbody,preport2);
            openravepy::UpdateCollisionReport(o2,_pyenv);
            return bCollision;
        }
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
}
bool PyCollisionCheckerBase::CheckCollision(object o1, object o2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(o2);
    bool bCollision = false;
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
        if( !!plink2 ) {
            bCollision = _pCollisionChecker->CheckCollision(plink,plink2, openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                bCollision = _pCollisionChecker->CheckCollision(plink,pbody2, openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
            }
        }
    }
    else {
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                bCollision = _pCollisionChecker->CheckCollision(plink2,pbody, openravepy::GetCollisionReport(pReport));
            }
            else {
                KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                if( !!pbody2 ) {
                    bCollision = _pCollisionChecker->CheckCollision(pbody,pbody2, openravepy::GetCollisionReport(pReport));
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 2"),ORE_InvalidArguments);
                }
            }
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(object o1, PyKinBodyPtr pybody2)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(pybody2);
    KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    if( !!plink ) {
        return _pCollisionChecker->CheckCollision(plink,pbody2);
    }
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
    if( !!pbody1 ) {
        return _pCollisionChecker->CheckCollision(pbody1,pbody2);
    }
    throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
}

bool PyCollisionCheckerBase::CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport)
{
    CHECK_POINTER(o1);
    CHECK_POINTER(pybody2);
    KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
    KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
    bool bCollision = false;
    if( !!plink ) {
        bCollision = _pCollisionChecker->CheckCollision(plink,pbody2,openravepy::GetCollisionReport(pReport));
    }
    else {
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
        if( !!pbody1 ) {
            bCollision = _pCollisionChecker->CheckCollision(pbody1,pbody2,openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0(_("CheckCollision(object) invalid argument"),ORE_InvalidArguments);
        }
    }
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(object o1, object bodyexcluded, object linkexcluded)
{
    CollisionReportPtr preport = openravepy::GetCollisionReport(linkexcluded);
    if( !!preport ) {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("3rd argument should be linkexcluded, rather than CollisionReport! Try report="),ORE_InvalidArguments);
    }

    KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }
    if( !!plink1 ) {
        return _pCollisionChecker->CheckCollision(plink1,vbodyexcluded,vlinkexcluded);
    }
    else if( !!pbody1 ) {
        return _pCollisionChecker->CheckCollision(pbody1,vbodyexcluded,vlinkexcluded);
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
    }
}

bool PyCollisionCheckerBase::CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

    for(size_t i = 0; i < len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }

    bool bCollision=false;
    if( !!plink1 ) {
        bCollision = _pCollisionChecker->CheckCollision(plink1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    }
    else if( !!pbody1 ) {
        bCollision = _pCollisionChecker->CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid argument 1"),ORE_InvalidArguments);
    }

    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }
    return _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)),vbodyexcluded,vlinkexcluded);
}

bool PyCollisionCheckerBase::CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
{
    std::vector<KinBodyConstPtr> vbodyexcluded;
    for(size_t i = 0; i < len(bodyexcluded); ++i) {
        PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
        if( !!pbody ) {
            vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
        }
        else {
            RAVELOG_ERROR("failed to get excluded body\n");
        }
    }
    std::vector<KinBody::LinkConstPtr> vlinkexcluded;
    for(size_t i = 0; i < len(linkexcluded); ++i) {
        KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(linkexcluded[i]);
        if( !!plink2 ) {
            vlinkexcluded.push_back(plink2);
        }
        else {
            RAVELOG_ERROR("failed to get excluded link\n");
        }
    }

    bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)), vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody)
{
    return _pCollisionChecker->CheckCollision(pyray->r,KinBodyConstPtr(openravepy::GetKinBody(pbody)));
}

bool PyCollisionCheckerBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
{
    bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

object PyCollisionCheckerBase::CheckCollisionRays(object rays, PyKinBodyPtr pbody, bool bFrontFacingOnly)
{
    object shape = rays.attr("shape");
    const int num = extract<int>(shape[0]);
    if( num == 0 ) {
        return py::make_tuple(py::empty_array_astype<int>(), py::empty_array_astype<dReal>());
    }
    if( extract<int>(shape[1]) != 6 ) {
        throw openrave_exception(_("rays object needs to be a Nx6 vector\n"));
    }
    CollisionReport report;
    CollisionReportPtr preport(&report,null_deleter());

    RAY r;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    py::array_t<dReal> pypos({num, 6});
    py::buffer_info bufpos = pypos.request();
    dReal* ppos = (dReal*) bufpos.ptr;

    py::array_t<bool> pycollision({num});
    py::buffer_info bufcollision = pycollision.request();
    bool* pcollision = (bool*) bufcollision.ptr;
#else // USE_PYBIND11_PYTHON_BINDINGS
    npy_intp dims[] = { num,6};
    PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* ppos = (dReal*)PyArray_DATA(pypos);
    PyObject* pycollision = PyArray_SimpleNew(1, dims, PyArray_BOOL);
    bool* pcollision = (bool*)PyArray_DATA(pycollision);
#endif // USE_PYBIND11_PYTHON_BINDINGS
    for(int i = 0; i < num; ++i, ppos += 6) {
        std::vector<dReal> ray = ExtractArray<dReal>(rays[i]);
        r.pos.x = ray[0];
        r.pos.y = ray[1];
        r.pos.z = ray[2];
        r.dir.x = ray[3];
        r.dir.y = ray[4];
        r.dir.z = ray[5];
        bool bCollision;
        if( !pbody ) {
            bCollision = _pCollisionChecker->CheckCollision(r, preport);
        }
        else {
            bCollision = _pCollisionChecker->CheckCollision(r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), preport);
        }
        pcollision[i] = false;
        ppos[0] = 0; ppos[1] = 0; ppos[2] = 0; ppos[3] = 0; ppos[4] = 0; ppos[5] = 0;
        if( bCollision &&( report.contacts.size() > 0) ) {
            if( !bFrontFacingOnly ||( report.contacts[0].norm.dot3(r.dir)<0) ) {
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
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(pycollision, pypos);
#else // USE_PYBIND11_PYTHON_BINDINGS
    return py::make_tuple(py::to_array_astype<bool>(pycollision), py::to_array_astype<dReal>(pypos));
#endif // USE_PYBIND11_PYTHON_BINDINGS
}

bool PyCollisionCheckerBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray)
{
    return _pCollisionChecker->CheckCollision(pyray->r);
}

bool PyCollisionCheckerBase::CheckCollision(OPENRAVE_SHARED_PTR<PyRay> pyray, PyCollisionReportPtr pReport)
{
    bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollisionTriMesh(object otrimesh, PyKinBodyPtr pybody, PyCollisionReportPtr pReport)
{
    TriMesh trimesh;
    if( !ExtractTriMesh(otrimesh,trimesh) ) {
        throw openrave_exception(_("bad trimesh"));
    }
    KinBodyConstPtr pbody(openravepy::GetKinBody(pybody));
    bool bCollision = _pCollisionChecker->CheckCollision(trimesh, pbody, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollisionTriMesh(object otrimesh, PyCollisionReportPtr pReport)
{
    TriMesh trimesh;
    if( !ExtractTriMesh(otrimesh,trimesh) ) {
        throw openrave_exception(_("bad trimesh"));
    }
    bool bCollision = _pCollisionChecker->CheckCollision(trimesh, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckCollisionOBB(object oaabb, object otransform, PyCollisionReportPtr pReport)
{
    AABB aabb = ExtractAABB(oaabb);
    Transform t = ExtractTransform(otransform);
    bool bCollision = _pCollisionChecker->CheckCollision(aabb, t, openravepy::GetCollisionReport(pReport));
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

bool PyCollisionCheckerBase::CheckSelfCollision(object o1, PyCollisionReportPtr pReport)
{
    KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
    KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
    bool bCollision;
    if( !!plink1 ) {
        bCollision = _pCollisionChecker->CheckSelfCollision(plink1, openravepy::GetCollisionReport(pReport));
    }
    else if( !!pbody1 ) {
        bCollision = _pCollisionChecker->CheckSelfCollision(pbody1, openravepy::GetCollisionReport(pReport));
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0(_("invalid parameters to CheckSelfCollision"), ORE_InvalidArguments);
    }
    openravepy::UpdateCollisionReport(pReport,_pyenv);
    return bCollision;
}

CollisionCheckerBasePtr GetCollisionChecker(PyCollisionCheckerBasePtr pyCollisionChecker)
{
    return !pyCollisionChecker ? CollisionCheckerBasePtr() : pyCollisionChecker->GetCollisionChecker();
}

PyInterfaceBasePtr toPyCollisionChecker(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv)
{
    return !pCollisionChecker ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyCollisionCheckerBase(pCollisionChecker,pyenv));
}

CollisionReportPtr GetCollisionReport(object o)
{
    if( IS_PYTHONOBJECT_NONE(o) ) {
        return CollisionReportPtr();
    }
    extract_<PyCollisionReportPtr> pyreport(o);
    if( pyreport.check() ) {
        return ((PyCollisionReportPtr)pyreport)->report;
    }
    return CollisionReportPtr();
}

CollisionReportPtr GetCollisionReport(PyCollisionReportPtr p)
{
    return !p ? CollisionReportPtr() : p->report;
}

PyCollisionReportPtr toPyCollisionReport(CollisionReportPtr p, PyEnvironmentBasePtr pyenv)
{
    if( !p ) {
        return PyCollisionReportPtr();
    }
    PyCollisionReportPtr pyreport(new PyCollisionReport(p));
    pyreport->init(pyenv);
    return pyreport;
}

void UpdateCollisionReport(PyCollisionReportPtr p, PyEnvironmentBasePtr pyenv)
{
    if( !!p ) {
        p->init(pyenv);
    }
}

void UpdateCollisionReport(object o, PyEnvironmentBasePtr pyenv)
{
    extract_<PyCollisionReportPtr> pyreport(o);
    if( pyreport.check() ) {
        return UpdateCollisionReport((PyCollisionReportPtr)pyreport,pyenv);
    }
}

PyCollisionCheckerBasePtr RaveCreateCollisionChecker(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    CollisionCheckerBasePtr p = OpenRAVE::RaveCreateCollisionChecker(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyCollisionCheckerBasePtr();
    }
    return PyCollisionCheckerBasePtr(new PyCollisionCheckerBase(p,pyenv));
}

#ifndef USE_PYBIND11_PYTHON_BINDINGS
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)
#endif

#ifdef USE_PYBIND11_PYTHON_BINDINGS
void init_openravepy_collisionchecker(py::module& m)
#else
void init_openravepy_collisionchecker()
#endif
{
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    using namespace py::literals;  // "..."_a
    enum_<CollisionOptions>(m, "CollisionOptions", py::arithmetic() DOXY_ENUM(CollisionOptions))
#else
    enum_<CollisionOptions>("CollisionOptions" DOXY_ENUM(CollisionOptions))
#endif
    .value("Distance",CO_Distance)
    .value("UseTolerance",CO_UseTolerance)
    .value("Contacts",CO_Contacts)
    .value("RayAnyHit",CO_RayAnyHit)
    .value("ActiveDOFs",CO_ActiveDOFs)
    .value("AllLinkCollisions", CO_AllLinkCollisions)
    .value("AllGeometryContacts", CO_AllGeometryContacts)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .export_values()
#endif
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    enum_<CollisionAction>(m, "CollisionAction", py::arithmetic() DOXY_ENUM(CollisionAction))
#else
    enum_<CollisionAction>("CollisionAction" DOXY_ENUM(CollisionAction))
#endif
    .value("DefaultAction",CA_DefaultAction)
    .value("Ignore",CA_Ignore)
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .export_values()
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    // should this be inside CollisionReport, instead of module "m"?
    class_<PyCollisionReport::PYCONTACT, OPENRAVE_SHARED_PTR<PyCollisionReport::PYCONTACT> >(m, "Contact", DOXY_CLASS(CollisionReport::CONTACT))
    .def(init<>())
    .def(init<const CollisionReport::CONTACT&>(), "c"_a)
#else
    class_<PyCollisionReport::PYCONTACT, OPENRAVE_SHARED_PTR<PyCollisionReport::PYCONTACT> >("Contact", DOXY_CLASS(CollisionReport::CONTACT))
#endif
    .def_readonly("pos",&PyCollisionReport::PYCONTACT::pos)
    .def_readonly("norm",&PyCollisionReport::PYCONTACT::norm)
    .def_readonly("depth",&PyCollisionReport::PYCONTACT::depth)
    .def("__str__",&PyCollisionReport::PYCONTACT::__str__)
    .def("__unicode__",&PyCollisionReport::PYCONTACT::__unicode__)
    ;
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyCollisionReport, OPENRAVE_SHARED_PTR<PyCollisionReport> >(m, "CollisionReport", DOXY_CLASS(CollisionReport))
    .def(init<>())
    .def(init<CollisionReportPtr>(), "preport"_a)
#else
    class_<PyCollisionReport, OPENRAVE_SHARED_PTR<PyCollisionReport> >("CollisionReport", DOXY_CLASS(CollisionReport))
#endif
    .def_readonly("options",&PyCollisionReport::options)
    .def_readonly("plink1",&PyCollisionReport::plink1)
    .def_readonly("plink2",&PyCollisionReport::plink2)
    .def_readonly("minDistance",&PyCollisionReport::minDistance)
    .def_readonly("numWithinTol",&PyCollisionReport::numWithinTol)
    .def_readonly("contacts",&PyCollisionReport::contacts)
    .def_readonly("vLinkColliding",&PyCollisionReport::vLinkColliding)
    .def_readonly("nKeepPrevious", &PyCollisionReport::nKeepPrevious)
    .def("__str__",&PyCollisionReport::__str__)
    .def("__unicode__",&PyCollisionReport::__unicode__)
    ;

    bool (PyCollisionCheckerBase::*pcolb)(PyKinBodyPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolbr)(PyKinBodyPtr, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolbb)(PyKinBodyPtr,PyKinBodyPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolbbr)(PyKinBodyPtr, PyKinBodyPtr,PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcoll)(object) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcollr)(object, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolll)(object,object) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolllr)(object,object, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcollb)(object, PyKinBodyPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcollbr)(object, PyKinBodyPtr, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolle)(object,object,object) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcoller)(object, object,object,PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolbe)(PyKinBodyPtr,object,object) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolber)(PyKinBodyPtr, object,object,PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolyb)(OPENRAVE_SHARED_PTR<PyRay>,PyKinBodyPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolybr)(OPENRAVE_SHARED_PTR<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcoly)(OPENRAVE_SHARED_PTR<PyRay>) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolyr)(OPENRAVE_SHARED_PTR<PyRay>, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcoltbr)(object, PyKinBodyPtr, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollisionTriMesh;
    bool (PyCollisionCheckerBase::*pcolter)(object, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollisionTriMesh;
    bool (PyCollisionCheckerBase::*pcolobb)(object, object, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollisionOBB;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    class_<PyCollisionCheckerBase, OPENRAVE_SHARED_PTR<PyCollisionCheckerBase>, PyInterfaceBase>(m, "CollisionChecker", DOXY_CLASS(CollisionCheckerBase))
    .def(init<CollisionCheckerBasePtr, PyEnvironmentBasePtr>(), "pcollisionchecker"_a, "penv"_a)
#else
    class_<PyCollisionCheckerBase, OPENRAVE_SHARED_PTR<PyCollisionCheckerBase>, bases<PyInterfaceBase> >("CollisionChecker", DOXY_CLASS(CollisionCheckerBase), no_init)
#endif
    .def("InitEnvironment", &PyCollisionCheckerBase::InitEnvironment, DOXY_FN(CollisionCheckerBase, InitEnvironment))
    .def("DestroyEnvironment", &PyCollisionCheckerBase::DestroyEnvironment, DOXY_FN(CollisionCheckerBase, DestroyEnvironment))
    .def("InitKinBody", &PyCollisionCheckerBase::InitKinBody, DOXY_FN(CollisionCheckerBase, InitKinBody))
    .def("RemoveKinBody", &PyCollisionCheckerBase::RemoveKinBody, DOXY_FN(CollisionCheckerBase, RemoveKinBody))
    .def("SetGeometryGroup", &PyCollisionCheckerBase::SetGeometryGroup, DOXY_FN(CollisionCheckerBase, SetGeometryGroup))
    .def("SetBodyGeometryGroup", &PyCollisionCheckerBase::SetBodyGeometryGroup, PY_ARGS("body", "groupname") DOXY_FN(CollisionCheckerBase, SetBodyGeometryGroup))
    .def("GetGeometryGroup", &PyCollisionCheckerBase::GetGeometryGroup, DOXY_FN(CollisionCheckerBase, GetGeometryGroup))
    .def("SetCollisionOptions",&PyCollisionCheckerBase::SetCollisionOptions, DOXY_FN(CollisionCheckerBase,SetCollisionOptions "int"))
    .def("GetCollisionOptions",&PyCollisionCheckerBase::GetCollisionOptions, DOXY_FN(CollisionCheckerBase,GetCollisionOptions))
    .def("CheckCollision",pcolb, PY_ARGS("body") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbr, PY_ARGS("body","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbb, PY_ARGS("body1","body2") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbbr, PY_ARGS("body1","body2","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcoll, PY_ARGS("link") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollr, PY_ARGS("link","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolll, PY_ARGS("link1","link2") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolllr, PY_ARGS("link1","link2","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollb, PY_ARGS("link","body") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollbr, PY_ARGS("link","body","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolle, PY_ARGS("link","bodyexcluded","linkexcluded") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcoller, PY_ARGS("link","bodyexcluded","linkexcluded","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolbe, PY_ARGS("body","bodyexcluded","linkexcluded") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolber, PY_ARGS("body","bodyexcluded","linkexcluded","report") DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolyb, PY_ARGS("ray","body") DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolybr, PY_ARGS("ray","body","report") DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcoly, PY_ARGS("ray") DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; CollisionReportPtr"))
    .def("CheckCollision",pcolyr, PY_ARGS("ray", "report") DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; CollisionReportPtr"))
    .def("CheckCollisionTriMesh",pcolter, PY_ARGS("trimesh", "report") DOXY_FN(CollisionCheckerBase,CheckCollision "const TriMesh; CollisionReportPtr"))
    .def("CheckCollisionTriMesh",pcoltbr, PY_ARGS("trimesh", "body", "report") DOXY_FN(CollisionCheckerBase,CheckCollision "const TriMesh; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollisionOBB", pcolobb, PY_ARGS("aabb", "pose", "report") DOXY_FN(CollisionCheckerBase,CheckCollision "const AABB; const Transform; CollisionReport"))
    .def("CheckSelfCollision",&PyCollisionCheckerBase::CheckSelfCollision, PY_ARGS("linkbody", "report") DOXY_FN(CollisionCheckerBase,CheckSelfCollision "KinBodyConstPtr, CollisionReportPtr"))
#ifdef USE_PYBIND11_PYTHON_BINDINGS
    .def("CheckCollisionRays", &PyCollisionCheckerBase::CheckCollisionRays,
         "rays"_a,
         "body"_a,
         "front_facing_only"_a = false,
         "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columns are position, last 3 are direction*range. The return value is: (N array of hit points, Nx6 array of hit position and surface normals."
        )
#else
    .def("CheckCollisionRays",&PyCollisionCheckerBase::CheckCollisionRays,
         CheckCollisionRays_overloads(PY_ARGS("rays","body","front_facing_only")
                                      "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columns are position, last 3 are direction*range. The return value is: (N array of hit points, Nx6 array of hit position and surface normals."))
#endif
    ;

#ifdef USE_PYBIND11_PYTHON_BINDINGS
    m.def("RaveCreateCollisionChecker", openravepy::RaveCreateCollisionChecker, PY_ARGS("env","name") DOXY_FN1(RaveCreateCollisionChecker));
#else
    def("RaveCreateCollisionChecker",openravepy::RaveCreateCollisionChecker, PY_ARGS("env","name") DOXY_FN1(RaveCreateCollisionChecker));
#endif
}

}
