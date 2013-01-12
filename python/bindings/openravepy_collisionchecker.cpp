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

namespace openravepy {

class PyCollisionReport
{
public:
    PyCollisionReport() : report(new CollisionReport()) {
    }
    PyCollisionReport(CollisionReportPtr report) : report(report) {
    }
    virtual ~PyCollisionReport() {
    }

    struct PYCONTACT
    {
        PYCONTACT() {
        }
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
            ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
            ss << "pos=["<<vpos.x<<", "<<vpos.y<<", "<<vpos.z<<"], norm=["<<vnorm.x<<", "<<vnorm.y<<", "<<vnorm.z<<"]";
            return ss.str();
        }
        object __unicode__() {
            return ConvertStringToUnicode(__str__());
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
        if( !!report->plink1 ) {
            plink1 = openravepy::toPyKinBodyLink(boost::const_pointer_cast<KinBody::Link>(report->plink1), pyenv);
        }
        else {
            plink1 = object();
        }
        if( !!report->plink2 ) {
            plink2 = openravepy::toPyKinBodyLink(boost::const_pointer_cast<KinBody::Link>(report->plink2), pyenv);
        }
        else {
            plink2 = object();
        }
        boost::python::list newcontacts;
        FOREACH(itc, report->contacts) {
            newcontacts.append(PYCONTACT(*itc));
        }
        contacts = newcontacts;
    }

    string __str__()
    {
        return report->__str__();
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    int options;
    object plink1, plink2;
    int numCols;
    //std::vector<KinBody::Link*> vLinkColliding;
    dReal minDistance;
    int numWithinTol;
    boost::python::list contacts;

    CollisionReportPtr report;
};

class PyCollisionCheckerBase : public PyInterfaceBase
{
protected:
    CollisionCheckerBasePtr _pCollisionChecker;
public:
    PyCollisionCheckerBase(CollisionCheckerBasePtr pCollisionChecker, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pCollisionChecker, pyenv), _pCollisionChecker(pCollisionChecker) {
    }
    virtual ~PyCollisionCheckerBase() {
    }

    CollisionCheckerBasePtr GetCollisionChecker() {
        return _pCollisionChecker;
    }

    bool SetCollisionOptions(int options) {
        return _pCollisionChecker->SetCollisionOptions(options);
    }
    int GetCollisionOptions() const {
        return _pCollisionChecker->GetCollisionOptions();
    }

    bool InitEnvironment()
    {
        return _pCollisionChecker->InitEnvironment();
    }

    void DestroyEnvironment()
    {
        return _pCollisionChecker->DestroyEnvironment();
    }

    bool InitKinBody(PyKinBodyPtr pbody)
    {
        return _pCollisionChecker->InitKinBody(openravepy::GetKinBody(pbody));
    }

    void RemoveKinBody(PyKinBodyPtr pbody)
    {
        _pCollisionChecker->RemoveKinBody(openravepy::GetKinBody(pbody));
    }

    bool CheckCollision(PyKinBodyPtr pbody1)
    {
        CHECK_POINTER(pbody1);
        return _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)));
    }
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        return _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)));
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        bool bCollision = _pCollisionChecker->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1)
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
        throw OPENRAVE_EXCEPTION_FORMAT0("CheckCollision(object) invalid argument",ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyCollisionReportPtr pReport)
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
                throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument",ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, object o2)
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
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 2",ORE_InvalidArguments);
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
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 2",ORE_InvalidArguments);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 1",ORE_InvalidArguments);
    }
    bool CheckCollision(object o1, object o2, PyCollisionReportPtr pReport)
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
                    throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 2",ORE_InvalidArguments);
                }
            }
        }
        {
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
                        throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 2",ORE_InvalidArguments);
                    }
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 1",ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2)
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
        throw OPENRAVE_EXCEPTION_FORMAT0("CheckCollision(object) invalid argument",ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2, PyCollisionReportPtr pReport)
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
                throw OPENRAVE_EXCEPTION_FORMAT0("CheckCollision(object) invalid argument",ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded)
    {
        CollisionReportPtr preport = openravepy::GetCollisionReport(linkexcluded);
        if( !!preport ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("3rd argument should be linkexcluded, rather than CollisionReport! Try report=",ORE_InvalidArguments);
        }

        KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody ) {
                vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
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
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 1",ORE_InvalidArguments);
        }
    }

    bool CheckCollision(object o1, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        KinBody::LinkConstPtr plink1 = openravepy::GetKinBodyLinkConst(o1);
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);

        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody ) {
                vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
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
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 1",ORE_InvalidArguments);
        }

        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody ) {
                vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
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

    bool CheckCollision(PyKinBodyPtr pbody, object bodyexcluded, object linkexcluded, PyCollisionReportPtr pReport)
    {
        std::vector<KinBodyConstPtr> vbodyexcluded;
        for(int i = 0; i < len(bodyexcluded); ++i) {
            PyKinBodyPtr pbody = extract<PyKinBodyPtr>(bodyexcluded[i]);
            if( !!pbody ) {
                vbodyexcluded.push_back(openravepy::GetKinBody(pbody));
            }
            else {
                RAVELOG_ERROR("failed to get excluded body\n");
            }
        }
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        for(int i = 0; i < len(linkexcluded); ++i) {
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

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody)
    {
        return _pCollisionChecker->CheckCollision(pyray->r,KinBodyConstPtr(openravepy::GetKinBody(pbody)));
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }

    object CheckCollisionRays(object rays, PyKinBodyPtr pbody,bool bFrontFacingOnly=false)
    {
        object shape = rays.attr("shape");
        int num = extract<int>(shape[0]);
        if( num == 0 ) {
            return boost::python::make_tuple(numeric::array(boost::python::list()).astype("i4"),numeric::array(boost::python::list()));
        }
        if( extract<int>(shape[1]) != 6 ) {
            throw openrave_exception("rays object needs to be a Nx6 vector\n");
        }
        CollisionReport report;
        CollisionReportPtr preport(&report,null_deleter());

        RAY r;
        npy_intp dims[] = { num,6};
        PyObject *pypos = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
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

        return boost::python::make_tuple(static_cast<numeric::array>(handle<>(pycollision)),static_cast<numeric::array>(handle<>(pypos)));
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray)
    {
        return _pCollisionChecker->CheckCollision(pyray->r);
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyCollisionReportPtr pReport)
    {
        bool bCollision = _pCollisionChecker->CheckCollision(pyray->r, openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,_pyenv);
        return bCollision;
    }
};

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
    extract<PyCollisionReportPtr> pyreport(o);
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
    extract<PyCollisionReportPtr> pyreport(o);
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

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)

void init_openravepy_collisionchecker()
{
    enum_<CollisionOptions>("CollisionOptions" DOXY_ENUM(CollisionOptions))
    .value("Distance",CO_Distance)
    .value("UseTolerance",CO_UseTolerance)
    .value("Contacts",CO_Contacts)
    .value("RayAnyHit",CO_RayAnyHit)
    .value("ActiveDOFs",CO_ActiveDOFs);
    enum_<CollisionAction>("CollisionAction" DOXY_ENUM(CollisionAction))
    .value("DefaultAction",CA_DefaultAction)
    .value("Ignore",CA_Ignore)
    ;

    class_<PyCollisionReport::PYCONTACT, boost::shared_ptr<PyCollisionReport::PYCONTACT> >("Contact", DOXY_CLASS(CollisionReport::CONTACT))
    .def_readonly("pos",&PyCollisionReport::PYCONTACT::pos)
    .def_readonly("norm",&PyCollisionReport::PYCONTACT::norm)
    .def_readonly("depth",&PyCollisionReport::PYCONTACT::depth)
    .def("__str__",&PyCollisionReport::PYCONTACT::__str__)
    .def("__unicode__",&PyCollisionReport::PYCONTACT::__unicode__)
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
    bool (PyCollisionCheckerBase::*pcolyb)(boost::shared_ptr<PyRay>,PyKinBodyPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolybr)(boost::shared_ptr<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcoly)(boost::shared_ptr<PyRay>) = &PyCollisionCheckerBase::CheckCollision;
    bool (PyCollisionCheckerBase::*pcolyr)(boost::shared_ptr<PyRay>, PyCollisionReportPtr) = &PyCollisionCheckerBase::CheckCollision;

    class_<PyCollisionCheckerBase, boost::shared_ptr<PyCollisionCheckerBase>, bases<PyInterfaceBase> >("CollisionChecker", DOXY_CLASS(CollisionCheckerBase), no_init)
    .def("InitEnvironment", &PyCollisionCheckerBase::InitEnvironment, DOXY_FN(CollisionCheckerBase, InitEnvironment))
    .def("DestroyEnvironment", &PyCollisionCheckerBase::DestroyEnvironment, DOXY_FN(CollisionCheckerBase, DestroyEnvironment))
    .def("InitKinBody", &PyCollisionCheckerBase::InitKinBody, DOXY_FN(CollisionCheckerBase, InitKinBody))
    .def("RemoveKinBody", &PyCollisionCheckerBase::RemoveKinBody, DOXY_FN(CollisionCheckerBase, RemoveKinBody))
    .def("SetCollisionOptions",&PyCollisionCheckerBase::SetCollisionOptions, DOXY_FN(CollisionCheckerBase,SetCollisionOptions "int"))
    .def("GetCollisionOptions",&PyCollisionCheckerBase::GetCollisionOptions, DOXY_FN(CollisionCheckerBase,GetCollisionOptions))
    .def("CheckCollision",pcolb,args("body"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbr,args("body","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbb,args("body1","body2"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolbbr,args("body1","body2","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcoll,args("link"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollr,args("link","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolll,args("link1","link2"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolllr,args("link1","link2","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBody::LinkConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollb,args("link","body"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcollbr,args("link","body","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolle,args("link","bodyexcluded","linkexcluded"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcoller,args("link","bodyexcluded","linkexcluded","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolbe,args("body","bodyexcluded","linkexcluded"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolber,args("body","bodyexcluded","linkexcluded","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
    .def("CheckCollision",pcolyb,args("ray","body"), DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcolybr,args("ray","body","report"), DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
    .def("CheckCollision",pcoly,args("ray"), DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; CollisionReportPtr"))
    .def("CheckCollision",pcolyr,args("ray"), DOXY_FN(CollisionCheckerBase,CheckCollision "const RAY; CollisionReportPtr"))
    .def("CheckCollisionRays",&PyCollisionCheckerBase::CheckCollisionRays,
         CheckCollisionRays_overloads(args("rays","body","front_facing_only"),
                                      "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction+range."))
    ;

    def("RaveCreateCollisionChecker",openravepy::RaveCreateCollisionChecker,args("env","name"),DOXY_FN1(RaveCreateCollisionChecker));
}

}
