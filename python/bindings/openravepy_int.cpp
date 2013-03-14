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
#include "openravepy_int.h"

namespace openravepy
{

/// if set, will return all transforms are 1x7 vectors where first 4 compoonents are quaternion
static bool s_bReturnTransformQuaternions = false;
bool GetReturnTransformQuaternions() {
    return s_bReturnTransformQuaternions;
}
void SetReturnTransformQuaternions(bool bset) {
    s_bReturnTransformQuaternions = bset;
}

Transform ExtractTransform(const object& oraw)
{
    return ExtractTransformType<dReal>(oraw);
}

TransformMatrix ExtractTransformMatrix(const object& oraw)
{
    return ExtractTransformMatrixType<dReal>(oraw);
}

object toPyArray(const TransformMatrix& t)
{
    npy_intp dims[] = { 4,4};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.m[0]; pdata[1] = t.m[1]; pdata[2] = t.m[2]; pdata[3] = t.trans.x;
    pdata[4] = t.m[4]; pdata[5] = t.m[5]; pdata[6] = t.m[6]; pdata[7] = t.trans.y;
    pdata[8] = t.m[8]; pdata[9] = t.m[9]; pdata[10] = t.m[10]; pdata[11] = t.trans.z;
    pdata[12] = 0; pdata[13] = 0; pdata[14] = 0; pdata[15] = 1;
    return static_cast<numeric::array>(handle<>(pyvalues));
}


object toPyArray(const Transform& t)
{
    npy_intp dims[] = { 7};
    PyObject *pyvalues = PyArray_SimpleNew(1,dims, sizeof(dReal)==8 ? PyArray_DOUBLE : PyArray_FLOAT);
    dReal* pdata = (dReal*)PyArray_DATA(pyvalues);
    pdata[0] = t.rot.x; pdata[1] = t.rot.y; pdata[2] = t.rot.z; pdata[3] = t.rot.w;
    pdata[4] = t.trans.x; pdata[5] = t.trans.y; pdata[6] = t.trans.z;
    return static_cast<numeric::array>(handle<>(pyvalues));
}

AttributesList toAttributesList(boost::python::dict odict)
{
    AttributesList atts;
    if( !(odict == object()) ) {
        boost::python::list iterkeys = (boost::python::list)odict.iterkeys();
        size_t num = boost::python::len(iterkeys);
        for (size_t i = 0; i < num; i++) {
            // Because we know they're strings, we can do this
            std::string key = boost::python::extract<std::string>(iterkeys[i]);
            std::string value = boost::python::extract<std::string>(odict[iterkeys[i]]);
            atts.push_back(make_pair(key,value));
        }
    }
    return atts;
}

AttributesList toAttributesList(boost::python::list oattributes)
{
    AttributesList atts;
    if( !(oattributes == object()) ) {
        size_t num=len(oattributes);
        for (size_t i = 0; i < num; i++) {
            // Because we know they're strings, we can do this
            std::string key = boost::python::extract<std::string>(oattributes[i][0]);
            std::string value = boost::python::extract<std::string>(oattributes[i][1]);
            atts.push_back(make_pair(key,value));
        }
    }
    return atts;
}

AttributesList toAttributesList(boost::python::object oattributes)
{
    if( !(oattributes == object()) ) {
        boost::python::extract<boost::python::dict> odictextractor(oattributes);
        if( odictextractor.check() ) {
            return toAttributesList((boost::python::dict)odictextractor());
        }
        // assume list
        boost::python::extract<boost::python::list> olistextractor(oattributes);
        return toAttributesList((boost::python::list)olistextractor());
    }
    return AttributesList();
}

PyInterfaceBase::PyInterfaceBase(InterfaceBasePtr pbase, PyEnvironmentBasePtr pyenv) : _pbase(pbase), _pyenv(pyenv)
{
    CHECK_POINTER(_pbase);
    CHECK_POINTER(_pyenv);
}

object PyInterfaceBase::GetUserData(const std::string& key) const {
    return openravepy::GetUserData(_pbase->GetUserData(key));
}

object PyInterfaceBase::GetReadableInterfaces()
{
    boost::python::dict ointerfaces;
    FOREACHC(it,_pbase->GetReadableInterfaces()) {
        ointerfaces[it->first] = toPyXMLReadable(it->second);
    }
    return ointerfaces;
}

object PyInterfaceBase::GetReadableInterface(const std::string& xmltag)
{
    return toPyXMLReadable(_pbase->GetReadableInterface(xmltag));
}

void PyInterfaceBase::SetReadableInterface(const std::string& xmltag, object oreadable)
{
    _pbase->SetReadableInterface(xmltag,ExtractXMLReadable(oreadable));
}

class PyEnvironmentBase : public boost::enable_shared_from_this<PyEnvironmentBase>
{
#if BOOST_VERSION < 103500
    boost::mutex _envmutex;
    std::list<boost::shared_ptr<EnvironmentMutex::scoped_lock> > _listenvlocks, _listfreelocks;
#endif
    ViewerBasePtr _pviewer;
protected:
    EnvironmentBasePtr _penv;
    boost::shared_ptr<boost::thread> _threadviewer;
    boost::mutex _mutexViewer;
    boost::condition _conditionViewer;

    PyInterfaceBasePtr _toPyInterface(InterfaceBasePtr pinterface)
    {
        if( !pinterface ) {
            return PyInterfaceBasePtr();
        }
        switch(pinterface->GetInterfaceType()) {
        case PT_Planner: return openravepy::toPyPlanner(boost::static_pointer_cast<PlannerBase>(pinterface),shared_from_this());
        case PT_Robot: return openravepy::toPyRobot(boost::static_pointer_cast<RobotBase>(pinterface),shared_from_this());
        case PT_SensorSystem: return openravepy::toPySensorSystem(boost::static_pointer_cast<SensorSystemBase>(pinterface),shared_from_this());
        case PT_Controller: return openravepy::toPyController(boost::static_pointer_cast<ControllerBase>(pinterface),shared_from_this());
        case PT_Module: return openravepy::toPyModule(boost::static_pointer_cast<ModuleBase>(pinterface),shared_from_this());
        case PT_IkSolver: return openravepy::toPyIkSolver(boost::static_pointer_cast<IkSolverBase>(pinterface),shared_from_this());
        case PT_KinBody: return openravepy::toPyKinBody(boost::static_pointer_cast<KinBody>(pinterface),shared_from_this());
        case PT_PhysicsEngine: return openravepy::toPyPhysicsEngine(boost::static_pointer_cast<PhysicsEngineBase>(pinterface),shared_from_this());
        case PT_Sensor: return openravepy::toPySensor(boost::static_pointer_cast<SensorBase>(pinterface),shared_from_this());
        case PT_CollisionChecker: return openravepy::toPyCollisionChecker(boost::static_pointer_cast<CollisionCheckerBase>(pinterface),shared_from_this());
        case PT_Trajectory: return openravepy::toPyTrajectory(boost::static_pointer_cast<TrajectoryBase>(pinterface),shared_from_this());
        case PT_Viewer: return openravepy::toPyViewer(boost::static_pointer_cast<ViewerBase>(pinterface),shared_from_this());
        case PT_SpaceSampler: return openravepy::toPySpaceSampler(boost::static_pointer_cast<SpaceSamplerBase>(pinterface),shared_from_this());
        }
        return PyInterfaceBasePtr();
    }

    void _ViewerThread(const string &strviewer, bool bShowViewer)
    {
        _pviewer.reset();
        {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _pviewer = RaveCreateViewer(_penv, strviewer);
            if( !!_pviewer ) {
                _penv->AddViewer(_pviewer);
            }
            _conditionViewer.notify_all();
        }

        if( !_pviewer ) {
            return;
        }
        _pviewer->main(bShowViewer);     // spin until quitfrommainloop is called
        _penv->Remove(_pviewer);
        _pviewer.reset();
    }

    CollisionAction _CollisionCallback(object fncallback, CollisionReportPtr preport, bool bFromPhysics)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            res = fncallback(openravepy::toPyCollisionReport(preport,shared_from_this()),bFromPhysics);
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in python collision callback:\n");
            PyErr_Print();
        }
        PyGILState_Release(gstate);
        if(( res == object()) || !res ) {
            return CA_DefaultAction;
        }
        extract<int> xi(res);
        if( xi.check() ) {
            return (CollisionAction)(int) xi;
        }
        RAVELOG_WARN("collision callback nothing returning, so executing default action\n");
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

    PyEnvironmentBase(const PyEnvironmentBase &pyenv)
    {
        _penv = pyenv._penv;
    }

    virtual ~PyEnvironmentBase()
    {
        if( !!_threadviewer ) {
            _threadviewer->join();
        }
        _threadviewer.reset();
        _pviewer.reset();
    }

    void Reset() {
        _penv->Reset();
    }
    void Destroy() {
        _penv->Destroy();
        if( !!_threadviewer ) {
            _threadviewer->join();
        }
    }

    PyEnvironmentBasePtr CloneSelf(int options)
    {
//        string strviewer;
//        if( options & Clone_Viewer ) {
//            boost::mutex::scoped_lock lockcreate(_mutexViewer);
//            if( !!_penv->GetViewer() ) {
//                strviewer = _penv->GetViewer()->GetXMLId();
//            }
//        }
        PyEnvironmentBasePtr pnewenv(new PyEnvironmentBase(_penv->CloneSelf(options)));
//        if( strviewer.size() > 0 ) {
//            pnewenv->SetViewer(strviewer);
//        }
        return pnewenv;
    }

    void Clone(PyEnvironmentBasePtr pyreference, int options)
    {
        if( options & Clone_Viewer ) {
            if( !!_penv->GetViewer() && !!pyreference->GetEnv()->GetViewer() ) {
                if( _penv->GetViewer()->GetXMLId() != pyreference->GetEnv()->GetViewer()->GetXMLId() ) {
                    RAVELOG_VERBOSE("reset the viewer since it has to be cloned\n");
                    boost::mutex::scoped_lock lockcreate(pyreference->_mutexViewer);
                    SetViewer("");
                }
            }
        }
        _penv->Clone(pyreference->GetEnv(),options);
    }

    bool SetCollisionChecker(PyCollisionCheckerBasePtr pchecker)
    {
        return _penv->SetCollisionChecker(openravepy::GetCollisionChecker(pchecker));
    }
    object GetCollisionChecker()
    {
        return object(openravepy::toPyCollisionChecker(_penv->GetCollisionChecker(), shared_from_this()));
    }
    bool CheckCollision(PyKinBodyPtr pbody1)
    {
        CHECK_POINTER(pbody1);
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)));
    }
    bool CheckCollision(PyKinBodyPtr pbody1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)));
    }

    bool CheckCollision(PyKinBodyPtr pbody1, PyKinBodyPtr pbody2, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(pbody1);
        CHECK_POINTER(pbody2);
        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody1)), KinBodyConstPtr(openravepy::GetKinBody(pbody2)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _penv->CheckCollision(plink);
        }
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            return _penv->CheckCollision(pbody);
        }
        throw OPENRAVE_EXCEPTION_FORMAT0("CheckCollision(object) invalid argument",ORE_InvalidArguments);
    }

    bool CheckCollision(object o1, PyCollisionReportPtr pReport)
    {
        CHECK_POINTER(o1);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        bool bCollision;
        if( !!plink ) {
            bCollision = _penv->CheckCollision(plink,openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
            if( !!pbody ) {
                bCollision = _penv->CheckCollision(pbody,openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument",ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
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
                return _penv->CheckCollision(plink,plink2);
            }
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                return _penv->CheckCollision(plink,pbody2);
            }
            CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _penv->CheckCollision(plink,preport2);
                openravepy::UpdateCollisionReport(o2,shared_from_this());
                return bCollision;
            }
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 2",ORE_InvalidArguments);
        }
        KinBodyConstPtr pbody = openravepy::GetKinBody(o1);
        if( !!pbody ) {
            KinBody::LinkConstPtr plink2 = openravepy::GetKinBodyLinkConst(o2);
            if( !!plink2 ) {
                return _penv->CheckCollision(plink2,pbody);
            }
            KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
            if( !!pbody2 ) {
                return _penv->CheckCollision(pbody,pbody2);
            }
            CollisionReportPtr preport2 = openravepy::GetCollisionReport(o2);
            if( !!preport2 ) {
                bool bCollision = _penv->CheckCollision(pbody,preport2);
                openravepy::UpdateCollisionReport(o2,shared_from_this());
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
                bCollision = _penv->CheckCollision(plink,plink2, openravepy::GetCollisionReport(pReport));
            }
            else {
                KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                if( !!pbody2 ) {
                    bCollision = _penv->CheckCollision(plink,pbody2, openravepy::GetCollisionReport(pReport));
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
                    bCollision = _penv->CheckCollision(plink2,pbody, openravepy::GetCollisionReport(pReport));
                }
                else {
                    KinBodyConstPtr pbody2 = openravepy::GetKinBody(o2);
                    if( !!pbody2 ) {
                        bCollision = _penv->CheckCollision(pbody,pbody2, openravepy::GetCollisionReport(pReport));
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
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(object o1, PyKinBodyPtr pybody2)
    {
        CHECK_POINTER(o1);
        CHECK_POINTER(pybody2);
        KinBodyConstPtr pbody2 = openravepy::GetKinBody(pybody2);
        KinBody::LinkConstPtr plink = openravepy::GetKinBodyLinkConst(o1);
        if( !!plink ) {
            return _penv->CheckCollision(plink,pbody2);
        }
        KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
        if( !!pbody1 ) {
            return _penv->CheckCollision(pbody1,pbody2);
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
            bCollision = _penv->CheckCollision(plink,pbody2,openravepy::GetCollisionReport(pReport));
        }
        else {
            KinBodyConstPtr pbody1 = openravepy::GetKinBody(o1);
            if( !!pbody1 ) {
                bCollision = _penv->CheckCollision(pbody1,pbody2,openravepy::GetCollisionReport(pReport));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0("CheckCollision(object) invalid argument",ORE_InvalidArguments);
            }
        }
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
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
            return _penv->CheckCollision(plink1,vbodyexcluded,vlinkexcluded);
        }
        else if( !!pbody1 ) {
            return _penv->CheckCollision(pbody1,vbodyexcluded,vlinkexcluded);
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
            bCollision = _penv->CheckCollision(plink1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        }
        else if( !!pbody1 ) {
            bCollision = _penv->CheckCollision(pbody1, vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT0("invalid argument 1",ORE_InvalidArguments);
        }

        openravepy::UpdateCollisionReport(pReport,shared_from_this());
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
        return _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)),vbodyexcluded,vlinkexcluded);
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

        bool bCollision = _penv->CheckCollision(KinBodyConstPtr(openravepy::GetKinBody(pbody)), vbodyexcluded, vlinkexcluded, openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody)
    {
        return _penv->CheckCollision(pyray->r,KinBodyConstPtr(openravepy::GetKinBody(pbody)));
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyKinBodyPtr pbody, PyCollisionReportPtr pReport)
    {
        bool bCollision = _penv->CheckCollision(pyray->r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
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
                bCollision = _penv->CheckCollision(r, preport);
            }
            else {
                bCollision = _penv->CheckCollision(r, KinBodyConstPtr(openravepy::GetKinBody(pbody)), preport);
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
        return _penv->CheckCollision(pyray->r);
    }

    bool CheckCollision(boost::shared_ptr<PyRay> pyray, PyCollisionReportPtr pReport)
    {
        bool bCollision = _penv->CheckCollision(pyray->r, openravepy::GetCollisionReport(pReport));
        openravepy::UpdateCollisionReport(pReport,shared_from_this());
        return bCollision;
    }

    bool Load(const string &filename) {
        return _penv->Load(filename);
    }
    bool Load(const string &filename, object odictatts) {
        return _penv->Load(filename, toAttributesList(odictatts));
    }
    bool LoadURI(const string &filename, object odictatts=object()) {
        return _penv->LoadURI(filename, toAttributesList(odictatts));
    }
    bool LoadData(const string &data) {
        return _penv->LoadData(data);
    }
    bool LoadData(const string &data, object odictatts) {
        return _penv->LoadData(data, toAttributesList(odictatts));
    }

    void Save(const string &filename, EnvironmentBase::SelectionOptions options=EnvironmentBase::SO_Everything, object odictatts=object()) {
        extract<std::string> otarget(odictatts);
        if( otarget.check() ) {
            // old versions
            AttributesList atts;
            atts.push_back(std::make_pair(std::string("target"),(std::string)otarget));
            _penv->Save(filename,options,atts);
        }
        else {
            _penv->Save(filename,options,toAttributesList(odictatts));
        }
    }

    object ReadRobotURI(const string &filename)
    {
        return object(openravepy::toPyRobot(_penv->ReadRobotURI(filename),shared_from_this()));
    }
    object ReadRobotURI(const string &filename, object odictatts)
    {
        return object(openravepy::toPyRobot(_penv->ReadRobotURI(RobotBasePtr(), filename,toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadRobotData(const string &data)
    {
        return object(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, AttributesList()), shared_from_this()));
    }
    object ReadRobotData(const string &data, object odictatts)
    {
        return object(openravepy::toPyRobot(_penv->ReadRobotData(RobotBasePtr(), data, toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadKinBodyURI(const string &filename)
    {
        return object(openravepy::toPyKinBody(_penv->ReadKinBodyURI(filename), shared_from_this()));
    }
    object ReadKinBodyURI(const string &filename, object odictatts)
    {
        return object(openravepy::toPyKinBody(_penv->ReadKinBodyURI(KinBodyPtr(), filename, toAttributesList(odictatts)),shared_from_this()));
    }
    object ReadKinBodyData(const string &data)
    {
        return object(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, AttributesList()),shared_from_this()));
    }
    object ReadKinBodyData(const string &data, object odictatts)
    {
        return object(openravepy::toPyKinBody(_penv->ReadKinBodyData(KinBodyPtr(), data, toAttributesList(odictatts)),shared_from_this()));
    }
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename)
    {
        return _toPyInterface(_penv->ReadInterfaceURI(filename));
    }
    PyInterfaceBasePtr ReadInterfaceURI(const std::string& filename, object odictatts)
    {
        return _toPyInterface(_penv->ReadInterfaceURI(filename, toAttributesList(odictatts)));
    }
    object ReadTrimeshURI(const std::string& filename)
    {
        boost::shared_ptr<TriMesh> ptrimesh = _penv->ReadTrimeshURI(boost::shared_ptr<TriMesh>(),filename);
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }
    object ReadTrimeshURI(const std::string& filename, object odictatts)
    {
        boost::shared_ptr<TriMesh> ptrimesh = _penv->ReadTrimeshURI(boost::shared_ptr<TriMesh>(),filename,toAttributesList(odictatts));
        if( !ptrimesh ) {
            return object();
        }
        return toPyTriMesh(*ptrimesh);
    }

    void Add(PyInterfaceBasePtr pinterface, bool bAnonymous=false, const std::string& cmdargs="") {
        _penv->Add(pinterface->GetInterfaceBase(), bAnonymous, cmdargs);
    }

    void AddKinBody(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody));
    }
    void AddKinBody(PyKinBodyPtr pbody, bool bAnonymous) {
        CHECK_POINTER(pbody); _penv->Add(openravepy::GetKinBody(pbody),bAnonymous);
    }
    void AddRobot(PyRobotBasePtr robot) {
        CHECK_POINTER(robot);
        _penv->Add(openravepy::GetRobot(robot));
    }
    void AddRobot(PyRobotBasePtr robot, bool bAnonymous) {
        CHECK_POINTER(robot);
        _penv->Add(openravepy::GetRobot(robot),bAnonymous);
    }
    void AddSensor(PySensorBasePtr sensor) {
        CHECK_POINTER(sensor);
        _penv->Add(openravepy::GetSensor(sensor));
    }
    void AddSensor(PySensorBasePtr sensor, bool bAnonymous) {
        CHECK_POINTER(sensor);
        _penv->Add(openravepy::GetSensor(sensor),bAnonymous);
    }
    void AddViewer(PyViewerBasePtr viewer) {
        CHECK_POINTER(viewer);
        _penv->Add(openravepy::GetViewer(viewer));
    }

    bool RemoveKinBody(PyKinBodyPtr pbody) {
        CHECK_POINTER(pbody);
        RAVELOG_WARN("openravepy RemoveKinBody deprecated, use Remove\n");
        return _penv->Remove(openravepy::GetKinBody(pbody));
    }

    object GetKinBody(const string &name)
    {
        return object(openravepy::toPyKinBody(_penv->GetKinBody(name),shared_from_this()));
    }
    object GetRobot(const string &name)
    {
        return object(openravepy::toPyRobot(_penv->GetRobot(name), shared_from_this()));
    }
    object GetSensor(const string &name)
    {
        return object(openravepy::toPySensor(_penv->GetSensor(name),shared_from_this()));
    }

    object GetBodyFromEnvironmentId(int id)
    {
        return object(openravepy::toPyKinBody(_penv->GetBodyFromEnvironmentId(id),shared_from_this()));
    }

    int AddModule(PyModuleBasePtr prob, const string &args) {
        CHECK_POINTER(prob);
        return _penv->AddModule(openravepy::GetModule(prob),args);
    }
    bool RemoveProblem(PyModuleBasePtr prob) {
        CHECK_POINTER(prob);
        RAVELOG_WARN("openravepy RemoveProblem deprecated, use Remove\n");
        return _penv->Remove(openravepy::GetModule(prob));
    }
    bool Remove(PyInterfaceBasePtr obj) {
        CHECK_POINTER(obj);
        return _penv->Remove(obj->GetInterfaceBase());
    }

    object GetModules()
    {
        std::list<ModuleBasePtr> listModules;
        _penv->GetModules(listModules);
        boost::python::list modules;
        FOREACHC(itprob, listModules) {
            modules.append(openravepy::toPyModule(*itprob,shared_from_this()));
        }
        return modules;
    }

    bool SetPhysicsEngine(PyPhysicsEngineBasePtr pengine)
    {
        return _penv->SetPhysicsEngine(openravepy::GetPhysicsEngine(pengine));
    }
    object GetPhysicsEngine() {
        return object(openravepy::toPyPhysicsEngine(_penv->GetPhysicsEngine(),shared_from_this()));
    }

    object RegisterCollisionCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        UserDataPtr p = _penv->RegisterCollisionCallback(boost::bind(&PyEnvironmentBase::_CollisionCallback,shared_from_this(),fncallback,_1,_2));
        if( !p ) {
            throw openrave_exception("registration handle is NULL");
        }
        return openravepy::GetUserData(p);
    }

    bool HasRegisteredCollisionCallbacks()
    {
        return _penv->HasRegisteredCollisionCallbacks();
    }

    void StepSimulation(dReal timeStep) {
        _penv->StepSimulation(timeStep);
    }
    void StartSimulation(dReal fDeltaTime, bool bRealTime=true) {
        _penv->StartSimulation(fDeltaTime,bRealTime);
    }
    void StopSimulation() {
        _penv->StopSimulation();
    }
    uint64_t GetSimulationTime() {
        return _penv->GetSimulationTime();
    }
    bool IsSimulationRunning() {
        return _penv->IsSimulationRunning();
    }

    void Lock()
    {
        Py_BEGIN_ALLOW_THREADS;
#if BOOST_VERSION < 103500
        boost::mutex::scoped_lock envlock(_envmutex);
        if( _listfreelocks.size() > 0 ) {
            _listfreelocks.back()->lock();
            _listenvlocks.splice(_listenvlocks.end(),_listfreelocks,--_listfreelocks.end());
        }
        else {
            _listenvlocks.push_back(boost::shared_ptr<EnvironmentMutex::scoped_lock>(new EnvironmentMutex::scoped_lock(_penv->GetMutex())));
        }
#else
        _penv->GetMutex().lock();
#endif
        Py_END_ALLOW_THREADS;
    }
    void Unlock()
    {
#if BOOST_VERSION < 103500
        boost::mutex::scoped_lock envlock(_envmutex);
        BOOST_ASSERT(_listenvlocks.size()>0);
        _listenvlocks.back()->unlock();
        _listfreelocks.splice(_listfreelocks.end(),_listenvlocks,--_listenvlocks.end());
#else
        _penv->GetMutex().unlock();
#endif
    }

    void Lock(float timeout)
    {
        RAVELOG_WARN("Environment.Lock timeout is ignored\n");
        Lock();
    }

    void __enter__()
    {
        Lock();
    }

    void __exit__(object type, object value, object traceback)
    {
        Unlock();
    }

    bool SetViewer(const string &viewername, bool showviewer=true)
    {
        if( !!_threadviewer ) {     // wait for the viewer
            _threadviewer->join();
        }
        _threadviewer.reset();

        if( viewername.size() > 0 ) {
            boost::mutex::scoped_lock lock(_mutexViewer);
            _threadviewer.reset(new boost::thread(boost::bind(&PyEnvironmentBase::_ViewerThread, shared_from_this(), viewername, showviewer)));
            _conditionViewer.wait(lock);
            //            if( !_penv->GetViewer() || _penv->GetViewer()->GetXMLId() != viewername ) {
            //                RAVELOG_WARN("failed to create viewer %s\n", viewername.c_str());
            //                _threadviewer->join();
            //                _threadviewer.reset();
            //                return false;
            //            }
            //            else {
            //                RAVELOG_INFOA("viewer %s successfully attached\n", viewername.c_str());
            //            }
        }
        return true;
    }

    object GetViewer()
    {
        return object(openravepy::toPyViewer(_penv->GetViewer(),shared_from_this()));
    }

    /// returns the number of points
    static size_t _getGraphPoints(object opoints, vector<float>&vpoints)
    {
        if( PyObject_HasAttrString(opoints.ptr(),"shape") ) {
            object pointshape = opoints.attr("shape");
            switch(len(pointshape)) {
            case 1:
                vpoints = ExtractArray<float>(opoints);
                if( vpoints.size()%3 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("points have bad size %d", vpoints.size(),ORE_InvalidArguments);
                }
                return vpoints.size()/3;
            case 2: {
                int num = extract<int>(pointshape[0]);
                int dim = extract<int>(pointshape[1]);
                vpoints = ExtractArray<float>(opoints.attr("flat"));
                if(dim % 3) {
                    throw OPENRAVE_EXCEPTION_FORMAT("points have bad size %dx%d", num%dim,ORE_InvalidArguments);
                }
                return num*(dim/3);
            }
            default:
                throw openrave_exception("points have bad dimension");
            }
        }
        // assume it is a regular 1D list
        vpoints = ExtractArray<float>(opoints);
        if( vpoints.size()% 3 ) {
            throw OPENRAVE_EXCEPTION_FORMAT("points have bad size %d", vpoints.size(),ORE_InvalidArguments);
        }
        return vpoints.size()/3;
    }

    /// returns the number of colors
    static size_t _getGraphColors(object ocolors, vector<float>&vcolors)
    {
        if( !(ocolors == object()) ) {
            if( PyObject_HasAttrString(ocolors.ptr(),"shape") ) {
                object colorshape = ocolors.attr("shape");
                switch( len(colorshape) ) {
                case 1:
                    break;
                case 2: {
                    int numcolors = extract<int>(colorshape[0]);
                    int colordim = extract<int>(colorshape[1]);
                    if(( colordim != 3) &&( colordim != 4) ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("colors dim %d needs to be 3 or 4",colordim, ORE_InvalidArguments);
                    }
                    vcolors = ExtractArray<float>(ocolors.attr("flat"));
                    return numcolors;
                }
                default:
                    throw OPENRAVE_EXCEPTION_FORMAT("colors has %d dimensions",len(colorshape), ORE_InvalidArguments);
                }
            }
            vcolors = ExtractArray<float>(ocolors);
            if( vcolors.size() == 3 ) {
                vcolors.push_back(1.0f);
            }
            else if( vcolors.size() != 4 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("colors has incorrect number of values %d",vcolors.size(), ORE_InvalidArguments);
            }
            return 1;
        }
        // default
        RaveVector<float> vcolor(1,0.5,0.5,1.0);
        vcolors.resize(4);
        vcolors[0] = 1; vcolors[1] = 0.5f; vcolors[2] = 0.5f; vcolors[3] = 1.0f;
        return 1;
    }

    static pair<size_t,size_t> _getGraphPointsColors(object opoints, object ocolors, vector<float>&vpoints, vector<float>&vcolors)
    {
        size_t numpoints = _getGraphPoints(opoints,vpoints);
        size_t numcolors = _getGraphColors(ocolors,vcolors);
        if( numpoints <= 0 ) {
            throw openrave_exception("points cannot be empty",ORE_InvalidArguments);
        }
        if(( numcolors > 1) &&( numpoints != numcolors) ) {
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
            return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,&vcolors[0],drawstyle,bhasalpha));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->plot3(&vpoints[0],sizes.first,sizeof(float)*3,pointsize,vcolor,drawstyle));
    }

    object drawlinestrip(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->drawlinestrip(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
    }

    object drawlinelist(object opoints,float linewidth,object ocolors=object(),int drawstyle=0)
    {
        vector<float> vpoints, vcolors;
        pair<size_t,size_t> sizes = _getGraphPointsColors(opoints,ocolors,vpoints,vcolors);
        //bool bhasalpha = vcolors.size() == 4*sizes.second;
        if( sizes.first == sizes.second ) {
            return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,&vcolors[0]));
        }
        BOOST_ASSERT(vcolors.size()<=4);
        RaveVector<float> vcolor;
        for(int i = 0; i < (int)vcolors.size(); ++i) {
            vcolor[i] = vcolors[i];
        }
        return toPyGraphHandle(_penv->drawlinelist(&vpoints[0],sizes.first,sizeof(float)*3,linewidth,vcolor));
    }

    object drawarrow(object op1, object op2, float linewidth=0.002, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !(ocolor == object()) ) {
            vcolor = ExtractVector34(ocolor,1.0f);
        }
        return toPyGraphHandle(_penv->drawarrow(ExtractVector3(op1),ExtractVector3(op2),linewidth,vcolor));
    }

    object drawbox(object opos, object oextents, object ocolor=object())
    {
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !(ocolor == object()) ) {
            vcolor = ExtractVector34(ocolor,1.0f);
        }
        return toPyGraphHandle(_penv->drawbox(ExtractVector3(opos),ExtractVector3(oextents)));
    }

    object drawplane(object otransform, object oextents, const boost::multi_array<float,2>&_vtexture)
    {
        boost::multi_array<float,3> vtexture(boost::extents[1][_vtexture.shape()[0]][_vtexture.shape()[1]]);
        vtexture[0] = _vtexture;
        boost::array<size_t,3> dims = { { _vtexture.shape()[0],_vtexture.shape()[1],1}};
        vtexture.reshape(dims);
        return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture));
    }
    object drawplane(object otransform, object oextents, const boost::multi_array<float,3>&vtexture)
    {
        return toPyGraphHandle(_penv->drawplane(RaveTransform<float>(ExtractTransform(otransform)), RaveVector<float>(extract<float>(oextents[0]),extract<float>(oextents[1]),0), vtexture));
    }

    object drawtrimesh(object opoints, object oindices=object(), object ocolors=object())
    {
        vector<float> vpoints;
        _getGraphPoints(opoints,vpoints);
        vector<int> vindices;
        int* pindices = NULL;
        int numTriangles = vpoints.size()/9;
        if( !(oindices == object()) ) {
            vindices = ExtractArray<int>(oindices.attr("flat"));
            if( vindices.size() > 0 ) {
                numTriangles = vindices.size()/3;
                pindices = &vindices[0];
            }
        }
        RaveVector<float> vcolor(1,0.5,0.5,1);
        if( !(ocolors == object()) ) {
            object shape = ocolors.attr("shape");
            if( len(shape) == 1 ) {
                return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,ExtractVector34(ocolors,1.0f)));
            }
            else {
                BOOST_ASSERT(extract<size_t>(shape[0])==vpoints.size()/3);
                return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,extract<boost::multi_array<float,2> >(ocolors)));
            }
        }
        return toPyGraphHandle(_penv->drawtrimesh(&vpoints[0],sizeof(float)*3,pindices,numTriangles,RaveVector<float>(1,0.5,0.5,1)));
    }

    object GetBodies()
    {
        std::vector<KinBodyPtr> vbodies;
        _penv->GetBodies(vbodies);
        boost::python::list bodies;
        FOREACHC(itbody, vbodies) {

            if( (*itbody)->IsRobot() ) {
                bodies.append(openravepy::toPyRobot(RaveInterfaceCast<RobotBase>(*itbody),shared_from_this()));
            }
            else {
                bodies.append(openravepy::toPyKinBody(*itbody,shared_from_this()));
            }
        }
        return bodies;
    }

    object GetRobots()
    {
        std::vector<RobotBasePtr> vrobots;
        _penv->GetRobots(vrobots);
        boost::python::list robots;
        FOREACHC(itrobot, vrobots) {
            robots.append(openravepy::toPyRobot(*itrobot,shared_from_this()));
        }
        return robots;
    }

    object GetSensors()
    {
        std::vector<SensorBasePtr> vsensors;
        _penv->GetSensors(vsensors);
        boost::python::list sensors;
        FOREACHC(itsensor, vsensors) {
            sensors.append(openravepy::toPySensor(*itsensor,shared_from_this()));
        }
        return sensors;
    }

    void UpdatePublishedBodies()
    {
        _penv->UpdatePublishedBodies();
    }

    object Triangulate(PyKinBodyPtr pbody)
    {
        CHECK_POINTER(pbody);
        TriMesh mesh;
        _penv->Triangulate(mesh,openravepy::GetKinBody(pbody));
        return toPyTriMesh(mesh);
    }

    object TriangulateScene(EnvironmentBase::SelectionOptions options, const string &name)
    {
        TriMesh mesh;
        _penv->TriangulateScene(mesh,options,name);
        return toPyTriMesh(mesh);
    }

    void SetDebugLevel(object olevel) {
        _penv->SetDebugLevel(pyGetIntFromPy(olevel,Level_Info));
    }
    int GetDebugLevel() const {
        return _penv->GetDebugLevel();
    }

    string GetHomeDirectory() {
        RAVELOG_WARN("Environment.GetHomeDirectory is deprecated, use RaveGetHomeDirectory\n"); return RaveGetHomeDirectory();
    }

    void SetUserData(PyUserData pdata) {
        _penv->SetUserData(pdata._handle);
    }
    void SetUserData(object o) {
        _penv->SetUserData(boost::shared_ptr<UserData>(new PyUserObject(o)));
    }
    object GetUserData() const {
        return openravepy::GetUserData(_penv->GetUserData());
    }

    bool __eq__(PyEnvironmentBasePtr p) {
        return !!p && _penv==p->_penv;
    }
    bool __ne__(PyEnvironmentBasePtr p) {
        return !p || _penv!=p->_penv;
    }
    string __repr__() {
        return boost::str(boost::format("RaveGetEnvironment(%d)")%RaveGetEnvironmentId(_penv));
    }
    string __str__() {
        return boost::str(boost::format("<env %d>")%RaveGetEnvironmentId(_penv));
    }
    object __unicode__() {
        return ConvertStringToUnicode(__str__());
    }

    EnvironmentBasePtr GetEnv() const {
        return _penv;
    }
};

PyEnvironmentBasePtr PyInterfaceBase::GetEnv() const
{
#if BOOST_VERSION >= 103500
    return _pyenv;
#else
    // if raw shared_ptr is returned, then python will throw RuntimeError: tr1::bad_weak_ptr when env is used
    return PyEnvironmentBasePtr(new PyEnvironmentBase(_pyenv->GetEnv()));
#endif
}

object GetUserData(UserDataPtr pdata)
{
    boost::shared_ptr<PyUserObject> po = boost::dynamic_pointer_cast<PyUserObject>(pdata);
    if( !!po ) {
        return po->_o;
    }
    else {
        SerializableDataPtr pserializable = boost::dynamic_pointer_cast<SerializableData>(pdata);
        if( !!pserializable ) {
            return object(PySerializableData(pserializable));
        }
        else if( !!pdata ) {
            return object(PyUserData(pdata));
        }
        else {
            return object();
        }
    }
}

EnvironmentBasePtr GetEnvironment(PyEnvironmentBasePtr pyenv)
{
    return pyenv->GetEnv();
}

EnvironmentBasePtr GetEnvironment(object o)
{
    extract<PyEnvironmentBasePtr> pyenv(o);
    if( pyenv.check() ) {
        return ((PyEnvironmentBasePtr)pyenv)->GetEnv();
    }
    return EnvironmentBasePtr();
}

object toPyEnvironment(object o)
{
    extract<PyInterfaceBasePtr> pyinterface(o);
    if( pyinterface.check() ) {
        return object(((PyInterfaceBasePtr)pyinterface)->GetEnv());
    }
    return object();
}

void LockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Lock();
}

void UnlockEnvironment(PyEnvironmentBasePtr pyenv)
{
    pyenv->Unlock();
}

object RaveGetEnvironments()
{
    std::list<EnvironmentBasePtr> listenvironments;
    OpenRAVE::RaveGetEnvironments(listenvironments);
    boost::python::list oenvironments;
    FOREACH(it,listenvironments) {
        oenvironments.append(PyEnvironmentBasePtr(new PyEnvironmentBase(*it)));
    }
    return oenvironments;
}
int RaveGetEnvironmentId(PyEnvironmentBasePtr pyenv)
{
    return OpenRAVE::RaveGetEnvironmentId(pyenv->GetEnv());
}

PyEnvironmentBasePtr RaveGetEnvironment(int id)
{
    EnvironmentBasePtr penv = OpenRAVE::RaveGetEnvironment(id);
    if( !penv ) {
        return PyEnvironmentBasePtr();
    }
    return PyEnvironmentBasePtr(new PyEnvironmentBase(penv));
}

PyInterfaceBasePtr RaveCreateInterface(PyEnvironmentBasePtr pyenv, InterfaceType type, const std::string& name)
{
    InterfaceBasePtr p = OpenRAVE::RaveCreateInterface(pyenv->GetEnv(), type, name);
    if( !p ) {
        return PyInterfaceBasePtr();
    }
    return PyInterfaceBasePtr(new PyInterfaceBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(LoadURI_overloads, LoadURI, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetCamera_overloads, SetCamera, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(StartSimulation_overloads, StartSimulation, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SetViewer_overloads, SetViewer, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CheckCollisionRays_overloads, CheckCollisionRays, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(plot3_overloads, plot3, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinestrip_overloads, drawlinestrip, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawlinelist_overloads, drawlinelist, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawarrow_overloads, drawarrow, 2, 4)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawbox_overloads, drawbox, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(drawtrimesh_overloads, drawtrimesh, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SendCommand_overloads, SendCommand, 1, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Add_overloads, Add, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Save_overloads, Save, 1, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(GetUserData_overloads, GetUserData, 0, 1)

object get_openrave_exception_unicode(openrave_exception* p)
{
    std::string s = p->message();
    return ConvertStringToUnicode(s);
}

std::string get_openrave_exception_repr(openrave_exception* p)
{
    return boost::str(boost::format("<openrave_exception('%s',ErrorCode.%s)>")%p->message()%GetErrorCodeString(p->GetCode()));
}

}

BOOST_PYTHON_MODULE(openravepy_int)
{
    using namespace openravepy;
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
    .def("GetCode", &openrave_exception::GetCode )
    .def( "__str__", &openrave_exception::message, return_copy_const_ref() )
    .def( "__unicode__", get_openrave_exception_unicode)
    .def( "__repr__", get_openrave_exception_repr)
    ;
    exception_translator<openrave_exception>();
    exception_translator<std::runtime_error>();
    exception_translator<boost::bad_function_call>();

    class_<PyEnvironmentBase, PyEnvironmentBasePtr > classenv("Environment", DOXY_CLASS(EnvironmentBase));

    {
        void (PyInterfaceBase::*setuserdata1)(PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata2)(object) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata3)(const std::string&, PyUserData) = &PyInterfaceBase::SetUserData;
        void (PyInterfaceBase::*setuserdata4)(const std::string&, object) = &PyInterfaceBase::SetUserData;
        std::string sSendCommandDoc = std::string(DOXY_FN(InterfaceBase,SendCommand)) + std::string("The calling conventions between C++ and Python differ a little.\n\n\
In C++ the syntax is::\n\n  success = SendCommand(OUT, IN)\n\n\
In python, the syntax is::\n\n\
  OUT = SendCommand(IN,releasegil)\n\
  success = OUT is not None\n\n\n\
The **releasegil** parameter controls whether the python Global Interpreter Lock should be released when executing this code. For calls that take a long time and if there are many threads running called from different python threads, releasing the GIL could speed up things a lot. Please keep in mind that releasing and re-acquiring the GIL also takes computation time.\n");
        class_<PyInterfaceBase, boost::shared_ptr<PyInterfaceBase> >("Interface", DOXY_CLASS(InterfaceBase), no_init)
        .def("GetInterfaceType",&PyInterfaceBase::GetInterfaceType, DOXY_FN(InterfaceBase,GetInterfaceType))
        .def("GetXMLId",&PyInterfaceBase::GetXMLId, DOXY_FN(InterfaceBase,GetXMLId))
        .def("GetPluginName",&PyInterfaceBase::GetPluginName, DOXY_FN(InterfaceBase,GetPluginName))
        .def("GetDescription",&PyInterfaceBase::GetDescription, DOXY_FN(InterfaceBase,GetDescription))
        .def("SetDescription",&PyInterfaceBase::SetDescription, DOXY_FN(InterfaceBase,SetDescription))
        .def("GetEnv",&PyInterfaceBase::GetEnv, DOXY_FN(InterfaceBase,GetEnv))
        .def("Clone",&PyInterfaceBase::Clone,args("ref","cloningoptions"), DOXY_FN(InterfaceBase,Clone))
        .def("SetUserData",setuserdata1,args("data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata2,args("data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata3,args("key","data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("SetUserData",setuserdata4,args("key", "data"), DOXY_FN(InterfaceBase,SetUserData))
        .def("RemoveUserData", &PyInterfaceBase::RemoveUserData, DOXY_FN(InterfaceBase, RemoveUserData))
        .def("GetUserData",&PyInterfaceBase::GetUserData, GetUserData_overloads(args("key"), DOXY_FN(InterfaceBase,GetUserData)))
        .def("SendCommand",&PyInterfaceBase::SendCommand,SendCommand_overloads(args("cmd","releasegil"), sSendCommandDoc.c_str()))
        .def("GetReadableInterfaces",&PyInterfaceBase::GetReadableInterfaces,DOXY_FN(InterfaceBase,GetReadableInterfaces))
        .def("GetReadableInterface",&PyInterfaceBase::GetReadableInterface,DOXY_FN(InterfaceBase,GetReadableInterface))
        .def("SetReadableInterface",&PyInterfaceBase::SetReadableInterface,args("xmltag","xmlreadable"), DOXY_FN(InterfaceBase,SetReadableInterface))
        .def("__repr__", &PyInterfaceBase::__repr__)
        .def("__str__", &PyInterfaceBase::__str__)
        .def("__unicode__", &PyInterfaceBase::__unicode__)
        .def("__hash__",&PyInterfaceBase::__hash__)
        .def("__eq__",&PyInterfaceBase::__eq__)
        .def("__ne__",&PyInterfaceBase::__ne__)
        ;
    }

    {
        bool (PyEnvironmentBase::*pcolb)(PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbr)(PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbb)(PyKinBodyPtr,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbbr)(PyKinBodyPtr, PyKinBodyPtr,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoll)(object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollr)(object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolll)(object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolllr)(object,object, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollb)(object, PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcollbr)(object, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolle)(object,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoller)(object, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolbe)(PyKinBodyPtr,object,object) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolber)(PyKinBodyPtr, object,object,PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyb)(boost::shared_ptr<PyRay>,PyKinBodyPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolybr)(boost::shared_ptr<PyRay>, PyKinBodyPtr, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcoly)(boost::shared_ptr<PyRay>) = &PyEnvironmentBase::CheckCollision;
        bool (PyEnvironmentBase::*pcolyr)(boost::shared_ptr<PyRay>, PyCollisionReportPtr) = &PyEnvironmentBase::CheckCollision;

        void (PyEnvironmentBase::*Lock1)() = &PyEnvironmentBase::Lock;
        void (PyEnvironmentBase::*Lock2)(float) = &PyEnvironmentBase::Lock;

        object (PyEnvironmentBase::*drawplane1)(object, object, const boost::multi_array<float,2>&) = &PyEnvironmentBase::drawplane;
        object (PyEnvironmentBase::*drawplane2)(object, object, const boost::multi_array<float,3>&) = &PyEnvironmentBase::drawplane;

        void (PyEnvironmentBase::*addkinbody1)(PyKinBodyPtr) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addkinbody2)(PyKinBodyPtr,bool) = &PyEnvironmentBase::AddKinBody;
        void (PyEnvironmentBase::*addrobot1)(PyRobotBasePtr) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addrobot2)(PyRobotBasePtr,bool) = &PyEnvironmentBase::AddRobot;
        void (PyEnvironmentBase::*addsensor1)(PySensorBasePtr) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*addsensor2)(PySensorBasePtr,bool) = &PyEnvironmentBase::AddSensor;
        void (PyEnvironmentBase::*setuserdata1)(PyUserData) = &PyEnvironmentBase::SetUserData;
        void (PyEnvironmentBase::*setuserdata2)(object) = &PyEnvironmentBase::SetUserData;
        bool (PyEnvironmentBase::*load1)(const string &) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*load2)(const string &, object) = &PyEnvironmentBase::Load;
        bool (PyEnvironmentBase::*loaddata1)(const string &) = &PyEnvironmentBase::LoadData;
        bool (PyEnvironmentBase::*loaddata2)(const string &, object) = &PyEnvironmentBase::LoadData;
        object (PyEnvironmentBase::*readrobotxmlfile1)(const string &) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmlfile2)(const string &,object) = &PyEnvironmentBase::ReadRobotURI;
        object (PyEnvironmentBase::*readrobotxmldata1)(const string &) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readrobotxmldata2)(const string &,object) = &PyEnvironmentBase::ReadRobotData;
        object (PyEnvironmentBase::*readkinbodyxmlfile1)(const string &) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmlfile2)(const string &,object) = &PyEnvironmentBase::ReadKinBodyURI;
        object (PyEnvironmentBase::*readkinbodyxmldata1)(const string &) = &PyEnvironmentBase::ReadKinBodyData;
        object (PyEnvironmentBase::*readkinbodyxmldata2)(const string &,object) = &PyEnvironmentBase::ReadKinBodyData;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile1)(const string &) = &PyEnvironmentBase::ReadInterfaceURI;
        PyInterfaceBasePtr (PyEnvironmentBase::*readinterfacexmlfile2)(const string &,object) = &PyEnvironmentBase::ReadInterfaceURI;
        object (PyEnvironmentBase::*readtrimeshfile1)(const std::string&) = &PyEnvironmentBase::ReadTrimeshURI;
        object (PyEnvironmentBase::*readtrimeshfile2)(const std::string&,object) = &PyEnvironmentBase::ReadTrimeshURI;
        scope env = classenv
                    .def(init<>())
                    .def("Reset",&PyEnvironmentBase::Reset, DOXY_FN(EnvironmentBase,Reset))
                    .def("Destroy",&PyEnvironmentBase::Destroy, DOXY_FN(EnvironmentBase,Destroy))
                    .def("CloneSelf",&PyEnvironmentBase::CloneSelf,args("options"), DOXY_FN(EnvironmentBase,CloneSelf))
                    .def("Clone",&PyEnvironmentBase::Clone,args("reference","options"), DOXY_FN(EnvironmentBase,Clone))
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
                    .def("CheckCollision",pcoller,args("link","bodyexcluded","linkexcluded","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBody::LinkConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolbe,args("body","bodyexcluded","linkexcluded"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolber,args("body","bodyexcluded","linkexcluded","report"), DOXY_FN(EnvironmentBase,CheckCollision "KinBodyConstPtr; const std::vector; const std::vector; CollisionReportPtr"))
                    .def("CheckCollision",pcolyb,args("ray","body"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcolybr,args("ray","body","report"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; KinBodyConstPtr; CollisionReportPtr"))
                    .def("CheckCollision",pcoly,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
                    .def("CheckCollision",pcolyr,args("ray"), DOXY_FN(EnvironmentBase,CheckCollision "const RAY; CollisionReportPtr"))
                    .def("CheckCollisionRays",&PyEnvironmentBase::CheckCollisionRays,
                         CheckCollisionRays_overloads(args("rays","body","front_facing_only"),
                                                      "Check if any rays hit the body and returns their contact points along with a vector specifying if a collision occured or not. Rays is a Nx6 array, first 3 columsn are position, last 3 are direction+range."))
                    .def("LoadURI",&PyEnvironmentBase::LoadURI,LoadURI_overloads(args("filename","atts"), DOXY_FN(EnvironmentBase,LoadURI)))
                    .def("Load",load1,args("filename"), DOXY_FN(EnvironmentBase,Load))
                    .def("Load",load2,args("filename","atts"), DOXY_FN(EnvironmentBase,Load))
                    .def("LoadData",loaddata1,args("data"), DOXY_FN(EnvironmentBase,LoadData))
                    .def("LoadData",loaddata2,args("data","atts"), DOXY_FN(EnvironmentBase,LoadData))
                    .def("Save",&PyEnvironmentBase::Save,Save_overloads(args("filename","options","atts"), DOXY_FN(EnvironmentBase,Save)))
                    .def("ReadRobotURI",readrobotxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                    .def("ReadRobotXMLFile",readrobotxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadRobotURI "const std::string"))
                    .def("ReadRobotURI",readrobotxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLFile",readrobotxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadRobotURI "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotData",readrobotxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLData",readrobotxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotData",readrobotxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadRobotXMLData",readrobotxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadRobotData "RobotBasePtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyURI",readkinbodyxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                    .def("ReadKinBodyXMLFile",readkinbodyxmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "const std::string"))
                    .def("ReadKinBodyURI",readkinbodyxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLFile",readkinbodyxmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyURI "KinBody; const std::string; const AttributesList"))
                    .def("ReadKinBodyData",readkinbodyxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLData",readkinbodyxmldata1,args("data"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyData",readkinbodyxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadKinBodyXMLData",readkinbodyxmldata2,args("data","atts"), DOXY_FN(EnvironmentBase,ReadKinBodyData "KinBodyPtr; const std::string; const AttributesList"))
                    .def("ReadInterfaceURI",readinterfacexmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceXMLFile",readinterfacexmlfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceURI",readinterfacexmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadInterfaceXMLFile",readinterfacexmlfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadInterfaceURI "InterfaceBasePtr; InterfaceType; const std::string; const AttributesList"))
                    .def("ReadTrimeshURI",readtrimeshfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshURI",readtrimeshfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshFile",readtrimeshfile1,args("filename"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("ReadTrimeshFile",readtrimeshfile2,args("filename","atts"), DOXY_FN(EnvironmentBase,ReadTrimeshURI))
                    .def("Add", &PyEnvironmentBase::Add, Add_overloads(args("interface","anonymous","cmdargs"), DOXY_FN(EnvironmentBase,Add)))
                    .def("AddKinBody",addkinbody1,args("body"), DOXY_FN(EnvironmentBase,AddKinBody))
                    .def("AddKinBody",addkinbody2,args("body","anonymous"), DOXY_FN(EnvironmentBase,AddKinBody))
                    .def("AddRobot",addrobot1,args("robot"), DOXY_FN(EnvironmentBase,AddRobot))
                    .def("AddRobot",addrobot2,args("robot","anonymous"), DOXY_FN(EnvironmentBase,AddRobot))
                    .def("AddSensor",addsensor1,args("sensor"), DOXY_FN(EnvironmentBase,AddSensor))
                    .def("AddSensor",addsensor2,args("sensor","anonymous"), DOXY_FN(EnvironmentBase,AddSensor))
                    .def("AddViewer",addsensor2,args("sensor","anonymous"), DOXY_FN(EnvironmentBase,AddViewer))
                    .def("RemoveKinBody",&PyEnvironmentBase::RemoveKinBody,args("body"), DOXY_FN(EnvironmentBase,RemoveKinBody))
                    .def("Remove",&PyEnvironmentBase::Remove,args("interface"), DOXY_FN(EnvironmentBase,Remove))
                    .def("GetKinBody",&PyEnvironmentBase::GetKinBody,args("name"), DOXY_FN(EnvironmentBase,GetKinBody))
                    .def("GetRobot",&PyEnvironmentBase::GetRobot,args("name"), DOXY_FN(EnvironmentBase,GetRobot))
                    .def("GetSensor",&PyEnvironmentBase::GetSensor,args("name"), DOXY_FN(EnvironmentBase,GetSensor))
                    .def("GetBodyFromEnvironmentId",&PyEnvironmentBase::GetBodyFromEnvironmentId, DOXY_FN(EnvironmentBase,GetBodyFromEnvironmentId))
                    .def("AddModule",&PyEnvironmentBase::AddModule,args("module","args"), DOXY_FN(EnvironmentBase,AddModule))
                    .def("LoadProblem",&PyEnvironmentBase::AddModule,args("module","args"), DOXY_FN(EnvironmentBase,AddModule))
                    .def("RemoveProblem",&PyEnvironmentBase::RemoveProblem,args("prob"), DOXY_FN(EnvironmentBase,RemoveProblem))
                    .def("GetModules",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                    .def("GetLoadedProblems",&PyEnvironmentBase::GetModules, DOXY_FN(EnvironmentBase,GetModules))
                    .def("SetPhysicsEngine",&PyEnvironmentBase::SetPhysicsEngine,args("physics"), DOXY_FN(EnvironmentBase,SetPhysicsEngine))
                    .def("GetPhysicsEngine",&PyEnvironmentBase::GetPhysicsEngine, DOXY_FN(EnvironmentBase,GetPhysicsEngine))
                    .def("RegisterCollisionCallback",&PyEnvironmentBase::RegisterCollisionCallback,args("callback"), DOXY_FN(EnvironmentBase,RegisterCollisionCallback))
                    .def("HasRegisteredCollisionCallbacks",&PyEnvironmentBase::HasRegisteredCollisionCallbacks,DOXY_FN(EnvironmentBase,HasRegisteredCollisionCallbacks))
                    .def("StepSimulation",&PyEnvironmentBase::StepSimulation,args("timestep"), DOXY_FN(EnvironmentBase,StepSimulation))
                    .def("StartSimulation",&PyEnvironmentBase::StartSimulation,StartSimulation_overloads(args("timestep","realtime"), DOXY_FN(EnvironmentBase,StartSimulation)))
                    .def("StopSimulation",&PyEnvironmentBase::StopSimulation, DOXY_FN(EnvironmentBase,StopSimulation))
                    .def("GetSimulationTime",&PyEnvironmentBase::GetSimulationTime, DOXY_FN(EnvironmentBase,GetSimulationTime))
                    .def("IsSimulationRunning",&PyEnvironmentBase::IsSimulationRunning, DOXY_FN(EnvironmentBase,IsSimulationRunning))
                    .def("Lock",Lock1,"Locks the environment mutex.")
                    //.def("Lock",Lock2,args("timeout"), "Locks the environment mutex with a timeout.")
                    .def("Unlock",&PyEnvironmentBase::Unlock,"Unlocks the environment mutex.")
                    .def("LockPhysics",Lock1,args("lock"), "Locks the environment mutex.")
                    .def("LockPhysics",Lock2,args("lock","timeout"), "Locks the environment mutex with a timeout.")
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
                    .def("SetUserData",setuserdata1,args("data"), DOXY_FN(InterfaceBase,SetUserData))
                    .def("SetUserData",setuserdata2,args("data"), DOXY_FN(InterfaceBase,SetUserData))
                    .def("GetUserData",&PyEnvironmentBase::GetUserData, DOXY_FN(InterfaceBase,GetUserData))
                    .def("__enter__",&PyEnvironmentBase::__enter__)
                    .def("__exit__",&PyEnvironmentBase::__exit__)
                    .def("__eq__",&PyEnvironmentBase::__eq__)
                    .def("__ne__",&PyEnvironmentBase::__ne__)
                    .def("__repr__",&PyEnvironmentBase::__repr__)
                    .def("__str__",&PyEnvironmentBase::__str__)
                    .def("__unicode__",&PyEnvironmentBase::__unicode__)
        ;

        object selectionoptions = enum_<EnvironmentBase::SelectionOptions>("SelectionOptions" DOXY_ENUM(SelectionOptions))
                                  .value("NoRobots",EnvironmentBase::SO_NoRobots)
                                  .value("Robots",EnvironmentBase::SO_Robots)
                                  .value("Everything",EnvironmentBase::SO_Everything)
                                  .value("Body",EnvironmentBase::SO_Body)
                                  .value("AllExceptBody",EnvironmentBase::SO_AllExceptBody)
        ;
        env.attr("TriangulateOptions") = selectionoptions;
    }

    {
        scope options = class_<DummyStruct>("options")
                        .add_static_property("returnTransformQuaternion",GetReturnTransformQuaternions,SetReturnTransformQuaternions);
    }

    scope().attr("__version__") = OPENRAVE_VERSION_STRING;
    scope().attr("__author__") = "Rosen Diankov";
    scope().attr("__copyright__") = "2009-2012 Rosen Diankov (rosen.diankov@gmail.com)";
    scope().attr("__license__") = "Lesser GPL";
    scope().attr("__docformat__") = "restructuredtext";

    openravepy::init_openravepy_global();

    openravepy::init_openravepy_collisionchecker();
    openravepy::init_openravepy_controller();
    openravepy::init_openravepy_ikparameterization();
    openravepy::init_openravepy_iksolver();
    openravepy::init_openravepy_kinbody();
    openravepy::init_openravepy_robot();
    openravepy::init_openravepy_module();
    openravepy::init_openravepy_physicsengine();
    openravepy::init_openravepy_planner();
    openravepy::init_openravepy_trajectory();
    openravepy::init_openravepy_sensor();
    openravepy::init_openravepy_sensorsystem();
    openravepy::init_openravepy_spacesampler();
    openravepy::init_openravepy_viewer();

    def("RaveGetEnvironmentId",openravepy::RaveGetEnvironmentId,DOXY_FN1(RaveGetEnvironmentId));
    def("RaveGetEnvironment",openravepy::RaveGetEnvironment,DOXY_FN1(RaveGetEnvironment));
    def("RaveGetEnvironments",openravepy::RaveGetEnvironments,DOXY_FN1(RaveGetEnvironments));
    def("RaveCreateInterface",openravepy::RaveCreateInterface,args("env","type","name"),DOXY_FN1(RaveCreateInterface));
}
