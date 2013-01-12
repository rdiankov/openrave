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
#include <csignal>

#if defined(_WIN32) && !defined(sighandler_t)
typedef void (*sighandler_t)(int);
#endif

namespace openravepy {
#ifndef _WIN32
static struct sigaction s_signalActionPrev;
#else
static sighandler_t s_signalActionPrev;
#endif
static std::list<ViewerBasePtr> s_listViewersToQuit;
static int s_nInterruptCount=0;
}

extern "C" void openravepy_viewer_sigint_handler(int sig) //, siginfo_t *siginfo, void *context)
{
    RAVELOG_VERBOSE("openravepy_viewer_sigint_handler\n");
    openravepy::s_nInterruptCount += 1;
    //PyErr_SetInterrupt();
    //PyGILState_STATE gstate = PyGILState_Ensure();
//    try
//    {
//        exec("raise KeyboardInterrupt()\n"); // boost::python::globals(),locals);
//        //exec("import thread; thread.interrupt_main()\n"); // boost::python::globals(),locals);
//    }
//    catch( const error_already_set& e )
//    {
//        // should be the interrupt we just raised...
//        //RAVELOG_WARN(boost::str(boost::format("failed to throw KeyboardInterrupt from signal handler: %s\n")%e));
//    }
////
//    PyGILState_Release(gstate);


//    FOREACHC(itviewer, openravepy::s_listViewersToQuit) {
//        ++(*itviewer)->_nSetInterrupt;
//    }
//        (*itviewer)->quitmainloop();
//    }
//    openravepy::s_listViewersToQuit.clear();
    // is this call necessary? perhaps could get the C++ planners out of their loops?
//    RaveDestroy();

#ifndef _WIN32
    //struct sigaction act;
    if( sigaction(SIGINT, &openravepy::s_signalActionPrev, NULL) < 0 ) {
        RAVELOG_WARN("failed to restore old signal\n");
    }
    kill(0 /*getpid()*/, SIGINT);
    //sigaction(SIGINT,&act,NULL);
#else
    signal(SIGINT, openravepy::s_signalActionPrev);
#endif
}

namespace openravepy {

class PyViewerBase : public PyInterfaceBase
{
protected:
    ViewerBasePtr _pviewer;
    UserDataPtr _viewercallback;
    int64_t _sig_thread_id;

    static bool _ViewerCallback(object fncallback, PyEnvironmentBasePtr pyenv, KinBody::LinkPtr plink,RaveVector<float> position,RaveVector<float> direction)
    {
        object res;
        PyGILState_STATE gstate = PyGILState_Ensure();
        try {
            res = fncallback(openravepy::toPyKinBodyLink(plink,pyenv),toPyVector3(position),toPyVector3(direction));
        }
        catch(...) {
            RAVELOG_ERROR("exception occured in python viewer callback:\n");
            PyErr_Print();
        }
        PyGILState_Release(gstate);
        extract<bool> xb(res);
        if( xb.check() ) {
            return (bool)xb;
        }
        extract<int> xi(res);
        if( xi.check() ) {
            return (int)xi;
        }
        extract<double> xd(res);
        if( xd.check() ) {
            return (double)xd>0;
        }
        return true;
    }

    void _ThreadCallback()
    {
        if( openravepy::s_nInterruptCount > 0 ) {
            --openravepy::s_nInterruptCount;
            if( _sig_thread_id != 0 ) {
                // calling handler for other thread
                PyGILState_STATE gstate = PyGILState_Ensure();
                int count = PyThreadState_SetAsyncExc(_sig_thread_id, PyExc_KeyboardInterrupt);
                if( count == 0 ) {
                    RAVELOG_WARN("PyThreadState_SetAsyncExc invalid thread id %d\n",_sig_thread_id);
                }
                else if(count != 1 ) {
                    RAVELOG_WARN("we're in trouble!\n");
                    PyThreadState_SetAsyncExc(_sig_thread_id, NULL);
                }
                PyGILState_Release(gstate);

#ifndef _WIN32
                // restore the viewer signal handler
                memset(&openravepy::s_signalActionPrev,0,sizeof(openravepy::s_signalActionPrev));
                struct sigaction act;
                memset(&act,0,sizeof(act));
                //sigfillset(&act.sa_mask);
                sigemptyset(&act.sa_mask);
                act.sa_flags = 0;
                act.sa_handler = &openravepy_viewer_sigint_handler;
                int ret = sigaction(SIGINT,&act,&openravepy::s_signalActionPrev);
                if( ret < 0 ) {
                    RAVELOG_WARN("failed to set sigaction, might not be able to use Ctrl-C\n");
                }
#else
                openravepy::s_signalActionPrev = signal(SIGINT,&openravepy_viewer_sigint_handler);
#endif
            }
            else {
                RAVELOG_INFO("destroying viewer and openrave runtime\n");
                _pviewer->quitmainloop();
                RaveDestroy();
            }
        }
    }

public:

    PyViewerBase(ViewerBasePtr pviewer, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pviewer, pyenv), _pviewer(pviewer) {
        _sig_thread_id = 0;
        if( !!_pviewer ) {
            _viewercallback = _pviewer->RegisterViewerThreadCallback(boost::bind(&PyViewerBase::_ThreadCallback,this));
        }
    }
    virtual ~PyViewerBase() {
    }

    ViewerBasePtr GetViewer() {
        return _pviewer;
    }

    int main(bool bShow, int64_t sig_thread_id=0)
    {
        _sig_thread_id = sig_thread_id;
        openravepy::s_listViewersToQuit.push_back(_pviewer);
        // very helpful: https://www.securecoding.cert.org/confluence/display/cplusplus/SIG01-CPP.+Understand+implementation-specific+details+regarding+signal+handler+persistence
#ifndef _WIN32
        memset(&openravepy::s_signalActionPrev,0,sizeof(openravepy::s_signalActionPrev));
        struct sigaction act;
        memset(&act,0,sizeof(act));
        //sigfillset(&act.sa_mask);
        sigemptyset(&act.sa_mask);
        act.sa_flags = 0;
        act.sa_handler = &openravepy_viewer_sigint_handler;
        int ret = sigaction(SIGINT,&act,&openravepy::s_signalActionPrev);
        if( ret < 0 ) {
            RAVELOG_WARN("failed to set sigaction, might not be able to use Ctrl-C\n");
        }
#else
        openravepy::s_signalActionPrev = signal(SIGINT,&openravepy_viewer_sigint_handler);
#endif
        //s_prevsignal = signal(SIGINT,viewer_sigint_handler); // control C
        // have to release the GIL since this is an infinite loop
        openravepy::PythonThreadSaverPtr statesaver(new openravepy::PythonThreadSaver());
        bool bSuccess=false;
        try {
            bSuccess = _pviewer->main(bShow);
        }
        catch(const error_already_set& e) {
            RAVELOG_WARN("received python error\n");
        }
        catch(...) {
        }
        openravepy::s_listViewersToQuit.remove(_pviewer); // might not be in list anymore
#ifndef _WIN32
        // restore again?
        sigaction(SIGINT,&openravepy::s_signalActionPrev,NULL);
#else
        signal(SIGINT,openravepy::s_signalActionPrev);
#endif
        return bSuccess;
    }

    void quitmainloop() {
        return _pviewer->quitmainloop();
    }

    void SetSize(int w, int h) {
        _pviewer->SetSize(w,h);
    }
    void Move(int x, int y) {
        _pviewer->Move(x,y);
    }
    void SetName(const string &title) {
        _pviewer->SetName(title);
    }
    string GetName() {
        return _pviewer->GetName();
    }

    object RegisterCallback(object properties, object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        UserDataPtr p = _pviewer->RegisterItemSelectionCallback(boost::bind(&PyViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p ) {
            throw openrave_exception("no registration callback returned");
        }
        return openravepy::GetUserData(p);
    }

    object RegisterItemSelectionCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        UserDataPtr p = _pviewer->RegisterItemSelectionCallback(boost::bind(&PyViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p ) {
            throw openrave_exception("no registration callback returned");
        }
        return openravepy::GetUserData(p);
    }

    void EnvironmentSync() {
        return _pviewer->EnvironmentSync();
    }

    void SetCamera(object transform) {
        _pviewer->SetCamera(ExtractTransform(transform));
    }
    void SetCamera(object transform, float focalDistance) {
        _pviewer->SetCamera(ExtractTransform(transform),focalDistance);
    }

    void SetBkgndColor(object ocolor) {
        _pviewer->SetBkgndColor(ExtractVector3(ocolor));
    }

    object GetCameraTransform() {
        return ReturnTransform(_pviewer->GetCameraTransform());
    }

    object GetCameraImage(int width, int height, object extrinsic, object oKK)
    {
        vector<float> vKK = ExtractArray<float>(oKK);
        if( vKK.size() != 4 ) {
            throw openrave_exception("KK needs to be of size 4");
        }
        SensorBase::CameraIntrinsics KK(vKK[0],vKK[1],vKK[2],vKK[3]);
        vector<uint8_t> memory;
        if( !_pviewer->GetCameraImage(memory, width,height,RaveTransform<float>(ExtractTransform(extrinsic)), KK) ) {
            throw openrave_exception("failed to get camera image");
        }
        std::vector<npy_intp> dims(3); dims[0] = height; dims[1] = width; dims[2] = 3;
        return toPyArray(memory,dims);
    }
};

ViewerBasePtr GetViewer(PyViewerBasePtr pyviewer)
{
    return !pyviewer ? ViewerBasePtr() : pyviewer->GetViewer();
}

PyInterfaceBasePtr toPyViewer(ViewerBasePtr pviewer, PyEnvironmentBasePtr pyenv)
{
    return !pviewer ? PyInterfaceBasePtr() : PyInterfaceBasePtr(new PyViewerBase(pviewer,pyenv));
}

PyViewerBasePtr RaveCreateViewer(PyEnvironmentBasePtr pyenv, const std::string& name)
{
    ViewerBasePtr p = OpenRAVE::RaveCreateViewer(GetEnvironment(pyenv), name);
    if( !p ) {
        return PyViewerBasePtr();
    }
    return PyViewerBasePtr(new PyViewerBase(p,pyenv));
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(main_overloads, main, 1, 2)

void init_openravepy_viewer()
{
    memset(&s_signalActionPrev,0,sizeof(s_signalActionPrev));

    {
        void (PyViewerBase::*setcamera1)(object) = &PyViewerBase::SetCamera;
        void (PyViewerBase::*setcamera2)(object,float) = &PyViewerBase::SetCamera;
        scope viewer = class_<PyViewerBase, boost::shared_ptr<PyViewerBase>, bases<PyInterfaceBase> >("Viewer", DOXY_CLASS(ViewerBase), no_init)
                       .def("main",&PyViewerBase::main, main_overloads(args("show","sig_thread_id"), DOXY_FN(ViewerBase,main)))
                       .def("quitmainloop",&PyViewerBase::quitmainloop, DOXY_FN(ViewerBase,quitmainloop))
                       .def("SetSize",&PyViewerBase::SetSize, DOXY_FN(ViewerBase,SetSize))
                       .def("Move",&PyViewerBase::Move, DOXY_FN(ViewerBase,Move))
                       .def("SetTitle",&PyViewerBase::SetName, DOXY_FN(ViewerBase,SetName))
                       .def("SetName",&PyViewerBase::SetName, DOXY_FN(ViewerBase,SetName))
                       .def("GetName",&PyViewerBase::GetName, DOXY_FN(ViewerBase,GetName))
                       .def("RegisterCallback",&PyViewerBase::RegisterCallback, args("callback"), DOXY_FN(ViewerBase,RegisterItemSelectionCallback))
                       .def("RegisterItemSelectionCallback",&PyViewerBase::RegisterItemSelectionCallback, args("callback"), DOXY_FN(ViewerBase,RegisterItemSelectionCallback))
                       .def("EnvironmentSync",&PyViewerBase::EnvironmentSync, DOXY_FN(ViewerBase,EnvironmentSync))
                       .def("SetCamera",setcamera1,args("transform"), DOXY_FN(ViewerBase,SetCamera))
                       .def("SetCamera",setcamera2,args("transform","focalDistance"), DOXY_FN(ViewerBase,SetCamera))
                       .def("SetBkgndColor",&PyViewerBase::SetBkgndColor,DOXY_FN(ViewerBase,SetBkgndColor))
                       .def("GetCameraTransform",&PyViewerBase::GetCameraTransform, DOXY_FN(ViewerBase,GetCameraTransform))
                       .def("GetCameraImage",&PyViewerBase::GetCameraImage,args("width","height","transform","K"), DOXY_FN(ViewerBase,GetCameraImage))
        ;

//        enum_<ViewerBase::ViewerEvents>("Events" DOXY_ENUM(ViewerEvents))
//        .value("ItemSelection",ViewerBase::VE_ItemSelection)
//        ;
    }

    def("RaveCreateViewer",openravepy::RaveCreateViewer,args("env","name"),DOXY_FN1(RaveCreateViewer));
}

}
