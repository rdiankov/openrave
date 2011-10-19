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

class PyViewerBase : public PyInterfaceBase
{
protected:
    ViewerBasePtr _pviewer;

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
public:

    PyViewerBase(ViewerBasePtr pviewer, PyEnvironmentBasePtr pyenv) : PyInterfaceBase(pviewer, pyenv), _pviewer(pviewer) {
    }
    virtual ~PyViewerBase() {
    }

    ViewerBasePtr GetViewer() {
        return _pviewer;
    }

    int main(bool bShow) {
        return _pviewer->main(bShow);
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

    PyVoidHandle RegisterCallback(object properties, object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        boost::shared_ptr<void> p = _pviewer->RegisterItemSelectionCallback(boost::bind(&PyViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p ) {
            throw openrave_exception("no registration callback returned");
        }
        return PyVoidHandle(p);
    }

    PyVoidHandle RegisterItemSelectionCallback(object fncallback)
    {
        if( !fncallback ) {
            throw openrave_exception("callback not specified");
        }
        boost::shared_ptr<void> p = _pviewer->RegisterItemSelectionCallback(boost::bind(&PyViewerBase::_ViewerCallback,fncallback,_pyenv,_1,_2,_3));
        if( !p ) {
            throw openrave_exception("no registration callback returned");
        }
        return PyVoidHandle(p);
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

namespace openravepy {

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

void init_openravepy_viewer()
{
    {
        void (PyViewerBase::*setcamera1)(object) = &PyViewerBase::SetCamera;
        void (PyViewerBase::*setcamera2)(object,float) = &PyViewerBase::SetCamera;
        scope viewer = class_<PyViewerBase, boost::shared_ptr<PyViewerBase>, bases<PyInterfaceBase> >("Viewer", DOXY_CLASS(ViewerBase), no_init)
                       .def("main",&PyViewerBase::main, DOXY_FN(ViewerBase,main))
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
