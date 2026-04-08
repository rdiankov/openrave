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
#ifndef OPENRAVEPY_INTERNAL_VIEWER_H
#define OPENRAVEPY_INTERNAL_VIEWER_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>
#include <openravepy/openravepy_sensorbase.h>

namespace openravepy {
using py::object;

class PyViewerBase : public PyInterfaceBase
{
protected:
    ViewerBasePtr _pviewer;
    UserDataPtr _viewercallback;
    int64_t _sig_thread_id;

    static bool _ViewerCallback(object fncallback, PyEnvironmentBasePtr pyenv, KinBody::LinkPtr plink,RaveVector<float> position,RaveVector<float> direction);
    void _ThreadCallback();

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

    int main(bool bShow, int64_t sig_thread_id=0);

    void quitmainloop();

    void SetSize(const double w, const double h);
    void Move(const double x, const double y);
    void Show(int showtype);
    void SetName(const std::string &title);
    void SetUserText(const std::string &userText);
    void SetTextSize(const double size);
    std::string GetName();

    object RegisterCallback(object properties, object fncallback);

    object RegisterItemSelectionCallback(object fncallback);

    void EnvironmentSync();

    void SetCamera(object transform);
    void SetCamera(object transform, float focalDistance);

    void SetBkgndColor(object ocolor);

    py::array_t<dReal> GetCameraTransform();

    float GetCameraDistanceToFocus();

    py::array_t<uint8_t> GetCameraImage(int width, int height, object extrinsic, object oKK);

    PyCameraIntrinsicsPtr GetCameraIntrinsics();
};

} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_VIEWER_H
