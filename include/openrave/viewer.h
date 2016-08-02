// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
/** \file viewer.h
    \brief Graphical interface functions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_VIEWER_H
#define OPENRAVE_VIEWER_H

namespace OpenRAVE {

/** \brief Handle holding the plot from the viewers. The plot will continue to be drawn as long as a reference to this handle is held.

    Designed to be multi-thread safe and destruction and modification of the viewer plot can be done at any time. The viewers
    internally handle synchronization and threading issues.
 */
class OPENRAVE_API GraphHandle
{
public:
    virtual ~GraphHandle() {
    }

    /// \brief Changes the underlying transformation of the plot. <b>[multi-thread safe]</b>
    ///
    /// \param t new transformation of the plot
    virtual void SetTransform(const RaveTransform<float>& t) OPENRAVE_DUMMY_IMPLEMENTATION;
    /// \brief Shows or hides the plot without destroying its resources. <b>[multi-thread safe]</b>
    virtual void SetShow(bool bshow) OPENRAVE_DUMMY_IMPLEMENTATION;
};

typedef std::shared_ptr<GraphHandle> GraphHandlePtr;
typedef std::shared_ptr<GraphHandle const> GraphHandleConstPtr;
typedef std::weak_ptr<GraphHandle const> GraphHandleWeakPtr;

/** \brief <b>[interface]</b> Base class for the graphics and gui engine that renders the environment and provides visual sensor information. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_viewer.
    \ingroup interfaces
 */
class OPENRAVE_API ViewerBase : public InterfaceBase
{
public:
    enum ViewerEvents
    {
        VE_ItemSelection = 1,
    } RAVE_DEPRECATED;

    ViewerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Viewer, penv) {
    }
    virtual ~ViewerBase() {
    }

    /// \brief return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_Viewer;
    }

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief goes into the main loop
    ///
    /// \param bShow if true will show the window
    virtual int main(bool bShow = true) = 0;

    /// \brief destroys the main loop
    virtual void quitmainloop() = 0;

    //@{ GUI interaction methods

    /// \brief Set the camera transformation.
    ///
    /// \param trans new camera transformation in the world coordinate system
    /// \param distanceToFocus The new distance of the camera to the center of its rotation (higher values is higher zoom). If 0, then the previous focal distance is preserved.
    virtual void SetCamera(const RaveTransform<float>& trans, float distanceToFocus=0) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Return the current camera transform that the viewer is rendering the environment at.
    virtual RaveTransform<float> GetCameraTransform() const OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Return the distance to the camera focus position (ie center of rotation)
    virtual float GetCameraDistanceToFocus() const OPENRAVE_DUMMY_IMPLEMENTATION;
    
    /// \brief Return the closest camera intrinsics that the viewer is rendering the environment at.
    virtual geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const OPENRAVE_DUMMY_IMPLEMENTATION;

    /** \brief Renders a 24bit RGB image of dimensions width and height from the current scene.

        The camera is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
        rendered with the plotX and drawX functions are hidden by default. Some viewers support the SetFiguresInCamera command to allow graphs to be also displayed.
        \param memory the memory where the image will be stored at, has to store 3*width*height
        \param width width of the image, if 0 the width of the viewer is used
        \param height height of the image, if 0 the width of the viewer is used
        \param t the rotation and translation of the camera. Note that +z is treated as the camera direction axis! So all points in front of the camera have a positive dot product with its +z direction.
        \param intrinsics the intrinsic parameters of the camera defining FOV, distortion, principal point, and focal length. The focal length is used to define the near plane for culling.
     */
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& intrinsics) OPENRAVE_DUMMY_IMPLEMENTATION;

    //@}

    virtual void Reset() OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual void SetBkgndColor(const RaveVector<float>& color) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief callback function for item selection
    ///
    /// If the function returns true, then the object will be selected. Otherwise, the object remains unselected.
    /// callback(target link,offset,direction)
    typedef boost::function<bool (KinBody::LinkPtr plink,RaveVector<float>,RaveVector<float>)> ItemSelectionCallbackFn;

    /// \brief registers a function with the viewer that gets called everytime mouse button is clicked
    ///
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief callback function for item selection
    /// callback(imagememory,width,height,pixeldepth)
    ///
    /// \param imagememory width x height x pixeldepth RGB image
    typedef boost::function<void (const uint8_t*,int,int,int)> ViewerImageCallbackFn;

    /// \brief registers a function with the viewer that gets called for every new image rendered.
    ///
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterViewerImageCallback(const ViewerImageCallbackFn& fncallback) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief callback function for viewer thread
    typedef boost::function<void ()> ViewerThreadCallbackFn;

    /// \brief registers a function with the viewer that gets called in the viewer's GUI thread for every cycle the viewer refreshes at
    ///
    /// The environment will not be locked when the thread is called
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback) OPENRAVE_DUMMY_IMPLEMENTATION;


    /// \brief controls whether the viewer synchronizes with the newest environment automatically
    virtual void SetEnvironmentSync(bool bUpdate) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief forces synchronization with the environment, returns when the environment is fully synchronized.
    ///
    /// Note that this method might not work if environment is locked in current thread
    virtual void EnvironmentSync() OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual void SetSize(int w, int h) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \deprecated (11/06/13)
    virtual void ViewerSetSize(int w, int h) RAVE_DEPRECATED {
        SetSize(w,h);
    }

    virtual void Move(int x, int y) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \deprecated (11/06/13)
    virtual void ViewerMove(int x, int y) RAVE_DEPRECATED {
        Move(x,y);
    }

    virtual void SetName(const std::string& name) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \deprecated (11/06/13)
    virtual void ViewerSetTitle(const std::string& ptitle) RAVE_DEPRECATED {
        SetName(ptitle);
    }

    /// \brief controls showing the viewer.
    ///
    /// \param showtype If zero, will hide all viewers. If != 0, should show viewers (dependent on plugin could have different meanings)
    virtual void Show(int showtype) OPENRAVE_DUMMY_IMPLEMENTATION;
    
    virtual const std::string& GetName() const OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \deprecated (11/06/10)
    virtual void UpdateCameraTransform() RAVE_DEPRECATED OPENRAVE_DUMMY_IMPLEMENTATION;
    /// \deprecated (11/06/10)
    typedef ItemSelectionCallbackFn ViewerCallbackFn RAVE_DEPRECATED;
    /// \deprecated (11/06/10)
    virtual UserDataPtr RegisterCallback(int properties, const ItemSelectionCallbackFn& fncallback) RAVE_DEPRECATED
    {
        return RegisterItemSelectionCallback(fncallback);
    }

protected:
    /// \deprecated (12/12/11)
    virtual void SetViewerData(KinBodyPtr body, UserDataPtr data) RAVE_DEPRECATED {
        body->SetUserData(GetXMLId(), data);
    }

    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false) OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture) OPENRAVE_DUMMY_IMPLEMENTATION;

    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) OPENRAVE_DUMMY_IMPLEMENTATION;
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors) OPENRAVE_DUMMY_IMPLEMENTATION;

    inline ViewerBasePtr shared_viewer() {
        return std::static_pointer_cast<ViewerBase>(shared_from_this());
    }
    inline ViewerBaseConstPtr shared_viewer_const() const {
        return std::static_pointer_cast<ViewerBase const>(shared_from_this());
    }

private:
    virtual const char* GetHash() const {
        return OPENRAVE_VIEWER_HASH;
    }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
#else
    friend class ::Environment;
#endif
#endif
};

} // end namespace OpenRAVE

#endif
