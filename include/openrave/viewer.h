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
*/
#ifndef OPENRAVE_VIEWER_H
#define OPENRAVE_VIEWER_H

namespace OpenRAVE {

/// \brief Handle holding the plot from the viewers. The plot will continue to be drawn as long as a reference to this handle is held.
///
/// Designed to be multi-thread safe and destruction and modification of the viewer plot can be done at any time. The viewers
/// internally handle synchronization and threading issues.
class OPENRAVE_API GraphHandle
{
public:
    virtual ~GraphHandle() {}

    /// \brief Changes the underlying transformation of the plot. <b>[multi-thread safe]</b>
    ///
    /// \param t new transformation of the plot
    virtual void SetTransform(const RaveTransform<float>& t) { throw openrave_exception("GraphHandle::SetTransform not implemented",ORE_NotImplemented); }
    /// \brief Shows or hides the plot without destroying its resources. <b>[multi-thread safe]</b>
    virtual void SetShow(bool bshow) { throw openrave_exception("GraphHandle::SetShow not implemented",ORE_NotImplemented); }
};

typedef boost::shared_ptr<GraphHandle> GraphHandlePtr;
typedef boost::shared_ptr<GraphHandle const> GraphHandleConstPtr;
typedef boost::weak_ptr<GraphHandle const> GraphHandleWeakPtr;

/** \brief <b>[interface]</b> Base class for the graphics and gui engine that renders the environment and provides visual sensor information.
    \ingroup interfaces
*/
class OPENRAVE_API ViewerBase : public InterfaceBase
{
public:
    enum ViewerEvents
    {
        /// mouse button is clicked. If the function
        /// returns true, then the object will be selected. Otherwise, the object remains unselected.
        VE_ItemSelection = 1,
    };

    ViewerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Viewer, penv) {}
    virtual ~ViewerBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_Viewer; }
    
    /// \brief goes into the main loop
    /// \param bShow if true will show the window
    virtual int main(bool bShow = true) = 0;
    /// destroys the main loop
    virtual void quitmainloop() = 0;

    //@{ GUI interaction methods

    /// \brief Set the camera transformation.
    ///
    /// \param trans new camera transformation in the world coordinate system
    /// \param focalDistance The new focal distance of the camera (higher values is higher zoom). If 0, then the previous focal distance is preserved.
    virtual void SetCamera(const RaveTransform<float>& trans, float focalDistance=0) { throw openrave_exception("ViewerBase::SetCamera not implemented",ORE_NotImplemented); }

    /// \brief Return the current camera transform that the viewer is rendering the environment at.
    virtual RaveTransform<float> GetCameraTransform() { throw openrave_exception("ViewerBase::GetCameraTransform not implemented",ORE_NotImplemented); }

    /// \brief reset the camera depending on its mode
    virtual void UpdateCameraTransform() { throw openrave_exception("ViewerBase::UpdateCameraTransform not implemented",ORE_NotImplemented); }

    /** \brief Renders a 24bit RGB image of dimensions width and height from the current scene.

        The camera is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
        rendered with the plotX and drawX functions are hidden by default. Some viewers support the SetFiguresInCamera command to allow graphs to be also displayed.
        \param memory the memory where the image will be stored at, has to store 3*width*height
        \param width width of the image
        \param height height of the image
        \param t the rotation and translation of the camera. Note that +z is treated as the camera direction axis! So all points in front of the camera have a positive dot product with its +z direction.
        \param intrinsics the intrinsic parameters of the camera defining FOV, distortion, principal point, and focal length. The focal length is used to define the near plane for culling.
    */
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& intrinsics) { throw openrave_exception("ViewerBase::GetCameraImage not implemented",ORE_NotImplemented); }

    //@}

    virtual void Reset() { throw openrave_exception("ViewerBase::Reset not implemented",ORE_NotImplemented); }
    virtual void SetBkgndColor(const RaveVector<float>& color) { throw openrave_exception("ViewerBase::SetBkgndColor not implemented",ORE_NotImplemented); }

    /// callback viewer function when for viewer events
    /// first parameter - target openrave link
    /// second parameter - offset
    /// third parameter - direction
    typedef boost::function<bool(KinBody::LinkPtr plink,RaveVector<float>,RaveVector<float>)> ViewerCallbackFn;

    /// \brief registers a function with the viewer that gets called everytime a specified event occurs (part of ViewerEvents enum)
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered
    virtual boost::shared_ptr<void> RegisterCallback(int properties, const ViewerCallbackFn& fncallback) { throw openrave_exception("ViewerBase::RegisterCallback not implemented",ORE_NotImplemented); }

    /// \brief controls whether the viewer synchronizes with the newest environment
    virtual void SetEnvironmentSync(bool bUpdate) { throw openrave_exception("ViewerBase::SetEnvironmentSync not implemented",ORE_NotImplemented); }

    /// \brief forces synchronization with the environment, returns when the environment is fully synchronized.
    ///
    /// Note that this method might not work if environment is locked in current thread
    virtual void EnvironmentSync() { throw openrave_exception("ViewerBase::EnvironmentSync not implemented",ORE_NotImplemented); }

    virtual void ViewerSetSize(int w, int h) { throw openrave_exception("ViewerBase::ViewerSetSize not implemented",ORE_NotImplemented); }
    virtual void ViewerMove(int x, int y) { throw openrave_exception("ViewerBase::ViewerMove not implemented",ORE_NotImplemented); }
    virtual void ViewerSetTitle(const std::string& ptitle) { throw openrave_exception("ViewerBase::ViewerSetTitle not implemented",ORE_NotImplemented); }

    /// \deprecated (11/03/02) Any type of model should be added through the openrave environment instead of viewer directly.
    virtual bool LoadModel(const std::string& pfilename) { throw openrave_exception("ViewerBase::LoadModel not implemented",ORE_NotImplemented); }

protected:
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0) { throw openrave_exception("ViewerBase::plot3 not implemented",ORE_NotImplemented); }
    virtual GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false) { throw openrave_exception("ViewerBase::plot3 not implemented",ORE_NotImplemented); }

    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { throw openrave_exception("ViewerBase::drawlinestrip not implemented",ORE_NotImplemented); }
    virtual GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { throw openrave_exception("ViewerBase::drawlinestrip not implemented",ORE_NotImplemented); }

    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) { throw openrave_exception("ViewerBase::drawlinelist not implemented",ORE_NotImplemented); }
    virtual GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) { throw openrave_exception("ViewerBase::drawlinelist not implemented",ORE_NotImplemented); }

    virtual GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) { throw openrave_exception("ViewerBase::drawarrow not implemented",ORE_NotImplemented); }

    virtual GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) { throw openrave_exception("ViewerBase::drawbox not implemented",ORE_NotImplemented); }
    virtual GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture) { throw openrave_exception("ViewerBase::drawplane not implemented",ORE_NotImplemented); }
    
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) { throw openrave_exception("ViewerBase::drawtrimesh not implemented",ORE_NotImplemented); }
    virtual GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors) { throw openrave_exception("ViewerBase::drawtrimesh not implemented",ORE_NotImplemented); }

private:
    virtual const char* GetHash() const { return OPENRAVE_VIEWER_HASH; }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
#else
    friend class ::Environment;
#endif
#endif
};

typedef ViewerBase RaveViewerBase RAVE_DEPRECATED;
typedef ViewerBasePtr RaveViewerBasePtr RAVE_DEPRECATED;
typedef ViewerBaseConstPtr RaveViewerBaseConstPtr RAVE_DEPRECATED;
typedef ViewerBaseWeakPtr RaveViewerBaseWeakPtr RAVE_DEPRECATED;

} // end namespace OpenRAVE

#endif
