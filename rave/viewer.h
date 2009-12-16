// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_VIEWER_H
#define OPENRAVE_VIEWER_H

namespace OpenRAVE {

/// Base class for the graphics and gui engine. Derive a class called
/// RaveViewer for anything more specific
class RAVE_API RaveViewerBase : public InterfaceBase
{
public:
    enum ViewerEvents
    {
        /// mouse button is clicked. If the function
        /// returns true, then the object will be selected. Otherwise, the object remains unselected.
        VE_ItemSelection = 1,
    };

    RaveViewerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Viewer, penv) {}
    virtual ~RaveViewerBase() {}

    /// reset the camera depending on its mode
    virtual void UpdateCameraTransform() = 0;

    /// goes into the main loop
    /// \param bShow if true will show the window
    virtual int main(bool bShow = true) = 0;
    /// destroys the main loop
    virtual void quitmainloop() = 0;

    /// Retries a 24bit RGB image of dimensions width and height from the current scene
    /// extrinsic is the rotation and translation of the camera
    /// pKK is 4 values such that the intrinsic matrix can be reconstructed [pKK[0] 0 pKK[2]; 0 pKK[1] pKK[3]; 0 0 1];
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK) = 0;
    virtual bool WriteCameraImage(int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& KK, const std::string& filename, const std::string& extension) = 0;
    virtual void SetCamera(const RaveVector<float>& pos, const RaveVector<float>& quat) = 0;
    virtual void SetCameraLookAt(const RaveVector<float>& lookat, const RaveVector<float>& campos, const RaveVector<float>& camup) = 0;
    virtual RaveTransform<float> GetCameraTransform() = 0;

    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0) = 0;
    virtual void* plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0) = 0;

    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) = 0;
    virtual void* drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) = 0;
    virtual void* drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) = 0;

    virtual void* drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) = 0;

    virtual void* drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents) = 0;
    
    virtual void* drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) = 0;

    virtual void closegraph(void* handle) = 0;
    virtual void deselect() = 0;

    virtual void Reset() = 0;
    virtual void SetBkgndColor(const RaveVector<float>& color) = 0;

    /// callback viewer function when for viewer events
    /// first parameter - target openrave link
    /// second parameter - offset
    /// third parameter - direction
    typedef boost::function<bool(KinBody::LinkPtr plink,RaveVector<float>,RaveVector<float>)> ViewerCallbackFn;

    /// registers a function with the viewer that gets called everytime a specified event occurs (part of ViewerEvents enum)
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered
    virtual boost::shared_ptr<void> RegisterCallback(int properties, const ViewerCallbackFn& fncallback) = 0;

    /// controls whether the viewer synchronizes with the newest environment
    virtual void SetEnvironmentSync(bool bUpdate) = 0;

    /// forces synchronization with the environment, returns when the environment is fully synchronized.
    /// Note that this method might not work if environment is locked in current thread
    virtual void EnvironmentSync() = 0;

    virtual void ViewerSetSize(int w, int h) = 0;
    virtual void ViewerMove(int x, int y) = 0;
    virtual void ViewerSetTitle(const std::string& ptitle) = 0;
    virtual bool LoadModel(const std::string& pfilename) = 0;

private:
    virtual const char* GetHash() const { return OPENRAVE_VIEWER_HASH; }
};

} // end namespace OpenRAVE

#endif
