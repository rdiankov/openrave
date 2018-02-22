#ifndef OPENRAVE_QTOGRE_VIEWER_H
#define OPENRAVE_QTOGRE_VIEWER_H

#include "qtogre.h"
#include "qtogrewindow.h"

using namespace OpenRAVE;

namespace qtogrerave {

class QtOgreViewer;
typedef boost::shared_ptr<QtOgreViewer> QtOgreViewerPtr;
typedef boost::weak_ptr<QtOgreViewer> QtOgreViewerWeakPtr;
typedef boost::shared_ptr<QtOgreViewer const> QtOgreViewerConstPtr;

class QtOgreViewer : public ViewerBase
{
public:
    QtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput);
    virtual ~QtOgreViewer();

    /// \brief notified when a body has been removed from the environment
    virtual void RemoveKinBody(KinBodyPtr pbody) {
        if( !!pbody ) {
            pbody->RemoveUserData("qtogre");
        }
    }

    /// \brief goes into the main loop
    ///
    /// \param bShow if true will show the window
    int main(bool bShow = true);

    bool startmainloop(std::ostream& sout, std::istream& sinput);

    /// \brief destroys the main loop
    void quitmainloop();

    //@{ GUI interaction methods

    /// \brief Set the camera transformation.
    ///
    /// \param trans new camera transformation in the world coordinate system
    /// \param distanceToFocus The new distance of the camera to the center of its rotation (higher values is higher zoom). If 0, then the previous focal distance is preserved.
    virtual void SetCamera(const RaveTransform<float>& trans, float distanceToFocus=0) {}

    /// \brief Return the current camera transform that the viewer is rendering the environment at.
    virtual RaveTransform<float> GetCameraTransform() const {}

    /// \brief Return the distance to the camera focus position (ie center of rotation)
    virtual float GetCameraDistanceToFocus() const {}

    /// \brief Return the closest camera intrinsics that the viewer is rendering the environment at.
    virtual geometry::RaveCameraIntrinsics<float> GetCameraIntrinsics() const {}

    /** \brief Renders a 24bit RGB image of dimensions width and height from the current scene.

        The camera is meant to show the underlying OpenRAVE world as a robot would see it, so all graphs
        rendered with the plotX and drawX functions are hidden by default. Some viewers support the SetFiguresInCamera command to allow graphs to be also displayed.
        \param memory the memory where the image will be stored at, has to store 3*width*height
        \param width width of the image, if 0 the width of the viewer is used
        \param height height of the image, if 0 the width of the viewer is used
        \param t the rotation and translation of the camera. Note that +z is treated as the camera direction axis! So all points in front of the camera have a positive dot product with its +z direction.
        \param intrinsics the intrinsic parameters of the camera defining FOV, distortion, principal point, and focal length. The focal length is used to define the near plane for culling.
     */
    virtual bool GetCameraImage(std::vector<uint8_t>& memory, int width, int height, const RaveTransform<float>& t, const SensorBase::CameraIntrinsics& intrinsics) {}

    //@}

    virtual void Reset() {}
    virtual void SetBkgndColor(const RaveVector<float>& color) {}

    /// \brief registers a function with the viewer that gets called everytime mouse button is clicked
    ///
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterItemSelectionCallback(const ItemSelectionCallbackFn& fncallback) {}

    /// \brief registers a function with the viewer that gets called for every new image rendered.
    ///
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterViewerImageCallback(const ViewerImageCallbackFn& fncallback) {}

    /// \brief registers a function with the viewer that gets called in the viewer's GUI thread for every cycle the viewer refreshes at
    ///
    /// The environment will not be locked when the thread is called
    /// \return a handle to the callback. If this handle is deleted, the callback will be unregistered.
    virtual UserDataPtr RegisterViewerThreadCallback(const ViewerThreadCallbackFn& fncallback) {}


    /// \brief controls whether the viewer synchronizes with the newest environment automatically
    virtual void SetEnvironmentSync(bool bUpdate) {}

    /// \brief forces synchronization with the environment, returns when the environment is fully synchronized.
    ///
    /// Note that this method might not work if environment is locked in current thread
    virtual void EnvironmentSync() {}

    virtual void SetSize(int w, int h) {}

    virtual void Move(int x, int y) {}

    virtual void SetName(const std::string& name) {}

    /// \brief controls showing the viewer.
    ///
    /// \param showtype If zero, will hide all viewers. If != 0, should show viewers (dependent on plugin could have different meanings)
    virtual void Show(int showtype) {}

    virtual const std::string& GetName() const {}

    /// \deprecated (11/06/10)
    virtual void UpdateCameraTransform() RAVE_DEPRECATED {}
    /// \deprecated (11/06/10)
    typedef ItemSelectionCallbackFn ViewerCallbackFn RAVE_DEPRECATED;
    /// \deprecated (11/06/10)
    virtual UserDataPtr RegisterCallback(int properties, const ItemSelectionCallbackFn& fncallback) RAVE_DEPRECATED
    {
        return RegisterItemSelectionCallback(fncallback);
    }

    void _drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);

protected:
    GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const RaveVector<float>& color, int drawstyle = 0) {}
    GraphHandlePtr plot3(const float* ppoints, int numPoints, int stride, float fPointSize, const float* colors, int drawstyle = 0, bool bhasalpha=false) {}

    GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) {}
    GraphHandlePtr drawlinestrip(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) {}

    GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const RaveVector<float>& color) {}
    GraphHandlePtr drawlinelist(const float* ppoints, int numPoints, int stride, float fwidth, const float* colors) {}

    GraphHandlePtr drawarrow(const RaveVector<float>& p1, const RaveVector<float>& p2, float fwidth, const RaveVector<float>& color) {}

    GraphHandlePtr drawbox(const RaveVector<float>& vpos, const RaveVector<float>& vextents);
    GraphHandlePtr drawplane(const RaveTransform<float>& tplane, const RaveVector<float>& vextents, const boost::multi_array<float,3>& vtexture) {}

    GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const RaveVector<float>& color) {}
    GraphHandlePtr drawtrimesh(const float* ppoints, int stride, const int* pIndices, int numTriangles, const boost::multi_array<float,2>& colors) {}

    inline QtOgreViewerPtr shared_viewer() {
        return boost::static_pointer_cast<QtOgreViewer>(shared_from_this());
    }
    inline QtOgreViewerWeakPtr weak_viewer() {
        return QtOgreViewerWeakPtr(shared_viewer());
    }
    inline QtOgreViewerConstPtr shared_viewer_const() const {
        return boost::static_pointer_cast<QtOgreViewer const>(shared_from_this());
    }

private:
    boost::shared_ptr<QtOgreWindow> _ogreWindow;
};

}; // namespace qtogrerave

#endif