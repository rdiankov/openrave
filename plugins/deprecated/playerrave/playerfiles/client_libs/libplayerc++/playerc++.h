/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/***************************************************************************
 * Desc: Player v2.0 C++ client
 * Authors: Brad Kratochvil, Toby Collett
 *
 * Date: 23 Sep 2005
 # CVS: $Id: playerc++.h,v 1.54.2.9 2007/04/30 21:57:46 gerkey Exp $
 **************************************************************************/


#ifndef PLAYERCC_H
#define PLAYERCC_H

#include <cmath>
#include <string>
#include <list>
#include <vector>

#include "libplayerc/playerc.h"
#include "libplayerc++/utility.h"
#include "libplayerc++/playerc++config.h"
#include "libplayerc++/playerclient.h"
#include "libplayerc++/playererror.h"
#include "libplayerc++/clientproxy.h"

#ifdef HAVE_BOOST_SIGNALS
  #include <boost/signal.hpp>
  #include <boost/bind.hpp>
#endif

#ifdef HAVE_BOOST_THREAD
  #include <boost/thread/mutex.hpp>
  #include <boost/thread/thread.hpp>
  #include <boost/thread/xtime.hpp>
#endif

namespace PlayerCc
{

// /**
// * The @p SomethingProxy class is a template for adding new subclasses of
// * ClientProxy.  You need to have at least all of the following:
// */
// class SomethingProxy : public ClientProxy
// {
//
//   private:
//
//     // Subscribe
//     void Subscribe(uint aIndex);
//     // Unsubscribe
//     void Unsubscribe();
//
//     // libplayerc data structure
//     playerc_something_t *mDevice;
//
//   public:
//     // Constructor
//     SomethingProxy(PlayerClient *aPc, uint aIndex=0);
//     // Destructor
//     ~SomethingProxy();
//
// };

/** @ingroup player_clientlib_cplusplus
 * @addtogroup player_clientlib_cplusplus_proxies Proxies
 * @brief A proxy class is associated with each kind of device

  The proxies all inherit from @p ClientProxy and implement the functions
  from @ref player_clientlib_libplayerc.

 @{

 */

// ==============================================================
//
// These are alphabetized, please keep them that way!!!
//
// ==============================================================

/**
The @p ActArrayProxy class is used to control a @ref interface_actarray
device.
 */
class ActArrayProxy : public ClientProxy
{
  private:

   void Subscribe(uint aIndex);
   void Unsubscribe();

   // libplayerc data structure
   playerc_actarray_t *mDevice;

  public:

    /// Default constructor
    ActArrayProxy(PlayerClient *aPc, uint aIndex=0);
    /// Default destructor
    ~ActArrayProxy();

    /// Geometry request - call before getting the
    /// geometry of a joint through the accessor method
    void RequestGeometry(void);

    /// Power control
    void SetPowerConfig(uint8_t aVal);
    /// Brakes control
    void SetBrakesConfig(uint8_t aVal);
    /// Speed control
    void SetSpeedConfig(uint aJoint, float aSpeed);

    /// Send an actuator to a position
    void MoveTo(uint aJoint, float aPos);
    /// Move an actuator at a speed
    void MoveAtSpeed(uint aJoint, float aSpeed);
    /// Send an actuator, or all actuators, home
    void MoveHome(int aJoint);

    /// moves with a trajectory, trajformat determines the format in pData
    /// at the moment, pData is just a contiguous array of points each point
    /// specifies all the actuator positions
    int MoveWithTrajectory(int numpoints, int trajformat, float time_of_divergence, uint32_t trajectory_id, const float* pData);

    /// Signals the driver to start the currently queued trajectory. Only valid
    ///if PLAYER_ACTARRAY_TRAJFMT_FLAG_SYNCEXECUTION was specified as a trajectory flag format.
    void Start(int64_t timestamp);

    /// Gets the number of actuators in the array
    uint GetCount(void) const { return GetVar(mDevice->actuators_count); }
    /// Accessor method for getting an actuator's data
    player_actarray_actuator_t GetActuatorData(uint aJoint) const;
    /// Same again for getting actuator geometry
    player_actarray_actuatorgeom_t GetActuatorGeom(uint aJoint) const;

    /// Current trajectory robot is executing. If 0, then robot is not executing a trajectory, but can still be moving.
    uint32_t GetTrajectoryId() const;

    /// how far the robot is along the current trajectory
    float GetTrajectoryTime() const;

    /// Actuator data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p ActArrayProxy named @p ap, the following
    ///    expressions are equivalent: @p ap.GetActuatorData[0] and @p ap[0].
    player_actarray_actuator_t operator [](uint aJoint)
      { return(GetActuatorData(aJoint)); }
};

/**
The @p AioProxy class is used to read from a @ref interface_aio
(analog I/O) device.  */
class AioProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_aio_t *mDevice;

  public:

    AioProxy (PlayerClient *aPc, uint aIndex=0);
    ~AioProxy();

    /// Accessor function
    uint GetCount() const { return(GetVar(mDevice->voltages_count)); };

    /// Accessor function
    double GetVoltage(uint aIndex)  const
      { return(GetVar(mDevice->voltages[aIndex])); };

    /// Set the output voltage
    void SetVoltage(uint aIndex, double aVoltage);

    /// AioProxy data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p AioProxy named @p bp, the following
    ///    expressions are equivalent: @p ap.GetVoltage(0) and @p ap[0].
    double operator [](uint aIndex) const
      { return GetVoltage(aIndex); }

};

#if 0 // Not in libplayerc

/**
The @p AudioProxy class controls an @ref interface_audio device.
*/
class AudioProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_audio_t *mDevice;

  public:

    AudioProxy(PlayerClient *aPc, uint aIndex=0)
    ~AudioProxy();

    uint GetCount() const { return(GetVar(mDevice->count)); };

    double GetFrequency(uint aIndex) const
      { return(GetVar(mDevice->frequency[aIndex])); };
    double GetAmplitude(uint aIndex) const
      { return(GetVar(mDevice->amplitude[aIndex])); };

    // Play a fixed-frequency tone
    void PlayTone(uint aFreq, uint aAmp, uint aDur);
};

/**
The @p AudioDspProxy class controls an @ref interface_audiodsp device.

@todo can we make this a subclass of @ref AudioProxy?
*/
class AudioDspProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_audiodsp_t *mDevice;

  public:
    AudioDspProxy(PlayerClient *aPc, uint aIndex=0);
    ~AudioDspProxy

    /** Format code of each sample */
    uint SetFormat(int aFormat);

    /** Rate at which to sample (Hz) */
    uint SetRate(uint aRate);

    uint GetCount() const { return(GetVar(mDevice->count)); };

    double GetFrequency(uint aIndex) const
      { return(GetVar(mDevice->frequency[aIndex])); };
    double GetAmplitude(uint aIndex) const
      { return(GetVar(mDevice->amplitude[aIndex])); };

    void Configure(uint aChan, uint aRate, int16_t aFormat=0xFFFFFFFF);

    void RequestConfigure();

    /// Play a fixed-frequency tone
    void PlayTone(uint aFreq, uint aAmp, uint aDur);
    void PlayChirp(uint aFreq, uint aAmp, uint aDur,
                   const uint8_t aBitString, uint aBitStringLen);
    void Replay();

    /// Print the current data.
    void Print ();
};

/**
The @p AudioMixerProxy class controls an @ref interface_audiomixer device.
*/
class AudioMixerProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_audiodsp_t *mDevice;

  public:

    AudioMixerProxy (PlayerClient *aPc, uint aIndex=0);

    void GetConfigure();

    void SetMaster(uint aLeft, uint aRight);
    void SetPCM(uint aLeft, uint aRight);
    void SetLine(uint aLeft, uint aRight);
    void SetMic(uint aLeft, uint aRight);
    void SetIGain(uint aGain);
    void SetOGain(uint aGain);

};

/**
The @p BlinkenlightProxy class is used to enable and disable
a flashing indicator light, and to set its period, via a @ref
interface_blinkenlight device */
class BlinkenLightProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_blinkenlight_t *mDevice;

  public:
    /** Constructor.
        Leave the access field empty to start unconnected.
    */
    BlinkenLightProxy(PlayerClient *aPc, uint aIndex=0);
    ~BlinkenLightProxy();

    // true: indicator light enabled, false: disabled.
    bool GetEnable();

    /** The current period (one whole on/off cycle) of the blinking
        light. If the period is zero and the light is enabled, the light
        is on.
    */
    void SetPeriod(double aPeriod);

    /** Set the state of the indicator light. A period of zero means
        the light will be unblinkingly on or off. Returns 0 on
        success, else -1.
      */
    void SetEnable(bool aSet);
};

#endif

/**
The @p BlobfinderProxy class is used to control a  @ref
interface_blobfinder device.  It contains no methods.  The latest
color blob data is stored in @p blobs, a dynamically allocated 2-D array,
indexed by color channel.
*/
class BlobfinderProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_blobfinder_t *mDevice;

  public:
    /// default contsructor
    BlobfinderProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~BlobfinderProxy();

    /// returns the number of blobs
    uint GetCount() const { return GetVar(mDevice->blobs_count); };
    /// returns a blob
    playerc_blobfinder_blob_t GetBlob(uint aIndex) const
      { return GetVar(mDevice->blobs[aIndex]);};

    /// get the width of the image
    uint GetWidth() const { return GetVar(mDevice->width); };
    /// get the height of the image
    uint GetHeight() const { return GetVar(mDevice->height); };

    /// Blobfinder data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p BlobfinderProxy named @p bp, the following
    ///    expressions are equivalent: @p bp.GetBlob[0] and @p bp[0].
    playerc_blobfinder_blob_t operator [](uint aIndex) const
      { return(GetBlob(aIndex)); }

/*
    /// Set the color to be tracked
    void SetTrackingColor(uint aReMin=0,   uint aReMax=255, uint aGrMin=0,
                          uint aGrMax=255, uint aBlMin=0,   uint aBlMax=255);
    void SetImagerParams(int aContrast, int aBrightness,
                         int aAutogain, int aColormode);
    void SetContrast(int aC);
    void SetColorMode(int aM);
    void SetBrightness(int aB);
    void SetAutoGain(int aG);*/

};

/**
The @p BumperProxy class is used to read from a @ref
interface_bumper device.
*/
class BumperProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_bumper_t *mDevice;

  public:

    BumperProxy(PlayerClient *aPc, uint aIndex=0);
    ~BumperProxy();

    uint GetCount() const { return GetVar(mDevice->bumper_count); };

    /// Returns true if the specified bumper has been bumped, false otherwise.
    uint IsBumped(uint aIndex) const
      { return GetVar(mDevice->bumpers[aIndex]); };

    /// Returns true if any bumper has been bumped, false otherwise.
    bool IsAnyBumped();

    /// Requests the geometries of the bumpers.
    void RequestBumperConfig();

    /// Returns the number bumper poses
    uint GetPoseCount() const { return GetVar(mDevice->pose_count); };

    /// Returns a specific bumper pose
    player_bumper_define_t GetPose(uint aIndex) const
      { return GetVar(mDevice->poses[aIndex]); };

    /// BumperProxy data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p BumperProxy named @p bp, the following
    ///    expressions are equivalent: @p bp.IsBumped[0] and @p bp[0].
    bool operator [](uint aIndex) const
      { return IsBumped(aIndex); }

};

/**
The @p CameraProxy class can be used to get images from a @ref
interface_camera device. */
class CameraProxy : public ClientProxy
{

  private:

    virtual void Subscribe(uint aIndex);
    virtual void Unsubscribe();

    // libplayerc data structure
    playerc_camera_t *mDevice;

    std::string mPrefix;
    int mFrameNo;

  public:

    /// Constructor
    CameraProxy (PlayerClient *aPc, uint aIndex=0);

    virtual ~CameraProxy();

    /// Save the frame
    /// @arg aPrefix is the string prefix to name the image.
    /// @arg aWidth is the number of 0s to pad the image numbering with.
    void SaveFrame(const std::string aPrefix, uint aWidth=4);

    /// decompress the image
    void Decompress();

    /// Image color depth
    uint GetDepth() const { return GetVar(mDevice->bpp); };

    /// Image dimensions (pixels)
    uint GetWidth() const { return GetVar(mDevice->width); };

    /// Image dimensions (pixels)
    uint GetHeight() const { return GetVar(mDevice->height); };

    /// @brief Image format
    /// Possible values include
    /// - @ref PLAYER_CAMERA_FORMAT_MONO8
    /// - @ref PLAYER_CAMERA_FORMAT_MONO16
    /// - @ref PLAYER_CAMERA_FORMAT_RGB565
    /// - @ref PLAYER_CAMERA_FORMAT_RGB888
    uint GetFormat() const { return GetVar(mDevice->format); };

    /// Size of the image (bytes)
    uint GetImageSize() const { return GetVar(mDevice->image_count); };

    /// @brief Image data
    /// This function copies the image data into the data buffer aImage.
    /// The buffer should be allocated according to the width, height, and
    /// depth of the image.  The size can be found by calling @ref GetImageSize().
    void GetImage(uint8_t* aImage) const
      {
        return GetVarByRef(mDevice->image,
                           mDevice->image+GetVar(mDevice->image_count),
                           aImage);
      };

    /// @brief What is the compression type?
    /// Currently supported compression types are:
    /// - @ref PLAYER_CAMERA_COMPRESS_RAW
    /// - @ref PLAYER_CAMERA_COMPRESS_JPEG
    uint GetCompression() const { return GetVar(mDevice->compression); };

};


/**
The @p DioProxy class is used to read from a @ref interface_dio
(digital I/O) device.
*/
class DioProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_dio_t *mDevice;

  public:
    /// constructor
    DioProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~DioProxy();

    /// The number of valid digital inputs.
    uint GetCount() const { return GetVar(mDevice->count); };

    /// A bitfield of the current digital inputs.
    uint32_t GetDigin() const { return GetVar(mDevice->digin); };

    /// Get a specific bit
    bool GetInput(uint aIndex) const;

    /// Set the output to the bitfield aDigout
    void SetOutput(uint aCount, uint32_t aDigout);

    /// DioProxy data access operator.
    ///    This operator provides an alternate way of access the dio data.
    ///    For example, given a @p DioProxy named @p dp, the following
    ///    expressions are equivalent: @p dp.GetInput(0) and @p dp[0].
    uint operator [](uint aIndex) const
      { return GetInput(aIndex); }
};

// /**
// The @p EnergyProxy class is used to read from an @ref
// interface_energy device.
// */
// class EnergyProxy : public ClientProxy
// {
//   private:
//
//     void Subscribe(uint aIndex);
//     void Unsubscribe();
//
//     // libplayerc data structure
//     playerc_energy_t *mDevice;
//
// public:
//
//     EnergyProxy(PlayerClient *aPc, uint aIndex=0);
//     ~EnergyProxy();
//
//     /** These members give the current amount of energy stored [Joules] */
//     uint GetJoules() const { return GetVar(mDevice->joules); };
//     /** the amount of energy current being consumed [Watts] */
//     uint GetWatts() const { return GetVar(mDevice->watts); };
//     /** The charging flag is true if we are currently charging, else false. */
//     bool GetCharging() const { return GetVar(mDevice->charging); };
// };

/**
The @p FiducialProxy class is used to control @ref
interface_fiducial devices.  The latest set of detected beacons
is stored in the @p beacons array.
*/
class FiducialProxy : public ClientProxy
{
  private:
    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_fiducial_t *mDevice;

  public:
    /// constructor
    FiducialProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~FiducialProxy();

    /// The number of beacons detected
    uint GetCount() const { return GetVar(mDevice->fiducials_count); };

    /// Get detected beacon description
    player_fiducial_item_t GetFiducialItem(uint aIndex) const
      { return GetVar(mDevice->fiducials[aIndex]);};

    /// The pose of the sensor
    player_pose_t GetSensorPose() const
      { return GetVar(mDevice->fiducial_geom.pose);};

    /// The size of the sensor
    player_bbox_t GetSensorSize() const
      { return GetVar(mDevice->fiducial_geom.size);};

    /// The size of the most recently detected fiducial
    player_bbox_t GetFiducialSize() const
      { return GetVar(mDevice->fiducial_geom.fiducial_size);};

    /// Get the sensor's geometry configuration
    void RequestGeometry();

    /// FiducialProxy data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p FiducialProxy named @p fp, the following
    ///    expressions are equivalent: @p fp.GetFiducialItem[0] and @p fp[0].
    player_fiducial_item_t operator [](uint aIndex) const
      { return GetFiducialItem(aIndex); }
};

/**
The @p VisionserverProxy class provides pose information on tracked objects.  The latest set of detected objects
is stored in the @p objects array.
*/
class VisionserverProxy : public ClientProxy
{
  private:
    void Subscribe(uint aIndex);
    void Unsubscribe();

    /// libplayerc data structure
    playerc_visionserver_t *mDevice;

    bool _bChanged;

    /// read signal to update _bChanged
    virtual void UpdateChanged();
  public:
    /// constructor
    VisionserverProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~VisionserverProxy();

    // request the geometry of the object
    void RequestGeometry();

    /// Get the server's type
    std::string GetType() const
      { return GetVar(std::string(mDevice->_geom.type));};

    /// Get the object's geometry configuration; fills in the proxy
    /// \return true if data has changed since last call to GetVisionObjects
    bool GetObjects(std::list<player_visionserver_object_t>& listobjects) {

      size_t numobjs = GetVar(mDevice->_data.numobjects);
      listobjects.clear();
      for(size_t i = 0; i < numobjs; ++i)
          listobjects.push_back(mDevice->_data.objectdata[i]);
      bool bChanged = _bChanged;
      _bChanged = false;
      return bChanged;
    };
};

/**
The @p GpsProxy class is used to control a @ref interface_gps
device.  The latest pose data is stored in three class attributes.  */
class GpsProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_gps_t *mDevice;

  public:

    // Constructor
    GpsProxy(PlayerClient *aPc, uint aIndex=0);

    ~GpsProxy();

    /// Latitude and longitude, in degrees.
    double GetLatitude() const { return GetVar(mDevice->lat); };
    double GetLongitude() const { return GetVar(mDevice->lon); };

    /// Altitude, in meters.
    double GetAltitude() const { return GetVar(mDevice->alt); };

    /// Number of satellites in view.
    uint GetSatellites() const { return GetVar(mDevice->sat_count); };

    /// Fix quality
    uint GetQuality() const { return GetVar(mDevice->quality); };

    /// Horizontal dilution of position (HDOP)
    double GetHdop() const { return GetVar(mDevice->hdop); };

    /// Vertical dilution of position (HDOP)
    double GetVdop() const { return GetVar(mDevice->vdop); };

    /// UTM easting and northing (meters).
    double GetUtmEasting() const { return GetVar(mDevice->utm_e); };
    double GetUtmNorthing() const { return GetVar(mDevice->utm_n); };

    /// Time, since the epoch
    double GetTime() const { return GetVar(mDevice->utc_time); };

    /// Errors
    double GetErrHorizontal() const { return GetVar(mDevice->err_horz); };
    double GetErrVertical() const { return GetVar(mDevice->err_vert); };
};

/**
 * The @p Graphics2dProxy class is used to draw simple graphics into a
 * rendering device provided by Player using the graphics2d
 * interface. For example, the Stage plugin implements this interface
 * so you can draw into the Stage window. This is very useful to
 * visualize what's going on in your controller.
 */
class Graphics2dProxy : public ClientProxy
{

  private:

    // Subscribe
    void Subscribe(uint aIndex);
    // Unsubscribe
    void Unsubscribe();

    // libplayerc data structure
    playerc_graphics2d_t *mDevice;

  public:
    // Constructor
    Graphics2dProxy(PlayerClient *aPc, uint aIndex=0);
    // Destructor
    ~Graphics2dProxy();

    /// Set the current pen color
    void Color(player_color_t col);

    /// Set the current pen color
    void Color(uint8_t red,  uint8_t green,  uint8_t blue,  uint8_t alpha);

    /// Clear the drawing area
    void Clear(void);

    /// Draw a set of points
    void DrawPoints(player_point_2d_t pts[], int count);

    /// Draw a polygon defined by a set of points
    void DrawPolygon(player_point_2d_t pts[],
                     int count,
                     bool filled,
                     player_color_t fill_color);

    /// Draw a line connecting  set of points
    void DrawPolyline(player_point_2d_t pts[], int count);
};

/**
 * The @p Graphics3dProxy class is used to draw simple graphics into a
 * rendering device provided by Player using the graphics3d
 * interface.
 */
class Graphics3dProxy : public ClientProxy
{

  private:

    // Subscribe
    void Subscribe(uint aIndex);
    // Unsubscribe
    void Unsubscribe();

    // libplayerc data structure
    playerc_graphics3d_t *mDevice;

  public:
    // Constructor
    Graphics3dProxy(PlayerClient *aPc, uint aIndex=0);
    // Destructor
    ~Graphics3dProxy();

    /// Set the current pen color
    void Color(player_color_t col);

    /// Set the current pen color
    void Color(uint8_t red,  uint8_t green,  uint8_t blue,  uint8_t alpha);

    /// Clear the drawing area
    void Clear(void);

    /// Draw a set of verticies
    void Draw(player_graphics3d_draw_mode_t mode, player_point_3d_t pts[], int count);

};

/**
The @p GripperProxy class is used to control a @ref
interface_gripper device.  The latest gripper data held in a
handful of class attributes.  A single method provides user control.
*/
class GripperProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_gripper_t *mDevice;

  public:

    /// constructor
    GripperProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~GripperProxy();

    ///
    uint GetState() const { return GetVar(mDevice->state); };
    ///
    uint GetBeams() const { return GetVar(mDevice->beams); };
    ///
    uint GetOuterBreakBeam() const
      { return GetVar(mDevice->outer_break_beam); };
    ///
    uint GetInnerBreakBeam() const
      { return GetVar(mDevice->inner_break_beam); };
    ///
    uint GetPaddlesOpen() const
      { return GetVar(mDevice->paddles_open); };
    ///
    uint GetPaddlesClosed() const
      { return GetVar(mDevice->paddles_closed); };
    ///
    uint GetPaddlesMoving() const
      { return GetVar(mDevice->paddles_moving); };
    ///
    uint GetGripperError() const
      { return GetVar(mDevice->gripper_error); };
    ///
    uint GetLiftUp() const { return GetVar(mDevice->lift_up); };
    ///
    uint GetLiftDown() const { return GetVar(mDevice->lift_down); };
    ///
    uint GetLiftMoving() const { return GetVar(mDevice->lift_moving); };
    ///
    uint GetLiftError() const { return GetVar(mDevice->lift_error); };

    /// Send a gripper command.  Look in the Player user manual for details
    /// on the command and argument.
    void SetGrip(uint8_t aCmd, uint8_t aArg=0);
};



/**
The @p IrProxy class is used to control an @ref interface_ir
device.
*/
class IrProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_ir_t *mDevice;

  public:

    /// Constructor
    IrProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~IrProxy();

    /// get the number of IR rangers
    uint GetCount() const { return GetVar(mDevice->ranges.ranges_count); };
    /// get the current range
    double GetRange(uint aIndex) const
      { return GetVar(mDevice->ranges.ranges[aIndex]); };
    /// get the current voltage
    double GetVoltage(uint aIndex) const
      { return GetVar(mDevice->ranges.voltages[aIndex]); };
    /// get the number of poses
    uint GetPoseCount() const { return GetVar(mDevice->poses.poses_count); };
    /// get a particular pose
    player_pose_t GetPose(uint aIndex) const
      {return GetVar(mDevice->poses.poses[aIndex]);};

    /// Request IR pose information
    void RequestGeom();

    /// Range access operator.
    /// This operator provides an alternate way of access the range data.
    /// For example, given a @p IrProxy named @p ip, the following
    /// expressions are equivalent: @p ip.GetRange[0] and @p ip[0].
    double operator [](uint aIndex) const
      { return GetRange(aIndex); }

};

/**
The @p LaserProxy class is used to control a @ref interface_laser
device.  The latest scan data is held in two arrays: @p ranges and @p
intensity.  The laser scan range, resolution and so on can be configured
using the Configure() method.  */
class LaserProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_laser_t *mDevice;

    double aMinLeft;
    double aMinRight;

    // local storage of config
    double min_angle, max_angle, scan_res, range_res;
    bool intensity;

  public:

    /// constructor
    LaserProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~LaserProxy();

    /// Number of points in scan
    uint GetCount() const { return GetVar(mDevice->scan_count); };
    
    /// Max range for the latest set of data (meters)
    double GetMaxRange() const { return GetVar(mDevice->max_range); };

    /// Angular resolution of scan (radians)
    double GetScanRes() const { return GetVar(mDevice->scan_res); };

    /// Range resolution of scan (mm)
    double GetRangeRes() const { return GetVar(mDevice->range_res); };


    /// Scan range for the latest set of data (radians)
    double GetMinAngle() const { return GetVar(mDevice->scan_start); };
    /// Scan range for the latest set of data (radians)
    double GetMaxAngle() const
    {
      scoped_lock_t lock(mPc->mMutex);
      return mDevice->scan_start + mDevice->scan_count*mDevice->scan_res;
    };

    /// Whether or not reflectance (i.e., intensity) values are being returned.
    bool IntensityOn() const { return GetVar(mDevice->intensity_on); };

//    /// Scan data (polar): range (m) and bearing (radians)
//    double GetScan(uint aIndex) const
//      { return GetVar(mDevice->scan[aIndex]); };

    /// Scan data (Cartesian): x,y (m)
    player_point_2d_t GetPoint(uint aIndex) const
      { return GetVar(mDevice->point[aIndex]); };


    /// get the range
    double GetRange(uint aIndex) const
      { return GetVar(mDevice->ranges[aIndex]); };

    /// get the bearing
    double GetBearing(uint aIndex) const
      { return GetVar(mDevice->scan[aIndex][1]); };


    /// get the intensity
    int GetIntensity(uint aIndex) const
      { return GetVar(mDevice->intensity[aIndex]); };

    /// Configure the laser scan pattern.  Angles @p min_angle and
    /// @p max_angle are measured in radians.
    /// @p scan_res is measured in units of 0.01 degrees;
    /// valid values are: 25 (0.25 deg), 50 (0.5 deg) and
    /// 100 (1 deg).  @p range_res is measured in mm; valid values
    /// are: 1, 10, 100.  Set @p intensity to @p true to
    /// enable intensity measurements, or @p false to disable.
    void Configure(double aMinAngle,
                   double aMaxAngle,
                   uint aScanRes,
                   uint aRangeRes,
                   bool aIntensity);

    /// Get the current laser configuration; it is read into the
    /// relevant class attributes.
    void RequestConfigure();

    /// Get the laser's geometry; it is read into the
    /// relevant class attributes.
    void RequestGeom();

    /// Accessor for the pose of the laser with respect to its parent
    /// object (e.g., a robot).  Fill it in by calling RequestGeom.
    player_pose_t GetPose()
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);

      p.px = mDevice->pose[0];
      p.py = mDevice->pose[1];
      p.pa = mDevice->pose[2];
      return(p);
    }
    
    /// Accessor for the pose of the laser's parent object (e.g., a robot).
    /// Filled in by some (but not all) laser data messages.
    player_pose_t GetRobotPose()
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);

      p.px = mDevice->robot_pose[0];
      p.py = mDevice->robot_pose[1];
      p.pa = mDevice->robot_pose[2];
      return(p);
    }

    /// Accessor for the size (fill it in by calling RequestGeom)
    player_bbox_t GetSize()
    {
      player_bbox_t b;
      scoped_lock_t lock(mPc->mMutex);

      b.sl = mDevice->size[0];
      b.sw = mDevice->size[1];
      return(b);
    }

    /// Min left
    double MinLeft () { return aMinLeft; }
    /// Min right
    double MinRight () { return aMinRight; }

    /// Range access operator.  This operator provides an alternate
    /// way of access the range data.  For example, given an @p
    /// LaserProxy named @p lp, the following expressions are
    /// equivalent: @p lp.ranges[0], @p lp.Ranges(0),
    /// and @p lp[0].
    double operator [] (uint index) const
      { return GetRange(index);}

};


/**
The @p LimbProxy class is used to control a @ref interface_limb
device.
 */
class LimbProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

   // libplayerc data structure
    playerc_limb_t *mDevice;

  public:

    LimbProxy(PlayerClient *aPc, uint aIndex=0);
    ~LimbProxy();

    /// Geometry request - call before getting the
    /// geometry of a joint through the accessor method
    void RequestGeometry(void);

    /// Power control
    void SetPowerConfig(bool aVal);
    /// Brakes control
    void SetBrakesConfig(bool aVal);
    /// Speed control
    void SetSpeedConfig(float aSpeed);

    /// Move the limb to the home position
    void MoveHome(void);
    /// Stop the limb immediately
    void Stop(void);
    /// Move the end effector to a given pose
    void SetPose(float aPX, float aPY, float aPZ,
                 float aAX, float aAY, float aAZ,
                 float aOX, float aOY, float aOZ);
    /// Move the end effector to a given position, ignoring orientation
    void SetPosition(float aX, float aY, float aZ);
    /// Move the end effector along a vector of given length,
    /// maintaining current orientation
    void VectorMove(float aX, float aY, float aZ, float aLength);

    /// Accessor method for getting the limb's data
    player_limb_data_t GetData(void) const;
    /// Same again for getting the limb's geometry
    player_limb_geom_req_t GetGeom(void) const;
};



/**
The @p LocalizeProxy class is used to control a @ref
interface_localize device, which can provide multiple pose
hypotheses for a robot.
*/
class LocalizeProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_localize_t *mDevice;

  public:

    /// constructor
    LocalizeProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~LocalizeProxy();

    /// Map dimensions (cells)
    // @todo should these be in a player_pose_t?
    uint GetMapSizeX() const { return GetVar(mDevice->map_size_x); };
    uint GetMapSizeY() const { return GetVar(mDevice->map_size_y); };

    // @todo should these be in a player_pose_t?
    uint GetMapTileX() const { return GetVar(mDevice->map_tile_x); };
    uint GetMapTileY() const { return GetVar(mDevice->map_tile_y); };

    /// Map scale (m/cell)
    double GetMapScale() const { return GetVar(mDevice->map_scale); };

    // Map data (empty = -1, unknown = 0, occupied = +1)
    // is this still needed?  if so,
    //void GetMapCells(uint8_t* aCells) const
    //{
    //  return GetVarByRef(mDevice->map_cells,
    //                     mDevice->image+GetVar(mDevice->??map_cell_cout??),
    //                     aCells);
    //};

    /// Number of pending (unprocessed) sensor readings
    uint GetPendingCount() const { return GetVar(mDevice->pending_count); };

    /// Number of possible poses
    uint GetHypothCount() const { return GetVar(mDevice->hypoth_count); };

    /// Array of possible poses.
    player_localize_hypoth_t GetHypoth(uint aIndex) const
      { return GetVar(mDevice->hypoths[aIndex]); };

    /// Get the particle set
    int GetParticles()
      { return playerc_localize_get_particles(mDevice); }

    /// Get the Pose of a particle
    player_pose_t GetParticlePose(int index) const;

    /// Set the current pose hypothesis (m, m, radians).
    void SetPose(double pose[3], double cov[3]);

    /// Get the number of localization hypoths.
    uint GetNumHypoths() const { return GetVar(mDevice->hypoth_count); };

    /// Get the number of particles (for particle filter-based localization
    /// systems).  Returns the number of particles.
    uint GetNumParticles() const { return GetVar(mDevice->num_particles); };
};


/**
The @p LogProxy proxy provides access to a @ref interface_log device.
*/
class LogProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_log_t *mDevice;

  public:
    /// Constructor
    LogProxy(PlayerClient *aPc, uint aIndex=0);

    /// Destructor
    ~LogProxy();

    /// What kind of log device is this? Either PLAYER_LOG_TYPE_READ or
    /// PLAYER_LOG_TYPE_WRITE. Call GetState() to fill it.
    int GetType() const { return GetVar(mDevice->type); };

    /// Is logging/playback enabled? Call GetState() to fill it.
    int GetState() const { return GetVar(mDevice->state); };

    /// Start/stop (1/0) writing to the log file.
    void SetWriteState(int aState);

    /// Start/stop (1/0) reading from the log file.
    void SetReadState(int aState);

    /// Rewind the log file.
    void Rewind();

    /// Set the name of the logfile to write to.
    void SetFilename(const std::string aFilename);
};

/**
The @p map proxy provides access to a @ref interface_map device.
*/
class MapProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_map_t *mDevice;

  public:
    /// constructor
    MapProxy(PlayerClient *aPc, uint aIndex=0);

    /// destructor
    ~MapProxy();

    /// Get the map and store it in the proxy
    void RequestMap();

    /// Return the index of the (x,y) item in the cell array
    int GetCellIndex(int x, int y) const
    { return y*GetWidth() + x; };

    /// Get the (x,y) cell
    unsigned char GetCell(int x, int y) const
    { return GetVar(mDevice->cells[GetCellIndex(x,y)]); };

    /// Map resolution, m/cell
    double GetResolution() const { return GetVar(mDevice->resolution); };

    /// Map size, in cells
    ///    @todo should this be returned as a player_size_t?
    uint GetWidth() const { return GetVar(mDevice->width); };
    /// Map size, in cells
    /// @todo should this be returned as a player_size_t?
    uint GetHeight() const { return GetVar(mDevice->height); };

    double GetOriginX() const { return GetVar(mDevice->origin[0]); };
    double GetOriginY() const { return GetVar(mDevice->origin[1]); };

    /// Occupancy for each cell (empty = -1, unknown = 0, occupied = +1)
    void GetMap(int8_t* aMap) const
    {
      return GetVarByRef(reinterpret_cast<int8_t*>(mDevice->cells),
                         reinterpret_cast<int8_t*>(mDevice->cells+GetWidth()*GetHeight()),
                         aMap);
    };
};

/**
The @p OpaqueProxy proxy provides an interface to a generic @ref
interface_opaque. See examples/plugins/opaquedriver for an example of using
this interface in combination with a custom plugin.
*/
class OpaqueProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_opaque_t *mDevice;

  public:

    /// constructor
    OpaqueProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~OpaqueProxy();

    /// How long is the data?
    uint GetCount() const { return GetVar(mDevice->data_count); };

    /// Opaque data
    void GetData(uint8_t* aDest) const
      {
        return GetVarByRef(mDevice->data,
                           mDevice->data+GetVar(mDevice->data_count),
                           aDest);
      };

    /// Send a command
    void SendCmd(player_opaque_data_t* aData);

};

/**
The @p PlannerProxy proxy provides an interface to a 2D motion @ref
interface_planner. */
class PlannerProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_planner_t *mDevice;

  public:

    /// constructor
    PlannerProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~PlannerProxy();

    /// Set the goal pose (gx, gy, ga)
    void SetGoalPose(double aGx, double aGy, double aGa);

    /// Get the list of waypoints. Writes the result into the proxy
    /// rather than returning it to the caller.
    void RequestWaypoints();

    /// Enable/disable the robot's motion.  Set state to true to enable,
    /// false to disable.
    void SetEnable(bool aEnable);

    /// Did the planner find a valid path?
    uint GetPathValid() const { return GetVar(mDevice->path_valid); };

    /// Have we arrived at the goal?
    uint GetPathDone() const { return GetVar(mDevice->path_done); };

    /// @brief Current pose (m)
    /// @deprecated use GetPose() instead
    double GetPx() const { return GetVar(mDevice->px); };
    /// @brief Current pose (m)
    /// @deprecated use GetPose() instead
    double GetPy() const { return GetVar(mDevice->py); };
    /// @brief Current pose (radians)
    /// @deprecated use GetPose() instead
    double GetPa() const { return GetVar(mDevice->pa); };

    /// Get the current pose
    player_pose_t GetPose() const
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->px;
      p.py = mDevice->py;
      p.pa = mDevice->pa;
      return(p);
    }

    /// @brief Goal location (m)
    /// @deprecated use GetGoal() instead
    double GetGx() const { return GetVar(mDevice->gx); };
    /// @brief Goal location (m)
    /// @deprecated use GetGoal() instead
    double GetGy() const { return GetVar(mDevice->gy); };
    /// @brief Goal location (radians)
    /// @deprecated use GetGoal() instead
    double GetGa() const { return GetVar(mDevice->ga); };

    /// Get the goal
    player_pose_t GetGoal() const
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->gx;
      p.py = mDevice->gy;
      p.pa = mDevice->ga;
      return(p);
    }

    /// @brief Current waypoint location (m)
    /// @deprecated use GetCurWaypoint() instead
    double GetWx() const { return GetVar(mDevice->wx); };
    /// @brief Current waypoint location (m)
    /// @deprecated use GetCurWaypoint() instead
    double GetWy() const { return GetVar(mDevice->wy); };
    /// @brief Current waypoint location (rad)
    /// @deprecated use GetCurWaypoint() instead
    double GetWa() const { return GetVar(mDevice->wa); };

    /// Get the current waypoint
    player_pose_t GetCurrentWaypoint() const
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->wx;
      p.py = mDevice->wy;
      p.pa = mDevice->wa;
      return(p);
    }

    /// @brief Grab a particular waypoint location (m)
    /// @deprecated use GetWaypoint() instead
    double GetIx(int i) const;
    /// @brief Grab a particular waypoint location (m)
    /// @deprecated use GetWaypoint() instead
    double GetIy(int i) const;
    /// @brief Grab a particular waypoint location (rad)
    /// @deprecated use GetWaypoint() instead
    double GetIa(int i) const;

    /// Get the waypoint
    player_pose_t GetWaypoint(uint aIndex) const
    {
      assert(aIndex < GetWaypointCount());
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->waypoints[aIndex][0];
      p.py = mDevice->waypoints[aIndex][1];
      p.pa = mDevice->waypoints[aIndex][2];
      return(p);
    }

    /// Current waypoint index (handy if you already have the list
    /// of waypoints). May be negative if there's no plan, or if
    /// the plan is done
    int GetCurrentWaypointId() const
      { return GetVar(mDevice->curr_waypoint); };

    /// Number of waypoints in the plan
    uint GetWaypointCount() const
      { return GetVar(mDevice->waypoint_count); };

    /// Waypoint access operator
    /// This operator provides an alternate way of access the waypoint data.
    /// For example, given a @p PlannerProxy named @p pp, the following
    /// expressions are equivalent: @p pp.GetWaypoint(0) and @p pp[0].
    player_pose_t operator [](uint aIndex) const
      { return GetWaypoint(aIndex); }

};

/**
The @p Position1dProxy class is used to control a @ref
interface_position1d device.  The latest position data is contained
in the attributes pos, vel , etc.  */
class Position1dProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_position1d_t *mDevice;

  public:

    /// constructor
    Position1dProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~Position1dProxy();

    /// Send a motor command for velocity control mode.
    /// Specify the forward, sideways, and angular speeds in m/sec, m/sec,
    /// and radians/sec, respectively.
    void SetSpeed(double aVel);

    /// Send a motor command for position control mode.  Specify the
    /// desired pose of the robot in [m] or [rad]
    /// desired motion in [m/s] or [rad/s]
    void GoTo(double aPos, double aVel);

    /// Get the device's geometry; it is read into the
    /// relevant class attributes.
    void RequestGeom();

    /// Accessor for the pose (fill it in by calling RequestGeom)
    player_pose_t GetPose() const
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->pose[0];
      p.py = mDevice->pose[1];
      p.pa = mDevice->pose[2];
      return(p);
    }

    /// Accessor for the size (fill it in by calling RequestGeom)
    player_bbox_t GetSize() const
    {
      player_bbox_t b;
      scoped_lock_t lock(mPc->mMutex);
      b.sl = mDevice->size[0];
      b.sw = mDevice->size[1];
      return(b);
    }

    /// Enable/disable the motors.
    /// Set @p state to 0 to disable or 1 to enable.
    /// Be VERY careful with this method!  Your robot is likely to run across the
    /// room with the charger still attached.
    void SetMotorEnable(bool enable);

    /// Sets the odometry to the pose @p aPos.
    /// @note aPos is in either [m] or [rad] depending on the actuator type
    void SetOdometry(double aPos);

    /// Reset odometry to 0.
    void ResetOdometry() { SetOdometry(0); };

    /// Set PID terms
    //void SetSpeedPID(double kp, double ki, double kd);

    /// Set PID terms
    //void SetPositionPID(double kp, double ki, double kd);

    /// Set speed ramping profile
    /// spd rad/s, acc rad/s/s
    //void SetPositionSpeedProfile(double spd, double acc);

    /// Accessor method
    double  GetPos() const { return GetVar(mDevice->pos); };

    /// Accessor method
    double  GetVel() const { return GetVar(mDevice->vel); };

    /// Accessor method
    bool GetStall() const { return GetVar(mDevice->stall); };

    /// Accessor method
    uint8_t GetStatus() const { return GetVar(mDevice->status); };

    /// Accessor method
    bool IsLimitMin() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_LIMIT_MIN)) > 0; };

    /// Accessor method
    bool IsLimitCen() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_LIMIT_CEN)) > 0; };

    /// Accessor method
    bool IsLimitMax() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_LIMIT_MAX)) > 0; };

    /// Accessor method
    bool IsOverCurrent() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_OC)) > 0; };

    /// Accessor method
    bool IsTrajComplete() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_TRAJ_COMPLETE)) > 0; };

    /// Accessor method
    bool IsEnabled() const
      { return (GetVar(mDevice->status) &
               (1 << PLAYER_POSITION1D_STATUS_ENABLED)) > 0; };

};

/**
The @p Position2dProxy class is used to control a @ref
interface_position2d device.  The latest position data is contained
in the attributes xpos, ypos, etc.  */
class Position2dProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_position2d_t *mDevice;

  public:

    /// constructor
    Position2dProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~Position2dProxy();

    /// Send a motor command for velocity control mode.
    /// Specify the forward, sideways, and angular speeds in m/sec, m/sec,
    /// and radians/sec, respectively.
    void SetSpeed(double aXSpeed, double aYSpeed, double aYawSpeed);

    /// Same as the previous SetSpeed(), but doesn't take the yspeed speed
    /// (so use this one for non-holonomic robots).
    void SetSpeed(double aXSpeed, double aYawSpeed)
        { return SetSpeed(aXSpeed, 0, aYawSpeed);}

    /// Overloaded SetSpeed that takes player_pose_t as an argument
    void SetSpeed(player_pose_t vel)
        { return SetSpeed(vel.px, vel.py, vel.pa);}

    /// Send a motor command for position control mode.  Specify the
    /// desired pose of the robot as a player_pose_t.
    /// desired motion speed  as a player_pose_t.
    void GoTo(player_pose_t pos, player_pose_t vel);

    /// Same as the previous GoTo(), but doesn't take speed
    void GoTo(player_pose_t pos)
      {GoTo(pos,(player_pose_t) {0,0,0}); }

    /// Same as the previous GoTo(), but only takes position arguments,
    /// no motion speed setting
    void GoTo(double aX, double aY, double aYaw)
      {GoTo((player_pose_t) {aX,aY,aYaw},(player_pose_t) {0,0,0}); }

    /// Sets command for carlike robot
    void SetCarlike(double aXSpeed, double aDriveAngle);

    /// Get the device's geometry; it is read into the
    /// relevant class attributes.
    void RequestGeom();

    /// Accessor for the pose (fill it in by calling RequestGeom)
    player_pose_t GetPose()
    {
      player_pose_t p;
      scoped_lock_t lock(mPc->mMutex);
      p.px = mDevice->pose[0];
      p.py = mDevice->pose[1];
      p.pa = mDevice->pose[2];
      return(p);
    }

    /// Accessor for the size (fill it in by calling RequestGeom)
    player_bbox_t GetSize()
    {
      player_bbox_t b;
      scoped_lock_t lock(mPc->mMutex);
      b.sl = mDevice->size[0];
      b.sw = mDevice->size[1];
      return(b);
    }

    /// Enable/disable the motors.
    /// Set @p state to 0 to disable or 1 to enable.
    /// Be VERY careful with this method!  Your robot is likely to run across the
    /// room with the charger still attached.
    void SetMotorEnable(bool enable);

    // Select velocity control mode.
    //
    // For the the p2os_position driver, set @p mode to 0 for direct wheel
    // velocity control (default), or 1 for separate translational and
    // rotational control.
    //
    // For the reb_position driver: 0 is direct velocity control, 1 is for
    // velocity-based heading PD controller (uses DoDesiredHeading()).
    //void SelectVelocityControl(unsigned char mode);

    /// Reset odometry to (0,0,0).
    void ResetOdometry();

    /// Select position mode
    /// Set @p mode for 0 for velocity mode, 1 for position mode.
    //void SelectPositionMode(unsigned char mode);

    /// Sets the odometry to the pose @p (x, y, yaw).
    /// Note that @p x and @p y are in m and @p yaw is in radians.
    void SetOdometry(double aX, double aY, double aYaw);

    /// Set PID terms
    //void SetSpeedPID(double kp, double ki, double kd);

    /// Set PID terms
    //void SetPositionPID(double kp, double ki, double kd);

    /// Set speed ramping profile
    /// spd rad/s, acc rad/s/s
    //void SetPositionSpeedProfile(double spd, double acc);

    //
    // void DoStraightLine(double m);

    //
    //void DoRotation(double yawspeed);

    //
    //void DoDesiredHeading(double yaw, double xspeed, double yawspeed);

    //
    //void SetStatus(uint8_t cmd, uint16_t value);

    //
    //void PlatformShutdown();

    /// Accessor method
    double  GetXPos() const { return GetVar(mDevice->px); };

    /// Accessor method
    double  GetYPos() const { return GetVar(mDevice->py); };

    /// Accessor method
    double GetYaw() const { return GetVar(mDevice->pa); };

    /// Accessor method
    double  GetXSpeed() const { return GetVar(mDevice->vx); };

    /// Accessor method
    double  GetYSpeed() const { return GetVar(mDevice->vy); };

    /// Accessor method
    double  GetYawSpeed() const { return GetVar(mDevice->va); };

    /// Accessor method
    bool GetStall() const { return GetVar(mDevice->stall); };

};

/**

The @p Position3dProxy class is used to control
a interface_position3d device.  The latest position data is
contained in the attributes xpos, ypos, etc.
*/
class Position3dProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_position3d_t *mDevice;

  public:

    /// constructor
    Position3dProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~Position3dProxy();

    /// Send a motor command for a planar robot.
    /// Specify the forward, sideways, and angular speeds in m/s, m/s, m/s,
    /// rad/s, rad/s, and rad/s, respectively.
    void SetSpeed(double aXSpeed, double aYSpeed, double aZSpeed,
                  double aRollSpeed, double aPitchSpeed, double aYawSpeed);

    /// Send a motor command for a planar robot.
    /// Specify the forward, sideways, and angular speeds in m/s, m/s,
    /// and rad/s, respectively.
    void SetSpeed(double aXSpeed, double aYSpeed,
                  double aZSpeed, double aYawSpeed)
      { SetSpeed(aXSpeed,aYSpeed,aZSpeed,0,0,aYawSpeed); }

    /// simplified version of SetSpeed
    void SetSpeed(double aXSpeed, double aYSpeed, double aYawSpeed)
      { SetSpeed(aXSpeed, aYSpeed, 0, 0, 0, aYawSpeed); }

    /// Same as the previous SetSpeed(), but doesn't take the sideways speed
    /// (so use this one for non-holonomic robots).
    void SetSpeed(double aXSpeed, double aYawSpeed)
      { SetSpeed(aXSpeed,0,0,0,0,aYawSpeed);}

    /// Overloaded SetSpeed that takes player_pose3d_t as input
    void SetSpeed(player_pose3d_t vel)
      { SetSpeed(vel.px,vel.py,vel.pz,vel.proll,vel.ppitch,vel.pyaw);}


    /// Send a motor command for position control mode.  Specify the
    /// desired pose of the robot as a player_pose3d_t structure
    /// desired motion speed as a player_pose3d_t structure
    void GoTo(player_pose3d_t aPos, player_pose3d_t aVel);

    /// Same as the previous GoTo(), but does'n take vel argument
    void GoTo(player_pose3d_t aPos)
      { GoTo(aPos, (player_pose3d_t) {0,0,0,0,0,0}); }


    /// Same as the previous GoTo(), but only takes position arguments,
    /// no motion speed setting
    void GoTo(double aX, double aY, double aZ,
              double aRoll, double aPitch, double aYaw)
      { GoTo((player_pose3d_t) {aX,aY,aZ,aRoll,aPitch,aYaw},
              (player_pose3d_t) {0,0,0,0,0,0});
      }

    /// Enable/disable the motors.
    /// Set @p state to 0 to disable or 1 to enable.
    /// @attention Be VERY careful with this method!  Your robot is likely
    /// to run across the room with the charger still attached.
    void SetMotorEnable(bool aEnable);

    /// Select velocity control mode.
    /// This is driver dependent.
    void SelectVelocityControl(int aMode);

    /// Reset odometry to (0,0,0,0,0,0).
    void ResetOdometry();

    /// Sets the odometry to the pose @p (x, y, z, roll, pitch, yaw).
    /// Note that @p x, @p y, and @p z are in m and @p roll,
    /// @p pitch, and @p yaw are in radians.
    void SetOdometry(double aX, double aY, double aZ,
                     double aRoll, double aPitch, double aYaw);

    // Select position mode
    // Set @p mode for 0 for velocity mode, 1 for position mode.
    //void SelectPositionMode(unsigned char mode);

    //
    //void SetSpeedPID(double kp, double ki, double kd);

    //
    //void SetPositionPID(double kp, double ki, double kd);

    // Sets the ramp profile for position based control
    // spd rad/s, acc rad/s/s
    //void SetPositionSpeedProfile(double spd, double acc);

    /// Accessor method
    double  GetXPos() const { return GetVar(mDevice->pos_x); };

    /// Accessor method
    double  GetYPos() const { return GetVar(mDevice->pos_y); };

    /// Accessor method
    double  GetZPos() const { return GetVar(mDevice->pos_z); };

    /// Accessor method
    double  GetRoll() const { return GetVar(mDevice->pos_roll); };

    /// Accessor method
    double  GetPitch() const { return GetVar(mDevice->pos_pitch); };

    /// Accessor method
    double  GetYaw() const { return GetVar(mDevice->pos_yaw); };

    /// Accessor method
    double  GetXSpeed() const { return GetVar(mDevice->vel_x); };

    /// Accessor method
    double  GetYSpeed() const { return GetVar(mDevice->vel_y); };

    /// Accessor method
    double  GetZSpeed() const { return GetVar(mDevice->vel_z); };

    /// Accessor method
    double  GetRollSpeed() const { return GetVar(mDevice->vel_roll); };

    /// Accessor method
    double  GetPitchSpeed() const { return GetVar(mDevice->vel_pitch); };

    /// Accessor method
    double  GetYawSpeed() const { return GetVar(mDevice->vel_yaw); };

    /// Accessor method
    bool GetStall () const { return GetVar(mDevice->stall); };
};
/**
The @p PowerProxy class controls a @ref interface_power device. */
class PowerProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_power_t *mDevice;

  public:
    /// constructor
    PowerProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~PowerProxy();

    /// Returns the current charge.
    double GetCharge() const { return GetVar(mDevice->charge); };

};

/**

The @p PtzProxy class is used to control a @ref interface_ptz
device.  The state of the camera can be read from the pan, tilt, zoom
attributes and changed using the SetCam() method.
*/
class PtzProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_ptz_t *mDevice;

  public:
    /// constructor
    PtzProxy(PlayerClient *aPc, uint aIndex=0);
    // destructor
    ~PtzProxy();

  public:

    /// Change the camera state.
    /// Specify the new @p pan, @p tilt, and @p zoom values
    /// (all degrees).
    void SetCam(double aPan, double aTilt, double aZoom);

    /// Specify new target velocities
    void SetSpeed(double aPanSpeed=0, double aTiltSpeed=0, double aZoomSpeed=0);

    /// Select new control mode.  Use either @ref PLAYER_PTZ_POSITION_CONTROL
    /// or @ref PLAYER_PTZ_VELOCITY_CONTROL.
    void SelectControlMode(uint aMode);

    /// Return Pan (rad)
    double GetPan() const { return GetVar(mDevice->pan); };
    /// Return Tilt (rad)
    double GetTilt() const { return GetVar(mDevice->tilt); };
    /// Return Zoom
    double GetZoom() const { return GetVar(mDevice->zoom); };

};

/**
The @p RFIDProxy class is used to control a  @ref interface_rfid device. */
class RFIDProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_rfid_t *mDevice;

  public:
    /// constructor
    RFIDProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~RFIDProxy();

    /// returns the number of RFID tags
    uint GetTagsCount() const { return GetVar(mDevice->tags_count); };
    /// returns a RFID tag
    playerc_rfidtag_t GetRFIDTag(uint aIndex) const
      { return GetVar(mDevice->tags[aIndex]);};

    /// RFID data access operator.
    ///    This operator provides an alternate way of access the actuator data.
    ///    For example, given a @p RFIDProxy named @p rp, the following
    ///    expressions are equivalent: @p rp.GetRFIDTag[0] and @p rp[0].
    playerc_rfidtag_t operator [](uint aIndex) const
      { return(GetRFIDTag(aIndex)); }
};

/**
The @p SimulationProxy proxy provides access to a
@ref interface_simulation device.
*/
class SimulationProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_simulation_t *mDevice;

  public:
    /// constructor
    SimulationProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~SimulationProxy();

    /// set the 2D pose of an object in the simulator, identified by the
    /// std::string. Returns 0 on success, else a non-zero error code.
    void SetPose2d(char* identifier, double x, double y, double a);

    /// get the pose of an object in the simulator, identified by the
    /// std::string Returns 0 on success, else a non-zero error code.
    void GetPose2d(char* identifier, double& x, double& y, double& a);
};


/**
The @p SonarProxy class is used to control a @ref interface_sonar
device.  The most recent sonar range measuremts can be read from the
range attribute, or using the the [] operator.
*/
class SonarProxy : public ClientProxy
{
  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_sonar_t *mDevice;

  public:
    /// constructor
    SonarProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~SonarProxy();

    /// return the sensor count
    uint GetCount() const { return GetVar(mDevice->scan_count); };

    /// return a particular scan value
    double GetScan(uint aIndex) const
      { return GetVar(mDevice->scan[aIndex]); };
    /// This operator provides an alternate way of access the scan data.
    /// For example, SonarProxy[0] == SonarProxy.GetRange(0)
    double operator [] (uint aIndex) const { return GetScan(aIndex); }

    /// Number of valid sonar poses
    uint GetPoseCount() const { return GetVar(mDevice->pose_count); };

    /// Sonar poses (m,m,radians)
    player_pose_t GetPose(uint aIndex) const
      { return GetVar(mDevice->poses[aIndex]); };

    // Enable/disable the sonars.
    // Set @p state to 1 to enable, 0 to disable.
    // Note that when sonars are disabled the client will still receive sonar
    // data, but the ranges will always be the last value read from the sonars
    // before they were disabled.
    //void SetEnable(bool aEnable);

    /// Request the sonar geometry.
    void RequestGeom();
};

// /**
// The @p SoundProxy class is used to control a @ref interface_sound
// device, which allows you to play pre-recorded sound files on a robot.
// */
// class SoundProxy : public ClientProxy
// {
//
//   private:
//
//     void Subscribe(uint aIndex);
//     void Unsubscribe();
//
//     // libplayerc data structure
//     playerc_sound_t *mDevice;
//
//   public:
//     // Constructor
//     SoundProxy(PlayerClient *aPc, uint aIndex=0);
//
//     ~SoundProxy();
//
//     /** Play the sound indicated by the index. */
//     void Play(int aIndex);
// };

/**
The @p SpeechProxy class is used to control a @ref interface_speech
device.  Use the say method to send things to say.
*/
class SpeechProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_speech_t *mDevice;

  public:
    /// constructor
    SpeechProxy(PlayerClient *aPc, uint aIndex=0);
    /// constructor
    ~SpeechProxy();

    /// Send a phrase to say.
    /// The phrase is an ASCII std::string.
    void Say(std::string aStr);
};

/**
 The @p SpeechRecognition proxy provides access to a @ref interface_speech_recognition device.
 */
class SpeechRecognitionProxy : public ClientProxy
{
   void Subscribe(uint aIndex);
   void Unsubscribe();

   ///libplayerc data structure
   playerc_speechrecognition_t *mDevice;
  public:
   ///Constructor
   SpeechRecognitionProxy(PlayerClient *aPc, uint aIndex=0);
   ~SpeechRecognitionProxy();
   /// Accessor method for getting speech recognition data i.e. words.
   std::string GetWord(uint aWord) const{ 
     scoped_lock_t lock(mPc->mMutex);
     return std::string(mDevice->words[aWord]); 
   }

   /// Gets the number of words.
   uint GetCount(void) const { return GetVar(mDevice->wordCount); }

   /// Word access operator.
   ///    This operator provides an alternate way of access the speech recognition data.
   std::string operator [](uint aWord) { return(GetWord(aWord)); }
};

// /**
// The @p WaveformProxy class is used to read raw digital waveforms from
// a @ref interface_waveform device.  */
// class WaveformProxy : public ClientProxy
// {
//
//   private:
//
//     void Subscribe(uint aIndex);
//     void Unsubscribe();
//
//     // libplayerc data structure
//     playerc_waveform_t *mDevice;
//
//   public:
//     /// Constructor
//     WaveformProxy(PlayerClient *aPc, uint aIndex=0);
//
//     /// Destructor
//     ~WaveformProxy();
//
//     /// How many samples?
//     uint GetCount() const { return GetVar(mDevice->data_count); };
//
//     /// sample rate in bits per second
//     uint GetBitRate() const { return GetVar(mDevice->rate); };
//
//     /// sample depth in bits
//     uint GetDepth() const { return GetVar(mDevice->depth); };
//
//     /// the data is buffered here for playback
//     void GetImage(uint8_t* aBuffer) const
//     {
//       return GetVarByRef(mDevice->data,
//                          mDevice->data+GetCount(),
//                          aBuffer);
//     };
//
//     // dsp file descriptor
//     //uint GetFd() const { return GetVar(mDevice->fd); };
//
//     /// set up the DSP to the current bitrate and depth
//     void ConfigureDSP(uint aBitRate, uint aDepth);
//
//     /// open the sound device
//     void OpenDSPforWrite();
//
//     /// Play the waveform through the DSP
//     void Play();
// };

/**
The @p WiFiProxy class controls a @ref interface_wifi device.  */
class WiFiProxy: public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_wifi_t *mDevice;

  public:
    /// constructor
    WiFiProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~WiFiProxy();

//     int GetLinkQuality(char/// ip = NULL);
//     int GetLevel(char/// ip = NULL);
//     int GetLeveldBm(char/// ip = NULL) { return GetLevel(ip) - 0x100; }
//     int GetNoise(char/// ip = NULL);
//     int GetNoisedBm(char/// ip = NULL) { return GetNoise(ip) - 0x100; }
//
//     uint16_t GetMaxLinkQuality() { return maxqual; }
//     uint8_t GetMode() { return op_mode; }
//
//     int GetBitrate();
//
//     char/// GetMAC(char *buf, int len);
//
//     char/// GetIP(char *buf, int len);
//     char/// GetAP(char *buf, int len);
//
//     int AddSpyHost(char *address);
//     int RemoveSpyHost(char *address);
//
//   private:
//     int GetLinkIndex(char *ip);
//
//     /// The current wifi data.
//     int link_count;
//     player_wifi_link_t links[PLAYER_WIFI_MAX_LINKS];
//     uint32_t throughput;
//     uint8_t op_mode;
//     int32_t bitrate;
//     uint16_t qual_type, maxqual, maxlevel, maxnoise;
//
//     char access_point[32];
};

/**
The @p WSNProxy class is used to control a @ref interface_wsn device. */
class WSNProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_wsn_t *mDevice;

  public:
    /// constructor
    WSNProxy(PlayerClient *aPc, uint aIndex=0);
    /// destructor
    ~WSNProxy();

    uint GetNodeType    () const { return GetVar(mDevice->node_type);      };
    uint GetNodeID      () const { return GetVar(mDevice->node_id);        };
    uint GetNodeParentID() const { return GetVar(mDevice->node_parent_id); };

    player_wsn_node_data_t
       GetNodeDataPacket() const { return GetVar(mDevice->data_packet);    };

    void SetDevState(int nodeID, int groupID, int devNr, int value);
    void Power(int nodeID, int groupID, int value);
    void DataType(int value);
    void DataFreq(int nodeID, int groupID, float frequency);
};

/** @} (proxies)*/
}

namespace std
{
  std::ostream& operator << (std::ostream& os, const player_point_2d_t& c);
  std::ostream& operator << (std::ostream& os, const player_pose_t& c);
  std::ostream& operator << (std::ostream& os, const player_pose3d_t& c);
  std::ostream& operator << (std::ostream& os, const player_bbox_t& c);
  std::ostream& operator << (std::ostream& os, const player_segment_t& c);
  std::ostream& operator << (std::ostream& os, const playerc_device_info_t& c);

  std::ostream& operator << (std::ostream& os, const PlayerCc::ClientProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::ActArrayProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::AioProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::AudioDspProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::AudioMixerProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::BlinkenLightProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::BlobfinderProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::BumperProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::CameraProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::DioProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::EnergyProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::FiducialProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::VisionserverProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::GpsProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::GripperProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::IrProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::LaserProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::LimbProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::LocalizeProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::LogProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::MapProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::OpaqueProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::PlannerProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::Position1dProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::Position2dProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::Position3dProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::PowerProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::PtzProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::SimulationProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::SonarProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::SpeechProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::SpeechRecognitionProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::WafeformProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::WiFiProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::RFIDProxy& c);
  std::ostream& operator << (std::ostream& os, const PlayerCc::WSNProxy& c);
  //std::ostream& operator << (std::ostream& os, const PlayerCc::TruthProxy& c);
}

#endif

