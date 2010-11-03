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
/** \file sensor.h
    \brief Sensor and sensing related defintions.
*/
#ifndef OPENRAVE_SENSOR_H
#define OPENRAVE_SENSOR_H

namespace OpenRAVE {

/** \brief <b>[interface]</b> A sensor measures physical properties from the environment. See \ref arch_sensor.     
  \ingroup interfaces
*/
class RAVE_API SensorBase : public InterfaceBase
{
public:
    enum SensorType
    {
        ST_Invalid=0,
        ST_Laser=1,
        ST_Camera=2,
        ST_JointEncoder=3,
        ST_Force6D=4,
        ST_IMU=5,
        ST_Odometry=6,
        ST_NumberofSensorTypes=6
    };

    class CameraIntrinsics
    {
    public:
        CameraIntrinsics() : fx(0),fy(0),cx(0),cy(0) {}
        CameraIntrinsics(dReal fx, dReal fy, dReal cx, dReal cy) : fx(fx), fy(fy), cx(cx), cy(cy) {}
        dReal fx,fy, cx,cy;
    };

    /// used to pass sensor data around
    class RAVE_API SensorData
    {
    public:
        virtual ~SensorData() {}
        virtual SensorType GetType() = 0;

        /// Serialize the sensor data to stream in XML format
        virtual bool serialize(std::ostream& O) const;

        uint64_t __stamp; ///< time stamp of the sensor data in microseconds (floating-point precision is bad here). This can be either simulation or real time depending on the sensor.
    };
    typedef boost::shared_ptr<SensorData> SensorDataPtr;
    typedef boost::shared_ptr<SensorData const> SensorDataConstPtr;

    class RAVE_API LaserSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Laser; }

        Transform t;     ///< the coordinate system all the measurements are in
        std::vector<RaveVector<dReal> > positions; ///< world coordinates of the origins of each of the laser points.
                                       ///< if positions is empty, assume the origin is t.trans for all points
        std::vector<RaveVector<dReal> > ranges; ///< Range and direction readings, should be returned in the order laser detected them in.
        std::vector<dReal> intensity; ///< Intensity readings.

        virtual bool serialize(std::ostream& O) const;
    };
    class RAVE_API CameraSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Camera; }
        Transform t;     ///< the coordinate system all the measurements were taken in
        std::vector<uint8_t> vimagedata; ///< rgb image data, if camera only outputs in grayscale, fill each channel with the same value
        virtual bool serialize(std::ostream& O) const;
    };

    /// \brief Stores joint angles and EE position.
    class RAVE_API JointEncoderSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_JointEncoder; }
        std::vector<dReal> encoderValues; ///< measured joint angles in radians
        std::vector<dReal> encoderVelocity; ///< measured joint velocity in radians
    };
    
    /// \brief Stores force data
    class RAVE_API Force6DSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Force6D; }
        Vector force; ///< Force in X Y Z, in newtons
        Vector torque; ///< Torque in X Y Z, in newtonmeters
    };

    /// \brief Stores IMU data
    class RAVE_API IMUSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_IMU; }
        Vector rotation; ///< quaternion
        Vector angular_velocity;
        Vector linear_acceleration;
        boost::array<dReal,9> rotation_covariance; ///< Row major about x, y, z axes
        boost::array<dReal,9> angular_velocity_covariance; ///< Row major about x, y, z axes
        boost::array<dReal,9> linear_acceleration_covariance; ///< Row major x, y z axes
    };

    /// \brief odometry data storing full 6D pose and velocity
    class RAVE_API OdometrySensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Odometry; }
        Transform pose; ///< measured pose
        Vector linear_velocity, angular_velocity; ///< measured velocity
        boost::array<dReal,36> pose_covariance; ///< Row major of 6x6 matrix about linear x, y, z axes
        boost::array<dReal,36> velocity_covariance; ///< Row major of 6x6 matrix about rotational x, y, z axes
    };

    /// permanent properties of the sensors
    class RAVE_API SensorGeometry
    {
    public:
        virtual ~SensorGeometry() {}
        virtual SensorType GetType() = 0;
    };
    typedef boost::shared_ptr<SensorGeometry> SensorGeometryPtr;
    typedef boost::shared_ptr<SensorGeometry const> SensorGeometryConstPtr;

    class RAVE_API LaserGeomData : public SensorGeometry
    {
    public:
    LaserGeomData() : min_range(0), max_range(0), time_increment(0), time_scan(0) { min_angle[0] = min_angle[1] = max_angle[0] = max_angle[1] = resolution[0] = resolution[1] = 0; }
        virtual SensorType GetType() { return ST_Laser; }
        boost::array<dReal,2> min_angle; ///< Start for the laser scan [rad].
        boost::array<dReal,2> max_angle; ///< End angles for the laser scan [rad].
        boost::array<dReal,2> resolution; ///< Angular resolutions for each axis of rotation [rad].
        dReal min_range, max_range; ///< Maximum range [m].
        dReal time_increment; ///< time between individual measurements [seconds]
        dReal time_scan; ///< time between scans [seconds]
    };
    class RAVE_API CameraGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_Camera; }
        CameraIntrinsics KK; ///< intrinsic matrix
        int width, height; ///< width and height of image
    };
    class RAVE_API JointEncoderGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_JointEncoder; }
    };
    class RAVE_API Force6DGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_Force6D; }
    };
    class RAVE_API IMUGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_IMU; }
        dReal time_measurement; ///< time between measurements
    };
    class RAVE_API OdometryGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_Odometry; }
        std::string targetid; ///< id of the target whose odometry/pose messages are being published for
    };

    SensorBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Sensor, penv) {}
    virtual ~SensorBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_Sensor; }
    
    /// Initializes the sensor.
    /// \param cmd extra arguments that the sensor
    /// \return true on successful initialization
    virtual bool Init(const std::string& cmd) = 0;

    /// Resets any state associated with the sensor
    virtual void Reset(int options) = 0;

    /// \brief Simulate one step forward for sensors.
    ///
    /// Only valid if this sensor is simulation based. A sensor hooked up to a real device can ignore this call
    virtual bool SimulationStep(dReal fTimeElapsed) = 0;

    /// \brief Returns the sensor geometry. This method is thread safe.
    ///
    /// \param type the requested sensor type to create. A sensor can support many types. If type is ST_Invalid, then returns a data structure
    /// \return sensor geometry pointer, use delete to destroy it
    virtual SensorGeometryPtr GetSensorGeometry(SensorType type=ST_Invalid) = 0;

    /// \brief Creates the sensor data to be specifically used by this class
    /// \param type the requested sensor type to create. A sensor can support many types. If type is ST_Invalid, then returns a data structure
    /// of the type most representative of this sensor.
    /// \return new SensorData class
    virtual SensorDataPtr CreateSensorData(SensorType type=ST_Invalid) = 0;

    /// \brief Copy the most recent published data of the sensor given the type.
    ///
    /// Once GetSensorData returns, the caller has full unrestricted access to the data. This method is thread safe.
    /// \param psensordata A pointer to SensorData returned from CreateSensorData, the plugin will use
    /// psensordata->GetType() in order to return the correctly supported type.
    virtual bool GetSensorData(SensorDataPtr psensordata) = 0;

    /// \brief returns true if sensor supports a particular sensor type
    virtual bool Supports(SensorType type) = 0;

    /// \brief Set the transform of a sensor (global coordinate system).
    ///
    /// Sensors attached to the robot have their transforms automatically set every time the robot is moved
    /// \param trans - The transform defining the frame of the sensor.
    virtual void SetTransform(const Transform& trans) = 0;
    virtual Transform GetTransform() = 0;

    /// \brief Register a callback whenever new sensor data comes in.
    /// \param type the sensor type to register for
    /// \param callback the user function to call, note that this might block the thread generating/receiving sensor data
    virtual boost::shared_ptr<void> RegisterDataCallback(SensorType type, const boost::function<void(SensorDataConstPtr)>& callback) { throw openrave_exception("SensorBase::RegisterDataCallback",ORE_NotImplemented); }
	
    /// \return the name of the sensor
    virtual const std::string& GetName() const { return _name; }
    virtual void SetName(const std::string& newname) { _name = newname; }

protected:
    std::string _name; ///< name of the sensor

private:
    virtual const char* GetHash() const { return OPENRAVE_SENSOR_HASH; }
};

} // end namespace OpenRAVE

#endif
