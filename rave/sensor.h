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
#ifndef OPENRAVE_SENSOR_H
#define OPENRAVE_SENSOR_H

namespace OpenRAVE {

/// A sensor measures physical properties from the environment 
/// and converts them to data. Each sensor is associated with a 
/// particular position in space, has a geometry with 
/// properties defining the type of sensor, and can be
/// queried for sensor data.
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
    };

    struct CameraIntrinsics
    {
        CameraIntrinsics() : fx(0),fy(0),cx(0),cy(0) {}
        CameraIntrinsics(float fx, float fy, float cx, float cy) : fx(fx), fy(fy), cx(cx), cy(cy) {}
        float fx,fy, cx,cy;
    };

    /// used to pass sensor data around
    class RAVE_API SensorData
    {
    public:
        virtual ~SensorData() {}
        virtual SensorType GetType() = 0;

        /// Serialize the sensor data to stream in XML format
        virtual bool serialize(std::ostream& O) const { return true; }
    };
    typedef boost::shared_ptr<SensorData> SensorDataPtr;
    typedef boost::shared_ptr<SensorData const> SensorDataConstPtr;

    class RAVE_API LaserSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Laser; }

        Transform t;     ///< the coordinate system all the measurements are in
        std::vector<RaveVector<float> > positions; ///< world coordinates of the origins of each of the laser points.
                                       ///< if positions is empty, assume the origin is t.trans for all points
        std::vector<RaveVector<float> > ranges; ///< Range and direction readings, should be returned in the order laser detected them in.
        std::vector<float> intensity; ///< Intensity readings.
        int id; ///< A unique, increasing, ID for the scan

        virtual bool serialize(std::ostream& O) const;
    };
    class RAVE_API CameraSensorData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Camera; }
        
        Transform t;     ///< the coordinate system all the measurements were taken in
        std::vector<uint8_t> vimagedata; ///< rgb image data, if camera only outputs in grayscale, fill each channel with the same value
        
        int id; ///< A unique, increasing, ID for the image

        virtual bool serialize(std::ostream& O) const;
    };
    /// Class for storing joint angles and EE position
    class RAVE_API JointSensorData : public SensorData
    {
    public:
        public:
        virtual SensorType GetType() { return ST_JointEncoder; }
		/// Encoder values, i.e. joint angles in radians
		std::vector<dReal> _encoderValues;
		/// Pose of the end-effector in relation to the robot base (in meters and radians)
		std::vector<dReal> _eePose;
    };
    
     /// Class for storing force data (JR3, etc)
    class RAVE_API ForceSensorData : public SensorData
    {
    public:
        public:
        virtual SensorType GetType() { return ST_Force6D; }
		/// Force in X Y Z, in newtons
		std::vector<dReal> _forceXYZ;
		/// Torque in X Y Z, in newtonmeters
		std::vector<dReal> _torqueXYZ;
    };
    
	/// Class for hardware failure data (singularity, etc)
    class RAVE_API HardwareFailureData : public SensorData
    {
    public:
        virtual SensorType GetType() { return ST_Invalid; }
		/// Specifically for singularity detection
		bool _singularity;
		/// For other modes of hardware failure
		std::string _failure;
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
        virtual SensorType GetType() { return ST_Laser; }
        
        float min_angle[2]; ///< Start for the laser scan [rad].
        float max_angle[2]; ///< End angles for the laser scan [rad].
        float resolution[2]; ///< Angular resolutions for each axis of rotation [rad].
        float max_range; ///< Maximum range [m].
    };
    class RAVE_API CameraGeomData : public SensorGeometry
    {
    public:
        virtual SensorType GetType() { return ST_Camera; }
        CameraIntrinsics KK; ///< intrinsic matrix
        int width, height; ///< width and height of image
    };

    SensorBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Sensor, penv) {}
    virtual ~SensorBase() {}

    /// Initializes the sensor
    /// \param args extra arguments that the sensor takes, can be NULL
    /// \return true on successful initialization
    virtual bool Init(const std::string& cmd) = 0;

    /// Resets any state associated with the sensor
    virtual void Reset(int options) = 0;

    /// Simulate one step forward for sensors, only valid if this sensor is simulation based
    /// A sensor hooked up to a real device can ignore this call
    virtual bool SimulationStep(dReal fTimeElapsed) = 0;

    /// Returns the sensor geometry. This method is thread safe.
    /// \return sensor geometry pointer, use delete to destroy it
    virtual SensorGeometryPtr GetSensorGeometry() = 0;

    /// Creates the sensor data struct to be specifically used by this class
    /// \return new SensorData class, destroy with delete
    virtual SensorDataPtr CreateSensorData() = 0;

    /// Copy the most recent published data of the sensor. Once GetSensorData returns, the
    /// caller has full access to the data. This method is thread safe.
    /// \param psensordata A pointer to SensorData returned from CreateSensorData
    virtual bool GetSensorData(SensorDataPtr psensordata) = 0;

    /// Set the transform of a sensor.
    /// \param trans - The transform defining the frame of the sensor.
    virtual void SetTransform(const Transform& trans) = 0;
    virtual Transform GetTransform() = 0;
	
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
