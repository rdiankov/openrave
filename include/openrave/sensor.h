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
/** \file sensor.h
    \brief Sensor and sensing related defintions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_SENSOR_H
#define OPENRAVE_SENSOR_H

namespace OpenRAVE {

/** \brief <b>[interface]</b> A sensor measures physical properties from the environment. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_sensor.
   \ingroup interfaces
 */
class OPENRAVE_API SensorBase : public InterfaceBase
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
        ST_Tactile=7,
        ST_Actuator=8,
        ST_NumberofSensorTypes=8
    };

    typedef geometry::RaveCameraIntrinsics<dReal> CameraIntrinsics;

    /// used to pass sensor data around
    class OPENRAVE_API SensorData
    {
public:
        virtual ~SensorData() {
        }
        virtual SensorType GetType() = 0;

        /// Serialize the sensor data to stream in XML format
        virtual bool serialize(std::ostream& O) const;

        uint64_t __stamp;         ///< time stamp of the sensor data in microseconds. If 0, then the data is uninitialized! (floating-point precision is bad here). This can be either simulation or real time depending on the sensor.
        Transform __trans;             ///< the coordinate system the sensor was when the measurement was taken, this is taken directly from SensorBase::GetTransform
    };
    typedef boost::shared_ptr<SensorBase::SensorData> SensorDataPtr;
    typedef boost::shared_ptr<SensorBase::SensorData const> SensorDataConstPtr;

    class OPENRAVE_API LaserSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_Laser;
        }

        /** \brief World coordinates of the origins of each of the photon (laser) rays.

           Each of the photons start from some 3D position and go a particular direction. For most common 2D lasers and the kinect, the starting point from each of the photons is the same, it is also called the focal point.
           If positions is empty, assume the origin is t.trans for all rays.
         */
        std::vector<RaveVector<dReal> > positions;
        std::vector<RaveVector<dReal> > ranges;         ///< Range and direction readings in the form of direction*distance. The direction is in world coordinates. The values should be returned in the order laser detected them in.
        std::vector<dReal> intensity;         ///< Intensity readings.

        virtual bool serialize(std::ostream& O) const;
    };
    class OPENRAVE_API CameraSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_Camera;
        }
        std::vector<uint8_t> vimagedata;         ///< rgb image data, if camera only outputs in grayscale, fill each channel with the same value
        virtual bool serialize(std::ostream& O) const;
    };

    /// \brief Stores joint angles and EE position.
    class OPENRAVE_API JointEncoderSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_JointEncoder;
        }
        std::vector<dReal> encoderValues;         ///< measured joint angles in radians
        std::vector<dReal> encoderVelocity;         ///< measured joint velocity in radians
    };

    /// \brief Stores force data
    class OPENRAVE_API Force6DSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_Force6D;
        }
        Vector force;         ///< Force in X Y Z, in newtons
        Vector torque;         ///< Torque in X Y Z, in newtonmeters
    };

    /// \brief Stores IMU data
    class OPENRAVE_API IMUSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_IMU;
        }
        Vector rotation;         ///< quaternion
        Vector angular_velocity;
        Vector linear_acceleration;
        boost::array<dReal,9> rotation_covariance;         ///< Row major about x, y, z axes
        boost::array<dReal,9> angular_velocity_covariance;         ///< Row major about x, y, z axes
        boost::array<dReal,9> linear_acceleration_covariance;         ///< Row major x, y z axes
    };

    /// \brief odometry data storing full 6D pose and velocity
    class OPENRAVE_API OdometrySensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_Odometry;
        }
        Transform pose;         ///< measured pose
        Vector linear_velocity, angular_velocity;         ///< measured velocity
        boost::array<dReal,36> pose_covariance;         ///< Row major of 6x6 matrix about linear x, y, z axes
        boost::array<dReal,36> velocity_covariance;         ///< Row major of 6x6 matrix about rotational x, y, z axes
    };

    /// \brief tactle data
    class OPENRAVE_API TactileSensorData : public SensorData
    {
public:
        virtual SensorType GetType() {
            return ST_Tactile;
        }
        std::vector<Vector> forces;         /// xyz force of each individual element
        boost::array<dReal,9> force_covariance;         ///< row major 3x3 matrix of the uncertainty on the xyz force measurements
    };

    /// \brief An actuator for modeling motors and other mechanisms that produce torque/force. The actuator has only one degree of freedom.
    class OPENRAVE_API ActuatorSensorData : public SensorData
    {
public:
        /// \brief the state of the actuator
        enum ActuatorState {
            AS_Undefined=0,         ///< returned when no state is defined
            AS_Idle=1,          ///< this actuator is idle
            AS_Moving=2,         ///< this actuator is in motion from previous commands
            AS_Stalled=3,         ///< the actuator is stalled, needs to be unstalled by sending a ready signal
            AS_Braked=4,         ///< the actuator is braked
        };

        ActuatorSensorData() : state(AS_Undefined), measuredcurrent(0), measuredtemperature(0), appliedcurrent(0) {
        }
        virtual SensorType GetType() {
            return ST_Actuator;
        }

        ActuatorState state;
        dReal measuredcurrent;         ///< measured current from the actuator
        dReal measuredtemperature;         ///< measured temperature from the actuator
        dReal appliedcurrent;         ///< current sent to the actuator
    };

    /// permanent properties of the sensors
    class OPENRAVE_API SensorGeometry : public Readable
    {
public:
        SensorGeometry(const std::string& xmlid) : Readable(xmlid) {
        }
        virtual ~SensorGeometry() {
        }
        virtual SensorType GetType() const = 0;

        bool SerializeXML(BaseXMLWriterPtr writer, int options=0) const override;
        bool SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0) const override;
        bool DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale=1.0) override;

        virtual SensorGeometry& operator=(const SensorGeometry& r) {
            hardware_id = r.hardware_id;
            return *this;
        }

        bool operator==(const Readable& r) override {
            const SensorGeometry* pOther = dynamic_cast<const SensorGeometry*>(&r);
            if (!pOther) {
                return false;
            }
            return hardware_id == pOther->hardware_id;
        }

        std::string hardware_id; ///< optional hardware identifier of the sensor
    };
    typedef boost::shared_ptr<SensorBase::SensorGeometry> SensorGeometryPtr;
    typedef boost::shared_ptr<SensorBase::SensorGeometry const> SensorGeometryConstPtr;

    class OPENRAVE_API LaserGeomData : public SensorGeometry
    {
public:
        LaserGeomData() : SensorGeometry("Laser"), min_range(0), max_range(0), time_increment(0), time_scan(0) {
            min_angle[0] = min_angle[1] = max_angle[0] = max_angle[1] = resolution[0] = resolution[1] = 0;
        }
        virtual SensorType GetType() const {
            return ST_Laser;
        }

        bool operator==(const Readable& other) override {
            const LaserGeomData* pOther = dynamic_cast<const LaserGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other)
                   && min_angle == pOther->min_angle
                   && max_angle == pOther->max_angle
                   && resolution == pOther->resolution
                   && min_range == pOther->min_range
                   && max_range == pOther->max_range
                   && time_increment == pOther->time_increment
                   && time_scan == pOther->time_scan;
        }

        boost::array<dReal,2> min_angle;         ///< Start for the laser scan [rad].
        boost::array<dReal,2> max_angle;         ///< End angles for the laser scan [rad].
        boost::array<dReal,2> resolution;         ///< Angular resolutions for each axis of rotation [rad].
        dReal min_range, max_range;         ///< Maximum range [m].
        dReal time_increment;         ///< time between individual measurements [seconds]
        dReal time_scan;         ///< time between scans [seconds]
    };

    typedef boost::shared_ptr<LaserGeomData> LaserGeomDataPtr;
    typedef boost::shared_ptr<LaserGeomData const> LaserGeomDataConstPtr;

    class OPENRAVE_API CameraGeomData : public SensorGeometry
    {
public:
        CameraGeomData() : SensorGeometry("Camera"), width(0), height(0), measurement_time(1), gain(1), KK(intrinsics) {
        }
        virtual SensorType GetType() const {
            return ST_Camera;
        }

        // need this because of the deprecated KK
        virtual CameraGeomData& operator=(const CameraGeomData& r) {
            SensorGeometry::operator=(r);
            intrinsics = r.intrinsics;
            width = r.width;
            height = r.height;
            sensor_reference = r.sensor_reference;
            target_region = r.target_region;
            measurement_time = r.measurement_time;
            gain = r.gain;
            return *this;
        }

        bool operator==(const Readable& other) override {
            const CameraGeomData* pOther = dynamic_cast<const CameraGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other)
                   && intrinsics == pOther->intrinsics
                   && width == pOther->width
                   && height == pOther->height
                   && sensor_reference == pOther->sensor_reference
                   && target_region == pOther->target_region
                   && measurement_time == pOther->measurement_time
                   && gain == pOther->gain;
        }

        bool SerializeXML(BaseXMLWriterPtr writer, int options=0) const override;
        bool SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale=1.0, int options=0) const override;
        bool DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale=1.0) override;

        std::string sensor_reference; ///< name of sensor that whose data is referenced. This sensor transforms the data in a particular way.
        std::string target_region; ///< name of the kinbody that describes the region of interest for the camera.
        CameraIntrinsics intrinsics;         ///< intrinsic matrix
        int width, height;         ///< width and height of image
        dReal measurement_time; ///< specifies time used to take one image (also known as exposure).
        dReal gain; ///< camera gain setting
        CameraIntrinsics& KK;         ///< \deprecated (14/01/15)
    };
    typedef boost::shared_ptr<CameraGeomData> CameraGeomDataPtr;
    typedef boost::shared_ptr<CameraGeomData const> CameraGeomDataConstPtr;

    class OPENRAVE_API JointEncoderGeomData : public SensorGeometry
    {
public:
        JointEncoderGeomData() : SensorGeometry("JointEncoder"), resolution(0) {
        }
        virtual SensorType GetType() const {
            return ST_JointEncoder;
        }

        bool operator==(const Readable& other) override {
            const JointEncoderGeomData* pOther = dynamic_cast<const JointEncoderGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other) && resolution == pOther->resolution;
        }

        std::vector<dReal> resolution;         ///< the delta value of one encoder tick
    };
    class OPENRAVE_API Force6DGeomData : public SensorGeometry
    {
public:
        Force6DGeomData() : SensorGeometry("Force6D") {
        }
        virtual SensorType GetType() const {
            return ST_Force6D;
        }
        bool operator==(const Readable& other) override {
            const Force6DGeomData* pOther = dynamic_cast<const Force6DGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other);
        }
    };
    class OPENRAVE_API IMUGeomData : public SensorGeometry
    {
public:
        IMUGeomData() : SensorGeometry("IMU") {
        }
        virtual SensorType GetType() const {
            return ST_IMU;
        }

        bool operator==(const Readable& other) override {
            const IMUGeomData* pOther = dynamic_cast<const IMUGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other) && time_measurement == pOther->time_measurement;
        }

        dReal time_measurement;         ///< time between measurements
    };
    class OPENRAVE_API OdometryGeomData : public SensorGeometry
    {
public:
        OdometryGeomData() : SensorGeometry("Odometry") {
        }
        virtual SensorType GetType() const {
            return ST_Odometry;
        }

        bool operator==(const Readable& other) override {
            const OdometryGeomData* pOther = dynamic_cast<const OdometryGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other)
                   && targetid == pOther->targetid;
        }

        std::string targetid;         ///< id of the target whose odometry/pose messages are being published for
    };

    class OPENRAVE_API TactileGeomData : public SensorGeometry
    {
public:
        TactileGeomData() : SensorGeometry("Tactile") {
        }
        virtual SensorType GetType() const {
            return ST_Tactile;
        }

        bool operator==(const Readable& other) override {
            const TactileGeomData* pOther = dynamic_cast<const TactileGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other)
                   && positions == pOther->positions
                   && thickness == pOther->thickness;
            // && _mapfriction == pOther->_mapfriction;
        }

        /// LuGre friction model?
        class Friction
        {
public:
            dReal sigma_0;         ///< the stiffness coefficient of the contacting surfaces
            dReal sigma_1;         ///< the friction damping coefficient.
            dReal mu_s;         ///< static friction coefficient
            dReal mu_d;         ///< dynamic friction coefficient

            virtual bool operator==(const Friction& other) {
                return sigma_0 == other.sigma_0
                       && sigma_1 == other.sigma_1
                       && mu_s == other.mu_s
                       && mu_d == other.mu_d;
            }
        };
        std::vector<Vector> positions;         ///< 3D positions of all the elements in the sensor frame
        dReal thickness;         ///< the thickness of the tactile sensors (used for determining contact and computing force)
        ///dReal normal_force_stiffness, normal_force_damping; ///< simple model for determining contact force from depressed distance... necessary?
        std::map<std::string, Friction> _mapfriction;         ///< friction coefficients references by target objects
    };

    class OPENRAVE_API ActuatorGeomData : public SensorGeometry
    {
public:
        ActuatorGeomData() : SensorGeometry("Actuator") {
        }
        virtual SensorType GetType() const {
            return ST_Actuator;
        }

        bool operator==(const Readable& other) override {
            const ActuatorGeomData* pOther = dynamic_cast<const ActuatorGeomData*>(&other);
            if (!pOther) {
                return false;
            }
            return SensorGeometry::operator==(other)
                   && maxtorque == pOther->maxtorque
                   && maxcurrent == pOther->maxcurrent
                   && nominalcurrent == pOther->nominalcurrent
                   && maxvelocity == pOther->maxvelocity
                   && maxacceleration == pOther->maxacceleration
                   && maxjerk == pOther->maxjerk
                   && staticfriction == pOther->staticfriction
                   && viscousfriction == pOther->viscousfriction;
        }

        dReal maxtorque;         ///< Maximum possible torque actuator can apply (on output side). This includes the actuator's rotor, if one exists.
        dReal maxcurrent;         ///< Maximum permissible current of the actuator. If this current value is exceeded for a prolonged period of time, then an error could occur (due to heat, etc).
        dReal nominalcurrent;          ///< Rated current of the actuator.
        dReal maxvelocity;         ///< Maximum permissible velocity of the system (on output side).
        dReal maxacceleration;         ///< Maximum permissible acceleration of the system (on output side).
        dReal maxjerk;         ///< Maximum permissible jerking of the system (on output side). The jerk results from a sudden change in acceleration.
        dReal staticfriction;         ///< minimum torque that must be applied for actuator to overcome static friction
        dReal viscousfriction;         ///< friction depending on the velocity of the actuator
    };

    SensorBase(EnvironmentBasePtr penv) : InterfaceBase(PT_Sensor, penv) {
    }
    virtual ~SensorBase() {
    }

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_Sensor;
    }

    /// \brief A set of commands used for run-time sensor configuration.
    enum ConfigureCommand
    {
        CC_PowerOn=0x10,     ///< turns the sensor on, starts gathering data and using processor cycles. If the power is already on, servers as a reset. (off by default)
        CC_PowerOff=0x11,     ///< turns the sensor off, stops gathering data (off by default).
        CC_PowerCheck=0x12,     ///< returns whether power is on
        CC_RenderDataOn=0x20,     ///< turns on any rendering of the sensor data (off by default)
        CC_RenderDataOff=0x21,     ///< turns off any rendering of the sensor data (off by default)
        CC_RenderDataCheck=0x23,     ///< returns whether data rendering is on
        CC_RenderGeometryOn=0x30,     ///< turns on any rendering of the sensor geometry (on by default)
        CC_RenderGeometryOff=0x31,     ///< turns off any rendering of the sensor geometry (on by default)
        CC_RenderGeometryCheck=0x32,     ///< returns whether geometry rendering is on
    };

    /// \brief Configures properties of the sensor like power.
    ///
    /// \param type \ref ConfigureCommand
    /// \param blocking If set to true, makes sure the configuration ends before this function returns.(might cause problems if environment is locked).
    /// \throw openrave_exception if command doesn't succeed
    virtual int Configure(ConfigureCommand command, bool blocking=false) = 0;

    /// \brief Simulate one step forward for sensors.
    ///
    /// Only valid if this sensor is simulation based. A sensor hooked up to a real device can ignore this call
    virtual bool SimulationStep(dReal fTimeElapsed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Returns the sensor geometry. This method is thread safe.
    ///
    /// \param type the requested sensor type to create. A sensor can support many types. If type is ST_Invalid, then returns any structure that represents the geometry.
    /// \return sensor geometry
    virtual SensorGeometryConstPtr GetSensorGeometry(SensorType type=ST_Invalid) = 0;

    /// \brief Sets a new geometry for the sensor
    virtual void SetSensorGeometry(SensorGeometryConstPtr pgeometry) OPENRAVE_DUMMY_IMPLEMENTATION;

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

    /// \brief the position of the sensor in the world coordinate system
    virtual const Transform& GetTransform() = 0;

    /// \brief Register a callback whenever new sensor data comes in.
    ///
    /// \param type the sensor type to register for
    /// \param callback the user function to call, note that this might block the thread generating/receiving sensor data
    virtual UserDataPtr RegisterDataCallback(SensorType type, const boost::function<void(SensorDataConstPtr)>& callback) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \return the name of the sensor
    virtual const std::string& GetName() const {
        return _name;
    }
    virtual void SetName(const std::string& newname) {
        _name = newname;
    }

    /// \brief serialize the sensor geometry and other attributes.
    virtual void Serialize(BaseXMLWriterPtr writer, int options=0) const;

protected:
    std::string _name;     ///< name of the sensor

private:
    virtual const char* GetHash() const {
        return OPENRAVE_SENSOR_HASH;
    }
};

} // end namespace OpenRAVE

#endif
