// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file   kinbody.h
    \brief  Kinematics body related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_KINBODY_H
#define OPENRAVE_KINBODY_H

#include <unordered_map>
#include <unordered_set>

namespace OpenRAVE {

class OpenRAVEFunctionParserReal;
typedef boost::shared_ptr< OpenRAVEFunctionParserReal > OpenRAVEFunctionParserRealPtr;

/// \brief Result of UpdateFromInfo() call
enum UpdateFromInfoResult : uint8_t
{
    UFIR_NoChange = 0, ///< Nothing changed
    UFIR_Success = 1, ///< Updated successfully
    UFIR_RequireRemoveFromEnvironment = 2, ///< Failed to update, require the kinbody to be removed from environment before update can succeed
    UFIR_RequireReinitialize = 3, ///< Failed to update, require InitFromInfo() to be called before update can succeed
};

enum ExtractInfoOptions : uint8_t
{
    EIO_Everything = 0, ///< extract everything
    EIO_SkipDOFValues = 1, ///< extracts everything but dof values, this allows extraction even when body is not added to the environment
};

/// \brief The type of geometry primitive.
enum GeometryType : uint8_t
{
    GT_None = 0,
    GT_Box = 1,
    GT_Sphere = 2,
    GT_Cylinder = 3, ///< oriented towards z-axis
    GT_TriMesh = 4,
    GT_Container=5, ///< a container shaped geometry that has inner and outer extents. container opens on +Z. The origin is at the bottom of the base.
    GT_Cage=6, ///< a container shaped geometry with removable side walls. The side walls can be on any of the four sides. The origin is at the bottom of the base. The inner volume of the cage is measured from the base to the highest wall.
    GT_CalibrationBoard=7, ///< a box shaped geometry with grid of cylindrical dots of two sizes. The dots are always on the +z side of the box and are oriented towards z-axis.
    GT_Axial = 8, ///< a geometry defined by many slices along an axis, oriented towards z-axis
    GT_ConicalFrustum = 9, ///< a geometry defined by a conical frustum, oriented towards z-axis
    GT_Prism = 10, ///< Non-trimesh right prisms with arbitrary non-convex cross-section and infinite height, which represent “walls” or “safe zones”. _meshcollision.vertices[2 * i].xy describes the cross-section.
    GT_Capsule = 11, ///< Non-trimesh capsules (oriented towards z-axis), representing "robot link model" or "gripper model", which are able of degenerating to perfect spheres with the height tends to zero.
};

OPENRAVE_API const char* GetGeometryTypeString(GeometryType geometryType);

enum DynamicsConstraintsType : int8_t
{
    DC_Unknown = -1, ///< constraints type is not set.
    DC_IgnoreTorque        = 0, ///< Do no check torque limits
    DC_NominalTorque       = 1, ///< Compute and check torque limits using nominal torque
    DC_InstantaneousTorque = 2, ///< Compute and check torque limits using instantaneous torque
};

OPENRAVE_API const char* GetDynamicsConstraintsTypeString(DynamicsConstraintsType type);

enum JointControlMode : uint8_t
{
    JCM_None = 0, ///< unspecified
    JCM_RobotController = 1, ///< joint controlled by the robot controller
    JCM_IO = 2,              ///< joint controlled by I/O signals
    JCM_ExternalDevice = 3,  ///< joint controlled by an external device
};

OPENRAVE_API const char* GetJointControlModeString(JointControlMode jcm);

/// \brief holds parameters for an electric motor
///
/// all speed is in revolutions/second
class OPENRAVE_API ElectricMotorActuatorInfo : public InfoBase
{
public:
    ElectricMotorActuatorInfo() {
    };
    bool operator==(const ElectricMotorActuatorInfo& other) const {
        return model_type == other.model_type
               && assigned_power_rating == other.assigned_power_rating
               && max_speed == other.max_speed
               && no_load_speed == other.no_load_speed
               && stall_torque == other.stall_torque
               && max_instantaneous_torque == other.max_instantaneous_torque
               && nominal_speed_torque_points == other.nominal_speed_torque_points
               && max_speed_torque_points == other.max_speed_torque_points
               && nominal_torque == other.nominal_torque
               && rotor_inertia == other.rotor_inertia
               && torque_constant == other.torque_constant
               && nominal_voltage == other.nominal_voltage
               && speed_constant == other.speed_constant
               && starting_current == other.starting_current
               && terminal_resistance == other.terminal_resistance
               && gear_ratio == other.gear_ratio
               && coloumb_friction == other.coloumb_friction
               && viscous_friction == other.viscous_friction;
    }
    bool operator!=(const ElectricMotorActuatorInfo& other) const {
        return !operator==(other);
    }

    void Reset() override;
    void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;
    void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

    std::string model_type; ///< the type of actuator it is. Usually the motor model name is ok, but can include other info like gear box, etc
    //@{ from motor data sheet
    dReal assigned_power_rating = 0; ///< the nominal power the electric motor can safely produce. Units are **Mass * Distance² * Time-³**
    dReal max_speed = 0; ///< the maximum speed of the motor **Time-¹**
    dReal no_load_speed = 0; ///< specifies the speed of the motor powered by the nominal voltage when the motor provides zero torque. Units are **Time-¹**.
    dReal stall_torque = 0; ///< the maximum torque achievable by the motor at the nominal voltage. This torque is achieved at zero velocity (stall). Units are **Mass * Distance * Time-²**.
    dReal max_instantaneous_torque = 0; ///< the maximum instantenous torque achievable by the motor when voltage <= nominal voltage. Motor going between nominal_torque and max_instantaneous_torque can overheat, so should not be driven at it for a long time. Units are **Mass * Distance * Time-²**.
    std::vector<std::pair<dReal, dReal> > nominal_speed_torque_points; ///< the speed and torque achievable when the motor is powered by the nominal voltage. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the stall torque directly (ideal). Should be ordered from increasing speed.
    std::vector<std::pair<dReal, dReal> > max_speed_torque_points; ///< the speed and torque achievable when the motor is powered by the max voltage/current. Given the speed, the max torque can be computed. If not specified, the speed-torque curve will just be a line connecting the no load speed and the max_instantaneous_torque directly (ideal). Should be ordered from increasing speed.
    dReal nominal_torque = 0; ///< the maximum torque the motor can provide continuously without overheating. Units are **Mass * Distance * Time-²**.
    dReal rotor_inertia = 0; ///< the inertia of the rotating element about the axis of rotation. Units are **Mass * Distance²**.
    dReal torque_constant = 0; ///< specifies the proportion relating current to torque. Units are **Mass * Distance * Time-¹ * Charge-¹**.
    dReal nominal_voltage = 0; ///< the nominal voltage the electric motor can safely produce. Units are **Mass * Distance² * Time-² * Charge**.
    dReal speed_constant = 0; ///< the constant of proportionality relating speed to voltage. Units are **Mass-¹ * Distance-² * Time * Charge-¹**.
    dReal starting_current = 0; ///< specifies the current through the motor at zero velocity, equal to the nominal voltage divided by the terminal resistance. Also called the stall current.  Units are **Time-¹ * Charge**.
    dReal terminal_resistance = 0; ///< the resistance of the motor windings. Units are **Mass * Distance² * Time-¹ * Charge-²**.
    //@}

    //@{ depending on gear box
    dReal gear_ratio = 0; ///< specifies the ratio between the input speed of the transmission (the speed of the motor shaft) and the output speed of the transmission.
    dReal coloumb_friction = 0; ///< static coloumb friction on each joint after the gear box. Units are **Mass * Distance * Time-²**.
    dReal viscous_friction = 0; ///< viscous friction on each joint after the gear box. Units are **Mass * Distance * Time-²**.
    //@}
};

typedef boost::shared_ptr<ElectricMotorActuatorInfo> ElectricMotorActuatorInfoPtr;

class OPENRAVE_API JointControlInfo_RobotController : public InfoBase
{
public:
    JointControlInfo_RobotController() {
    };

    void Reset() override;
    void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;
    void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

    std::string controllerType; ///< the type of the controller used to control this joint
    //int robotId = -1; ///< index of the robot within that controller?
    boost::array<int16_t, 3> robotControllerAxisIndex = {-1, -1, -1}; ///< indicates which DOF in the robot controller controls which joint axis (up to the DOF of the joint). -1 if not specified/not valid.
    boost::array<dReal, 3> robotControllerAxisMult = {1.0, 1.0, 1.0}; ///< indicates the multiplier to convert the joint values from the environment units to the robot controller units, per joint axis. (valueInRobotControllerUnits - robotControllerAxisOffset) / robotControllerAxisMult = valueInEnvUnits
    boost::array<dReal, 3> robotControllerAxisOffset = {0.0, 0.0, 0.0}; ///< indicates the offset, in the robot controller units, that should be applied to the joint values, per joint axis.
    boost::array<std::string, 3> robotControllerAxisManufacturerCode = {"", "", ""}; ///< indicates the manufacturer (vendor) codes of the servo devices per joint axis, in order for the robot controller to validate communication with the servo devices.
    boost::array<std::string, 3> robotControllerAxisProductCode = {"", "", ""}; ///< indicates the product codes of the servo devices per joint axis, in order for the robot controller to validate communication with the servo devices. This is usually determined by each manufacturer. Manufacturer code and product code together makes a globally unique identification in the protocol. This is different from ElectricMotorActuatorInfo::model_type, which is the name of the motor type attached to the servo devices.
};
typedef boost::shared_ptr<JointControlInfo_RobotController> JointControlInfo_RobotControllerPtr;

class OPENRAVE_API JointControlInfo_IO : public InfoBase
{
public:
    JointControlInfo_IO() {
    };

    void Reset() override;
    void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;
    void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

    std::string deviceType; ///< type of device this joint is for
    boost::array< std::vector<std::string>, 3 > moveIONames;       ///< io names for controlling positions of this joint.
    boost::array< std::vector<std::string>, 3 > upperLimitIONames; ///< io names for detecting if the joint is at its upper limit
    boost::array< std::vector<uint8_t>, 3 > upperLimitSensorIsOn;  ///< if true, the corresponding upper limit sensor reads 1 when the joint is at its upper limit. otherwise, the upper limit sensor reads 0 when the joint is at its upper limit. the default value is 1.
    boost::array< std::vector<std::string>, 3 > lowerLimitIONames; ///< io names for detecting if the joint is at its lower limit
    boost::array< std::vector<uint8_t>, 3 > lowerLimitSensorIsOn;  ///< if true, the corresponding lower limit sensor reads 1 when the joint is at its lower limit. otherwise, the lower limit sensor reads 0 when the joint is at its upper limit. the default value is 1.
};
typedef boost::shared_ptr<JointControlInfo_IO> JointControlInfo_IOPtr;

class OPENRAVE_API JointControlInfo_ExternalDevice : public InfoBase
{
public:
    JointControlInfo_ExternalDevice() {
    };

    void Reset() override;
    void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;
    void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

    std::string externalDeviceType; ///< id for the external device
};

typedef boost::shared_ptr<JointControlInfo_ExternalDevice> JointControlInfo_ExternalDevicePtr;

///< New kinematics functions to be used by KinBody
class OPENRAVE_API KinematicsFunctions
{
public:
    virtual ~KinematicsFunctions() {
    }

    /// \brief function that sets links' transforms
    ///
    /// \param[in] pJointValues the full dof values of the robot in DOF order
    /// \param[out] Output the link transforms to this vector
    /// \return true if function completed and changed the link transforms. If returns false, the operation could not be compelted.
    virtual bool SetLinkTransforms(const dReal* pJointValues, const std::vector<Transform*>& vLinkTransformPointers) = 0;
};

typedef boost::shared_ptr<KinematicsFunctions> KinematicsFunctionsPtr;

class OPENRAVE_API KinematicsGenerator
{
public:
    virtual ~KinematicsGenerator() {
    }

    /// \brief generates kinematics functions for a body and returns the struct
    virtual KinematicsFunctionsPtr GenerateKinematicsFunctions(const KinBody& body) = 0;
};

typedef boost::shared_ptr<KinematicsGenerator> KinematicsGeneratorPtr;


/// \brief checks if link is enabled from vector of link enable state mask
/// intended to be used on return value of GetLinkEnableStatesMasks()
inline bool IsLinkStateBitEnabled(const std::vector<uint64_t>& linkEnableStateMasks, size_t linkIndex)
{
    return linkEnableStateMasks.at(linkIndex >> 6) & (1LU << (linkIndex & 0x3f));
}

/// \brief checks if link is enabled from vector of link enable state mask
/// intended to be used on return value of GetLinkEnableStatesMasks()
inline void DisableLinkStateBit(std::vector<uint64_t>& linkEnableStateMasks, size_t linkIndex)
{
    linkEnableStateMasks.at(linkIndex >> 6) &= ~(1LU << (linkIndex & 0x3f));
}

/// \brief checks if link is enabled from vector of link enable state mask
/// intended to be used on return value of GetLinkEnableStatesMasks()
inline void EnableLinkStateBit(std::vector<uint64_t>& linkEnableStateMasks, size_t linkIndex)
{
    linkEnableStateMasks.at(linkIndex >> 6) |= 1LU << (linkIndex & 0x3f);
}

/// \brief resizes linkEnableStateMasks according to numLinks and initialize to 0's (disabled)
inline void ResizeLinkStateBitMasks(std::vector<uint64_t>& linkEnableStateMasks, size_t numLinks)
{
    const size_t size = (numLinks >> 6) + 1;
    if (linkEnableStateMasks.size() < size) {
        linkEnableStateMasks.resize(size, 0);
    }
}

/// \brief resizes linkEnableStateMasks according to numLinks and initialize to 0's (disabled)
inline void InitializeLinkStateBitMasks(std::vector<uint64_t>& linkEnableStateMasks, size_t numLinks)
{
    linkEnableStateMasks.resize((numLinks >> 6) + 1);
    std::fill(linkEnableStateMasks.begin(), linkEnableStateMasks.end(), 0);
}

/// \brief sets linkEnableStateMasks to all 0's (disabled)
inline void DisableAllLinkStateBitMasks(std::vector<uint64_t>& linkEnableStateMasks)
{
    std::fill(linkEnableStateMasks.begin(), linkEnableStateMasks.end(), 0);
}

/// \brief sets linkEnableStateMasks to all 1's according to numLinks
inline void EnableAllLinkStateBitMasks(std::vector<uint64_t>& linkEnableStateMasks, size_t numLinks)
{
    if (linkEnableStateMasks.empty()) {
        return;
    }
    // enable every bit, but make sure that bits beyond numLinks is 0, otherwise KinBody::IsEnabled may return true even when every link is disabled.
    if (linkEnableStateMasks.size() > 1) {
        std::fill(linkEnableStateMasks.begin(), linkEnableStateMasks.end() - 1, UINT64_MAX);
    }

    linkEnableStateMasks.back() = (1LU << (numLinks & 0x3f)) - 1;
}

class Grabbed;
typedef boost::shared_ptr<Grabbed> GrabbedPtr;
typedef boost::shared_ptr<Grabbed const> GrabbedConstPtr;


/** \brief <b>[interface]</b> A kinematic body of links and joints. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_kinbody.
    \ingroup interfaces
 */
class OPENRAVE_API KinBody : public InterfaceBase
{
public:
    /// \brief A set of properties for the kinbody. These properties are used to describe a set of variables used in KinBody.
    enum KinBodyProperty {
        Prop_JointMimic=0x1,     ///< joint mimic equations
        Prop_JointLimits=0x2,     ///< regular limits
        Prop_JointOffset=0x4,
        Prop_JointProperties=0x8,     ///< resolution, weights
        Prop_JointAccelerationVelocityTorqueLimits=0x10,     ///< velocity + acceleration + jerk + torque
        Prop_Joints=Prop_JointMimic|Prop_JointLimits|Prop_JointOffset|Prop_JointProperties|Prop_JointAccelerationVelocityTorqueLimits, ///< all properties of all joints

        Prop_Name=0x20,     ///< name changed
        Prop_LinkDraw=0x40,     ///< toggle link geometries rendering
        Prop_LinkGeometry=0x80,     ///< the current geometry of the link changed
        Prop_LinkTransforms=0x100, ///< if any of the link transforms changed, this implies the DOF values of the robot changed
        Prop_LinkGeometryGroup=0x200, ///< the geometry informations of some geometry group changed
        Prop_LinkStatic=0x400,     ///< static property of link changed
        Prop_LinkEnable=0x800,     ///< enable property of link changed
        Prop_LinkDynamics=0x1000,     ///< mass/inertia properties of link changed
        Prop_Links=Prop_LinkDraw|Prop_LinkGeometry|Prop_LinkStatic|Prop_LinkGeometryGroup|Prop_LinkEnable|Prop_LinkDynamics,     ///< all properties of all links
        Prop_JointCustomParameters = 0x2000, ///< when Joint::SetFloatParameters(), Joint::SetIntParameters(), and Joint::SetStringParameters() are called
        Prop_LinkCustomParameters = 0x4000, ///< when Link::SetFloatParameters(), Link::SetIntParameters(), Link::SetStringParameters() are called
        Prop_BodyAttached=0x8000, ///< if attached bodies changed

        // robot only
        // 0x00010000
        Prop_RobotSensors = 0x00020000,     ///< [robot only] all properties of all sensors
        Prop_Sensors = 0x00020000,
        Prop_RobotSensorPlacement = 0x00040000,     ///< [robot only] relative sensor placement of sensors
        Prop_SensorPlacement = 0x00040000,
        Prop_RobotActiveDOFs = 0x00080000,     ///< [robot only] active dofs changed
        Prop_RobotManipulatorTool = 0x00100000, ///< [robot only] the tool coordinate system changed
        Prop_RobotManipulatorName = 0x00200000, ///< [robot only] the manipulator name
        Prop_RobotManipulatorSolver = 0x00400000,
        Prop_RobotManipulators = Prop_RobotManipulatorTool | Prop_RobotManipulatorName | Prop_RobotManipulatorSolver,     ///< [robot only] all properties of all manipulators
        Prop_RobotGrabbed = 0x01000000, ///< [robot only] if grabbed bodies changed

        Prop_BodyRemoved = 0x10000000, ///< if a KinBody is removed from the environment
    };

    class OPENRAVE_API Link; // forward decl
    class OPENRAVE_API Geometry; // forward decl

    /// \brief used for specifying the type of limit checking and the messages associated with it
    enum CheckLimitsAction
    {
        CLA_Nothing = 0, ///< no checking
        CLA_CheckLimits = 1, /// < checks and warns if the limits are overboard (default)
        CLA_CheckLimitsSilent = 2, ///< checks the limits and silently clamps the joint values (used if the code expects bad values as part of normal operation)
        CLA_CheckLimitsThrow = 3, ///< check the limits and throws if something went wrong
    };


    /// \brief Describes the properties of a geometric primitive.
    ///
    /// Contains everything associated with a geometry's appearance and shape
    class OPENRAVE_API GeometryInfo : public InfoBase
    {
public:
        GeometryInfo() {
        }
        bool operator==(const GeometryInfo& other) const {
            return _t == other._t
                   && _vGeomData == other._vGeomData
                   && _vGeomData2 == other._vGeomData2
                   && _vGeomData3 == other._vGeomData3
                   && _vGeomData4 == other._vGeomData4
                   && _vSideWalls == other._vSideWalls
                   && _vAxialSlices == other._vAxialSlices
                   && _vDiffuseColor == other._vDiffuseColor
                   && _vAmbientColor == other._vAmbientColor
                   && _meshcollision == other._meshcollision
                   && _id == other._id
                   && _name == other._name
                   && _type == other._type
                   && _filenamerender == other._filenamerender
                   && _filenamecollision == other._filenamecollision
                   && _vRenderScale == other._vRenderScale
                   && _vCollisionScale == other._vCollisionScale
                   && _fTransparency == other._fTransparency
                   && _bVisible == other._bVisible
                   && _bModifiable == other._bModifiable
                   && _calibrationBoardParameters == other._calibrationBoardParameters
                   && _vNegativeCropContainerMargins == other._vNegativeCropContainerMargins
                   && _vPositiveCropContainerMargins == other._vPositiveCropContainerMargins
                   && _vNegativeCropContainerEmptyMargins == other._vNegativeCropContainerEmptyMargins
                   && _vPositiveCropContainerEmptyMargins == other._vPositiveCropContainerEmptyMargins;
        }
        bool operator!=(const GeometryInfo& other) const {
            return !operator==(other);
        }

        void Reset() override;

        void SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;

        void DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale, int options) override;

        /// \brief compare two geometry infos. If the floating differences are within fEpsilon, then comparison will return true.
        ///
        /// \param rhs the right hand geometry info to compare
        /// \param fUnitScale rhs should be scaled by fUnitScale when being compared.
        /// \return 0 if geometries are similar within epsilon. Otherwise a non-zero code indicating what was not the same
        int Compare(const GeometryInfo& rhs, dReal fUnitScale, dReal fEpsilon) const;

        /// \brief converts the unit scale of the geometry
        void ConvertUnitScale(dReal fUnitScale);

        /// triangulates the geometry object and initializes collisionmesh. GeomTrimesh types must already be triangulated
        /// \param fTessellation to control how fine the triangles need to be. 1.0f is the default value
        bool InitCollisionMesh(float fTessellation=1);

        inline dReal GetSphereRadius() const {
            return _vGeomData.x;
        }

        inline dReal GetCylinderRadius() const {
            return _vGeomData.x;
        }
        inline dReal GetCylinderHeight() const {
            return _vGeomData.y;
        }
        inline dReal GetConicalFrustumTopRadius() const {
            return _vGeomData.x;
        }
        inline dReal GetConicalFrustumBottomRadius() const {
            return _vGeomData.y;
        }
        inline dReal GetConicalFrustumHeight() const {
            return _vGeomData.z;
        }
        inline dReal GetPrismHeight() const {
            return _vGeomData.y;
        }
        inline dReal GetCapsuleRadius() const {
            return _vGeomData.x;
        }
        inline dReal GetCapsuleHeight() const {
            return _vGeomData.y;
        }
        inline const Vector& GetBoxExtents() const { // deprecated?
            return _vGeomData;
        }
        inline void SetBoxHalfExtents(const Vector& halfExtents) {
            _vGeomData = halfExtents;
        }
        inline const Vector& GetBoxHalfExtents() const {
            return _vGeomData;
        }

        inline const Vector& GetCageBaseHalfExtents() const {
            return _vGeomData;
        }

        inline void SetCageBaseHalfExtents(const Vector& halfExtents) {
            _vGeomData = halfExtents;
        }

        /// \brief compute the inner empty volume in the geometry coordinate system
        ///
        /// \return bool true if the geometry has a concept of empty volume nad tInnerEmptyVolume/abInnerEmptyVolume are filled
        bool ComputeInnerEmptyVolume(Transform& tInnerEmptyVolume, Vector& abInnerEmptyExtents) const;

        /// \brief computes the bounding box in the world. tGeometryWorld is for the world transform.
        AABB ComputeAABB(const Transform& tGeometryWorld) const;

        uint8_t GetSideWallExists() const;

        inline const std::string& GetId() const {
            return _id;
        }
        inline const std::string& GetName() const {
            return _name;
        }

        inline const Vector& GetContainerOuterExtents() const {
            return _vGeomData;
        }
        inline const Vector& GetContainerInnerExtents() const {
            return _vGeomData2;
        }

        inline void SetContainerOuterExtents(const Vector& outerExtents) {
            _vGeomData = outerExtents;
        }
        inline void SetContainerInnerExtents(const Vector& innerExtents) {
            _vGeomData2 = innerExtents;
        }

        ///< for sphere it is radius
        ///< for cylinder, first 2 values are radius and height
        ///< for trimesh, none
        ///< for conical frustum, first 3 values are top radius, bottom radius, height
        /// for boxes, first 3 values are half extents. For containers, the first 3 values are the full outer extents.
        /// For GT_Cage, this is the base box extents with the origin being at the -Z center.
        Vector _vGeomData;

        ///< For GT_Container, the first 3 values are the full inner extents.
        ///< For GT_Cage, if any are non-zero, then force the full inner extents (bottom center) to be this much, starting at the base center top
        Vector _vGeomData2;
        Vector _vGeomData3; ///< For containers, the first 3 values is the bottom cross XY full extents and Z height from bottom face.

        ///< For containers, the first 3 values are the full extents of the bottom pad (a box attached to the container
        ///  beneath the container bottom). This geometry only valid if the first 3 values are all positive. The origin
        ///  of the container will still be at the outer bottom of the container.
        Vector _vGeomData4;

        // For GT_Cage
        enum SideWallType
        {
            SWT_NX=0,
            SWT_PX=1,
            SWT_NY=2,
            SWT_PY=3,
            SWT_First=SWT_NX,
            SWT_Last=SWT_PY,
        };

        struct SideWall
        {
            Transform transf;
            Vector vExtents;
            SideWallType type;
            int Compare(const SideWall& rhs, dReal fUnitScale=1.0, dReal fEpsilon=10e-7) const;

            bool operator==(const SideWall& other) const {
                return transf == other.transf
                       && vExtents == other.vExtents
                       && type == other.type;
            }
            bool operator!=(const SideWall& other) const {
                return !operator==(other);
            }
        };
        std::vector<SideWall> _vSideWalls; ///< used by GT_Cage

        RaveVector<float> _vDiffuseColor = Vector(1,1,1);
        RaveVector<float> _vAmbientColor; ///< hints for how to color the meshes

        /// \brief trimesh representation of the collision data of this object in this local coordinate system
        ///
        /// Should be transformed by \ref _t before rendering.
        /// For spheres and cylinders, an appropriate discretization value is chosen.
        /// If empty, will be automatically computed from the geometry's type and render data
        TriMesh _meshcollision;

        GeometryType _type = GT_None; ///< the type of geometry primitive
        std::string _id;   ///< unique id of the geometry
        std::string _name; ///< the name of the geometry

        /// \brief filename for render model (optional)
        ///
        /// Should be transformed by _t before rendering.
        /// If the value is "__norenderif__:x", then the viewer should not render the object if it supports *.x files where"x" is the file extension.
        std::string _filenamerender;

        /// \brief filename for collision data (optional)
        ///
        /// This is for record keeping only and geometry does not get automatically loaded to _meshcollision.
        /// The user should call _meshcollision = *env->ReadTrimeshURI(_filenamecollision) by themselves.
        std::string _filenamecollision;

        Vector _vRenderScale = Vector(1,1,1); ///< render scale of the object (x,y,z) from _filenamerender
        Vector _vCollisionScale = Vector(1,1,1); ///< render scale of the object (x,y,z) from _filenamecollision
        float _fTransparency = 0; ///< value from 0-1 for the transparency of the rendered object, 0 is opaque
        bool _bVisible = true; ///< if true, geometry is visible as part of the 3d model (default is true)
        bool _bModifiable = true; ///< if true, object geometry can be dynamically modified (default is true)
        Vector _vNegativeCropContainerMargins = Vector(0,0,0); ///< The negative crop margins component
        Vector _vPositiveCropContainerMargins = Vector(0,0,0); ///< The positive crop margins component
        Vector _vNegativeCropContainerEmptyMargins = Vector(0,0,0); ///< The negative crop empty margins component
        Vector _vPositiveCropContainerEmptyMargins = Vector(0,0,0); ///< The positive crop empty margins component

        struct CalibrationBoardParameters { ///< used by GT_CalibrationBoard
            CalibrationBoardParameters() : numDotsX(3), numDotsY(3), dotsDistanceX(1), dotsDistanceY(1), patternName("threeBigDotsDotGrid"), dotDiameterDistanceRatio(0.25), bigDotDiameterDistanceRatio(0.5) {
            } ///< constructor
            int numDotsX; ///< number of dots in x direction, minimum 3
            int numDotsY; ///< number of dots in y direction, minimum 3
            dReal dotsDistanceX; ///< distance between center of dots in x direction
            dReal dotsDistanceY; ///< distance between center of dots in y direction
            RaveVector<float> dotColor; ///< color of dot mesh, which can differ from the color of the board mesh
            std::string patternName; ///< string that determines pattern of big and normal dots
                                     /// currently, the only supported type is "threeBigDotsDotGrid"
                                     /// any unsupported pattern will generate a full grid of normal dots
            dReal dotDiameterDistanceRatio; ///< dot diameter divided by minimum of dot distances x and y
            dReal bigDotDiameterDistanceRatio; ///< big dot diameter divided by minimum of dot distances x and y
            int Compare(const CalibrationBoardParameters& other, dReal fEpsilon) const;
            bool operator==(const CalibrationBoardParameters& other) const {
                return numDotsX == other.numDotsX
                       && numDotsY == other.numDotsY
                       && dotsDistanceX == other.dotsDistanceX
                       && dotsDistanceY == other.dotsDistanceY
                       && dotColor == other.dotColor
                       && patternName == other.patternName
                       && dotDiameterDistanceRatio == other.dotDiameterDistanceRatio
                       && bigDotDiameterDistanceRatio == other.bigDotDiameterDistanceRatio;
            }
            bool operator!=(const CalibrationBoardParameters& other) const {
                return !operator==(other);
            }
        };
        std::vector<CalibrationBoardParameters> _calibrationBoardParameters;

        /// \brief Generates the calibration board's dot grid mesh based on calibration board settings
        void GenerateCalibrationBoardDotMesh(TriMesh& tri, float fTessellation=1) const;


        /// \brief A slice used to construct axial geometry.
        struct AxialSlice
        {
            dReal zOffset; ///< z coordinate of the axial slice
            dReal radius; ///< radius of the axial slice

            int Compare(const AxialSlice& rhs, dReal fUnitScale=1.0, dReal fEpsilon=10e-7) const;
            bool operator==(const AxialSlice& other) const {
                return zOffset == other.zOffset && radius == other.radius;
            }
            bool operator!=(const AxialSlice& other) const {
                return !operator==(other);
            }
            bool operator<(const AxialSlice& other) const {
                return zOffset < other.zOffset;
            }
        };

        /// \brief An axial geometry representation
        ///
        /// An axial might look like drawing below.
        /// The dotted lines represents the z and y axes. Each row
        /// represents an axial slice.
        ///
        /// There has to be at least two axial slices in an axial,
        /// which are called top and bottom axial slices.
        ///
        /// If the radius of the top and bottom slices are the same,
        /// the resulting shape is essentially a cylinder.
        ///
        /// The axial slices stored in this vector are not sorted by
        /// the z coordinate.
        ///
        ///         z
        ///         :
        ///      ___:___
        ///  ...|...:...|... y
        ///     |   :   |
        ///      \__:__/
        ///         :
        ///         :
        ///
        std::vector<AxialSlice> _vAxialSlices;

        enum GeometryInfoField : uint32_t
        {
            GIF_Transform = (1 << 0), // _t field
            GIF_Mesh = (1 << 1), // _meshcollision field
        };

        inline const Transform& GetTransform() const {
            return _t;
        }
        inline void SetTransform(const Transform& t) {
            _t = t;
            _modifiedFields |= GIF_Transform;
        }

        inline bool IsModifiedField(GeometryInfoField field) const {
            return !!(_modifiedFields & field);
        }
private:
        Transform _t; ///< Local transformation of the geom primitive with respect to the link's coordinate system.

        uint32_t _modifiedFields = 0xffffffff; ///< a bitmap of GeometryInfoField, for supported fields, indicating which fields are touched, otherwise they can be skipped in UpdateFromInfo. By default, assume all fields are modified.

        /// \brief adds modified fields
        inline void AddModifiedField(GeometryInfoField field) {
            _modifiedFields |= field;
        }

        friend class Geometry;
        friend class Link;
        friend class KinBody;
        friend class RobotBase;
    };

    typedef boost::shared_ptr<GeometryInfo> GeometryInfoPtr;
    typedef boost::shared_ptr<GeometryInfo const> GeometryInfoConstPtr;

    /// \brief geometry object holding a link parent and wrapping access to a protected geometry info
    class OPENRAVE_API Geometry
    {
public:
        /// \deprecated (12/07/16)
        static const GeometryType GeomNone RAVE_DEPRECATED = OpenRAVE::GT_None;
        static const GeometryType GeomBox RAVE_DEPRECATED = OpenRAVE::GT_Box;
        static const GeometryType GeomSphere RAVE_DEPRECATED = OpenRAVE::GT_Sphere;
        static const GeometryType GeomCylinder RAVE_DEPRECATED = OpenRAVE::GT_Cylinder;
        static const GeometryType GeomTrimesh RAVE_DEPRECATED = OpenRAVE::GT_TriMesh;

        Geometry(boost::shared_ptr<Link> parent, const KinBody::GeometryInfo& info);
        ~Geometry() {
        }

        /// \brief parent link that geometry belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline boost::shared_ptr<Link> GetParentLink(bool trylock=false) const {
            if( trylock ) {
                return _parent.lock();
            }
            else {
                return LinkPtr(_parent);
            }
        }

        /// \brief get local geometry transform
        inline const Transform& GetTransform() const {
            return _info._t;
        }
        inline GeometryType GetType() const {
            return _info._type;
        }

        inline const Vector& GetRenderScale() const {
            return _info._vRenderScale;
        }

        inline const std::string& GetRenderFilename() const {
            return _info._filenamerender;
        }
        inline float GetTransparency() const {
            return _info._fTransparency;
        }
        /// \deprecated (12/1/12)
        inline bool IsDraw() const RAVE_DEPRECATED {
            return _info._bVisible;
        }
        inline bool IsVisible() const {
            return _info._bVisible;
        }
        inline bool IsModifiable() const {
            return _info._bModifiable;
        }

        inline dReal GetSphereRadius() const {
            return _info.GetSphereRadius();
        }
        inline dReal GetCylinderRadius() const {
            return _info.GetCylinderRadius();
        }
        inline dReal GetCylinderHeight() const {
            return _info.GetCylinderHeight();
        }
        inline dReal GetConicalFrustumTopRadius() const {
            return _info.GetConicalFrustumTopRadius();
        }
        inline dReal GetConicalFrustumBottomRadius() const {
            return _info.GetConicalFrustumBottomRadius();
        }
        inline dReal GetConicalFrustumHeight() const {
            return _info.GetConicalFrustumHeight();
        }
        inline dReal GetPrismHeight() const {
            return _info.GetPrismHeight();
        }
        inline dReal GetCapsuleRadius() const {
            return _info.GetCapsuleRadius();
        }
        inline dReal GetCapsuleHeight() const {
            return _info.GetCapsuleHeight();
        }
        inline const Vector& GetBoxExtents() const {
            return _info.GetBoxExtents();
        }
        inline const Vector& GetContainerOuterExtents() const {
            return _info.GetContainerOuterExtents();
        }
        inline const Vector& GetContainerInnerExtents() const {
            return _info.GetContainerInnerExtents();
        }
        inline const Vector& GetContainerBottomCross() const {
            return _info._vGeomData3;
        }
        inline const Vector& GetContainerBottom() const {
            return _info._vGeomData4;
        }
        inline const RaveVector<float>& GetDiffuseColor() const {
            return _info._vDiffuseColor;
        }
        inline const RaveVector<float>& GetAmbientColor() const {
            return _info._vAmbientColor;
        }
        inline const std::string& GetId() const {
            return _info._id;
        }
        inline const std::string& GetName() const {
            return _info._name;
        }
        inline const Vector& GetNegativeCropContainerMargins() const {
            return _info._vNegativeCropContainerMargins;
        }
        inline const Vector& GetPositiveCropContainerMargins() const {
            return _info._vPositiveCropContainerMargins;
        }
        inline const Vector& GetNegativeCropContainerEmptyMargins() const {
            return _info._vNegativeCropContainerEmptyMargins;
        }
        inline const Vector& GetPositiveCropContainerEmptyMargins() const {
            return _info._vPositiveCropContainerEmptyMargins;
        }
        inline int GetNumberOfAxialSlices() const {
            return _info._vAxialSlices.size();
        }

        /// \brief returns the local collision mesh
        inline const TriMesh& GetCollisionMesh() const {
            return _info._meshcollision;
        }

        inline const KinBody::GeometryInfo& GetInfo() const {
            return _info;
        }

        inline const KinBody::GeometryInfo& UpdateAndGetInfo() {
            UpdateInfo();
            return _info;
        }

        void UpdateInfo();

        /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
        void ExtractInfo(KinBody::GeometryInfo& info) const;

        /// \brief update Geometry according to new GeometryInfo, returns false if update cannot be performed and requires InitFromInfo
        UpdateFromInfoResult UpdateFromInfo(const KinBody::GeometryInfo& info);

        /// cage
        //@{
        inline const Vector& GetCageBaseExtents() const {
            return _info._vGeomData;
        }

        /// \brief compute the inner empty volume in the parent link coordinate system
        ///
        /// \return bool true if the geometry has a concept of empty volume nad tInnerEmptyVolume/abInnerEmptyVolume are filled
        bool ComputeInnerEmptyVolume(Transform& tInnerEmptyVolume, Vector& abInnerEmptyExtents) const;
        //@}

        bool InitCollisionMesh(float fTessellation=1);

        /// \brief returns an axis aligned bounding box given that the geometry is transformed by trans
        AABB ComputeAABB(const Transform& trans) const;

        inline uint8_t GetSideWallExists() const {
            return _info.GetSideWallExists();
        }

        void serialize(std::ostream& o, int options) const;

        /// \brief sets a new collision mesh and notifies every registered callback about it
        void SetCollisionMesh(const TriMesh& mesh);
        /// \brief sets visible flag. if changed, notifies every registered callback about it.
        ///
        /// \return true if changed
        bool SetVisible(bool visible);

        /// \deprecated (12/1/12)
        inline void SetDraw(bool bDraw) RAVE_DEPRECATED {
            SetVisible(bDraw);
        }
        /// \brief set transparency level (0 is opaque)
        void SetTransparency(float f);
        /// \brief override diffuse color of geometry material
        void SetDiffuseColor(const RaveVector<float>& color);
        /// \brief override ambient color of geometry material
        void SetAmbientColor(const RaveVector<float>& color);

        /// \brief validates the contact normal on the surface of the geometry and makes sure the normal faces "outside" of the shape.
        ///
        /// \param position the position of the contact point specified in the link's coordinate system
        /// \param normal the unit normal of the contact point specified in the link's coordinate system
        /// \return true if the normal is changed to face outside of the shape
        bool ValidateContactNormal(const Vector& position, Vector& normal) const;

        /// \brief sets a new render filename for the geometry. This does not change the collision
        void SetRenderFilename(const std::string& renderfilename);

        /// \brief sets the name of the geometry
        void SetName(const std::string& name);

        /// \brief set the negative crop margin of the geometry
        void SetNegativeCropContainerMargins(const Vector& negativeCropContainerMargins);

        /// \brief set the positive crop margin of the geometry
        void SetPositiveCropContainerMargins(const Vector& positiveCropContainerMargins);

        /// \brief set the negative crop empty margin of the geometry
        void SetNegativeCropContainerEmptyMargins(const Vector& negativeCropContainerEmptyMargins);

        /// \brief set the positive crop empty margin of the geometry
        void SetPositiveCropContainerEmptyMargins(const Vector& positiveCropContainerEmptyMargins);

        /// \brief generates the dot mesh of a calibration board
        inline void GetCalibrationBoardDotMesh(TriMesh& tri) {
            _info.GenerateCalibrationBoardDotMesh(tri);
        }
        /// \brief returns the color of the calibration board's dot mesh
        inline RaveVector<float> GetCalibrationBoardDotColor() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].dotColor;
            }
            return Vector(0, 0, 0);
        }
        /// \brief returns x dimension (in dots) of the calibration board dot grid
        inline int GetCalibrationBoardNumDotsX() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].numDotsX;
            }
            return 0;
        }
        /// \brief returns y dimension (in dots) of the calibration board dot grid
        inline int GetCalibrationBoardNumDotsY() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].numDotsY;
            }
            return 0;
        }
        /// \brief returns x dot distance of the calibration board dot grid
        inline dReal GetCalibrationBoardDotsDistanceX() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].dotsDistanceX;
            }
            return 0;
        }
        /// \brief returns y dot distance of the calibration board dot grid
        inline dReal GetCalibrationBoardDotsDistanceY() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].dotsDistanceY;
            }
            return 0;
        }

        /// \brief returns pattern name of the calibration board dot grid
        inline std::string GetCalibrationBoardPatternName() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].patternName;
            }
            return std::string();
        }
        /// \brief returns x dot distance of the calibration board dot grid
        inline dReal GetCalibrationBoardDotDiameterDistanceRatio() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].dotDiameterDistanceRatio;
            }
            return 0;
        }
        /// \brief returns x dot distance of the calibration board dot grid
        inline dReal GetCalibrationBoardBigDotDiameterDistanceRatio() const {
            if (_info._calibrationBoardParameters.size() != 0) {
                return _info._calibrationBoardParameters[0].bigDotDiameterDistanceRatio;
            }
            return 0;
        }

protected:
        boost::weak_ptr<Link> _parent;
        KinBody::GeometryInfo _info; ///< geometry info
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class OpenRAVEXMLParser::LinkXMLReader;
        friend class OpenRAVEXMLParser::KinBodyXMLReader;
        friend class XFileReader;
#else
        friend class ::OpenRAVEXMLParser::LinkXMLReader;
        friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
        friend class ::XFileReader;
#endif
#endif
        friend class ColladaReader;
        friend class RobotBase;
        friend class KinBody;
        friend class KinBody::Link;
    };

    typedef boost::shared_ptr<Geometry> GeometryPtr;
    typedef boost::shared_ptr<Geometry const> GeometryConstPtr;

    /// \brief Describes the properties of a link used to initialize it
    class OPENRAVE_API LinkInfo : public InfoBase
    {
public:
        LinkInfo() {
        };
        bool operator==(const LinkInfo& other) const;
        bool operator!=(const LinkInfo& other) const {
            return !operator==(other);
        }

        void Reset() override;
        void SerializeJSON(rapidjson::Value &value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options) const override;
        void DeserializeJSON(const rapidjson::Value &value, dReal fUnitScale, int options) override;

        /// \brief compare two link infos. If the floating differences are within fEpsilon, then comparison will return true.
        ///
        /// \param linkCompareOptions 0 to compare everything. 1 to ignore _t
        /// \param fUnitScale rhs should be scaled by fUnitScale when being compared.
        /// \return 0 if links are similar within epsilon. Otherwise a non-zero code indicating what was not the same
        int Compare(const LinkInfo& rhs, int linkCompareOptions, dReal fUnitScale, dReal fEpsilon) const;

        /// \brief converts the unit scale of the link properties and geometries
        void ConvertUnitScale(dReal fUnitScale);

        inline const std::string& GetId() const {
            return _id;
        }
        inline const std::string& GetName() const {
            return _name;
        }

        std::vector<GeometryInfoPtr> _vgeometryinfos;
        /// extra-purpose geometries like
        /// self -  self-collision specific geometry. By default, this type of geometry will be always set
        std::map< std::string, std::vector<GeometryInfoPtr> > _mapExtraGeometries;

        ///\brief unique id of the link
        std::string _id;
        /// \brief unique link name
        std::string _name;

        /// the frame for inertia and center of mass of the link in the link's coordinate system
        Transform _tMassFrame;
        /// mass of link
        dReal _mass = 1e-10; ///< kg, set default to non-zero value to avoid divide by 0 for inverse dynamics/physics computations

        Vector _vinertiamoments; ///< kg*unit**2 inertia along the axes of _tMassFrame
        std::map<std::string, std::vector<dReal> > _mapFloatParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::vector<int> > _mapIntParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::string > _mapStringParameters; ///< custom key-value pairs that could not be fit in the current model
        /// force the following links to be treated as adjacent to this link
        std::vector<std::string> _vForcedAdjacentLinks; // link names. sorted.
        std::map<std::string, ReadablePtr> _mReadableInterfaces; ///< readable interface mapping
        /// \brief Indicates a static body that does not move with respect to the root link.
        ///
        //// Static should be used when an object has infinite mass and
        /// shouldn't be affected by physics (including gravity). Collision still works.
        bool _bStatic = false;

        bool _bIsEnabled = true; ///< false if the link is disabled. disabled links do not participate in collision detection

        bool _bIgnoreSelfCollision = false; ///< true if the link ignored from self collision computation. If true, this link is not considered in self collision check against any other links.

        bool __padding; // for 4-byte alignment

        enum LinkInfoField : uint32_t
        {
            LIF_Transform = 0x1, // _t field
        };

        inline const Transform& GetTransform() const {
            return _t;
        }
        inline void SetTransform(const Transform& t) {
            _t = t;
            _modifiedFields |= LIF_Transform;
        }

        inline bool IsModifiedField(LinkInfoField field) const {
            return !!(_modifiedFields & field);
        }
        /// \brief adds modified fields
        inline void AddModifiedField(LinkInfoField field) {
            _modifiedFields |= field;
        }

        inline void SetNoncollidingLink(const std::string& name) {
            std::vector<std::string>::const_iterator it = lower_bound(_vForcedAdjacentLinks.begin(),
                                                                      _vForcedAdjacentLinks.end(),
                                                                      name);
            if (it == _vForcedAdjacentLinks.end() || *it != name) {
                _vForcedAdjacentLinks.insert(it, name);
            }
        }

private:
        Transform _t; ///< the current transformation of the link with respect to the world coordinate system

        uint32_t _modifiedFields = 0xffffffff; ///< a bitmap of LinkInfoField, for supported fields, indicating which fields are touched, otherwise they can be skipped in UpdateFromInfo. By default, assume all fields are modified.

        /// \brief deserializes a readable from rReadable and stores it into _mReadableInterfaces[id]
        void _DeserializeReadableInterface(const std::string& id, const rapidjson::Value& rReadable, dReal fUnitScale);

        friend class Link;
        friend class KinBody;
        friend class RobtBase;
    };
    typedef boost::shared_ptr<LinkInfo> LinkInfoPtr;
    typedef boost::shared_ptr<LinkInfo const> LinkInfoConstPtr;

    /// \brief A rigid body holding all its collision and rendering data.
    class OPENRAVE_API Link : public boost::enable_shared_from_this<Link>, public ReadablesContainer
    {
public:
        Link(KinBodyPtr parent);         ///< pass in a ODE world
        ~Link();

        /// \deprecated (12/10/18)
        typedef KinBody::GeometryInfo GeometryInfo RAVE_DEPRECATED;
        typedef KinBody::Geometry Geometry;
        typedef KinBody::GeometryPtr GeometryPtr;
        typedef KinBody::GeometryConstPtr GeometryConstPtr;
        typedef boost::shared_ptr<KinBody::GeometryInfo> GeometryInfoPtr RAVE_DEPRECATED;
        typedef TriMesh TRIMESH RAVE_DEPRECATED;
        typedef GeometryType GeomType RAVE_DEPRECATED;
        static const GeometryType GeomNone RAVE_DEPRECATED = OpenRAVE::GT_None;
        static const GeometryType GeomBox RAVE_DEPRECATED = OpenRAVE::GT_Box;
        static const GeometryType GeomSphere RAVE_DEPRECATED = OpenRAVE::GT_Sphere;
        static const GeometryType GeomCylinder RAVE_DEPRECATED = OpenRAVE::GT_Cylinder;
        static const GeometryType GeomTrimesh RAVE_DEPRECATED = OpenRAVE::GT_TriMesh;

        inline const std::string& GetId() const {
            return _info._id;
        }
        inline const std::string& GetName() const {
            return _info._name;
        }

        /// \brief Indicates a static body that does not move with respect to the root link.
        ///
        //// Static should be used when an object has infinite mass and
        ///< shouldn't be affected by physics (including gravity). Collision still works.
        inline bool IsStatic() const {
            return _info._bStatic;
        }

        /// \brief Enables a Link. An enabled link takes part in collision detection and physics simulations
        void Enable(bool enable);

        /// \brief returns true if the link is enabled. \see Enable
        bool IsEnabled() const;

        /// \brief sets this link to be ignored in self collision checking.
        void SetIgnoreSelfCollision(bool bIgnore);

        /// \brief returns true if the link is ignored from self collision. \see IgnoreSelfCollision
        bool IsSelfCollisionIgnored() const;

        /// \brief Sets all the geometries of the link as visible or non visible.
        ///
        /// \return true if changed
        bool SetVisible(bool visible);

        /// \return true if any geometry of the link is visible.
        bool IsVisible() const;

        /// \brief parent body that link belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline KinBodyPtr GetParent(bool trylock=false) const {
            if( trylock ) {
                return _parent.lock();
            }
            else {
                return KinBodyPtr(_parent);
            }
        }

        /// \brief unique index into parent KinBody::GetLinks vector
        inline int GetIndex() const {
            return _index;
        }
        inline const TriMesh& GetCollisionData() const {
            return _collision;
        }

        /// \brief Compute the aabb of all the geometries of the link in the link coordinate system
        AABB ComputeLocalAABB() const;

        /// \brief Compute the aabb of all the geometries of the link in the world coordinate system
        AABB ComputeAABB() const;

        /// \brief returns an axis-aligned bounding box when link has transform tLink.
        ///
        /// AABB equivalent to setting link transform to tLink and calling ComptueAABB()
        /// \brief tLink the world transform to put this link in when computing the AABB
        AABB ComputeAABBFromTransform(const Transform& tLink) const;

        /// \brief Compute the aabb of all the geometries from the given groupname. If the given geomgroupname does not exist in this link, will throw an exception.
        AABB ComputeAABBForGeometryGroup(const std::string& geomgroupname) const;
        AABB ComputeLocalAABBForGeometryGroup(const std::string& geomgroupname) const;
        AABB ComputeAABBForGeometryGroupFromTransform(const std::string& geomgroupname, const Transform& tLink) const;

        /// \brief Return the current transformation of the link in the world coordinate system.
        inline const Transform& GetTransform() const {
            return _info._t;
        }

        /// \brief Return all the direct parent links in the kinematics hierarchy of this link.
        ///
        /// A parent link is is immediately connected to this link by a joint and has a path to the root joint so that it is possible
        /// to compute this link's transformation from its parent.
        /// \param[out] filled with the parent links
        void GetParentLinks(std::vector< boost::shared_ptr<Link> >& vParentLinks) const;

        /// \brief Tests if a link is a direct parent.
        ///
        /// \see GetParentLinks
        /// \param link The link to test if it is one of the parents of this link.
        bool IsParentLink(boost::shared_ptr<Link const> plink) const RAVE_DEPRECATED {
            return IsParentLink(*plink);
        }
        bool IsParentLink(const Link &link) const;

        /// \brief return center of mass offset in the link's local coordinate frame
        inline Vector GetLocalCOM() const {
            return _info._tMassFrame.trans;
        }

        /// \brief return center of mass of the link in the global coordinate system
        inline Vector GetGlobalCOM() const {
            return _info._t*_info._tMassFrame.trans;
        }

        inline Vector GetCOMOffset() const {
            return _info._tMassFrame.trans;
        }

        /// \brief return inertia in link's local coordinate frame. The translation component is the the COM in the link's frame.
        TransformMatrix GetLocalInertia() const;

        // \brief return inertia around the link's COM in the global coordinate frame.
        TransformMatrix GetGlobalInertia() const;

        /// \brief sets a new mass frame with respect to the link coordinate system
        void SetLocalMassFrame(const Transform& massframe);

        /// \brief sets new principal moments of inertia (with respect to the mass frame)
        void SetPrincipalMomentsOfInertia(const Vector& inertiamoments);

        /// \brief set a new mass
        void SetMass(dReal mass);

        /// \brief return the mass frame in the link's local coordinate system that holds the center of mass and principal axes for inertia.
        inline const Transform& GetLocalMassFrame() const {
            return _info._tMassFrame;
        }

        /// \brief return the mass frame in the global coordinate system that holds the center of mass and principal axes for inertia.
        inline Transform GetGlobalMassFrame() const {
            return _info._t*_info._tMassFrame;
        }

        /// \brief return the principal moments of inertia inside the mass frame
        inline const Vector& GetPrincipalMomentsOfInertia() const {
            return _info._vinertiamoments;
        }
        inline dReal GetMass() const {
            return _info._mass;
        }

        /// \brief sets a link to be static.
        ///
        /// Because this can affect the kinematics, it requires the body's internal structures to be recomputed
        void SetStatic(bool bStatic);

        /// \brief Sets the transform of the link regardless of kinematics
        ///
        /// \param[in] transform the new transformation
        void SetTransform(const Transform& transform);

        /// adds an external force at pos (absolute coords)
        /// \param[in] force the direction and magnitude of the force
        /// \param[in] pos in the world where the force is getting applied
        /// \param[in] add if true, force is added to previous forces, otherwise it is set
        void SetForce(const Vector& force, const Vector& pos, bool add);

        /// adds torque to a body (absolute coords)
        /// \param add if true, torque is added to previous torques, otherwise it is set
        void SetTorque(const Vector& torque, bool add);

        /// forces the velocity of the link
        /// \param[in] linearvel the translational velocity
        /// \param[in] angularvel is the rotation axis * angular speed
        void SetVelocity(const Vector& linearvel, const Vector& angularvel);

        /// \brief get the velocity of the link
        /// \param[out] linearvel the translational velocity
        /// \param[out] angularvel is the rotation axis * angular speed
        void GetVelocity(Vector& linearvel, Vector& angularvel) const;

        /// \brief return the linear/angular velocity of the link
        std::pair<Vector,Vector> GetVelocity() const;

        /// \brief returns a list of all the geometry objects.
        inline const std::vector<GeometryPtr>& GetGeometries() const {
            return _vGeometries;
        }
        GeometryPtr GetGeometry(int index) const;

        GeometryPtr GetGeometry(const string_view geomame) const;

        /// \brief inits the current geometries with the new geometry info.
        ///
        /// This gives a user control for dynamically changing the object geometry. Note that the kinbody/robot hash could change.
        /// \param geometries a list of geometry infos to be initialized into new geometry objects, note that the geometry info data is copied
        /// \param bForceRecomputeMeshCollision if true, then recompute mesh collision for all non-tri meshes
        void InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries, bool bForceRecomputeMeshCollision=true);
        void InitGeometries(std::list<KinBody::GeometryInfo>& geometries, bool bForceRecomputeMeshCollision=true);

private:
        /// \brief _vGeometries is expected to be filled with the new Geometries already. This will initialize them.
        ///
        /// This is an internal method to provide a common implementation to the public InitGeometries API.
        void _InitGeometriesInternal(bool bForceRecomputeMeshCollision);

public:
        /// \brief adds geometry info to all the current geometries and possibly stored extra group geometries
        ///
        /// Will store the geometry pointer to use for later, so do not modify after this.
        /// \param addToGroups if true, will add the same ginfo to all groups
        void AddGeometry(KinBody::GeometryInfoPtr pginfo, bool addToGroups);

        /// \brief adds geometry info to the geometry group specified by groupname
        ///
        /// Will store the geometry pointer to use for later, so do not modify after this.
        /// \param groupname the name of the geometry group to add this new geometry info to
        void AddGeometryToGroup(KinBody::GeometryInfoPtr pginfo, const std::string& groupname);

        /// \brief removes geometry that matches a name from the current geometries and possibly stored extra group geometries
        ///
        /// \param removeFromAllGroups if true, will check and remove the geometry from all stored groups
        void RemoveGeometryByName(const std::string& geometryname, bool removeFromAllGroups);

        /// \brief initializes the link with geometries from the extra geomeries in LinkInfo
        ///
        /// \param name The name of the geometry group. If name is empty, will initialize the default geometries.
        /// \throw If name does not exist in GetInfo()._mapExtraGeometries, then throw an exception.
        void SetGeometriesFromGroup(const std::string& name);

        /// \brief returns a const reference to the vector of geometries for a particular group
        ///
        /// \param name The name of the geometry group.
        /// \throw openrave_exception If the group does not exist, throws an exception.
        const std::vector<KinBody::GeometryInfoPtr>& GetGeometriesFromGroup(const std::string& name) const;

        /// \brief stores geometries for later retrieval
        ///
        /// the list is stored inside _GetInfo()._mapExtraGeometries. Note that the pointers are copied and not the data, so
        /// any be careful not to modify the geometries afterwards
        void SetGroupGeometries(const std::string& name, const std::vector<KinBody::GeometryInfoPtr>& geometries);

private:
        /// \brief stores geometries for later retrieval
        ///
        /// This call is identical to SetGroupGeometries except that it does not automatically post a Prop_LinkGeometryGroup update to the body. It will be up to the caller to ensure this occurs.
        void _SetGroupGeometriesNoPostprocess(const std::string& name, const std::vector<KinBody::GeometryInfoPtr>& geometries);

public:
        /// \brief returns the number of geometries stored from a particular key
        ///
        /// \return if -1, then the geometries are not in _mapExtraGeometries, otherwise the number
        int GetGroupNumGeometries(const std::string& name) const;

        /// \brief swaps the geometries with the link
        void SwapGeometries(boost::shared_ptr<Link>& link);

        /// validates the contact normal on link and makes sure the normal faces "outside" of the geometry shape it lies on. An exception can be thrown if position is not on a geometry surface
        /// \param position the position of the contact point specified in the link's coordinate system, assumes it is on a particular geometry
        /// \param normal the unit normal of the contact point specified in the link's coordinate system
        /// \return true if the normal is changed to face outside of the shape
        bool ValidateContactNormal(const Vector& position, Vector& normal) const;

        bool IsRigidlyAttached(boost::shared_ptr<Link const> plink) const RAVE_DEPRECATED {
            return IsRigidlyAttached(*plink);
        }

        /// \brief returns true if link is rigidily attahced to this link.
        ///
        /// \param link Assumes link shares the same parent as this link.
        bool IsRigidlyAttached(const Link &link) const;

        /// \brief returns true if linkIndex (index into the parent's _veclinks) is rigidily attahced to this link.
        bool IsRigidlyAttached(const int linkIndex) const;

        /// \brief Gets all the rigidly attached links to linkindex, also adds the link to the list.
        ///
        /// \param vattachedlinks the array to insert all links attached to linkindex with the link itself.
        void GetRigidlyAttachedLinks(std::vector<boost::shared_ptr<Link> >& vattachedlinks) const;

        void serialize(std::ostream& o, int options) const;

        /// \brief return a map of custom float parameters
        inline const std::map<std::string, std::vector<dReal> >& GetFloatParameters() const {
            return _info._mapFloatParameters;
        }

        /// \brief set custom float parameters
        ///
        /// \param parameters if empty, then removes the parameter
        void SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters);

        /// \brief return a map of custom integer parameters
        inline const std::map<std::string, std::vector<int> >& GetIntParameters() const {
            return _info._mapIntParameters;
        }

        /// \brief set custom int parameters
        ///
        /// \param parameters if empty, then removes the parameter
        void SetIntParameters(const std::string& key, const std::vector<int>& parameters);

        /// \brief return a map of custom float parameters
        inline const std::map<std::string, std::string >& GetStringParameters() const {
            return _info._mapStringParameters;
        }

        /// \brief set custom string parameters
        ///
        /// \param value if empty, then removes the parameter
        void SetStringParameters(const std::string& key, const std::string& value);

        /// \brief Updates several fields in \ref _info depending on the current state of the link
        void UpdateInfo();

        /// \brief returns the current info structure of the link.
        ///
        /// The LinkInfo::_vgeometryinfos do not reflect geometry changes that happened since the robot was created. User
        /// will need to call Geometry::GetInfo on each individual geometry.
        inline const KinBody::LinkInfo& GetInfo() const {
            return _info;
        }

        /// \brief Calls \ref UpdateInfo and returns the link structure
        inline const KinBody::LinkInfo& UpdateAndGetInfo() {
            UpdateInfo();
            return _info;
        }

        /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
        void ExtractInfo(KinBody::LinkInfo& info) const;

        /// \brief update Link according to new LinkInfo, returns false if update cannot be performed and requires InitFromInfo
        UpdateFromInfoResult UpdateFromInfo(const KinBody::LinkInfo& info);

protected:
        /// \brief enables / disables LinkInfo as well as notifies parent KinBody
        void _Enable(bool enable);

        /// \brief Updates the cached information due to changes in the collision data.
        ///
        /// \param parameterschanged if true, will
        void _Update(bool parameterschanged=true, uint32_t extraParametersChanged=0);

        std::vector<GeometryPtr> _vGeometries;         ///< \see GetGeometries

        LinkInfo _info; ///< parameter information of the link

private:
        /// Sensitive variables that are auto-generated and should not be modified by the user.
        /// @name Private Link Variables
        //@{
        int _index;                  ///< \see GetIndex
        KinBodyWeakPtr _parent;         ///< \see GetParent
        std::vector<int> _vParentLinks;         ///< \see GetParentLinks, IsParentLink
        std::vector<int> _vRigidlyAttachedLinks;         ///< \see IsRigidlyAttached, GetRigidlyAttachedLinks
        TriMesh _collision; ///< triangles for collision checking, triangles are always the triangulation
                            ///< of the body when it is at the identity transformation
        //@}
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class OpenRAVEXMLParser::LinkXMLReader;
        friend class OpenRAVEXMLParser::KinBodyXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
        friend class XFileReader;
#else
        friend class ::OpenRAVEXMLParser::LinkXMLReader;
        friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
        friend class ::XFileReader;
#endif
#endif
        friend class ColladaReader;
        friend class ColladaWriter;
        friend class KinBody;
        friend class RobotBase;
    };
    typedef boost::shared_ptr<KinBody::Link> LinkPtr;
    typedef boost::shared_ptr<KinBody::Link const> LinkConstPtr;
    typedef boost::weak_ptr<KinBody::Link> LinkWeakPtr;

    /** \brief The type of joint movement.

        Non-special joints that are combinations of revolution and prismatic joints.
        The first 4 bits specify the joint DOF, the next bits specify whether the joint is revolute (0) or prismatic (1).
        There can be also special joint types that are valid if the JointSpecialBit is set.

        For multi-dof joints, the order is transform(parentlink) * transform(axis0) * transform(axis1) ...
     */
    enum JointType {
        JointNone = 0,
        JointHinge = 0x01,
        JointRevolute = 0x01,
        JointSlider = 0x11,
        JointPrismatic = 0x11,
        JointRR = 0x02,
        JointRP = 0x12,
        JointPR = 0x22,
        JointPP = 0x32,
        JointSpecialBit = 0x80000000,
        JointUniversal = 0x80000001,
        JointHinge2 = 0x80000002,
        JointSpherical = 0x80000003,
        JointTrajectory = 0x80000004, ///< there is no axis defined, instead the relative transformation is directly output from the trajectory affine_transform structure
    };

    class Joint;

    /// \brief Holds mimic information about position, velocity, and acceleration of one axis of the joint.
    ///
    /// In every array, [0] is position, [1] is velocity, [2] is acceleration.
    class OPENRAVE_API MimicInfo : public InfoBase
    {
public:
        MimicInfo() {
        };

        void Reset() override;
        void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options=0) const override;
        void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

        boost::array< std::string, 3>  _equations;         ///< the original equations

        bool operator==(const MimicInfo& other) const {
            return _equations == other._equations;
        }
        bool operator!=(const MimicInfo& other) const {
            return !operator==(other);
        }
    };
    typedef boost::shared_ptr<MimicInfo> MimicInfoPtr;
    typedef boost::shared_ptr<MimicInfo const> MimicInfoConstPtr;

    class OPENRAVE_API Mimic
    {
public:
        boost::array< std::string, 3>  _equations;         ///< the original equations

        struct DOFFormat
        {
            int16_t jointindex = -1; ///< the index into the joints, if >= GetJoints.size(), then points to the passive joints
            int16_t dofindex = -1; ///< if >= 0, then points to a DOF of the robot that is NOT mimiced
            uint8_t axis = 0; ///< the axis of the joint index
            bool operator <(const DOFFormat& r) const;
            bool operator ==(const DOFFormat& r) const;
            boost::shared_ptr<Joint> GetJoint(KinBody &parent) const;
            boost::shared_ptr<Joint const> GetJoint(const KinBody &parent) const;
        };

        struct DOFHierarchy
        {
            int16_t dofindex;         ///< >=0 dof index
            uint16_t dofformatindex;         ///< index into _vdofformat to follow the computation
            bool operator ==(const DOFHierarchy& r) const {
                return dofindex==r.dofindex && dofformatindex == r.dofformatindex;
            }
        };

        /// @name automatically set
        //@{
        std::vector< DOFFormat > _vdofformat;         ///< the format of the values the equation takes order is important.
        std::vector<DOFHierarchy> _vmimicdofs;         ///< all dof indices that the equations depends on. DOFHierarchy::dofindex can repeat
        OpenRAVEFunctionParserRealPtr _posfn;
        std::vector<OpenRAVEFunctionParserRealPtr > _velfns, _accelfns;         ///< the velocity and acceleration partial derivatives with respect to each of the values in _vdofformat
        //@}
    };
    typedef boost::shared_ptr<Mimic> MimicPtr;
    typedef boost::shared_ptr<Mimic const> MimicConstPtr;

    /// \brief Describes the properties of a joint used to initialize it
    class OPENRAVE_API JointInfo : public InfoBase
    {
public:
        JointInfo() {
        }
        bool operator==(const JointInfo& other) const;
        bool operator!=(const JointInfo& other) const {
            return !operator==(other);
        }

        void Reset() override;
        void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options=0) const override;
        void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

        inline int GetDOF() const {
            if(_type & KinBody::JointSpecialBit) {
                switch(_type) {
                case KinBody::JointHinge2:
                case KinBody::JointUniversal: return 2;
                case KinBody::JointSpherical: return 3;
                case KinBody::JointTrajectory: return 1;
                default:
                    throw OPENRAVE_EXCEPTION_FORMAT("invalid joint type 0x%x", _type, ORE_Failed);
                }
            }

            return int(_type & 0xf);
        }

        inline const std::string& GetId() const {
            return _id;
        }
        inline const std::string& GetName() const {
            return _name;
        }

        JointType _type = JointNone; /// The joint type

        std::string _id;   // joint unique id
        std::string _name;         ///< the unique joint name
        std::string _linkname0, _linkname1; ///< attaching links, all axes and anchors are defined in the link pointed to by _linkname0 coordinate system. _linkname0 is usually the parent link.
        Vector _vanchor; ///< the anchor of the rotation axes defined in _linkname0's coordinate system. this is only used to construct the internal left/right matrices. passed into Joint::_ComputeInternalInformation
        boost::array<Vector,3> _vaxes = {Vector(0,0,1), Vector(0,0,1), Vector(0,0,1)};                ///< axes in _linkname0's or environment coordinate system used to define joint movement. passed into Joint::_ComputeInternalInformation
        std::vector<dReal> _vcurrentvalues; ///< joint values at initialization. passed into Joint::_ComputeInternalInformation

        boost::array<dReal,3> _vresolution = {0.02, 0.02, 0.02};              ///< interpolation resolution
        boost::array<dReal,3> _vmaxvel = {10, 10, 10};                  ///< the soft maximum velocity (rad/s) to move the joint when planning
        boost::array<dReal,3> _vhardmaxvel = {0, 0, 0};              ///< the hard maximum velocity, robot cannot exceed this velocity. used for verification checking. Default hard limits is 0. if 0, the user should not use the hard limit value.
        boost::array<dReal,3> _vmaxaccel = {50, 50, 50};                ///< the soft maximum acceleration (rad/s^2) of the joint
        boost::array<dReal,3> _vhardmaxaccel = {0, 0, 0};            ///< the hard maximum acceleration (rad/s^2), robot cannot exceed this acceleration. used for verification checking. Default hard limits is 0. if 0, the user should not use the hard limit value.
        boost::array<dReal,3> _vmaxjerk = {50*1000, 50*1000, 50*1000};                 ///< the soft maximum jerk (rad/s^3) of the joint
        // Set negligibly large jerk by default which can change acceleration between min and max within a typical time step.
        boost::array<dReal,3> _vhardmaxjerk = {0, 0, 0};             ///< the hard maximum jerk (rad/s^3), robot cannot exceed this jerk. used for verification checking. Default hard limits is 0. if 0, the user should not use the hard limit value.
        boost::array<dReal,3> _vmaxtorque = {0, 0, 0};               ///< maximum torque (N.m, kg m^2/s^2) that should be applied to the joint. Usually this is computed from the motor nominal torque and gear ratio. Ignore if values are 0. Set max torque to 0 to notify the system that dynamics parameters might not be valid.
        boost::array<dReal,3> _vmaxinertia = {0, 0, 0};             ///< maximum inertia (kg m^2) that the joint can exhibit. Usually this is set for safety reasons. Ignore if values are 0.
        boost::array<dReal,3> _vweights = {1, 1, 1};                ///< the weights of the joint for computing distance metrics.

        /// \brief internal offset parameter that determines the branch the angle centers on
        ///
        /// Wrap offsets are needed for rotation joints since the range is limited to 2*pi.
        /// This allows the wrap offset to be set so the joint can function in [-pi+offset,pi+offset]..
        /// \param iaxis the axis to get the offset from
        boost::array<dReal,3> _voffsets = {0, 0, 0};
        boost::array<dReal,3> _vlowerlimit = {0, 0, 0};
        boost::array<dReal,3> _vupperlimit = {0, 0, 0};         ///< joint limits
        TrajectoryBasePtr _trajfollow; ///< used if joint type is JointTrajectory

        boost::array<MimicInfoPtr,3> _vmimic;          ///< the mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free. When cloning, is it ok to copy this and assume it is constant?

        std::map<std::string, std::vector<dReal> > _mapFloatParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::vector<int> > _mapIntParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::string > _mapStringParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, ReadablePtr> _mReadableInterfaces; ///< readable interface mapping

        ElectricMotorActuatorInfoPtr _infoElectricMotor;

        /// true if joint axis has an identification at some of its lower and upper limits.
        ///
        /// An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
        /// at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
        /// joint does not stop at limits.
        /// Although currently not developed, it could be possible to support identification for joints that are not revolute.
        boost::array<uint8_t, 3> _bIsCircular = {0, 0, 0};

        bool _bIsActive = true;                 ///< if true, should belong to the DOF of the body, unless it is a mimic joint (_ComputeInternalInformation decides this)

        /// \brief _controlMode specifies how this joint is controlled. For possible control modes, see enum JointControlMode.
        JointControlMode _controlMode = JCM_None;
        JointControlInfo_RobotControllerPtr _jci_robotcontroller; ///< valid if controlMode==JCM_RobotController
        JointControlInfo_IOPtr _jci_io; ///< valid if controlMode==JCM_IO
        JointControlInfo_ExternalDevicePtr _jci_externaldevice; ///< valid if controlMode==JCM_ExternalDevice
        /// \brief deserializes a readable from rReadable and stores it into _mReadableInterfaces[id]
        void _DeserializeReadableInterface(const std::string& id, const rapidjson::Value& rReadable, dReal fUnitScale);
    };

    typedef boost::shared_ptr<JointInfo> JointInfoPtr;
    typedef boost::shared_ptr<JointInfo const> JointInfoConstPtr;

    /// \brief Information about a joint that controls the relationship between two links.
    class OPENRAVE_API Joint : public boost::enable_shared_from_this<Joint>, public ReadablesContainer
    {
public:
        /// \deprecated 12/10/19
        typedef Mimic MIMIC RAVE_DEPRECATED;
        typedef KinBody::JointType JointType RAVE_DEPRECATED;
        static const KinBody::JointType JointNone RAVE_DEPRECATED = KinBody::JointNone;
        static const KinBody::JointType JointHinge RAVE_DEPRECATED = KinBody::JointHinge;
        static const KinBody::JointType JointRevolute RAVE_DEPRECATED = KinBody::JointRevolute;
        static const KinBody::JointType JointSlider RAVE_DEPRECATED = KinBody::JointSlider;
        static const KinBody::JointType JointPrismatic RAVE_DEPRECATED = KinBody::JointPrismatic;
        static const KinBody::JointType JointRR RAVE_DEPRECATED = KinBody::JointRR;
        static const KinBody::JointType JointRP RAVE_DEPRECATED = KinBody::JointRP;
        static const KinBody::JointType JointPR RAVE_DEPRECATED = KinBody::JointPR;
        static const KinBody::JointType JointPP RAVE_DEPRECATED = KinBody::JointPP;
        static const KinBody::JointType JointSpecialBit RAVE_DEPRECATED = KinBody::JointSpecialBit;
        static const KinBody::JointType JointUniversal RAVE_DEPRECATED = KinBody::JointUniversal;
        static const KinBody::JointType JointHinge2 RAVE_DEPRECATED = KinBody::JointHinge2;
        static const KinBody::JointType JointSpherical RAVE_DEPRECATED = KinBody::JointSpherical;
        static const KinBody::JointType JointTrajectory RAVE_DEPRECATED = KinBody::JointTrajectory;

        Joint(KinBodyPtr parent, KinBody::JointType type = KinBody::JointNone);
        ~Joint();

        inline const std::string& GetId() const {
            return _info._id;
        }
        /// \brief The unique name of the joint
        inline const std::string& GetName() const {
            return _info._name;
        }

        inline dReal GetMaxVel(int iaxis=0) const {
            return _info._vmaxvel[iaxis];
        }
        inline dReal GetMaxAccel(int iaxis=0) const {
            return _info._vmaxaccel[iaxis];
        }
        inline dReal GetMaxJerk(int iaxis=0) const {
            return _info._vmaxjerk[iaxis];
        }

        inline dReal GetHardMaxVel(int iaxis=0) const {
            return _info._vhardmaxvel[iaxis];
        }
        inline dReal GetHardMaxAccel(int iaxis=0) const {
            return _info._vhardmaxaccel[iaxis];
        }
        inline dReal GetHardMaxJerk(int iaxis=0) const {
            return _info._vhardmaxjerk[iaxis];
        }

        ///< \brief gets the max instantaneous torque of the joint
        ///
        /// If _infoElectricMotor is filled, the will compute the max instantaneous torque depending on the current speed of the joint.
        dReal GetMaxTorque(int iaxis=0) const;

        ///< \brief gets the max instantaneous torque limits of the joint
        ///
        /// If _infoElectricMotor is filled, the will compute the nominal torque limits depending on the current speed of the joint.
        /// \return min and max of torque limits
        std::pair<dReal, dReal> GetInstantaneousTorqueLimits(int iaxis=0) const;

        ///< \brief gets the nominal torque limits of the joint
        ///
        /// If _infoElectricMotor is filled, the will compute the nominal torque limits depending on the current speed of the joint.
        /// \return min and max of torque limits
        std::pair<dReal, dReal> GetNominalTorqueLimits(int iaxis=0) const;

        inline dReal GetMaxInertia(int iaxis=0) const {
            return _info._vmaxinertia[iaxis];
        }

        /// \brief Get the degree of freedom index in the body's DOF array.
        ///
        /// This does not index in KinBody::GetJoints() directly! In other words, KinBody::GetDOFValues()[GetDOFIndex()] == GetValues()[0]
        inline int GetDOFIndex() const {
            return dofindex;
        }

        /// \brief Get the joint index into KinBody::GetJoints.
        inline int GetJointIndex() const {
            return jointindex;
        }

        /// \brief parent body that joint belongs to.
        ///
        /// \param trylock if true then will try to get the parent pointer and return empty pointer if parent was already destroyed. Otherwise throws an exception if parent is already destroyed. By default this should be
        inline KinBodyPtr GetParent(bool trylock=false) const {
            if( trylock ) {
                return _parent.lock();
            }
            else {
                return KinBodyPtr(_parent);
            }
        }

        inline const LinkPtr& GetFirstAttached() const {
            return _attachedbodies[0];
        }
        inline const LinkPtr& GetSecondAttached() const {
            return _attachedbodies[1];
        }

        inline KinBody::JointType GetType() const {
            return _info._type;
        }

        /// \brief gets all resolutions for the joint axes
        ///
        /// \param[in] bAppend if true will append to the end of the vector instead of erasing it
        void GetResolutions(std::vector<dReal>& resolutions, bool bAppend=false) const;

        /// \brief The discretization of the joint used when line-collision checking.
        ///
        /// The resolutions are set as large as possible such that the joint will not go through obstacles of determined size.
        dReal GetResolution(int iaxis=0) const;

        void SetResolution(dReal resolution, int iaxis=0);

        /// \brief The degrees of freedom of the joint. Each joint supports a max of 3 degrees of freedom.
        inline int GetDOF() const {
            return _info.GetDOF();
        }

        /// \brief Return true if any of the joint axes has an identification at some of its lower and upper limits.
        inline bool IsCircular() const {
            return _info._bIsCircular[0] || _info._bIsCircular[1] || _info._bIsCircular[2];
        }

        /// \brief Return true if joint is active
        bool IsActive() const;

        /// \brief Return true if joint axis has an identification at some of its lower and upper limits.
        ///
        /// An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
        /// at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
        /// joint does not stop at limits.
        /// Although currently not developed, it could be possible to support identification for joints that are not revolute.
        inline bool IsCircular(int iaxis) const {
            return static_cast<bool>(_info._bIsCircular[iaxis]);
        }

        /// \brief returns true if the axis describes a rotation around an axis.
        ///
        /// \param iaxis the axis of the joint to return the results for
        bool IsRevolute(int iaxis) const;

        /// \brief returns true if the axis describes a translation around an axis.
        ///
        /// \param iaxis the axis of the joint to return the results for
        bool IsPrismatic(int iaxis) const;

        /// \brief Return true if joint can be treated as a static binding (ie all limits are 0)
        bool IsStatic() const;

        /// \brief Return all the joint values with the correct offsets applied
        ///
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return degrees of freedom of the joint (even if pValues is NULL)
        void GetValues(std::vector<dReal>& values, bool bAppend=false) const;
        void GetValues(boost::array<dReal,3>& values) const;

        /// \brief Return the value of the specified joint axis only.
        dReal GetValue(int axis) const;

        /// \brief Gets the joint velocities
        ///
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return the degrees of freedom of the joint (even if pValues is NULL)
        void GetVelocities(std::vector<dReal>& values, bool bAppend=false) const;
        void GetVelocities(boost::array<dReal,3>& values) const;

        /// \brief Return the velocity of the specified joint axis only.
        dReal GetVelocity(int axis) const;

        /// \brief Add effort (force or torque) to the joint
        void AddTorque(const std::vector<dReal>& torques);

        /// \brief The anchor of the joint in global coordinates.
        Vector GetAnchor() const;

        /// \brief The axis of the joint in global coordinates
        ///
        /// \param[in] axis the axis to get
        Vector GetAxis(int axis = 0) const;

        /** \brief Get the limits of the joint

            \param[out] vLowerLimit the lower limits
            \param[out] vUpperLimit the upper limits
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend=false) const;

        inline void GetLimits(boost::array<dReal,3>& vLowerLimit, boost::array<dReal,3>& vUpperLimit) const {
            vLowerLimit = _info._vlowerlimit;
            vUpperLimit = _info._vupperlimit;
        }

        /// \brief returns the lower and upper limit of one axis of the joint
        inline std::pair<dReal, dReal> GetLimit(int iaxis=0) const {
            return std::make_pair(_info._vlowerlimit.at(iaxis),_info._vupperlimit.at(iaxis));
        }

        inline dReal GetLowerLimit(int iaxis=0) const {
            return _info._vlowerlimit[iaxis];
        }

        inline dReal GetUpperLimit(int iaxis=0) const {
            return _info._vupperlimit[iaxis];
        }

        /// \brief \see GetLimits
        void SetLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper);

        /** \brief Returns the max velocities of the joint

            \param[out] the max velocity
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        void GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool bAppend=false) const;

        /// \brief returns the lower and upper velocity limit of one axis of the joint
        std::pair<dReal, dReal> GetVelocityLimit(int iaxis=0) const;

        /// \brief \see GetVelocityLimits
        void SetVelocityLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max accelerations of the joint

            \param[out] the max acceleration
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        dReal GetAccelerationLimit(int iaxis=0) const;

        /// \brief \see GetAccelerationLimits
        void SetAccelerationLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max jerks of the joint

            \param[out] the max jerk
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetJerkLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        dReal GetJerkLimit(int iaxis=0) const;

        /// \brief \see GetJerkLimits
        void SetJerkLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the hard max velocities of the joint

            \param[out] the max vel
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetHardVelocityLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        dReal GetHardVelocityLimit(int iaxis=0) const;

        /// \brief \see GetHardVelocityLimits
        void SetHardVelocityLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the hard max accelerations of the joint

            \param[out] the max accel
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetHardAccelerationLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        dReal GetHardAccelerationLimit(int iaxis=0) const;

        /// \brief \see GetHardAccelerationLimits
        void SetHardAccelerationLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the hard max jerks of the joint

            \param[out] the max jerk
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetHardJerkLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        dReal GetHardJerkLimit(int iaxis=0) const;

        /// \brief \see GetHardJerkLimits
        void SetHardJerkLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max torques of the joint

            \param[out] the max torque
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetTorqueLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        /// \brief \see GetTorqueLimits
        void SetTorqueLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max inertias of the joint

            \param[out] the max inertia
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        void GetInertiaLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        /// \brief \see GetInertiaLimits
        void SetInertiaLimits(const std::vector<dReal>& vmax);

        /// \brief gets all weights for the joint axes
        ///
        /// \param[in] bAppend if true will append to the end of the vector instead of erasing it
        void GetWeights(std::vector<dReal>& weights, bool bAppend=false) const;

        /// \brief The weight associated with a joint's axis for computing a distance in the robot configuration space.
        dReal GetWeight(int axis=0) const;

        /// \brief \see GetWeight
        void SetWeights(const std::vector<dReal>& weights);

        /// \brief Computes the configuration difference values1-values2 and stores it in values1.
        ///
        /// Takes into account joint limits and wrapping of circular joints.
        void SubtractValues(std::vector<dReal>& values1, const std::vector<dReal>& values2) const;

        /// \brief Returns the configuration difference value1-value2 for axis i.
        ///
        /// Takes into account joint limits and wrapping of circular joints.
        dReal SubtractValue(dReal value1, dReal value2, int iaxis) const;

        /// \brief Return internal offset parameter that determines the branch the angle centers on
        ///
        /// Wrap offsets are needed for rotation joints since the range is limited to 2*pi.
        /// This allows the wrap offset to be set so the joint can function in [-pi+offset,pi+offset]..
        /// \param iaxis the axis to get the offset from
        inline dReal GetWrapOffset(int iaxis=0) const {
            return _info._voffsets.at(iaxis);
        }

        inline dReal GetOffset(int iaxis=0) const RAVE_DEPRECATED {
            return GetWrapOffset(iaxis);
        }

        /// \brief \see GetWrapOffset
        void SetWrapOffset(dReal offset, int iaxis=0);

        void serialize(std::ostream& o, int options) const;

        /// @name Internal Hierarchy Methods
        //@{
        /// \brief Return the parent link which the joint measures its angle off from (either GetFirstAttached() or GetSecondAttached())
        LinkPtr GetHierarchyParentLink() const;
        /// \brief Return the child link whose transformation is computed by this joint's values (either GetFirstAttached() or GetSecondAttached())
        LinkPtr GetHierarchyChildLink() const;
        /// \brief The axis of the joint in local coordinates.
        const Vector& GetInternalHierarchyAxis(int axis = 0) const;
        /// \brief Left multiply transform given the base body.
        const Transform& GetInternalHierarchyLeftTransform() const;
        /// \brief Right multiply transform given the base body.
        const Transform& GetInternalHierarchyRightTransform() const;
        //@}

        /// A mimic joint's angles are automatically determined from other joints based on a general purpose formula.
        /// A user does not have control of the the mimic joint values, even if they appear in the DOF list.
        /// @name Mimic Joint Properties
        //@{

        /// \deprecated (11/1/1)
        int GetMimicJointIndex() const RAVE_DEPRECATED;
        /// \deprecated (11/1/1)
        const std::vector<dReal> GetMimicCoeffs() const RAVE_DEPRECATED;

        /// \brief Returns true if a particular axis of the joint is mimiced.
        ///
        /// \param axis the axis to query. When -1 returns true if any of the axes have mimic joints
        bool IsMimic(int axis=-1) const;

        /** \brief If the joint is mimic, returns the equation to compute its value

            \param[in] axis the axis index
            \param[in] type 0 for position, 1 for velocity, 2 for acceleration.
            \param[in] format the format the equations are returned in. If empty or "fparser", equation in fparser format. Also supports: "mathml".

            MathML:

            Set 'format' to "mathml". The joint variables are specified with <csymbol>. If a targetted joint has more than one degree of freedom, then axis is suffixed with _\%d. If 'type' is 1 or 2, the partial derivatives are outputted as consecutive <math></math> tags in the same order as \ref Mimic::_vdofformat
         */
        std::string GetMimicEquation(int axis=0, int type=0, const std::string& format="") const;

        /** \brief Returns the set of DOF indices that the computation of a joint axis depends on. Order is arbitrary.

            If the mimic joint uses the values of other mimic joints, then the dependent DOFs of that joint are also
            copied over. Therefore, the dof indices returned can be more than the actual variables used in the equation.
            \throw openrave_exception Throws an exception if the axis is not mimic.
         */
        void GetMimicDOFIndices(std::vector<int>& vmimicdofs, int axis=0) const;

        /** \brief Sets the mimic properties of the joint.

            The equations can use the joint names directly in the equation, which represent the position of the joint. Any non-mimic joint part of KinBody::GetJoints() can be used in the computation of the values.
            If a joint has more than one degree of freedom, then suffix it '_' and the axis index. For example universaljoint_0 * 10 + sin(universaljoint_1).

            See http://warp.povusers.org/FunctionParser/fparser.html for a full description of the equation formats.

            The velocity and acceleration equations are specified in terms of partial derivatives, which means one expression needs to be specified per degree of freedom of used. In order to separate the expressions use "|name ...". The name should immediately follow  '|'.  For example:

         |universaljoint_0 10 |universaljoint_1 10*cos(universaljoint_1)

            If there is only one variable used in the position equation, then the equation can be specified directly without using "{}".

            \param[in] axis the axis to set the properties for.
            \param[in] poseq Equation for joint's position. If it is empty, the mimic properties are turned off for this joint.
            \param[in] veleq First-order partial derivatives of poseq with respect to all used DOFs. Only the variables used in poseq are allowed to be used. If poseq is not empty, this is required.
            \param[in] acceleq Second-order partial derivatives of poseq with respect to all used DOFs. Only the variables used in poseq are allowed to be used. Optional.
            \throw openrave_exception Throws an exception if the mimic equation is invalid in any way.
         */
        void SetMimicEquations(int axis, const std::string& poseq, const std::string& veleq, const std::string& acceleq="");
        //@}

        /// \brief return a map of custom float parameters
        inline const std::map<std::string, std::vector<dReal> >& GetFloatParameters() const {
            return _info._mapFloatParameters;
        }

        /// \brief set custom float parameters
        ///
        /// \param parameters if empty, then removes the parameter
        void SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters);

        /// \brief return a map of custom integer parameters
        inline const std::map<std::string, std::vector<int> >& GetIntParameters() const {
            return _info._mapIntParameters;
        }

        /// \brief set custom int parameters
        ///
        /// \param parameters if empty, then removes the parameter
        void SetIntParameters(const std::string& key, const std::vector<int>& parameters);

        /// \brief return a map of custom string parameters
        inline const std::map<std::string, std::string >& GetStringParameters() const {
            return _info._mapStringParameters;
        }

        /// \brief set custom string parameters
        ///
        /// \param parameters if empty, then removes the parameter
        void SetStringParameters(const std::string& key, const std::string& value);

        /// \brief return controlMode for this joint
        inline JointControlMode GetControlMode() const {
            return _info._controlMode;
        }

        /// \brief Updates several fields in \ref _info depending on the current state of the joint.
        void UpdateInfo();
        /// \brief returns the JointInfo structure containing all information.
        ///
        /// Some values in this structure like _vcurrentvalues need to be updated, so make sure to call \ref UpdateInfo() right before this function is called.
        inline const KinBody::JointInfo& GetInfo() const {
            return _info;
        }

        /// \brief Calls \ref UpdateInfo and returns the joint structure
        inline const KinBody::JointInfo& UpdateAndGetInfo() {
            UpdateInfo();
            return _info;
        }

        /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
        void ExtractInfo(KinBody::JointInfo& info) const;

        /// \brief update Joint according to new JointInfo, returns false if update cannot be performed and requires InitFromInfo
        UpdateFromInfoResult UpdateFromInfo(const KinBody::JointInfo& info);

        /// \brief update the cached _doflastsetvalues
        //virtual void SetDOFLastSetValue(dReal dofvalue, const int iaxis = 0);

protected:
        JointInfo _info;

        boost::array< MimicPtr,3> _vmimic;          ///< the mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free. When cloning, is it ok to copy this and assume it is constant?

        /** \brief computes the partial velocities with respect to all dependent DOFs specified by Mimic::_vmimicdofs.

            If the joint is not mimic, then just returns its own index
            \param[out] vpartials A list of dofindex/velocity_partial pairs. The final velocity is computed by taking the dot product. The dofindices do not repeat.
            \param[in] iaxis the axis
            \param[in,out] vcachedpartials set of cached partials for each degree of freedom
         */
        void _ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials, const int iaxis, std::map< std::pair<Mimic::DOFFormat, int>, dReal >& mapcachedpartials) const;

        /** \brief Compute internal transformations and specify the attached links of the joint.

            Called after the joint protected parameters {vAxes, vanchor, and _voffsets}  have been initialized. vAxes and vanchor should be in the frame of plink0.
            Compute the left and right multiplications of the joint transformation and cleans up the attached bodies.
            After function completes, the following parameters are initialized: _tRight, _tLeft, _tinvRight, _tinvLeft, _attachedbodies. _attachedbodies does not necessarily contain the links in the same order as they were input.
            \param plink0 the first attaching link, all axes and anchors are defined in its coordinate system
            \param plink1 the second attaching link
            \param vanchor the anchor of the rotation axes
            \param vaxes the axes in plink0's coordinate system of the joints
            \param vinitialvalues the current values of the robot used to set the 0 offset of the robot
            \param bProcessStatic if true, then check to see if the joint is static and then set its cache and reduce its limits. If false, treat the joint as non-static.
         */
        void _ComputeJointInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchor, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues);

        /// \brief once all the joints have been computed and initiailzed, call this function
        void _ComputeInternalStaticInformation();

        /// \brief evaluates the mimic joint equation using vdependentvalues
        ///
        /// \param[in] axis the joint axis
        /// \param[in] timederiv the time derivative to evaluate. 0 is position, 1 is velocity, 2 is acceleration, etc
        /// \param[in] vdependentvalues input values ordered with respect to _vdofformat[iaxis]
        /// \param[out] voutput the output values
        /// \return an internal error code, 0 if no error
        int _Eval(int axis, uint32_t timederiv, const std::vector<dReal>& vdependentvalues, std::vector<dReal>& voutput) const;

        /// \brief compute joint velocities given the parent and child link transformations/velocities
        ///
        /// \param pValues[out] where velocities get output to, allows to write GetDOF() entries
        void _GetVelocities(dReal* pValues, const std::pair<Vector,Vector>& linkparentvelocity, const std::pair<Vector,Vector>& linkchildvelocity) const;

        /// \brief Return the velocity of the specified joint axis only.
        dReal _GetVelocity(int axis, const std::pair<Vector,Vector>&linkparentvelocity, const std::pair<Vector,Vector>&linkchildvelocity) const;

        boost::array<dReal,3> _doflastsetvalues; ///< the last set value by the kinbody (_voffsets not applied). For revolute joints that have a range greater than 2*pi, it is only possible to recover the joint value from the link positions mod 2*pi. In order to recover the branch, multiplies of 2*pi are added/subtracted to this value that is closest to _doflastsetvalues. For circular joints, the last set value can be ignored since they always return a value from [-pi,pi)

private:
        /// Sensitive variables that should not be modified.
        /// @name Private Joint Variables
        //@{
        int dofindex;                   ///< the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        int jointindex;                 ///< the joint index into KinBody::_vecjoints
        boost::array<dReal,3> _vcircularlowerlimit, _vcircularupperlimit;         ///< for circular joints, describes where the identification happens. this is set internally in _ComputeInternalInformation

        KinBodyWeakPtr _parent;               ///< body that joint belong to
        boost::array<LinkPtr,2> _attachedbodies;         ///< attached bodies. The link in [0] is computed first in the hierarchy before the other body.
        boost::array<Vector,3> _vaxes;                ///< normalized axes, this can be different from _info._vaxes and reflects how _tRight and _tLeft are computed
        Transform _tRight, _tLeft;         ///< transforms used to get body[1]'s transformation with respect to body[0]'s: Tbody1 = Tbody0 * tLeft * JointOffsetLeft * JointRotation * JointOffsetRight * tRight
        Transform _tRightNoOffset, _tLeftNoOffset;         ///< same as _tLeft and _tRight except it doesn't not include the offset
        Transform _tinvRight, _tinvLeft;         ///< the inverse transformations of tRight and tLeft
        bool _bInitialized;
        int8_t _nIsStatic; ///< If 1, then joint is static and shouldnot move. If 0, then joint is not static. If -1, then still unknown
        //@}
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class OpenRAVEXMLParser::JointXMLReader;
        friend class OpenRAVEXMLParser::KinBodyXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
        friend class XFileReader;
#else
        friend class ::OpenRAVEXMLParser::JointXMLReader;
        friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
        friend class ::XFileReader;
#endif
#endif
        friend class ColladaReader;
        friend class ColladaWriter;
        friend class KinBody;
        friend class RobotBase;
    };
    typedef boost::shared_ptr<KinBody::Joint> JointPtr;
    typedef boost::shared_ptr<KinBody::Joint const> JointConstPtr;
    typedef boost::weak_ptr<KinBody::Joint> JointWeakPtr;

    /// \brief holds all user-set attached sensor information used to initialize the AttachedSensor class.
    ///
    /// This is serializable and independent of environment.
    class OPENRAVE_API GrabbedInfo : public InfoBase
    {
public:
        GrabbedInfo() {
        }
        GrabbedInfo(const GrabbedInfo& other);
        bool operator==(const GrabbedInfo& other) const;
        bool operator!=(const GrabbedInfo& other) const {
            return !operator==(other);
        }

        GrabbedInfo& operator=(const GrabbedInfo& other);

        void Reset() override;
        void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options=0) const override;
        void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

        void serialize(std::ostream& o) const; ///< used only for hash computation
        std::string GetGrabbedInfoHash() const;

        std::string _id; ///< unique id of the grabbed info
        std::string _grabbedname; ///< the name of the body to grab
        std::string _robotlinkname;  ///< the name of the body link that is grabbing the body
        Transform _trelative; ///< transform of first link of body relative to _robotlinkname's transform. In other words, grabbed->GetTransform() == bodylink->GetTransform()*trelative
        std::set<std::string> _setIgnoreRobotLinkNames; ///< names of links of the body to force ignoring because of pre-existing collions at the time of grabbing. Note that this changes depending on the configuration of the body and the relative position of the grabbed body.
        rapidjson::Document _rGrabbedUserData; ///< user-defined data to be updated when kinbody grabs and releases objects
    };
    typedef boost::shared_ptr<GrabbedInfo> GrabbedInfoPtr;
    typedef boost::shared_ptr<GrabbedInfo const> GrabbedInfoConstPtr;

    /// \brief Stores the state of the current body that is published in a thread safe way from the environment without requiring locking the environment.
    class BodyState
    {
public:
        BodyState() : updatestamp(0), environmentid(0) {
        }
        ~BodyState() {
        }

        /// \brief clears any previous set state
        inline void Reset() {
            strname.clear();
            pbody.reset();
            vectrans.clear();
            vLinkEnableStates.clear();
            vConnectedBodyActiveStates.clear();
            jointvalues.clear();
            uri.clear();
            updatestamp = 0;
            environmentid = 0;
            activeManipulatorName.clear();
            activeManipulatorTransform = Transform();
            vGrabbedInfos.clear();
        }

        KinBodyPtr pbody; ///< pointer to the body. if using this, make sure the environment is locked.
        std::string strname;         ///< \see KinBody::GetName
        std::vector<Transform> vectrans; ///< \see KinBody::GetLinkTransformations
        std::vector<uint8_t> vLinkEnableStates; ///< \see KinBody::GetLinkEnableStates
        std::vector<int8_t> vConnectedBodyActiveStates; ///< IsActive state of ConnectedBody \see RobotBase::GetConnectedBodyActiveStates
        std::vector<dReal> jointvalues; ///< \see KinBody::GetDOFValues
        std::string uri; ///< \see KinBody::GetURI
        int updatestamp; ///< \see KinBody::GetUpdateStamp
        int environmentid; ///< \see KinBody::GetEnvironmentBodyIndex
        std::string activeManipulatorName; ///< the currently active manpiulator set for the body
        Transform activeManipulatorTransform; ///< the active manipulator's transform
        std::vector<GrabbedInfo> vGrabbedInfos; ///< list of grabbed bodies
    };
    typedef boost::shared_ptr<KinBody::BodyState> BodyStatePtr;
    typedef boost::shared_ptr<KinBody::BodyState const> BodyStateConstPtr;

    /// \brief Access point of the sensor system that manages the body.
    class OPENRAVE_API ManageData : public boost::enable_shared_from_this<ManageData>
    {
public:
        ManageData(SensorSystemBasePtr psensorsystem) : _psensorsystem(psensorsystem) {
        }
        virtual ~ManageData() {
        }

        virtual SensorSystemBasePtr GetSystem() {
            return SensorSystemBasePtr(_psensorsystem);
        }

        /// returns a pointer to the data used to initialize the BODY with AddKinBody.
        /// if psize is not NULL, will be filled with the size of the data in bytes
        /// This function will be used to restore bodies that were removed
        virtual ReadableConstPtr GetData() const = 0;

        /// particular link that sensor system is tracking.
        /// All transformations describe this link.
        virtual KinBody::LinkPtr GetOffsetLink() const = 0;

        /// true if the object is being updated by the system due to its presence in the real environment
        virtual bool IsPresent() const = 0;

        /// true if should update openrave body
        virtual bool IsEnabled() const = 0;

        /// if true, the vision system should not destroy this object once it stops being present
        virtual bool IsLocked() const = 0;

        /// set a lock on a particular body
        virtual bool Lock(bool bDoLock) = 0;

private:
        /// the system that owns this class, note that it is a weak pointer in order because
        /// this object is managed by the sensor system and should be deleted when it goes out of scope.
        SensorSystemBaseWeakPtr _psensorsystem;
    };

    typedef boost::shared_ptr<KinBody::ManageData> ManageDataPtr;
    typedef boost::shared_ptr<KinBody::ManageData const> ManageDataConstPtr;

    /// \brief Parameters passed into the state savers to control what information gets saved.
    enum SaveParameters
    {
        Save_LinkTransformation              = 0x00000001, ///< [default] save link transformations and joint branches
        Save_LinkEnable                      = 0x00000002, ///< [default] save link enable states
        Save_LinkVelocities                  = 0x00000004, ///< save the link velocities
        Save_JointMaxVelocityAndAcceleration = 0x00000008, ///< save the max joint velocities and accelerations for the controller DOF
        Save_JointWeights                    = 0x00000010, ///< saves the dof weights
        Save_JointLimits                     = 0x00000020, ///< saves the dof limits
        Save_JointResolutions                = 0x00000040, ///< saves the dof resolutions
        Save_ActiveDOF                       = 0x00010000, ///< [robot only], saves and restores the current active degrees of freedom
        Save_ActiveManipulator               = 0x00020000, ///< [robot only], saves the active manipulator
        Save_GrabbedBodies                   = 0x00040000, ///< saves the grabbed state of the bodies. This does not affect the configuraiton of those bodies. Although saver support restoring to the kinbody in other env in general, this Save_GrabbedBodies option does not support for restoring to the kinbody in other env. This is because it cannot successfuly identify the grabbed bodies in other env based on environment body index.
        Save_ActiveManipulatorToolTransform  = 0x00080000, ///< [robot only], saves the active manipulator's LocalToolTransform, LocalToolDirection, and IkSolver
        Save_ManipulatorsToolTransform       = 0x00100000, ///< [robot only], saves every manipulator's LocalToolTransform, LocalToolDirection, and IkSolver
        Save_ConnectedBodies                 = 0x00200000, ///< [robot only], saves the connected body states
    };

    /// \brief info structure used to initialize a kinbody
    class OPENRAVE_API KinBodyInfo : public InfoBase
    {
public:
        KinBodyInfo() {
        }
        bool operator==(const KinBodyInfo& other) const;
        bool operator!=(const KinBodyInfo& other) const {
            return !operator==(other);
        }

        void Reset() override;
        void SerializeJSON(rapidjson::Value& value, rapidjson::Document::AllocatorType& allocator, dReal fUnitScale, int options=0) const override;
        void DeserializeJSON(const rapidjson::Value& value, dReal fUnitScale, int options) override;

        std::string _id; ///< unique id of the body used to identify it when changing it.
        std::string _name; ///< name of the body
        std::string _uri; ///< uri this body comes from
        std::string _referenceUri;  ///< referenced body info uri this body should be loaded from before the Info data is applied
        std::string _interfaceType; ///< the interface type

        Transform _transform; ///< transform of the base link
        std::vector< std::pair< std::pair<std::string, int>, dReal> > _dofValues; ///< mapping from (jointName, jointAxis) to dofValue
        std::vector<GrabbedInfoPtr> _vGrabbedInfos; ///< list of pointers to GrabbedInfo

        std::vector<LinkInfoPtr> _vLinkInfos; ///< list of pointers to LinkInfo
        std::vector<JointInfoPtr> _vJointInfos; ///< list of pointers to JointInfo

        std::map<std::string, ReadablePtr> _mReadableInterfaces; ///< readable interface mapping

        boost::shared_ptr<rapidjson::Document> _prAssociatedFileEntries; ///< files tag maintaining entries of data files associated with this object

        bool _isRobot = false; ///< true if should create a RobotBasePtr
        bool _isPartial = true; ///< true if this info contains partial information. false if the info contains the full body information and can ignore anything that is currently saved on the environment when updating.

        enum KinBodyInfoField {
            KBIF_Transform = (1 << 0), // _transform field
            KBIF_DOFValues = (1 << 1), // _dofValues field
            KBIF_URI = (1 << 2), // _uri field
            KBIF_ReferenceURI = (1 << 3), // _referenceUri field
        };
        inline bool IsModifiedField(KinBodyInfoField field) const {
            return !!(_modifiedFields & field);
        }
        /// \brief adds modified fields
        inline void AddModifiedField(KinBodyInfoField field) {
            _modifiedFields |= field;
        }

private:
        uint32_t _modifiedFields = 0xffffffff; ///< a bitmap of KinBodyInfoField, for supported fields, indicating which fields are touched, otherwise they can be skipped in UpdateFromInfo. By default, assume all fields are modified.

protected:
        /// \brief deserializes a readable from rReadable and stores it into _mReadableInterfaces[id]
        virtual void _DeserializeReadableInterface(const std::string& id, const rapidjson::Value& rReadable, dReal fUnitScale);

        friend class KinBody; ///< for changing _modifiedFields

    };
    typedef boost::shared_ptr<KinBodyInfo> KinBodyInfoPtr;
    typedef boost::shared_ptr<KinBodyInfo const> KinBodyInfoConstPtr;

    /// \brief Saved data for Grabbed used in KinBodyStateSaver and KinBodyStateSaverRef
    ///        When KinBody::Grab, KinBody::Release, ...etc are called, new Grabbed instance is created in KinBody and the original ptr for the original Grabbed instance is swapped.
    ///        Thus, the original information of Grabbed instance is unchanged and holding the ptr of it as pGrabbed is enough for the saver.
    ///        However, some of the grab-related API mutates the internal states of Grabbed instance.
    ///        To restore the original information even such case, each saver should have copied values of such internal states.
    struct SavedGrabbedData
    {
        GrabbedPtr pGrabbed; ///< pointer of original Grabbed instance, which originally in KinBody's _grabbedBodiesByEnvironmentIndex.
        std::list<KinBody::LinkConstPtr> listNonCollidingLinksWhenGrabbed; ///< copied values of Grabbed's _listNonCollidingLinksWhenGrabbed. See also the documentation of Grabbed class.
        std::set<int> setGrabberLinkIndicesToIgnore; ///< copied values of Grabbed's _setGrabberLinkIndicesToIgnore. See also the documentation of Grabbed class.
        bool listNonCollidingIsValid = false; ///< copied values of Grabbed's _listNonCollidingIsValid. See also the documentation of Grabbed class.
    };

    /// \brief Helper class to save and restore the entire kinbody state.
    ///
    /// Options can be passed to the constructor in order to choose which parameters to save (see \ref SaveParameters)
    class OPENRAVE_API KinBodyStateSaver
    {
public:
        KinBodyStateSaver(KinBodyPtr pbody, int options = Save_LinkTransformation|Save_LinkEnable);
        virtual ~KinBodyStateSaver();
        inline const KinBodyPtr& GetBody() const {
            return _pbody;
        }

        /// \brief restore the state
        ///
        /// \param body if set, will attempt to restore the stored state to the passed in body, otherwise will restore it for the original body.
        /// \throw openrave_exception if the passed in body is not compatible with the saved state, will throw
        virtual void Restore(boost::shared_ptr<KinBody> body=boost::shared_ptr<KinBody>());

        /// \brief release the body state. _pbody will not get restored on destruction
        ///
        /// After this call, it will still be possible to use \ref Restore.
        virtual void Release();

        /// \brief sets whether the state saver will restore the state on destruction. by default this is true.
        virtual void SetRestoreOnDestructor(bool restore);
protected:
        KinBodyPtr _pbody;
        int _options;         ///< saved options
        std::vector<Transform> _vLinkTransforms;
        std::vector<uint8_t> _vEnabledLinks;
        std::vector<std::pair<Vector,Vector> > _vLinkVelocities;
        std::vector<dReal> _vdoflastsetvalues;
        std::vector<dReal> _vMaxVelocities, _vMaxAccelerations, _vMaxJerks, _vDOFWeights, _vDOFLimits[2], _vDOFResolutions;
        std::unordered_map<int, SavedGrabbedData> _grabbedDataByEnvironmentIndex;
        bool _bRestoreOnDestructor;
private:
        virtual void _RestoreKinBody(boost::shared_ptr<KinBody> body);
    };

    typedef boost::shared_ptr<KinBodyStateSaver> KinBodyStateSaverPtr;

    /// \brief Helper class to save and restore the entire kinbody state, using only kinbody references.
    ///
    /// Since using only reference, have to be careful of the scope of the kinbody
    /// Options can be passed to the constructor in order to choose which parameters to save (see \ref SaveParameters)
    class OPENRAVE_API KinBodyStateSaverRef
    {
public:
        KinBodyStateSaverRef(KinBody& body, int options = Save_LinkTransformation|Save_LinkEnable);
        virtual ~KinBodyStateSaverRef();
        inline KinBody& GetBody() const {
            return _body;
        }

        /// \brief restore the state
        ///
        /// \throw openrave_exception if the passed in body is not compatible with the saved state, will throw
        virtual void Restore();

        /// \param body if set, will attempt to restore the stored state to the passed in body, otherwise will restore it for the original body.
        /// \throw openrave_exception if the passed in body is not compatible with the saved state, will throw
        virtual void Restore(KinBody& newbody);

        /// \brief release the body state. _pbody will not get restored on destruction
        ///
        /// After this call, it will still be possible to use \ref Restore.
        virtual void Release();

        /// \brief sets whether the state saver will restore the state on destruction. by default this is true.
        virtual void SetRestoreOnDestructor(bool restore);
protected:
        KinBody& _body;

        int _options;         ///< saved options
        std::vector<Transform> _vLinkTransforms;
        std::vector<uint8_t> _vEnabledLinks;
        std::vector<std::pair<Vector,Vector> > _vLinkVelocities;
        std::vector<dReal> _vdoflastsetvalues;
        std::vector<dReal> _vMaxVelocities, _vMaxAccelerations, _vMaxJerks, _vDOFWeights, _vDOFLimits[2], _vDOFResolutions;
        std::unordered_map<int, SavedGrabbedData> _grabbedDataByEnvironmentIndex;
        bool _bRestoreOnDestructor;
        bool _bReleased; ///< if true, then body should not be restored
private:
        virtual void _RestoreKinBody(KinBody& body);
    };

    typedef boost::shared_ptr<KinBodyStateSaverRef> KinBodyStateSaverRefPtr;

    virtual ~KinBody();

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_KinBody;
    }

    virtual void Destroy();

    /// \brief Create a kinbody with one link composed of an array of aligned bounding boxes.
    ///
    /// \param boxes the array of aligned bounding boxes that will comprise of the body
    /// \param visible if true, the boxes will be rendered in the scene
    /// \param uri the new URI to set for the interface
    virtual bool InitFromBoxes(const std::vector<AABB>& boxes, bool visible=true, const std::string& uri=std::string());

    /// \brief Create a kinbody with one link composed of an array of oriented bounding boxes.
    ///
    /// \param boxes the array of oriented bounding boxes that will comprise of the body
    /// \param visible if true, the boxes will be rendered in the scene
    /// \param uri the new URI to set for the interface
    virtual bool InitFromBoxes(const std::vector<OBB>& boxes, bool visible=true, const std::string& uri=std::string());

    /// \brief Create a kinbody with one link composed of an array of spheres
    ///
    /// \param spheres the XYZ position of the spheres with the W coordinate representing the individual radius
    /// \param visible if true, the boxes will be rendered in the scene
    /// \param uri the new URI to set for the interface
    virtual bool InitFromSpheres(const std::vector<Vector>& spheres, bool visible=true, const std::string& uri=std::string());

    /// \brief Create a kinbody with one link composed of a triangle mesh surface
    ///
    /// \param trimesh the triangle mesh
    /// \param visible if true, will be rendered in the scene
    /// \param uri the new URI to set for the interface
    virtual bool InitFromTrimesh(const TriMesh& trimesh, bool visible=true, const std::string& uri=std::string());

    /// \brief Create a kinbody with one link composed of a list of geometries
    ///
    /// \param geometries a list of geometry infos to be initialized into new geometry objects, note that the geometry info data is copied
    /// \param visible if true, will be rendered in the scene
    /// \param uri the new URI to set for the interface
    virtual bool InitFromGeometries(const std::vector<KinBody::GeometryInfo>& geometries, const std::string& uri=std::string());

    /// \brief initializes an complex kinematics body with links and joints
    ///
    /// \param linkinfos information for all the links. Links will be created in this order
    /// \param jointinfos information for all the joints. Joints might be rearranged depending on their mimic properties
    /// \param uri the new URI to set for the interface
    virtual bool Init(const std::vector<LinkInfoConstPtr>& linkinfos, const std::vector<JointInfoConstPtr>& jointinfos, const std::string& uri=std::string());

    /// \brief initializes a simple kinematics body with rigidly attached links
    ///
    /// \param linkinfos information for all the links. Links will be created in this order
    /// \param uri the new URI to set for the interface
    virtual void InitFromLinkInfos(const std::vector<LinkInfo>& linkinfos, const std::string& uri=std::string());

    /// \brief initializes an complex kinematics body from info structure
    virtual bool InitFromKinBodyInfo(const KinBodyInfo& info);

    /// \brief Sets new geometries for all the links depending on the stored extra geometries each link has.
    ///
    /// \param name The name of the extra geometries group stored in each link.
    /// \param propagateGroupNameToSelfCollisionChecker It true, propagate this geometry group change to self collision checker. It is always propagated to env collision checker.
    /// The geometries can be set while the body is added to the environment. No other information will be touched.
    /// This method is faster than Link::SetGeometriesFromGroup since it makes only one change callback.
    /// \throw If any links do not have the particular geometry, an exception will be raised.
    virtual void SetLinkGeometriesFromGroup(const std::string& name, const bool propagateGroupNameToSelfCollisionChecker);

    /// \brief Stores geometries for later retrieval for all the links at the same time.
    ///
    /// \param name The name of the extra geometries group to be stored in each link.
    /// \param linkgeometries a vector containing a collection of geometry infos ptr for each links
    /// Note that the pointers are copied and not the data, so be careful not to modify the geometries afterwards
    /// This method is faster than Link::SetGeometriesFromGroup since it makes only one change callback.
    virtual void SetLinkGroupGeometries(const std::string& name, const std::vector< std::vector<KinBody::GeometryInfoPtr> >& linkgeometries);

    /// \brief Unique name of the body.
    virtual const std::string& GetName() const {
        return _name;
    }

    /// \brief Set the name of the body, notifies the environment and checks for uniqueness.
    virtual void SetName(const std::string& name);

    /// \brief Set the id of the body, notifies the environment and checks for uniqueness.
    virtual void SetId(const std::string& newid);

    /// \brief Unique name of the body.
    inline const std::string& GetId() const {
        return _id;
    }

    /// \brief Unique name of the body.
    inline const std::string& GetReferenceURI() const {
        return _referenceUri;
    }

    /// Methods for accessing basic information about joints
    /// @name Basic Information
    //@{

    /// \brief Number controllable degrees of freedom of the body.
    ///
    /// Only uses _vecjoints and last joint for computation, so can work before _ComputeInternalInformation is called.
    int GetDOF() const;

    /// \brief Returns all the joint values as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFValues(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint velocities as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFVelocities(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint limits as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint velocity limits as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFVelocityLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max velocity for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFVelocityLimits(std::vector<dReal>& maxvelocities, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max acceleration for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFAccelerationLimits(std::vector<dReal>& maxaccelerations, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max jerk for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFJerkLimits(std::vector<dReal>& maxjerks, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the hard max velocity for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFHardVelocityLimits(std::vector<dReal>& maxvels, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the hard max acceleration for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFHardAccelerationLimits(std::vector<dReal>& maxaccels, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the hard max jerk for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFHardJerkLimits(std::vector<dReal>& maxjerks, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max torque for each DOF
    void GetDOFTorqueLimits(std::vector<dReal>& maxaccelerations) const;

    void GetDOFMaxTorque(std::vector<dReal>& v) const;

    /// \brief get the dof resolutions
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFResolutions(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief get dof weights
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    void GetDOFWeights(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief \see GetDOFVelocityLimits
    virtual void SetDOFVelocityLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFAccelerationLimits
    virtual void SetDOFAccelerationLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFJerkLimits
    virtual void SetDOFJerkLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFHardVelocityLimits
    virtual void SetDOFHardVelocityLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFHardAccelerationLimits
    virtual void SetDOFHardAccelerationLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFHardJerkLimits
    virtual void SetDOFHardJerkLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFTorqueLimits
    virtual void SetDOFTorqueLimits(const std::vector<dReal>& maxlimits);

    /// \brief sets dof weights
    ///
    /// \param dofindices the dof indices to set the values for. If empty, will use all the dofs
    virtual void SetDOFWeights(const std::vector<dReal>& weights, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief sets dof resolutoins
    ///
    /// \param dofindices the dof indices to set the values for. If empty, will use all the dofs
    virtual void SetDOFResolutions(const std::vector<dReal>& resolutions, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief \see GetDOFLimits
    virtual void SetDOFLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief Returns the joints making up the controllable degrees of freedom of the body.
    inline const std::vector<JointPtr>& GetJoints() const {
        return _vecjoints;
    }

    /** \brief Returns the passive joints, order does not matter.

        A passive joint is not directly controlled by the body's degrees of freedom so it has no
        joint index and no dof index. Passive joints allows mimic joints to be hidden from the users.
        However, there are cases when passive joints are not mimic; for example, suspension mechanism on vehicles.
     */
    inline const std::vector<JointPtr>& GetPassiveJoints() const {
        return _vPassiveJoints;
    }

    /// \brief Returns the joints in hierarchical order starting at the base link.
    ///
    /// In the case of closed loops, the joints are returned in the order closest to the root.
    /// All the joints affecting a particular joint's transformation will always come before the joint in the list.
    const std::vector<JointPtr>& GetDependencyOrderedJoints() const;

    /// \brief Returns all active and passive joints in hierarchical order starting at the base link.
    ///
    /// In the case of closed loops, the joints are returned in the order closest to the root.
    /// All the joints affecting a particular joint's transformation will always come before the joint in the list.
    const std::vector<JointPtr>& GetDependencyOrderedJointsAll() const;

    /** \brief Return the set of unique closed loops of the kinematics hierarchy.

        Each loop is a set of link indices and joint indices. For example, a loop of link indices:
        [l_0,l_1,l_2] will consist of three joints connecting l_0 to l_1, l_1 to l_2, and l_2 to l_0.
        The first element in the pair is the link l_X, the second element in the joint connecting l_X to l_(X+1).
     */
    const std::vector< std::vector< std::pair<LinkPtr, JointPtr> > >& GetClosedLoops() const;

    /** \en \brief Computes the minimal chain of joints that are between two links in the order of linkindex1 to linkindex2

        Passive joints are also used in the computation of the chain and can be returned.
        Note that a passive joint has a joint index and dof index of -1.
        \param[in] linkindex1 the link index to start the search
        \param[in] linkindex2 the link index where the search ends
        \param[out] vjoints the joints to fill that describe the chain
        \return true if the two links are connected (vjoints will be filled), false if the links are separate

        \ja \brief 2つのリンクを繋ぐ関節の最短経路を計算する．

        受動的な関節は，位置関係が固定されているリンクを見つけるために調べられている
        受動的な関節も返される可能があるから，注意する必要があります．
        \param[in] linkindex1 始点リンクインデックス
        \param[in] linkindex2 終点リンクインデックス
        \param[out] vjoints　関節の経路
        \return 経路が存在している場合，trueを返す．
     */
    bool GetChain(int linkindex1, int linkindex2, std::vector<JointPtr>& vjoints) const;

    /// \brief similar to \ref GetChain(int,int,std::vector<JointPtr>&) except returns the links along the path.
    bool GetChain(int linkindex1, int linkindex2, std::vector<LinkPtr>& vlinks) const;

    /// \brief Returns true if the dof index affects the relative transformation between the two links.
    ///
    /// The internal implementation uses \ref KinBody::DoesAffect, therefore mimic indices are correctly handled.
    /// \param[in] linkindex1 the link index to start the search
    /// \param[in] linkindex2 the link index where the search ends
    bool IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const;

    /// \brief Return the index of the joint with the given name, else -1.
    int GetJointIndex(const std::string& name) const;

    /// \brief Return a pointer to the joint with the given name. Search in the regular and passive joints.
    JointPtr GetJoint(const std::string& name) const;

    /// \brief Returns the joint that covers the degree of freedom index.
    ///
    /// Note that the mapping of joint structures is not the same as the values in GetJointValues since each joint can have more than one degree of freedom.
    inline const JointPtr& GetJointFromDOFIndex(int dofindex) const {
        return _vecjoints.at(_vDOFIndices.at(dofindex));
    }

    //@}

    /// \brief Computes the configuration difference values1-values2 and stores it in values1.
    ///
    /// Takes into account joint limits and wrapping of circular joints.
    /// \param[inout] values1 the result is stored back in this
    /// \param[in] values2
    /// \param dofindices the dof indices to compute the subtraction for. If empty, will compute for all the dofs
    virtual void SubtractDOFValues(std::vector<dReal>& values1, const std::vector<dReal>& values2, const std::vector<int>& dofindices=std::vector<int>()) const;

    /// \brief Adds a torque to every joint.
    ///
    /// \param bAdd if true, adds to previous torques, otherwise resets the torques on all bodies and starts from 0
    virtual void SetDOFTorques(const std::vector<dReal>& torques, bool add);

    /// \brief Returns all the rigid links of the body.
    const std::vector<LinkPtr>& GetLinks() const {
        return _veclinks;
    }

    /// \brief returns true if the DOF describes a rotation around an axis.
    ///
    /// \param dofindex the degree of freedom index
    bool IsDOFRevolute(int dofindex) const;

    /// \brief returns true if the DOF describes a translation around an axis.
    ///
    /// \param dofindex the degree of freedom index
    bool IsDOFPrismatic(int dofindex) const;

    /// \brief return a pointer to the link with the given name
    LinkPtr GetLink(const std::string& linkname) const;

    LinkPtr GetLink(const string_view linkname) const;

    LinkPtr GetLink(const char* plinkname) const;

    /// \brief return the link index that matches the name. -1 if did not find
    int GetLinkIndex(const string_view name) const;

    /// Updates the bounding box and any other parameters that could have changed by a simulation step
    virtual void SimulationStep(dReal fElapsedTime);

    /// \brief get the transformations of all the links at once
    void GetLinkTransformations(std::vector<Transform>& transforms) const;

    /// \brief get the transformations of all the links and the dof branches at once.
    ///
    /// Knowing the dof branches allows the robot to recover the full state of the joints with SetLinkTransformations
    void GetLinkTransformations(std::vector<Transform>& transforms, std::vector<dReal>& doflastsetvalues) const;

    /// \brief gets the enable states of all links
    void GetLinkEnableStates(std::vector<uint8_t>& enablestates) const;

    /// \brief notifies link is enabled / disabled. Should be called from Link class to keep internal cache of KinBody consistent with that of LinkInfo.
    void NotifyLinkEnabled(size_t linkIndex, bool bEnable);

    /// \brief gets a vector of masks of the link enable states.
    ///
    inline const std::vector<uint64_t>& GetLinkEnableStatesMasks() const
    {
        return _vLinkEnableStatesMask;
    }

    /// queries the transfromation of the first link of the body
    inline const Transform GetTransform() const {
        return _veclinks.size() > 0 ? _veclinks.front()->GetTransform() * _invBaseLinkInBodyTransform : Transform();
    }

    /// \brief Set the velocity of the base link, rest of links are set to a consistent velocity so entire robot moves correctly.
    /// \param linearvel linear velocity
    /// \param angularvel is the rotation axis * angular speed
    virtual bool SetVelocity(const Vector& linearvel, const Vector& angularvel);

    /** \brief Sets the velocity of the base link and each of the joints.

        Computes internally what the correponding velocities of each of the links should be in order to
        achieve consistent results with the joint velocities. Sends the velocities to the physics engine.
        Velocities correspond to the link's coordinate system origin.
        \param[in] linearvel linear velocity of base link
        \param[in] angularvel angular velocity rotation_axis*theta_dot
        \param[in] dofvelocities - velocities of each of the degrees of freeom
        \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint velocity limits before setting the values and clamp them.
     */
    virtual void SetDOFVelocities(const std::vector<dReal>& dofvelocities, const Vector& linearvel, const Vector& angularvel,uint32_t checklimits = CLA_CheckLimits);

    /// \brief Sets the velocity of the joints.
    ///
    /// Copies the current velocity of the base link and calls SetDOFVelocities(linearvel,angularvel,vDOFVelocities)
    /// \param[in] dofvelocities - velocities of each of the degrees of freeom
    /// \param[in] checklimits if >0, will excplicitly check the joint velocity limits before setting the values and clamp them. If == 1, then will warn if the limits are overboard, if == 2, then will not warn (used for code that knows it's giving bad values)
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void SetDOFVelocities(const std::vector<dReal>& dofvelocities, uint32_t checklimits = CLA_CheckLimits, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief Returns the linear and angular velocities for each link
    ///
    /// \param[out] velocities The velocities of the link frames with respect to the world coordinate system are returned.
    void GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const;

    /// \brief link index and the linear and angular accelerations of the link. First is linear acceleration of the link's coordinate system, and second is the angular acceleration of this coordinate system.
    typedef std::map<int, std::pair<Vector,Vector> > AccelerationMap;
    typedef boost::shared_ptr<AccelerationMap> AccelerationMapPtr;
    typedef boost::shared_ptr<AccelerationMap const> AccelerationMapConstPtr;

    /** \brief Returns the linear and angular accelerations for each link given the dof accelerations

        Computes accelerations of the link frames with respect to the world coordinate system are returned.
        The base angular velocity is used when computing accelerations.
        If externalaccelerations is null, the gravity vector from the physics engine is used as the negative acceleration of the base link.
        The derivate is taken with respect to the world origin fixed in space (also known as spatial acceleration).
        The current angles and velocities set on the robot are used.
        Note that this function calls the internal _ComputeLinkAccelerations function, so for users that are interested in overriding it, override _ComputeLinkAccelerations
        \param[in] dofaccelerations the accelerations of each of the DOF
        \param[out] linkaccelerations the linear and angular accelerations of link (in that order)
        \param[in] externalaccelerations [optional] The external accelerations to add to each link.
     */
    void GetLinkAccelerations(const std::vector<dReal>& dofaccelerations, std::vector<std::pair<Vector,Vector> >& linkaccelerations, AccelerationMapConstPtr externalaccelerations=AccelerationMapConstPtr()) const;

    /** \en \brief set the transform of the first link (the rest of the links are computed based on the joint values).

        \param transform affine transformation

        \ja \brief 胴体の絶対姿勢を設定、残りのリンクは運動学の構造に従って変換される．

        \param transform 変換行列
     */
    virtual void SetTransform(const Transform& transform);

    /// \brief Return an axis-aligned bounding box of the entire object in the world coordinate system.
    ///
    /// \brief bEnabledOnlyLinks if true, will only count links that are enabled. By default this is false
    AABB ComputeAABB(bool bEnabledOnlyLinks=false) const;

    /// \brief returns an axis-aligned bounding box when body has transform tBody.
    ///
    /// AABB equivalent to SetTransform(tBody); aabb = ComptueAABB(bEnabledOnlyLinks).
    /// \brief tBody the transform to put this kinbody in when computing the AABB
    /// \brief bEnabledOnlyLinks if true, will only count links that are enabled. By default this is false
    AABB ComputeAABBFromTransform(const Transform& tBody, bool bEnabledOnlyLinks=false) const;

    /// \brief returns an axis-aligned bounding box of the current body transform with respect to a rotation quatWorld.
    ///
    /// \brief quaterion of the orientation of the box in the world coordinate system.
    /// \brief bEnabledOnlyLinks if true, will only count links that are enabled. By default this is false
    /// \return an oriented bounding box whose orientation is quatWorld
    OrientedBox ComputeOBBOnAxes(const Vector& quatWorld, bool bEnabledOnlyLinks=false) const;

    /// \brief returns an axis-aligned bounding box when body has identity transform
    ///
    /// Internally equivalent to ComputeAABBFromTransform(Transform(), ...)
    AABB ComputeLocalAABB(bool bEnabledOnlyLinks=false) const;

    /// \brief Return the axis-aligned bounding box of the specified geometries of the kinbody in the world coordinate system. If any of the links does not have the geometry group geomgroupname, will throw an exception.
    AABB ComputeAABBForGeometryGroup(const std::string& geomgroupname, bool bEnabledOnlyLinks=false) const;
    AABB ComputeLocalAABBForGeometryGroup(const std::string& geomgroupname, bool bEnabledOnlyLinks=false) const;
    AABB ComputeAABBForGeometryGroupFromTransform(const std::string& geomgroupname, const Transform& tBody, bool bEnabledOnlyLinks=false) const;

    /// \brief Returns the total mass of entire body.
    dReal GetMass() const;

    /// \brief Return the center of mass of entire body in the world coordinate system.
    Vector GetCenterOfMass() const;

    /// \brief Enables or disables all the links.
    void Enable(bool enable);

    /// \return true if any link of the KinBody is enabled
    bool IsEnabled() const;

    /// \brief Sets all the links as visible or not visible.
    ///
    /// \return true if changed
    bool SetVisible(bool visible);

    /// \return true if any link of the KinBody is visible.
    bool IsVisible() const;

    /// \brief Sets the joint values of the robot.
    ///
    /// \param[in] values the values to set the joint angles (ordered by the dof indices)
    /// \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint limits before setting the values and clamp them.
    /// \param[in] dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void SetDOFValues(const std::vector<dReal>& values, uint32_t checklimits = CLA_CheckLimits, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief Sets the joint values of the robot.
    ///
    /// \param[in] pJointValues pointer to head of array that holds joint angles (ordered by the dof indices)
    /// \param[in] dof number of dof described by array of pJointValues.
    /// \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint limits before setting the values and clamp them.
    /// \param[in] dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void SetDOFValues(const dReal* pJointValues, int dof, uint32_t checklimits = CLA_CheckLimits, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief Sets the joint values and transformation of the body.
    ///
    /// \param[in] values the values to set the joint angles (ordered by the dof indices)
    /// \param[in] transform represents the transformation of the first body.
    /// \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint limits before setting the values and clamp them.
    virtual void SetDOFValues(const std::vector<dReal>& values, const Transform& transform, uint32_t checklimits = CLA_CheckLimits);

    virtual void SetJointValues(const std::vector<dReal>& values, const Transform& transform, bool checklimits = true)
    {
        SetDOFValues(values,transform,static_cast<uint32_t>(checklimits));
    }

    /// \brief sets the transformations of all the links at once
    virtual void SetLinkTransformations(const std::vector<Transform>& transforms);

    /// \brief sets the transformations of all the links and dof branches at once.
    ///
    /// Using dof branches allows the full joint state to be recovered
    virtual void SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<dReal>& doflastsetvalues);

    /// \brief sets the link velocities
    virtual void SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities);

    /// \brief sets the link enable states
    virtual void SetLinkEnableStates(const std::vector<uint8_t>& enablestates);

    /// \brief Computes the translation jacobian with respect to a world position.
    ///
    /// Gets the jacobian with respect to a link by computing the partial differentials for all joints that in the path from the root node to GetLinks()[index]
    /// (doesn't touch the rest of the values)
    /// \param linkindex of the link that defines the frame the position is attached to
    /// \param position position in world space where to compute derivatives from.
    /// \param jacobian 3xDOF matrix
    /// \param dofindices the dof indices to compute the jacobian for. If empty, will compute for all the dofs
    virtual void ComputeJacobianTranslation(const int linkindex, const Vector& position, std::vector<dReal>& jacobian, const std::vector<int>& dofindices = {}) const;

    /// \brief calls std::vector version of ComputeJacobian internally
    virtual void CalculateJacobian(const int linkindex, const Vector& position, std::vector<dReal>& jacobian) const;

    /// \brief calls std::vector version of ComputeJacobian internally, a little inefficient since it copies memory
    virtual void CalculateJacobian(const int linkindex, const Vector& position, boost::multi_array<dReal, 2>& jacobian) const;

    /// \brief Computes the rotational jacobian as a quaternion with respect to an initial rotation.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param qInitialRot the rotation in world space whose derivative to take from.
    /// \param jacobian 4xDOF matrix
    virtual void CalculateRotationJacobian(const int linkindex, const Vector& quat, std::vector<dReal>& jacobian) const;

    /// \brief calls std::vector version of CalculateRotationJacobian internally, a little inefficient since it copies memory
    virtual void CalculateRotationJacobian(const int linkindex, const Vector& quat, boost::multi_array<dReal, 2>& jacobian) const;

    /// \brief Computes the angular velocity jacobian of a specified link about the axes of world coordinates.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param vjacobian 3xDOF matrix
    virtual void ComputeJacobianAxisAngle(const int linkindex, std::vector<dReal>& jacobian, const std::vector<int>& dofindices = {}) const;

    /// \brief Computes the angular velocity jacobian of a specified link about the axes of world coordinates.
    virtual void CalculateAngularVelocityJacobian(const int linkindex, std::vector<dReal>& jacobian) const;

    /// \brief calls std::vector version of CalculateAngularVelocityJacobian internally, a little inefficient since it copies memory
    virtual void CalculateAngularVelocityJacobian(const int linkindex, boost::multi_array<dReal, 2>& jacobian) const;

    /** \brief Computes the DOFx3xDOF hessian of the linear translation

        Arjang Hourtash. "The Kinematic Hessian and Higher Derivatives", IEEE Symposium on Computational Intelligence in Robotics and Automation (CIRA), 2005.

        Can be used to find the world position acceleration
        \code
        accel = Jacobian * dofaccelerations + dofvelocities^T * Hessian * dofvelocities
        \endcode

        It can also be used for a second-order approximation of the position given delta dof values
        \code
        newposition = position + Jacobian * delta + 0.5 * delta^T * Hessian * delta
        \endcode

        H[i,j.k] = hessian[k+DOF*(j+3*i)]
        delta[j] = sum_i sum_k values[i] * H[i,j,k] * values[k]

        \param linkindex of the link that defines the frame the position is attached to
        \param position position in world space where to compute derivatives from.
        \param hessian DOFx3xDOF matrix such that numpy.dot(dq,numpy.dot(hessian,dq)) is the expected second-order delta translation
        \param dofindices the dof indices to compute the hessian for. If empty, will compute for all the dofs
     */
    virtual void ComputeHessianTranslation(int linkindex, const Vector& position, std::vector<dReal>& hessian, const std::vector<int>& dofindices=std::vector<int>()) const;

    /** \brief Computes the DOFx3xDOF hessian of the rotation represented as angle-axis

        Arjang Hourtash. "The Kinematic Hessian and Higher Derivatives", IEEE Symposium on Computational Intelligence in Robotics and Automation (CIRA), 2005.

        Can be used to find the world axis-angle acceleration
        \code
        accel = Jacobian * dofaccelerations + dofvelocities^T * Hessian * dofvelocities
        \endcode

        It can also be used for a second-order approximation of the axis-angle given delta dof values
        \code
        newaxisangle = axisangle + Jacobian * delta + 0.5 * delta^T * Hessian * delta
        \endcode

        H[i,j.k] = hessian[k+DOF*(j+3*i)]
        delta[j] = sum_i sum_k values[i] * H[i,j,k] * values[k]

        \param linkindex of the link that defines the frame the position is attached to
        \param hessian DOFx3xDOF matrix such that numpy.dot(dq,numpy.dot(hessian,dq)) is the expected second-order delta angle-axis
        \param dofindices the dof indices to compute the hessian for. If empty, will compute for all the dofs
     */
    virtual void ComputeHessianAxisAngle(int linkindex, std::vector<dReal>& hessian, const std::vector<int>& dofindices=std::vector<int>()) const;

    /// \brief link index and the linear forces and torques. Value.first is linear force acting on the link's COM and Value.second is torque
    typedef std::map<int, std::pair<Vector,Vector> > ForceTorqueMap;

    /** \brief Computes the inverse dynamics (torques) from the current robot position, velocity, and acceleration.

        The dof values are ready from GetDOFValues() and GetDOFVelocities(). Because openrave does not have a state for robot acceleration,
        it has to be inserted as a parameter to this function. Acceleration due to gravitation is extracted from GetEnv()->GetPhysicsEngine()->GetGravity().
        The method uses Recursive Newton Euler algorithm from  Walker Orin and Corke.
        \param[out] doftorques The output torques.
        \param[in] dofaccelerations The dof accelerations of the current robot state. If the size is 0, assumes all accelerations are 0 (this should be faster)
        \param[in] externalforcetorque [optional] Specifies all the external forces/torques acting on the links at their center of mass.
     */
    virtual void ComputeInverseDynamics(std::vector<dReal>& doftorques, const std::vector<dReal>& dofaccelerations, const ForceTorqueMap& externalforcetorque=ForceTorqueMap()) const;

    /** \brief Computes the separated inverse dynamics torque terms from the current robot position, velocity, and acceleration.

        torques = M(dofvalues) * dofaccel + C(dofvalues,dofvel) * dofvel + G(dofvalues)

        Where
        torques - generalized forces associated with dofvalues
        M - manipulator inertia tensor (symmetric joint-space inertia)
        C - coriolis and centripetal effects
        G - gravity loading + external forces due to externalforcetorque + base link angular acceleration contribution

        The dof values are ready from GetDOFValues() and GetDOFVelocities(). Because openrave does not have a state for robot acceleration,
        it has to be inserted as a parameter to this function. Acceleration due to gravitation is extracted from GetEnv()->GetPhysicsEngine()->GetGravity().
        The method uses Recursive Newton Euler algorithm from  Walker Orin and Corke.
        \param[out] doftorquecomponents A set of 3 torques [M(dofvalues) * dofaccel, C(dofvalues,dofvel) * dofvel, G(dofvalues)]
        \param[in] dofaccelerations The dof accelerations of the current robot state. If the size is 0, assumes all accelerations are 0 (this should be faster)
        \param[in] externalforcetorque [optional] Specifies all the external forces/torques acting on the links at their center of mass.
     */
    virtual void ComputeInverseDynamics(boost::array< std::vector<dReal>, 3>& doftorquecomponents, const std::vector<dReal>& dofaccelerations, const ForceTorqueMap& externalforcetorque=ForceTorqueMap()) const;

    /** \brief Computes dynamic limits for acceleration and jerks, which are dynamically changing based on the given positions and velocities of the robot.

        Since not all robots supports dynamic limits, so this function should be overriden in the subclass.
        \param[out] vDynamicAccelerationLimits, vDynamicJerkLimits : dynamically changing acceleration limits and jerk limits. the sizes of vectors are in the DOF order. 0 means there is no dynamic limit
        \param[in] vDOFValues, vDOFPositions : DOF positions and velocities to compute dynamic limits. note that if these are related to finite difference, need to input [k-1] timestamp and the returned limits are at [k] timestamp. the sizes of the vectors are the DOF the robot.
        \return true if the robot has dynamic limits.
     */
    virtual bool GetDOFDynamicAccelerationJerkLimits(std::vector<dReal>& vDynamicAccelerationLimits, std::vector<dReal>& vDynamicJerkLimits,
                                                     const std::vector<dReal>& vDOFValues, const std::vector<dReal>& vDOFVelocities) const;

    /// \brief sets a self-collision checker to be used whenever \ref CheckSelfCollision is called
    ///
    /// This function allows self-collisions to use a different, un-padded geometry for self-collisions
    /// \param collisionchecker The new collision checker to use. If empty, will use the environment set collision checker.
    virtual void SetSelfCollisionChecker(CollisionCheckerBasePtr collisionchecker);

    /// \brief Returns the self-collision checker set specifically for this robot. If none has been set, return empty.
    virtual CollisionCheckerBasePtr GetSelfCollisionChecker() const;

    /// Collision checking utilities that use internal structures of the kinbody like grabbed info or the self-collision checker.
    /// @name Collision Checking Utilities
    //@{

    /** \brief Check if body is self colliding with its links or its grabbed bodies.

        Links that are joined together are ignored.
        Collisions between grabbed bodies are also considered as self-collisions for this body.
        \param report [optional] collision report
        \param collisionchecker An option collision checker to use for checking self-collisions. If not specified, then will use the environment collision checker.
     */
    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr(), CollisionCheckerBasePtr collisionchecker=CollisionCheckerBasePtr()) const;

    /** \brief checks collision of a robot link with the surrounding environment using a new transform. Attached/Grabbed bodies to this link are also checked for collision. Rigidly attached links to the specified link are not checked for collision.

       \param[in] ilinkindex the index of the link to check
       \param[in] tlinktrans The transform of the link to check
       \param[out] report [optional] collision report
     */
    virtual bool CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report = CollisionReportPtr());

    /** \brief checks collision of a robot link with the surrounding environment using the current link's transform. Attached/Grabbed bodies to this link are also checked for collision. Rigidly attached links to the specified link are not checked for collision.

        \param[in] ilinkindex the index of the link to check
        \param[out] report [optional] collision report
     */
    virtual bool CheckLinkCollision(int ilinkindex, CollisionReportPtr report = CollisionReportPtr());

    /** \brief checks collision of a robot link with a specified body using a new transform. Attached/Grabbed bodies to this link are also checked for collision. Rigidly attached links to the specified link are not checked for collision.

       \param[in] ilinkindex the index of the link to check
       \param[in] tlinktrans The transform of the link to check
       \param[in] pbody the body to check
       \param[out] report [optional] collision report
     */
    virtual bool CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr());

    /** \brief checks collision of a robot link with a specified body using the current link's transform. Attached/Grabbed bodies to this link are also checked for collision. Rigidly attached links to the specified link are not checked for collision.

        \param[in] ilinkindex the index of the link to check
        \param[in] pbody the body to check
        \param[out] report [optional] collision report
     */
    virtual bool CheckLinkCollision(int ilinkindex, KinBodyConstPtr pbody, CollisionReportPtr report = CollisionReportPtr());

    /** \brief checks self-collision of a robot link with the other robot links. Attached/Grabbed bodies to this link are also checked for self-collision. Rigidly attached links to the specified link are not checked for self-collision.

        \param[in] ilinkindex the index of the link to check
        \param[out] report [optional] collision report
     */
    virtual bool CheckLinkSelfCollision(int ilinkindex, CollisionReportPtr report = CollisionReportPtr());

    /** \brief checks self-collision of a robot link with the other robot links. Attached/Grabbed bodies to this link are also checked for self-collision. Rigidly attached links to the specified link are not checked for self-collision.

        \param[in] ilinkindex the index of the link to check
        \param[in] tlinktrans The transform of the link to check
        \param[out] report [optional] collision report
     */
    virtual bool CheckLinkSelfCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report = CollisionReportPtr());

    //@}

    /// \return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    bool IsAttached(KinBodyConstPtr body) const RAVE_DEPRECATED {
        return IsAttached(*body);
    }
    bool IsAttached(const KinBody &body) const;

    /// \brief Recursively get all attached bodies of this body, including this body.
    ///
    /// \param[out] setAttached fills with the attached bodies.
    void GetAttached(std::set<KinBodyPtr>& setAttached) const;
    void GetAttached(std::set<KinBodyConstPtr>& setAttached) const;

    /// \brief Recursively get all attached bodies of this body, including this body.
    ///
    /// \param[out] vAttached fills with the attached bodies in ascending order of environment body index.
    void GetAttached(std::vector<KinBodyPtr>& vAttached) const;
    void GetAttached(std::vector<KinBodyConstPtr>& vAttached) const;

    /// \brief Recursively get all attached bodies of this body, including this body.
    ///
    /// \param[out] vAttached fills with the environment body index of attached bodies sorted in ascending order.
    void GetAttachedEnvironmentBodyIndices(std::vector<int>& vAttached) const;

    /// \brief Non-recursively get all bodies that are directly attached to this body.
    /// No guarantees are made about the ordering of the returned bodies.
    ///
    /// \param[out] vBodies filled with all bodies that are directly attached to this body
    void GetDirectlyAttachedBodies(std::vector<KinBodyPtr>& vBodies) const;

    /// \brief return true if there are attached bodies. Used in place of GetAttached for quicker computation.
    inline bool HasAttached() const {
        return _listAttachedBodies.size() > 0;
    }

    /// \brief Return true if this body is derived from RobotBase.
    virtual bool IsRobot() const {
        return false;
    }

    /// \brief return a unique id of the body used in the environment.
    ///
    /// If object is not added to the environment, this will return 0. So checking if GetEnvironmentBodyIndex() is 0 is a good way to check if object is present in the environment.
    /// This id will not be copied when cloning in order to respect another environment's ids.
    inline int GetEnvironmentBodyIndex() const {
        return _environmentBodyIndex;
    }

    /// \brief return a unique id of the body used in the environment.
    ///
    /// deprecated, GetEnvironmentBodyIndex should be used
    int GetEnvironmentId() const RAVE_DEPRECATED {
        return _environmentBodyIndex;
    }

    /** \brief Returns a nonzero value if the joint index effects the link transformation.

        In closed loops, all joints on all paths to the root link are counted as affecting the link.
        If a mimic joint affects the link, then all the joints used in the mimic joint's computation affect the link.
        If negative, the partial derivative of the Jacobian should be negated.
        \param jointindex index of the joint
        \param linkindex index of the link
     */
    int8_t DoesAffect(int jointindex, int linkindex) const;

    /** \brief Returns a nonzero value if the dof index effects the link transformation.

        In closed loops, all joints on all paths to the root link are counted as affecting the link.
        If a mimic joint affects the link, then all the joints used in the mimic joint's computation affect the link.
        If negative, the partial derivative of the Jacobian should be negated.
        \param dofindex index of DOF as returned from \ref GetDOFValues
        \param linkindex index of the link
     */
    int8_t DoesDOFAffectLink(int dofindex, int linkindex) const;

    /// \brief specifies the type of adjacent link information to receive
    enum AdjacentOptions
    {
        AO_Enabled = 1,     ///< return only enabled link pairs
        AO_ActiveDOFs = 2,     ///< return only link pairs that have an active in its path
    };

    /// \brief return all possible link pairs that could get in collision.
    /// \param adjacentoptions a bitmask of \ref AdjacentOptions values
    virtual const std::vector<int>& GetNonAdjacentLinks(int adjacentoptions=0) const;

    /// \brief adds the pair of links to the adjacency list. This is
    void SetAdjacentLinks(int linkindex0, int linkindex1);

    /// \brief return if two links are adjacent links are not.
    bool AreAdjacentLinks(int linkindex0, int linkindex1) const;

    /// \brief adds the pair of links to the adjacency list.
    ///
    /// \param linkIndices vector containing pair of link indies. Each pair of is set as adjacent links. For each pair, first and second must be different.
    void SetAdjacentLinksPairs(const std::vector<std::pair<int, int> >& linkIndices);

    /// \brief adds the pair of links to the adjacency list.
    ///
    /// \param linkIndices vector of link index. Each combination among them is set as adjacent links. elements have to be unique
    void SetAdjacentLinksCombinations(const std::vector<int>& linkIndices);

    inline ManageDataPtr GetManageData() const {
        return _pManageData;
    }

    /// \brief Return a unique id for every transformation state change of any link. Used to check if robot state has changed.
    ///
    /// The stamp is used by the collision checkers, physics engines, or any other item
    /// that needs to keep track of any changes of the KinBody as it moves.
    /// Currently stamps monotonically increment for every transformation/joint angle change.
    inline int GetUpdateStamp() const {
        return _nUpdateStampId;
    }

    /// \brief Increments the unique id that indicates the number of transformation state changes of any link. Used to check if robot state has changed.
    void IncrementUpdateStamp(const int inc=1) {
        _nUpdateStampId += inc;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions) override;

    /// \brief Register a callback with the interface.
    ///
    /// Everytime a static property of the interface changes, all
    /// registered callbacks are called to update the users of the changes. Note that the callbacks will
    /// block the thread that made the parameter change.
    /// \param callback
    /// \param properties a mask of the \ref KinBodyProperty values that the callback should be called for when they change
    virtual UserDataPtr RegisterChangeCallback(uint32_t properties, const boost::function<void()>& callback) const;

    void Serialize(BaseXMLWriterPtr writer, int options=0) const override;

    /// \brief A md5 hash unique to the particular kinematic and geometric structure of a KinBody.
    ///
    /// This 32 byte string can be used to check if two bodies have the same kinematic structure and can be used
    /// to index into tables when looking for body-specific models. OpenRAVE stores all
    /// such models in the OPENRAVE_HOME directory (usually ~/.openrave), indexed by the particular robot/body hashes.
    /// \return md5 hash string of kinematics/geometry
    virtual const std::string& GetKinematicsGeometryHash() const;

    /// \brief Sets the joint offsets so that the current configuration becomes the new zero state of the robot.
    ///
    /// When this function returns, the returned DOF values should be all zero for controllable joints.
    /// Mimic equations will use the new offsetted values when computing their joints.
    /// This is primarily used for calibrating a robot's zero position
    virtual void SetZeroConfiguration();

    /// \brief Treats the current pose as a pose not in collision, which sets the adjacent pairs of links
    virtual void SetNonCollidingConfiguration();

    /// Functions dealing with configuration specifications
    /// @name Configuration Specification API
    //@{

    /// \brief return the configuration specification of the joint values and transform
    ///
    /// Note that the return type is by-value, so should not be used in iteration
    virtual ConfigurationSpecification GetConfigurationSpecification(const std::string& interpolation="") const;

    /// \brief return the configuration specification of the specified joint indices.
    ///
    /// Note that the return type is by-value, so should not be used in iteration
    virtual ConfigurationSpecification GetConfigurationSpecificationIndices(const std::vector<int>& indices, const std::string& interpolation="") const;

    /// \brief sets joint values and transform of the body using configuration values as specified by \ref GetConfigurationSpecification()
    ///
    /// \param itvalues the iterator to the vector containing the dof values. Must have GetConfigurationSpecification().GetDOF() values!
    /// \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint limits before setting the values and clamp them.
    virtual void SetConfigurationValues(std::vector<dReal>::const_iterator itvalues, uint32_t checklimits = CLA_CheckLimits);

    /// \brief returns the configuration values as specified by \ref GetConfigurationSpecification()
    virtual void GetConfigurationValues(std::vector<dReal>& v) const;

    //@}

    /** A grabbed body becomes part of the body and its relative pose with respect to a body's
        link will be fixed. KinBody::_AttachBody is called for every grabbed body in order to make
        the grabbed body a part of the body. Once grabbed, the inter-collisions between the grabbing body
        and the grabbed body are regarded as self-collisions; any outside collisions of the grabbed body and the
        environment are regarded as environment collisions with the grabbing body.
        @name Grabbing Bodies
        @{
     */

    /** \brief Grab the body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pBodyLinkToGrabWith the link of this body that will perform the grab
        \param[in] setBodyLinksToIgnore Additional body link indices that collision checker ignore
        when checking collisions between the grabbed body and the body.
        \param[in] rGrabbedUserData custom data to keep in the body
        \return true if successful and body is grabbed.
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pBodyLinkToGrabWith, const std::set<int>& setBodyLinksToIgnore, const rapidjson::Value& rGrabbedUserData);

    /** \brief Grab the body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pBodyLinkToGrabWith the link of this body that will perform the grab
        \param[in] setBodyLinksToIgnore Additional body link names that collision checker ignore
        when checking collisions between the grabbed body and the body.
        \param[in] rGrabbedUserData custom data to keep in the body
        \return true if successful and body is grabbed.
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pBodyLinkToGrabWith, const std::set<std::string>& setIgnoreBodyLinkNames, const rapidjson::Value& rGrabbedUserData);

    /** \brief Grab a body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pBodyLinkToGrabWith the link of this body that will perform the grab
        \param[in] rGrabbedUserData custom data to keep in the body
        \return true if successful and body is grabbed/
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pBodyLinkToGrabWith, const rapidjson::Value& rGrabbedUserData);

    /** \brief Release the body if grabbed.

        \param body body to release
     */
    void Release(KinBodyPtr body) RAVE_DEPRECATED {
        Release(*body);
    }
    virtual void Release(KinBody &body);

    /// Release all grabbed bodies.
    virtual void ReleaseAllGrabbed();     ///< release all bodies

    void ReleaseAllGrabbedWithLink(LinkPtr pBodyLinkToGrabWith) {
        ReleaseAllGrabbedWithLink(*pBodyLinkToGrabWith);
    }
    virtual void ReleaseAllGrabbedWithLink(const KinBody::Link& bodyLinkToGrabWith);

    /** \brief Releases and grabs all bodies, has the effect of recalculating all the initial collision with the bodies.

        This has the effect of resetting the current collisions any grabbed body makes with the body into an ignore list.
     */
    virtual void RegrabAll();

    /** \brief return the body link that is currently grabbing the body. If the body is not grabbed, will return an  empty pointer.

        \param[in] body the body to check
     */
    LinkPtr IsGrabbing(KinBodyConstPtr body) const RAVE_DEPRECATED {
        return IsGrabbing(*body);
    }
    LinkPtr IsGrabbing(const KinBody &body) const;

    /// \brief Result of CheckGrabbedInfo() call
    enum GrabbedInfoCheckResult
    {
        GICR_Identical = 0, ///< Specified body is grabbed with specified grabbing link (and optionally, specified ignored links)
        GICR_BodyNotGrabbed = 1,  //< Specified body is not grabbed
        GICR_GrabbingLinkNotMatch = 2, ///< Specified body is grabbed, but grabbing link does not match
        GICR_IgnoredLinksNotMatch = 3, ///< Specified body is grabbed and grabbing link matches, but ignored links do not match
        GICR_UserDataNotMatch = 4, ///< Specified body is grabbed, grabbing link matches, and ignored links match, but user data do not match
    };

    /** \brief Checks whether a body is grabbed with the given robot link and the ignored robot links match.
     *  \return One of GrabbedInfoComparisonResult codes. 0 (=GICR_Identical) if all given information match.
     */
    int CheckGrabbedInfo(const KinBody& body, const KinBody::Link& bodyLinkToGrabWith, const std::set<std::string>& setBodyLinksToIgnore, const rapidjson::Value& rGrabbedUserData) const;

    /** \brief gets all grabbed bodies of the body

        \param[out] vbodies filled with the grabbed bodies
     */
    void GetGrabbed(std::vector<KinBodyPtr>& vbodies) const;

    /** \brief get the set of body names that are grabbed by this body

        \param[out] set to fill with the grabbed body names
     */
    void GetGrabbedBodyNames(std::unordered_set<std::string>& bodyNames) const;

    /// \brief returns number of grabbed targets
    int GetNumGrabbed() const {
        return _grabbedBodiesByEnvironmentIndex.size();
    }

    /** \brief gets info of all grabbed bodies

        \param[out] vGrabbedInfos filled with the grabbed info for every grabbed body. The pointers are newly created.
     */
    void GetGrabbedInfo(std::vector<GrabbedInfoPtr>& vgrabbedinfo) const;

    /** \brief gets info of all grabbed bodies

        \param[out] vGrabbedInfos vector containing grabbedInfos of all the grabbed bodies
     */
    void GetGrabbedInfo(std::vector<GrabbedInfo>& vgrabbedinfo) const;

    /** \brief gets the grabbed info of a grabbed object whose name matches grabbedname

        \param[in] grabbedname the name to use for checking
        \param[out] grabbedInfo will be initialized with the grabbed info of the grabbed object specified by "grabbedname"
        \return true if this body is grabbing body with name "grabbedname" and grabbedInfo is initialized
     */
    bool GetGrabbedInfo(const std::string& grabbedname, GrabbedInfo& grabbedInfo) const;

    /** \brief resets the grabbed objects of this body and set them according to the input vGrabbedInfos

        Any currently grabbed bodies will be first released.
        \param[in] vGrabbedInfos vector of GrabbedInfo, filled with the grabbed info for every body to be grabbed
     */
    void ResetGrabbed(const std::vector<GrabbedInfoConstPtr>& vGrabbedInfos);

    /** \brief returns all the links of grabber (this body) that are being ignored (self-collision) by the grabbed body.

        \param[in] body the grabbed body
        \param[out] list of the ignored links
     */
    void GetIgnoredLinksOfGrabbed(KinBodyConstPtr body, std::list<KinBody::LinkConstPtr>& ignorelinks) const;

    inline int64_t GetLastModifiedAtUS() {
        return _lastModifiedAtUS;
    }

    inline void SetLastModifiedAtUS(int64_t lastModifiedAtUS) {
        _lastModifiedAtUS = lastModifiedAtUS;
    }

    inline int64_t GetRevisionId() {
        return _revisionId;
    }

    inline void SetRevisionId(int64_t revisionId) {
        _revisionId = revisionId;
    }

    //@}

    /// only used for hashes...
    virtual void serialize(std::ostream& o, int options) const;

    inline KinBodyPtr shared_kinbody() {
        return boost::static_pointer_cast<KinBody>(shared_from_this());
    }
    inline KinBodyConstPtr shared_kinbody_const() const {
        return boost::static_pointer_cast<KinBody const>(shared_from_this());
    }

    /// \brief similar to GetInfo, but creates a copy of an up-to-date info, safe for caller to manipulate
    ///
    /// if bSkipDOFValues is false, body needs to be added to the environment in order to work.
    /// \param bSkipDOFValues if true, then will skip extracting the DOF values of the kinbody and initializing _dofValues. This allows the fucntion to be called even if the body is not added to the environment.
    virtual void ExtractInfo(KinBodyInfo& info, ExtractInfoOptions options);

    /// \brief update KinBody according to new KinBodyInfo, returns false if update cannot be performed and requires InitFromInfo
    virtual UpdateFromInfoResult UpdateFromKinBodyInfo(const KinBodyInfo& info);

    /// \brief Associate the kinbody's current kinematics geometry hash with a forward kinematics generator
    virtual void SetKinematicsGenerator(KinematicsGeneratorPtr pGenerator);

    /// \brief gets the associated file entries
    inline const boost::shared_ptr<rapidjson::Document>& GetAssociatedFileEntries() const {
        return _prAssociatedFileEntries;
    }

protected:
    using MapGrabbedByEnvironmentIndex = std::unordered_map<int, GrabbedPtr>;

    /// \brief constructors declared protected so that user always goes through environment to create bodies
    KinBody(InterfaceType type, EnvironmentBasePtr penv);

    inline const Joint& _GetJointFromDOFIndex(int dofindex) const {
        return *_vecjoints.at(_vDOFIndices.at(dofindex));
    }

    inline Joint& _GetJointFromDOFIndex(int dofindex) {
        return *_vecjoints.at(_vDOFIndices.at(dofindex));
    }

    void SetManageData(ManageDataPtr pdata) {
        _pManageData = pdata;
    }

    /** \brief Final post-processing stage before a kinematics body can be used.

        This method is called after the body is finished being initialized with data and before being added to the environment. Also builds the hashes. Builds the internal hierarchy and kinematic body hash.

        Avoids making specific calls on the collision checker (like CheckCollision) or physics engine (like simulating velocities/torques) since this information can change depending on the attached plugin.
     */
    virtual void _ComputeInternalInformation();

    /// \brief de-initializes any internal information computed
    virtual void _DeinitializeInternalInformation();

    /// \brief returns the dof velocities and link velocities
    ///
    /// \param[in] usebaselinkvelocity if true, will compute all velocities using the base link velocity. otherwise will assume it is 0
    virtual void _ComputeDOFLinkVelocities(std::vector<dReal>& dofvelocities, std::vector<std::pair<Vector,Vector> >& linkvelocities, bool usebaselinkvelocity=true) const;

    /// \brief Computes accelerations of the links given all the necessary data of the robot. \see GetLinkAccelerations
    ///
    /// for passive joints that are not mimic and are not static, will call Joint::GetVelocities to get their initial velocities (this is state dependent!)
    /// \param dofvelocities if size is 0, will assume all velocities are 0
    /// \param dofaccelerations if size is 0, will assume all accelerations are 0
    /// \param[in] externalaccelerations [optional] The external accelerations to add to each link. When doing inverse dynamics, should set the base link's acceleration to -gravity.
    virtual void _ComputeLinkAccelerations(const std::vector<dReal>& dofvelocities, const std::vector<dReal>& dofaccelerations, const std::vector< std::pair<Vector, Vector> >& linkvelocities, std::vector<std::pair<Vector,Vector> >& linkaccelerations, AccelerationMapConstPtr externalaccelerations=AccelerationMapConstPtr()) const;

    /// \brief Called to notify the body that certain groups of parameters have been changed.
    ///
    /// This function in calls every registers calledback that is tracking the changes. It also
    /// recomputes the hashes if geometry changed.
    virtual void _PostprocessChangedParameters(uint32_t parameters);

    /// \brief Return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    bool _IsAttached(const KinBody &body, std::set<KinBodyConstPtr>& setChecked) const;

    /// \brief Return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    ///
    /// \param visited is a vector indexed by each body's EnvironmentBodyIndex to show whether it was visited or not in the computation. If 0, then not initialized, if 1, then visited and attached, if -1, then visited and not attached
    bool _IsAttached(int otherBodyId, std::vector<int8_t>& vAttachedVisited) const;

    /// \brief finds all attached bodies who should be considered as one during collision (ie one is grabbing the other)
    ///
    /// recursively visits all attached bodies, and sets vAttachedVisited for attached bodies to 1.
    /// \param vAttachedVisited indexed by environmentBodyIndex
    /// \return number of elements attached (excluding initially non-zero elements in vAttachedVisited)
    int _GetNumAttached(std::vector<int8_t>& vAttachedVisited) const;

    /// \brief adds an attached body
    void _AttachBody(KinBodyPtr body);

    /// \brief removes an attached body
    ///
    /// \return true if body was successfully found and removed
    bool _RemoveAttachedBody(KinBody &body);

    /// \brief Update transforms and velocities of the grabbed bodies
    void _UpdateGrabbedBodies();

    /// \brief removes grabbed body. cleans links from the grabbed body in _listNonCollidingLinksWhenGrabbed of other grabbed bodies.
    MapGrabbedByEnvironmentIndex::iterator _RemoveGrabbedBody(MapGrabbedByEnvironmentIndex::iterator itGrabbed);

    /// \brief resets cached information dependent on the collision checker (usually called when the collision checker is switched or some big mode is set.
    virtual void _ResetInternalCollisionCache();

    /// \brief initializes and adds a link to internal hierarchy.
    ///
    /// Assumes plink has _info initialized correctly, so will be initializing the other data depending on it.
    /// Can only be called before internal robot hierarchy is initialized.
    void _InitAndAddLink(LinkPtr plink);

    /// \brief initializes the internal link vector with the provided link infos.
    ///
    /// Internal method called as part of Init() or InitFromLinkInfos()
    /// LinkInfoT may be a LinkInfo struct or a pointer to a LinkInfo
    template <typename LinkInfoT>
    void _InitWithInitialLinks(const std::vector<LinkInfoT>& linkInfos);

    /// Initialize a valid link pointer from the specified link info
    static void _InitLinkFromInfo(KinBody::LinkPtr& linkPtr, const KinBody::LinkInfo& linkInfo);

    /// \brief initializes and adds a link to internal hierarchy.
    ///
    /// Assumes plink has _info initialized correctly, so will be initializing the other data depending on it.
    /// Can only be called before internal robot hierarchy is initialized
    void _InitAndAddJoint(JointPtr pjoint);

    void _SetForcedAdjacentLinks(int linkindex0, int linkindex1);

    void _SetAdjacentLinksInternal(int linkindex0, int linkindex1);

    /// \brief Restore kinbody's states from other kinbody. This is sets bCalledFromClone=true for _RestoreGrabbedBodiesFromSavedData.
    ///        _RestoreGrabbedBodiesFromSavedData with bCalledFromClone=true allows to restore grabbed bodies from one env to another env.
    ///        To do so, it's referring that _environmentBodyIndex is consistent between two envs. Otherwise, we cannnot identify the correct bodies.
    ///        This should be called from Clone where we can assume that _environmentBodyIndex is configured consistent between two envs.
    ///        Please do not call this from other use cases.
    /// \param[in] pOriginalBody : This function restores the states from pOriginalBody to 'this'.
    void _RestoreStateForClone(const KinBodyPtr& pOriginalBody);

    /// \brief Restore kinbody's grabbed bodies information from saved data.
    /// \param[in] savedBody : saved KinBody inside of saver.
    /// \param[in] options : SaveParameters inside of saver.
    /// \param[in] savedGrabbedBodiesByEnvironmentIndex : _grabbedBodiesByEnvironmentIndex held in saver.
    /// \param[in] bCalledFromClone : true this is called from clone, e.g. called from _RestoreGrabbedBodiesForClone. false if  Assumes that this is called from _RestoreKinBody of saver classes.
    void _RestoreGrabbedBodiesFromSavedData(const KinBody& savedBody,
                                            const int options,
                                            const std::unordered_map<int, SavedGrabbedData>& savedGrabbedDataByEnvironmentIndex,
                                            const bool bCalledFromClone = false);

    /// \brief Save this kinbody's information.
    /// \param[out] savedGrabbedDataByEnvironmentIndex : saved information about _grabbedBodiesByEnvironmentIndex.
    void _SaveKinBodySavedGrabbedData(std::unordered_map<int, SavedGrabbedData>& savedGrabbedDataByEnvironmentIndex) const;

    /// Ensures that _vAllPairsShortestPaths is initialized if it is not already
    void _EnsureAllPairsShortestPaths() const;

    /// \brief print the computed _vNonAdjacentLinks contents.
    /// \param[in] vNonAdjacentLinks : from KinBody::_vNonAdjacentLinks
    /// \param[in] nonAdjacentMask : index of element to print, which is mask. _vNonAdjacentLinks[nonAdjacentMask] is printed.
    /// \param[in] envNameId, bodyName : for print message.
    static void _PrintNonAdjacentLinks(const boost::array<std::vector<int>, 4>& vNonAdjacentLinks, const size_t nonAdjacentMask, const std::string& envNameId, const std::string& bodyName);

    std::string _name; ///< name of body

    std::vector<JointPtr> _vecjoints; ///< \see GetJoints
    std::vector<JointPtr> _vTopologicallySortedJoints; ///< \see GetDependencyOrderedJoints
    std::vector<JointPtr> _vTopologicallySortedJointsAll; ///< Similar to _vDependencyOrderedJoints except includes _vecjoints and _vPassiveJoints
    std::vector<int> _vTopologicallySortedJointIndicesAll; ///< the joint indices of the joints in _vTopologicallySortedJointsAll. Passive joint indices have _vecjoints.size() added to them.
    std::vector<JointPtr> _vDOFOrderedJoints; ///< all joints of the body ordered on how they are arranged within the degrees of freedom
    std::vector<LinkPtr> _veclinks; ///< \see GetLinks
    std::vector<int> _vDOFIndices; ///< cached start joint indices, indexed by dof indices
    std::vector<uint64_t> _vLinkEnableStatesMask; /// bit mask containing enabled info of links. If bit 0 of _vLinkEnableBitMap[0] is 1, link 0 is enabled. If bit 2 of _vLinkEnableBitMap[1] is 0, link 66 is disabled. Used for fast access to the LinkInfo::_bIsEnabled

    /// All-pairs shortest paths through the link hierarchy.
    /// The first value describes the parent link index, and the second value is an index into _vecjoints or _vPassiveJoints.
    /// If the second value is greater or equal to  _vecjoints.size() then it indexes into _vPassiveJoints.
    /// Note that this value is lazily calculated - accesses to this variable must be fenced by a call to _EnsureAllPairsShortestPaths
    /// This value is mutable, as due to the lazy calculation it may need to be generated as part of a const method call.
    mutable std::vector<std::pair<int16_t,int16_t> > _vAllPairsShortestPaths;

    std::vector<int8_t> _vJointsAffectingLinks; ///< joint x link: (jointindex*_veclinks.size()+linkindex). entry is non-zero if the joint affects the link in the forward kinematics. If negative, the partial derivative of ds/dtheta should be negated.
    std::vector< std::vector< std::pair<LinkPtr,JointPtr> > > _vClosedLoops; ///< \see GetClosedLoops
    std::vector< std::vector< std::pair<int16_t,int16_t> > > _vClosedLoopIndices; ///< \see GetClosedLoops
    std::vector<JointPtr> _vPassiveJoints; ///< \see GetPassiveJoints()
    std::vector<int8_t> _vAdjacentLinks; ///< a vector of which links are connected to which if link i and j are connected and i < j, then value at (i + j * (j - 1) /2) is 1 where N is the number of links for the body
    std::vector<int8_t> _vForcedAdjacentLinks; ///< internally stores forced adjacent links. \see _vAdjacentLinks for internal representation
    std::list<KinBodyWeakPtr> _listAttachedBodies; ///< list of bodies that are directly attached to this body (can have duplicates)

    std::vector<Transform*> _vLinkTransformPointers; ///< holds a pointers to the Transform Link::_t  in _veclinks. Used for fast access fo the custom kinematics

    /// Map of grabbed body record indexed by the environment index of the grabbed body to provide faster lookup
    /// It is assumed that bodies cannot have their environment index change while they are grabbed.
    MapGrabbedByEnvironmentIndex _grabbedBodiesByEnvironmentIndex;

    mutable std::vector<std::list<UserDataWeakPtr> > _vlistRegisteredCallbacks; ///< callbacks to call when particular properties of the body change. _vlistRegisteredCallbacks[index] is the list of change callbacks where 1<<index is part of KinBodyProperty, this makes it easy to find out if any particular bits have callbacks. The registration/de-registration of the lists can happen at any point and does not modify the kinbody state exposed to the user, hence it is mutable.

    mutable boost::array<std::vector<int>, 4> _vNonAdjacentLinks; ///< contains cached versions of the non-adjacent links depending on values in AdjacentOptions. Declared as mutable since data is cached.
    mutable boost::array<std::set<int>, 4> _cacheSetNonAdjacentLinks; ///< used for caching return value of GetNonAdjacentLinks.
    mutable int _nNonAdjacentLinkCache; ///< specifies what information is currently valid in the AdjacentOptions.  Declared as mutable since data is cached. If 0x80000000 (ie < 0), then everything needs to be recomputed including _setNonAdjacentLinks[0].
    std::vector<Transform> _vInitialLinkTransformations; ///< the initial transformations of each link specifying at least one pose where the robot is collision free

    mutable std::vector<int8_t> _vAttachedVisitedCache; ///< cache
    mutable std::vector<std::pair<Vector,Vector> > _vVelocitiesCache;
    mutable std::vector< boost::array<dReal, 3> > _vPassiveJointValuesCache;
    mutable std::vector< boost::array<dReal, 3> > _vPassiveJointAccelerationsCache;
    mutable std::vector<uint8_t> _vLinksVisitedCache;
    mutable std::vector<dReal> _vTempMimicValues, _vTempMimicValues2, _vTempMimicValues3;


    ConfigurationSpecification _spec;
    CollisionCheckerBasePtr _selfcollisionchecker; ///< optional checker to use for self-collisions

    KinematicsGeneratorPtr _pKinematicsGenerator; ///< holds the generator for kinematics. KinBody calls it everytime its kinematics change
    KinematicsFunctionsPtr _pCurrentKinematicsFunctions; ///< currently generated kinematics functions

    int _environmentBodyIndex; ///< \see GetEnvironmentBodyIndex
    mutable int _nUpdateStampId; ///< \see GetUpdateStamp
    uint32_t _nParametersChanged; ///< set of parameters that changed and need callbacks
    ManageDataPtr _pManageData;
    uint32_t _nHierarchyComputed; ///< 2 if the joint heirarchy and other cached information is computed. 1 if the hierarchy information is computing
    bool _bMakeJoinedLinksAdjacent; ///< if true, then automatically add adjacent links to the adjacency list so that their self-collisions are ignored.
    bool _bAreAllJoints1DOFAndNonCircular; ///< if true, then all controllable joints  of the robot are guaranteed to be either revolute or prismatic and non-circular. This allows certain functions that do operations on the joint values (like SubtractActiveDOFValues) to be optimized without calling Joint functions.

    std::string _id; ///< unique id of the KinBody
    std::string _referenceUri; ///< reference uri saved from InitFromInfo
    boost::shared_ptr<rapidjson::Document> _prAssociatedFileEntries; ///< files tag maintaining entries of data files associated with this object
    Transform _baseLinkInBodyTransform; ///< the transform of the base link in the body coordinate frame. The body transform returned is baselink->GetTransform() * _baseLinkInBodyTransform.inverse(). When setting a transform, the base link transform becomes body->GetTransform() * _baseLinkInBodyTransform
    Transform _invBaseLinkInBodyTransform; ///< _baseLinkInBodyTransform.inverse() for speedup
    mutable std::string __hashKinematicsGeometryDynamics; ///< hash serializing kinematics, dynamics and geometry properties of the KinBody
    int64_t _lastModifiedAtUS=0; ///< us, linux epoch, last modified time of the kinbody when it was originally loaded from the environment.
    int64_t _revisionId = 0; ///< the webstack revision for this loaded kinbody

private:
    mutable std::vector<dReal> _vTempJoints;
    virtual const char* GetHash() const override {
        return OPENRAVE_KINBODY_HASH;
    }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
    friend class OpenRAVEXMLParser::KinBodyXMLReader;
    friend class OpenRAVEXMLParser::JointXMLReader;
    friend class XFileReader;
#else
    friend class ::Environment;
    friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
    friend class ::OpenRAVEXMLParser::JointXMLReader;
    friend class ::XFileReader;
#endif
#endif

    friend class ColladaReader;
    friend class ColladaWriter;
    friend class PhysicsEngineBase;
    friend class CollisionCheckerBase;
    friend class ViewerBase;
    friend class SensorSystemBase;
    friend class RaveDatabase;
    friend class ChangeCallbackData;
    friend class Grabbed;
};

class OPENRAVE_API Grabbed : public UserData, public boost::enable_shared_from_this<Grabbed>
{
public:
    Grabbed(KinBodyPtr pGrabbedBody, KinBody::LinkPtr pGrabbingLink);
    virtual ~Grabbed() {
    }

    /// \brief This function initializes _listNonCollidingLinksWhenGrabbed. First it restores the state of both the
    ///        grabber and the grabbed body to the snapshot when "Grab" function was called. Then it performs the check
    ///        and populates the list.
    ///
    ///        It is important to note that link enable states do not affect the result of this computation. That is,
    ///
    ///        1. if a link L is colliding with _pGrabbedBody at the time of grabbing, L will *not* be in
    ///          _listNonCollidingLinksWhenGrabbed regardless of L's enable state. If L is initially collision-disabled
    ///          when grabbing _pGrabbingBody and later L has become enabled, CheckSelfCollision will ignore
    ///          L-_pGrabbedBody collision (if any).
    ///
    ///        2. if a link L is not colliding with _pGrabbedBody at the time of grabbing, L will be in
    ///           _listNonCollidingLinksWhenGrabbed regardless of L's enable state.
    ///
    ///        Therefore, _listNonCollidingLinksWhenGrabbed has to be computed only once. The computation result remains
    ///        valid until the grabbed body is released.
    void ComputeListNonCollidingLinks();

    inline void SetLinkNonCollidingIsValid(bool bIsValid)
    {
        _listNonCollidingIsValid = bIsValid;
    }

    inline bool IsListNonCollidingLinksValid() const
    {
        return _listNonCollidingIsValid;
    }

    /// \brief Add more links to force to be ignored during grabber's self-collision checking into
    ///        _setGrabberLinkIndicesToIgnore. This function is called when we make the grabber grab the same grabbed body
    ///        more than once with different input links to ignore.
    void AddMoreIgnoreLinks(const std::set<int>& setAdditionalGrabberLinksToIgnore);

    // Member Variables
    KinBodyWeakPtr _pGrabbedBody; ///< the body being grabbed
    KinBody::LinkPtr _pGrabbingLink; ///< the link used for grabbing _pGrabbedBody. Its transform (as well as the transforms of other links rigidly attached to _pGrabbingLink) relative to the grabbed body remains constant until the grabbed body is released.
    std::list<KinBody::LinkConstPtr> _listNonCollidingLinksWhenGrabbed; ///< list of links of the grabber that are not touching the grabbed body *at the time of grabbing*. Since these links are not colliding with the grabbed body at the time of grabbing, they should remain non-colliding with the grabbed body throughout. If, while grabbing, they collide with the grabbed body at some point, CheckSelfCollision should return true. It is important to note that the enable state of a link does *not* affect its membership of this list.
    Transform _tRelative; ///< the relative transform between the grabbed body and the grabbing link. tGrabbingLink*tRelative = tGrabbedBody.
    std::set<int> _setGrabberLinkIndicesToIgnore; ///< indices to the links of the grabber whose collisions with the grabbed bodies should be ignored.
    rapidjson::Document _rGrabbedUserData; ///< user-defined data to be updated when kinbody grabs and releases objects
private:
    bool _listNonCollidingIsValid = false; ///< a flag indicating whether the current _listNonCollidingLinksWhenGrabbed is valid or not.
    std::vector<KinBody::LinkPtr> _vAttachedToGrabbingLink; ///< vector of all links that are rigidly attached to _pGrabbingLink
    KinBody::KinBodyStateSaverPtr _pGrabberSaver; ///< statesaver that saves the snapshot of the grabber at the time Grab is called. The saved state will be used (i.e. restored) temporarily when computation of _listNonCollidingLinksWhenGrabbed is necessary.
    KinBody::KinBodyStateSaverPtr _pGrabbedSaver; ///< statesaver that saves the snapshot of the grabbed at the time Grab is called. The saved state will be used (i.e. restored) temporarily when computation of _listNonCollidingLinksWhenGrabbed is necessary.

}; // end class Grabbed

} // end namespace OpenRAVE

#endif
