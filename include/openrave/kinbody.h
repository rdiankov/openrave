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
/** \file   kinbody.h
    \brief  Kinematics body related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_KINBODY_H
#define OPENRAVE_KINBODY_H

namespace OpenRAVE {

class OpenRAVEFunctionParserReal;
typedef boost::shared_ptr< OpenRAVEFunctionParserReal > OpenRAVEFunctionParserRealPtr;

/// \brief The type of geometry primitive.
enum GeometryType {
    GT_None = 0,
    GT_Box = 1,
    GT_Sphere = 2,
    GT_Cylinder = 3, ///< oriented towards z-axis
    GT_TriMesh = 4,
};

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
        Prop_JointAccelerationVelocityTorqueLimits=0x10,     ///< velocity + acceleration + torque
        Prop_Joints=Prop_JointMimic|Prop_JointLimits|Prop_JointOffset|Prop_JointProperties|Prop_JointAccelerationVelocityTorqueLimits, ///< all properties of all joints

        Prop_Name=0x20,     ///< name changed
        Prop_LinkDraw=0x40,     ///< toggle link geometries rendering
        Prop_LinkGeometry=0x80,     ///< the geometry of the link changed
        // 0x100
        // 0x200
        Prop_LinkStatic=0x400,     ///< static property of link changed
        Prop_LinkEnable=0x800,     ///< enable property of link changed
        Prop_LinkDynamics=0x1000,     ///< mass/inertia properties of link changed
        Prop_Links=Prop_LinkDraw|Prop_LinkGeometry|Prop_LinkStatic|Prop_LinkEnable|Prop_LinkDynamics,     ///< all properties of all links
        Prop_JointCustomParameters = 0x2000, ///< when Joint::SetFloatParameters() and Joint::SetIntParameters() are called
        Prop_LinkCustomParameters = 0x4000, ///< when Link::SetFloatParameters() and Link::SetIntParameters() are called

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
    };

    /// \brief used for specifying the type of limit checking and the messages associated with it
    enum CheckLimitsAction {
        CLA_Nothing = 0, ///< no checking
        CLA_CheckLimits = 1, /// < checks and warns if the limits are overboard (default)
        CLA_CheckLimitsSilent = 2, ///< checks the limits and silently clamps the joint values (used if the code expects bad values as part of normal operation)
        CLA_CheckLimitsThrow = 3, ///< check the limits and throws if something went wrong
    };

    /// \brief Describes the properties of a geometric primitive.
    ///
    /// Contains everything associated with a geometry's appearance and shape
    class OPENRAVE_API GeometryInfo : public XMLReadable
    {
public:
        GeometryInfo();
        virtual ~GeometryInfo() {
        }

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
        inline const Vector& GetBoxExtents() const {
            return _vGeomData;
        }

        Transform _t; ///< Local transformation of the geom primitive with respect to the link's coordinate system.
        Vector _vGeomData; ///< for boxes, first 3 values are extents
        ///< for sphere it is radius
        ///< for cylinder, first 2 values are radius and height
        ///< for trimesh, none
        RaveVector<float> _vDiffuseColor, _vAmbientColor; ///< hints for how to color the meshes

        /// \brief trimesh representation of the collision data of this object in this local coordinate system
        ///
        /// Should be transformed by \ref _t before rendering.
        /// For spheres and cylinders, an appropriate discretization value is chosen.
        /// If empty, will be automatically computed from the geometry's type and render data
        TriMesh _meshcollision;

        GeometryType _type; ///< the type of geometry primitive

        /// \brief filename for render model (optional)
        ///
        /// Should be transformed by _t before rendering.
        /// If the value is "__norenderif__:x", then the viewer should not render the object if it supports *.x files where"x" is the file extension.
        std::string _filenamerender;

        /// \brief filename for collision data (optional)
        ///
        /// If the value is "__norenderif__:x", then the viewer should not render the object if it supports *.x files where"x" is the file extension.
        std::string _filenamecollision;

        Vector _vRenderScale; ///< render scale of the object (x,y,z) from _filenamerender
        Vector _vCollisionScale; ///< render scale of the object (x,y,z) from _filenamecollision
        float _fTransparency; ///< value from 0-1 for the transparency of the rendered object, 0 is opaque
        bool _bVisible; ///< if true, geometry is visible as part of the 3d model (default is true)
        bool _bModifiable; ///< if true, object geometry can be dynamically modified (default is true)
    };
    typedef boost::shared_ptr<GeometryInfo> GeometryInfoPtr;
    typedef boost::shared_ptr<GeometryInfo const> GeometryInfoConstPtr;

    /// \brief Describes the properties of a link used to initialize it
    class OPENRAVE_API LinkInfo : public XMLReadable
    {
public:
        LinkInfo();
        virtual ~LinkInfo() {
        }
        std::vector<GeometryInfoPtr> _vgeometryinfos;
        /// unique link name
        std::string _name;
        ///< the current transformation of the link with respect to the body coordinate system
        Transform _t;
        /// the frame for inertia and center of mass of the link in the link's coordinate system
        Transform _tMassFrame;
        /// mass of link
        dReal _mass;
        /// inertia along the axes of _tMassFrame
        Vector _vinertiamoments;
        std::map<std::string, std::vector<dReal> > _mapFloatParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::vector<int> > _mapIntParameters; ///< custom key-value pairs that could not be fit in the current model
        /// force the following links to be treated as adjacent to this link
        std::vector<std::string> _vForcedAdjacentLinks;
        /// \brief Indicates a static body that does not move with respect to the root link.
        ///
        //// Static should be used when an object has infinite mass and
        ///< shouldn't be affected by physics (including gravity). Collision still works.
        bool _bStatic;

        /// \true false if the link is disabled. disabled links do not participate in collision detection
        bool _bIsEnabled;
        bool __padding0, __padding1; // for 4-byte alignment
    };
    typedef boost::shared_ptr<LinkInfo> LinkInfoPtr;
    typedef boost::shared_ptr<LinkInfo const> LinkInfoConstPtr;

    /// \brief A rigid body holding all its collision and rendering data.
    class OPENRAVE_API Link : public boost::enable_shared_from_this<Link>
    {
public:
        Link(KinBodyPtr parent);         ///< pass in a ODE world
        virtual ~Link();

        /// \deprecated (12/10/18)
        typedef KinBody::GeometryInfo GeometryInfo RAVE_DEPRECATED;
        typedef boost::shared_ptr<KinBody::GeometryInfo> GeometryInfoPtr RAVE_DEPRECATED;
        typedef TriMesh TRIMESH RAVE_DEPRECATED;
        typedef GeometryType GeomType RAVE_DEPRECATED;
        static const GeometryType GeomNone RAVE_DEPRECATED = OpenRAVE::GT_None;
        static const GeometryType GeomBox RAVE_DEPRECATED = OpenRAVE::GT_Box;
        static const GeometryType GeomSphere RAVE_DEPRECATED = OpenRAVE::GT_Sphere;
        static const GeometryType GeomCylinder RAVE_DEPRECATED = OpenRAVE::GT_Cylinder;
        static const GeometryType GeomTrimesh RAVE_DEPRECATED = OpenRAVE::GT_TriMesh;

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
            virtual ~Geometry() {
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
                return _info._vGeomData.x;
            }
            inline dReal GetCylinderRadius() const {
                return _info._vGeomData.x;
            }
            inline dReal GetCylinderHeight() const {
                return _info._vGeomData.y;
            }
            inline const Vector& GetBoxExtents() const {
                return _info._vGeomData;
            }
            inline const RaveVector<float>& GetDiffuseColor() const {
                return _info._vDiffuseColor;
            }
            inline const RaveVector<float>& GetAmbientColor() const {
                return _info._vAmbientColor;
            }

            /// \brief returns the local collision mesh
            inline const TriMesh& GetCollisionMesh() const {
                return _info._meshcollision;
            }

            inline const KinBody::GeometryInfo& GetInfo() const {
                return _info;
            }

            /// \brief returns an axis aligned bounding box given that the geometry is transformed by trans
            virtual AABB ComputeAABB(const Transform& trans) const;
            virtual void serialize(std::ostream& o, int options) const;

            /// \brief sets a new collision mesh and notifies every registered callback about it
            virtual void SetCollisionMesh(const TriMesh& mesh);
            /// \brief sets visible flag. if changed, notifies every registered callback about it.
            ///
            /// \return true if changed
            virtual bool SetVisible(bool visible);
            /// \deprecated (12/1/12)
            inline void SetDraw(bool bDraw) RAVE_DEPRECATED {
                SetVisible(bDraw);
            }
            /// \brief set transparency level (0 is opaque)
            virtual void SetTransparency(float f);
            /// \brief override diffuse color of geometry material
            virtual void SetDiffuseColor(const RaveVector<float>& color);
            /// \brief override ambient color of geometry material
            virtual void SetAmbientColor(const RaveVector<float>& color);

            /// \brief validates the contact normal on the surface of the geometry and makes sure the normal faces "outside" of the shape.
            ///
            /// \param position the position of the contact point specified in the link's coordinate system
            /// \param normal the unit normal of the contact point specified in the link's coordinate system
            /// \return true if the normal is changed to face outside of the shape
            virtual bool ValidateContactNormal(const Vector& position, Vector& normal) const;

            /// \brief sets a new render filename for the geometry. This does not change the collision
            virtual void SetRenderFilename(const std::string& renderfilename);

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
        typedef Geometry GEOMPROPERTIES RAVE_DEPRECATED;

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
        virtual void Enable(bool enable);

        /// \brief returns true if the link is enabled. \see Enable
        virtual bool IsEnabled() const;

        /// \brief Sets all the geometries of the link as visible or non visible.
        ///
        /// \return true if changed
        virtual bool SetVisible(bool visible);

        /// \return true if any geometry of the link is visible.
        virtual bool IsVisible() const;

        /// \brief parent body that link belong to.
        inline KinBodyPtr GetParent() const {
            return KinBodyPtr(_parent);
        }

        /// \brief unique index into parent KinBody::GetLinks vector
        inline int GetIndex() const {
            return _index;
        }
        inline const TriMesh& GetCollisionData() const {
            return _collision;
        }

        /// \brief Compute the aabb of all the geometries of the link in the link coordinate system
        virtual AABB ComputeLocalAABB() const;

        /// \brief Compute the aabb of all the geometries of the link in the world coordinate system
        virtual AABB ComputeAABB() const;

        /// \brief Return the current transformation of the link in the world coordinate system.
        inline Transform GetTransform() const {
            return _info._t;
        }

        /// \brief Return all the direct parent links in the kinematics hierarchy of this link.
        ///
        /// A parent link is is immediately connected to this link by a joint and has a path to the root joint so that it is possible
        /// to compute this link's transformation from its parent.
        /// \param[out] filled with the parent links
        virtual void GetParentLinks(std::vector< boost::shared_ptr<Link> >& vParentLinks) const;

        /// \brief Tests if a link is a direct parent.
        ///
        /// \see GetParentLinks
        /// \param link The link to test if it is one of the parents of this link.
        virtual bool IsParentLink(boost::shared_ptr<Link const> plink) const;

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
        virtual TransformMatrix GetLocalInertia() const;

        // \brief return inertia around the link's COM in the global coordinate frame.
        virtual TransformMatrix GetGlobalInertia() const;

        /// \deprecated (12/1/20)
        inline TransformMatrix GetInertia() const RAVE_DEPRECATED {
            RAVELOG_WARN("KinBody::Link::GetInertia is deprecated, use KinBody::Link::GetGlobalInertia\n");
            return GetLocalInertia();
        }

        /// \brief sets a new mass frame with respect to the link coordinate system
        virtual void SetLocalMassFrame(const Transform& massframe);

        /// \brief sets new principal moments of inertia (with respect to the mass frame)
        virtual void SetPrincipalMomentsOfInertia(const Vector& inertiamoments);

        /// \brief set a new mass
        virtual void SetMass(dReal mass);

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
        virtual void SetStatic(bool bStatic);

        /// \brief Sets the transform of the link regardless of kinematics
        ///
        /// \param[in] t the new transformation
        virtual void SetTransform(const Transform& transform);

        /// adds an external force at pos (absolute coords)
        /// \param[in] force the direction and magnitude of the force
        /// \param[in] pos in the world where the force is getting applied
        /// \param[in] add if true, force is added to previous forces, otherwise it is set
        virtual void SetForce(const Vector& force, const Vector& pos, bool add);

        /// adds torque to a body (absolute coords)
        /// \param add if true, torque is added to previous torques, otherwise it is set
        virtual void SetTorque(const Vector& torque, bool add);

        /// forces the velocity of the link
        /// \param[in] linearvel the translational velocity
        /// \param[in] angularvel is the rotation axis * angular speed
        virtual void SetVelocity(const Vector& linearvel, const Vector& angularvel);

        /// \brief get the velocity of the link
        /// \param[out] linearvel the translational velocity
        /// \param[out] angularvel is the rotation axis * angular speed
        virtual void GetVelocity(Vector& linearvel, Vector& angularvel) const;

        /// \brief return the linear/angular velocity of the link
        virtual std::pair<Vector,Vector> GetVelocity() const;

        /// \brief returns a list of all the geometry objects.
        inline const std::vector<GeometryPtr>& GetGeometries() const {
            return _vGeometries;
        }
        virtual GeometryPtr GetGeometry(int index);

        /// \brief inits the current geometries with the new geometry info.
        ///
        /// This gives a user control for dynamically changing the object geometry. Note that the kinbody/robot hash could change.
        /// \param geometries a list of geometry infos to be initialized into new geometry objects, note that the geometry info data is copied
        virtual void InitGeometries(std::vector<KinBody::GeometryInfoConstPtr>& geometries);
        virtual void InitGeometries(std::list<KinBody::GeometryInfo>& geometries);

        /// \brief swaps the geometries with the link
        virtual void SwapGeometries(boost::shared_ptr<Link>& link);

        /// validates the contact normal on link and makes sure the normal faces "outside" of the geometry shape it lies on. An exception can be thrown if position is not on a geometry surface
        /// \param position the position of the contact point specified in the link's coordinate system, assumes it is on a particular geometry
        /// \param normal the unit normal of the contact point specified in the link's coordinate system
        /// \return true if the normal is changed to face outside of the shape
        virtual bool ValidateContactNormal(const Vector& position, Vector& normal) const;

        /// \brief returns true if plink is rigidily attahced to this link.
        virtual bool IsRigidlyAttached(boost::shared_ptr<Link const> plink) const;

        /// \brief Gets all the rigidly attached links to linkindex, also adds the link to the list.
        ///
        /// \param vattachedlinks the array to insert all links attached to linkindex with the link itself.
        virtual void GetRigidlyAttachedLinks(std::vector<boost::shared_ptr<Link> >& vattachedlinks) const;

        virtual void serialize(std::ostream& o, int options) const;

        /// \brief return a map of custom float parameters
        inline const std::map<std::string, std::vector<dReal> >& GetFloatParameters() const {
            return _info._mapFloatParameters;
        }

        /// \brief set custom float parameters
        ///
        /// \param parameters if empty, then removes the parameter
        virtual void SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters);

        /// \brief return a map of custom integer parameters
        inline const std::map<std::string, std::vector<int> >& GetIntParameters() const {
            return _info._mapIntParameters;
        }

        /// \brief set custom int parameters
        ///
        /// \param parameters if empty, then removes the parameter
        virtual void SetIntParameters(const std::string& key, const std::vector<int>& parameters);

        /// \brief Updates several fields in \ref _info depending on the current state of the link
        virtual void UpdateInfo();

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

protected:
        /// \brief Updates the cached information due to changes in the collision data.
        virtual void _Update();

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
    class OPENRAVE_API MimicInfo
    {
public:
        boost::array< std::string, 3>  _equations;         ///< the original equations
    };
    typedef boost::shared_ptr<MimicInfo> MimicInfoPtr;
    typedef boost::shared_ptr<MimicInfo const> MimicInfoConstPtr;

    class OPENRAVE_API Mimic
    {
public:
        boost::array< std::string, 3>  _equations;         ///< the original equations

        struct DOFFormat
        {
            int16_t jointindex;         ///< the index into the joints, if >= GetJoints.size(), then points to the passive joints
            int16_t dofindex : 14;         ///< if >= 0, then points to a DOF of the robot that is NOT mimiced
            uint8_t axis : 2;         ///< the axis of the joint index
            bool operator <(const DOFFormat& r) const;
            bool operator ==(const DOFFormat& r) const;
            boost::shared_ptr<Joint> GetJoint(KinBodyPtr parent) const;
            boost::shared_ptr<Joint const> GetJoint(KinBodyConstPtr parent) const;
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
    class OPENRAVE_API JointInfo : public XMLReadable
    {
public:
        JointInfo();
        virtual ~JointInfo() {
        }

        JointType _type; /// The joint type
        std::string _name;         ///< the unique joint name
        std::string _linkname0, _linkname1; ///< attaching links, all axes and anchors are defined in the link pointed to by _linkname0 coordinate system. _linkname0 is usually the parent link.
        Vector _vanchor; ///< the anchor of the rotation axes defined in _linkname0's coordinate system. this is only used to construct the internal left/right matrices. passed into Joint::_ComputeInternalInformation
        boost::array<Vector,3> _vaxes;                ///< axes in _linkname0's or environment coordinate system used to define joint movement. passed into Joint::_ComputeInternalInformation
        std::vector<dReal> _vcurrentvalues; ///< joint values at initialization. passed into Joint::_ComputeInternalInformation

        boost::array<dReal,3> _vresolution;              ///< interpolation resolution
        boost::array<dReal,3> _vmaxvel;                  ///< the soft maximum velocity (rad/s) to move the joint when planning
        boost::array<dReal,3> _vhardmaxvel;              ///< the hard maximum velocity, robot cannot exceed this velocity. used for verification checking
        boost::array<dReal,3> _vmaxaccel;                ///< the maximum acceleration (rad/s^2) of the joint
        boost::array<dReal,3> _vmaxtorque;               ///< maximum torque (N.m, kg m^2/s^2) that can be applied to the joint
        boost::array<dReal,3> _vweights;                ///< the weights of the joint for computing distance metrics.

        /// \brief internal offset parameter that determines the branch the angle centers on
        ///
        /// Wrap offsets are needed for rotation joints since the range is limited to 2*pi.
        /// This allows the wrap offset to be set so the joint can function in [-pi+offset,pi+offset]..
        /// \param iaxis the axis to get the offset from
        boost::array<dReal,3> _voffsets;
        boost::array<dReal,3> _vlowerlimit, _vupperlimit;         ///< joint limits
        TrajectoryBasePtr _trajfollow; ///< used if joint type is JointTrajectory

        boost::array<MimicInfoPtr,3> _vmimic;          ///< the mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free. When cloning, is it ok to copy this and assume it is constant?

        std::map<std::string, std::vector<dReal> > _mapFloatParameters; ///< custom key-value pairs that could not be fit in the current model
        std::map<std::string, std::vector<int> > _mapIntParameters; ///< custom key-value pairs that could not be fit in the current model

        /// true if joint axis has an identification at some of its lower and upper limits.
        ///
        /// An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
        /// at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
        /// joint does not stop at limits.
        /// Although currently not developed, it could be possible to support identification for joints that are not revolute.
        boost::array<uint8_t,3> _bIsCircular;

        bool _bIsActive;                 ///< if true, should belong to the DOF of the body, unless it is a mimic joint (_ComputeInternalInformation decides this)
    };
    typedef boost::shared_ptr<JointInfo> JointInfoPtr;
    typedef boost::shared_ptr<JointInfo const> JointInfoConstPtr;

    /// \brief Information about a joint that controls the relationship between two links.
    class OPENRAVE_API Joint : public boost::enable_shared_from_this<Joint>
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
        virtual ~Joint();

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
        inline dReal GetMaxTorque(int iaxis=0) const {
            return _info._vmaxtorque[iaxis];
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

        inline KinBodyPtr GetParent() const {
            return KinBodyPtr(_parent);
        }

        inline LinkPtr GetFirstAttached() const {
            return _attachedbodies[0];
        }
        inline LinkPtr GetSecondAttached() const {
            return _attachedbodies[1];
        }

        inline KinBody::JointType GetType() const {
            return _info._type;
        }

        /// \brief gets all resolutions for the joint axes
        ///
        /// \param[in] bAppend if true will append to the end of the vector instead of erasing it
        virtual void GetResolutions(std::vector<dReal>& resolutions, bool bAppend=false) const;

        /// \brief The discretization of the joint used when line-collision checking.
        ///
        /// The resolutions are set as large as possible such that the joint will not go through obstacles of determined size.
        virtual dReal GetResolution(int iaxis=0) const;

        virtual void SetResolution(dReal resolution, int iaxis=0);

        /// \brief The degrees of freedom of the joint. Each joint supports a max of 3 degrees of freedom.
        virtual int GetDOF() const;

        /// \brief Return true if joint axis has an identification at some of its lower and upper limits.
        ///
        /// An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
        /// at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
        /// joint does not stop at limits.
        /// Although currently not developed, it could be possible to support identification for joints that are not revolute.
        virtual bool IsCircular(int iaxis) const;

        /// \brief returns true if the axis describes a rotation around an axis.
        ///
        /// \param iaxis the axis of the joint to return the results for
        virtual bool IsRevolute(int iaxis) const;

        /// \brief returns true if the axis describes a translation around an axis.
        ///
        /// \param iaxis the axis of the joint to return the results for
        virtual bool IsPrismatic(int iaxis) const;

        /// \brief Return true if joint can be treated as a static binding (ie all limits are 0)
        virtual bool IsStatic() const;

        /// \brief Return all the joint values with the correct offsets applied
        ///
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return degrees of freedom of the joint (even if pValues is NULL)
        virtual void GetValues(std::vector<dReal>& values, bool bAppend=false) const;

        /// \brief Return the value of the specified joint axis only.
        virtual dReal GetValue(int axis) const;

        /// \brief Gets the joint velocities
        ///
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return the degrees of freedom of the joint (even if pValues is NULL)
        virtual void GetVelocities(std::vector<dReal>& values, bool bAppend=false) const;

        /// \brief Return the velocity of the specified joint axis only.
        virtual dReal GetVelocity(int axis) const;

        /// \brief Add effort (force or torque) to the joint
        virtual void AddTorque(const std::vector<dReal>& torques);

        /// \brief The anchor of the joint in global coordinates.
        virtual Vector GetAnchor() const;

        /// \brief The axis of the joint in global coordinates
        ///
        /// \param[in] axis the axis to get
        virtual Vector GetAxis(int axis = 0) const;

        /** \brief Get the limits of the joint

            \param[out] vLowerLimit the lower limits
            \param[out] vUpperLimit the upper limits
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend=false) const;

        /// \brief returns the lower and upper limit of one axis of the joint
        virtual std::pair<dReal, dReal> GetLimit(int iaxis=0) const;

        /// \brief \see GetLimits
        virtual void SetLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper);

        /** \brief Returns the max velocities of the joint

            \param[out] the max velocity
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        virtual void GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool bAppend=false) const;

        /// \brief returns the lower and upper velocity limit of one axis of the joint
        virtual std::pair<dReal, dReal> GetVelocityLimit(int iaxis=0) const;

        /// \brief \see GetVelocityLimits
        virtual void SetVelocityLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max accelerations of the joint

            \param[out] the max acceleration
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        virtual dReal GetAccelerationLimit(int iaxis=0) const;

        /// \brief \see GetAccelerationLimits
        virtual void SetAccelerationLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max torques of the joint

            \param[out] the max torque
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetTorqueLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        /// \brief \see GetTorqueLimits
        virtual void SetTorqueLimits(const std::vector<dReal>& vmax);

        /// \brief gets all weights for the joint axes
        ///
        /// \param[in] bAppend if true will append to the end of the vector instead of erasing it
        virtual void GetWeights(std::vector<dReal>& weights, bool bAppend=false) const;

        /// \brief The weight associated with a joint's axis for computing a distance in the robot configuration space.
        virtual dReal GetWeight(int axis=0) const;

        /// \brief \see GetWeight
        virtual void SetWeights(const std::vector<dReal>& weights);

        /// \brief Computes the configuration difference values1-values2 and stores it in values1.
        ///
        /// Takes into account joint limits and wrapping of circular joints.
        virtual void SubtractValues(std::vector<dReal>& values1, const std::vector<dReal>& values2) const;

        /// \brief Returns the configuration difference value1-value2 for axis i.
        ///
        /// Takes into account joint limits and wrapping of circular joints.
        virtual dReal SubtractValue(dReal value1, dReal value2, int iaxis) const;

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
        virtual void SetWrapOffset(dReal offset, int iaxis=0);

        virtual void serialize(std::ostream& o, int options) const;

        /// @name Internal Hierarchy Methods
        //@{
        /// \brief Return the parent link which the joint measures its angle off from (either GetFirstAttached() or GetSecondAttached())
        virtual LinkPtr GetHierarchyParentLink() const;
        /// \brief Return the child link whose transformation is computed by this joint's values (either GetFirstAttached() or GetSecondAttached())
        virtual LinkPtr GetHierarchyChildLink() const;
        /// \brief The axis of the joint in local coordinates.
        virtual Vector GetInternalHierarchyAxis(int axis = 0) const;
        /// \brief Left multiply transform given the base body.
        virtual Transform GetInternalHierarchyLeftTransform() const;
        /// \brief Right multiply transform given the base body.
        virtual Transform GetInternalHierarchyRightTransform() const;
        //@}

        /// A mimic joint's angles are automatically determined from other joints based on a general purpose formula.
        /// A user does not have control of the the mimic joint values, even if they appear in the DOF list.
        /// @name Mimic Joint Properties
        //@{

        /// \deprecated (11/1/1)
        virtual int GetMimicJointIndex() const RAVE_DEPRECATED;
        /// \deprecated (11/1/1)
        virtual const std::vector<dReal> GetMimicCoeffs() const RAVE_DEPRECATED;

        /// \brief Returns true if a particular axis of the joint is mimiced.
        ///
        /// \param axis the axis to query. When -1 returns true if any of the axes have mimic joints
        virtual bool IsMimic(int axis=-1) const;

        /** \brief If the joint is mimic, returns the equation to compute its value

            \param[in] axis the axis index
            \param[in] type 0 for position, 1 for velocity, 2 for acceleration.
            \param[in] format the format the equations are returned in. If empty or "fparser", equation in fparser format. Also supports: "mathml".

            MathML:

            Set 'format' to "mathml". The joint variables are specified with <csymbol>. If a targetted joint has more than one degree of freedom, then axis is suffixed with _\%d. If 'type' is 1 or 2, the partial derivatives are outputted as consecutive <math></math> tags in the same order as \ref Mimic::_vdofformat
         */
        virtual std::string GetMimicEquation(int axis=0, int type=0, const std::string& format="") const;

        /** \brief Returns the set of DOF indices that the computation of a joint axis depends on. Order is arbitrary.

            If the mimic joint uses the values of other mimic joints, then the dependent DOFs of that joint are also
            copied over. Therefore, the dof indices returned can be more than the actual variables used in the equation.
            \throw openrave_exception Throws an exception if the axis is not mimic.
         */
        virtual void GetMimicDOFIndices(std::vector<int>& vmimicdofs, int axis=0) const;

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
        virtual void SetMimicEquations(int axis, const std::string& poseq, const std::string& veleq, const std::string& acceleq="");
        //@}

        /// \brief return a map of custom float parameters
        inline const std::map<std::string, std::vector<dReal> >& GetFloatParameters() const {
            return _info._mapFloatParameters;
        }

        /// \brief set custom float parameters
        ///
        /// \param parameters if empty, then removes the parameter
        virtual void SetFloatParameters(const std::string& key, const std::vector<dReal>& parameters);

        /// \brief return a map of custom integer parameters
        inline const std::map<std::string, std::vector<int> >& GetIntParameters() const {
            return _info._mapIntParameters;
        }

        /// \brief set custom int parameters
        ///
        /// \param parameters if empty, then removes the parameter
        virtual void SetIntParameters(const std::string& key, const std::vector<int>& parameters);

        /// \brief Updates several fields in \ref _info depending on the current state of the joint.
        virtual void UpdateInfo();

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

protected:
        JointInfo _info;

        boost::array< MimicPtr,3> _vmimic;          ///< the mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free. When cloning, is it ok to copy this and assume it is constant?

        /** \brief computes the partial velocities with respect to all dependent DOFs specified by Mimic::_vmimicdofs.

            If the joint is not mimic, then just returns its own index
            \param[out] vpartials A list of dofindex/velocity_partial pairs. The final velocity is computed by taking the dot product. The dofindices do not repeat.
            \param[in] iaxis the axis
            \param[in,out] vcachedpartials set of cached partials for each degree of freedom
         */
        virtual void _ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials, int iaxis, std::map< std::pair<Mimic::DOFFormat, int>, dReal >& mapcachedpartials) const;

        /** \brief Compute internal transformations and specify the attached links of the joint.

            Called after the joint protected parameters {vAxes, vanchor, and _voffsets}  have been initialized. vAxes and vanchor should be in the frame of plink0.
            Compute the left and right multiplications of the joint transformation and cleans up the attached bodies.
            After function completes, the following parameters are initialized: _tRight, _tLeft, _tinvRight, _tinvLeft, _attachedbodies. _attachedbodies does not necessarily contain the links in the same order as they were input.
            \param plink0 the first attaching link, all axes and anchors are defined in its coordinate system
            \param plink1 the second attaching link
            \param vanchor the anchor of the rotation axes
            \param vaxes the axes in plink0's coordinate system of the joints
            \param vinitialvalues the current values of the robot used to set the 0 offset of the robot
         */
        virtual void _ComputeInternalInformation(LinkPtr plink0, LinkPtr plink1, const Vector& vanchor, const std::vector<Vector>& vaxes, const std::vector<dReal>& vcurrentvalues);

        /// \brief evaluates the mimic joint equation using vdependentvalues
        ///
        /// \param[in] axis the joint axis
        /// \param[in] timederiv the time derivative to evaluate. 0 is position, 1 is velocity, 2 is acceleration, etc
        /// \param[in] vdependentvalues input values ordered with respect to _vdofformat[iaxis]
        /// \param[out] voutput the output values
        /// \return an internal error code, 0 if no error
        virtual int _Eval(int axis, uint32_t timederiv, const std::vector<dReal>& vdependentvalues, std::vector<dReal>& voutput);

        /// \brief compute joint velocities given the parent and child link transformations/velocities
        virtual void _GetVelocities(std::vector<dReal>& values, bool bAppend, const std::pair<Vector,Vector>& linkparentvelocity, const std::pair<Vector,Vector>& linkchildvelocity) const;

        /// \brief Return the velocity of the specified joint axis only.
        virtual dReal _GetVelocity(int axis, const std::pair<Vector,Vector>&linkparentvelocity, const std::pair<Vector,Vector>&linkchildvelocity) const;

        boost::array<int,3> _dofbranches; ///< the branch that identified joints are on. +1 means one loop around the identification. For revolute joints, the actual joint value incremented by 2*pi*branch. Branches are important for maintaining joint ranges greater than 2*pi. For circular joints, the branches can be ignored or not.

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
    };
    typedef boost::shared_ptr<KinBody::Joint> JointPtr;
    typedef boost::shared_ptr<KinBody::Joint const> JointConstPtr;
    typedef boost::weak_ptr<KinBody::Joint> JointWeakPtr;

    /// \brief Stores the state of the current body that is published in a thread safe way from the environment without requiring locking the environment.
    class BodyState
    {
public:
        BodyState() : environmentid(0){
        }
        virtual ~BodyState() {
        }
        KinBodyPtr pbody;
        std::vector<RaveTransform<dReal> > vectrans;
        std::vector<dReal> jointvalues;
        std::string strname;         ///< name of the body
        int environmentid;
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
        virtual XMLReadableConstPtr GetData() const = 0;

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
        Save_LinkTransformation=0x00000001,     ///< [default] save link transformations and joint branches
        Save_LinkEnable=0x00000002,     ///< [default] save link enable states
        Save_LinkVelocities=0x00000004,     ///< save the link velocities
        Save_JointMaxVelocityAndAcceleration=0x00000008, ///< save the max joint velocities and accelerations for the controller DOF
        Save_ActiveDOF=0x00010000,     ///< [robot only], saves and restores the current active degrees of freedom
        Save_ActiveManipulator=0x00020000,     ///< [robot only], saves the active manipulator
        Save_GrabbedBodies=0x00040000,     ///< [robot only], saves the grabbed state of the bodies. This does not affect the configuraiton of those bodies.
    };

    /// \brief Helper class to save and restore the entire kinbody state.
    ///
    /// Options can be passed to the constructor in order to choose which parameters to save (see \ref SaveParameters)
    class OPENRAVE_API KinBodyStateSaver
    {
public:
        KinBodyStateSaver(KinBodyPtr pbody, int options = Save_LinkTransformation|Save_LinkEnable);
        virtual ~KinBodyStateSaver();
        inline KinBodyPtr GetBody() const {
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
protected:
        int _options;         ///< saved options
        std::vector<Transform> _vLinkTransforms;
        std::vector<uint8_t> _vEnabledLinks;
        std::vector<std::pair<Vector,Vector> > _vLinkVelocities;
        std::vector<int> _vdofbranches;
        std::vector<dReal> _vMaxVelocities, _vMaxAccelerations;
        KinBodyPtr _pbody;
private:
        virtual void _RestoreKinBody(boost::shared_ptr<KinBody> body);
    };

    typedef boost::shared_ptr<KinBodyStateSaver> KinBodyStateSaverPtr;

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
    virtual bool InitFromBoxes(const std::vector<AABB>& boxes, bool visible);

    /// \brief Create a kinbody with one link composed of an array of oriented bounding boxes.
    ///
    /// \param boxes the array of oriented bounding boxes that will comprise of the body
    /// \param visible if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<OBB>& boxes, bool visible);

    /// \brief Create a kinbody with one link composed of an array of spheres
    ///
    /// \param spheres the XYZ position of the spheres with the W coordinate representing the individual radius
    /// \param visible if true, the boxes will be rendered in the scene
    virtual bool InitFromSpheres(const std::vector<Vector>& spheres, bool visible);

    /// \brief Create a kinbody with one link composed of a triangle mesh surface
    ///
    /// \param trimesh the triangle mesh
    /// \param visible if true, will be rendered in the scene
    virtual bool InitFromTrimesh(const TriMesh& trimesh, bool visible);

    /// \brief Create a kinbody with one link composed of a list of geometries
    ///
    /// \param geometries a list of geometry infos to be initialized into new geometry objects, note that the geometry info data is copied
    /// \param visible if true, will be rendered in the scene
    virtual bool InitFromGeometries(const std::vector<KinBody::GeometryInfoConstPtr>& geometries);
    virtual bool InitFromGeometries(const std::list<KinBody::GeometryInfo>& geometries);

    /// \brief initializes an complex kinematics body with links and joints
    ///
    /// \param linkinfos information for all the links. Links will be created in this order
    /// \param jointinfos information for all the joints. Joints might be rearranged depending on their mimic properties
    virtual bool Init(const std::vector<LinkInfoConstPtr>& linkinfos, const std::vector<JointInfoConstPtr>& jointinfos);

    /// \brief Unique name of the robot.
    virtual const std::string& GetName() const {
        return _name;
    }

    /// \brief Set the name of the body, notifies the environment and checks for uniqueness.
    virtual void SetName(const std::string& name);

    /// Methods for accessing basic information about joints
    /// @name Basic Information
    //@{

    /// \brief Number controllable degrees of freedom of the body.
    ///
    /// Only uses _vecjoints and last joint for computation, so can work before _ComputeInternalInformation is called.
    virtual int GetDOF() const;

    /// \brief Returns all the joint values as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFValues(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint velocities as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFVelocities(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint limits as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns all the joint velocity limits as organized by the DOF indices.
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFVelocityLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max velocity for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFVelocityLimits(std::vector<dReal>& maxvelocities, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max acceleration for each DOF
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFAccelerationLimits(std::vector<dReal>& maxaccelerations, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief Returns the max torque for each DOF
    virtual void GetDOFTorqueLimits(std::vector<dReal>& maxaccelerations) const;

    /// \deprecated (11/05/26)
    virtual void GetDOFMaxVel(std::vector<dReal>& v) const RAVE_DEPRECATED {
        GetDOFVelocityLimits(v);
    }
    virtual void GetDOFMaxAccel(std::vector<dReal>& v) const RAVE_DEPRECATED {
        GetDOFAccelerationLimits(v);
    }
    virtual void GetDOFMaxTorque(std::vector<dReal>& v) const;

    /// \brief get the dof resolutions
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFResolutions(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief get dof weights
    ///
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void GetDOFWeights(std::vector<dReal>& v, const std::vector<int>& dofindices = std::vector<int>()) const;

    /// \brief \see GetDOFVelocityLimits
    virtual void SetDOFVelocityLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFAccelerationLimits
    virtual void SetDOFAccelerationLimits(const std::vector<dReal>& maxlimits);

    /// \brief \see GetDOFTorqueLimits
    virtual void SetDOFTorqueLimits(const std::vector<dReal>& maxlimits);

    /// \brief sets dof weights
    ///
    /// \param dofindices the dof indices to set the values for. If empty, will use all the dofs
    virtual void SetDOFWeights(const std::vector<dReal>& weights, const std::vector<int>& dofindices = std::vector<int>());

    /// \brief \see GetDOFLimits
    virtual void SetDOFLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper);

    /// \brief Returns the joints making up the controllable degrees of freedom of the body.
    const std::vector<JointPtr>& GetJoints() const {
        return _vecjoints;
    }

    /** \brief Returns the passive joints, order does not matter.

        A passive joint is not directly controlled by the body's degrees of freedom so it has no
        joint index and no dof index. Passive joints allows mimic joints to be hidden from the users.
        However, there are cases when passive joints are not mimic; for example, suspension mechanism on vehicles.
     */
    const std::vector<JointPtr>& GetPassiveJoints() const {
        return _vPassiveJoints;
    }

    /// \brief Returns the joints in hierarchical order starting at the base link.
    ///
    /// In the case of closed loops, the joints are returned in the order closest to the root.
    /// All the joints affecting a particular joint's transformation will always come before the joint in the list.
    virtual const std::vector<JointPtr>& GetDependencyOrderedJoints() const;

    /** \brief Return the set of unique closed loops of the kinematics hierarchy.

        Each loop is a set of link indices and joint indices. For example, a loop of link indices:
        [l_0,l_1,l_2] will consist of three joints connecting l_0 to l_1, l_1 to l_2, and l_2 to l_0.
        The first element in the pair is the link l_X, the second element in the joint connecting l_X to l_(X+1).
     */
    virtual const std::vector< std::vector< std::pair<LinkPtr, JointPtr> > >& GetClosedLoops() const;

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
    virtual bool GetChain(int linkindex1, int linkindex2, std::vector<JointPtr>& vjoints) const;

    /// \brief similar to \ref GetChain(int,int,std::vector<JointPtr>&) except returns the links along the path.
    virtual bool GetChain(int linkindex1, int linkindex2, std::vector<LinkPtr>& vlinks) const;

    /// \brief Returns true if the dof index affects the relative transformation between the two links.
    ///
    /// The internal implementation uses \ref KinBody::DoesAffect, therefore mimic indices are correctly handled.
    /// \param[in] linkindex1 the link index to start the search
    /// \param[in] linkindex2 the link index where the search ends
    virtual bool IsDOFInChain(int linkindex1, int linkindex2, int dofindex) const;

    /// \brief Return the index of the joint with the given name, else -1.
    virtual int GetJointIndex(const std::string& name) const;

    /// \brief Return a pointer to the joint with the given name. Search in the regular and passive joints.
    virtual JointPtr GetJoint(const std::string& name) const;

    /// \brief Returns the joint that covers the degree of freedom index.
    ///
    /// Note that the mapping of joint structures is not the same as the values in GetJointValues since each joint can have more than one degree of freedom.
    virtual JointPtr GetJointFromDOFIndex(int dofindex) const;
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
    virtual const std::vector<LinkPtr>& GetLinks() const {
        return _veclinks;
    }

    /// return a pointer to the link with the given name
    virtual LinkPtr GetLink(const std::string& name) const;

    /// Updates the bounding box and any other parameters that could have changed by a simulation step
    virtual void SimulationStep(dReal fElapsedTime);

    /// \brief get the transformations of all the links at once
    virtual void GetLinkTransformations(std::vector<Transform>& transforms) const;

    /// \brief get the transformations of all the links and the dof branches at once.
    ///
    /// Knowing the dof branches allows the robot to recover the full state of the joints with SetLinkTransformations
    virtual void GetLinkTransformations(std::vector<Transform>& transforms, std::vector<int>& dofbranches) const;

    /// \deprecated (11/05/26)
    virtual void GetBodyTransformations(std::vector<Transform>& transforms) const RAVE_DEPRECATED {
        GetLinkTransformations(transforms);
    }

    /// queries the transfromation of the first link of the body
    virtual Transform GetTransform() const;

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
    virtual void GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const;

    /** \brief Returns the linear and angular accelerations for each link given the dof accelerations

        Computes accelerations of the link frames with respect to the world coordinate system are returned.
        The base angular velocity is used when computing accelerations.
        The gravity vector from the physics engine is used as the  accelerations for the base link and static links.
        The derivate is taken with respect to the world origin fixed in space (also known as spatial acceleration).
        The current angles and velocities set on the robot are used.
        Note that this function calls the internal _ComputeLinkAccelerations function, so for users that are interested in overriding it, override _ComputeLinkAccelerations
        \param[in] dofaccelerations the accelerations of each of the DOF
        \param[out] linkaccelerations the linear and angular accelerations of link (in that order)
     */
    virtual void GetLinkAccelerations(const std::vector<dReal>& dofaccelerations, std::vector<std::pair<Vector,Vector> >& linkaccelerations) const;

    /** \en \brief set the transform of the first link (the rest of the links are computed based on the joint values).

        \param transform affine transformation

        \ja \brief 胴体の絶対姿勢を設定、残りのリンクは運動学の構造に従って変換される．

        \param transform 変換行列
     */
    virtual void SetTransform(const Transform& transform);

    /// \brief Return an axis-aligned bounding box of the entire object in the world coordinate system.
    virtual AABB ComputeAABB() const;

    /// \brief Return the center of mass of entire robot in the world coordinate system.
    virtual Vector GetCenterOfMass() const;

    /// \brief Enables or disables all the links.
    virtual void Enable(bool enable);

    /// \return true if any link of the KinBody is enabled
    virtual bool IsEnabled() const;

    /// \brief Sets all the links as visible or not visible.
    ///
    /// \return true if changed
    virtual bool SetVisible(bool visible);

    /// \return true if any link of the KinBody is visible.
    virtual bool IsVisible() const;

    /// \brief Sets the joint values of the robot.
    ///
    /// \param values the values to set the joint angles (ordered by the dof indices)
    /// \param[in] checklimits one of \ref CheckLimitsAction and will excplicitly check the joint limits before setting the values and clamp them.
    /// \param dofindices the dof indices to return the values for. If empty, will compute for all the dofs
    virtual void SetDOFValues(const std::vector<dReal>& values, uint32_t checklimits = CLA_CheckLimits, const std::vector<int>& dofindices = std::vector<int>());

    virtual void SetJointValues(const std::vector<dReal>& values, bool checklimits = true) {
        SetDOFValues(values,static_cast<uint32_t>(checklimits));
    }

    /// \brief Sets the joint values and transformation of the body.
    ///
    /// \param values the values to set the joint angles (ordered by the dof indices)
    /// \param transform represents the transformation of the first body.
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
    virtual void SetLinkTransformations(const std::vector<Transform>& transforms, const std::vector<int>& dofbranches);

    /// \deprecated (11/05/26)
    virtual void SetBodyTransformations(const std::vector<Transform>& transforms) RAVE_DEPRECATED {
        SetLinkTransformations(transforms);
    }

    /// \brief sets the link velocities
    virtual void SetLinkVelocities(const std::vector<std::pair<Vector,Vector> >& velocities);

    /// \brief Computes the translation jacobian with respect to a world position.
    ///
    /// Gets the jacobian with respect to a link by computing the partial differentials for all joints that in the path from the root node to GetLinks()[index]
    /// (doesn't touch the rest of the values)
    /// \param linkindex of the link that defines the frame the position is attached to
    /// \param position position in world space where to compute derivatives from.
    /// \param jacobian 3xDOF matrix
    /// \param dofindices the dof indices to compute the jacobian for. If empty, will compute for all the dofs
    virtual void ComputeJacobianTranslation(int linkindex, const Vector& position, std::vector<dReal>& jacobian, const std::vector<int>& dofindices=std::vector<int>()) const;

    /// \brief calls std::vector version of ComputeJacobian internally
    virtual void CalculateJacobian(int linkindex, const Vector& position, std::vector<dReal>& jacobian) const {
        ComputeJacobianTranslation(linkindex,position,jacobian);
    }

    /// \brief calls std::vector version of ComputeJacobian internally, a little inefficient since it copies memory
    virtual void CalculateJacobian(int linkindex, const Vector& position, boost::multi_array<dReal,2>& jacobian) const;

    /// \brief Computes the rotational jacobian as a quaternion with respect to an initial rotation.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param qInitialRot the rotation in world space whose derivative to take from.
    /// \param jacobian 4xDOF matrix
    virtual void CalculateRotationJacobian(int linkindex, const Vector& quat, std::vector<dReal>& jacobian) const;

    /// \brief calls std::vector version of CalculateRotationJacobian internally, a little inefficient since it copies memory
    virtual void CalculateRotationJacobian(int linkindex, const Vector& quat, boost::multi_array<dReal,2>& jacobian) const;

    /// \brief Computes the angular velocity jacobian of a specified link about the axes of world coordinates.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param vjacobian 3xDOF matrix
    virtual void ComputeJacobianAxisAngle(int linkindex, std::vector<dReal>& jacobian, const std::vector<int>& dofindices=std::vector<int>()) const;

    /// \brief Computes the angular velocity jacobian of a specified link about the axes of world coordinates.
    virtual void CalculateAngularVelocityJacobian(int linkindex, std::vector<dReal>& jacobian) const {
        ComputeJacobianAxisAngle(linkindex,jacobian);
    }

    /// \brief calls std::vector version of CalculateAngularVelocityJacobian internally, a little inefficient since it copies memory
    virtual void CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& jacobian) const;

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

        /// \param linkindex of the link that defines the frame the position is attached to
        /// \param position position in world space where to compute derivatives from.
        /// \param hessian DOFx3xDOF matrix such that numpy.dot(dq,numpy.dot(hessian,dq)) is the expected second-order delta translation
        /// \param dofindices the dof indices to compute the hessian for. If empty, will compute for all the dofs
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

        /// \param linkindex of the link that defines the frame the position is attached to
        /// \param hessian DOFx3xDOF matrix such that numpy.dot(dq,numpy.dot(hessian,dq)) is the expected second-order delta angle-axis
        /// \param dofindices the dof indices to compute the hessian for. If empty, will compute for all the dofs
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

    /// \brief Check if body is self colliding. Links that are joined together are ignored.
    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr()) const;

    /// \return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    virtual bool IsAttached(KinBodyConstPtr body) const;

    /// \brief Recursively get all attached bodies of this body, including this body.
    ///
    /// \param setAttached fills with the attached bodies. If any bodies are already in setAttached, then ignores recursing on their attached bodies.
    virtual void GetAttached(std::set<KinBodyPtr>& setAttached) const;

    /// \brief Return true if this body is derived from RobotBase.
    virtual bool IsRobot() const {
        return false;
    }

    /// \brief return a unique id of the body used in the environment.
    ///
    /// If object is not added to the environment, this will return 0. So checking if GetEnvironmentId() is 0 is a good way to check if object is present in the environment.
    /// This id will not be copied when cloning in order to respect another environment's ids.
    virtual int GetEnvironmentId() const;

    /** \brief Returns a nonzero value if the joint effects the link transformation.

        In closed loops, all joints on all paths to the root link are counted as affecting the link.
        If a mimic joint affects the link, then all the joints used in the mimic joint's computation affect the link.
        If negative, the partial derivative of the Jacobian should be negated.
        \param jointindex index of the joint
        \param linkindex index of the link
     */
    virtual int8_t DoesAffect(int jointindex, int linkindex) const;

    /// \deprecated (12/12/11)
    virtual UserDataPtr GetViewerData() const RAVE_DEPRECATED {
        return GetUserData("_viewer_");
    }

    /// \brief specifies the type of adjacent link information to receive
    enum AdjacentOptions
    {
        AO_Enabled = 1,     ///< return only enabled link pairs
        AO_ActiveDOFs = 2,     ///< return only link pairs that have an active in its path
    };

    /// \brief return all possible link pairs that could get in collision.
    /// \param adjacentoptions a bitmask of \ref AdjacentOptions values
    virtual const std::set<int>& GetNonAdjacentLinks(int adjacentoptions=0) const;

    /// \brief return all possible link pairs whose collisions are ignored.
    virtual const std::set<int>& GetAdjacentLinks() const;

    /// \deprecated (12/12/11)
    virtual UserDataPtr GetPhysicsData() const RAVE_DEPRECATED {
        return GetUserData("_physics_");
    }
    /// \deprecated (12/12/11)
    virtual UserDataPtr GetCollisionData() const RAVE_DEPRECATED {
        return GetUserData("_collision_");
    }
    virtual ManageDataPtr GetManageData() const {
        return _pManageData;
    }

    /// \brief Return a unique id for every transformation state change of any link. Used to check if robot state has changed.
    ///
    /// The stamp is used by the collision checkers, physics engines, or any other item
    /// that needs to keep track of any changes of the KinBody as it moves.
    /// Currently stamps monotonically increment for every transformation/joint angle change.
    virtual int GetUpdateStamp() const {
        return _nUpdateStampId;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /// \brief Register a callback with the interface.
    ///
    /// Everytime a static property of the interface changes, all
    /// registered callbacks are called to update the users of the changes. Note that the callbacks will
    /// block the thread that made the parameter change.
    /// \param callback
    /// \param properties a mask of the \ref KinBodyProperty values that the callback should be called for when they change
    virtual UserDataPtr RegisterChangeCallback(int properties, const boost::function<void()>& callback) const;

    void Serialize(BaseXMLWriterPtr writer, int options=0) const;

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

    /// only used for hashes...
    virtual void serialize(std::ostream& o, int options) const;

protected:
    /// \brief constructors declared protected so that user always goes through environment to create bodies
    KinBody(InterfaceType type, EnvironmentBasePtr penv);
    inline KinBodyPtr shared_kinbody() {
        return boost::static_pointer_cast<KinBody>(shared_from_this());
    }
    inline KinBodyConstPtr shared_kinbody_const() const {
        return boost::static_pointer_cast<KinBody const>(shared_from_this());
    }

    /// \deprecated (12/12/11)
    virtual void SetPhysicsData(UserDataPtr pdata) RAVE_DEPRECATED {
        SetUserData("_physics_", pdata);
    }
    /// \deprecated (12/12/11)
    virtual void SetCollisionData(UserDataPtr pdata) RAVE_DEPRECATED {
        SetUserData("_collision_", pdata);
    }

    /// \deprecated (12/12/11)
    virtual void SetViewerData(UserDataPtr pdata) RAVE_DEPRECATED {
        SetUserData("_viewer_",pdata);
    }
    virtual void SetManageData(ManageDataPtr pdata) {
        _pManageData = pdata;
    }

    /** \brief Final post-processing stage before a kinematics body can be used.

        This method is called after the body is finished being initialized with data and before being added to the environment. Also builds the hashes. Builds the internal hierarchy and kinematic body hash.

        Avoids making specific calls on the collision checker (like CheckCollision) or physics engine (like simulating velocities/torques) since this information can change depending on the attached plugin.
     */
    virtual void _ComputeInternalInformation();

    /// \brief returns the dof velocities and link velocities
    ///
    /// \param[in] usebaselinkvelocity if true, will compute all velocities using the base link velocity. otherwise will assume it is 0
    virtual void _ComputeDOFLinkVelocities(std::vector<dReal>& dofvelocities, std::vector<std::pair<Vector,Vector> >& linkvelocities, bool usebaselinkvelocity=true) const;

    /// \brief Computes accelerations of the links given all the necessary data of the robot. \see GetLinkAccelerations
    ///
    /// for passive joints that are not mimic and are not static, will call Joint::GetVelocities to get their initial velocities (this is state dependent!)
    /// \param dofvelocities if size is 0, will assume all velocities are 0
    /// \param dofaccelerations if size is 0, will assume all accelerations are 0
    virtual void _ComputeLinkAccelerations(const std::vector<dReal>& dofvelocities, const std::vector<dReal>& dofaccelerations, const std::vector< std::pair<Vector, Vector> >& linkvelocities, std::vector<std::pair<Vector,Vector> >& linkaccelerations, const Vector& gravity) const;

    /// \brief Called to notify the body that certain groups of parameters have been changed.
    ///
    /// This function in calls every registers calledback that is tracking the changes. It also
    /// recomputes the hashes if geometry changed.
    virtual void _ParametersChanged(int parameters);

    /// \brief Return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    virtual bool _IsAttached(KinBodyConstPtr body, std::set<KinBodyConstPtr>& setChecked) const;

    /// \brief adds an attached body
    virtual void _AttachBody(KinBodyPtr body);

    /// \brief removes an attached body
    ///
    /// \return true if body was successfully found and removed
    virtual bool _RemoveAttachedBody(KinBodyPtr body);

    /// \brief resets cached information dependent on the collision checker (usually called when the collision checker is switched or some big mode is set.
    virtual void _ResetInternalCollisionCache();

    std::string _name; ///< name of body
    std::vector<JointPtr> _vecjoints; ///< \see GetJoints
    std::vector<JointPtr> _vTopologicallySortedJoints; ///< \see GetDependencyOrderedJoints
    std::vector<JointPtr> _vTopologicallySortedJointsAll; ///< Similar to _vDependencyOrderedJoints except includes _vecjoints and _vPassiveJoints
    std::vector<int> _vTopologicallySortedJointIndicesAll; ///< the joint indices of the joints in _vTopologicallySortedJointsAll. Passive joint indices have _vecjoints.size() added to them.
    std::vector<JointPtr> _vDOFOrderedJoints; ///< all joints of the body ordered on how they are arranged within the degrees of freedom
    std::vector<LinkPtr> _veclinks; ///< \see GetLinks
    std::vector<int> _vDOFIndices; ///< cached start joint indices, indexed by dof indices
    std::vector<std::pair<int16_t,int16_t> > _vAllPairsShortestPaths; ///< all-pairs shortest paths through the link hierarchy. The first value describes the parent link index, and the second value is an index into _vecjoints or _vPassiveJoints. If the second value is greater or equal to  _vecjoints.size() then it indexes into _vPassiveJoints.
    std::vector<int8_t> _vJointsAffectingLinks; ///< joint x link: (jointindex*_veclinks.size()+linkindex). entry is non-zero if the joint affects the link in the forward kinematics. If negative, the partial derivative of ds/dtheta should be negated.
    std::vector< std::vector< std::pair<LinkPtr,JointPtr> > > _vClosedLoops; ///< \see GetClosedLoops
    std::vector< std::vector< std::pair<int16_t,int16_t> > > _vClosedLoopIndices; ///< \see GetClosedLoops
    std::vector<JointPtr> _vPassiveJoints; ///< \see GetPassiveJoints()
    std::set<int> _setAdjacentLinks; ///< a set of which links are connected to which if link i and j are connected then
                                     ///< i|(j<<16) will be in the set where i<j.
    std::vector< std::pair<std::string, std::string> > _vForcedAdjacentLinks; ///< internally stores forced adjacent links
    std::list<KinBodyWeakPtr> _listAttachedBodies; ///< list of bodies that are directly attached to this body (can have duplicates)
    mutable std::list<UserDataWeakPtr> _listRegisteredCallbacks; ///< callbacks to call when particular properties of the body change. the registration/deregistration of the list can happen at any point and does not modify the kinbody state exposed to the user, hence it is mutable

    mutable boost::array<std::set<int>, 4> _setNonAdjacentLinks; ///< contains cached versions of the non-adjacent links depending on values in AdjacentOptions. Declared as mutable since data is cached.
    mutable int _nNonAdjacentLinkCache; ///< specifies what information is currently valid in the AdjacentOptions.  Declared as mutable since data is cached. If 0x80000000 (ie < 0), then everything needs to be recomputed including _setNonAdjacentLinks[0].
    std::vector<Transform> _vInitialLinkTransformations; ///< the initial transformations of each link specifying at least one pose where the robot is collision free

    ConfigurationSpecification _spec;

    int _environmentid; ///< \see GetEnvironmentId
    mutable int _nUpdateStampId; ///< \see GetUpdateStamp
    int _nParametersChanged; ///< set of parameters that changed and need callbacks
    ManageDataPtr _pManageData;
    uint32_t _nHierarchyComputed; ///< true if the joint heirarchy and other cached information is computed
    bool _bMakeJoinedLinksAdjacent;
private:
    mutable std::string __hashkinematics;
    mutable std::vector<dReal> _vTempJoints;
    virtual const char* GetHash() const {
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
};

} // end namespace OpenRAVE

#endif
