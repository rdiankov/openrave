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
 */
#ifndef OPENRAVE_KINBODY_H
#define OPENRAVE_KINBODY_H

/// declare function parser class from fparser library
template<typename Value_t> class FunctionParserBase;

namespace OpenRAVE {

/** \brief <b>[interface]</b> A kinematic body of links and joints. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_kinbody.
    \ingroup interfaces
 */
class OPENRAVE_API KinBody : public InterfaceBase
{
public:
    /// \brief A set of properties for the kinbody. These properties are used to describe a set of variables used in KinBody.
    enum KinBodyProperty {
        Prop_Joints=0x1,     ///< all properties of all joints
        Prop_JointLimits=0x2|Prop_Joints,     ///< regular limits
        Prop_JointOffset=0x4|Prop_Joints,
        Prop_JointProperties=0x8|Prop_Joints,     ///< max velocity, max acceleration, resolution, max torque
        Prop_Links=0x10,     ///< all properties of all links
        Prop_Name=0x20,     ///< name changed
        Prop_LinkDraw=0x40,     ///< toggle link geometries rendering
        Prop_LinkGeometry=0x80|Prop_Links,     ///< the geometry of the link changed
        Prop_JointMimic=0x100|Prop_Joints,     ///< joint mimic equations
        Prop_JointAccelerationVelocityLimits=0x200|Prop_Joints,     ///< velocity + acceleration
        Prop_LinkStatic=0x400|Prop_Links,     ///< static property of link changed
        // robot only
        Prop_RobotManipulators = 0x00010000,     ///< [robot only] all properties of all manipulators
        Prop_Manipulators = 0x00010000,
        Prop_RobotSensors = 0x00020000,     ///< [robot only] all properties of all sensors
        Prop_Sensors = 0x00020000,
        Prop_RobotSensorPlacement = 0x00040000,     ///< [robot only] relative sensor placement of sensors
        Prop_SensorPlacement = 0x00040000,
        Prop_RobotActiveDOFs = 0x00080000,     ///< [robot only] active dofs changed
        Prop_RobotManipulatorTool = 0x00100000, ///< [robot only] the tool coordinate system changed
    };

    /// \brief A rigid body holding all its collision and rendering data.
    class OPENRAVE_API Link : public boost::enable_shared_from_this<Link>
    {
public:
        Link(KinBodyPtr parent);         ///< pass in a ODE world
        virtual ~Link();

        /// \brief User data for trimesh geometries. Vertices are defined in counter-clockwise order for outward pointing faces.
        class OPENRAVE_API TRIMESH
        {
public:
            std::vector<Vector> vertices;
            std::vector<int> indices;

            void ApplyTransform(const Transform& t);
            void ApplyTransform(const TransformMatrix& t);

            /// append another TRIMESH to this tri mesh
            void Append(const TRIMESH& mesh);
            void Append(const TRIMESH& mesh, const Transform& trans);

            AABB ComputeAABB() const;
            void serialize(std::ostream& o, int options=0) const;

            friend OPENRAVE_API std::ostream& operator<<(std::ostream& O, const TRIMESH &trimesh);
            friend OPENRAVE_API std::istream& operator>>(std::istream& I, TRIMESH& trimesh);
        };

        /// Describes the properties of a basic geometric primitive.
        /// Contains everything associated with a physical body along with a seprate (optional) render file.
        class OPENRAVE_API GEOMPROPERTIES
        {
public:
            /// \brief The type of geometry primitive.
            enum GeomType {
                GeomNone = 0,
                GeomBox = 1,
                GeomSphere = 2,
                GeomCylinder = 3,
                GeomTrimesh = 4,
            };

            GEOMPROPERTIES(boost::shared_ptr<Link> parent);
            virtual ~GEOMPROPERTIES() {
            }

            /// \brief Local transformation of the geom primitive with respect to the link's coordinate system.
            inline const Transform& GetTransform() const {
                return _t;
            }
            inline GeomType GetType() const {
                return _type;
            }
            inline const Vector& GetRenderScale() const {
                return vRenderScale;
            }

            /// \brief render resource file, should be transformed by _t before rendering
            ///
            /// If the value is "__norenderif__:x", then the viewer should not render the object if it supports *.x files where"x" is the file extension.
            inline const std::string& GetRenderFilename() const {
                return _renderfilename;
            }
            inline float GetTransparency() const {
                return ftransparency;
            }
            inline bool IsDraw() const {
                return _bDraw;
            }
            inline bool IsModifiable() const {
                return _bModifiable;
            }

            inline dReal GetSphereRadius() const {
                return vGeomData.x;
            }
            inline dReal GetCylinderRadius() const {
                return vGeomData.x;
            }
            inline dReal GetCylinderHeight() const {
                return vGeomData.y;
            }
            inline const Vector& GetBoxExtents() const {
                return vGeomData;
            }
            inline const RaveVector<float>& GetDiffuseColor() const {
                return diffuseColor;
            }
            inline const RaveVector<float>& GetAmbientColor() const {
                return ambientColor;
            }

            /// \brief collision data of the specific object in its local coordinate system.
            ///
            /// Should be transformed by \ref GEOMPROPERTIES::GetTransform() before rendering.
            /// For spheres and cylinders, an appropriate discretization value is chosen.
            inline const TRIMESH& GetCollisionMesh() const {
                return collisionmesh;
            }

            /// \brief returns an axis aligned bounding box given that the geometry is transformed by trans
            virtual AABB ComputeAABB(const Transform& trans) const;
            virtual void serialize(std::ostream& o, int options) const;

            /// \brief sets a new collision mesh and notifies every registered callback about it
            virtual void SetCollisionMesh(const TRIMESH& mesh);
            /// \brief sets a drawing and notifies every registered callback about it
            virtual void SetDraw(bool bDraw);
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
            /// triangulates the geometry object and initializes collisionmesh. GeomTrimesh types must already be triangulated
            /// \param fTessellation to control how fine the triangles need to be. 1.0f is the default value
            bool InitCollisionMesh(float fTessellation=1);

            boost::weak_ptr<Link> _parent;
            Transform _t;             ///< see \ref GetTransform
            Vector vGeomData;             ///< for boxes, first 3 values are extents
            ///< for sphere it is radius
            ///< for cylinder, first 2 values are radius and height
            ///< for trimesh, none
            RaveVector<float> diffuseColor, ambientColor;             ///< hints for how to color the meshes
            TRIMESH collisionmesh;             ///< see \ref GetCollisionMesh
            GeomType _type;                     ///< the type of geometry primitive
            std::string _renderfilename;              ///< \see ref GetRenderFilename
            Vector vRenderScale;             ///< render scale of the object (x,y,z)
            float ftransparency;             ///< value from 0-1 for the transparency of the rendered object, 0 is opaque

            bool _bDraw;                     ///< if true, object is drawn as part of the 3d model (default is true)
            bool _bModifiable;             ///< if true, object geometry can be dynamically modified (default is true)

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
            friend class ColladaReader;
            friend class OpenRAVEXMLParser::LinkXMLReader;
            friend class OpenRAVEXMLParser::KinBodyXMLReader;
#else
            friend class ::ColladaReader;
            friend class ::OpenRAVEXMLParser::LinkXMLReader;
            friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
#endif
#endif
            friend class KinBody;
            friend class KinBody::Link;
        };

        inline const std::string& GetName() const {
            return _name;
        }

        /// \brief Indicates a static body that does not move with respect to the root link.
        ///
        //// Static should be used when an object has infinite mass and
        ///< shouldn't be affected by physics (including gravity). Collision still works.
        inline bool IsStatic() const {
            return _bStatic;
        }

        /// \brief returns true if the link is enabled. \see Enable
        virtual bool IsEnabled() const;

        /// \brief Enables a Link. An enabled link takes part in collision detection and physics simulations
        virtual void Enable(bool enable);

        /// \brief parent body that link belong to.
        inline KinBodyPtr GetParent() const {
            return KinBodyPtr(_parent);
        }

        /// \brief unique index into parent KinBody::GetLinks vector
        inline int GetIndex() const {
            return _index;
        }
        inline const TRIMESH& GetCollisionData() const {
            return collision;
        }

        /// \brief Compute the aabb of all the geometries of the link in the world coordinate system
        virtual AABB ComputeAABB() const;

        /// \brief Return the current transformation of the link in the world coordinate system.
        inline Transform GetTransform() const {
            return _t;
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

        /// \return center of mass offset in the link's local coordinate frame
        inline Vector GetCOMOffset() const {
            return _transMass.trans;
        }
        inline const TransformMatrix& GetInertia() const {
            return _transMass;
        }
        inline dReal GetMass() const {
            return _mass;
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

        /// get the velocity of the link
        /// \param[out] linearvel the translational velocity
        /// \param[out] angularvel is the rotation axis * angular speed
        virtual void GetVelocity(Vector& linearvel, Vector& angularvel) const;

        //typedef std::list<GEOMPROPERTIES>::iterator GeomPtr;
        //typedef std::list<GEOMPROPERTIES>::const_iterator GeomConstPtr;

        /// \brief returns a list of all the geometry objects.
        inline const std::list<GEOMPROPERTIES>& GetGeometries() const {
            return _listGeomProperties;
        }
        virtual GEOMPROPERTIES& GetGeometry(int index);

        /// \brief swaps the current geometries with the new geometries.
        ///
        /// This gives a user control for dynamically changing the object geometry. Note that the kinbody/robot hash could change.
        virtual void SwapGeometries(std::list<GEOMPROPERTIES>& listNewGeometries);

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

protected:
        /// \brief Updates the cached information due to changes in the collision data.
        virtual void _Update();

        Transform _t;                   ///< \see GetTransform
        TransformMatrix _transMass;         ///< the 3x3 inertia and center of mass of the link in the link's coordinate system
        dReal _mass;
        TRIMESH collision;         ///< triangles for collision checking, triangles are always the triangulation
                                   ///< of the body when it is at the identity transformation

        std::string _name;             ///< optional link name
        std::list<GEOMPROPERTIES> _listGeomProperties;         ///< \see GetGeometries

        bool _bStatic;               ///< \see IsStatic
        bool _bIsEnabled;         ///< \see IsEnabled

private:
        /// Sensitive variables that should not be modified.
        /// @name Private Link Variables
        //@{
        int _index;                  ///< \see GetIndex
        KinBodyWeakPtr _parent;         ///< \see GetParent
        std::vector<int> _vParentLinks;         ///< \see GetParentLinks, IsParentLink
        std::vector<int> _vRigidlyAttachedLinks;         ///< \see IsRigidlyAttached, GetRigidlyAttachedLinks
        //@}
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class ColladaReader;
        friend class OpenRAVEXMLParser::LinkXMLReader;
        friend class OpenRAVEXMLParser::KinBodyXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
#else
        friend class ::ColladaReader;
        friend class ::OpenRAVEXMLParser::LinkXMLReader;
        friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
#endif
#endif
        friend class KinBody;
    };
    typedef boost::shared_ptr<KinBody::Link> LinkPtr;
    typedef boost::shared_ptr<KinBody::Link const> LinkConstPtr;
    typedef boost::weak_ptr<KinBody::Link> LinkWeakPtr;

    /// \brief Information about a joint that controls the relationship between two links.
    class OPENRAVE_API Joint : public boost::enable_shared_from_this<Joint>
    {
public:
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
        };

        Joint(KinBodyPtr parent);
        virtual ~Joint();

        /// \brief The unique name of the joint
        inline const std::string& GetName() const {
            return _name;
        }

        inline dReal GetMaxVel(int iaxis=0) const {
            return _vmaxvel[iaxis];
        }
        inline dReal GetMaxAccel(int iaxis=0) const {
            return _vmaxaccel[iaxis];
        }
        inline dReal GetMaxTorque(int iaxis=0) const {
            return _vmaxtorque[iaxis];
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

        inline JointType GetType() const {
            return _type;
        }

        /// \brief The discretization of the joint used when line-collision checking.
        ///
        /// The resolutions are set as large as possible such that the joint will not go through obstacles of determined size.
        inline dReal GetResolution() const {
            return fResolution;
        }
        virtual void SetResolution(dReal resolution);

        /// \brief The degrees of freedom of the joint. Each joint supports a max of 3 degrees of freedom.
        virtual int GetDOF() const;

        /// \brief Return true if joint axis has an identification at some of its lower and upper limits.
        ///
        /// An identification of the lower and upper limits means that once the joint reaches its upper limits, it is also
        /// at its lower limit. The most common identification on revolute joints at -pi and pi. 'circularity' means the
        /// joint does not stop at limits.
        /// Although currently not developed, it could be possible to support identification for joints that are not revolute.
        virtual bool IsCircular(int iaxis) const {
            return _bIsCircular.at(iaxis);
        }

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

        /// \brief Add effort (force or torque) to the joint
        virtual void AddTorque(const std::vector<dReal>& torques);

        /// \brief The anchor of the joint in global coordinates.
        virtual Vector GetAnchor() const;

        /// \brief The axis of the joint in global coordinates
        ///
        /// \param[in] axis the axis to get
        virtual Vector GetAxis(int axis = 0) const;

        /** \brief Returns the limits of the joint

            \param[out] vLowerLimit the lower limits
            \param[out] vUpperLimit the upper limits
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend=false) const;

        /// \brief \see GetLimits
        virtual void SetLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper);

        /// \deprecated (11/1/1)
        virtual void SetJointLimits(const std::vector<dReal>& lower, const std::vector<dReal>& upper) RAVE_DEPRECATED {
            SetLimits(lower,upper);
        }

        /** \brief Returns the max velocities of the joint

            \param[out] the max velocity
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetVelocityLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        virtual void GetVelocityLimits(std::vector<dReal>& vlower, std::vector<dReal>& vupper, bool bAppend=false) const;

        /// \brief \see GetVelocityLimits
        virtual void SetVelocityLimits(const std::vector<dReal>& vmax);

        /** \brief Returns the max accelerations of the joint

            \param[out] the max acceleration
            \param[in] bAppend if true will append to the end of the vector instead of erasing it
         */
        virtual void GetAccelerationLimits(std::vector<dReal>& vmax, bool bAppend=false) const;

        /// \brief \see GetAccelerationLimits
        virtual void SetAccelerationLimits(const std::vector<dReal>& vmax);

        /// \brief The weight associated with a joint's axis for computing a distance in the robot configuration space.
        virtual dReal GetWeight(int axis=0) const;
        /// \brief \see GetWeight
        virtual void SetWeights(const std::vector<dReal>& weights);

        /// \brief Return internal offset parameter that determines the branch the angle centers on
        ///
        /// Wrap offsets are needed for rotation joints since the range is limited to 2*pi.
        /// This allows the wrap offset to be set so the joint can function in [-pi+offset,pi+offset]..
        /// \param iaxis the axis to get the offset from
        inline dReal GetWrapOffset(int iaxis=0) const {
            return _voffsets.at(iaxis);
        }

        inline dReal GetOffset(int iaxis=0) const RAVE_DEPRECATED {
            return GetWrapOffset(iaxis);
        }

        /// \brief \see GetWrapOffset
        virtual void SetWrapOffset(dReal offset, int iaxis=0);
        /// \deprecated (11/1/16)
        virtual void SetOffset(dReal offset, int iaxis=0) RAVE_DEPRECATED {
            SetWrapOffset(offset,iaxis);
        }

        virtual void serialize(std::ostream& o, int options) const;

        /// @name Internal Hierarchy Methods
        //@{
        /// \brief Return the parent link which the joint measures its angle off from (either GetFirstAttached() or GetSecondAttached())
        virtual LinkPtr GetHierarchyParentLink() const;
        /// \brief Return the child link whose transformation is computed by this joint's values (either GetFirstAttached() or GetSecondAttached())
        virtual LinkPtr GetHierarchyChildLink() const;
        /// \deprecated (11/1/27)
        virtual Vector GetInternalHierarchyAnchor() const RAVE_DEPRECATED;
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

            Set 'format' to "mathml". The joint variables are specified with <csymbol>. If a targetted joint has more than one degree of freedom, then axis is suffixed with _\%d. If 'type' is 1 or 2, the partial derivatives are outputted as consecutive <math></math> tags in the same order as \ref MIMIC::_vdofformat
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

protected:
        boost::array<Vector,3> _vaxes;                ///< axes in body[0]'s or environment coordinate system used to define joint movement
        Vector vanchor;                 ///< anchor of the joint, this is only used to construct the internal left/right matrices
        dReal fResolution;              ///< interpolation resolution
        boost::array<dReal,3> _vmaxvel;                  ///< the soft maximum velocity (rad/s) to move the joint when planning
        boost::array<dReal,3> fHardMaxVel;              ///< the hard maximum velocity, robot cannot exceed this velocity. used for verification checking
        boost::array<dReal,3> _vmaxaccel;                ///< the maximum acceleration (rad/s^2) of the joint
        boost::array<dReal,3> _vmaxtorque;               ///< maximum torque (N.m, kg m^2/s^2) that can be applied to the joint
        boost::array<dReal,3> _vweights;                ///< the weights of the joint for computing distance metrics.
        boost::array<dReal,3> _voffsets;                   ///< \see GetOffset
        boost::array<dReal,3> _vlowerlimit, _vupperlimit;         ///< joint limits
        /// \brief Holds mimic information about position, velocity, and acceleration of one axis of the joint.
        ///
        /// In every array, [0] is position, [1] is velocity, [2] is acceleration.
        struct OPENRAVE_API MIMIC
        {
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
            std::vector< DOFFormat > _vdofformat;         ///< the format of the values the equation takes order is important.
            struct DOFHierarchy
            {
                int16_t dofindex;         ///< >=0 dof index
                uint16_t dofformatindex;         ///< index into _vdofformat to follow the computation
                bool operator ==(const DOFHierarchy& r) const {
                    return dofindex==r.dofindex && dofformatindex == r.dofformatindex;
                }
            };
            std::vector<DOFHierarchy> _vmimicdofs;         ///< all dof indices that the equations depends on. DOFHierarchy::dofindex can repeat
            boost::shared_ptr<FunctionParserBase<dReal> > _posfn;
            std::vector<boost::shared_ptr<FunctionParserBase<dReal> > > _velfns, _accelfns;         ///< the velocity and acceleration partial derivatives with respect to each of the values in _vdofformat
            boost::array< std::string, 3>  _equations;         ///< the original equations
        };
        boost::array< boost::shared_ptr<MIMIC>,3> _vmimic;          ///< the mimic properties of each of the joint axes. It is theoretically possible for a multi-dof joint to have one axes mimiced and the others free. When cloning, is it ok to copy this and assume it is constant?

        /** \brief computes the partial velocities with respect to all dependent DOFs specified by MIMIC::_vmimicdofs.

            If the joint is not mimic, then just returns its own index
            \param[out] vpartials A list of dofindex/velocity_partial pairs. The final velocity is computed by taking the dot product. The dofindices do not repeat.
            \param[in] iaxis the axis
            \param[in,out] vcachedpartials set of cached partials for each degree of freedom
         */
        virtual void _ComputePartialVelocities(std::vector<std::pair<int,dReal> >& vpartials, int iaxis, std::map< std::pair<MIMIC::DOFFormat, int>, dReal >& mapcachedpartials) const;

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

        std::string _name;         ///< \see GetName
        boost::array<bool,3> _bIsCircular;            ///< \see IsCircular
private:
        /// Sensitive variables that should not be modified.
        /// @name Private Joint Variables
        //@{
        int dofindex;                   ///< the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        int jointindex;                 ///< the joint index into KinBody::_vecjoints
        JointType _type;
        bool _bActive;                 ///< if true, should belong to the DOF of the body, unless it is a mimic joint (_ComputeInternalInformation decides this)

        KinBodyWeakPtr _parent;               ///< body that joint belong to
        boost::array<LinkPtr,2> _attachedbodies;         ///< attached bodies. The link in [0] is computed first in the hierarchy before the other body.
        Transform _tRight, _tLeft;         ///< transforms used to get body[1]'s transformation with respect to body[0]'s: Tbody1 = Tbody0 * tLeft * JointOffsetLeft * JointRotation * JointOffsetRight * tRight
        Transform _tRightNoOffset, _tLeftNoOffset;         ///< same as _tLeft and _tRight except it doesn't not include the offset
        Transform _tinvRight, _tinvLeft;         ///< the inverse transformations of tRight and tLeft
        bool _bInitialized;
        //@}
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class ColladaReader;
        friend class ColladaWriter;
        friend class OpenRAVEXMLParser::JointXMLReader;
        friend class OpenRAVEXMLParser::KinBodyXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
#else
        friend class ::ColladaReader;
        friend class ::ColladaWriter;
        friend class ::OpenRAVEXMLParser::JointXMLReader;
        friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
#endif
#endif
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
        UserDataPtr pviewerdata, puserdata;
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
        Save_LinkTransformation=0x00000001,     ///< [default] save link transformations
        Save_LinkEnable=0x00000002,     ///< [default] save link enable states
        Save_LinkVelocities=0x00000004,     ///< save the link velocities
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
        virtual void Restore();
protected:
        int _options;         ///< saved options
        std::vector<Transform> _vLinkTransforms;
        std::vector<uint8_t> _vEnabledLinks;
        std::vector<std::pair<Vector,Vector> > _vLinkVelocities;
        KinBodyPtr _pbody;
private:
        virtual void _RestoreKinBody();
    };

    virtual ~KinBody();

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_KinBody;
    }

    virtual void Destroy();

    /// \deprecated (11/02/18) \see EnvironmentBase::ReadKinBodyXMLFile
    virtual bool InitFromFile(const std::string& filename, const AttributesList& atts = AttributesList()) RAVE_DEPRECATED;
    /// \deprecated (11/02/18) \see EnvironmentBase::ReadKinBodyXMLData
    virtual bool InitFromData(const std::string& data, const AttributesList& atts = AttributesList()) RAVE_DEPRECATED;

    /// \brief Create a kinbody with one link composed of an array of aligned bounding boxes.
    ///
    /// \param boxes the array of aligned bounding boxes that will comprise of the body
    /// \param draw if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<AABB>& boxes, bool draw);

    /// \brief Create a kinbody with one link composed of an array of oriented bounding boxes.
    ///
    /// \param boxes the array of oriented bounding boxes that will comprise of the body
    /// \param draw if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<OBB>& boxes, bool draw);

    /// \brief Create a kinbody with one link composed of an array of spheres
    ///
    /// \param spheres the XYZ position of the spheres with the W coordinate representing the individual radius
    virtual bool InitFromSpheres(const std::vector<Vector>& spheres, bool draw);

    /// \brief Create a kinbody with one link composed of a triangle mesh surface
    ///
    /// \param trimesh the triangle mesh
    /// \param draw if true, will be rendered in the scene
    virtual bool InitFromTrimesh(const Link::TRIMESH& trimesh, bool draw);

    /// \brief Create a kinbody with one link composed of a list of geometries
    ///
    /// \param geometries In order to save memory, the geometries in this list are transferred to the link. After function completes, the size should be 0.
    /// \param draw if true, will be rendered in the scene
    bool InitFromGeometries(std::list<KinBody::Link::GEOMPROPERTIES>& geometries, bool draw);

    /// \brief Unique name of the robot.
    virtual const std::string& GetName() const {
        return _name;
    }

    /// \brief Set the name of the robot, notifies the environment and checks for uniqueness.
    virtual void SetName(const std::string& name);

    /// Methods for accessing basic information about joints
    /// @name Basic Information
    //@{

    /// \brief Number controllable degrees of freedom of the body.
    ///
    /// Only uses _vecjoints and last joint for computation, so can work before _ComputeInternalInformation is called.
    virtual int GetDOF() const;

    /// \brief Returns all the joint values as organized by the DOF indices.
    virtual void GetDOFValues(std::vector<dReal>& v) const;
    /// \brief Returns all the joint velocities as organized by the DOF indices.
    virtual void GetDOFVelocities(std::vector<dReal>& v) const;
    /// \brief Returns all the joint limits as organized by the DOF indices.
    virtual void GetDOFLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit) const;
    /// \brief Returns all the joint velocity limits as organized by the DOF indices.
    virtual void GetDOFVelocityLimits(std::vector<dReal>& lowerlimit, std::vector<dReal>& upperlimit) const;
    /// \brief Returns the max velocity for each DOF
    virtual void GetDOFVelocityLimits(std::vector<dReal>& maxvelocities) const;
    /// \brief Returns the max acceleration for each DOF
    virtual void GetDOFAccelerationLimits(std::vector<dReal>& maxaccelerations) const;

    /// \deprecated (11/05/26)
    virtual void GetDOFMaxVel(std::vector<dReal>& v) const RAVE_DEPRECATED {
        GetDOFVelocityLimits(v);
    }
    virtual void GetDOFMaxAccel(std::vector<dReal>& v) const RAVE_DEPRECATED {
        GetDOFAccelerationLimits(v);
    }
    virtual void GetDOFMaxTorque(std::vector<dReal>& v) const;
    virtual void GetDOFResolutions(std::vector<dReal>& v) const;
    virtual void GetDOFWeights(std::vector<dReal>& v) const;

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

    /// \deprecated \see Link::GetRigidlyAttachedLinks (10/12/12)
    virtual void GetRigidlyAttachedLinks(int linkindex, std::vector<LinkPtr>& vattachedlinks) const RAVE_DEPRECATED;

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
    virtual void SubtractDOFValues(std::vector<dReal>& values1, const std::vector<dReal>& values2) const;

    /// \deprecated (01/01/11)
    virtual void SubtractJointValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const RAVE_DEPRECATED {
        SubtractDOFValues(q1,q2);
    }

    /// \brief Adds a torque to every joint.
    ///
    /// \param bAdd if true, adds to previous torques, otherwise resets the torques on all bodies and starts from 0
    virtual void SetDOFTorques(const std::vector<dReal>& torques, bool add);

    /// \deprecated (11/06/17)
    virtual void SetJointTorques(const std::vector<dReal>& torques, bool add) RAVE_DEPRECATED {
        SetDOFTorques(torques,add);
    }

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
        \param[in] vDOFVelocities - velocities of each of the degrees of freeom
        \param checklimits if true, will excplicitly check the joint velocity limits before setting the values.
     */
    virtual void SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, const Vector& linearvel, const Vector& angularvel,bool checklimits = false);

    /// \brief Sets the velocity of the joints.
    ///
    /// Copies the current velocity of the base link and calls SetDOFVelocities(linearvel,angularvel,vDOFVelocities)
    /// \param[in] vDOFVelocity - velocities of each of the degrees of freeom
    /// \praam checklimits if true, will excplicitly check the joint velocity limits before setting the values.
    virtual void SetDOFVelocities(const std::vector<dReal>& vDOFVelocities, bool checklimits = false);

    /// \brief Returns the linear and angular velocities for each link
    virtual void GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const;

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

    /// \brief Enables or disables the bodies.
    virtual void Enable(bool enable);

    /// \deprecated (10/09/08)
    virtual void EnableLink(LinkPtr plink, bool bEnable) RAVE_DEPRECATED {
        plink->Enable(bEnable);
    }

    /// \return true if any link of the KinBody is enabled
    virtual bool IsEnabled() const;

    /// \brief Sets the joint values of the robot.
    ///
    /// \param values the values to set the joint angles (ordered by the dof indices)
    /// \praam checklimits if true, will excplicitly check the joint limits before setting the values.
    virtual void SetDOFValues(const std::vector<dReal>& values, bool checklimits = false);

    virtual void SetJointValues(const std::vector<dReal>& values, bool checklimits = false) {
        SetDOFValues(values,checklimits);
    }

    /// \brief Sets the joint values and transformation of the body.
    ///
    /// \param values the values to set the joint angles (ordered by the dof indices)
    /// \param transform represents the transformation of the first body.
    /// \praam checklimits if true, will excplicitly check the joint limits before setting the values.
    virtual void SetDOFValues(const std::vector<dReal>& values, const Transform& transform, bool checklimits = false);

    virtual void SetJointValues(const std::vector<dReal>& values, const Transform& transform, bool checklimits = false)
    {
        SetDOFValues(values,transform,checklimits);
    }

    /// \brief sets the transformations of all the links at once
    virtual void SetLinkTransformations(const std::vector<Transform>& transforms);

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
    /// \param linkindex of the link that the rotation is attached to
    /// \param position position in world space where to compute derivatives from.
    /// \param vjacobian 3xDOF matrix
    virtual void CalculateJacobian(int linkindex, const Vector& offset, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateJacobian(int linkindex, const Vector& offset, std::vector<dReal>& pfJacobian) const;

    /// \brief Computes the rotational jacobian as a quaternion with respect to an initial rotation.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param qInitialRot the rotation in world space whose derivative to take from.
    /// \param vjacobian 4xDOF matrix
    virtual void CalculateRotationJacobian(int linkindex, const Vector& quat, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateRotationJacobian(int linkindex, const Vector& quat, std::vector<dReal>& pfJacobian) const;

    /// \brief Computes the angular velocity jacobian of a specified link about the axes of world coordinates.
    ///
    /// \param linkindex of the link that the rotation is attached to
    /// \param vjacobian 3xDOF matrix
    virtual void CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateAngularVelocityJacobian(int linkindex, std::vector<dReal>& pfJacobian) const;

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

    /// \see SetViewerData
    virtual UserDataPtr GetViewerData() const {
        return _pViewerData;
    }
    /// \deprecated (11/09/21)
    virtual UserDataPtr GetGuiData() const RAVE_DEPRECATED {
        return GetViewerData();
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

    /// \see SetPhysicsData
    virtual UserDataPtr GetPhysicsData() const {
        return _pPhysicsData;
    }
    /// \brief SetCollisionData
    virtual UserDataPtr GetCollisionData() const {
        return _pCollisionData;
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
    virtual UserDataPtr RegisterChangeCallback(int properties, const boost::function<void()>& callback);

    virtual void serialize(std::ostream& o, int options) const;

    /// \brief A md5 hash unique to the particular kinematic and geometric structure of a KinBody.
    ///
    /// This 32 byte string can be used to check if two bodies have the same kinematic structure and can be used
    /// to index into tables when looking for body-specific models. OpenRAVE stores all
    /// such models in the OPENRAVE_HOME directory (usually ~/.openrave), indexed by the particular robot/body hashes.
    /// \return md5 hash string of kinematics/geometry
    virtual const std::string& GetKinematicsGeometryHash() const;

    /// \deprecated (10/11/18)
    virtual void SetJointVelocities(const std::vector<dReal>& pJointVelocities) RAVE_DEPRECATED {
        SetDOFVelocities(pJointVelocities);
    }

    /// \deprecated (10/11/18)
    virtual void GetVelocity(Vector& linearvel, Vector& angularvel) const RAVE_DEPRECATED;

    /// \brief Sets the joint offsets so that the current configuration becomes the new zero state of the robot.
    ///
    /// When this function returns, the returned DOF values should be all zero for controllable joints.
    /// Mimic equations will use the new offsetted values when computing their joints.
    /// This is primarily used for calibrating a robot's zero position
    virtual void SetZeroConfiguration();

    /// Functions dealing with configuration specifications
    /// @name Configuration Specification API
    //@{

    /// \brief return the configuration specification of the joint values and transform
    virtual const ConfigurationSpecification& GetConfigurationSpecification() const;

    /// \brief return the configuration specification of the specified joint indices.
    ///
    /// Note that the return type is by-value, so should not be used in iteration
    virtual ConfigurationSpecification GetConfigurationSpecificationIndices(const std::vector<int>& indices) const;

    /// \brief sets joint values and transform of the body using configuration values as specified by \ref GetConfigurationSpecification()
    ///
    /// \param itvalues the iterator to the vector containing the dof values. Must have GetConfigurationSpecification().GetDOF() values!
    virtual void SetConfigurationValues(std::vector<dReal>::const_iterator itvalues, bool checklimits = false);

    /// \brief returns the configuration values as specified by \ref GetConfigurationSpecification()
    virtual void GetConfigurationValues(std::vector<dReal>& v) const;

    //@}

protected:
    /// \brief constructors declared protected so that user always goes through environment to create bodies
    KinBody(InterfaceType type, EnvironmentBasePtr penv);
    inline KinBodyPtr shared_kinbody() {
        return boost::static_pointer_cast<KinBody>(shared_from_this());
    }
    inline KinBodyConstPtr shared_kinbody_const() const {
        return boost::static_pointer_cast<KinBody const>(shared_from_this());
    }

    /// \brief custom data managed by the current active physics engine, should be set only by PhysicsEngineBase
    virtual void SetPhysicsData(UserDataPtr pdata) {
        _pPhysicsData = pdata;
    }
    /// \brief custom data managed by the current active collision checker, should be set only by CollisionCheckerBase
    virtual void SetCollisionData(UserDataPtr pdata) {
        _pCollisionData = pdata;
    }
    /// \brief custom data managed by the current active viewer, should be set only by ViewerBase
    virtual void SetViewerData(UserDataPtr pdata) {
        _pViewerData = pdata;
    }
    virtual void SetManageData(ManageDataPtr pdata) {
        _pManageData = pdata;
    }

    /** \brief Final post-processing stage before a kinematics body can be used.

        This method is called after the body is finished being initialized with data and before being added to the environment. Also builds the hashes. Builds the internal hierarchy and kinematic body hash.

        Avoids making specific calls on the collision checker (like CheckCollision) or physics engine (like simulating velocities/torques) since this information can change depending on the attached plugin.
     */
    virtual void _ComputeInternalInformation();

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

    /// creates the function parser connected to this body's joint values
    virtual boost::shared_ptr<FunctionParserBase<dReal> > _CreateFunctionParser();

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
    std::list<UserDataWeakPtr> _listRegisteredCallbacks; ///< callbacks to call when particular properties of the body change.

    mutable boost::array<std::set<int>, 4> _setNonAdjacentLinks; ///< contains cached versions of the non-adjacent links depending on values in AdjacentOptions. Declared as mutable since data is cached.
    mutable int _nNonAdjacentLinkCache; ///< specifies what information is currently valid in the AdjacentOptions.  Declared as mutable since data is cached. If 0x80000000 (ie < 0), then everything needs to be recomputed including _setNonAdjacentLinks[0].
    std::vector<Transform> _vInitialLinkTransformations; ///< the initial transformations of each link specifying at least one pose where the robot is collision free

    ConfigurationSpecification _spec;

    int _environmentid; ///< \see GetEnvironmentId
    mutable int _nUpdateStampId; ///< \see GetUpdateStamp
    int _nParametersChanged; ///< set of parameters that changed and need callbacks
    UserDataPtr _pViewerData; ///< \see SetViewerData
    UserDataPtr _pPhysicsData; ///< \see SetPhysicsData
    UserDataPtr _pCollisionData; ///< \see SetCollisionData
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
    friend class ColladaReader;
    friend class ColladaWriter;
    friend class OpenRAVEXMLParser::KinBodyXMLReader;
    friend class OpenRAVEXMLParser::JointXMLReader;
#else
    friend class ::Environment;
    friend class ::ColladaReader;
    friend class ::ColladaWriter;
    friend class ::OpenRAVEXMLParser::KinBodyXMLReader;
    friend class ::OpenRAVEXMLParser::JointXMLReader;
#endif
#endif

    friend class PhysicsEngineBase;
    friend class CollisionCheckerBase;
    friend class ViewerBase;
    friend class SensorSystemBase;
    friend class RaveDatabase;
    friend class ChangeCallbackData;
};

OPENRAVE_API std::ostream& operator<<(std::ostream& O, const KinBody::Link::TRIMESH& trimesh);
OPENRAVE_API std::istream& operator>>(std::istream& I, KinBody::Link::TRIMESH& trimesh);

} // end namespace OpenRAVE

#endif
