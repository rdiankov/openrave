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
/*! --------------------------------------------------------------------
  \file   kinbody.h
  \brief  Encapsulate a kinematic chain of links
 -------------------------------------------------------------------- */
#ifndef  KIN_CHAIN_H
#define  KIN_CHAIN_H

namespace OpenRAVE {

/// Encapsulate a kinematic body of links and joints
class RAVE_API KinBody : public InterfaceBase
{
public:
    /// A set of properties for the kinbody. These properties are used to describe a set of variables used in KinBody.
    enum KinBodyProperty {
        Prop_Joints=0x1, ///< all properties of all joints
        Prop_JointLimits=0x2,
        Prop_JointOffset=0x4,
        Prop_JointProperties=0x8, ///< max velocity, max acceleration, resolution, max torque
        Prop_Links=0x10, ///< all properties of all links
        Prop_Name=0x20, ///< name changed
        Prop_LinkDraw=0x40, ///< toggle link geometries rendering
        Prop_LinkGeometry=0x80, ///< the geometry of the link changed
    };

    /// rigid body defined by an arbitrary ODE body and a render object
    class RAVE_API Link : public boost::enable_shared_from_this<Link>
    {
    public:
        Link(KinBodyPtr parent); ///< pass in a ODE world
        virtual ~Link();
        
        inline const std::string& GetName() const { return name; }

        inline bool IsStatic() const { return bStatic; }
        virtual bool IsEnabled() const; ///< returns true if enabled

        ///< enables a Link. An enabled link takes part in collision detection and physics simulations
        virtual void Enable(bool bEnable);

        /// user data for trimesh geometries
        struct RAVE_API TRIMESH
        {
            std::vector<Vector> vertices;
            std::vector<int> indices;

            void ApplyTransform(const Transform& t);
            void ApplyTransform(const TransformMatrix& t);

            /// append another TRIMESH to this tri mesh
            void Append(const TRIMESH& mesh);
            void Append(const TRIMESH& mesh, const Transform& trans);

            AABB ComputeAABB() const;
            void serialize(std::ostream& o, int options) const;
        };

        /// Describes the properties of a basic geometric primitive.
        /// Contains everything associated with a physical body along with a seprate (optional) render file.
        struct RAVE_API GEOMPROPERTIES
        {
            enum GeomType {
                GeomNone = 0,
                GeomBox = 1,
                GeomSphere = 2,
                GeomCylinder = 3,
                GeomTrimesh = 4,
            };
            
            GEOMPROPERTIES(boost::shared_ptr<Link> parent);
            virtual ~GEOMPROPERTIES() {}

            inline const Transform& GetTransform() const { return _t; }
            inline GeomType GetType() const { return type; }
            inline const Vector& GetRenderScale() const { return vRenderScale; }
            inline const std::string& GetRenderFilename() const { return renderfile; }
            inline float GetTransparency() const { return ftransparency; }
            inline bool IsDraw() const { return _bDraw; }
            inline bool IsModifiable() const { return _bModifiable; }
            
            inline dReal GetSphereRadius() const { return vGeomData.x; }
            inline dReal GetCylinderRadius() const { return vGeomData.x; }
            inline dReal GetCylinderHeight() const { return vGeomData.y; }
            inline const Vector& GetBoxExtents() const { return vGeomData; }
            inline const RaveVector<float>& GetDiffuseColor() const { return diffuseColor; }
            inline const RaveVector<float>& GetAmbientColor() const { return ambientColor; }
            
            inline const TRIMESH& GetCollisionMesh() const { return collisionmesh; }
            
            virtual AABB ComputeAABB(const Transform& t) const;
            virtual void serialize(std::ostream& o, int options) const;

            /// sets a new collision mesh and notifies every registered callback about it
            virtual void SetCollisionMesh(const TRIMESH& mesh);
            /// sets a drawing and notifies every registered callback about it
            virtual void SetDraw(bool bDraw);
            virtual void SetTransparency(float f);

            /// validates the contact normal on the surface of the geometry and makes sure the normal faces "outside" of the shape.
            /// \param position the position of the contact point specified in the link's coordinate system
            /// \param normal the unit normal of the contact point specified in the link's coordinate system
            /// \return true if the normal is changed to face outside of the shape
            virtual bool ValidateContactNormal(const Vector& position, Vector& normal) const;

        protected:
            /// triangulates the geometry object and initializes collisionmesh. GeomTrimesh types must already be triangulated
            /// \param fTesselation to control how fine the triangles need to be
            /// 1.0f is the default value that 
            bool InitCollisionMesh(float fTessellation=1);

            boost::weak_ptr<Link> _parent;
            Transform _t;                ///< local transformation of the geom primitive with respect to the link's coordinate system
            Vector vGeomData; ///< for boxes, first 3 values are extents
                                         ///< for sphere it is radius
                                         ///< for cylinder, first 2 values are radius and height
                                         ///< for trimesh, none
            RaveVector<float> diffuseColor, ambientColor; ///< hints for how to color the meshes
            TRIMESH collisionmesh; ///< collision data of the specific object. For spheres and cylinders, an appropriate
                                   ///< discretization value is chosen. Should be transformed by _t before rendering
            GeomType type;         ///< the type of geometry primitive
            std::string renderfile;  ///< render resource file, should be transformed by _t before rendering
            Vector vRenderScale; ///< render scale of the object (x,y,z)
            float ftransparency; ///< value from 0-1 for the transparency of the rendered object, 0 is opaque

            bool _bDraw;         ///< if true, object is drawn as part of the 3d model (default is true)
            bool _bModifiable; ///< if true, object geometry can be dynamically modified (default is true)

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

        inline KinBodyPtr GetParent() const { return KinBodyPtr(_parent); }

        inline int GetIndex() const { return index; }
        inline const TRIMESH& GetCollisionData() const { return collision; }

        /// return the aabb of all the geometries of the link
        virtual AABB ComputeAABB() const;

        virtual Transform GetTransform() const;

        /// Get the center of mass offset in the link's local coordinate frame
        virtual Vector GetCOMOffset() const {return _transMass.trans; }
        virtual const TransformMatrix& GetInertia() const { return _transMass; }
        virtual dReal GetMass() const { return _mass; }

        virtual void SetTransform(const Transform& t);
        
        /// adds an external force at pos (absolute coords)
        /// \param force the direction and magnitude of the force
        /// \param pos in the world where the force is getting applied
        /// \param bAdd if true, force is added to previous forces, otherwise it is set
        virtual void SetForce(const Vector& force, const Vector& pos, bool bAdd);

        /// adds torque to a body (absolute coords)
        /// \param bAdd if true, torque is added to previous torques, otherwise it is set
        virtual void SetTorque(const Vector& torque, bool bAdd);

        //typedef std::list<GEOMPROPERTIES>::iterator GeomPtr;
        //typedef std::list<GEOMPROPERTIES>::const_iterator GeomConstPtr;
        const std::list<GEOMPROPERTIES>& GetGeometries() const { return _listGeomProperties; }
        virtual GEOMPROPERTIES& GetGeometry(int index);
        /// swaps the current geometries with the new geometries. This gives a user control for dynamically changing the object geometry. Note that the kinbody/robot hash could change.
        void SwapGeometries(std::list<GEOMPROPERTIES>& listNewGeometries);
        
        /// validates the contact normal on link and makes sure the normal faces "outside" of the geometry shape it lies on. An exception can be thrown if position is not on a geometry surface
        /// \param position the position of the contact point specified in the link's coordinate system, assumes it is on a particular geometry
        /// \param normal the unit normal of the contact point specified in the link's coordinate system
        /// \return true if the normal is changed to face outside of the shape
        virtual bool ValidateContactNormal(const Vector& position, Vector& normal) const;

        virtual void serialize(std::ostream& o, int options) const;
    private:
        /// updates the collision mesh from the geometries. Note that this might change the robot/kinbody hash!!
        void UpdateCollisionMesh();

        Transform _t;           ///< current transform of the link
        TransformMatrix _transMass; ///< the 3x3 inertia and center of mass of the link in the link's coordinate system
        dReal _mass;
        TRIMESH collision; ///< triangles for collision checking, triangles are always the triangulation
                           ///< of the body when it is at the identity transformation

        std::string name;     ///< optional link name
        int index;          ///< index into parent KinBody::_veclinks
        KinBodyWeakPtr _parent; ///< body that link belong to
        
        std::list<GEOMPROPERTIES> _listGeomProperties; ///< a list of all the extra geometry properties
        
        int userdata;   ///< used when iterating through the links
        bool bStatic;       ///< indicates a static body. Static should be used when an object has infinite mass and
                            ///< shouldn't be affected by physics (including gravity). Collision still works
        bool _bIsEnabled;

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
    typedef boost::shared_ptr<Link> LinkPtr;
    typedef boost::shared_ptr<Link const> LinkConstPtr;
    typedef boost::weak_ptr<Link> LinkWeakPtr;

    /// Information about a joint
    class RAVE_API Joint : public boost::enable_shared_from_this<Joint>
    {
    public:
        enum JointType {
            JointNone = 0,
            JointHinge = 1,
            JointRevolute = 1,
            JointSlider = 2,
            JointPrismatic = 2,
            JointUniversal = 3,
            JointHinge2 = 4,
            JointSpherical = 5,
        };

        Joint(KinBodyPtr parent);
        virtual ~Joint();

        inline const std::string& GetName() const { return name; }
        inline int GetMimicJointIndex() const { return nMimicJointIndex; }
        inline const std::vector<dReal>& GetMimicCoeffs() const { return vMimicCoeffs; }

        inline dReal GetMaxVel() const { return fMaxVel; }
        inline dReal GetMaxAccel() const { return fMaxAccel; }
        inline dReal GetMaxTorque() const { return fMaxTorque; }
        inline dReal GetOffset() const { return offset; }

        /// Get the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        /// Ie, KinBody::GetJointValues()[GetDOFIndex()] == GetValues()
        inline int GetDOFIndex() const { return dofindex; }

        /// Get the joint index into KinBody::_vecjoints
        inline int GetJointIndex() const { return jointindex; }
        
        inline KinBodyPtr GetParent() const { return KinBodyPtr(_parent); }

        inline LinkPtr GetFirstAttached() const { return bodies[0]; }
        inline LinkPtr GetSecondAttached() const { return bodies[1]; }

        inline dReal GetResolution() const { return fResolution; }
        inline JointType GetType() const { return type; }

        virtual int GetDOF() const;

        /// return true if joint has no limits
        virtual bool IsCircular() const { return _bIsCircular; }

        /// return true if joint can be treated as a static binding (ie all limits are 0)
        virtual bool IsStatic() const;

        /// Gets the joint values with the correct offsets applied
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return degrees of freedom of the joint (even if pValues is NULL)
        virtual void GetValues(std::vector<dReal>& values, bool bAppend=false) const;

        /// Gets the joint velocities
        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return the degrees of freedom of the joint (even if pValues is NULL)
        virtual void GetVelocities(std::vector<dReal>& values, bool bAppend=false) const;

        /// add torque
        virtual void AddTorque(const std::vector<dReal>& torques);

        /// \return the anchor of the joint in global coordinates
        virtual Vector GetAnchor() const;

        /// \return the axis of the joint in global coordinates
        virtual Vector GetAxis(int iaxis = 0) const;

        /// \param bAppend if true will append to the end of the vector instead of erasing it
        /// \return degrees of freedom of the joint
        virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit, bool bAppend=false) const;

        /// \return the weight associated with a joint's axis for computing a distance in the robot configuration space.
        virtual dReal GetWeight(int iaxis=0) const;

        virtual void SetJointOffset(dReal offset);
        virtual void SetJointLimits(const std::vector<dReal>& vLowerLimit, const std::vector<dReal>& vUpperLimit);
        virtual void SetResolution(dReal resolution);
        virtual void SetWeights(const std::vector<dReal>& vweights);

        virtual void serialize(std::ostream& o, int options) const;

        /// @name Internal Hierarchy Methods
        //@{

        /// \return the anchor of the joint in local coordinates
        virtual Vector GetInternalHierarchyAnchor() const { return vanchor; }
        /// \return the axis of the joint in local coordinates
        virtual Vector GetInternalHierarchyAxis(int iaxis = 0) const;
        /// left multiply transform given the base body
        virtual Transform GetInternalHierarchyLeftTransform() const;
        /// right multiply transform given the base body
        virtual Transform GetInternalHierarchyRightTransform() const;
        //@}
    private:
        boost::array<Vector,3> vAxes;        ///< axes in body[0]'s or environment coordinate system used to define joint movement
        Vector vanchor;         ///< anchor of the joint
        Transform tRight, tLeft;///< transforms used to get body[1]'s transformation with respect to body[0]'s
                                ///< Tbody1 = Tbody0 * tLeft * JointRotation * tRight
        Transform tinvRight, tinvLeft; ///< the inverse transformations of tRight and tLeft
        dReal fResolution;      ///< interpolation resolution
        dReal fMaxVel;          ///< the maximum velocity (rad/s) to move the joint when planning
        dReal fHardMaxVel;      ///< the hard maximum velocity, robot cannot exceed this velocity. used for verification checking
        dReal fMaxAccel;        ///< the maximum acceleration (rad/s^2) of the joint
        dReal fMaxTorque;       ///< maximum torque (N.m, kg m^2/s^2) that can be applied to the joint
        boost::array<LinkPtr,2> bodies; ///< attached bodies
        std::vector<dReal> _vweights;        ///< the weights of the joint for computing distance metrics.
        dReal offset;           ///< needed for rotation joints since the range is limited to 2*pi. This allows the offset to be set so the joint can function in [-pi+offset,pi+offset]
                                ///< converts the ode joint angle to the expected joint angle
        std::vector<dReal> _vlowerlimit, _vupperlimit; ///< joint limits
        int nMimicJointIndex;   ///< only valid for passive joints and if value is >= 0
        std::vector<dReal> vMimicCoeffs;  ///< the angular position of this joint should be fMimicCoeffs[0]*mimic_joint+fMimicCoeffs[1]
                                ///< when specifying joint limits, note that they are still in terms of this joint
        int dofindex;           ///< the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        int jointindex;         ///< the joint index into KinBody::_vecjoints
        KinBodyWeakPtr _parent;       ///< body that joint belong to
        JointType type;
        std::string name; ///< joint name
        bool _bIsCircular;    ///< circular joint where the lower and upper limits identifiy with each other.

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
    typedef boost::shared_ptr<Joint> JointPtr;
    typedef boost::shared_ptr<Joint const> JointConstPtr;

    /// Holds the state of the current body that is published in a thread safe way
    /// from the environment without requiring locking the environment.
    class BodyState
    {
    public:
        BodyState() {}
        virtual ~BodyState() {}
        KinBodyPtr pbody;
        std::vector<RaveTransform<dReal> > vectrans;
        std::vector<dReal> jointvalues;
        boost::shared_ptr<void> pguidata, puserdata;
        std::string strname; ///< name of the body
        int environmentid; ///< unique network id
    };
    typedef boost::shared_ptr<BodyState> BodyStatePtr;
    typedef boost::shared_ptr<BodyState const> BodyStateConstPtr;

    /// Access point of the sensor system that manages the body.
    class RAVE_API ManageData : public boost::enable_shared_from_this<ManageData>
    {
    public:
    ManageData(SensorSystemBasePtr psensorsystem) : _psensorsystem(psensorsystem) {}
        virtual ~ManageData() {}

        virtual SensorSystemBasePtr GetSystem() { return SensorSystemBasePtr(_psensorsystem); }

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

    typedef boost::shared_ptr<ManageData> ManageDataPtr;
    typedef boost::shared_ptr<ManageData const> ManageDataConstPtr;

    /// Helper class to save the entire kinbody state
    class RAVE_API KinBodyStateSaver
    {
    public:
        KinBodyStateSaver(KinBodyPtr pbody);
        virtual ~KinBodyStateSaver();
    protected:
        std::vector<Transform> _vLinkTransforms;
        std::vector<uint8_t> _vEnabledLinks;
        KinBodyPtr _pbody;
    };

    virtual ~KinBody();

    virtual void Destroy();

    /// Build the robot from an XML filename
    virtual bool InitFromFile(const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts);
    /// Build the robot from a string representing XML information
    virtual bool InitFromData(const std::string& data, const std::list<std::pair<std::string,std::string> >& atts);

    /// Create a kinbody with one link composed of an array of aligned bounding boxes
    /// \param vaabbs the array of aligned bounding boxes that will comprise of the body
    /// \param bDraw if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<AABB>& vaabbs, bool bDraw);

    /// Create a kinbody with one link composed of an array of oriented bounding boxes
    /// \param vobbx the array of oriented bounding boxes that will comprise of the body
    /// \param bDraw if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<OBB>& vobbs, bool bDraw);

    //! Get the name of the robot
    virtual const std::string& GetName() const           { return name; }
    virtual void SetName(const std::string& newname);

    /// @name Basic Information
    /// Methods for accessing basic information about joints
    //@{

    /// \return number of controllable degrees of freedom of the body
    virtual int GetDOF() const;

    virtual void GetJointValues(std::vector<dReal>& v) const;
    virtual void GetJointVelocities(std::vector<dReal>& v) const;
    virtual void GetJointLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const;
    virtual void GetJointMaxVel(std::vector<dReal>& v) const;
    virtual void GetJointMaxAccel(std::vector<dReal>& v) const;
    virtual void GetJointMaxTorque(std::vector<dReal>& v) const;
    virtual void GetJointResolutions(std::vector<dReal>& v) const;
    virtual void GetJointWeights(std::vector<dReal>& v) const;

    ///< \return a vector that stores the start dof indices of each joint joints, size() is equal to GetJoints().size()
    virtual const std::vector<int>& GetJointIndices() const { return _vecJointIndices; }

    /// returns the joints making up the degrees of freedom in the user-defined order
    const std::vector<JointPtr>& GetJoints() const { return _vecjoints; }
    /// returns the passive joints, order does not matter
    const std::vector<JointPtr>& GetPassiveJoints() const { return _vecPassiveJoints; }

    /// Gets all the rigidly attached links to plink, also adds the link to the list.
    /// \param linkindex the index to check for attached links. If < 0, then will return all links attached to the environment
    /// \param vattachedlinks the array to insert all links attached to linkindex with the link itself.
    virtual void GetRigidlyAttachedLinks(int linkindex, std::vector<LinkPtr>& vattachedlinks) const;

    /// Returns the joints in hierarchical order starting at the base link such that the first joints affect the later ones.
    /// In the case of closed loops, the joints are returned in the order they are defined in _vecjoints.
    /// \param vjointindices a set of joint indices to be filled with the correct order
    const std::vector<JointPtr>& GetDependencyOrderedJoints() const { return _vDependencyOrderedJoints; }

    /// returns the minimal chain of joints that are between two links in the order of linkbaseindex to linkendindex.
    /// Passive joints are used to detect rigidly attached links and mimic joints, otherwise they are ignored in the computation of the chain.
    /// If a mimic joint is found along the path, the joint returned is the source joint!
    /// \param linkbase the base link to start the search
    /// \param linkend the link to end the search
    /// \param vjoints the joints to fill that describe the chain
    /// \return true if the two links are connected (vjoints will be filled), false if the links are separate
    bool GetChain(int linkbaseindex, int linkendindex, std::vector<JointPtr>& vjoints) const;

    /// return a pointer to the joint with the given name, else -1
    /// gets a joint indexed from GetJoints(). Note that the mapping of joint structures is not the same as
    /// the values in GetJointValues since each joint can have more than one degree of freedom.
    virtual int GetJointIndex(const std::string& jointname) const;
    virtual JointPtr GetJoint(const std::string& jointname) const;

    /// gets the joint that covers the degree of freedom index
    virtual JointPtr GetJointFromDOFIndex(int dofindex) const;
    //@}

    /// computes the configuration difference q1-q2 and stores it in q1. Takes into account joint limits and circular joints
    virtual void SubtractJointValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const;

    /// adds a torque to every joint
    /// \param bAdd if true, adds to previous torques, otherwise resets the torques on all bodies and starts from 0
    virtual void SetJointTorques(const std::vector<dReal>& torques, bool bAdd);

    /// \return the links of the robot
    const std::vector<LinkPtr>& GetLinks() const { return _veclinks; }

	/// return a pointer to the link with the given name
    virtual LinkPtr GetLink(const std::string& linkname) const;

    /// Updates the bounding box and any other parameters that could have changed by a simulation step
    virtual void SimulationStep(dReal fElapsedTime);
    virtual void GetBodyTransformations(std::vector<Transform>& vtrans) const;

    /// queries the transfromation of the first link of the body
    virtual Transform GetTransform() const;

    /// set the velocity of the base link
    /// \param angularvel is the rotation axis * angular speed
    virtual void SetVelocity(const Vector& linearvel, const Vector& angularvel);

    /// get the velocity of the base link
    /// \param angularvel is the rotation axis * angular speed
    virtual void GetVelocity(Vector& linearvel, Vector& angularvel) const;

    /// Returns the linear and angular velocities for each link
    virtual void GetLinkVelocities(std::vector<std::pair<Vector,Vector> >& velocities) const;

    /// \~japanese 最初リンクの絶対姿勢を設定、残りのリンクは運動学の構造を通して計算される
    /// \~english set the transform of the first link (the rest of the links are computed based on the joint values)
    virtual void SetTransform(const Transform& trans);

    /// \return an axis-aligned bounding box of the entire object
    virtual AABB ComputeAABB() const;

    /// \return center of mass of entire robot
    virtual Vector GetCenterOfMass() const;

    /// enables or disables the bodies
    virtual void Enable(bool bEnable);
    virtual void EnableLink(LinkConstPtr plink, bool bEnable);

    /// \return true if any link of the KinBody is enabled
    virtual bool IsEnabled() const;

    /// sets the joint angles manually, if ptrans is not NULL, represents the transformation of the first body
    /// calculates the transformations of every body by treating the first body as set with transformation ptrans
    virtual void SetJointValues(const std::vector<dReal>& vJointValues, const Transform& transBase, bool bCheckLimits = false);

    /// sets the joint angles manually, if ptrans is not NULL, represents the transformation of the first body
    /// calculates the transformations of every body by treating the first body as set with transformation ptrans
    virtual void SetJointValues(const std::vector<dReal>& vJointValues, bool bCheckLimits = false);

    virtual void SetBodyTransformations(const std::vector<Transform>& vbodies);
    virtual void SetJointVelocities(const std::vector<dReal>& pJointVelocities);

    /// gets the jacobian with respect to a link, pfArray is a 3 x DOF matrix (rotations are not taken into account)
    /// Calculates the partial differentials for all joints that in the path from the root node to _veclinks[index]
    /// (doesn't touch the rest of the values)
    /// \param linkindex of the link that the rotation is attached to
    virtual void CalculateJacobian(int linkindex, const Vector& offset, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateJacobian(int linkindex, const Vector& offset, std::vector<dReal>& pfJacobian) const;

    /// calculates the rotational jacobian as a quaternion with respect to an initial rotation
    /// \param linkindex of the link that the rotation is attached to
    /// \param pfJacobian 4xDOF matrix
    virtual void CalculateRotationJacobian(int linkindex, const Vector& qInitialRot, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateRotationJacobian(int linkindex, const Vector& qInitialRot, std::vector<dReal>& pfJacobian) const;

    /// calculates the angular velocity jacobian of a specified link about the axes of world coordinates
    /// \param index of the link that the rotation is attached to
    /// \param pfJacobian 3xDOF matrix
    virtual void CalculateAngularVelocityJacobian(int linkindex, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateAngularVelocityJacobian(int linkindex, std::vector<dReal>& pfJacobian) const;

    /// Check if body is self colliding. Links that are joined together are ignored.
    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr()) const;

    /// \return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    virtual bool IsAttached(KinBodyConstPtr pbody) const;

    /// \param setAttached inserts all attached bodies of this body, including this body. If any bodies are already in setAttached, then ignores recursing on their attached bodies.
    virtual void GetAttached(std::set<KinBodyPtr>& setAttached) const;

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const { return false; }
    
    /// \return an environment unique id. if object is not added to the environment, this will return 0. So checking if GetEnvironmentId() is 0 is a good way to check if object is present in the environment.
    virtual int GetEnvironmentId() const;

    virtual int GetNetworkId() const { return GetEnvironmentId(); }
    
    /// returns how the joint effects the link. If zero, link is unaffected. If negative, the partial derivative of the Jacobian should be negated.
    /// \param jointindex index of the joint
    /// \param linkindex index of the link
    virtual char DoesAffect(int jointindex, int linkindex) const;

    /// writes a string for the forward kinematics of the robot (only hinge joints are handled)
    /// format of the string is:
    /// [link_cur link_base joint_index joint_axis T_left T_right]
    /// basically T(link_cur) = T(link_base) * T_left * Rotation(joint_axis, joint_angle) * T_right
    /// T_left and T_right are 3x4 matrices specified in row order (write the first row first)
    /// joint_axis is the unit vector for the joint_axis
    /// if link_base is -1, attached to static environment
    virtual void WriteForwardKinematics(std::ostream& f);

    virtual void SetGuiData(boost::shared_ptr<void> pdata) { _pGuiData = pdata; }
    virtual boost::shared_ptr<void> GetGuiData() const { return _pGuiData; }

    /// \return all possible link pairs that could get in collision
    virtual const std::set<int>& GetNonAdjacentLinks() const;
    /// \return all possible link pairs whose collisions are ignored.
    virtual const std::set<int>& GetAdjacentLinks() const;
    
    virtual boost::shared_ptr<void> GetPhysicsData() const { return _pPhysicsData; }
    virtual boost::shared_ptr<void> GetCollisionData() const { return _pCollisionData; }
    virtual ManageDataPtr GetManageData() const { return _pManageData; }

    /// The stamp is used by the collision checkers, physics engines, or any other item
    /// that needs to keep track of any changes of the KinBody as it moves.
    /// Currently stamps monotonically increment for every transformation/joint angle change.
    virtual int GetUpdateStamp() const { return _nUpdateStampId; }

    /// Preserves the collision, physics, user, gui data fields.
    /// Preserves the attached bodies
    /// Preserves the XML readers
    virtual bool Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /// Register a callback with the interface. Everytime a static property of the interface changes, all
    /// registered callbacks are called to update the users of the changes. Note that the callbacks will
    /// block the thread that made the parameter change.
    /// \param callback 
    /// \param properties a mask of the set of properties that the callback should be called for when they change
    virtual boost::shared_ptr<void> RegisterChangeCallback(int properties, const boost::function<void()>& callback);

    virtual void serialize(std::ostream& o, int options) const;

    /// A md5 hash unique to the particular kinematic and geometric structure of a KinBody.
    /// This 32 byte string can be used to check if two bodies have the same kinematic structure and can be used
    /// to index into tables when looking for body-specific models. OpenRAVE stores all
    /// such models in the OPENRAVE_HOME directory (usually ~/.openrave), indexed by the particular robot/body hashes.
    /// \return md5 hash string of kinematics/geometry
    virtual std::string GetKinematicsGeometryHash() const;

protected:
    /// constructors declared protected so that user always goes through environment to create bodies
    KinBody(PluginType type, EnvironmentBasePtr penv);
    
    /// specific data about physics engine, should be set only by the current PhysicsEngineBase
    virtual void SetPhysicsData(boost::shared_ptr<void> pdata) { _pPhysicsData = pdata; }
    virtual void SetCollisionData(boost::shared_ptr<void> pdata) { _pCollisionData = pdata; }
    virtual void SetManageData(ManageDataPtr pdata) { _pManageData = pdata; }

    virtual void ComputeJointHierarchy();
    virtual void ParametersChanged(int parmameters);

    inline KinBodyPtr shared_kinbody() { return boost::static_pointer_cast<KinBody>(shared_from_this()); }
    inline KinBodyConstPtr shared_kinbody_const() const { return boost::static_pointer_cast<KinBody const>(shared_from_this()); }

    /// \return true if two bodies should be considered as one during collision (ie one is grabbing the other)
    virtual bool _IsAttached(KinBodyConstPtr pbody, std::set<KinBodyConstPtr>& setChecked) const;

    /// adds an attached body
    virtual void _AttachBody(KinBodyPtr pbody);

    /// removes an attached body
    /// \return true if pbody was successfully found and removed
    virtual bool _RemoveAttachedBody(KinBodyPtr pbody);

    std::string name; ///< name of body

    std::vector<JointPtr> _vecjoints;     ///< all the joints of the body, joints contain limits, torques, and velocities (the order of these joints dictate the order of the degrees of freedom)
    std::vector<JointPtr> _vDependencyOrderedJoints; ///< all joints of the body ordered on how they affect the joint hierarchy
    std::vector<LinkPtr> _veclinks;       ///< children, unlike render hierarchies, transformations
                                        ///< of the children are with respect to the global coordinate system
    std::vector<int> _vecJointIndices;  ///< cached start indices, indexed by joint indices
    std::vector<char> _vecJointHierarchy;   ///< joint x link, entry is non-zero if the joint affects the link in the forward kinematics
                                            ///< if negative, the partial derivative of ds/dtheta should be negated

    std::vector<JointPtr> _vecPassiveJoints; ///< joints not directly controlled

    std::set<int> _setAdjacentLinks;        ///< a set of which links are connected to which if link i and j are connected then
                                            ///< i|(j<<16) will be in the set where i<j.
    std::set<int> _setNonAdjacentLinks;     ///< the not of _setAdjacentLinks
    std::vector< std::pair<std::string, std::string> > _vForcedAdjacentLinks; ///< internally stores forced adjacent links

    std::list<KinBodyWeakPtr> _listAttachedBodies; ///< list of bodies that are directly attached to this body (can have duplicates)

    int _environmentid;                          ///< unique id of the body solely used by the environment. This id will not be copied when cloning.
    int _nUpdateStampId;                         ///< unique id for every unique transformation change of any link,
                                            ////< monotically increases as body is updated.
    boost::shared_ptr<void> _pGuiData;                        ///< GUI data to let the viewer store specific graphic handles for the object
    boost::shared_ptr<void> _pPhysicsData;                ///< data set by the physics engine
    boost::shared_ptr<void> _pCollisionData; ///< internal collision model
    ManageDataPtr _pManageData;

    std::list<std::pair<int,boost::function<void()> > > _listRegisteredCallbacks; ///< callbacks to call when particular properties of the body change.
    
    bool _bHierarchyComputed; ///< true if the joint heirarchy and other cached information is computed
    bool _bMakeJoinedLinksAdjacent;
private:
    std::string __hashkinematics;
    mutable std::vector<dReal> _vTempJoints;
    virtual const char* GetHash() const { return OPENRAVE_KINBODY_HASH; }

    static void __erase_iterator(KinBodyWeakPtr pweakbody, std::list<std::pair<int,boost::function<void()> > >::iterator* pit);

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
    friend class SensorSystemBase;
};

} // end namespace OpenRAVE

#endif   // KIN_CHAIN_H
