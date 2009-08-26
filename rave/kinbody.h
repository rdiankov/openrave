// Copyright (C) 2006-2009 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
  \file   KinBody.h
  \brief  Encapsulate a kinematic chain of links
 -------------------------------------------------------------------- */
#ifndef  KIN_CHAIN_H
#define  KIN_CHAIN_H

namespace OpenRAVE {

/// Encapsulate a kinematic body of links and joints
class KinBody : public InterfaceBase
{
public:

    /// rigid body defined by an arbitrary ODE body and a render object
    class Link
    {
    public:
        Link(KinBody* parent); ///< pass in a ODE world
        virtual ~Link();
        
        inline const wchar_t* GetName() const { return name.size() > 0 ? name.c_str() : L"(NULL)"; }

        inline bool IsStatic() const { return bStatic; }
        virtual bool IsEnabled() const; ///< returns true if enabled

        ///< enables a Link. An enabled link takes part in collision detection and physics simulations
        virtual void Enable(bool bEnable);

        /// user data for trimesh geometries
        struct TRIMESH
        {
            std::vector<Vector> vertices;
            std::vector<int> indices;

            void ApplyTransform(const Transform& t);
            void ApplyTransform(const TransformMatrix& t);

            /// append another TRIMESH to this tri mesh
            void Append(const TRIMESH& mesh);
            void Append(const TRIMESH& mesh, const Transform& trans);

            AABB ComputeAABB() const;
        };

        /// Describes the properties of a basic geometric primitive.
        /// Contains everything associated with a physical body along with a seprate (optional) render file.
        struct GEOMPROPERTIES
        {
            enum GeomType {
                GeomNone = 0,
                GeomBox = 1,
                GeomSphere = 2,
                GeomCylinder = 3,
                GeomTrimesh = 4,
            };
            
            GEOMPROPERTIES();
            virtual ~GEOMPROPERTIES() {}

            inline const Transform& GetTransform() const { return _t; }
            inline GeomType GetType() const { return type; }
            inline const Vector& GetRenderScale() const { return vRenderScale; }
            inline const std::string& GetRenderFilename() const { return renderfile; }
            inline float GetTransparency() const { return ftransparency; }
            inline bool IsDraw() const { return bDraw; }
            
            inline dReal GetSphereRadius() const { return vGeomData.x; }
            inline dReal GetCylinderRadius() const { return vGeomData.x; }
            inline dReal GetCylinderHeight() const { return vGeomData.y; }
            inline const Vector& GetBoxExtents() const { return vGeomData; }
            inline const RaveVector<float>& GetDiffuseColor() const { return diffuseColor; }
            inline const RaveVector<float>& GetAmbientColor() const { return ambientColor; }
            
            inline const TRIMESH& GetCollisionMesh() const { return collisionmesh; }
            
            virtual AABB ComputeAABB(const Transform& t) const;

        private:
            /// triangulates the geometry object and initializes collisionmesh. GeomTrimesh types must already be triangulated
            /// \param fTesselation to control how fine the triangles need to be
            /// 1.0f is the default value that 
            bool InitCollisionMesh(float fTessellation=1);

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

            bool bDraw;         ///< if true, object is drawn as part of the 3d model (default is true)

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
            friend class LinkXMLReader;
            friend class KinBodyXMLReader;
            friend class RobotXMLReader;
            friend class ColladaReader;
#else
            friend class ::LinkXMLReader;
            friend class ::KinBodyXMLReader;
            friend class ::RobotXMLReader;
            friend class ::ColladaReader;
#endif
#endif
            friend class KinBody;
        };

        inline KinBody* GetParent() const { return _parent; }

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

        const std::list<GEOMPROPERTIES>& GetGeometries() const { return _listGeomProperties; }
    private:
        
        Transform _t;           ///< current transform of the link
        TransformMatrix _transMass; ///< the 3x3 inertia and center of mass of the link in the link's coordinate system
        dReal _mass;
        TRIMESH collision; ///< triangles for collision checking, triangles are always the triangulation
                           ///< of the body when it is at the identity transformation

        std::wstring name;     ///< optional link name
        int index;          ///< index into parent KinBody::_veclinks
        KinBody* _parent; ///< body that link belong to
        
        std::list<GEOMPROPERTIES> _listGeomProperties; ///< a list of all the extra geometry properties
        
        int userdata;   ///< used when iterating through the links
        bool bStatic;       ///< indicates a static body. Static should be used when an object has infinite mass and
                            ///< shouldn't be affected by physics (including gravity). Collision still works
        bool _bIsEnabled;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class LinkXMLReader;
        friend class JointXMLReader;
        friend class KinBodyXMLReader;
        friend class RobotXMLReader;
        friend class ColladaReader;
#else
        friend class ::LinkXMLReader;
        friend class ::JointXMLReader;
        friend class ::KinBodyXMLReader;
        friend class ::RobotXMLReader;
        friend class ::ColladaReader;
#endif
#endif
        friend class KinBody;
    };

    /// Information about a joint
    class Joint
    {
    public:
        enum JointType {
            JointNone = 0,
            JointHinge = 1,
            JointRevolute = 1,
            JointSlider = 2,
            JointPrismatic = 2,
            JointUniversal = 3,
            JointHinge2 = 4
        };

        Joint(KinBody* parent);
        virtual ~Joint();

        inline const wchar_t* GetName() const { return name.c_str(); }
        inline int GetMimicJointIndex() const { return nMimicJointIndex; }
        inline const dReal* GetMimicCoeffs() const { return fMimicCoeffs; }

        inline dReal GetMaxVel() const { return fMaxVel; }
        inline dReal GetMaxAccel() const { return fMaxAccel; }
        inline dReal GetMaxTorque() const { return fMaxTorque; }

        inline dReal GetOffset() const { return offset; }

        /// Get the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        /// Ie, KinBody::GetJointValues()[GetDOFIndex()] == GetValues()
        inline int GetDOFIndex() const { return dofindex; }

        /// Get the joint index into KinBody::_vecjoints
        inline int GetJointIndex() const { return jointindex; }
        
        inline KinBody* GetParent() const { return _parent; }

        inline Link* GetFirstAttached() const { return bodies[0]; }
        inline Link* GetSecondAttached() const { return bodies[1]; }

        inline dReal GetResolution() const { return fResolution; }
        inline JointType GetType() const { return type; }

        virtual int GetDOF() const;

        /// Gets the joint values with the correct offsets applied
        /// \return degrees of freedom of the joint (even if pValues is NULL)
        virtual int GetValues(dReal* pValues) const;

        /// Gets the joint velocities
        /// \return the degrees of freedom of the joint (even if pValues is NULL)
        virtual int GetVelocities(dReal* pValues) const;

        /// add torque
        virtual void AddTorque(const dReal* pTorques);

        /// \return the anchor of the joint in global coordinates
        virtual Vector GetAnchor() const;

        /// \return the axis of the joint in global coordinates
        virtual Vector GetAxis(int iaxis = 0) const;

        /// \return degrees of freedom of the joint
        virtual int GetLimits(dReal* pLowerLimit, dReal* pUpperLimit) const;
        
    private:
        Link* bodies[2];
        dReal fResolution;      ///< interpolation resolution

        dReal fMaxVel;          ///< the maximum velocity (rad/s) of the joint
        dReal fMaxAccel;        ///< the maximum acceleration (rad/s^2) of the joint
        dReal fMaxTorque;       ///< maximum torque (N.m, kg m^2/s^2) that can be applied to the joint

        Transform tRight, tLeft;///< transforms used to get body[1]'s transformation with respect to body[0]'s
                                ///< Tbody1 = Tbody0 * tLeft * JointRotation * tRight
        Transform tinvRight, tinvLeft; ///< the inverse transformations of tRight and tLeft
        Vector vAxes[3];        ///< axes in body[0]'s or environment coordinate system used to define joint movement
        Vector vanchor;         ///< anchor of the joint
        dReal offset;           ///< needed because ode can only limit joints in the range of -pi to pi
                                ///< converts the ode joint angle to the expected joint angle
        std::vector<dReal> _vlowerlimit, _vupperlimit; ///< joint limits
        int nMimicJointIndex;   ///< only valid for passive joints and if value is >= 0
        dReal fMimicCoeffs[2];  ///< the angular position of this joint should be fMimicCoeffs[0]*mimic_joint+fMimicCoeffs[1]
                                ///< when specifying joint limits, note that they are still in terms of this joint
        int dofindex;           ///< the degree of freedom index in the body's DOF array, does not index in KinBody::_vecjoints!
        int jointindex;         ///< the joint index into KinBody::_vecjoints
        KinBody* _parent;       ///< body that joint belong to
        JointType type;

        std::wstring name; ///< optional joint name

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class JointXMLReader;
        friend class KinBodyXMLReader;
        friend class RobotXMLReader;
        friend class ColladaReader;
        friend class ColladaWriter;
#else
        friend class ::JointXMLReader;
        friend class ::KinBodyXMLReader;
        friend class ::RobotXMLReader;
        friend class ::ColladaReader;
        friend class ::ColladaWriter;
#endif
#endif
        friend class KinBody;
    };

    /// Helper class to save the entire kinbody state
    class KinBodyStateSaver
    {
    public:
        KinBodyStateSaver(KinBody* pbody);
        virtual ~KinBodyStateSaver();
    protected:
        std::vector<Transform> _vtransPrev;
        KinBody* _pbody;
    };

    virtual ~KinBody();

	virtual const char* GetXMLId() { return "KinBody"; }

    virtual void Destroy();

    /// Build the robot from an XML filename
    virtual bool Init(const char* filename, const char**atts);

    /// Create a kinbody with one link composed of an array of aligned bounding boxes
    /// \param vaabbs the array of aligned bounding boxes that will comprise of the body
    /// \param bDraw if true, the boxes will be rendered in the scene
    virtual bool InitFromBoxes(const std::vector<AABB>& vaabbs, bool bDraw);

    //! Get the name of the robot
    virtual const wchar_t* GetName() const           { return name.size() > 0 ? name.c_str() : L"(NULL)"; }
    virtual void SetName(const wchar_t* pNewName);
    virtual void SetName(const char* pNewName);

    /** @name Basic Information
    *  Methods for accessing basic information about joints */
    //@{

    /// \return number of joints of the body
    virtual int GetDOF() const { return (int)_vecJointWeights.size(); }

    virtual void GetJointValues(dReal* pValues) const;    ///< gets the current values
    virtual void GetJointValues(std::vector<dReal>& v) const;
    virtual void GetJointVelocities(dReal* pValues) const;    ///< gets the current velocities
    virtual void GetJointVelocities(std::vector<dReal>& v) const { v.resize(GetDOF()); if( GetDOF() > 0 ) GetJointVelocities(&v[0]); }
    virtual void GetJointLimits(dReal* pLowerLimit, dReal* pUpperLimit) const;
    virtual void GetJointLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const;
    
    virtual void GetJointMaxVel(dReal* pValues) const;
    virtual void GetJointMaxVel(std::vector<dReal>& v) const { v.resize(GetDOF()); if( GetDOF() > 0 ) GetJointMaxVel(&v[0]); }
    virtual void GetJointMaxAccel(dReal* pValues) const;
    virtual void GetJointMaxAccel(std::vector<dReal>& v) const { v.resize(GetDOF()); if( GetDOF() > 0 ) GetJointMaxVel(&v[0]); }
    virtual void GetJointMaxTorque(dReal* pValues) const;
    virtual void GetJointMaxTorque(std::vector<dReal>& v) const { v.resize(GetDOF()); if( GetDOF() > 0 ) GetJointMaxTorque(&v[0]); }
    virtual void GetJointResolutions(dReal* pValues) const;
    virtual void GetJointResolutions(std::vector<dReal>& v) const { v.resize(GetDOF()); if( GetDOF() > 0 ) GetJointResolutions(&v[0]); }
    //@}

    /// adds a torque to every joint
    /// \param bAdd if true, adds to previous torques, otherwise resets the torques on all bodies and starts from 0
    virtual void SetJointTorques(const dReal* pValues, bool bAdd);

    ///< gets the start index of the joint arrays returned by GetJointValues() of the joint
    virtual const std::vector<int>& GetJointIndices() const { return _vecJointIndices; }

    const std::vector<Joint*>& GetJoints() const { return _vecjoints; }
    const std::vector<Link*>& GetLinks() const { return _veclinks; }

	//! return a pointer to the link with the given name
    virtual Link* GetLink(const wchar_t* name) const
    {
        if( name == NULL )
            return NULL;
        
        for(std::vector<Link*>::const_iterator it = _veclinks.begin(); it != _veclinks.end(); ++it) {
            if ((*it)->GetName() != NULL && wcscmp((*it)->GetName(), name) == 0)
                return *it;
        }
        RAVELOG_VERBOSEA("Link::GetLink - Error Unknown Link %S\n",name);
        return NULL;
    }

    //! return a pointer to the joint with the given name, else -1
    virtual int GetJointIndex(const wchar_t* name) const
    {
        int index = 0;
        for(std::vector<Joint*>::const_iterator it = _vecjoints.begin(); it != _vecjoints.end(); ++it, index++) {
            if ((*it)->GetName() != NULL && wcscmp((*it)->GetName(), name) == 0)
                return index;
        }

        return -1;
    }
    
    /// gets a joint indexed from GetJoints(). Note that the mapping of joint structures is not the same as
    /// the values in GetJointValues since each joint can have more than one degree of freedom.
    virtual Joint* GetJoint(int index) const { assert(index >= 0 && index < (int)_vecjoints.size()); return _vecjoints[index]; }

    /// gets the joint that covers the degree of freedom index
    virtual Joint* GetJointFromDOFIndex(int dofindex) const;

    ///  Computes a state space metric
    virtual dReal ConfigDist(const dReal *q1) const;

    virtual dReal ConfigDist(const dReal *q1, const dReal *q2) const
    {
        dReal dist = 0.0;
        for (size_t i = 0; i < _vecJointWeights.size(); i++) {
            dist += _vecJointWeights[i] * (q2[i] - q1[i]) * (q2[i] - q1[i]);
        }
        return sqrtf(dist);
    }

    virtual dReal ConfigDist(const dReal *q1, const dReal *q2, int dof) const;

    virtual void SetDOFWeight(int nJointIndex, dReal weight) { _vecJointWeights[nJointIndex] = weight; }

    virtual void SetJointWeight(int nJointIndex, dReal weight) {
        // it is this complicated because each joint can have more than one dof
        assert( nJointIndex >= 0 && nJointIndex < (int)_vecjoints.size() );
        int end = nJointIndex == (int)_vecjoints.size()-1 ? (int)_vecJointWeights.size() : _vecJointIndices[nJointIndex+1];
        for(int i = _vecJointIndices[nJointIndex]; i < end; ++i )
            _vecJointWeights[i] = weight;
    }

    virtual dReal GetJointWeight(int nJointIndex) const { return _vecJointWeights[_vecJointIndices[nJointIndex]]; }

    /// Updates the bounding box and any other parameters that could have changed by a simulation step
    virtual void SimulationStep(dReal fElapsedTime);
    virtual void GetBodyTransformations(std::vector<Transform>& vtrans) const;

    /// returns true if the joint values satisfy all constraints
    //virtual bool IsValidJoint(const dReal* pJointValues, const dReal* pJointVelocities);

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

    /// set the transform of the first link (the rest of the links are computed based on the joint values)
    virtual void SetTransform(const Transform& trans);
    /// applies the transform to the current link's transforamtion
    virtual void ApplyTransform(const Transform& trans);

    virtual AABB ComputeAABB() const;

    virtual Vector GetCenterOfMass() const;
    virtual void Enable(bool bEnable); ///< enables or disables the bodies
    virtual bool IsEnabled() const; ///< returns true if any link of the KinBody is enabled

    /// sets the joint angles manually, if ptrans is not NULL, represents the transformation of the first body
    /// calculates the transformations of every body by treating the first body as set with transformation ptrans
    /// if ptrans is NULL, takes current body transformation
    /// if pvbodies is not NULL, fills it with the values of all the transformations
    virtual void SetJointValues(std::vector<Transform>* pvbodies, const Transform* ptrans, const dReal* pJointValues, bool bCheckLimits = false);

    virtual void SetBodyTransformations(const std::vector<Transform>& vbodies);

    virtual void SetJointVelocities(const dReal* pJointVelocities);

    /// gets the jacobian with respect to a link, pfArray is a 3 x DOF matrix (rotations are not taken into account)
    /// Calculates the partial differentials for all joints that in the path from the root node to _veclinks[index]
    /// (doesn't touch the rest of the values)
    virtual void CalculateJacobian(int index, const Vector& offset, dReal* pfJacobian) const;

    /// calculates the rotational jacobian as a quaternion with respect to an initial rotation
    /// \param index of the link that the rotation is attached to
    /// \param pfJacobian 4xDOF matrix
    virtual void CalculateRotationJacobian(int index, const Vector& qInitialRot, dReal* pfJacobian) const;

    /// calculates the angular velocity jacobian of a specified link about the axes of world coordinates
    /// \param index of the link that the rotation is attached to
    /// \param pfJacobian 3xDOF matrix
    virtual void CalculateAngularVelocityJacobian(int index, dReal* pfJacobian) const;

    /// Check if body is self colliding. Links that are joined together are ignored.
    virtual bool CheckSelfCollision(COLLISIONREPORT* pReport = NULL) const;

    /// used to attach bodies so the collision detection ignores them for planning purposes
    /// this doesn't physically attach them in any way to the bodies. So attached objects can be
    /// far apart.
    /// both bodies are affected!
    virtual void AttachBody(KinBody* pbody);
    virtual void RemoveBody(KinBody* pbody); ///< if pbody is NULL, removes all ignored bodies
    virtual bool IsAttached(const KinBody* pbody) const;
    virtual const std::set<KinBody*>& GetAttached() const { return _setAttachedBodies; }

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const { return false; }

    virtual int GetNetworkId() const;
    static KinBody* GetBodyFromNetworkId(int id);
    
    /// returns how the joint effects the link. If zero, link is unaffected.
    /// If negative, the partial derivative of the Jacobian should be negated.
    virtual char DoesAffect(int jointindex, int linkindex ) const { return _vecJointHierarchy[jointindex*_veclinks.size()+linkindex]; }

    /// writes a string for the forward kinematics of the robot (only hinge joints are handled)
    /// format of the string is:
    /// [link_cur link_base joint_index joint_axis T_left T_right]
    /// basically T(link_cur) = T(link_base) * T_left * Rotation(joint_axis, joint_angle) * T_right
    /// T_left and T_right are 3x4 matrices specified in row order (write the first row first)
    /// joint_axis is the unit vector for the joint_axis
    /// if link_base is -1, attached to static environment
    virtual void WriteForwardKinematics(std::ostream& f);

    virtual void SetGuiData(void* pdata) { _pGuiData = pdata; }
    virtual void* GetGuiData() const { return _pGuiData; }

    virtual const std::string GetXMLFilename() const { return strXMLFilename; }

    virtual const std::set<int>& GetNonAdjacentLinks() const {return _setNonAdjacentLinks;}   
    virtual const std::set<int>& GetAdjacentLinks() const {return _setAdjacentLinks;}   
    
    virtual void* GetPhysicsData() const { return _pPhysicsData; }
    virtual void* GetCollisionData() const { return _pCollisionData; }

    /// The stamp is used by the collision checkers, physics engines, or any other item
    /// that needs to keep track of any changes of the KinBody as it moves.
    /// Currently stamps monotonically increment for every transformation/joint angle change.
    virtual int GetUpdateStamp() const { return _nUpdateStampId; }

    /// Preserves the collision, physics, user, gui data fields. 
    /// Preserves the attached bodies
    /// Preserves the XML readers
    virtual bool Clone(const InterfaceBase* preference, int cloningoptions);

protected:
    /// constructors declared protected so that user always goes through environment to create bodies
    KinBody(PluginType type, EnvironmentBase* penv);
    
    /// specific data about physics engine, should be set only by the current PhysicsEngineBase
    virtual void SetPhysicsData(void* pdata) { _pPhysicsData = pdata; }
    virtual void SetCollisionData(void* pdata) { _pCollisionData = pdata; }

    virtual void ComputeJointHierarchy();

    std::wstring name; ///< name of body

    std::vector<Joint*> _vecjoints;     ///< all the joints of the body, joints contain limits, torques, and velocities
    std::vector<Link*> _veclinks;       ///< children, unlike render hierarchies, transformations
                                        ///< of the children are with respect to the global coordinate system

    std::vector<int> _vecJointIndices;  ///< cached start indices, indexed by joint indices
    std::vector<dReal> _vecJointWeights;///< for configuration distance, this is indexed by DOF
                                        ///< size is equivalent to total number of degrees of freedom in the joints 

    std::vector<char> _vecJointHierarchy;   ///< joint x link, entry is non-zero if the joint affects the link in the forward kinematics
                                            ///< if negative, the partial derivative of ds/dtheta should be negated

    std::vector<Joint*> _vecPassiveJoints; ///< joints not directly controlled

    std::set<int> _setAdjacentLinks;        ///< a set of which links are connected to which if link i and j are connected then
                                            ///< i|(j<<16) will be in the set where i<j.
    std::set<int> _setNonAdjacentLinks;     ///< the not of _setAdjacentLinks
    std::vector< std::pair<std::wstring, std::wstring> > _vForcedAdjacentLinks; ///< internally stores forced adjacent links

    std::set<KinBody*> _setAttachedBodies;

    std::string strXMLFilename;             ///< xml file used to load the body
    mutable std::vector<dReal> _vTempJoints;        ///< used as a temporary buffer

    int networkid;                          ///< unique id of the body used in the network interface
    int _nUpdateStampId;                         ///< unique id for every unique transformation change of any link,
                                            ////< monotically increases as body is updated.
    void* _pGuiData;                        ///< GUI data to let the viewer store specific graphic handles for the object
    void* _pPhysicsData;                ///< data set by the physics engine
    void* _pCollisionData; ///< internal collision model

private:
    virtual const char* GetHash() const { return OPENRAVE_KINBODY_HASH; }
    void (*DestroyCallback)(EnvironmentBase* penv, KinBody* pbody);

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class KinBodyXMLReader;
    friend class JointXMLReader;
    friend class RobotXMLReader;
    friend class Environment;
    friend class ColladaReader;
    friend class ColladaWriter;
#else
    friend class ::KinBodyXMLReader;
    friend class ::JointXMLReader;
    friend class ::RobotXMLReader;
    friend class ::Environment;
    friend class ::ColladaReader;
    friend class ::ColladaWriter;
#endif
#endif

    friend class PhysicsEngineBase;
    friend class CollisionCheckerBase;
};

} // end namespace OpenRAVE

#endif   // KIN_CHAIN_H
