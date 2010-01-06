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
  \file   Robot.h
  \brief  Encapsulate a virtual robot description
 -------------------------------------------------------------------- */

#ifndef  RAVE_ROBOT_H
#define  RAVE_ROBOT_H

namespace OpenRAVE {

/// General dynamic body that has manipulators, controllers, and sensors
class RAVE_API RobotBase : public KinBody
{
public:
    /// A set of properties for the kinbody. These properties are used to describe a set of variables used in KinBody.
    enum RobotProperty {
        Prop_Manipulators = 0x00010000, ///< all properties of all manipulators
        Prop_Sensors = 0x00020000, ///< all properties of all sensors
        Prop_SensorPlacement = 0x00040000, ///< relative sensor placement of sensors
    };

    /// handles manipulators of the robot (usually 1 dim)
    class RAVE_API Manipulator : public boost::enable_shared_from_this<Manipulator>
    {
        Manipulator(RobotBasePtr probot);
        Manipulator(const Manipulator& r);
        Manipulator(RobotBasePtr probot, const Manipulator& r);

    public:
        virtual ~Manipulator();

        virtual Transform GetEndEffectorTransform() const;

        virtual const std::string& GetName() const { return _name; }
        virtual RobotBasePtr GetRobot() const { return RobotBasePtr(_probot); }

		virtual void SetIKSolver(IkSolverBasePtr iksolver);
        virtual bool InitIKSolver();
        virtual const std::string& GetIKSolverName() const;
        virtual bool HasIKSolver() const;

        /// the base used for the iksolver
        virtual LinkPtr GetBase() const { return _pBase; }

        /// the end effector link (used to define workspace distance)
        virtual LinkPtr GetEndEffector() const { return _pEndEffector; }

        /// \return transform with respect to end effector defining the grasp coordinate system
        virtual Transform GetGraspTransform() const { return _tGrasp; }

        /// gripper indices of the joints that the  manipulator controls
        virtual const std::vector<int>& GetGripperJoints() const { return _vgripperjoints; }

        /// indices of the joints of the arm (used for IK, etc).
        /// They are usually the joints from pBase to pEndEffector
        virtual const std::vector<int>& GetArmJoints() const { return _varmjoints; }

        /// normal direction to move joints to 'close' the hand
        virtual const std::vector<dReal>& GetClosingDirection() const { return _vClosingDirection; }

        /// direction of palm used for approaching inside the grasp coordinate system
        virtual Vector GetPalmDirection() const { return _vpalmdirection; }

        /// \return Number of free parameters defining the null solution space.
        ///         Each parameter is always in the range of [0,1].
        virtual int GetNumFreeParameters() const;

        /// gets the free parameters from the current robot configuration
        /// \param pFreeParameters is filled with GetNumFreeParameters() parameters in [0,1] range
        /// \return true if succeeded
        virtual bool GetFreeParameters(std::vector<dReal>& vFreeParameters) const;

        /// will find the closest solution to the current robot's joint values
        /// Note that this does NOT use the active dof of the robot
        /// \param goal The transformation of the end-effector in the global coord system
        /// \param solution Will be of size _varmjoints.size() and contain the best solution
        /// \param bColCheck If true, will check collision with the environment. If false, will ignore environment. In every case, self-collisions with the robot are checked.
        virtual bool FindIKSolution(const Transform& goal, std::vector<dReal>& solution, bool bColCheck) const;
        virtual bool FindIKSolution(const Transform& goal, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, bool bColCheck) const;

        /// will find all the IK solutions for the given end effector transform
        /// \param goal The transformation of the end-effector in the global coord system
        /// \param solutions An array of all solutions, each element in solutions is of size _varmjoints.size()
        /// \param bColCheck If true, will check collision with the environment. If false, will ignore environment. In every case, self-collisions with the robot are checked.
        virtual bool FindIKSolutions(const Transform& goal, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const;
        virtual bool FindIKSolutions(const Transform& goal, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const;

        /// get all child joints of the manipulator starting at the pEndEffector link
        virtual void GetChildJoints(std::vector<JointPtr>& vjoints) const;

        /// get all child DOF indices of the manipulator starting at the pEndEffector link
        virtual void GetChildDOFIndices(std::vector<int>& vdofndices) const;

        /// get all child links of the manipulator starting at pEndEffector link
        virtual void GetChildLinks(std::vector<LinkPtr>& vlinks) const;

        /// Get all links that are independent of the arm and gripper joints  conditioned that the base and end effector links are static.
        /// In other words, returns all links not on the path from the base to the end effector and not children of the end effector.
        virtual void GetIndependentLinks(std::vector<LinkPtr>& vlinks) const;

        /// checks collision with only the gripper given its end-effector transform
        /// \param tEE the end effector transform
        /// \return true if a collision occurred
        virtual bool CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report = CollisionReportPtr()) const;

        /// checks collision with the environment with all the independent links of the robot
        /// \return true if a collision occurred
        virtual bool CheckIndependentCollision(CollisionReportPtr report = CollisionReportPtr()) const;

        virtual void serialize(std::ostream& o, int options) const;
        
    private:
        RobotBaseWeakPtr _probot;
        LinkPtr _pBase;
		LinkPtr _pEndEffector;
        Transform _tGrasp;
        std::vector<int> _vgripperjoints;
        std::vector<int> _varmjoints;
        
        std::vector<dReal> _vClosingDirection;
        Vector _vpalmdirection;

		IkSolverBasePtr _pIkSolver;
        std::string _strIkSolver;         ///< string name of the iksolver
        std::string _name;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class ColladaReader;
        friend class OpenRAVEXMLParser::ManipulatorXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
#else
        friend class ::ColladaReader;
        friend class ::OpenRAVEXMLParser::ManipulatorXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
#endif
#endif
        friend class RobotBase;
    };
    typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
    typedef boost::shared_ptr<Manipulator const> ManipulatorConstPtr;
    typedef boost::weak_ptr<Manipulator> ManipulatorWeakPtr;

    class RAVE_API AttachedSensor : public boost::enable_shared_from_this<AttachedSensor>
    {
    public:
        AttachedSensor(RobotBasePtr probot);
        AttachedSensor(RobotBasePtr probot, const AttachedSensor& sensor, int cloningoptions);
        virtual ~AttachedSensor();
        
        virtual SensorBasePtr GetSensor() const { return psensor; }
        virtual LinkPtr GetAttachingLink() const { return LinkPtr(pattachedlink); }
        virtual Transform GetRelativeTransform() const { return trelative; }
        virtual Transform GetTransform() const { return LinkPtr(pattachedlink)->GetTransform()*trelative; }
        virtual RobotBasePtr GetRobot() const { return RobotBasePtr(_probot); }
        virtual const std::string& GetName() const { return _name; }

        /// retrieves the current data from the sensor
        virtual SensorBase::SensorDataPtr GetData() const;

        virtual void SetRelativeTransform(const Transform& t);

        virtual void serialize(std::ostream& o, int options) const;

    private:
        RobotBaseWeakPtr _probot;
        SensorBasePtr psensor;
        LinkWeakPtr pattachedlink; ///< the robot link that the sensor is attached to
        Transform trelative; ///< relative transform of the sensor with respect to the attached link
        SensorBase::SensorDataPtr pdata; ///< pointer to a preallocated data struct using psensor->CreateSensorData()
        std::string _name; ///< name of the attached sensor

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class ColladaReader;
        friend class OpenRAVEXMLParser::AttachedSensorXMLReader;
        friend class OpenRAVEXMLParser::RobotXMLReader;
#else
        friend class ::ColladaReader;
        friend class ::OpenRAVEXMLParser::AttachedSensorXMLReader;
        friend class ::OpenRAVEXMLParser::RobotXMLReader;
#endif
#endif
        friend class RobotBase;
    };
    typedef boost::shared_ptr<AttachedSensor> AttachedSensorPtr;
    typedef boost::shared_ptr<AttachedSensor const> AttachedSensorConstPtr;

    /// describes the currently grabbed bodies
    struct GRABBED
    {
        KinBodyWeakPtr pbody; ///< the grabbed body
        LinkPtr plinkrobot; ///< robot link that is grabbing the body
        std::vector<LinkConstPtr> vCollidingLinks; ///< robot links that already collide with the body
        Transform troot; // root transform (of first link) relative to end effector
    };

    /// Helper class to save the entire robot state and its grabbed bodies
    /// Also saves and restores the current active degrees of freedom
    class RAVE_API RobotStateSaver : public KinBodyStateSaver
    {
    public:
        RobotStateSaver(RobotBasePtr probot);
        virtual ~RobotStateSaver();
    protected:
        std::vector<int> vactivedofs;
        int affinedofs;
        Vector rotationaxis;
        RobotBasePtr _probot;
    };

    virtual ~RobotBase();

    virtual void Destroy();

    /// Build the robot from a file
    virtual bool InitFromFile(const std::string& filename, const std::list<std::pair<std::string,std::string> >& atts);
    virtual bool InitFromData(const std::string& data, const std::list<std::pair<std::string,std::string> >& atts);

    /// returns true if reached goal
    virtual std::vector<ManipulatorPtr>& GetManipulators() { return _vecManipulators; }
    virtual void SetMotion(TrajectoryBaseConstPtr ptraj) {}

    /// manipulators, grasping (KinBodys), usually involves the active manipulator
    virtual void SetActiveManipulator(int index);
    virtual ManipulatorPtr GetActiveManipulator();
    virtual ManipulatorConstPtr GetActiveManipulator() const;
    virtual int GetActiveManipulatorIndex() const { return _nActiveManip; }

    virtual std::vector<AttachedSensorPtr>& GetSensors() { return _vecSensors; }    
    virtual ControllerBasePtr GetController() const { return ControllerBasePtr(); }
    
    /// set a controller for a robot and destroy the old
    /// \param pController - if NULL, sets the controller of this robot to NULL. otherwise attemps to set the controller to this robot.
    /// \param args - the argument list to pass when initializing the controller
    /// \param bDestroyOldController - if true, will destroy the old controller
    virtual bool SetController(ControllerBasePtr pController, const std::string& args) {return false;}
    virtual void SetJointValues(const std::vector<dReal>& vJointValues, bool bCheckLimits = false);
    virtual void SetJointValues(const std::vector<dReal>& vJointValues, const Transform& transbase, bool bCheckLimits = false);

    virtual void SetBodyTransformations(const std::vector<Transform>& vbodies);
    virtual void SetTransform(const Transform& trans);

    /** Set of active degrees of freedoms that all planners plan for. Planner should use the corresponding
     *  GetActive* methods rather than the Get* methods when setting joints.
     *  The affine transformation DOFs can be found after the joint DOFs in this order: X, Y, Z, Rotation
     *  where rotation is one value if a rotation axis is specified or 3 values (angle * axis) if a full 3D rotation is specified.
     *  Usually the affine transforamtion is with respect to the first link in the body
     */
    //@{

    /// if planner should plan with affine transformations, use this enumeartion to specify the real dofs
    enum DOFAffine
    {
        DOF_NoTransform = 0,
        DOF_X = 1, ///< robot can move in the x direction
        DOF_Y = 2, ///< robot can move in the y direction
        DOF_Z = 4, ///< robot can move in the z direction

        // DOF_RotationX fields are mutually exclusive
        DOF_RotationAxis = 8, ///< robot can rotate around an axis (1 dof)
        DOF_Rotation3D = 16, ///< robot can rotate freely (3 dof), the parameterization is
                             ///< theta * v, where v is the rotation axis and theta is the angle about that axis
        DOF_RotationQuat = 32, ///< robot can rotate freely (4 dof), parameterization is a quaternion
    };
    
    /// Set the joint indices and affine transformation dofs that the planner should use
    /// \param nAffineDOsBitmask A bitmask of DOFAffine values (DOF_X)
    /// if DOF_RotationAxis is specified, pRotationAxis is used as the axis
    virtual void SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOsBitmask = DOF_NoTransform);
    virtual void SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOsBitmask, const Vector& pRotationAxis);
    virtual int GetActiveDOF() const { return _nActiveDOF != 0 ? _nActiveDOF : GetDOF(); }
    virtual int GetAffineDOF() const { return _nAffineDOFs; }

    /// if dof is set in the affine dofs, returns its index in the dof values array, otherwise returns -1
    virtual int GetAffineDOFIndex(DOFAffine dof) const;

    virtual Vector GetAffineRotationAxis() const { return vActvAffineRotationAxis; }
    virtual void SetAffineTranslationLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotation3DLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotationQuatLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineTranslationMaxVels(const Vector& vels);
    virtual void SetAffineRotationAxisMaxVels(const Vector& vels);
    virtual void SetAffineRotation3DMaxVels(const Vector& vels);
    virtual void SetAffineRotationQuatMaxVels(const Vector& vels);
    virtual void SetAffineTranslationResolution(const Vector& resolution);
    virtual void SetAffineRotationAxisResolution(const Vector& resolution);
    virtual void SetAffineRotation3DResolution(const Vector& resolution);
    virtual void SetAffineRotationQuatResolution(const Vector& resolution);

    virtual void GetAffineTranslationLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotation3DLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotationQuatLimits(Vector& lower, Vector& upper) const;
    virtual Vector GetAffineTranslationMaxVels() const { return _vTranslationMaxVels; }
    virtual Vector GetAffineRotationAxisMaxVels() const { return _vRotationAxisMaxVels; }
    virtual Vector GetAffineRotation3DMaxVels() const { return _vRotation3DMaxVels; }
    virtual Vector GetAffineRotationQuatMaxVels() const { return _vRotationQuatMaxVels; }
    virtual Vector GetAffineTranslationResolution() const { return _vTranslationResolutions; }
    virtual Vector GetAffineRotationAxisResolution() const { return _vRotationAxisResolutions; }
    virtual Vector GetAffineRotation3DResolution() const { return _vRotation3DResolutions; }
    virtual Vector GetAffineRotationQuatResolution() const { return _vRotationQuatResolutions; }

    virtual void SetActiveDOFValues(const std::vector<dReal>& values, bool bCheckLimits=false);
    virtual void GetActiveDOFValues(std::vector<dReal>& v) const;
    virtual void SetActiveDOFVelocities(const std::vector<dReal>& velocities);
    virtual void GetActiveDOFVelocities(std::vector<dReal>& velocities) const;
    virtual void GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const;
    virtual void GetActiveDOFResolutions(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxVel(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxAccel(std::vector<dReal>& v) const;

    /// Specifies the controlled degrees of freedom used to control the robot through torque
    /// In the general sense, it is not always the case that there's a one-to-one mapping
    /// between a robot's joints and the motors used to control the robot. A good example
    /// of this is a differential-drive robot. If developers need such a robot, they
    /// should derive from RobotBase and override these methods. The function that maps
    /// control torques to actual movements of the robot should be put in SimulationStep.
    /// As default, the control degrees of freedom are tied directly to the active degrees
    /// of freedom; the max torques for affine dofs are 0 in this case.
    //@{
    virtual int GetControlDOF() const { return GetActiveDOF(); }
    virtual void GetControlMaxTorques(std::vector<dReal>& vmaxtorque) const;
    virtual void SetControlTorques(const std::vector<dReal>& pTorques);
    //@}

    /// Converts a trajectory specified with the current active degrees of freedom to a full joint/transform trajectory
    /// \param pFullTraj written with the final trajectory,
    /// \param pActiveTraj the input active dof trajectory
    /// \param bOverwriteTransforms if true will use the current robot transform as the base (ie will ignore any transforms specified in pActiveTraj). If false, will use the pActiveTraj transforms specified
    virtual void GetFullTrajectoryFromActive(TrajectoryBasePtr pFullTraj, TrajectoryBaseConstPtr pActiveTraj, bool bOverwriteTransforms = true);
    virtual int GetActiveJointIndex(int active_index) const {
        if( _nActiveDOF == 0 )
            return active_index;
        return active_index < (int)_vActiveJointIndices.size() ? _vActiveJointIndices[active_index] : -1;
    }
    virtual const std::vector<int>& GetActiveJointIndices();

    virtual void SetActiveMotion(TrajectoryBaseConstPtr ptraj) {}

    /// the speed at which the robot should go at
    virtual void SetActiveMotion(TrajectoryBaseConstPtr ptraj, dReal fSpeed) { SetActiveMotion(ptraj); }
    //@}

    /// gets the jacobian with respect to a link, pfArray is a 3 x ActiveDOF matrix (rotations are not taken into account)
    /// Calculates the partial differentials for the active degrees of freedom that in the path from the root node to _veclinks[index]
    /// (doesn't touch the rest of the values)
    virtual void CalculateActiveJacobian(int index, const Vector& offset, std::vector<dReal>& pfJacobian) const;
    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, std::vector<dReal>& pfJacobian) const;
    /// calculates the angular velocity jacobian of a specified link about the axes of world coordinates
    /// \param index of the link that the rotation is attached to
    /// \param pfJacobian 3x(num ACTIVE DOF) matrix
    virtual void CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& pfJacobian) const;

    /// grab and release the body with the active manipulator. A grabbed body becomes part of the robot
    /// and its relative pose with respect to a robot's link will be fixed. AttachBody is called for every
    /// grabbed body, so KinBody::GetAttached() can be used to figure out if anything is grabbing a body.
    //@{
    /// grabs a body with the current active manipulator. Body will be attached to the manipulator's end-effector
    /// \param pbody the body to be grabbed
    /// \return true if successful
    virtual bool Grab(KinBodyPtr pbody);

    /// grab and release the body with the active manipulator. A grabbed body becomes part of the robot
    /// and its relative pose with respect to a robot's link will be fixed. AttachBody is called for every
    /// grabbed body, so KinBody::GetAttached() can be used to figure out if anything is grabbing a body.
    //@{
    /// grabs a body with the current active manipulator. Body will be attached to the manipulator's end-effector
    /// \param pbody the body to be grabbed
    /// \param setRobotLinksToIgnore Additional robot link indices that collision checker ignore
    ///        when checking collisions between the grabbed body and the robot.
    /// \return true if successful
    virtual bool Grab(KinBodyPtr pbody, const std::set<int>& setRobotLinksToIgnore);

    /// grabs a body with a specified robot link
    /// \param pbody the body to be grabbed
    /// \param plink the link of this robot that will perform the grab
    /// \return true if successful
    virtual bool Grab(KinBodyPtr pbody, LinkPtr plink);

    /// grabs a body with a specified robot link
    /// \param pbody the body to be grabbed
    /// \param plink the link of this robot that will perform the grab
    /// \param setRobotLinksToIgnore Additional robot link indices that collision checker ignore
    ///        when checking collisions between the grabbed body and the robot.
    /// \return true if successful
    virtual bool Grab(KinBodyPtr pbody, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore);

    virtual void Release(KinBodyPtr pbody); ///< release the body
    virtual void ReleaseAllGrabbed(); ///< release all bodies

    /// releases and grabs all bodies, has the effect of recalculating all the initial collision with the bodies.
    /// In other words, the current collisions any grabbed body makes with the robot will be re-inserted into an ignore list
    virtual void RegrabAll();

    /// \return the robot link that is currently grabbing the body. If the body is not grabbed, will return an  empty pointer.
    virtual LinkPtr IsGrabbing(KinBodyConstPtr pbody) const;

    /// gets all grabbed bodies whose collisions should also be checked
    virtual void GetGrabbed(std::vector<KinBodyPtr>& vbodies) const;
    //@}

    /// Add simulating attached sensors
    virtual void SimulationStep(dReal fElapsedTime);

    /// Check if body is self colliding. Links that are joined together are ignored.
    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr()) const;
    
    /// does not clone the grabbed bodies since it requires pointers from other bodies (that might not be initialized yet)
    virtual bool Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const { return true; }

    virtual void ComputeJointHierarchy();

    virtual void serialize(std::ostream& o, int options) const;

    /// A md5 hash unique to the particular robot structure that involves manipulation and sensing components
    /// The serialization for the attached sensors will not involve any sensor specific properties (since they can change through calibration)
    virtual std::string GetRobotStructureHash() const;
    
protected:
    RobotBase(EnvironmentBasePtr penv);

    inline RobotBasePtr shared_robot() { return boost::static_pointer_cast<RobotBase>(shared_from_this()); }
    inline RobotBaseConstPtr shared_robot_const() const { return boost::static_pointer_cast<RobotBase const>(shared_from_this()); }

    std::vector<GRABBED> _vGrabbedBodies;   ///vector of grabbed bodies
    virtual void _UpdateGrabbedBodies();
    virtual void _UpdateAttachedSensors();

    /// manipulation planning
    std::vector<ManipulatorPtr> _vecManipulators;
    int _nActiveManip;                  ///< active manipulator

    std::vector<AttachedSensorPtr> _vecSensors;

    std::vector<int> _vActiveJointIndices, _vAllJointIndices;
    Vector vActvAffineRotationAxis;
    int _nActiveDOF;            ///< Active degrees of freedom; if 0, use robot dofs
    int _nAffineDOFs;           ///< dofs describe what affine transformations are allowed

    Vector _vTranslationLowerLimits, _vTranslationUpperLimits, _vTranslationMaxVels, _vTranslationResolutions;
    Vector _vRotationAxisLowerLimits, _vRotationAxisUpperLimits, _vRotationAxisMaxVels, _vRotationAxisResolutions;
    Vector _vRotation3DLowerLimits, _vRotation3DUpperLimits, _vRotation3DMaxVels, _vRotation3DResolutions;
    Vector _vRotationQuatLowerLimits, _vRotationQuatUpperLimits, _vRotationQuatMaxVels, _vRotationQuatResolutions;
private:
    std::string __hashrobotstructure;
    mutable std::vector<dReal> _vTempRobotJoints;
    virtual const char* GetHash() const { return OPENRAVE_ROBOT_HASH; }
    virtual const char* GetKinBodyHash() const { return OPENRAVE_KINBODY_HASH; }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class RaveDatabase;
    friend class Environment;
    friend class ColladaReader;
    friend class ColladaWriter;
    friend class OpenRAVEXMLParser::RobotXMLReader;
    friend class OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class OpenRAVEXMLParser::AttachedSensorXMLReader;
#else
    friend class ::RaveDatabase;
    friend class ::Environment;
    friend class ::ColladaReader;
    friend class ::ColladaWriter;
    friend class ::OpenRAVEXMLParser::RobotXMLReader;
    friend class ::OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class ::OpenRAVEXMLParser::AttachedSensorXMLReader;
#endif
#endif
};

} // end namespace OpenRAVE

#endif   // ROBOT_H
