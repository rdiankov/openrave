// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
class RobotBase : public KinBody
{
public:
    /// handles manipulators of the robot (usually 1 dim)
    class Manipulator
    {
    public:
        Manipulator(RobotBase* probot);
        Manipulator(const RobotBase::Manipulator& r);
        Manipulator(RobotBase* probot, const Manipulator& r);
        virtual ~Manipulator();

        virtual Transform GetEndEffectorTransform() const;

        virtual const char* GetName() const { return _name.c_str(); }

		virtual void SetIKSolver(IkSolverBase* iksolver);
        virtual bool InitIKSolver(int options);
        virtual const std::string& GetIKSolverName() const;
        virtual bool HasIKSolver() const;

        /// \return Number of free parameters defining the null solution space.
        ///         Each parameter is always in the range of [0,1].
        virtual int GetNumFreeParameters() const;

        /// gets the free parameters from the current robot configuration
        /// \param pFreeParameters is filled with GetNumFreeParameters() parameters in [0,1] range
        /// \return true if succeeded
        virtual bool GetFreeParameters(dReal* pFreeParameters) const;

        /// will find the closest solution to the current robot's joint values
        /// Note that this does NOT use the active dof of the robot
        /// \param goal The transformation of the end-effector in the global coord system
        /// \param solution Will be of size _vecarmjoints.size() and contain the best solution
        /// \param bColCheck If true, will check collision with the environment. If false, will ignore environment. In every case, self-collisions with the robot are checked.
        virtual bool FindIKSolution(const Transform& goal, std::vector<dReal>& solution, bool bColCheck) const;
        virtual bool FindIKSolution(const Transform& goal, const dReal* pFreeParameters, std::vector<dReal>& solution, bool bColCheck) const;

        /// will find all the IK solutions for the given end effector transform
        /// \param goal The transformation of the end-effector in the global coord system
        /// \param solutions An array of all solutions, each element in solutions is of size _vecarmjoints.size()
        /// \param bColCheck If true, will check collision with the environment. If false, will ignore environment. In every case, self-collisions with the robot are checked.
        virtual bool FindIKSolutions(const Transform& goal, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const;
        virtual bool FindIKSolutions(const Transform& goal, const dReal* pFreeParameters, std::vector<std::vector<dReal> >& solutions, bool bColCheck) const;

        Link* pBase;				///< the base used for the iksolver
		Link* pEndEffector;         ///< the end effector link (used to define workspace distance)
        Transform tGrasp;           ///< transform with respect to end effector for grasping goals
        std::vector<int> _vecjoints; ///< indices of the joints that the  manipulator controls
        std::vector<int> _vecarmjoints; ///< indices of the joints of the arm (used for IK, etc). They are usually the joints from pBase to pEndEffector
        std::vector<dReal> _vClosedGrasp, _vOpenGrasp; ///< closed and open grasps
        
    private:
        RobotBase* _probot;
		IkSolverBase* _pIkSolver;
        std::string _strIkSolver;         ///< string name of the iksolver
        int _ikoptions;
        std::string _name;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class ManipulatorXMLReader;
        friend class RobotXMLReader;
#else
        friend class ::ManipulatorXMLReader;
        friend class ::RobotXMLReader;
#endif
#endif
    };

    class AttachedSensor
    {
    public:
        AttachedSensor(RobotBase* probot);
        AttachedSensor(RobotBase* probot, const AttachedSensor& sensor, int cloningoptions);
        virtual ~AttachedSensor();
        
        virtual SensorBase* GetSensor() const { return psensor; }
        virtual Link* GetAttachingLink() const { return pattachedlink; }
        virtual Transform GetRelativeTransform() const { return trelative; }
        virtual RobotBase* GetRobot() const { return _probot; }
        virtual const char* GetName() const { return _name.c_str(); }

        // retrieves the current data from the sensor
        virtual SensorBase::SensorData* GetData() const;
        
    private:
        RobotBase* _probot;
        SensorBase* psensor;
        Link* pattachedlink; ///< the robot link that the sensor is attached to
        Transform trelative; ///< relative transform of the sensor with respect to the attached link
        SensorBase::SensorData* pdata; ///< pointer to a preallocated data struct using psensor->CreateSensorData()
        std::string _name; ///< name of the attached sensor

        friend class RobotBase;
        
#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
        friend class AttachedSensorXMLReader;
        friend class RobotXMLReader;
#else
        friend class ::AttachedSensorXMLReader;
        friend class ::RobotXMLReader;
#endif
#endif
    };

    /// describes the currently grabbed bodies
    struct GRABBED
    {
        KinBody* pbody; ///< the grabbed body
        Link* plinkrobot; ///< robot link that is grabbing the body
        std::set<KinBody::Link*> sValidColLinks; ///< links that are initially in collision
                                               ///< with the robot, used for self collisions
        Transform troot; // root transform (of first link) relative to end effector
    };

    /// Helper class to save the entire robot state and its grabbed bodies
    /// Also saves and restores the current active degrees of freedom
    class RobotStateSaver : public KinBodyStateSaver
    {
    public:
        RobotStateSaver(RobotBase* probot);
        virtual ~RobotStateSaver();
        
    protected:
        std::vector<int> vactivedofs;
        int affinedofs;
        Vector rotationaxis;
        RobotBase* _probot;
    };

    virtual ~RobotBase();

	virtual const char* GetXMLId() { return "RobotBase"; }

    virtual void Destroy();

    /// Build the robot from a file
    virtual bool Init(const char* filename, const char**atts);

    /// returns true if reached goal
    virtual std::vector<Manipulator>& GetManipulators() { return _vecManipulators; }
    virtual void SetMotion(const Trajectory* ptraj) {}

    /// manipulators, grasping (KinBodys), usually involves the active manipulator
    virtual void SetActiveManipulator(int index);
    virtual Manipulator* GetActiveManipulator();
    virtual int GetActiveManipulatorIndex() const { return _nActiveManip; }

    virtual std::vector<AttachedSensor>& GetSensors() { return _vecSensors; }
    
    virtual ControllerBase* GetController() const { return NULL; }

    /// set a controller for a robot and destroy the old
    /// \param pname - name of controller to query from environment
    /// \param args - the argument list to pass when initializing the controller
    /// \param bDestroyOldController - if true, will destroy the old controller
    virtual bool SetController(const wchar_t* pname, const char* args=NULL, bool bDestroyOldController=true) {return false;}
    
    /// set a controller for a robot and destroy the old
    /// \param pController - if NULL, sets the controller of this robot to NULL. otherwise attemps to set the controller to this robot.
    /// \param args - the argument list to pass when initializing the controller
    /// \param bDestroyOldController - if true, will destroy the old controller
    virtual bool SetController(ControllerBase * pController, const char* args=NULL, bool bDestroyOldController=true) {return false;}
    
    virtual void SetJointValues(std::vector<Transform>* pvbodies, const Transform* ptrans, const dReal* pJointValues, bool bCheckLimits=false);
    virtual void SetBodyTransformations(const std::vector<Transform>& vbodies);
    virtual void SetTransform(const Transform& trans);
    virtual void ApplyTransform(const Transform& trans);

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
    virtual void SetActiveDOFs(const std::vector<int>& vJointIndices, int nAffineDOsBitmask = DOF_NoTransform, const Vector* pRotationAxis = NULL);
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

    virtual void SetActiveDOFValues(std::vector<Transform>* pvbodies, const dReal* pValues, bool bCheckLimits=false);
    virtual void GetActiveDOFValues(dReal* pValues) const;
    virtual void GetActiveDOFValues(std::vector<dReal>& v) const;
    virtual void SetActiveDOFVelocities(dReal* pVelocities);
    virtual void GetActiveDOFVelocities(dReal* pVelocities) const;
    virtual void GetActiveDOFVelocities(std::vector<dReal>& velocities) const;
    virtual void GetActiveDOFLimits(dReal* pLowerLimit, dReal* pUpperLimit) const;
    virtual void GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const;
    virtual void GetActiveDOFResolutions(dReal* pResolution) const;
    virtual void GetActiveDOFResolutions(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxVel(dReal* pMaxVel) const;
    virtual void GetActiveDOFMaxVel(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxAccel(dReal* pMaxAccel) const;
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
    virtual void GetControlMaxTorques(dReal* pMaxTorque) const;
    virtual void GetControlMaxTorques(std::vector<dReal>& vmaxtorque) const;
    virtual void SetControlTorques(dReal* pTorques);
    //@}

    /// Converts a trajectory specified with the current active degrees of freedom to a full joint/transform trajectory
    /// \param pFullTraj written with the final trajectory,
    /// \param pActiveTraj the input active dof trajectory
    /// \param bOverwriteTransforms if true will use the current robot transform as the base (ie will ignore any transforms specified in pActiveTraj). If false, will use the pActiveTraj transforms specified
    virtual void GetFullTrajectoryFromActive(Trajectory* pFullTraj, const Trajectory* pActiveTraj, bool bOverwriteTransforms = true);
    virtual int GetActiveJointIndex(int active_index) const {
        if( _nActiveDOF == 0 )
            return active_index;
        return active_index < (int)_vActiveJointIndices.size() ? _vActiveJointIndices[active_index] : -1;
    }
    virtual const std::vector<int>& GetActiveJointIndices();

    virtual void SetActiveMotion(const Trajectory* ptraj) {}

    /// the speed at which the robot should go at
    virtual void SetActiveMotion(const Trajectory* ptraj, dReal fSpeed) { SetActiveMotion(ptraj); }
    //@}

    /// gets the jacobian with respect to a link, pfArray is a 3 x ActiveDOF matrix (rotations are not taken into account)
    /// Calculates the partial differentials for the active degrees of freedom that in the path from the root node to _veclinks[index]
    /// (doesn't touch the rest of the values)
    virtual void CalculateActiveJacobian(int index, const Vector& offset, dReal* pfJacobian) const;
    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, dReal* pfJacobian) const;
    /// calculates the angular velocity jacobian of a specified link about the axes of world coordinates
    /// \param index of the link that the rotation is attached to
    /// \param pfJacobian 3x(num ACTIVE DOF) matrix
    virtual void CalculateActiveAngularVelocityJacobian(int index, dReal* pfJacobian) const;

    /// grab and release the body with the active manipulator. A grabbed body becomes part of the robot
    /// and its relative pose with respect to a robot's link will be fixed. AttachBody is called for every
    /// grabbed body, so KinBody::GetAttached() can be used to figure out if anything is grabbing a body.
    //@{
    /// grabs a body with the current active manipulator. Body will be attached to the manipulator's end-effector
    /// \param pbody the body to be grabbed
    /// \param setRobotLinksToIgnore Additional robot link indices that collision checker ignore
    ///        when checking collisions between the grabbed body and the robot.
    /// \return true if successful
    virtual bool Grab(KinBody* pbody, const std::set<int>* psetRobotLinksToIgnore = NULL);
    /// grabs a body with a robot's links pecified by linkindex
    /// \param pbody the body to be grabbed
    /// \param setRobotLinksToIgnore Additional robot link indices that collision checker ignore
    ///        when checking collisions between the grabbed body and the robot.
    /// \return true if successful
    virtual bool Grab(KinBody* pbody, int linkindex, const std::set<int>* psetRobotLinksToIgnore);
    virtual void Release(KinBody* pbody); ///< release the body
    virtual void ReleaseAllGrabbed(); ///< release all bodies

    /// releases and grabs all bodies, has the effect of recalculating all the initial collision with the bodies.
    /// In other words, the current collisions any grabbed body makes with the robot will be re-inserted into an ignore list
    virtual void RegrabAll();
    virtual bool IsGrabbing(KinBody* pbody) const;
    
    /// get the grabbed bodies
    virtual const std::vector<GRABBED>& GetGrabbed() const { return _vGrabbedBodies; }
    //@}

    /// Add simulating attached sensors
    virtual void SimulationStep(dReal fElapsedTime);

    /// Check if body is self colliding. Links that are joined together are ignored.
    virtual bool CheckSelfCollision(COLLISIONREPORT* pReport = NULL) const;
    
    /// does not clone the grabbed bodies since it requires pointers from other bodies (that might not be initialized yet)
    virtual bool Clone(const InterfaceBase* preference, int cloningoptions);

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const { return true; }

protected:
    RobotBase(EnvironmentBase* penv);

    std::vector<GRABBED> _vGrabbedBodies;   ///vector of grabbed bodies
    virtual void _UpdateGrabbedBodies();
    virtual void _UpdateAttachedSensors();

    /// manipulation planning
    std::vector<Manipulator> _vecManipulators;
    int _nActiveManip;                  ///< active manipulator

    std::vector<AttachedSensor> _vecSensors;

    std::vector<int> _vActiveJointIndices, _vAllJointIndices;
    Vector vActvAffineRotationAxis;
    int _nActiveDOF;            ///< Active degrees of freedom; if 0, use robot dofs
    int _nAffineDOFs;           ///< dofs describe what affine transformations are allowed

    Vector _vTranslationLowerLimits, _vTranslationUpperLimits, _vTranslationMaxVels, _vTranslationResolutions;
    Vector _vRotationAxisLowerLimits, _vRotationAxisUpperLimits, _vRotationAxisMaxVels, _vRotationAxisResolutions;
    Vector _vRotation3DLowerLimits, _vRotation3DUpperLimits, _vRotation3DMaxVels, _vRotation3DResolutions;
    Vector _vRotationQuatLowerLimits, _vRotationQuatUpperLimits, _vRotationQuatMaxVels, _vRotationQuatResolutions;

    mutable std::vector<dReal> _vtempjoints; ///< used for temp access

private:    
    virtual const char* GetHash() const { return OPENRAVE_ROBOT_HASH; }
    virtual const char* GetKinBodyHash() const { return OPENRAVE_KINBODY_HASH; }

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class RobotXMLReader;
    friend class RaveDatabase;
    friend class Environment;
#else
    friend class ::RobotXMLReader;
    friend class ::RaveDatabase;
    friend class ::Environment;
#endif
#endif
};

} // end namespace OpenRAVE

#endif   // ROBOT_H
