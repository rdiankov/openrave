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
/** \file   robot.h
    \brief  Base robot and manipulator description.
 */
#ifndef OPENRAVE_ROBOT_H
#define OPENRAVE_ROBOT_H

namespace OpenRAVE {

/** \brief <b>[interface]</b> A robot is a kinematic body that has attached manipulators, sensors, and controllers. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_robot.
    \ingroup interfaces
 */
class OPENRAVE_API RobotBase : public KinBody
{
public:
    /// \brief Defines a chain of joints for an arm and set of joints for a gripper. Simplifies operating with them.
    class OPENRAVE_API Manipulator : public boost::enable_shared_from_this<Manipulator>
    {
        Manipulator(RobotBasePtr probot);
        Manipulator(const Manipulator &r);
        Manipulator(RobotBasePtr probot, const Manipulator &r);

public:
        virtual ~Manipulator();

        /// \brief Return the transformation of the end effector (manipulator frame).
        ///
        /// All inverse kinematics and grasping queries are specifying this frame.
        virtual Transform GetTransform() const;

        virtual Transform GetEndEffectorTransform() const {
            return GetTransform();
        }

        virtual const std::string& GetName() const {
            return _name;
        }
        virtual RobotBasePtr GetRobot() const {
            return RobotBasePtr(_probot);
        }

        /// \brief Sets the ik solver and initializes it with the current manipulator.
        ///
        /// Due to complications with translation,rotation,direction,and ray ik,
        /// the ik solver should take into account the grasp transform (_tLocalTool) internally.
        /// The actual ik primitives are transformed into the base frame only.
        virtual bool SetIkSolver(IkSolverBasePtr iksolver);

        /// \brief Returns the currently set ik solver
        virtual IkSolverBasePtr GetIkSolver() const {
            return _pIkSolver;
        }

        /// \deprecated (11/02/08) use GetIkSolver()->GetNumFreeParameters()
        virtual int GetNumFreeParameters() const RAVE_DEPRECATED;

        /// \deprecated (11/02/08) use GetIkSolver()->GetFreeParameters()
        virtual bool GetFreeParameters(std::vector<dReal>& vFreeParameters) const RAVE_DEPRECATED;

        /// \brief the base used for the iksolver
        virtual LinkPtr GetBase() const {
            return _pBase;
        }

        /// \brief the end effector link (used to define workspace distance)
        virtual LinkPtr GetEndEffector() const {
            return _pEndEffector;
        }

        /// \brief Return transform with respect to end effector defining the grasp coordinate system
        virtual Transform GetLocalToolTransform() const {
            return _tLocalTool;
        }

        /// \brief Sets the local tool transform with respect to the end effector.
        ///
        /// Because this call will change manipulator hash, it resets the loaded IK and sets the Prop_RobotManipulatorTool message.
        virtual void SetLocalToolTransform(const Transform& t);

        /// \deprecated (11/10/15) use GetLocalToolTransform
        virtual Transform GetGraspTransform() const RAVE_DEPRECATED {
            return GetLocalToolTransform();
        }

        /// \brief Gripper indices of the joints that the  manipulator controls.
        virtual const std::vector<int>& GetGripperIndices() const {
            return __vgripperdofindices;
        }

        /// \brief Return the indices of the DOFs of the arm (used for IK, etc).
        ///
        /// Usually the DOF indices from pBase to pEndEffector
        virtual const std::vector<int>& GetArmIndices() const {
            return __varmdofindices;
        }

        /// \brief return the normal direction to move joints to 'close' the hand
        virtual const std::vector<dReal>& GetClosingDirection() const {
            return _vClosingDirection;
        }

        /// \brief direction of palm/head/manipulator used for approaching. defined inside the manipulator/grasp coordinate system
        virtual Vector GetLocalToolDirection() const {
            return _vdirection;
        }

        /// \deprecated (11/10/15) use GetLocalToolDirection
        virtual Vector GetDirection() const {
            return GetLocalToolDirection();
        }

        /// \brief Find a close solution to the current robot's joint values.
        ///
        /// The function is a wrapper around the IkSolver interface.
        /// Note that the solution returned is not guaranteed to be the closest solution. In order to compute that, will have to
        /// compute all the ik solutions using FindIKSolutions.
        /// \param param The transformation of the end-effector in the global coord system
        /// \param solution Will be of size GetArmIndices().size() and contain the best solution
        /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        virtual bool FindIKSolution(const IkParameterization& param, std::vector<dReal>& solution, int filteroptions) const;
        virtual bool FindIKSolution(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, std::vector<dReal>& solution, int filteroptions) const;

        /// \brief Find all the IK solutions for the given end effector transform
        ///
        /// \param param The transformation of the end-effector in the global coord system
        /// \param solutions An array of all solutions, each element in solutions is of size GetArmIndices().size()
        /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        virtual bool FindIKSolutions(const IkParameterization& param, std::vector<std::vector<dReal> >& solutions, int filteroptions) const;
        virtual bool FindIKSolutions(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, std::vector<std::vector<dReal> >& solutions, int filteroptions) const;

        /** \brief returns the parameterization of a given IK type for the current manipulator position.

            Ideally pluging the returned ik parameterization into FindIkSolution should return the a manipulator configuration
            such that a new call to GetIkParameterization returns the same values. In other words:
            \code
            ikparam = manip->GetIkParameterization(iktype);
            ... move robot
            std::vector<dReal> sol;
            if( FindIKSolution(ikparam,sol, filteroptions) ) {
                manip->GetRobot()->SetActiveDOFs(manip->GetArmIndices());
                manip->GetRobot()->SetActiveDOFValues(sol);
                BOOST_ASSERT( dist(manip->GetIkParameterization(iktype), ikparam) <= epsilon );
            }
            \endcode
            \param iktype the type of parameterization to request
         */
        virtual IkParameterization GetIkParameterization(IkParameterizationType iktype) const;

        /** \brief returns a full parameterization of a given IK type for the current manipulator position using an existing IkParameterization as the seed.

            Ideally pluging the returned ik parameterization into FindIkSolution should return the a manipulator configuration
            such that a new call to GetIkParameterization returns the same values.
            \param ikparam Some IK types like Lookat3D and TranslationLocalGlobal6D set constraints in the global coordinate system of the manipulator. Because these values are not stored in manipulator itself, they have to be passed in through an existing IkParameterization.
         */
        virtual IkParameterization GetIkParameterization(const IkParameterization& ikparam) const;

        /// \brief Get all child joints of the manipulator starting at the pEndEffector link
        virtual void GetChildJoints(std::vector<JointPtr>& vjoints) const;

        /// \brief Get all child DOF indices of the manipulator starting at the pEndEffector link
        virtual void GetChildDOFIndices(std::vector<int>& vdofndices) const;

        /// \brief returns true if a link is part of the child links of the manipulator.
        ///
        /// The child links do not include the arm links.
        virtual bool IsChildLink(LinkConstPtr plink) const;

        /// \brief Get all child links of the manipulator starting at pEndEffector link.
        ///
        /// The child links do not include the arm links.
        virtual void GetChildLinks(std::vector<LinkPtr>& vlinks) const;

        /** \brief Get all links that are independent of the arm and gripper joints

            In other words, returns all links not on the path from the base to the end effector and not children of the end effector. The base and all links rigidly attached to it are also returned.
         */
        virtual void GetIndependentLinks(std::vector<LinkPtr>& vlinks) const;

        /** \brief Checks collision with only the gripper given its end-effector transform. Ignores disabled links.

            \param tEE the end effector transform
            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        virtual bool CheckEndEffectorCollision(const Transform& tEE, CollisionReportPtr report = CollisionReportPtr()) const;

        /** \brief Checks collision with the environment with all the independent links of the robot. Ignores disabled links.

            \param[out] report [optional] collision report
            \return true if a collision occurred
         */
        virtual bool CheckIndependentCollision(CollisionReportPtr report = CollisionReportPtr()) const;

        /// \brief return true if the body is being grabbed by any link on this manipulator
        virtual bool IsGrabbing(KinBodyConstPtr body) const;

        /// \brief computes the jacobian of the manipulator arm indices from the current manipulator frame origin.
        virtual void CalculateJacobian(boost::multi_array<dReal,2>& mjacobian) const;

        /// \brief computes the quaternion jacobian of the manipulator arm indices from the current manipulator frame rotation.
        virtual void CalculateRotationJacobian(boost::multi_array<dReal,2>& mjacobian) const;

        /// \brief computes the angule axis jacobian of the manipulator arm indices.
        virtual void CalculateAngularVelocityJacobian(boost::multi_array<dReal,2>& mjacobian) const;

        virtual void serialize(std::ostream& o, int options) const;

        /// \brief Return hash of just the manipulator definition.
        virtual const std::string& GetStructureHash() const;

        /// \brief Return hash of all kinematics information that involves solving the inverse kinematics equations.
        ///
        /// This includes joint axes, joint positions, and final grasp transform. Hash is used to cache the solvers.
        virtual const std::string& GetKinematicsStructureHash() const;
protected:
        std::string _name;
        LinkPtr _pBase, _pEndEffector;
        Transform _tLocalTool;
        std::vector<dReal> _vClosingDirection;
        Vector _vdirection;
        IkSolverBasePtr _pIkSolver;
        std::string _strIkSolver;
        std::vector<std::string> _vgripperjointnames;         ///< names of the gripper joints

private:
        RobotBaseWeakPtr _probot;
        std::vector<int> __vgripperdofindices, __varmdofindices;
        mutable std::string __hashstructure, __hashkinematicsstructure;

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
    typedef boost::shared_ptr<RobotBase::Manipulator> ManipulatorPtr;
    typedef boost::shared_ptr<RobotBase::Manipulator const> ManipulatorConstPtr;
    typedef boost::weak_ptr<RobotBase::Manipulator> ManipulatorWeakPtr;

    /// \brief Attaches a sensor to a link on the robot.
    class OPENRAVE_API AttachedSensor : public boost::enable_shared_from_this<AttachedSensor>
    {
public:
        AttachedSensor(RobotBasePtr probot);
        AttachedSensor(RobotBasePtr probot, const AttachedSensor &sensor, int cloningoptions);
        virtual ~AttachedSensor();

        virtual SensorBasePtr GetSensor() const {
            return psensor;
        }
        virtual LinkPtr GetAttachingLink() const {
            return LinkPtr(pattachedlink);
        }
        virtual Transform GetRelativeTransform() const {
            return trelative;
        }
        virtual Transform GetTransform() const {
            return LinkPtr(pattachedlink)->GetTransform()*trelative;
        }
        virtual RobotBasePtr GetRobot() const {
            return RobotBasePtr(_probot);
        }
        virtual const std::string& GetName() const {
            return _name;
        }

        /// retrieves the current data from the sensor
        virtual SensorBase::SensorDataPtr GetData() const;

        virtual void SetRelativeTransform(const Transform& t);

        virtual void serialize(std::ostream& o, int options) const;

        /// \return hash of the sensor definition
        virtual const std::string& GetStructureHash() const;
private:
        RobotBaseWeakPtr _probot;
        SensorBasePtr psensor;
        LinkWeakPtr pattachedlink;         ///< the robot link that the sensor is attached to
        Transform trelative;         ///< relative transform of the sensor with respect to the attached link
        SensorBase::SensorDataPtr pdata;         ///< pointer to a preallocated data using psensor->CreateSensorData()
        std::string _name;         ///< name of the attached sensor
        mutable std::string __hashstructure;
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
    typedef boost::shared_ptr<RobotBase::AttachedSensor> AttachedSensorPtr;
    typedef boost::shared_ptr<RobotBase::AttachedSensor const> AttachedSensorConstPtr;

    /// \brief The information of a currently grabbed body.
    class OPENRAVE_API Grabbed
    {
public:
        KinBodyWeakPtr pbody;         ///< the grabbed body
        LinkPtr plinkrobot;         ///< robot link that is grabbing the body
        std::vector<LinkConstPtr> vCollidingLinks, vNonCollidingLinks;         ///< robot links that already collide with the body
        Transform troot;         ///< root transform (of first link of body) relative to plinkrobot's transform. In other words, pbody->GetTransform() == plinkrobot->GetTransform()*troot
    };

    /// \brief Helper class derived from KinBodyStateSaver to additionaly save robot information.
    class OPENRAVE_API RobotStateSaver : public KinBodyStateSaver
    {
public:
        RobotStateSaver(RobotBasePtr probot, int options = Save_LinkTransformation|Save_LinkEnable|Save_ActiveDOF|Save_ActiveManipulator);
        virtual ~RobotStateSaver();
        virtual void Restore();
protected:
        RobotBasePtr _probot;
        std::vector<int> vactivedofs;
        int affinedofs;
        Vector rotationaxis;
        int nActiveManip;
        std::vector<Grabbed> _vGrabbedBodies;
private:
        virtual void _RestoreRobot();
    };

    virtual ~RobotBase();

    /// \brief Return the static interface type this class points to (used for safe casting).
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_Robot;
    }

    virtual void Destroy();

    /// \deprecated (11/02/18) \see EnvironmentBase::ReadRobotXMLFile
    virtual bool InitFromFile(const std::string& filename, const AttributesList& atts = AttributesList()) RAVE_DEPRECATED;
    /// \deprecated (11/02/18) \see EnvironmentBase::ReadRobotXMLData
    virtual bool InitFromData(const std::string& data, const AttributesList& atts = AttributesList()) RAVE_DEPRECATED;

    /// \brief Returns the manipulators of the robot
    virtual std::vector<ManipulatorPtr>& GetManipulators() {
        return _vecManipulators;
    }

    virtual std::vector<AttachedSensorPtr>& GetAttachedSensors() {
        return _vecSensors;
    }

    virtual void SetDOFValues(const std::vector<dReal>& vJointValues, bool bCheckLimits = false);
    virtual void SetDOFValues(const std::vector<dReal>& vJointValues, const Transform& transbase, bool bCheckLimits = false);

    virtual void SetLinkTransformations(const std::vector<Transform>& vbodies);

    /// Transforms the robot and updates the attached sensors and grabbed bodies.
    virtual void SetTransform(const Transform& trans);

    /** Methods using the active degrees of freedoms of the robot. Active DOFs are a way for the
        user to specify degrees of freedom of interest for a current execution block. All planners
        by default use the robot's active DOF and active manipultor. For every Get* method, there is
        a corresponding GetActive* method rather than the methods when setting joints.  The active
        DOFs also include affine transfomrations of the robot's base. Affine transformation DOFs can
        be found after the joint DOFs in this order: X, Y, Z, Rotation where rotation can be around
        a specified axis a full 3D rotation.  Usually the affine transforamtion is with respect to
        the first link in the body
        @name Affine DOFs
        @{
     */

    /// \deprecated (11/10/04), use OpenRAVE:: global variables
    static const DOFAffine DOF_NoTransform RAVE_DEPRECATED = OpenRAVE::DOF_NoTransform;
    static const DOFAffine DOF_X RAVE_DEPRECATED = OpenRAVE::DOF_X;
    static const DOFAffine DOF_Y RAVE_DEPRECATED = OpenRAVE::DOF_Y;
    static const DOFAffine DOF_Z RAVE_DEPRECATED = OpenRAVE::DOF_Z;
    static const DOFAffine DOF_RotationAxis RAVE_DEPRECATED = OpenRAVE::DOF_RotationAxis;
    static const DOFAffine DOF_Rotation3D RAVE_DEPRECATED = OpenRAVE::DOF_Rotation3D;
    static const DOFAffine DOF_RotationQuat RAVE_DEPRECATED = OpenRAVE::DOF_RotationQuat;

    /** \brief Set the joint indices and affine transformation dofs that the planner should use. If \ref DOF_RotationAxis is specified, the previously set axis is used.

        \param dofindices the indices of the original degrees of freedom to use.
        \param affine A bitmask of \ref DOFAffine values
     */
    virtual void SetActiveDOFs(const std::vector<int>& dofindices, int affine = OpenRAVE::DOF_NoTransform);

    /** \brief Set the joint indices and affine transformation dofs that the planner should use. If \ref DOF_RotationAxis is specified, then rotationaxis is set as the new axis.

        \param dofindices the indices of the original degrees of freedom to use.
        \param affine A bitmask of \ref DOFAffine values
        \param rotationaxis if \ref DOF_RotationAxis is specified, pRotationAxis is used as the new axis
     */
    virtual void SetActiveDOFs(const std::vector<int>& dofindices, int affine, const Vector& rotationaxis);
    virtual int GetActiveDOF() const {
        return _nActiveDOF >= 0 ? _nActiveDOF : GetDOF();
    }
    virtual int GetAffineDOF() const {
        return _nAffineDOFs;
    }

    /// \deprecated (11/10/07)
    virtual int GetAffineDOFIndex(DOFAffine dof) const {
        return GetActiveDOFIndices().size()+RaveGetIndexFromAffineDOF(GetAffineDOF(),dof);
    }

    /// \brief return the configuration specification of the active dofs
    virtual const ConfigurationSpecification& GetActiveConfigurationSpecification() const;

    /// \brief Return the set of active dof indices of the joints.
    virtual const std::vector<int>& GetActiveDOFIndices() const;

    virtual Vector GetAffineRotationAxis() const {
        return vActvAffineRotationAxis;
    }
    virtual void SetAffineTranslationLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotationAxisLimits(const Vector& lower, const Vector& upper);
    virtual void SetAffineRotation3DLimits(const Vector& lower, const Vector& upper);

    /// \brief sets the quaternion limits using a starting rotation and the max angle deviation from it.
    ///
    /// \param quatangle quaternion_start * max_angle. acos(q dot quaternion_start) <= max_angle.
    /// If max_angle is 0, then will take the current transform of the robot
    virtual void SetAffineRotationQuatLimits(const Vector& quatangle);
    virtual void SetAffineTranslationMaxVels(const Vector& vels);
    virtual void SetAffineRotationAxisMaxVels(const Vector& vels);
    virtual void SetAffineRotation3DMaxVels(const Vector& vels);
    virtual void SetAffineRotationQuatMaxVels(dReal vels);
    virtual void SetAffineTranslationResolution(const Vector& resolution);
    virtual void SetAffineRotationAxisResolution(const Vector& resolution);
    virtual void SetAffineRotation3DResolution(const Vector& resolution);
    virtual void SetAffineRotationQuatResolution(dReal resolution);
    virtual void SetAffineTranslationWeights(const Vector& weights);
    virtual void SetAffineRotationAxisWeights(const Vector& weights);
    virtual void SetAffineRotation3DWeights(const Vector& weights);
    virtual void SetAffineRotationQuatWeights(dReal weights);

    virtual void GetAffineTranslationLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotationAxisLimits(Vector& lower, Vector& upper) const;
    virtual void GetAffineRotation3DLimits(Vector& lower, Vector& upper) const;

    /// \brief gets the quaternion limits
    ///
    /// \param quatangle quaternion_start * max_angle. acos(q dot quaternion_start) <= max_angle
    virtual Vector GetAffineRotationQuatLimits() const {
        return _vRotationQuatLimitStart * _fQuatLimitMaxAngle;
    }
    virtual Vector GetAffineTranslationMaxVels() const {
        return _vTranslationMaxVels;
    }
    virtual Vector GetAffineRotationAxisMaxVels() const {
        return _vRotationAxisMaxVels;
    }
    virtual Vector GetAffineRotation3DMaxVels() const {
        return _vRotation3DMaxVels;
    }
    virtual dReal GetAffineRotationQuatMaxVels() const {
        return _fQuatMaxAngleVelocity;
    }
    virtual Vector GetAffineTranslationResolution() const {
        return _vTranslationResolutions;
    }
    virtual Vector GetAffineRotationAxisResolution() const {
        return _vRotationAxisResolutions;
    }
    virtual Vector GetAffineRotation3DResolution() const {
        return _vRotation3DResolutions;
    }
    virtual dReal GetAffineRotationQuatResolution() const {
        return _fQuatAngleResolution;
    }
    virtual Vector GetAffineTranslationWeights() const {
        return _vTranslationWeights;
    }
    virtual Vector GetAffineRotationAxisWeights() const {
        return _vRotationAxisWeights;
    }
    virtual Vector GetAffineRotation3DWeights() const {
        return _vRotation3DWeights;
    }
    virtual dReal GetAffineRotationQuatWeights() const {
        return _fQuatAngleResolution;
    }

    virtual void SetActiveDOFValues(const std::vector<dReal>& values, bool bCheckLimits=false);
    virtual void GetActiveDOFValues(std::vector<dReal>& v) const;
    virtual void SetActiveDOFVelocities(const std::vector<dReal>& velocities, bool bCheckLimits=false);
    virtual void GetActiveDOFVelocities(std::vector<dReal>& velocities) const;
    virtual void GetActiveDOFLimits(std::vector<dReal>& lower, std::vector<dReal>& upper) const;
    virtual void GetActiveDOFResolutions(std::vector<dReal>& v) const;
    virtual void GetActiveDOFWeights(std::vector<dReal>& v) const;
    virtual void GetActiveDOFVelocityLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFAccelerationLimits(std::vector<dReal>& v) const;
    virtual void GetActiveDOFMaxVel(std::vector<dReal>& v) const {
        return GetActiveDOFVelocityLimits(v);
    }
    virtual void GetActiveDOFMaxAccel(std::vector<dReal>& v) const {
        return GetActiveDOFAccelerationLimits(v);
    }

    /// computes the configuration difference q1-q2 and stores it in q1. Takes into account joint limits and circular joints
    virtual void SubtractActiveDOFValues(std::vector<dReal>& q1, const std::vector<dReal>& q2) const;

    /// sets the active manipulator of the robot
    /// \param index manipulator index
    virtual void SetActiveManipulator(int index);
    /// sets the active manipulator of the robot
    /// \param manipname manipulator name
    virtual void SetActiveManipulator(const std::string& manipname);
    virtual ManipulatorPtr GetActiveManipulator();
    virtual ManipulatorConstPtr GetActiveManipulator() const;
    /// \return index of the current active manipulator
    virtual int GetActiveManipulatorIndex() const {
        return _nActiveManip;
    }

    /// \deprecated (11/10/04) send directly through controller
    virtual bool SetMotion(TrajectoryBaseConstPtr ptraj) RAVE_DEPRECATED;

    /// \deprecated (11/10/04)
    virtual bool SetActiveMotion(TrajectoryBaseConstPtr ptraj) RAVE_DEPRECATED;

    /// \deprecated (11/10/04)
    virtual bool SetActiveMotion(TrajectoryBaseConstPtr ptraj, dReal fSpeed) RAVE_DEPRECATED;


    /** \brief Calculates the translation jacobian with respect to a link.

        Calculates the partial differentials for the active degrees of freedom that in the path from the root node to _veclinks[index]
        (doesn't touch the rest of the values).
        \param mjacobian a 3 x ActiveDOF matrix
     */
    virtual void CalculateActiveJacobian(int index, const Vector& offset, boost::multi_array<dReal,2>& mjacobian) const;
    virtual void CalculateActiveJacobian(int index, const Vector& offset, std::vector<dReal>& pfJacobian) const;

    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, boost::multi_array<dReal,2>& vjacobian) const;
    virtual void CalculateActiveRotationJacobian(int index, const Vector& qInitialRot, std::vector<dReal>& pfJacobian) const;

    /** Calculates the angular velocity jacobian of a specified link about the axes of world coordinates.

        \param index of the link that the rotation is attached to
        \param mjacobian 3x(num ACTIVE DOF) matrix
     */
    virtual void CalculateActiveAngularVelocityJacobian(int index, boost::multi_array<dReal,2>& mjacobian) const;
    virtual void CalculateActiveAngularVelocityJacobian(int index, std::vector<dReal>& pfJacobian) const;

    virtual const std::set<int>& GetNonAdjacentLinks(int adjacentoptions=0) const;

    //@}

    /** A grabbed body becomes part of the robot and its relative pose with respect to a robot's
        link will be fixed. KinBody::_AttachBody is called for every grabbed body in order to make
        the grabbed body a part of the robot. Once grabbed, the inter-collisions between the robot
        and the body are regarded as self-collisions; any outside collisions of the body and the
        environment are regarded as environment collisions with the robot.
        @name Grabbing Bodies
        @{
     */

    /** \brief Grab the body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pRobotLinkToGrabWith the link of this robot that will perform the grab
        \param[in] setRobotLinksToIgnore Additional robot link indices that collision checker ignore
        when checking collisions between the grabbed body and the robot.
        \return true if successful and body is grabbed.
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith, const std::set<int>& setRobotLinksToIgnore);

    /** \brief Grab a body with the specified link.

        \param[in] body the body to be grabbed
        \param[in] pRobotLinkToGrabWith the link of this robot that will perform the grab
        \return true if successful and body is grabbed/
     */
    virtual bool Grab(KinBodyPtr body, LinkPtr pRobotLinkToGrabWith);

    /** \brief Grabs the body with the active manipulator's end effector.

        \param[in] body the body to be grabbed
        \param[in] setRobotLinksToIgnore Additional robot link indices that collision checker ignore
        when checking collisions between the grabbed body and the robot.
        \return true if successful and body is grabbed
     */
    virtual bool Grab(KinBodyPtr body, const std::set<int>& setRobotLinksToIgnore);

    /** \brief Grabs the body with the active manipulator's end effector.

        \param[in] body the body to be grabbed
        \return true if successful and body is grabbed
     */
    virtual bool Grab(KinBodyPtr body);

    /** \brief Release the body if grabbed.

        \param body body to release
     */
    virtual void Release(KinBodyPtr body);

    /// Release all grabbed bodies.
    virtual void ReleaseAllGrabbed();     ///< release all bodies

    /** Releases and grabs all bodies, has the effect of recalculating all the initial collision with the bodies.

        This has the effect of resetting the current collisions any grabbed body makes with the robot into an ignore list.
     */
    virtual void RegrabAll();

    /** \brief return the robot link that is currently grabbing the body. If the body is not grabbed, will return an  empty pointer.

        \param[in] body the body to check
     */
    virtual LinkPtr IsGrabbing(KinBodyConstPtr body) const;

    /** \brief gets all grabbed bodies of the robot

        \param[out] vbodies filled with the grabbed bodies
     */
    virtual void GetGrabbed(std::vector<KinBodyPtr>& vbodies) const;
    //@}

    /** \brief Simulate the robot and update the grabbed bodies and attached sensors

        Do not call SimulationStep for the attached sensors in this function.
     */
    virtual void SimulationStep(dReal fElapsedTime);

    /** \brief Check if body is self colliding. Links that are joined together are ignored.

        \param report [optional] collision report
     */
    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr()) const;

    /** \brief checks collision of a robot link with the surrounding environment. Attached/Grabbed bodies to this link are also checked for collision.

        \param[in] ilinkindex the index of the link to check
        \param[in] tlinktrans The transform of the link to check
        \param[out] report [optional] collision report
     */
    virtual bool CheckLinkCollision(int ilinkindex, const Transform& tlinktrans, CollisionReportPtr report = CollisionReportPtr());

    /// does not clone the grabbed bodies since it requires pointers from other bodies (that might not be initialized yet)
    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions);

    /// \return true if this body is derived from RobotBase
    virtual bool IsRobot() const {
        return true;
    }

    virtual void serialize(std::ostream& o, int options) const;

    /// A md5 hash unique to the particular robot structure that involves manipulation and sensing components
    /// The serialization for the attached sensors will not involve any sensor specific properties (since they can change through calibration)
    virtual const std::string& GetRobotStructureHash() const;

    /// \brief gets the robot controller
    virtual ControllerBasePtr GetController() const {
        return ControllerBasePtr();
    }

    /// \brief set a controller for a robot
    /// \param pController - if NULL, sets the controller of this robot to NULL. otherwise attemps to set the controller to this robot.
    /// \param args - the argument list to pass when initializing the controller
    virtual bool SetController(ControllerBasePtr controller, const std::vector<int>& dofindices, int nControlTransformation);

    /// \deprecated (10/11/16)
    virtual bool SetController(ControllerBasePtr controller, const std::string& args) RAVE_DEPRECATED {
        std::vector<int> dofindices;
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices.push_back(i);
        }
        return SetController(controller,dofindices,1);
    }

    /// \deprecated (11/10/04)
    void GetFullTrajectoryFromActive(TrajectoryBasePtr pfulltraj, TrajectoryBaseConstPtr pActiveTraj, bool bOverwriteTransforms=true) RAVE_DEPRECATED;

protected:
    RobotBase(EnvironmentBasePtr penv);

    inline RobotBasePtr shared_robot() {
        return boost::static_pointer_cast<RobotBase>(shared_from_this());
    }
    inline RobotBaseConstPtr shared_robot_const() const {
        return boost::static_pointer_cast<RobotBase const>(shared_from_this());
    }

    /// \brief Proprocess the manipulators and sensors and build the specific robot hashes.
    virtual void _ComputeInternalInformation();

    /// \brief Called to notify the body that certain groups of parameters have been changed.
    ///
    /// This function in calls every registers calledback that is tracking the changes.
    virtual void _ParametersChanged(int parameters);

    std::vector<Grabbed> _vGrabbedBodies; ///< vector of grabbed bodies
    virtual void _UpdateGrabbedBodies();
    virtual void _UpdateAttachedSensors();
    std::vector<ManipulatorPtr> _vecManipulators; ///< \see GetManipulators
    int _nActiveManip; ///< \see GetActiveManipulatorIndex

    std::vector<AttachedSensorPtr> _vecSensors; ///< \see GetAttachedSensors

    std::vector<int> _vActiveDOFIndices, _vAllDOFIndices;
    Vector vActvAffineRotationAxis;
    int _nActiveDOF; ///< Active degrees of freedom; if -1, use robot dofs
    int _nAffineDOFs; ///< dofs describe what affine transformations are allowed

    Vector _vTranslationLowerLimits, _vTranslationUpperLimits, _vTranslationMaxVels, _vTranslationResolutions, _vTranslationWeights;
    /// the xyz components are used if the rotation axis is solely about X,Y,or Z; otherwise the W component is used.
    Vector _vRotationAxisLowerLimits, _vRotationAxisUpperLimits, _vRotationAxisMaxVels, _vRotationAxisResolutions, _vRotationAxisWeights;
    Vector _vRotation3DLowerLimits, _vRotation3DUpperLimits, _vRotation3DMaxVels, _vRotation3DResolutions, _vRotation3DWeights;
    Vector _vRotationQuatLimitStart;
    dReal _fQuatLimitMaxAngle, _fQuatMaxAngleVelocity, _fQuatAngleResolution, _fQuatAngleWeight;

    ConfigurationSpecification _activespec;
private:
    virtual const char* GetHash() const {
        return OPENRAVE_ROBOT_HASH;
    }
    virtual const char* GetKinBodyHash() const {
        return OPENRAVE_KINBODY_HASH;
    }
    mutable std::string __hashrobotstructure;
    mutable std::vector<dReal> _vTempRobotJoints;

#ifdef RAVE_PRIVATE
#ifdef _MSC_VER
    friend class Environment;
    friend class ColladaReader;
    friend class ColladaWriter;
    friend class OpenRAVEXMLParser::RobotXMLReader;
    friend class OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class OpenRAVEXMLParser::AttachedSensorXMLReader;
#else
    friend class ::Environment;
    friend class ::ColladaReader;
    friend class ::ColladaWriter;
    friend class ::OpenRAVEXMLParser::RobotXMLReader;
    friend class ::OpenRAVEXMLParser::ManipulatorXMLReader;
    friend class ::OpenRAVEXMLParser::AttachedSensorXMLReader;
#endif
#endif
    friend class RaveDatabase;
};

} // end namespace OpenRAVE

#endif   // ROBOT_H
