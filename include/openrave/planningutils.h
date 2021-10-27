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

/** \file planningutils.h
    \brief Planning related utilities likes samplers, distance metrics, etc.

    This file is optional and not automatically included with openrave.h
 */
#ifndef OPENRAVE_PLANNINGUTILS_H
#define OPENRAVE_PLANNINGUTILS_H

#include <openrave/openrave.h>

namespace OpenRAVE {

namespace planningutils {

/// \brief Jitters the active joint angles of the robot until it escapes collision.
///
/// \return 0 if jitter failed and robot is in collision, -1 if robot originally not in collision, 1 if jitter succeeded and position is different.
OPENRAVE_API int JitterActiveDOF(RobotBasePtr robot,int nMaxIterations=5000,dReal fRand=0.03,const PlannerBase::PlannerParameters::NeighStateFn& neighstatefn = PlannerBase::PlannerParameters::NeighStateFn());

/// \brief Jitters the transform of a body until it escapes collision.
OPENRAVE_API bool JitterTransform(KinBodyPtr pbody, float fJitter, int nMaxIterations=1000);

/** \brief If the current configuration does not satisfy constraints, then jitters it using a \ref PlannerBase::PlannerParameters structure

    \param parameters The planner parameters used to define the configuration space to jitter. The following fields are required: _getstatefn, _setstatefn, _vConfigUpperLimit, _vConfigLowerLimit, _checkpathvelocityconstraintsfn, _diffstatefn, _nRandomGeneratorSeed, _samplefn. The following are used and optional : _neighstatefn (used for constraining on manifolds)
    \param maxiterations number of different configurations to test
    \param maxjitter The max deviation of a dof value to jitter. value +- maxjitter
    \param perturbation Test with perturbations since very small changes in angles can produce collision inconsistencies
    \return Return 0 if jitter failed and constraints are not satisfied. -1 if constraints are originally satisfied. 1 if jitter succeeded, configuration is different, and constraints are satisfied.
 */
OPENRAVE_API int JitterCurrentConfiguration(PlannerBase::PlannerParametersConstPtr parameters, int maxiterations=5000, dReal maxjitter=0.015, dReal perturbation=1e-5);

/** \brief validates a trajectory with respect to the planning constraints.

    checks internal data structures and verifies that all trajectory via points do not violate joint position, velocity, and acceleration limits.
    Assume that the environment that is used to create parameters is locked. (If the given parameters is not initialized, will attempt to create a new PlannerParameters with trajectory->GetEnv(). In this case, trajectory->GetEnv() should be locked.)
    \param parameters the planner parameters passed to the planner that returned the trajectory. If not initialized, will attempt to create a new PlannerParameters structure from trajectory->GetConfigurationSpecification()
    \param trajectory trajectory of points to be checked
    \param samplingstep If == 0, then will only test the supports points in trajectory->GetPoints(). If > 0, then will sample the trajectory at this time interval and check that smoothness is satisfied along with segment constraints.
    \throw openrave_exception If the trajectory is invalid, will throw ORE_InconsistentConstraints.
 */
OPENRAVE_API void VerifyTrajectory(PlannerBase::PlannerParametersConstPtr parameters, TrajectoryBaseConstPtr trajectory, dReal samplingstep=0.002);

/** \brief Extends the last ramp of the trajectory in order to reach a goal. THe configuration space matches the positional data of the trajectory.

    Useful when appending jittered points to the trajectory.
    \param index the waypoint index of the trajectory
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t ExtendWaypoint(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner);

/** \brief Extends the last ramp of the trajectory in order to reach a goal. THe configuration space is just the active DOF of the robot.

    Useful when appending jittered points to the trajectory.
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t ExtendActiveDOFWaypoint(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="");

/** \brief Smooth the trajectory points to avoiding collisions by extracting and using the currently set active dofs of the robot. <b>[multi-thread safe]</b>

    Only initial and goal configurations are preserved.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param robot use the robot's active dofs to initialize the trajectory space
    \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory re-timer.
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the smoothing planner
 */

OPENRAVE_API PlannerStatus SmoothActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="");


/** \brief Smooth the trajectory points consisting of affine transformation values while avoiding collisions. <b>[multi-thread safe]</b>

    Only initial and goal configurations are preserved.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param maxvelocities the max velocities of each dof
    \param maxaccelerations the max acceleration of each dof
    \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory re-timer.
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the smoothing planner
 */
OPENRAVE_API PlannerStatus SmoothAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>&maxaccelerations, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Smooth the trajectory points to avoiding collisions by extracting and using the positional data from the trajectory. <b>[multi-thread safe]</b>

    Only initial and goal configurations are preserved.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory re-timer.
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the smoothing planner
 */
OPENRAVE_API PlannerStatus SmoothTrajectory(TrajectoryBasePtr traj, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Retime the trajectory points by extracting and using the currently set active dofs of the robot. <b>[multi-thread safe]</b>

    Collision is not checked. Every waypoint in the trajectory is guaranteed to be hit.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param robot use the robot's active dofs to initialize the trajectory space
    \param plannername the name of the planner to use to retime. If empty, will use the default trajectory re-timer.
    \param hastimestamps if true, use the already initialized timestamps of the trajectory
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the retiming planner
 */
OPENRAVE_API PlannerStatus RetimeActiveDOFTrajectory(TrajectoryBasePtr traj, RobotBasePtr robot, bool hastimestamps=false, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Smoother planner for the trajectory points to avoiding collisions by extracting and using the currently set active dofs of the robot.

    Caches all the planners and parameters so PlanPath can be called multiple times without creating new objects.
    Only initial and goal configurations are preserved.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
 */
class OPENRAVE_API ActiveDOFTrajectorySmoother
{
public:
    /**
       \param robot use the robot's active dofs to initialize the trajectory space
       \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory re-timer.
       \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
     **/
    ActiveDOFTrajectorySmoother(RobotBasePtr robot, const std::string& plannername="", const std::string& plannerparameters="");
    virtual ~ActiveDOFTrajectorySmoother() {
    }

    /// \brief Executes smoothing. <b>[multi-thread safe]</b>
    ///
    /// \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    /// \return PlannerStatus of the status of the smoothing planner
    virtual PlannerStatus PlanPath(TrajectoryBasePtr traj, int planningoptions=0);

protected:
    void _UpdateParameters();

    RobotBasePtr _robot;
    PlannerBasePtr _planner;
    PlannerBase::PlannerParametersPtr _parameters;
    std::vector<int> _vRobotActiveIndices;
    int _nRobotAffineDOF;
    Vector _vRobotRotationAxis;
    UserDataPtr _changehandler; ///< tracks changes for the robot and re-initializes parameters
};

typedef boost::shared_ptr<ActiveDOFTrajectorySmoother> ActiveDOFTrajectorySmootherPtr;

/** \brief Retimer planner the trajectory points by extracting and using the currently set active dofs of the robot. <b>[multi-thread safe]</b>

    Caches all the planners and parameters so PlanPath can be called multiple times without creating new objects.
    Collision is not checked. Every waypoint in the trajectory is guaranteed to be hit.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
 */
class OPENRAVE_API ActiveDOFTrajectoryRetimer
{
public:
    /**
       \param robot use the robot's active dofs to initialize the trajectory space
       \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory re-timer.
       \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
       \param hastimestamps if true, use the already initialized timestamps of the trajectory
     **/
    ActiveDOFTrajectoryRetimer(RobotBasePtr robot, const std::string& plannername="", const std::string& plannerparameters="");
    virtual ~ActiveDOFTrajectoryRetimer() {
    }

    /// \brief Executes smoothing. <b>[multi-thread safe]</b>
    ///
    /// \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    /// \return PlannerStatus of the status of the smoothing planner
    virtual PlannerStatus PlanPath(TrajectoryBasePtr traj, bool hastimestamps=false, int planningoptions=0);

protected:
    void _UpdateParameters();

    RobotBasePtr _robot;
    PlannerBasePtr _planner;
    PlannerBase::PlannerParametersPtr _parameters;
    std::vector<int> _vRobotActiveIndices;
    int _nRobotAffineDOF;
    Vector _vRobotRotationAxis;
    UserDataPtr _changehandler; ///< tracks changes for the robot and re-initializes parameters
};

typedef boost::shared_ptr<ActiveDOFTrajectoryRetimer> ActiveDOFTrajectoryRetimerPtr;

/** \brief Retime the trajectory points consisting of affine transformation values while avoiding collisions. <b>[multi-thread safe]</b>

    Collision is not checked. Every waypoint in the trajectory is guaranteed to be hit.
    The velocities for the current trajectory are overwritten.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param maxvelocities the max velocities of each dof
    \param maxaccelerations the max acceleration of each dof
    \param plannername the name of the planner to use to retime. If empty, will use the default trajectory re-timer.
    \param hastimestamps if true, use the already initialized timestamps of the trajectory
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the retiming planner
 */
OPENRAVE_API PlannerStatus RetimeAffineTrajectory(TrajectoryBasePtr traj, const std::vector<dReal>&maxvelocities, const std::vector<dReal>&maxaccelerations, bool hastimestamps=false, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Retimer planner the trajectory points consisting of affine transformation values while avoiding collisions. <b>[multi-thread safe]</b>

    Caches all the planners and parameters so PlanPath can be called multiple times without creating new objects.
    Collision is not checked. Every waypoint in the trajectory is guaranteed to be hit.
    The velocities for the current trajectory are overwritten.
 */
class OPENRAVE_API AffineTrajectoryRetimer
{
public:
    /**
       \param env the environment to create the planner in
       \param trajspec the ConfigurationSpecification to plan in, will extract that information from the trajectory
       \param maxvelocities the max velocities of each dof
       \param maxaccelerations the max acceleration of each dof
       \param plannername the name of the planner to use to retime. If empty, will use the default trajectory re-timer.
       \param hastimestamps if true, use the already initialized timestamps of the trajectory
     **/
    AffineTrajectoryRetimer(const std::string& plannername="", const std::string& plannerparameters="");
    virtual ~AffineTrajectoryRetimer() {
    }

    /// \breif reset the planner info with the following information
    virtual void SetPlanner(const std::string& plannername="", const std::string& plannerparameters="");

    /// \brief Executes smoothing. <b>[multi-thread safe]</b>
    ///
    /// \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    /// \return PlannerStatus of the status of the smoothing planner
    virtual PlannerStatus PlanPath(TrajectoryBasePtr traj, const std::vector<dReal>& maxvelocities, const std::vector<dReal>& maxaccelerations, bool hastimestamps=false, int planningoptions=0);

protected:
    std::string _plannername, _extraparameters;
    PlannerBase::PlannerParametersPtr _parameters;
    PlannerBasePtr _planner; ///< the planner is setup once in the constructor
};

typedef boost::shared_ptr<AffineTrajectoryRetimer> AffineTrajectoryRetimerPtr;

/** \brief Retime the trajectory points using all the positional data from the trajectory. <b>[multi-thread safe]</b>

    Collision is not checked. Every waypoint in the trajectory is guaranteed to be hit.
    The velocities for the current trajectory are overwritten.
    The returned trajectory will contain data only for the currenstly set active dofs of the robot.
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param plannername the name of the planner to use to retime. If empty, will use the default trajectory re-timer.
    \param hastimestamps if true, use the already initialized timestamps of the trajectory
    \param plannerparameters XML string to be appended to PlannerBase::PlannerParameters::_sExtraParameters passed in to the planner.
    \return PlannerStatus of the status of the retiming planner
 */
OPENRAVE_API PlannerStatus RetimeTrajectory(TrajectoryBasePtr traj, bool hastimestamps=false, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Inserts a waypoint into a trajectory at the index specified, and retimes the segment before and after the trajectory. This will \b not change the previous trajectory. <b>[multi-thread safe]</b>

    Collision is not checked on the modified segments of the trajectory.
    \param index The index where to start modifying the trajectory.
    \param dofvalues the configuration to insert into the trajectcory (active dof values of the robot)
    \param dofvelocities the velocities that the inserted point should start with
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param robot use the robot's active dofs to initialize the trajectory space
    \param plannername the name of the planner to use to retime. If empty, will use the default trajectory re-timer.
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t InsertActiveDOFWaypointWithRetiming(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, RobotBasePtr robot, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="", const std::string& plannerparameters="");

/** \brief Inserts a waypoint into a trajectory at the index specified, and retimes the segment before and after the trajectory using a planner. This will \b not change the previous trajectory. <b>[multi-thread safe]</b>

    Collision is not checked on the modified segments of the trajectory.
    \param index The index where to start modifying the trajectory.
    \param dofvalues the configuration to insert into the trajectcory (active dof values of the robot)
    \param dofvelocities the velocities that the inserted point should start with
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param planner initialized planner that will do the retiming. \ref PlannerBase::InitPlan should already be called.
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t InsertWaypointWithRetiming(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner);

/** \brief Insert a waypoint in a timed trajectory and smooth so that the trajectory always goes through the waypoint at the specified velocity. This might change the previous trajectory. <b>[multi-thread safe]</b>

    The PlannerParameters is automatically determined from the trajectory's configuration space
    \param[in] index The index where to insert the new waypoint. A negative value starts from the end.
    \param dofvalues the configuration to insert into the trajectcory (active dof values of the robot)
    \param dofvelocities the velocities that the inserted point should start with
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param plannername the name of the planner to use to smooth. If empty, will use the default trajectory smoother.
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t InsertWaypointWithSmoothing(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, dReal fmaxvelmult=1, dReal fmaxaccelmult=1, const std::string& plannername="");

/** \brief insert a waypoint in a timed trajectory and smooth so that the trajectory always goes through the waypoint at the specified velocity. This might change the previous trajectory. <b>[multi-thread safe]</b>

    The PlannerParameters is automatically determined from the trajectory's configuration space
    \param[in] index The index where to insert the new waypoint. A negative value starts from the end.
    \param dofvalues the configuration to insert into the trajectcory (active dof values of the robot)
    \param dofvelocities the velocities that the inserted point should start with
    \param traj the trajectory that initially contains the input points, it is modified to contain the new re-timed data.
    \param planner the initialized planner to use for smoothing. \ref PlannerBase::InitPlan should already be called on it. The planner parameters should be initialized to ignore timestamps. Optionally they could be initialized to accept velocities.
    \return the index of the first point in the original trajectory that comes after the modified trajectory.
 */
OPENRAVE_API size_t InsertWaypointWithSmoothing(int index, const std::vector<dReal>& dofvalues, const std::vector<dReal>& dofvelocities, TrajectoryBasePtr traj, PlannerBasePtr planner);

/// \brief convert the trajectory and all its points to a new specification
OPENRAVE_API void ConvertTrajectorySpecification(TrajectoryBasePtr traj, const ConfigurationSpecification &spec);

/** \brief computes the trajectory derivatives and modifies the trajetory configuration to store them.

    If necessary will change the configuration specification of the trajectory.
    If more derivatives are requested than the trajectory supports, will ignore them. For example, acceleration of a linear trajectory.
    \param traj the re-timed trajectory
    \param maxderiv the maximum derivative to assure. If 1, will assure velocities, if 2 will assure accelerations, etc.
 */
OPENRAVE_API void ComputeTrajectoryDerivatives(TrajectoryBasePtr traj, int maxderiv);

/// \brief returns a new trajectory with the order of the waypoints and times reversed.
///
/// Velocities are just negated and the new trajectory is not guaranteed to be executable or valid
OPENRAVE_API TrajectoryBasePtr ReverseTrajectory(TrajectoryBasePtr traj);

/// \brief returns a new trajectory with the order of the waypoints and times reversed.
///
/// Velocities are just negated and the new trajectory is not guaranteed to be executable or valid
OPENRAVE_API TrajectoryBasePtr GetReverseTrajectory(TrajectoryBaseConstPtr traj);

/// \brief segment the trajectory given the start and end points in-memory
///
/// this is an in-memory operation
/// \param traj the trajectory to segment
/// \param starttime the start time of the segment
/// \param endtime the end time of the segment
OPENRAVE_API void SegmentTrajectory(TrajectoryBasePtr traj, dReal starttime, dReal endtime);

/// \brief extract a segment of the trajectory given the start and end points and return a new trajectory
///
/// \param traj the trajectory to segment
/// \param starttime the start time of the segment. If < 0, returns the first waypoint of the trajectory
/// \param endtime the end time of the segment. If > duration, returns the last waypoint of the trajectory
OPENRAVE_API TrajectoryBasePtr GetTrajectorySegment(TrajectoryBaseConstPtr traj, dReal starttime, dReal endtime);

/// \brief merges the contents of multiple trajectories into one so that everything can be played simultaneously.
///
/// Each trajectory needs to have a 'deltatime' group for timestamps. The trajectories cannot share common configuration data because only one
/// trajectories's data can be set at a time.
/// \throw openrave_exception throws an exception if the trajectory data is incompatible and cannot be merged.
OPENRAVE_API TrajectoryBasePtr MergeTrajectories(const std::list<TrajectoryBaseConstPtr>&listtrajectories);

/** \brief represents the DH parameters for one joint

   T = Z_1 X_1 Z_2 X_2 ... X_n Z_n

   where
   Z_i = [cos(theta) -sin(theta) 0 0; sin(theta) cos(theta) 0 0; 0 0 1 d]
   X_i = [1 0 0 a; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0]

   http://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
 */
class DHParameter
{
public:
    KinBody::JointConstPtr joint; ///< pointer to joint
    int parentindex; ///< index into dh parameter array for getting cooreainte system of parent joint. If -1, no parent.
    Transform transform; ///< the computed coordinate system of this joint, this can be automatically computed from DH parameters
    dReal d; ///< distance along previous z
    dReal a; ///< orthogonal distance from previous z axis to current z
    dReal theta; ///< rotation of previous x around previous z to current x
    dReal alpha; ///< rotation of previous z to current z
};

/** \brief returns the Denavit-Hartenberg parameters of the kinematics structure of the body.

    If the robot has joints that cannot be represented by DH, will throw an exception.
    Passive joints are ignored. Joints are ordered by hierarchy dependency.
    By convention N joints give N-1 DH parameters, but GetDHParameters returns N parameters. The reason is because the first parameter is used to define the coordinate system of the first axis relative to the robot origin.
    \note The coordinate systems computed from the DH parameters do not match the OpenRAVE link coordinate systems.
    \param vparameters One set of parameters are returned for each joint. \see DHParameter.
    \param tstart the initial transform in the body coordinate system to the first joint
 */
OPENRAVE_API void GetDHParameters(std::vector<DHParameter>&vparameters, KinBodyConstPtr pbody);

/** \brief dynamics and collision checking with linear interpolation

    For any joints with maxtorque > 0, uses KinBody::ComputeInverseDynamics to check if the necessary torque exceeds the max torque. Max torque is always called via GetMaxTorque
 **/
class OPENRAVE_API DynamicsCollisionConstraint
{
public:
    /*
       \param listCheckBodies initialize with these bodies to check environment and self-collision with
       \param parameters stored inside the structure as a weak pointer
       \param filtermask A mask of \ref ConstraintFilterOptions specifying what checks the class should perform.
     */
    DynamicsCollisionConstraint(PlannerBase::PlannerParametersConstPtr parameters, const std::list<KinBodyPtr>& listCheckBodies, int filtermask=0xffffffff);
    virtual ~DynamicsCollisionConstraint() {
    }

    /// \brief sets a new planner parmaeters structure for checking
    virtual void SetPlannerParameters(PlannerBase::PlannerParametersConstPtr parameters);

    /// \brief Sets a new mask of \ref ConstraintFilterOptions specifying what checks the class should perform.
    virtual void SetFilterMask(int filtermask);

    /// \brief sets the perturbation to +- apply to all tested joint values.
    ///
    /// By default, perturbation is 0.1 and only applied if filtermask specifies it.
    /// \param perturbation It is multiplied by each DOF's resolution (_vConfigResolution) before added to the state.
    virtual void SetPerturbation(dReal perturbation);

    /// \brief if using dynamics limiting, choose whether to use the nominal torque or max instantaneous torque.
    ///
    /// \param torquelimitmode 1 if should use instantaneous max torque, 0 if should use nominal torque
    virtual void SetTorqueLimitMode(DynamicsConstraintsType torquelimitmode);

    /// \brief set user check fucntions
    ///
    /// Two functions can be set, one to be called before check collision and one after.
    /// \param bCallAfterCheckCollision if set, function will be called after check collision functions.
    virtual void SetUserCheckFunction(const boost::function<bool() >& usercheckfn, bool bCallAfterCheckCollision=false);

    /// \brief checks line collision. Uses the constructor's self-collisions
    virtual int Check(const std::vector<dReal>& q0, const std::vector<dReal>& q1, const std::vector<dReal>& dq0, const std::vector<dReal>& dq1, dReal timeelapsed, IntervalType interval, int options = 0xffff, ConstraintFilterReturnPtr filterreturn = ConstraintFilterReturnPtr());

    CollisionReportPtr GetReport() const {
        return _report;
    }

protected:
    /// \brief checks an already set state
    ///
    /// \param vdofvelocities the current velocities set on the robot
    /// \param vaccelconfig the current accelerations being applied on the robot
    /// \param options should already be masked with _filtermask
    virtual int _CheckState(const std::vector<dReal>& vdofvelocities, const std::vector<dReal>& vaccelconfig, int options, ConstraintFilterReturnPtr filterreturn);

    /// \brief sets and checks the state, also takes into account perturbations
    ///
    /// \param options should already be masked with _filtermask
    virtual int _SetAndCheckState(PlannerBase::PlannerParametersConstPtr params, const std::vector<dReal>& vdofvalues, const std::vector<dReal>& vdofvelocities, const std::vector<dReal>& vdofaccels, int options, ConstraintFilterReturnPtr filterreturn);
    virtual void _PrintOnFailure(const std::string& prefix);

    PlannerBase::PlannerParametersWeakConstPtr _parameters;
    std::vector<dReal> _vtempconfig, _vtempvelconfig, dQ, _vtempveldelta, _vtempaccelconfig, _vperturbedvalues, _vcoeff2, _vcoeff1, _vprevtempconfig, _vprevtempvelconfig, _vtempconfig2, _vdiffconfig, _vdiffvelconfig, _vstepconfig; ///< in configuration space
    CollisionReportPtr _report;
    std::list<KinBodyPtr> _listCheckBodies;
    int _filtermask;
    DynamicsConstraintsType _torquelimitmode; ///< 1 if should use instantaneous max torque, 0 if should use nominal torque
    dReal _perturbation;
    boost::array< boost::function<bool() >, 2> _usercheckfns;

    // for dynamics
    ConfigurationSpecification _specvel;
    std::vector< std::pair<int, std::pair<dReal, dReal> > > _vtorquevalues; ///< cache for dof indices and the torque limits that the current torque should be in
    std::vector< int > _vdofindices;
    std::vector<dReal> _doftorques, _dofaccelerations; ///< in body DOF space
    boost::shared_ptr<ConfigurationSpecification::SetConfigurationStateFn> _setvelstatefn;
};

typedef boost::shared_ptr<DynamicsCollisionConstraint> DynamicsCollisionConstraintPtr;

/// \deprecated (13/05/29)
class OPENRAVE_API LineCollisionConstraint
{
public:
    LineCollisionConstraint(const std::list<KinBodyPtr>& listCheckCollisions, bool bCheckEnv=true) : _constraint(PlannerBase::PlannerParametersPtr(), listCheckCollisions, bCheckEnv ? (CFO_CheckEnvCollisions|CFO_CheckSelfCollisions) : CFO_CheckSelfCollisions) {
    }
    virtual ~LineCollisionConstraint() {
    }

    inline void SetCheckEnvironmentCollision(bool check) RAVE_DEPRECATED
    {
        if( check ) {
            _constraint.SetFilterMask(CFO_CheckEnvCollisions|CFO_CheckSelfCollisions);
        }
        else {
            _constraint.SetFilterMask(CFO_CheckSelfCollisions);
        }
    }

    inline void SetUserCheckFunction(const boost::function<bool() >& usercheckfn, bool bCallAfterCheckCollision=false) RAVE_DEPRECATED
    {
        _constraint.SetUserCheckFunction(usercheckfn, bCallAfterCheckCollision);
    }

    inline bool Check(PlannerBase::PlannerParametersWeakPtr _params, const std::vector<dReal>& q0, const std::vector<dReal>& q1, IntervalType interval, PlannerBase::ConfigurationListPtr pvCheckedConfigurations) RAVE_DEPRECATED
    {
        PlannerBase::PlannerParametersPtr params = _params.lock();
        if( !!params ) {
            _constraint.SetPlannerParameters(params);
        }
        std::vector<dReal> vzero(q0.size());

        if( !!pvCheckedConfigurations ) {
            pvCheckedConfigurations->resize(0);
            if( !_filterreturn ) {
                _filterreturn.reset(new ConstraintFilterReturn());
            }
            else {
                _filterreturn->Clear();
            }
        }
        int ret = _constraint.Check(q0, q1, vzero, vzero, 0, interval, 0xffff, _filterreturn);
        if( !!_filterreturn && _filterreturn->_configurations.size() > 0 ) {
            BOOST_ASSERT(params->GetDOF() == (int)q0.size());
            pvCheckedConfigurations->resize(_filterreturn->_configurations.size()/params->GetDOF());
            for(std::vector<dReal>::iterator it = _filterreturn->_configurations.begin(); it != _filterreturn->_configurations.end(); it += params->GetDOF()) {
                pvCheckedConfigurations->push_back(std::vector<dReal>(it, it+params->GetDOF()));
            }
        }
        return ret;
    }

    CollisionReportPtr GetReport() const {
        return _constraint.GetReport();
    }

protected:
    DynamicsCollisionConstraint _constraint;
    ConstraintFilterReturnPtr _filterreturn;
} RAVE_DEPRECATED;

/// \brief simple distance metric based on joint weights
class OPENRAVE_API SimpleDistanceMetric
{
public:
    SimpleDistanceMetric(RobotBasePtr robot);
    dReal Eval(const std::vector<dReal>& c0, const std::vector<dReal>& c1);
protected:
    RobotBasePtr _robot;
//    int _activeaffine;
//    std::vector<int> _vdofindices;
    std::vector<dReal> weights2;
};

/// \brief samples the neighborhood of a configuration using the configuration space distance metric and sampler.
class OPENRAVE_API SimpleNeighborhoodSampler
{
public:
    SimpleNeighborhoodSampler(SpaceSamplerBasePtr psampler, const PlannerBase::PlannerParameters::DistMetricFn& distmetricfn, const PlannerBase::PlannerParameters::DiffStateFn& diffstatefn);

    bool Sample(std::vector<dReal>& vNewSample, const std::vector<dReal>& vCurSample, dReal fRadius);
    bool Sample(std::vector<dReal>& samples);
protected:
    SpaceSamplerBasePtr _psampler;
    PlannerBase::PlannerParameters::DistMetricFn _distmetricfn;
    PlannerBase::PlannerParameters::DiffStateFn _diffstatefn;
};

/// \brief Samples numsamples of solutions and each solution to vsolutions
///
/// \param nummaxsamples the max samples to query from a particular workspace goal. This does not necessarily mean every goal will have this many samples.
/// \param nummaxtries number of attemps to return a goal per Sample call.
/// \param fsampleprob The probability to attempt to sample a goal
/// \param searchfreeparameters if true, will search all the free parameters of the manipulator. Otherwise will use the current free parameters set on the robot
/// \param ikfilteroptions the filter options to pass to RobotBase::Manipulator::FindIKSolutions.
class OPENRAVE_API ManipulatorIKGoalSampler
{
public:
    ManipulatorIKGoalSampler(RobotBase::ManipulatorConstPtr pmanip, const std::list<IkParameterization>&listparameterizations, int nummaxsamples=20, int nummaxtries=10, dReal fsampleprob=1, bool searchfreeparameters=true, int ikfilteroptions=IKFO_CheckEnvCollisions, const std::vector<dReal>& freevalues = std::vector<dReal>());
    virtual ~ManipulatorIKGoalSampler() {
    }

    /// \brief if can sample, returns IkReturn pointer
    virtual IkReturnPtr Sample();
    virtual bool Sample(std::vector<dReal>& vgoal);

    /// \brief samples the rests of the samples until cannot be sampled anymore.
    ///
    /// \param vsamples vector is rest with samples
    /// \param maxsamples max successful samples to gather before returning. If 0, will gather all.
    /// \param maxchecksamples max samples to check before returning. If 0, will check all.
    /// \return true if a sample was inserted into vsamples
    bool SampleAll(std::list<IkReturnPtr>& samples, int maxsamples=0, int maxchecksamples=0);

    //void SetCheckPathConstraintsFn(const PlannerBase::PlannerParameters::CheckPathConstraintFn& checkfn)


    virtual int GetIkParameterizationIndex(int index);
    virtual void SetSamplingProb(dReal fsampleprob);

    /// \brief set a jitter distance for the goal
    ///
    /// \param maxdist If > 0, allows jittering of the goal IK if they cause the robot to be in collision and no IK solutions to be found
    virtual void SetJitter(dReal maxdist);

protected:
    struct SampleInfo
    {
        IkParameterization _ikparam;
        SpaceSamplerBasePtr _psampler; ///< sampler used to sample the free parameters of the iksolver
        std::vector<dReal> _vfreesamples; ///< optional samples N*vfree.size()
        std::vector< std::pair<int, dReal> > _vcachedists; ///< used for sorting freesamples. the sample closest to the midpoint is at the back of the vector (so it can be popped efficiently)
        int _numleft;
        int _orgindex;
    };
    RobotBasePtr _probot;
    RobotBase::ManipulatorConstPtr _pmanip;
    int _nummaxsamples, _nummaxtries;
    std::list<SampleInfo> _listsamples;
    SpaceSamplerBasePtr _pindexsampler;
    dReal _fsampleprob, _fjittermaxdist;
    CollisionReportPtr _report;
    std::vector< IkReturnPtr > _vikreturns;
    std::list<int> _listreturnedsamples;
    std::vector<dReal> _vfreestart;
    std::vector<dReal> _vfreeweights; ///< the joint weights of the free indices
    int _tempikindex; ///< if _vikreturns.size() > 0, points to the original ik index of those solutions
    int _ikfilteroptions;
    bool _searchfreeparameters;
    std::vector<dReal> _vfreegoalvalues;
};

typedef boost::shared_ptr<ManipulatorIKGoalSampler> ManipulatorIKGoalSamplerPtr;

} // planningutils
} // OpenRAVE

#endif
