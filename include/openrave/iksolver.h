// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file iksolver.h
    \brief Inverse kinematics related definitions.

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_IKSOLVER_H
#define OPENRAVE_IKSOLVER_H

namespace OpenRAVE {

/// \brief Controls what information gets validated when searching for an inverse kinematics solution.
///
/// By default, inverse kinematics checks joint values, self-collisions, and custom filters. Unless specified with an option, it will not check environment collisions.
enum IkFilterOptions
{
    IKFO_CheckEnvCollisions=1, ///< will check environment collisions with the robot (not checked by default)
    IKFO_IgnoreSelfCollisions=2, ///< will not check the self-collision of the robot (checked by default). This also ignores the end effector collisions.
    IKFO_IgnoreJointLimits=4, ///< will not check the joint limits of the robot (checked by default). This has the side effect of only returning solutions within 360 degrees for revolute joints, even if they have a range > 360.
    IKFO_IgnoreCustomFilters=8, ///< will not use the registered custom filters, even if one is set. Custom filters are registered through \ref IkSolverBase::RegisterCustomFilter
    IKFO_IgnoreEndEffectorCollisions=0x10, ///< \see IKFO_IgnoreEndEffectorEnvCollisions
    IKFO_IgnoreEndEffectorEnvCollisions=0x10, ///< will not check collision with the environment and the end effector links and bodies attached to the end effector links. The end effector links are defined by \ref RobotBase::Manipulator::GetChildLinks. Use this option when \ref RobotBase::Manipulator::CheckEndEffectorCollision has already been called, or it is ok for the end effector to collide given the IK constraints. Self-collisions between the moving links and end effector are still checked.
    IKFO_IgnoreEndEffectorSelfCollisions=0x20, ///< will not check self-collisions with the end effector. The end effector links are defined by \ref RobotBase::Manipulator::GetChildLinks. Use this option if it is ok for the end effector to collide given the IK constraints. Collisions between the moving links and end effector are still checked.
    //IKFO_FillCollisionReports=0x1000, ///< if set, will fill the collision reports of the IkReturn structure (TODO)
};

/// \brief Return value for the ik filter that can be optionally set on an ik solver.
enum IkReturnAction : uint64_t
{
    IKRA_Success = 0, ///< the ik solution is good
    IKRA_Reject = 1, ///< reject the ik solution
    IKRA_Quit = 2, ///< the ik solution is rejected and the ik call itself should quit with failure

    // reasons why rejected
    IKRA_QuitEndEffectorCollision = (IKRA_Quit|0x00000080),
    IKRA_RejectKinematics = (IKRA_Reject|0x00000010),
    IKRA_RejectSelfCollision = (IKRA_Reject|0x00000020),
    IKRA_RejectEnvCollision = (IKRA_Reject|0x00000040),
    IKRA_RejectJointLimits = (IKRA_Reject|0x00000100),
    IKRA_RejectKinematicsPrecision = (IKRA_Reject|0x00000200),
    IKRA_RejectCustomFilter = (IKRA_Reject|0x00008000), // the reason should be set in the upper 16 bits
};

/// \brief priorities for what an ik solver checks. Should be a signed int
enum IkSolverPriority
{
    IKSP_Default = 0,
    IKSP_MinPriority = 0x80000000,
    IKSP_MaxPriority = 0x7fffffff,
};

/// \deprecated (12/04/29)
static const IkReturnAction IKFR_Success RAVE_DEPRECATED = IKRA_Success;
static const IkReturnAction IKFR_Reject RAVE_DEPRECATED = IKRA_Reject;
static const IkReturnAction IKFR_Quit RAVE_DEPRECATED = IKRA_Quit;
typedef IkReturnAction IkFilterReturn RAVE_DEPRECATED;

class OPENRAVE_API IkReturn
{
public:
    IkReturn(IkReturnAction action) : _action(action) {
    }

    inline bool operator != (IkReturnAction action) const {
        return _action != action;
    }
    inline bool operator == (IkReturnAction action) const {
        return _action == action;
    }

    /// \brief appends the data of one IkReturn to this structure
    ///
    /// _action is untouched, _vsolution is overridden if non-empty
    /// \return If data clashes, will output text and return false
    bool Append(const IkReturn& r);

    /// \brief clears the data, leaves the _action unchanged
    ///
    /// if _preport is set, will call Reset on it.
    void Clear();

    typedef std::map<std::string, std::vector<dReal> > CustomData;
    IkReturnAction _action;
    std::vector< dReal > _vsolution; ///< the solution
    CustomData _mapdata; ///< name/value pairs for custom data computed in the filters. Cascading filters using the same name will overwrite this until the last executed filter (with lowest priority).
    UserDataPtr _userdata; ///< if the name/value pairs are not enough, can further use a pointer to custom data. Cascading filters with valid _userdata pointers will overwrite this until the last executed filter (with lowest priority).
    //std::vector<CollisionReport> _reports; ///< all the reports that are written with the collision information if ik failed due to collisions. Only valid if _action has IKRA_RejectSelfCollision or IKRA_RejectEnvCollision set. (TODO)
};

/** \brief <b>[interface]</b> Base class for all Inverse Kinematic solvers. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_iksolver.
   \ingroup interfaces
 */
class OPENRAVE_API IkSolverBase : public InterfaceBase
{
public:
    /** \brief Inverse kinematics filter callback function that can decide to accept or reject the ik.

        The filter is of the form <tt>return = filterfn(solution, manipulator, param)</tt>.
        The solution is guaranteed to be set on the robot's joint values before this function is called. The active dof values of the robot will be set to the manipulator's arms.
        If the filter internally modifies the robot state and it returns IKRA_Success, the filter **has** to restore the original robot dof values and active dofs before it returns.
        If the filter happens to modify solution, the the robot state has to be set to the new solution.

        \param solution The current solution of the manipulator. Can be modified by this function, but note that it will not go through previous checks again.
        \param manipulator The current manipulator that the ik is being solved for.
        \param param The paramterization that IK was called with. This is in the manipulator base link's coordinate system (which is not necessarily the world coordinate system).
        \return \ref IkReturn outputs the action to take for the current ik solution and any custom parameters the filter should pass to the user.
     */
    typedef boost::function<IkReturn(std::vector<dReal>&, RobotBase::ManipulatorConstPtr, const IkParameterization&)> IkFilterCallbackFn;

    /** \brief gets called when an ik solution is accepted.
     */
    typedef boost::function<void (IkReturnPtr, RobotBase::ManipulatorConstPtr, const IkParameterization&)> IkFinishCallbackFn;

    IkSolverBase(EnvironmentBasePtr penv) : InterfaceBase(PT_InverseKinematicsSolver, penv) {
    }
    virtual ~IkSolverBase() {
    }

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_InverseKinematicsSolver;
    }

    /// brief Sets the IkSolverBase attached to a specific robot and sets IkSolverBase specific options.
    ///
    /// For example, some ik solvers might have different ways of computing optimal solutions.
    /// \param pmanip The manipulator the IK solver is attached to
    virtual bool Init(RobotBase::ManipulatorConstPtr pmanip) = 0;

    virtual RobotBase::ManipulatorPtr GetManipulator() const = 0;

    /** \brief Sets an ik solution filter that is called for every ik solution.

        Multiple filters can be set at once, each filter will be called according to its priority; higher values get called first. The default implementation of IkSolverBase manages the filters internally. Users implementing their own IkSolverBase should call \ref _CallFilters to run the internally managed filters.
        \param priority - The priority of the filter that controls the order in which filters get called. Higher priority filters get called first. If not certain what to set, use IKSP_Default. If set to IKSP_MinPriority, then the callback gets called at the very end when the solution is accepted.
        \param filterfn - an optional filter function to be called, see \ref IkFilterCallbackFn.
        \return a managed handle to the filter. If this handle is released, then the fitler will be removed. Release operation is <b>[multi-thread safe]</b>.
     */
    virtual UserDataPtr RegisterCustomFilter(int32_t priority, const IkFilterCallbackFn& filterfn);

    /** \brief sets a finish callback for every ik solution.

        Most useful when calling SolveAll in order to process notifications while the ik solver is still working.
     */
    virtual UserDataPtr RegisterFinishCallback(const IkFinishCallbackFn& finishfn);

    /// \deprecated (11/09/21)
    virtual void SetCustomFilter(const IkFilterCallbackFn& filterfn) RAVE_DEPRECATED
    {
        RAVELOG_WARN("IkSolverBase::SetCustomFilter is deprecated, have to use handle=AddCustomFilter. This call will will leak memory\n");
        if( __listRegisteredFilters.size() > 0 ) {
            RAVELOG_WARN("IkSolverBase::SetCustomFilter is deprecated, deleting all current filters!\n");
        }
        new UserDataPtr(RegisterCustomFilter(0,filterfn));
    }

    /// \brief Number of free parameters defining the null solution space.
    ///
    /// Each parameter is always in the range of [0,1].
    virtual int GetNumFreeParameters() const = 0;

    /** \brief gets the free parameters from the current robot configuration

        \param[out] vFreeParameters is filled with GetNumFreeParameters() parameters in [0,1] range
        \return true if succeeded
     */
    virtual bool GetFreeParameters(std::vector<dReal>& vFreeParameters) const = 0;

    /** \brief gets the indices of the free parameters from the current robot configuration

        \param[out] Indices of the free parameters
        \return true if succeeded
     */
    virtual bool GetFreeIndices(std::vector<int>& vFreeIndices) const = 0;

    /** \brief Return a joint configuration for the given end effector transform.

        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is NULL, returns the first solution found
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solution [optional] Holds the IK solution
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution = boost::shared_ptr< std::vector<dReal> >()) = 0;

    /** \brief Return a joint configuration for the given end effector transform.

        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is NULL, returns the first solution found
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] ikreturn Holds all the ik output data (including ik solutions) from the many processes involved in solving ik.
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, IkReturnPtr ikreturn);

    /** \brief Return all joint configurations for the given end effector transform.

        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
        \return true if at least one solution is found
     */
    virtual bool SolveAll(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /// \deprecated (12/05/01)
    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& solutions) RAVE_DEPRECATED {
        return SolveAll(param,filteroptions,solutions);
    }

    /** \brief Return all joint configurations for the given end effector transform.

        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] ikreturns Holds all the ik output data (including ik solutions) from the many processes involved in solving ik.
        \return true if at least one solution is found
     */
    virtual bool SolveAll(const IkParameterization& param, int filteroptions, std::vector<IkReturnPtr>& ikreturns);

    /** Return a joint configuration for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is empty, returns the first solution found
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solution [optional] Holds the IK solution, must be of size RobotBase::Manipulator::_vecarmjoints
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution=boost::shared_ptr< std::vector<dReal> >()) = 0;

    /** Return a joint configuration for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is empty, returns the first solution found
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] ikreturn Holds all the ik output data (including ik solutions) from the many processes involved in solving ik.
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn);

    /** \brief Return all joint configurations for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
        \return true at least one solution is found
     */
    virtual bool SolveAll(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /// \deprecated (12/05/01)
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& solutions) RAVE_DEPRECATED {
        return SolveAll(param,vFreeParameters,filteroptions,solutions);
    }

    /** \brief Return all joint configurations for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve in the manipulator base's coordinate system. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] ikreturns Holds all the ik output data (including ik solutions) from the many processes involved in solving ik.
        \return true at least one solution is found
     */
    virtual bool SolveAll(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& ikreturns);

    /// \brief returns true if the solver supports a particular ik parameterization as input.
    virtual bool Supports(IkParameterizationType iktype) const OPENRAVE_DUMMY_IMPLEMENTATION;

    /** \brief Calls the registered filters in their priority order and returns the value of the last called filter.

        The current robot manipulator DOF values are used.
        The parameters are the same as \ref IkFilterCallbackFn, except the IkReturn is an optional input parameter and the return
        value is just the \ref IkReturnAction. For users that do not request the filter output, this allows the computation
        to be optimized away.

        \param param The paramterization that currently reflects the robot's configuration state, can also contain custom parameters. This is in the manipulator base link's coordinate system (which is not necessarily the world coordinate system).
        \param minpriority the minimum inclusive priority to consider
        \param maxpriority the maximum inclusive priority to consider
        \return \ref IkReturn outputs the action to take for the current ik solution and any custom parameters the filter should pass to the user.
     */
    virtual IkReturnAction CallFilters(const IkParameterization& param, IkReturnPtr ikreturn=IkReturnPtr(), int32_t minpriority=IKSP_MinPriority, int32_t maxpriority=IKSP_MaxPriority) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns the kinematics structure hash this ik solver is encoded to. Checked with \ref RobotBase::Manipulator::GetKinematicsStructureHash()
    virtual const std::string& GetKinematicsStructureHash() const OPENRAVE_DUMMY_IMPLEMENTATION;
    
protected:
    inline IkSolverBasePtr shared_iksolver() {
        return boost::static_pointer_cast<IkSolverBase>(shared_from_this());
    }
    inline IkSolverBaseConstPtr shared_iksolver_const() const {
        return boost::static_pointer_cast<IkSolverBase const>(shared_from_this());
    }

    virtual IkReturnAction _CallFilters(std::vector<dReal>& solution, RobotBase::ManipulatorPtr manipulator, const IkParameterization& param, IkReturnPtr ikreturn=IkReturnPtr(), int32_t minpriority=IKSP_MinPriority, int32_t maxpriority=IKSP_MaxPriority);

    /// \brief returns true if there's registered filters within the priority range (inclusive)
    virtual bool _HasFilterInRange(int32_t minpriority, int32_t maxpriority) const;

    virtual void _CallFinishCallbacks(IkReturnPtr, RobotBase::ManipulatorConstPtr, const IkParameterization &);

private:
    std::list<UserDataWeakPtr> __listRegisteredFilters; ///< internally managed filters
    std::list<UserDataWeakPtr> __listRegisteredFinishCallbacks; ///< internally managed callbacks

    friend class CustomIkSolverFilterData;
    friend class IkSolverFinishCallbackData;
};

} // end namespace OpenRAVE

#endif
