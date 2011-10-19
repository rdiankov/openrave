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
/** \file iksolver.h
    \brief Inverse kinematics related definitions.
 */
#ifndef OPENRAVE_IKSOLVER_H
#define OPENRAVE_IKSOLVER_H

namespace OpenRAVE {

/// \brief Return value for the ik filter that can be optionally set on an ik solver.
enum IkFilterReturn
{
    IKFR_Success = 0, ///< the ik solution is good
    IKFR_Reject = 1, ///< reject the ik solution
    IKFR_Quit = 2, ///< the ik solution is rejected and the ik call itself should quit with failure
};

/// \brief Controls what information gets validated when searching for an inverse kinematics solution.
enum IkFilterOptions
{
    IKFO_CheckEnvCollisions=1, ///< will check environment collisions with the robot (not checked by default)
    IKFO_IgnoreSelfCollisions=2, ///< will not check the self-collision of the robot (checked by default)
    IKFO_IgnoreJointLimits=4, ///< will not check the joint limits of the robot (checked by default)
    IKFO_IgnoreCustomFilters=8, ///< will not use the custom filter, even if one is set
    IKFO_IgnoreEndEffectorCollisions=16, ///< will not check collision with the environment and the end effector links and bodies attached to the end effector links. The end effector links are defined by \ref RobotBase::Manipulator::GetChildLinks. Use this option when \ref RobotBase::Manipulator::CheckEndEffectorCollision has already been called, or it is ok for the end effector to collide given the IK constraints. Self-collisions between the moving links and end effector are still checked.
};

/** \brief <b>[interface]</b> Base class for all Inverse Kinematic solvers. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_iksolver.
   \ingroup interfaces
 */
class OPENRAVE_API IkSolverBase : public InterfaceBase
{
public:
    /** Inverse kinematics filter callback function.

        The filter is of the form <tt>return = filterfn(solution, manipulator, param)</tt>.
        The solution is guaranteed to be set on the robot's joint values before this function is called.
        If modifying the robot state, should restore it before this function returns.

        \param solution The current solution of the manipulator. Can be modified by this function, but note that it will not go through previous checks again.
        \param manipulator The current manipulator that the ik is being solved for.
        \param param The paramterization that IK was called with. This is in the manipulator base link's coordinate system (which is not necessarily the world coordinate system).
        \return \ref IkFilterReturn controlling the behavior of the ik search process.
     */
    typedef boost::function<IkFilterReturn(std::vector<dReal>&, RobotBase::ManipulatorPtr, const IkParameterization&)> IkFilterCallbackFn;

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
    virtual bool Init(RobotBase::ManipulatorPtr pmanip) = 0;

    virtual RobotBase::ManipulatorPtr GetManipulator() const = 0;

    /** \brief Sets an ik solution filter that is called for every ik solution.

        Multiple filters can be set at once, each filter will be called according to its priority; higher values get called first. The default implementation of IkSolverBase manages the filters internally. Users implementing their own IkSolverBase should call \ref _CallFilters to run the internally managed filters.
        \param filterfn - an optional filter function to be called, see \ref IkFilterCallbackFn.
        \param priority - The priority of the filter that controls the order in which filters get called. Higher priority filters get called first. If not certain what to set, use 0.
        \return a managed handle to the filter. If this handle is released, then the fitler will be removed. Release operation is <b>[multi-thread safe]</b>.
     */
    virtual UserDataPtr RegisterCustomFilter(int priority, const IkFilterCallbackFn& filterfn);

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

    /** \brief Return a joint configuration for the given end effector transform.

        \param[in] param the pose the end effector has to achieve. Note that the end effector pose
        takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is NULL, returns the first solution found
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solution [optional] Holds the IK solution
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /** \brief Return all joint configurations for the given end effector transform.

       \param[in] param the pose the end effector has to achieve. Note that the end effector pose
       takes into account the grasp coordinate frame for the RobotBase::Manipulator
       \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
       \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
       \return true if at least one solution is found
     */
    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /** Return a joint configuration for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve. Note that the end effector pose takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance. If q0 is empty, returns the first solution found
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solution Holds the IK solution, must be of size RobotBase::Manipulator::_vecarmjoints
        \return true if solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /** \brief Return all joint configurations for the given end effector transform.

        Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
        \param[in] param the pose the end effector has to achieve. Note that the end effector pose
        takes into account the grasp coordinate frame for the RobotBase::Manipulator
        \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
        \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
        \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
        \return true at least one solution is found
     */
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /// \brief returns true if the solver supports a particular ik parameterization as input.
    virtual bool Supports(IkParameterizationType iktype) const OPENRAVE_DUMMY_IMPLEMENTATION;

protected:
    inline IkSolverBasePtr shared_iksolver() {
        return boost::static_pointer_cast<IkSolverBase>(shared_from_this());
    }
    inline IkSolverBaseConstPtr shared_iksolver_const() const {
        return boost::static_pointer_cast<IkSolverBase const>(shared_from_this());
    }

    /// \brief calls the registered filters in their priority order and returns the value of the last called filter.
    virtual IkFilterReturn _CallFilters(std::vector<dReal>& solution, RobotBase::ManipulatorPtr manipulator, const IkParameterization& param);

private:
    virtual const char* GetHash() const {
        return OPENRAVE_IKSOLVER_HASH;
    }

    std::list<UserDataWeakPtr> __listRegisteredFilters; ///< internally managed filters

    friend class CustomIkSolverFilterData;
};

} // end namespace OpenRAVE

#endif
