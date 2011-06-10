// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
    IKFO_IgnoreCustomFilter=8, ///< will not use the custom filter, even if one is set
};

/** \brief <b>[interface]</b> Base class for all Inverse Kinematic solvers. See \ref arch_iksolver.   
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

    IkSolverBase(EnvironmentBasePtr penv) : InterfaceBase(PT_InverseKinematicsSolver, penv) {}
    virtual ~IkSolverBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_InverseKinematicsSolver; }
    
    /// sets the IkSolverBase attached to a specific robot and sets IkSolverBase specific options
    /// For example, some ik solvers might have different ways of computing optimal solutions.
    /// \param pmanip The manipulator the IK solver is attached to
    virtual bool Init(RobotBase::ManipulatorPtr pmanip) = 0;

    virtual RobotBase::ManipulatorPtr GetManipulator() const = 0;

    /// \brief Sets an ik solution filter that is called for every ik solution.
    ///
    /// \param filterfn - an optional filter function to be called, see \ref IkFilterCallbackFn.
    /// \exception openrave_exception Throw if filters are not supported.
    virtual void SetCustomFilter(const IkFilterCallbackFn& filterfn) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Number of free parameters defining the null solution space.
    ///
    /// Each parameter is always in the range of [0,1].
    virtual int GetNumFreeParameters() const = 0;

    /// \brief gets the free parameters from the current robot configuration
    ///
    /// \param[out] vFreeParameters is filled with GetNumFreeParameters() parameters in [0,1] range
    /// \return true if succeeded
    virtual bool GetFreeParameters(std::vector<dReal>& vFreeParameters) const = 0;
    
    /// Return a joint configuration for the given end effector transform. Robot is checked for self-collisions.
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance.
    ///           If q0 is NULL, returns the first solution found
    /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
    /// \param[out] solution [optional] Holds the IK solution
    /// \return true if solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /// Return all joint configurations for the given end effector transform. Robot is checked for self-collisions.
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
    /// \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
    /// \return true if at least one solution is found
    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /// Return a joint configuration for the given end effector transform. Robot is checked for self-collisions.
    /// Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance.
    ///           If q0 is empty, returns the first solution found
    /// \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
    /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
    /// \param[out] solution Holds the IK solution, must be of size RobotBase::Manipulator::_vecarmjoints
    /// \return true if solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /// Return all joint configurations for the given end effector transform. Robot is checked for self-collisions.
    /// Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
    /// \param[in] filteroptions A bitmask of \ref IkFilterOptions values controlling what is checked for each ik solution.
    /// \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
    /// \return true at least one solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& solutions) = 0;

    /// \brief returns true if the solver supports a particular ik parameterization as input.
    virtual bool Supports(IkParameterization::Type iktype) const OPENRAVE_DUMMY_IMPLEMENTATION;

private:
    virtual const char* GetHash() const { return OPENRAVE_IKSOLVER_HASH; }
};

} // end namespace OpenRAVE

#endif
