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
#ifndef OPENRAVE_IKSOLVER_H
#define OPENRAVE_IKSOLVER_H

namespace OpenRAVE {

/* \brief Base class for all Inverse Kinematic solvers.
   
   \ingroup interfaces
   Each IK solver is defined on a subset of joints of a Robot specified by the robot's manipulator.
   Given the position in the 3D workspace that an end effector should go to, an IK solver will find
   the joint configuration to take that end-effector there.  Because it is common for an IK solution
   to have a null space, the IK solver give functionality to expose the free parameters to move the
   joints in null space.
*/
class RAVE_API IkSolverBase : public InterfaceBase
{
public:
    IkSolverBase(EnvironmentBasePtr penv) : InterfaceBase(PT_InverseKinematicsSolver, penv) {}
    virtual ~IkSolverBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_InverseKinematicsSolver; }
    
    /// sets the IkSolverBase attached to a specific robot and sets IkSolverBase specific options
    /// For example, some ik solvers might have different ways of computing optimal solutions.
    /// \param pmanip The manipulator the IK solver is attached to
    virtual bool Init(RobotBase::ManipulatorPtr pmanip) = 0;

    virtual RobotBase::ManipulatorPtr GetManipulator() const = 0;

    /// \return Number of free parameters defining the null solution space.
    ///         Each parameter is always in the range of [0,1].
    virtual int GetNumFreeParameters() const = 0;

    /// gets the free parameters from the current robot configuration
    /// \param[out] vFreeParameters is filled with GetNumFreeParameters() parameters in [0,1] range
    /// \return true if succeeded
    virtual bool GetFreeParameters(std::vector<dReal>& vFreeParameters) const = 0;
    
    /// Return a joint configuration for the given end effector transform. Robot is checked for self-collisions.
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance.
    ///           If q0 is NULL, returns the first solution found
    /// \param[in] bCheckEnvCollision If true, will only return solutions that are not colliding with the environment.
    /// \param[out] solution [optional] Holds the IK solution
    /// \return true if solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, bool bCheckEnvCollision, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /// Return all joint configurations for the given end effector transform. Robot is checked for self-collisions.
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] bCheckEnvCollision If true, will only return solutions that are not colliding with the environment.
    /// \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
    /// \return true if at least one solution is found
    virtual bool Solve(const IkParameterization& param, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& solutions) = 0;

    /// Return a joint configuration for the given end effector transform. Robot is checked for self-collisions.
    /// Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] q0 Return a solution nearest to the given configuration q0 in terms of the joint distance.
    ///           If q0 is empty, returns the first solution found
    /// \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
    /// \param[in] bCheckEnvCollision If true, will only return solutions that are not colliding with the environment.
    /// \param[out] solution Holds the IK solution, must be of size RobotBase::Manipulator::_vecarmjoints
    /// \return true if solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, bool bCheckEnvCollision, boost::shared_ptr< std::vector<dReal> > solution) = 0;

    /// Return all joint configurations for the given end effector transform. Robot is checked for self-collisions.
    /// Can specify the free parameters in [0,1] range. If NULL, the regular equivalent Solve is called
    /// \param[in] param the pose the end effector has to achieve. Note that the end effector pose 
    ///                        takes into account the grasp coordinate frame for the RobotBase::Manipulator
    /// \param[in] vFreeParameters The free parameters of the null space of the IK solutions. Always in range of [0,1]
    /// \param[in] bCheckEnvCollision If true, will only return solutions that are not colliding with the environment.
    /// \param[out] solutions All solutions within a reasonable discretization level of the free parameters.
    /// \return true at least one solution is found
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& solutions) = 0;

private:
    virtual const char* GetHash() const { return OPENRAVE_IKSOLVER_HASH; }
};

} // end namespace OpenRAVE

#endif
