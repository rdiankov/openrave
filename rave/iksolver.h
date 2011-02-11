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

/** \brief Parameterization of basic primitives for querying inverse-kinematics solutions.

    Holds the parameterization of a geometric primitive useful for autonomous manipulation scenarios like:
    6D pose, 3D translation, 3D rotation, 3D look at direction, and ray look at direction.
*/
class OPENRAVE_API IkParameterization
{
public:
    /// \brief The types of inverse kinematics parameterizations supported.
    ///
    /// The minimum degree of freedoms required is set in the upper 4 bits of each type.
    /// The lower bits contain a unique id of the type.
    enum Type {
        Type_None=0,
        Type_Transform6D=0x60000001, ///< end effector reaches desired 6D transformation
        Type_Rotation3D=0x30000002, ///< end effector reaches desired 3D rotation
        Type_Translation3D=0x30000003, ///< end effector origin reaches desired 3D translation
        Type_Direction3D=0x20000004, ///< direction on end effector coordinate system reaches desired direction
        Type_Ray4D=0x40000005, ///< ray on end effector coordinate system reaches desired global ray
        Type_Lookat3D=0x20000006, ///< direction on end effector coordinate system points to desired 3D position
        Type_TranslationDirection5D=0x50000007, ///< end effector origin and direction reaches desired 3D translation and direction. Can be thought of as Ray IK where the origin of the ray must coincide.
    };

    IkParameterization() : _type(Type_None) {}
    IkParameterization(const Transform& t) { SetTransform(t); }
    IkParameterization(const RAY& r) { SetRay(r); }

    inline void SetTransform(const Transform& t) { _type = Type_Transform6D; _transform = t; }
    inline void SetRotation(const Vector& quaternion) { _type = Type_Rotation3D; _transform.rot = quaternion; }
    inline void SetTranslation(const Vector& trans) { _type = Type_Translation3D; _transform.trans = trans; }
    inline void SetDirection(const Vector& dir) { _type = Type_Direction3D; _transform.rot = dir; }
    inline void SetRay(const RAY& ray) { _type = Type_Ray4D; _transform.trans = ray.pos; _transform.rot = ray.dir; }
    inline void SetLookat(const Vector& trans) { _type = Type_Lookat3D; _transform.trans = trans; }
    inline void SetTranslationDirection(const RAY& ray) { _type = Type_TranslationDirection5D; _transform.trans = ray.pos; _transform.rot = ray.dir; }

    inline Type GetType() const { return _type; }
    inline const Transform& GetTransform() const { return _transform; }
    inline const Vector& GetRotation() const { return _transform.rot; }
    inline const Vector& GetTranslation() const { return _transform.trans; }
    inline const Vector& GetDirection() const { return _transform.rot; }
    inline const Vector& GetLookat() const { return _transform.trans; }
    inline const RAY GetRay() const { return RAY(_transform.trans,_transform.rot); }
    inline const RAY GetTranslationDirection() const { return RAY(_transform.trans,_transform.rot); }

    /// \brief Returns the minimum degree of freedoms required for the IK type.
    static int GetDOF(Type type) { return (type>>28)&0xf; }

protected:
    Transform _transform;
    Type _type;

    friend IkParameterization operator* (const Transform& t, const IkParameterization& ikparam);
};

inline IkParameterization operator* (const Transform& t, const IkParameterization& ikparam)
{
    IkParameterization local;
    switch(ikparam.GetType()) {
    case IkParameterization::Type_Transform6D: local.SetTransform(t * ikparam.GetTransform()); break;
    case IkParameterization::Type_Rotation3D: local.SetRotation(quatMultiply(quatInverse(t.rot),ikparam.GetRotation())); break;
    case IkParameterization::Type_Translation3D: local.SetTranslation(t*ikparam.GetTranslation()); break;
    case IkParameterization::Type_Direction3D: local.SetDirection(t.rotate(ikparam.GetDirection())); break; 
    case IkParameterization::Type_Ray4D: {
            local.SetRay(RAY(t*ikparam.GetRay().pos,t.rotate(ikparam.GetRay().dir))); break;
    }
    case IkParameterization::Type_Lookat3D: local.SetLookat(t*ikparam.GetLookat()); break;
    case IkParameterization::Type_TranslationDirection5D: {
            local.SetRay(RAY(t*ikparam.GetTranslationDirection().pos,t.rotate(ikparam.GetTranslationDirection().dir))); break;
    }
    default: throw openrave_exception(str(boost::format("does not support parameterization %d")%ikparam.GetType()));
    }
    return local;
}

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
    virtual void SetCustomFilter(const IkFilterCallbackFn& filterfn) { throw openrave_exception("ik filters ignored",ORE_NotImplemented); }

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

private:
    virtual const char* GetHash() const { return OPENRAVE_IKSOLVER_HASH; }
};

} // end namespace OpenRAVE

#endif
