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
/** \file trajectory.h
    \brief  Define a time-parameterized trajectory of robot configurations.
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

namespace OpenRAVE {

/** \brief <b>[interface]</b> Encapsulate a time-parameterized trajectories of robot configurations. See \ref arch_trajectory.
    \ingroup interfaces
*/
class RAVE_API TrajectoryBase : public InterfaceBase
{
public:
    /// \brief trajectory interpolation and sampling methods 
    enum InterpEnum {
        NONE=0, ///< unspecified timing info
        LINEAR=1, ///< linear interpolation 
        LINEAR_BLEND=2, ///< linear with quadratic blends
        CUBIC=3, ///< cubic spline interpolation
        QUINTIC=4, ///< quintic min-jerk interpolation 
        NUM_METHODS=5, ///< number of interpolation methods
    };     

    /// \brief options for serializing trajectories
    enum TrajectoryOptions {
        TO_OneLine = 1, ///< if set, will write everything without newlines, otherwise
                        ///< will start a newline for the header and every trajectory point
        TO_NoHeader = 2, ///< do not write the header that specifies number of points, degrees of freedom, and other options
        TO_IncludeTimestamps = 4,
        TO_IncludeBaseTransformation = 8,
        TO_IncludeVelocities = 0x10, ///< include velocities. If TO_IncludeBaseTransformation is also set, include the base
                                   ///< base link velocity in terms of linear and angular velocity
        TO_IncludeTorques = 0x20, ///< include torques
        TO_InterpolationMask = 0x1c0, ///< bits to store the interpolation information
    };
    
    /// Via point along the trajectory (joint configuration with a timestamp)
    class TPOINT
    {
    public:
        TPOINT() : time(0), blend_radius(0) {}
        TPOINT(const std::vector<dReal>& newq, dReal newtime) : time(newtime), blend_radius(0) { q = newq; }
        TPOINT(const std::vector<dReal>& newq, const Transform& newtrans, dReal newtime) : time(newtime), blend_radius(0) { q = newq; trans = newtrans; }

        enum TPcomponent {  POS=0,   //!< joint angle position
                          VEL,     //!< joint angle velocity
                          ACC,     //!< joint angle acceleration
                          NUM_COMPONENTS };

        friend std::ostream& operator<<(std::ostream& O, const TPOINT& tp);
        void Setq(std::vector<dReal>* values)
        {
            assert(values->size() == q.size()); 
            for(int i = 0; i < (int)values->size(); i++)
                q[i] = values->at(i);
	    // reset the blend_radius
	    blend_radius=0;

        }

        Transform trans;            ///< transform of the first link
        Vector linearvel;           ///< instanteneous linear velocity
        Vector angularvel;          ///< instanteneous angular velocity
        std::vector<dReal> q;       ///< joint configuration
        std::vector<dReal> qdot;    ///< instantaneous joint velocities
        std::vector<dReal> qtorque; ///< feedforward torque [optional]
        dReal  time;                ///< time stamp of trajectory point   
        dReal  blend_radius;
    };

    class TSEGMENT
    {
    public:
        //! the different segment types
        enum Type {  START=0,     //!< starting trajectory segment
               MIDDLE,      //!< middle trajectory segment
               END,         //!< ending trajectory segment
               NUM_TYPES };

        void SetDimensions(int curve_degree, int num_dof) { coeff.resize((curve_degree+1)*num_dof); _curvedegrees = curve_degree; }

        inline dReal Get(int deg, int dof) const { return coeff[dof*(_curvedegrees+1)+deg]; }
        dReal& Get(int deg, int dof) { return coeff[dof*(_curvedegrees+1)+deg]; }

        friend std::ostream& operator<<(std::ostream& O, const TSEGMENT& tp);

        Vector linearvel;           ///< instanteneous linear velocity
        Vector angularvel;          ///< instanteneous angular velocity

    private:
        dReal _fduration;
        int _curvedegrees;
        std::vector<dReal> coeff;       ///< num_degrees x num_dof coefficients of the segment

        friend class TrajectoryBase;
    };

    TrajectoryBase(EnvironmentBasePtr penv, int nDOF);
    virtual ~TrajectoryBase() {}

    /// return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() { return PT_Trajectory; }
    
    /// clears all points and resets the dof of the trajectory
    virtual void Reset(int nDOF);

    /// getting information about the trajectory
    inline dReal GetTotalDuration() const      { return _vecpoints.size() > 0 ? _vecpoints.back().time : 0; }
    InterpEnum GetInterpMethod() const         { return _interpMethod; }
    const std::vector<TPOINT>& GetPoints() const { return _vecpoints; }
    std::vector<TPOINT>& GetPoints() { return _vecpoints; }
    const std::vector<TSEGMENT>& GetSegments() const { return _vecsegments; }

    virtual void Clear();

    /// add a point to the trajectory
    virtual void AddPoint(const TPOINT& p) { assert( _nDOF == (int)p.q.size()); _vecpoints.push_back(p); }

    /** \brief Preprocesses the trajectory for later sampling and set its interpolation method.
        
        \param[in] robot [optional] robot to do the timing for
        \param[in] interpolationMethod method to use for interpolation
        \param bAutoCalcTiming If true, will retime the trajectory using maximum joint velocities and accelerations, otherwise it expects the time stamps of each point to be set.
        \param[in] bActiveDOFs If true, then the trajectory is specified in the robot's active degrees of freedom
        and the maximum velocities and accelerations will be extracted appropriately. Affine transformations
        are ignored in the retiming if true. If false, then use the
        robot's full joint configuration and affine transformation for max velocities.
        \param[in] fMaxVelMult The percentage of the max velocity of each joint to use when retiming.
    */
    virtual bool CalcTrajTiming(RobotBaseConstPtr robot, InterpEnum interpolationMethod, bool bAutoCalcTiming, bool bActiveDOFs, dReal fMaxVelMult=1);

    /// perform basic error checking on the trajectory internal data.
    ///
    /// checks internal data structures and verifies that all trajectory
    /// via points do not violate joint position, velocity, and
    /// acceleration limits.
    virtual bool IsValid() const;
    /// tests if a point violates any position, velocity or accel constraints
    //virtual bool  IsValidPoint(const TPOINT& tp) const;

    /// \brief Sample the trajectory at the given time using the current interpolation method.
    virtual bool SampleTrajectory(dReal  time, TPOINT &sample) const;

    /// Write to a stream, see TrajectoryOptions for file format
    /// \param sinput stream to read the data from
    /// \param options a combination of enums in TrajectoryOptions
    virtual bool Write(std::ostream& sinput, int options) const;
        
    /// Reads the trajectory, expects the filename to have a header.
    /// \param sout stream to output the trajectory data
    /// \param robot The robot to attach the trajrectory to, if specified, will
    ///              call CalcTrajTiming to get the correct trajectory velocities.
    virtual bool Read(std::istream& sout, RobotBasePtr robot);

    virtual bool Read(const std::string& filename, RobotBasePtr robot) RAVE_DEPRECATED;
    virtual bool Write(const std::string& filename, int options) const RAVE_DEPRECATED;

    virtual int GetDOF() const { return _nDOF; }
private:

    /// \brief Linear interpolation using the maximum joint velocities for timing.
    virtual bool _SetLinear(bool bAutoCalcTiming, bool bActiveDOFs);

    //// linear interpolation with parabolic blends
    //virtual bool _SetLinearBlend(bool bAutoCalcTiming);

    /// calculate the coefficients of a the parabolic and linear blends
    ///  with continuous endpoint positions and velocities for via points.
    //virtual void _CalculateLinearBlendCoefficients(TSEGMENT::Type segType,TSEGMENT& seg, TSEGMENT& prev,TPOINT& p0, TPOINT& p1,const dReal blendAccel[]);

    /// cubic spline interpolation
    virtual bool _SetCubic(bool bAutoCalcTiming, bool bActiveDOFs);

    /// calculate the coefficients of a smooth cubic spline with
    ///  continuous endpoint positions and velocities for via points.
    virtual void _CalculateCubicCoefficients(TSEGMENT& , const TPOINT& tp0, const TPOINT& tp1);

    //bool _SetQuintic(bool bAutoCalcTiming, bool bActiveDOFs);

    /// calculate the coefficients of a smooth quintic spline with
    ///  continuous endpoint positions and velocities for via points
    ///  using minimum jerk heuristics
    //void _CalculateQuinticCoefficients(TSEGMENT&, TPOINT& tp0, TPOINT& tp1);

    /// recalculate all via point velocities and accelerations
    virtual void _RecalculateViaPointDerivatives();

    /// computes minimum time interval for linear interpolation between
    ///  path points that does not exceed the maximum joint velocities 
    virtual dReal _MinimumTimeLinear(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);

    /// computes minimum time interval for cubic interpolation between
    ///  path points that does not exceed the maximum joint velocities 
    ///  or accelerations
    virtual dReal _MinimumTimeCubic(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);

    /// computes minimum time interval for cubic interpolation between
    ///  path points that does not exceed the maximum joint velocities 
    ///  or accelerations assuming zero velocities at endpoints
    virtual dReal _MinimumTimeCubicZero(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);

    /// computes minimum time interval for quintic interpolation between
    ///  path points that does not exceed the maximum joint velocities 
    ///  or accelerations
    virtual dReal _MinimumTimeQuintic(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);
    virtual dReal _MinimumTimeTransform(const Transform& t0, const Transform& t1);

    /// find the active trajectory interval covering the given time
    ///  (returns the index of the start point of the interval)
    virtual int _FindActiveInterval(dReal time) const;

    /// \brief Sample the trajectory using linear interpolation.
    virtual bool _SampleLinear(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;

    /// \brief Sample using linear interpolation with parabolic blends.
    virtual bool _SampleLinearBlend(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;

    /// \brief Sample the trajectory using cubic interpolation.
    virtual bool _SampleCubic(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;

    /// \brief Sample the trajectory using quintic interpolation with minimum jerk.
    virtual bool _SampleQuintic(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;

    std::vector<TPOINT> _vecpoints;
    std::vector<TSEGMENT> _vecsegments;
    std::vector<dReal> _lowerJointLimit, _upperJointLimit, _maxJointVel, _maxJointAccel;
    Vector _maxAffineTranslationVel;
    dReal _maxAffineRotationQuatVel;
    int _nQuaternionIndex; ///< the index of a quaternion rotation, if one exists (interpolation is different for quaternions)

    /// computes the difference of two states necessary for correct interpolation when there are circular joints. Default is regular subtraction.
    /// _diffstatefn(q1,q2) -> q1 -= q2
    boost::function<void(std::vector<dReal>&,const std::vector<dReal>&)> _diffstatefn;

    InterpEnum   _interpMethod;
    int _nDOF;

    virtual const char* GetHash() const { return OPENRAVE_TRAJECTORY_HASH; }
};

typedef TrajectoryBase Trajectory;

} // end namespace OpenRAVE

#endif // TRAJECTORY_H
