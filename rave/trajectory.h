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
  \file   Trajectory.h
  \brief  Define a time-parameterized trajectory of robot configurations
 -------------------------------------------------------------------- */
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

namespace OpenRAVE {

/// Encapsulate a time-parameterized trajectories of robot configurations
class Trajectory : public InterfaceBase
{
public:
    /// exporting options
    /// The file format is:
    /// #points #dof #options
    /// [timestamp 1] [dof joint values 1] [transform 1] [dof joint velocities 1] [linear and angular velocity 1]
    /// [timestamp 2] [dof joint values 2] [transform 2] [dof joint velocities 2] [linear and angular velocity 2]
    /// ...
    /// linear velocity - xyz velocity
    /// angular velocity - axis * angular_speed
    /// transform - look at Transform::operator <<, outputs quaternion first then translation
    /// an example file that contains 2 points with timestamps for a 3 dof robot is
    /// 2 3 4
    /// 0.5 0 0   0
    /// 1.5 1 2.2 3.3
    enum TrajectoryOptions {
        TO_OneLine = 1, ///< if set, will write everything without newlines, otherwise
                        ///< will start a newline for the header and every trajectory point
        TO_NoHeader = 2, ///< do not write the header that specifies number of points, degrees of freedom, and other options
        TO_IncludeTimestamps = 4,
        TO_IncludeBaseTransformation = 8,
        TO_IncludeVelocities = 16, ///< include velocities. If TO_IncludeBaseTransformation is also set, include the base
                                   ///< base link velocity in terms of linear and angular velocity
    };
    
    /// trajectory interpolation and sampling methods 
    enum InterpEnum {  NONE=0,            //!< unspecified timing info
                     LINEAR=1,            //!< linear interpolation 
	         LINEAR_BLEND=2,      //!< linear with quadratic blends
	         CUBIC=3,             //!< cubic spline interpolation
	         QUINTIC=4,           //!< quintic min-jerk interpolation 
	         NUM_METHODS=5 };     //!< number of interpolation methods

    /// Via point along the trajectory (joint configuration with a timestamp)
    struct TPOINT
    {
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
        dReal  time;                ///< time stamp of trajectory point   
        dReal  blend_radius;
    };

    struct TSEGMENT
    {
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

        friend class Trajectory;
    };

    Trajectory(EnvironmentBase* penv, int nDOF);
    virtual ~Trajectory() {}

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

    /// sets the trajectory from a raw array of points
    /// \param pPathData every 
    virtual bool SetFromRawPathData(const dReal* pPathData, int numFrames);

    /// Specify the trajectory timing and interpolation method for a given robot.
    /// The trajectory will use the robot's active degrees of freedom, so
    /// make sure that the trajectory's _nDOF == pRobot->GetActiveDOF().

    /// Preprocesses the trajectory for later sampling
    /// if the 'bAutoCalTiming' flag is set, then the timing of the
    /// trajectory is automatically calculated based on the maximum
    /// joint velocities and accelerations.
    /// \param bActiveDOFs If true, then the trajectory is specified in the robot's active degrees of freedom
    /// and the maximum velocities and accelerations will be extracted appropriately. Affine transformations
    /// are ignored in the retiming if true. If false, then use the
    /// robot's full joint configuration and affine transformation for max velocities.
    virtual bool CalcTrajTiming(const RobotBase* pRobot, InterpEnum interpolationMethod, bool bAutoCalcTiming, bool bActiveDOFs, dReal fMaxVelMult=1);

    /// perform basic error checking on the trajectory internal data
    virtual bool IsValid() const;
    /// tests if a point violates any position, velocity or accel constraints
    //virtual bool  IsValidPoint(const TPOINT& tp) const;

    /// sample the trajectory at the given time 
    virtual bool SampleTrajectory(dReal  time, TPOINT &sample) const;

    /// Read/Write the data to and from streams/files
    //@{

    /// Write to a filename, see TrajectoryOptions for file format.
    /// \param options a combination of enums in TrajectoryOptions
    virtual bool Write(const char* filename, int options) const;

    /// Write to a FILE, see TrajectoryOptions for file format.
    /// \param options a combination of enums in TrajectoryOptions
    virtual bool Write(FILE* f, int options) const;

    /// Write to a stream, see TrajectoryOptions for file format
    /// \param options a combination of enums in TrajectoryOptions
    virtual bool Write(std::ostream& f, int options) const;
        
    /// Reads the trajectory, expects the filename to have a header.
    /// \param robot The robot to attach the trajrectory to, if specified, will
    ///              call CalcTrajTiming to get the correct trajectory velocities.
    virtual bool Read(const char* filename, RobotBase* robot);
    virtual bool Read(std::istream& f, RobotBase* robot);
    //@}

    virtual int GetDOF() const { return _nDOF; }
private:

    // timing methods
    bool _SetLinear(bool bAutoCalcTiming, bool bActiveDOFs);

    bool _SetLinearBlend(bool bAutoCalcTiming);
    void _CalculateLinearBlendCoefficients(TSEGMENT::Type segType,
                                                  TSEGMENT& seg, TSEGMENT& prev,
                                                  TPOINT& p0, TPOINT& p1,
                                                  const dReal blendAccel[]);

    bool _SetCubic(bool bAutoCalcTiming, bool bActiveDOFs);
    void _CalculateCubicCoefficients(TSEGMENT& , const TPOINT& tp0, const TPOINT& tp1);

    //bool _SetQuintic(bool bAutoCalcTiming, bool bActiveDOFs);
    //void _CalculateQuinticCoefficients(TSEGMENT&, TPOINT& tp0, TPOINT& tp1);

    void _RecalculateViaPointDerivatives();

    dReal _MinimumTimeLinear(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);
    dReal _MinimumTimeCubic(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);
    dReal _MinimumTimeCubicZero(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);
    dReal _MinimumTimeQuintic(const TPOINT& p0, const TPOINT& p1, bool bActiveDOFs);
    dReal _MinimumTimeTransform(const Transform& t0, const Transform& t1);

    // sampling methods
    inline int  _FindActiveInterval(dReal time) const;

    bool _SampleLinear(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;
    bool _SampleLinearBlend(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;
    bool _SampleCubic(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;
    bool _SampleQuintic(const TPOINT& p0, const TPOINT& p1, const TSEGMENT& seg, dReal time, TPOINT& sample) const;

    std::vector<TPOINT> _vecpoints;
    std::vector<TSEGMENT> _vecsegments;
    
    // cache
    std::vector<dReal> _lowerJointLimit, _upperJointLimit, _maxJointVel, _maxJointAccel;
    Vector _maxAffineTranslationVel, _maxAffineRotationQuatVel;

    InterpEnum   _interpMethod;
    int _nDOF;

    virtual const char* GetHash() const { return OPENRAVE_TRAJECTORY_HASH; }
};

} // end namespace OpenRAVE

#endif // TRAJECTORY_H
