/*****************************************************************************
 *
 * Copyright (c) 2010-2011, the Trustees of Indiana University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Indiana University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE TRUSTEES OF INDIANA UNIVERSITY ''AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE TRUSTEES OF INDIANA UNIVERSITY BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************/

#ifndef DYNAMIC_PATH_H
#define DYNAMIC_PATH_H

#include "ParabolicRamp.h"

namespace ParabolicRampInternal {

/** @brief A base class for a feasibility checker.
 */
class FeasibilityCheckerBase
{
public:
    virtual ~FeasibilityCheckerBase() {
    }
    virtual bool ConfigFeasible(const Vector& q1, const Vector& dq1)=0;
    virtual bool SegmentFeasible(const Vector& q1, const Vector& q2, const Vector& dq1, const Vector& dq2)=0;
    virtual bool NeedDerivativeForFeasibility() {
        return false;
    }
};

/** @brief A base class for a distance checker.
 * ObstacleDistance returns the radius of a L-z norm guaranteed to
 * be collision-free.  ObstacleDistanceNorm returns the value of z.
 *
 * DynamicPath can currently only handle L-Inf norms.
 */
class DistanceCheckerBase
{
public:
    virtual ~DistanceCheckerBase() {
    }
    virtual Real ObstacleDistanceNorm() const {
        return Inf;
    }
    virtual Real ObstacleDistance(const Vector& x)=0;
};

/// Checks whether the ramp is feasible using exact checking
bool CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* feas,DistanceCheckerBase* distance,int maxiters);

/// Checks whether the ramp is feasible using a piecewise linear approximation
/// with tolerance tol
bool CheckRamp(const ParabolicRampND& ramp,FeasibilityCheckerBase* space,const Vector& tol);


class RampFeasibilityChecker
{
public:
    RampFeasibilityChecker(FeasibilityCheckerBase* feas,const Vector& tol);
    RampFeasibilityChecker(FeasibilityCheckerBase* feas,DistanceCheckerBase* distance,int maxiters);
    bool Check(const ParabolicRampND& x);

    FeasibilityCheckerBase* feas;
    Vector tol;
    DistanceCheckerBase* distance;
    int maxiters;
};


/** @brief A custom random number generator that can be provided to
 * DynamicPath::Shortcut()
 */
class RandomNumberGeneratorBase
{
public:
    virtual Real Rand() {
        return ::ParabolicRampInternal::Rand();
    }
};

/** @brief A bounded-velocity, bounded-acceleration trajectory consisting
 * of parabolic ramps.
 *
 * Optionally, joint limits xMin and xMax may be specified as well.  If so,
 * then the XXXBounded functions are used for smoothing.
 *
 * The Shortcut and OnlineShortcut methods can optionally take a
 * custom random number generator (may be useful for multithreading).
 */
class DynamicPath
{
public:
    DynamicPath();
    void Init(const Vector& velMax,const Vector& accMax);
    void SetJointLimits(const Vector& qMin,const Vector& qMax);
    inline void Clear() {
        ramps.clear();
    }
    inline bool Empty() const {
        return ramps.empty();
    }
    Real GetTotalTime() const;
    int GetSegment(Real t,Real& u) const;
    void Evaluate(Real t,Vector& x) const;
    void Derivative(Real t,Vector& dx) const;
    void SetMilestones(const std::vector<Vector>& x);
    void SetMilestones(const std::vector<Vector>& x,const std::vector<Vector>& dx);
    void GetMilestones(std::vector<Vector>& x,std::vector<Vector>& dx) const;
    void Append(const Vector& x);
    void Append(const Vector& x,const Vector& dx);
    void Concat(const DynamicPath& suffix);
    void Split(Real t,DynamicPath& before,DynamicPath& after) const;
    bool TryShortcut(Real t1,Real t2,RampFeasibilityChecker& check);
    /// \param mintimestep the minimum time step for a shortcut
    int Shortcut(int numIters,RampFeasibilityChecker& check, Real mintimestep=0);
    int Shortcut(int numIters,RampFeasibilityChecker& check,RandomNumberGeneratorBase* rng, Real mintimestep=0);
    int ShortCircuit(RampFeasibilityChecker& check);
    /// leadTime: the amount of time before this path should be executable
    /// padTime: an approximate bound on the time it takes to check a shortcut
    int OnlineShortcut(Real leadTime,Real padTime,RampFeasibilityChecker& check);
    int OnlineShortcut(Real leadTime,Real padTime,RampFeasibilityChecker& check,RandomNumberGeneratorBase* rng);

    bool IsValid() const;

    /// The joint limits (optional), velocity bounds, and acceleration bounds
    Vector xMin,xMax,velMax,accMax;
    /// The path is stored as a series of ramps
    std::vector<ParabolicRampND> ramps;
    int _multidofinterp; ///< if true, will always force the max acceleration of the robot when retiming rather than using lesser acceleration whenever possible
};

} //namespace ParabolicRamp

#endif
